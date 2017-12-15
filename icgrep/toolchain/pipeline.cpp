/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "pipeline.h"
#include <toolchain/toolchain.h>
#include <kernels/kernel.h>
#include <kernels/streamset.h>
#include <llvm/IR/Module.h>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <kernels/kernel_builder.h>

using namespace kernel;
using namespace parabix;
using namespace llvm;

using Port = Kernel::Port;

template <typename Value>
using StreamSetBufferMap = boost::container::flat_map<const StreamSetBuffer *, Value>;

template <typename Value>
using FlatSet = boost::container::flat_set<Value>;

Function * makeThreadFunction(const std::unique_ptr<kernel::KernelBuilder> & b, const std::string & name) {
    Function * const f = Function::Create(FunctionType::get(b->getVoidTy(), {b->getVoidPtrTy()}, false), Function::InternalLinkage, name, b->getModule());
    f->setCallingConv(CallingConv::C);
    f->arg_begin()->setName("state");
    return f;
}

void applyOutputBufferExpansions(const std::unique_ptr<KernelBuilder> & b, const Kernel * kernel);

void handleInsufficientData(const std::unique_ptr<KernelBuilder> & b, Value * const produced, Value * const final, BasicBlock * const entry, const Kernel * const consumer,  const Binding & input, const StreamSetBuffer * const buffer);

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateSegmentParallelPipeline
 *
 * Given a computation expressed as a logical pipeline of K kernels k0, k_1, ...k_(K-1)
 * operating over an input stream set S, a segment-parallel implementation divides the input
 * into segments and coordinates a set of T <= K threads to each process one segment at a time.
 * Let S_0, S_1, ... S_N be the segments of S.   Segments are assigned to threads in a round-robin
 * fashion such that processing of segment S_i by the full pipeline is carried out by thread i mod T.
 ** ------------------------------------------------------------------------------------------------------------- */
void generateSegmentParallelPipeline(const std::unique_ptr<KernelBuilder> & b, const std::vector<Kernel *> & kernels) {

    const unsigned n = kernels.size();
    Module * const m = b->getModule();
    IntegerType * const sizeTy = b->getSizeTy();
    PointerType * const voidPtrTy = b->getVoidPtrTy();
    Constant * nullVoidPtrVal = ConstantPointerNull::getNullValue(voidPtrTy);
    std::vector<Type *> structTypes;
    codegen::BufferSegments = std::max(codegen::BufferSegments, codegen::ThreadNum);

    Value * instance[n];
    for (unsigned i = 0; i < n; ++i) {
        instance[i] = kernels[i]->getInstance();
        structTypes.push_back(instance[i]->getType());
    }
    StructType * const sharedStructType = StructType::get(m->getContext(), structTypes);
    StructType * const threadStructType = StructType::get(m->getContext(), {sharedStructType->getPointerTo(), sizeTy});

    const auto ip = b->saveIP();

    Function * const threadFunc = makeThreadFunction(b, "segment");
    auto args = threadFunc->arg_begin();

    // -------------------------------------------------------------------------------------------------------------------------
    // MAKE SEGMENT PARALLEL PIPELINE THREAD
    // -------------------------------------------------------------------------------------------------------------------------

     // Create the basic blocks for the thread function.
    BasicBlock * entryBlock = BasicBlock::Create(b->getContext(), "entry", threadFunc);
    b->SetInsertPoint(entryBlock);

    Value * const threadStruct = b->CreateBitCast(&*(args), threadStructType->getPointerTo());

    Value * const sharedStatePtr = b->CreateLoad(b->CreateGEP(threadStruct, {b->getInt32(0), b->getInt32(0)}));
    for (unsigned k = 0; k < n; ++k) {
        Value * ptr = b->CreateLoad(b->CreateGEP(sharedStatePtr, {b->getInt32(0), b->getInt32(k)}));
        kernels[k]->setInstance(ptr);
    }
    Value * const segOffset = b->CreateLoad(b->CreateGEP(threadStruct, {b->getInt32(0), b->getInt32(1)}));

    BasicBlock * segmentLoop = BasicBlock::Create(b->getContext(), "segmentLoop", threadFunc);
    b->CreateBr(segmentLoop);

    b->SetInsertPoint(segmentLoop);
    PHINode * const segNo = b->CreatePHI(b->getSizeTy(), 2, "segNo");
    segNo->addIncoming(segOffset, entryBlock);

    Value * terminated = b->getFalse();
    Value * const nextSegNo = b->CreateAdd(segNo, b->getSize(1));

    BasicBlock * const exitThreadBlock = BasicBlock::Create(b->getContext(), "exitThread", threadFunc);

    StreamSetBufferMap<Value *> producedItemCount;
    StreamSetBufferMap<Value *> consumedItemCount;

    Value * cycleCountStart = nullptr;
    Value * cycleCountEnd = nullptr;
    if (DebugOptionIsSet(codegen::EnableCycleCounter)) {
        cycleCountStart = b->CreateReadCycleCounter();
    }

    for (unsigned k = 0; k < n; ++k) {

        const auto & kernel = kernels[k];

        BasicBlock * const kernelWait = BasicBlock::Create(b->getContext(), kernel->getName() + "Wait", threadFunc);

        b->CreateBr(kernelWait);

        BasicBlock * const kernelBody = BasicBlock::Create(b->getContext(), kernel->getName() + "Do", threadFunc);

        b->SetInsertPoint(kernelWait);
        const unsigned waitIdx = codegen::DebugOptionIsSet(codegen::SerializeThreads) ? (n - 1) : k;

        b->setKernel(kernels[waitIdx]);
        Value * const processedSegmentCount = b->acquireLogicalSegmentNo();
        b->setKernel(kernel);

        assert (processedSegmentCount->getType() == segNo->getType());
        Value * const ready = b->CreateICmpEQ(segNo, processedSegmentCount);

        if (kernel->hasNoTerminateAttribute()) {
            b->CreateCondBr(ready, kernelBody, kernelWait);
        } else { // If the kernel was terminated in a previous segment then the pipeline is done.
            BasicBlock * kernelTerminated = BasicBlock::Create(b->getContext(), kernel->getName() + "Terminated", threadFunc, 0);
            BasicBlock * exitBlock = BasicBlock::Create(b->getContext(), kernel->getName() + "Exit", threadFunc, 0);
            b->CreateCondBr(ready, kernelTerminated, kernelWait);

            b->SetInsertPoint(kernelTerminated);
            Value * terminationSignal = b->getTerminationSignal();
            b->CreateCondBr(terminationSignal, exitBlock, kernelBody);
            b->SetInsertPoint(exitBlock);
            b->releaseLogicalSegmentNo(nextSegNo); // Ensure that the next thread will also exit.
            b->CreateBr(exitThreadBlock);
        }

        BasicBlock * const kernelEnd = BasicBlock::Create(b->getContext(), kernel->getName() + "End", threadFunc);

        // Execute the kernel segment
        b->SetInsertPoint(kernelBody);
        const auto & inputs = kernel->getStreamInputs();
        std::vector<Value *> args = {kernel->getInstance(), terminated};
        for (unsigned i = 0; i < inputs.size(); ++i) {
            const StreamSetBuffer * const buffer = kernel->getStreamSetInputBuffer(i);
            const auto f = producedItemCount.find(buffer);
            assert (f != producedItemCount.end());
            Value * const produced = f->second;
            args.push_back(produced);
            handleInsufficientData(b, produced, terminated, kernelEnd, kernel, inputs[i], buffer);
        }

        b->setKernel(kernel);
        b->createDoSegmentCall(args);
        b->CreateBr(kernelEnd);

        b->SetInsertPoint(kernelEnd);

        if (!kernel->hasNoTerminateAttribute()) {
            terminated = b->CreateOr(terminated, b->getTerminationSignal());
        }

        const auto & outputs = kernel->getStreamOutputs();
        for (unsigned i = 0; i < outputs.size(); ++i) {            
            Value * const produced = b->getProducedItemCount(outputs[i].getName());
            const StreamSetBuffer * const buf = kernel->getStreamSetOutputBuffer(i);
            assert (producedItemCount.count(buf) == 0);
            producedItemCount.emplace(buf, produced);
        }
        for (unsigned i = 0; i < inputs.size(); ++i) {
            Value * const processedItemCount = b->getProcessedItemCount(inputs[i].getName());
            const StreamSetBuffer * const buf = kernel->getStreamSetInputBuffer(i);            
            auto f = consumedItemCount.find(buf);
            if (f == consumedItemCount.end()) {
                consumedItemCount.emplace(buf, processedItemCount);
            } else {
                assert (f->second);
                f->second = b->CreateUMin(processedItemCount, f->second);
            }
        }

        if (DebugOptionIsSet(codegen::EnableCycleCounter)) {
            cycleCountEnd = b->CreateReadCycleCounter();
            Value * counterPtr = b->getCycleCountPtr();
            b->CreateStore(b->CreateAdd(b->CreateLoad(counterPtr), b->CreateSub(cycleCountEnd, cycleCountStart)), counterPtr);
            cycleCountStart = cycleCountEnd;
        }        
        b->releaseLogicalSegmentNo(nextSegNo);
    }

    exitThreadBlock->moveAfter(b->GetInsertBlock());
    for (const auto consumed : consumedItemCount) {
        const StreamSetBuffer * const buf = consumed.first;
        Kernel * const k = buf->getProducer();
        const auto & outputs = k->getStreamSetOutputBuffers();
        for (unsigned i = 0; i < outputs.size(); ++i) {
            if (outputs[i] == buf) {
                const auto & binding = k->getStreamOutput(i);
                if (LLVM_UNLIKELY(binding.getRate().isDerived())) {
                    continue;
                }
                b->setKernel(k);
                b->setConsumedItemCount(binding.getName(), consumed.second);
                break;
            }
        }
    }

    segNo->addIncoming(b->CreateAdd(segNo, b->getSize(codegen::ThreadNum)), b->GetInsertBlock());
    b->CreateUnlikelyCondBr(terminated, exitThreadBlock, segmentLoop);

    b->SetInsertPoint(exitThreadBlock);

    // only call pthread_exit() within spawned threads; otherwise it'll be equivalent to calling exit() within the process
    BasicBlock * const exitThread = BasicBlock::Create(b->getContext(), "ExitThread", threadFunc);
    BasicBlock * const exitFunction = BasicBlock::Create(b->getContext(), "ExitProcessFunction", threadFunc);

    Value * const exitCond = b->CreateICmpEQ(segOffset, ConstantInt::getNullValue(segOffset->getType()));
    b->CreateCondBr(exitCond, exitFunction, exitThread);
    b->SetInsertPoint(exitThread);
    b->CreatePThreadExitCall(nullVoidPtrVal);
    b->CreateBr(exitFunction);
    b->SetInsertPoint(exitFunction);
    b->CreateRetVoid();

    // -------------------------------------------------------------------------------------------------------------------------
    b->restoreIP(ip);

    for (unsigned i = 0; i < n; ++i) {
        kernels[i]->setInstance(instance[i]);
    }

    // -------------------------------------------------------------------------------------------------------------------------
    // MAKE SEGMENT PARALLEL PIPELINE DRIVER
    // -------------------------------------------------------------------------------------------------------------------------
    const unsigned threads = codegen::ThreadNum - 1;
    assert (codegen::ThreadNum > 1);
    Type * const pthreadsTy = ArrayType::get(sizeTy, threads);
    AllocaInst * const pthreads = b->CreateAlloca(pthreadsTy);
    Value * threadIdPtr[threads];

    for (unsigned i = 0; i < threads; ++i) {
        threadIdPtr[i] = b->CreateGEP(pthreads, {b->getInt32(0), b->getInt32(i)});
    }

    for (unsigned i = 0; i < n; ++i) {
        b->setKernel(kernels[i]);
        b->releaseLogicalSegmentNo(b->getSize(0));
    }

    AllocaInst * const sharedStruct = b->CreateCacheAlignedAlloca(sharedStructType);
    for (unsigned i = 0; i < n; ++i) {
        Value * ptr = b->CreateGEP(sharedStruct, {b->getInt32(0), b->getInt32(i)});
        b->CreateStore(kernels[i]->getInstance(), ptr);
    }

    // use the process thread to handle the initial segment function after spawning (n - 1) threads to handle the subsequent offsets
    for (unsigned i = 0; i < threads; ++i) {
        AllocaInst * const threadState = b->CreateAlloca(threadStructType);
        b->CreateStore(sharedStruct, b->CreateGEP(threadState, {b->getInt32(0), b->getInt32(0)}));
        b->CreateStore(b->getSize(i + 1), b->CreateGEP(threadState, {b->getInt32(0), b->getInt32(1)}));
        b->CreatePThreadCreateCall(threadIdPtr[i], nullVoidPtrVal, threadFunc, threadState);
    }

    AllocaInst * const threadState = b->CreateAlloca(threadStructType);
    b->CreateStore(sharedStruct, b->CreateGEP(threadState, {b->getInt32(0), b->getInt32(0)}));
    b->CreateStore(b->getSize(0), b->CreateGEP(threadState, {b->getInt32(0), b->getInt32(1)}));
    b->CreateCall(threadFunc, b->CreatePointerCast(threadState, voidPtrTy));

    AllocaInst * const status = b->CreateAlloca(voidPtrTy);
    for (unsigned i = 0; i < threads; ++i) {
        Value * threadId = b->CreateLoad(threadIdPtr[i]);
        b->CreatePThreadJoinCall(threadId, status);
    }
    
    if (DebugOptionIsSet(codegen::EnableCycleCounter)) {
        for (unsigned k = 0; k < kernels.size(); k++) {
            auto & kernel = kernels[k];
            b->setKernel(kernel);
            const auto & inputs = kernel->getStreamInputs();
            const auto & outputs = kernel->getStreamOutputs();
            Value * items = nullptr;
            if (inputs.empty()) {
                items = b->getProducedItemCount(outputs[0].getName());
            } else {
                items = b->getProcessedItemCount(inputs[0].getName());
            }
            Value * fItems = b->CreateUIToFP(items, b->getDoubleTy());
            Value * cycles = b->CreateLoad(b->getCycleCountPtr());
            Value * fCycles = b->CreateUIToFP(cycles, b->getDoubleTy());
            const auto formatString = kernel->getName() + ": %7.2e items processed; %7.2e CPU cycles,  %6.2f cycles per item.\n";
            Value * stringPtr = b->CreatePointerCast(b->GetString(formatString), b->getInt8PtrTy());
            b->CreateCall(b->GetDprintf(), {b->getInt32(2), stringPtr, fItems, fCycles, b->CreateFDiv(fCycles, fItems)});
        }
    }
    
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateParallelPipeline
 ** ------------------------------------------------------------------------------------------------------------- */
void generateParallelPipeline(const std::unique_ptr<KernelBuilder> & iBuilder, const std::vector<Kernel *> &kernels) {

    Module * const m = iBuilder->getModule();
    IntegerType * const sizeTy = iBuilder->getSizeTy();
    PointerType * const voidPtrTy = iBuilder->getVoidPtrTy();
    ConstantInt * bufferSegments = ConstantInt::get(sizeTy, codegen::BufferSegments - 1);
    ConstantInt * segmentItems = ConstantInt::get(sizeTy, codegen::SegmentSize * iBuilder->getBitBlockWidth());
    Constant * const nullVoidPtrVal = ConstantPointerNull::getNullValue(voidPtrTy);

    const unsigned n = kernels.size();

    Type * const pthreadsTy = ArrayType::get(sizeTy, n);
    AllocaInst * const pthreads = iBuilder->CreateAlloca(pthreadsTy);
    Value * threadIdPtr[n];
    for (unsigned i = 0; i < n; ++i) {
        threadIdPtr[i] = iBuilder->CreateGEP(pthreads, {iBuilder->getInt32(0), iBuilder->getInt32(i)});
    }

    Value * instance[n];
    Type * structTypes[n];
    for (unsigned i = 0; i < n; ++i) {
        instance[i] = kernels[i]->getInstance();
        structTypes[i] = instance[i]->getType();
    }

    Type * const sharedStructType = StructType::get(m->getContext(), ArrayRef<Type *>{structTypes, n});


    AllocaInst * sharedStruct = iBuilder->CreateAlloca(sharedStructType);
    for (unsigned i = 0; i < n; ++i) {
        Value * ptr = iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(i)});
        iBuilder->CreateStore(instance[i], ptr);
    }

    for (auto & kernel : kernels) {
        iBuilder->setKernel(kernel);
        iBuilder->releaseLogicalSegmentNo(iBuilder->getSize(0));
    }

    // GENERATE THE PRODUCING AND CONSUMING KERNEL MAPS
    StreamSetBufferMap<unsigned> producingKernel;
    StreamSetBufferMap<std::vector<unsigned>> consumingKernels;
    for (unsigned id = 0; id < n; ++id) {
        const auto & kernel = kernels[id];
        const auto & inputs = kernel->getStreamInputs();
        const auto & outputs = kernel->getStreamOutputs();
        // add any outputs from this kernel to the producing kernel map
        for (unsigned j = 0; j < outputs.size(); ++j) {
            const StreamSetBuffer * const buf = kernel->getStreamSetOutputBuffer(j);
            if (LLVM_UNLIKELY(producingKernel.count(buf) != 0)) {
                report_fatal_error(kernel->getName() + " redefines stream set " + outputs[j].getName());
            }
            producingKernel.emplace(buf, id);
        }
        // and any inputs to the consuming kernels list
        for (unsigned j = 0; j < inputs.size(); ++j) {
            const StreamSetBuffer * const buf = kernel->getStreamSetInputBuffer(j);
            auto f = consumingKernels.find(buf);
            if (f == consumingKernels.end()) {
                if (LLVM_UNLIKELY(producingKernel.count(buf) == 0)) {
                    report_fatal_error(kernel->getName() + " uses stream set " + inputs[j].getName() + " prior to its definition");
                }
                consumingKernels.emplace(buf, std::vector<unsigned>{ id });
            } else {
                f->second.push_back(id);
            }
        }
    }

    const auto ip = iBuilder->saveIP();

    // GENERATE UNIQUE PIPELINE PARALLEL THREAD FUNCTION FOR EACH KERNEL
    FlatSet<unsigned> kernelSet;
    kernelSet.reserve(n);

    Function * thread_functions[n];
    Value * producerSegNo[n];
    for (unsigned id = 0; id < n; id++) {
        const auto & kernel = kernels[id];

        iBuilder->setKernel(kernel);

        const auto & inputs = kernel->getStreamInputs();

        Function * const threadFunc = makeThreadFunction(iBuilder, "ppt:" + kernel->getName());
        auto ai = threadFunc->arg_begin();
        
         // Create the basic blocks for the thread function.
        BasicBlock * entryBlock = BasicBlock::Create(iBuilder->getContext(), "entry", threadFunc);
        BasicBlock * outputCheckBlock = BasicBlock::Create(iBuilder->getContext(), "outputCheck", threadFunc);
        BasicBlock * inputCheckBlock = BasicBlock::Create(iBuilder->getContext(), "inputCheck", threadFunc);
        BasicBlock * doSegmentBlock = BasicBlock::Create(iBuilder->getContext(), "doSegment", threadFunc);
        BasicBlock * exitThreadBlock = BasicBlock::Create(iBuilder->getContext(), "exitThread", threadFunc);

        iBuilder->SetInsertPoint(entryBlock);

        Value * const sharedStruct = iBuilder->CreateBitCast(&*(ai), sharedStructType->getPointerTo());

        for (unsigned k = 0; k < n; k++) {
            Value * const ptr = iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(k)});
            kernels[k]->setInstance(iBuilder->CreateLoad(ptr));
        }

        iBuilder->CreateBr(outputCheckBlock);

        // Check whether the output buffers are ready for more data
        iBuilder->SetInsertPoint(outputCheckBlock);
        PHINode * segNo = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3, "segNo");
        segNo->addIncoming(iBuilder->getSize(0), entryBlock);
        segNo->addIncoming(segNo, outputCheckBlock);

        Value * outputWaitCond = iBuilder->getTrue();
        for (const StreamSetBuffer * buf : kernel->getStreamSetOutputBuffers()) {
            const auto & list = consumingKernels[buf];
            assert(std::is_sorted(list.begin(), list.end()));
            kernelSet.insert(list.begin(), list.end());
        }
        for (unsigned k : kernelSet) {
            iBuilder->setKernel(kernels[k]);
            Value * consumerSegNo = iBuilder->acquireLogicalSegmentNo();
            assert (consumerSegNo->getType() == segNo->getType());
            Value * consumedSegNo = iBuilder->CreateAdd(consumerSegNo, bufferSegments);
            outputWaitCond = iBuilder->CreateAnd(outputWaitCond, iBuilder->CreateICmpULE(segNo, consumedSegNo));
        }
        kernelSet.clear();
        iBuilder->setKernel(kernel);
        iBuilder->CreateCondBr(outputWaitCond, inputCheckBlock, outputCheckBlock);

        // Check whether the input buffers have enough data for this kernel to begin
        iBuilder->SetInsertPoint(inputCheckBlock);
        for (const StreamSetBuffer * buf : kernel->getStreamSetInputBuffers()) {
            kernelSet.insert(producingKernel[buf]);
        }

        Value * inputWaitCond = iBuilder->getTrue();
        for (unsigned k : kernelSet) {
            iBuilder->setKernel(kernels[k]);
            producerSegNo[k] = iBuilder->acquireLogicalSegmentNo();
            assert (producerSegNo[k]->getType() == segNo->getType());
            inputWaitCond = iBuilder->CreateAnd(inputWaitCond, iBuilder->CreateICmpULT(segNo, producerSegNo[k]));
        }
        iBuilder->setKernel(kernel);
        iBuilder->CreateCondBr(inputWaitCond, doSegmentBlock, inputCheckBlock);

        // Process the segment
        iBuilder->SetInsertPoint(doSegmentBlock);

        Value * const nextSegNo = iBuilder->CreateAdd(segNo, iBuilder->getSize(1));
        Value * terminated = nullptr;
        if (kernelSet.empty()) {
            // if this kernel has no input streams, the kernel itself must decide when it terminates.
            terminated = iBuilder->getTerminationSignal();
        } else {
            // ... otherwise the kernel terminates only when it exhausts all of its input streams
            terminated = iBuilder->getTrue();
            for (unsigned k : kernelSet) {
                iBuilder->setKernel(kernels[k]);
                terminated = iBuilder->CreateAnd(terminated, iBuilder->getTerminationSignal());
                terminated = iBuilder->CreateAnd(terminated, iBuilder->CreateICmpEQ(nextSegNo, producerSegNo[k]));
            }
            kernelSet.clear();
            iBuilder->setKernel(kernel);
        }

        std::vector<Value *> args = {kernel->getInstance(), terminated};
        args.insert(args.end(), inputs.size(), iBuilder->CreateMul(segmentItems, segNo));

        iBuilder->createDoSegmentCall(args);
        segNo->addIncoming(nextSegNo, doSegmentBlock);
        iBuilder->releaseLogicalSegmentNo(nextSegNo);

        iBuilder->CreateCondBr(terminated, exitThreadBlock, outputCheckBlock);

        iBuilder->SetInsertPoint(exitThreadBlock);

        iBuilder->CreatePThreadExitCall(nullVoidPtrVal);

        iBuilder->CreateRetVoid();

        thread_functions[id] = threadFunc;
    }

    iBuilder->restoreIP(ip);

    for (unsigned i = 0; i < n; ++i) {
        kernels[i]->setInstance(instance[i]);
    }

    for (unsigned i = 0; i < n; ++i) {
        iBuilder->CreatePThreadCreateCall(threadIdPtr[i], nullVoidPtrVal, thread_functions[i], sharedStruct);
    }

    AllocaInst * const status = iBuilder->CreateAlloca(voidPtrTy);
    for (unsigned i = 0; i < n; ++i) {
        Value * threadId = iBuilder->CreateLoad(threadIdPtr[i]);
        iBuilder->CreatePThreadJoinCall(threadId, status);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generatePipelineLoop
 ** ------------------------------------------------------------------------------------------------------------- */
void generatePipelineLoop(const std::unique_ptr<KernelBuilder> & b, const std::vector<Kernel *> & kernels) {

    BasicBlock * entryBlock = b->GetInsertBlock();
    Function * main = entryBlock->getParent();

    // Create the basic blocks for the loop.
    BasicBlock * pipelineLoop = BasicBlock::Create(b->getContext(), "pipelineLoop", main);
    BasicBlock * pipelineExit = BasicBlock::Create(b->getContext(), "pipelineExit", main);

    StreamSetBufferMap<Value *> producedItemCount;
    StreamSetBufferMap<Value *> consumedItemCount;

    b->CreateBr(pipelineLoop);
    b->SetInsertPoint(pipelineLoop);
    
    Value * cycleCountStart = nullptr;
    Value * cycleCountEnd = nullptr;
    if (DebugOptionIsSet(codegen::EnableCycleCounter)) {
        cycleCountStart = b->CreateReadCycleCounter();
    }
    Value * terminated = b->getFalse();

    for (Kernel * const kernel : kernels) {

        b->setKernel(kernel);
        const auto & inputs = kernel->getStreamInputs();
        const auto & outputs = kernel->getStreamOutputs();

        std::vector<Value *> args = {kernel->getInstance(), terminated};

        for (unsigned i = 0; i < inputs.size(); ++i) {
            const StreamSetBuffer * const buffer = kernel->getStreamSetInputBuffer(i);
            const auto f = producedItemCount.find(buffer);
            if (LLVM_UNLIKELY(f == producedItemCount.end())) {
                report_fatal_error(kernel->getName() + " uses stream set " + inputs[i].getName() + " prior to its definition");
            }
            Value * const produced = f->second;
            args.push_back(produced);
            handleInsufficientData(b, produced, terminated, pipelineLoop, kernel, inputs[i], buffer);
        }

        applyOutputBufferExpansions(b, kernel);

        b->createDoSegmentCall(args);

        if (!kernel->hasNoTerminateAttribute()) {
            Value * terminatedSignal = b->getTerminationSignal();
            terminated = b->CreateOr(terminated, terminatedSignal);
        }
        for (unsigned i = 0; i < outputs.size(); ++i) {
            Value * const produced = b->getProducedItemCount(outputs[i].getName());
            const StreamSetBuffer * const buf = kernel->getStreamSetOutputBuffer(i);
            assert (producedItemCount.count(buf) == 0);
            producedItemCount.emplace(buf, produced);
        }

        for (unsigned i = 0; i < inputs.size(); ++i) {
            Value * const processed = b->getProcessedItemCount(inputs[i].getName());
            const StreamSetBuffer * const buf = kernel->getStreamSetInputBuffer(i);
            auto f = consumedItemCount.find(buf);
            if (f == consumedItemCount.end()) {
                consumedItemCount.emplace(buf, processed);
            } else {
                f->second = b->CreateUMin(processed, f->second);
            }
        }

        if (DebugOptionIsSet(codegen::EnableCycleCounter)) {
            cycleCountEnd = b->CreateReadCycleCounter();
            Value * counterPtr = b->getCycleCountPtr();
            b->CreateStore(b->CreateAdd(b->CreateLoad(counterPtr), b->CreateSub(cycleCountEnd, cycleCountStart)), counterPtr);
            cycleCountStart = cycleCountEnd;
        }
//        Value * const segNo = b->acquireLogicalSegmentNo();
//        Value * nextSegNo = b->CreateAdd(segNo, b->getSize(1));
//        b->releaseLogicalSegmentNo(nextSegNo);
    }

    for (const auto consumed : consumedItemCount) {
        const StreamSetBuffer * const buffer = consumed.first;
        Kernel * const kernel = buffer->getProducer();
        const auto & binding = kernel->getStreamOutput(buffer);
        if (LLVM_UNLIKELY(binding.getRate().isDerived())) {
            continue;
        }
        b->setKernel(kernel);
        b->setConsumedItemCount(binding.getName(), consumed.second);
    }

    b->CreateCondBr(terminated, pipelineExit, pipelineLoop);

    b->SetInsertPoint(pipelineExit);

    if (DebugOptionIsSet(codegen::EnableCycleCounter)) {
        for (unsigned k = 0; k < kernels.size(); k++) {
            auto & kernel = kernels[k];
            b->setKernel(kernel);
            const auto & inputs = kernel->getStreamInputs();
            const auto & outputs = kernel->getStreamOutputs();
            Value * items = nullptr;
            if (inputs.empty()) {
                items = b->getProducedItemCount(outputs[0].getName());
            } else {
                items = b->getProcessedItemCount(inputs[0].getName());
            }
            Value * fItems = b->CreateUIToFP(items, b->getDoubleTy());
            Value * cycles = b->CreateLoad(b->getCycleCountPtr());
            Value * fCycles = b->CreateUIToFP(cycles, b->getDoubleTy());
            const auto formatString = kernel->getName() + ": %7.2e items processed; %7.2e CPU cycles,  %6.2f cycles per item.\n";
            Value * stringPtr = b->CreatePointerCast(b->GetString(formatString), b->getInt8PtrTy());
            b->CreateCall(b->GetDprintf(), {b->getInt32(2), stringPtr, fItems, fCycles, b->CreateFDiv(fCycles, fItems)});
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief applyOutputBufferExpansions
 ** ------------------------------------------------------------------------------------------------------------- */
void applyOutputBufferExpansions(const std::unique_ptr<KernelBuilder> & b, const std::string & name, DynamicBuffer * const db, const uint64_t baseSize) {
    BasicBlock * const doExpand = BasicBlock::Create(b->getContext(), name + "Expand", b->GetInsertBlock()->getParent());
    BasicBlock * const nextBlock = b->GetInsertBlock()->getNextNode();
    doExpand->moveAfter(b->GetInsertBlock());
    BasicBlock * const bufferReady = b->CreateBasicBlock(name + "Ready");
    bufferReady->moveAfter(doExpand);
    if (nextBlock) nextBlock->moveAfter(bufferReady);

    Value * const handle = db->getStreamSetHandle();

    Value * const produced = b->getProducedItemCount(name);
    Value * const consumed = b->getConsumedItemCount(name);
    Value * const required = b->CreateAdd(b->CreateSub(produced, consumed), b->getSize(2 * baseSize));

    b->CreateCondBr(b->CreateICmpUGT(required, db->getCapacity(b.get(), handle)), doExpand, bufferReady);

    b->SetInsertPoint(doExpand);
    db->doubleCapacity(b.get(), handle);
    // Ensure that capacity is sufficient by successive doubling, if necessary.
    b->CreateCondBr(b->CreateICmpUGT(required, db->getBufferedSize(b.get(), handle)), doExpand, bufferReady);

    b->SetInsertPoint(bufferReady);
}

void applyOutputBufferExpansions(const std::unique_ptr<KernelBuilder> & b, const Kernel * k) {
    const auto & outputs = k->getStreamSetOutputBuffers();
    for (unsigned i = 0; i < outputs.size(); i++) {
        if (isa<DynamicBuffer>(outputs[i])) {
            const auto ub = k->getUpperBound(k->getStreamOutput(i).getRate());
            const auto baseSize = (ub.numerator() * k->getStride() + ub.denominator() - 1) / ub.denominator();
            if (LLVM_LIKELY(baseSize > 0)) {
                const auto & name = k->getStreamOutput(i).getName();
                applyOutputBufferExpansions(b, name, cast<DynamicBuffer>(outputs[i]), baseSize);
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief handleInsufficientData
 ** ------------------------------------------------------------------------------------------------------------- */
inline void handleInsufficientData(const std::unique_ptr<KernelBuilder> & b, Value * const produced, Value * const final, BasicBlock * const insufficient,
                                   const Kernel * const consumer,  const Binding & input, const StreamSetBuffer * const buffer) {
    const Kernel * const producer = buffer->getProducer();
    const Binding & output = producer->getStreamOutput(buffer);
    auto producedRate = producer->getLowerBound(output.getRate()) * producer->getStride();
    const auto consumedRate = consumer->getUpperBound(input.getRate()) * consumer->getStride();
    if (LLVM_UNLIKELY(input.hasLookahead())) {
        producedRate -= input.getLookahead();
//        const auto amount = input.getLookahead();
//        const auto strides = ((amount + consumer->getStride() - 1) / consumer->getStride());
//        consumedRate += strides * consumer->getStride();
    }
    if (LLVM_UNLIKELY(producedRate < consumedRate)) {
        const auto name = input.getName();
        BasicBlock * const sufficient = BasicBlock::Create(b->getContext(), name + "IsSufficient", b->GetInsertBlock()->getParent());
        Value * const processed = b->getProcessedItemCount(name);
        Value * const unread = b->CreateSub(produced, processed);
        Constant * const amount = ConstantInt::get(unread->getType(), ceiling(consumedRate));
        Value * const cond = b->CreateOr(b->CreateICmpUGE(unread, amount), final);
        b->CreateLikelyCondBr(cond, sufficient, insufficient);
        b->SetInsertPoint(sufficient);
    }
}

