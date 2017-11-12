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
    f->arg_begin()->setName("input");
    return f;
}

void applyOutputBufferExpansions(const std::unique_ptr<KernelBuilder> & b, const Kernel * kernel);

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateSegmentParallelPipeline
 *
 * Given a computation expressed as a logical pipeline of K kernels k0, k_1, ...k_(K-1)
 * operating over an input stream set S, a segment-parallel implementation divides the input
 * into segments and coordinates a set of T <= K threads to each process one segment at a time.
 * Let S_0, S_1, ... S_N be the segments of S.   Segments are assigned to threads in a round-robin
 * fashion such that processing of segment S_i by the full pipeline is carried out by thread i mod T.
 ** ------------------------------------------------------------------------------------------------------------- */
void generateSegmentParallelPipeline(const std::unique_ptr<KernelBuilder> & iBuilder, const std::vector<Kernel *> & kernels) {

    const unsigned n = kernels.size();
    Module * const m = iBuilder->getModule();
    IntegerType * const sizeTy = iBuilder->getSizeTy();
    PointerType * const voidPtrTy = iBuilder->getVoidPtrTy();
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

    Function * const threadFunc = makeThreadFunction(iBuilder, "segment");
    Function::arg_iterator args = threadFunc->arg_begin();
    Value * threadStruct = iBuilder->CreateBitCast(&*(args), threadStructType->getPointerTo());

    // -------------------------------------------------------------------------------------------------------------------------
    // MAKE SEGMENT PARALLEL PIPELINE THREAD
    // -------------------------------------------------------------------------------------------------------------------------
    const auto ip = iBuilder->saveIP();

     // Create the basic blocks for the thread function.
    BasicBlock * entryBlock = BasicBlock::Create(iBuilder->getContext(), "entry", threadFunc);
    iBuilder->SetInsertPoint(entryBlock);
    Value * const sharedStatePtr = iBuilder->CreateLoad(iBuilder->CreateGEP(threadStruct, {iBuilder->getInt32(0), iBuilder->getInt32(0)}));
    for (unsigned k = 0; k < n; ++k) {
        Value * ptr = iBuilder->CreateLoad(iBuilder->CreateGEP(sharedStatePtr, {iBuilder->getInt32(0), iBuilder->getInt32(k)}));
        kernels[k]->setInstance(ptr);
    }
    Value * const segOffset = iBuilder->CreateLoad(iBuilder->CreateGEP(threadStruct, {iBuilder->getInt32(0), iBuilder->getInt32(1)}));

    BasicBlock * segmentLoop = BasicBlock::Create(iBuilder->getContext(), "segmentLoop", threadFunc);
    iBuilder->CreateBr(segmentLoop);

    iBuilder->SetInsertPoint(segmentLoop);
    PHINode * const segNo = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "segNo");
    segNo->addIncoming(segOffset, entryBlock);

    Value * terminated = iBuilder->getFalse();
    Value * const nextSegNo = iBuilder->CreateAdd(segNo, iBuilder->getSize(1));

    BasicBlock * segmentLoopBody = nullptr;
    BasicBlock * const exitThreadBlock = BasicBlock::Create(iBuilder->getContext(), "exitThread", threadFunc);

    StreamSetBufferMap<Value *> producedPos;
    StreamSetBufferMap<Value *> consumedPos;

    Value * cycleCountStart = nullptr;
    Value * cycleCountEnd = nullptr;
    if (DebugOptionIsSet(codegen::EnableCycleCounter)) {
        cycleCountStart = iBuilder->CreateReadCycleCounter();
    }

    for (unsigned k = 0; k < n; ++k) {

        const auto & kernel = kernels[k];

        BasicBlock * const segmentWait = BasicBlock::Create(iBuilder->getContext(), kernel->getName() + "Wait", threadFunc);

        BasicBlock * segmentYield = segmentWait;
        iBuilder->CreateBr(segmentWait);

        segmentLoopBody = BasicBlock::Create(iBuilder->getContext(), kernel->getName() + "Do", threadFunc);

        iBuilder->SetInsertPoint(segmentWait);
        const unsigned waitIdx = codegen::DebugOptionIsSet(codegen::SerializeThreads) ? (n - 1) : k;

        iBuilder->setKernel(kernels[waitIdx]);
        Value * const processedSegmentCount = iBuilder->acquireLogicalSegmentNo();
        iBuilder->setKernel(kernel);

        assert (processedSegmentCount->getType() == segNo->getType());
        Value * const ready = iBuilder->CreateICmpEQ(segNo, processedSegmentCount);

        if (kernel->hasNoTerminateAttribute()) {
            iBuilder->CreateCondBr(ready, segmentLoopBody, segmentYield);
        } else { // If the kernel was terminated in a previous segment then the pipeline is done.
            BasicBlock * completionTest = BasicBlock::Create(iBuilder->getContext(), kernel->getName() + "Completed", threadFunc, 0);
            BasicBlock * exitBlock = BasicBlock::Create(iBuilder->getContext(), kernel->getName() + "Exit", threadFunc, 0);
            iBuilder->CreateCondBr(ready, completionTest, segmentYield);

            iBuilder->SetInsertPoint(completionTest);
            Value * terminationSignal = iBuilder->getTerminationSignal();
            iBuilder->CreateCondBr(terminationSignal, exitBlock, segmentLoopBody);
            iBuilder->SetInsertPoint(exitBlock);
            // Ensure that the next thread will also exit.
            iBuilder->releaseLogicalSegmentNo(nextSegNo);
            iBuilder->CreateBr(exitThreadBlock);
        }

        // Execute the kernel segment
        iBuilder->SetInsertPoint(segmentLoopBody);
        const auto & inputs = kernel->getStreamInputs();
        std::vector<Value *> args = {kernel->getInstance(), terminated};
        for (unsigned i = 0; i < inputs.size(); ++i) {
            const auto f = producedPos.find(kernel->getStreamSetInputBuffer(i));
            assert (f != producedPos.end());
            args.push_back(f->second);
        }

        iBuilder->setKernel(kernel);
        iBuilder->createDoSegmentCall(args);
        if (!kernel->hasNoTerminateAttribute()) {
            terminated = iBuilder->CreateOr(terminated, iBuilder->getTerminationSignal());
        }

        const auto & outputs = kernel->getStreamOutputs();
        for (unsigned i = 0; i < outputs.size(); ++i) {            
            Value * const produced = iBuilder->getProducedItemCount(outputs[i].getName()); // terminated
            const StreamSetBuffer * const buf = kernel->getStreamSetOutputBuffer(i);
            assert (producedPos.count(buf) == 0);
            producedPos.emplace(buf, produced);
        }
        for (unsigned i = 0; i < inputs.size(); ++i) {
            Value * const processedItemCount = iBuilder->getProcessedItemCount(inputs[i].getName());
            const StreamSetBuffer * const buf = kernel->getStreamSetInputBuffer(i);            
            auto f = consumedPos.find(buf);
            if (f == consumedPos.end()) {
                consumedPos.emplace(buf, processedItemCount);
            } else {
                Value * lesser = iBuilder->CreateICmpULT(processedItemCount, f->second);
                f->second = iBuilder->CreateSelect(lesser, processedItemCount, f->second);
            }
        }
        if (DebugOptionIsSet(codegen::EnableCycleCounter)) {
            cycleCountEnd = iBuilder->CreateReadCycleCounter();
            Value * counterPtr = iBuilder->getCycleCountPtr();
            iBuilder->CreateStore(iBuilder->CreateAdd(iBuilder->CreateLoad(counterPtr), iBuilder->CreateSub(cycleCountEnd, cycleCountStart)), counterPtr);
            cycleCountStart = cycleCountEnd;
        }
        
        iBuilder->releaseLogicalSegmentNo(nextSegNo);
    }

    assert (segmentLoopBody);
    exitThreadBlock->moveAfter(segmentLoopBody);

    for (const auto consumed : consumedPos) {
        const StreamSetBuffer * const buf = consumed.first;
        Kernel * const k = buf->getProducer();
        const auto & outputs = k->getStreamSetOutputBuffers();
        for (unsigned i = 0; i < outputs.size(); ++i) {
            if (outputs[i] == buf) {
                const auto binding = k->getStreamOutput(i);
                if (LLVM_UNLIKELY(binding.getRate().isDerived())) {
                    continue;
                }
                iBuilder->setKernel(k);
                iBuilder->setConsumedItemCount(binding.getName(), consumed.second);
                break;
            }
        }
    }

    segNo->addIncoming(iBuilder->CreateAdd(segNo, iBuilder->getSize(codegen::ThreadNum)), segmentLoopBody);
    iBuilder->CreateCondBr(terminated, exitThreadBlock, segmentLoop);

    iBuilder->SetInsertPoint(exitThreadBlock);

    // only call pthread_exit() within spawned threads; otherwise it'll be equivalent to calling exit() within the process
    BasicBlock * const exitThread = BasicBlock::Create(iBuilder->getContext(), "ExitThread", threadFunc);
    BasicBlock * const exitFunction = BasicBlock::Create(iBuilder->getContext(), "ExitProcessFunction", threadFunc);

    Value * const exitCond = iBuilder->CreateICmpEQ(segOffset, ConstantInt::getNullValue(segOffset->getType()));
    iBuilder->CreateCondBr(exitCond, exitFunction, exitThread);
    iBuilder->SetInsertPoint(exitThread);
    iBuilder->CreatePThreadExitCall(nullVoidPtrVal);
    iBuilder->CreateBr(exitFunction);
    iBuilder->SetInsertPoint(exitFunction);
    iBuilder->CreateRetVoid();

    // -------------------------------------------------------------------------------------------------------------------------
    iBuilder->restoreIP(ip);

    for (unsigned i = 0; i < n; ++i) {
        kernels[i]->setInstance(instance[i]);
    }

    // -------------------------------------------------------------------------------------------------------------------------
    // MAKE SEGMENT PARALLEL PIPELINE DRIVER
    // -------------------------------------------------------------------------------------------------------------------------
    const unsigned threads = codegen::ThreadNum - 1;
    assert (codegen::ThreadNum > 1);
    Type * const pthreadsTy = ArrayType::get(sizeTy, threads);
    AllocaInst * const pthreads = iBuilder->CreateAlloca(pthreadsTy);
    Value * threadIdPtr[threads];

    for (unsigned i = 0; i < threads; ++i) {
        threadIdPtr[i] = iBuilder->CreateGEP(pthreads, {iBuilder->getInt32(0), iBuilder->getInt32(i)});
    }

    for (unsigned i = 0; i < n; ++i) {
        iBuilder->setKernel(kernels[i]);
        iBuilder->releaseLogicalSegmentNo(iBuilder->getSize(0));
    }

    AllocaInst * const sharedStruct = iBuilder->CreateCacheAlignedAlloca(sharedStructType);
    for (unsigned i = 0; i < n; ++i) {
        Value * ptr = iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(i)});
        iBuilder->CreateStore(kernels[i]->getInstance(), ptr);
    }

    // use the process thread to handle the initial segment function after spawning (n - 1) threads to handle the subsequent offsets
    for (unsigned i = 0; i < threads; ++i) {
        AllocaInst * const threadState = iBuilder->CreateAlloca(threadStructType);
        iBuilder->CreateStore(sharedStruct, iBuilder->CreateGEP(threadState, {iBuilder->getInt32(0), iBuilder->getInt32(0)}));
        iBuilder->CreateStore(iBuilder->getSize(i + 1), iBuilder->CreateGEP(threadState, {iBuilder->getInt32(0), iBuilder->getInt32(1)}));
        iBuilder->CreatePThreadCreateCall(threadIdPtr[i], nullVoidPtrVal, threadFunc, threadState);
    }

    AllocaInst * const threadState = iBuilder->CreateAlloca(threadStructType);
    iBuilder->CreateStore(sharedStruct, iBuilder->CreateGEP(threadState, {iBuilder->getInt32(0), iBuilder->getInt32(0)}));
    iBuilder->CreateStore(iBuilder->getSize(0), iBuilder->CreateGEP(threadState, {iBuilder->getInt32(0), iBuilder->getInt32(1)}));
    iBuilder->CreateCall(threadFunc, iBuilder->CreatePointerCast(threadState, voidPtrTy));

    AllocaInst * const status = iBuilder->CreateAlloca(voidPtrTy);
    for (unsigned i = 0; i < threads; ++i) {
        Value * threadId = iBuilder->CreateLoad(threadIdPtr[i]);
        iBuilder->CreatePThreadJoinCall(threadId, status);
    }
    
    if (DebugOptionIsSet(codegen::EnableCycleCounter)) {
        for (unsigned k = 0; k < kernels.size(); k++) {
            auto & kernel = kernels[k];
            iBuilder->setKernel(kernel);
            const auto & inputs = kernel->getStreamInputs();
            const auto & outputs = kernel->getStreamOutputs();
            Value * items = nullptr;
            if (inputs.empty()) {
                items = iBuilder->getProducedItemCount(outputs[0].getName());
            } else {
                items = iBuilder->getProcessedItemCount(inputs[0].getName());
            }
            Value * fItems = iBuilder->CreateUIToFP(items, iBuilder->getDoubleTy());
            Value * cycles = iBuilder->CreateLoad(iBuilder->getCycleCountPtr());
            Value * fCycles = iBuilder->CreateUIToFP(cycles, iBuilder->getDoubleTy());
            std::string formatString = kernel->getName() + ": %7.2e items processed; %7.2e CPU cycles,  %6.2f cycles per item.\n";
            Value * stringPtr = iBuilder->CreatePointerCast(iBuilder->GetString(formatString), iBuilder->getInt8PtrTy());
            iBuilder->CreateCall(iBuilder->GetDprintf(), {iBuilder->getInt32(2), stringPtr, fItems, fCycles, iBuilder->CreateFDiv(fCycles, fItems)});
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
        Function::arg_iterator ai = threadFunc->arg_begin();
        Value * sharedStruct = iBuilder->CreateBitCast(&*(ai), sharedStructType->getPointerTo());
        
         // Create the basic blocks for the thread function.
        BasicBlock * entryBlock = BasicBlock::Create(iBuilder->getContext(), "entry", threadFunc);
        BasicBlock * outputCheckBlock = BasicBlock::Create(iBuilder->getContext(), "outputCheck", threadFunc);
        BasicBlock * inputCheckBlock = BasicBlock::Create(iBuilder->getContext(), "inputCheck", threadFunc);
        BasicBlock * doSegmentBlock = BasicBlock::Create(iBuilder->getContext(), "doSegment", threadFunc);
        BasicBlock * exitThreadBlock = BasicBlock::Create(iBuilder->getContext(), "exitThread", threadFunc);

        iBuilder->SetInsertPoint(entryBlock);


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
void generatePipelineLoop(const std::unique_ptr<KernelBuilder> & iBuilder, const std::vector<Kernel *> & kernels) {

    BasicBlock * entryBlock = iBuilder->GetInsertBlock();
    Function * main = entryBlock->getParent();

    // Create the basic blocks for the loop.
    BasicBlock * pipelineLoop = BasicBlock::Create(iBuilder->getContext(), "pipelineLoop", main);
    BasicBlock * pipelineExit = BasicBlock::Create(iBuilder->getContext(), "pipelineExit", main);

    StreamSetBufferMap<Value *> producedPos;
    StreamSetBufferMap<Value *> consumedPos;

    iBuilder->CreateBr(pipelineLoop);
    iBuilder->SetInsertPoint(pipelineLoop);
    
    Value * cycleCountStart = nullptr;
    Value * cycleCountEnd = nullptr;
    if (DebugOptionIsSet(codegen::EnableCycleCounter)) {
        cycleCountStart = iBuilder->CreateReadCycleCounter();
    }
    Value * terminated = iBuilder->getFalse();

    for (Kernel * const kernel : kernels) {

        iBuilder->setKernel(kernel);
        const auto & inputs = kernel->getStreamInputs();
        const auto & outputs = kernel->getStreamOutputs();

        std::vector<Value *> args = {kernel->getInstance(), terminated};

        for (unsigned i = 0; i < inputs.size(); ++i) {
            const auto f = producedPos.find(kernel->getStreamSetInputBuffer(i));
            if (LLVM_UNLIKELY(f == producedPos.end())) {
                report_fatal_error(kernel->getName() + " uses stream set " + inputs[i].getName() + " prior to its definition");
            }
            args.push_back(f->second);
        }

        applyOutputBufferExpansions(iBuilder, kernel);

        iBuilder->createDoSegmentCall(args);

        if (!kernel->hasNoTerminateAttribute()) {
            Value * terminatedSignal = iBuilder->getTerminationSignal();
            terminated = iBuilder->CreateOr(terminated, terminatedSignal);
        }
        for (unsigned i = 0; i < outputs.size(); ++i) {
            Value * const produced = iBuilder->getProducedItemCount(outputs[i].getName()); // , terminated
            const StreamSetBuffer * const buf = kernel->getStreamSetOutputBuffer(i);
            assert (producedPos.count(buf) == 0);
            producedPos.emplace(buf, produced);
        }

        for (unsigned i = 0; i < inputs.size(); ++i) {
            Value * const processed = iBuilder->getProcessedItemCount(inputs[i].getName());
            const StreamSetBuffer * const buf = kernel->getStreamSetInputBuffer(i);
            auto f = consumedPos.find(buf);
            if (f == consumedPos.end()) {
                consumedPos.emplace(buf, processed);
            } else {
                Value * lesser = iBuilder->CreateICmpULT(processed, f->second);
                f->second = iBuilder->CreateSelect(lesser, processed, f->second);
            }
        }
        if (DebugOptionIsSet(codegen::EnableCycleCounter)) {
            cycleCountEnd = iBuilder->CreateReadCycleCounter();
            Value * counterPtr = iBuilder->getCycleCountPtr();
            iBuilder->CreateStore(iBuilder->CreateAdd(iBuilder->CreateLoad(counterPtr), iBuilder->CreateSub(cycleCountEnd, cycleCountStart)), counterPtr);
            cycleCountStart = cycleCountEnd;
        }

        Value * const segNo = iBuilder->acquireLogicalSegmentNo();
        Value * nextSegNo = iBuilder->CreateAdd(segNo, iBuilder->getSize(1));
        iBuilder->releaseLogicalSegmentNo(nextSegNo);
    }

    for (const auto consumed : consumedPos) {
        const StreamSetBuffer * const buf = consumed.first;
        Kernel * const k = buf->getProducer();
        const auto & outputs = k->getStreamSetOutputBuffers();
        for (unsigned i = 0; i < outputs.size(); ++i) {
            if (outputs[i] == buf) {
                const auto binding = k->getStreamOutput(i);
                if (LLVM_UNLIKELY(binding.getRate().isDerived())) {
                    continue;
                }
                iBuilder->setKernel(k);
                iBuilder->setConsumedItemCount(binding.getName(), consumed.second);
                break;
            }
        }
    }

    iBuilder->CreateCondBr(terminated, pipelineExit, pipelineLoop);

    iBuilder->SetInsertPoint(pipelineExit);

    if (DebugOptionIsSet(codegen::EnableCycleCounter)) {
        for (unsigned k = 0; k < kernels.size(); k++) {
            auto & kernel = kernels[k];
            iBuilder->setKernel(kernel);
            const auto & inputs = kernel->getStreamInputs();
            const auto & outputs = kernel->getStreamOutputs();
            Value * items = nullptr;
            if (inputs.empty()) {
                items = iBuilder->getProducedItemCount(outputs[0].getName());
            } else {
                items = iBuilder->getProcessedItemCount(inputs[0].getName());
            }
            Value * fItems = iBuilder->CreateUIToFP(items, iBuilder->getDoubleTy());
            Value * cycles = iBuilder->CreateLoad(iBuilder->getCycleCountPtr());
            Value * fCycles = iBuilder->CreateUIToFP(cycles, iBuilder->getDoubleTy());
            std::string formatString = kernel->getName() + ": %7.2e items processed; %7.2e CPU cycles,  %6.2f cycles per item.\n";
            Value * stringPtr = iBuilder->CreatePointerCast(iBuilder->GetString(formatString), iBuilder->getInt8PtrTy());
            iBuilder->CreateCall(iBuilder->GetDprintf(), {iBuilder->getInt32(2), stringPtr, fItems, fCycles, iBuilder->CreateFDiv(fCycles, fItems)});
        }
    }
}

void applyOutputBufferExpansions(const std::unique_ptr<KernelBuilder> & b, const std::string & name, DynamicBuffer * const db, const uint64_t l) {

    BasicBlock * const doExpand = b->CreateBasicBlock(name + "Expand");
    BasicBlock * const nextBlock = b->GetInsertBlock()->getNextNode();
    doExpand->moveAfter(b->GetInsertBlock());
    BasicBlock * const bufferReady = b->CreateBasicBlock(name + "Ready");
    bufferReady->moveAfter(doExpand);
    if (nextBlock) nextBlock->moveAfter(bufferReady);

    Value * const handle = db->getStreamSetHandle();

    Value * const produced = b->getProducedItemCount(name);
    Value * const consumed = b->getConsumedItemCount(name);
    Value * const required = b->CreateAdd(b->CreateSub(produced, consumed), b->getSize(2 * l));

    b->CreateCondBr(b->CreateICmpUGT(required, db->getCapacity(b.get(), handle)), doExpand, bufferReady);

    b->SetInsertPoint(doExpand);
    db->doubleCapacity(b.get(), handle);
    // Ensure that capacity is sufficient by successive doubling, if necessary.
    b->CreateCondBr(b->CreateICmpUGT(required, db->getBufferedSize(b.get(), handle)), doExpand, bufferReady);

    b->SetInsertPoint(bufferReady);
}

inline const Binding & getBinding(const Kernel * k, const std::string & name) {
    Port port; unsigned index;
    std::tie(port, index) = k->getStreamPort(name);
    if (port == Port::Input) {
        return k->getStreamInput(index);
    } else {
        return k->getStreamOutput(index);
    }
}

void applyOutputBufferExpansions(const std::unique_ptr<KernelBuilder> & b, const Kernel * k) {
    const auto & outputs = k->getStreamSetOutputBuffers();
    for (unsigned i = 0; i < outputs.size(); i++) {
        if (isa<DynamicBuffer>(outputs[i])) {
            const ProcessingRate & rate = k->getStreamOutput(i).getRate();
            if (rate.isFixed() || rate.isBounded()) {
                const auto & name = k->getStreamOutput(i).getName();
                const auto l = rate.getUpperBound() * k->getKernelStride();
                applyOutputBufferExpansions(b, name, cast<DynamicBuffer>(outputs[i]), l);
            } else if (rate.isExactlyRelative()) {
                const auto binding = getBinding(k, rate.getReference());
                const ProcessingRate & ref = binding.getRate();
                if (rate.isFixed() || rate.isBounded()) {
                    const auto & name = k->getStreamOutput(i).getName();
                    const auto l = (ref.getUpperBound() * rate.getNumerator() * k->getKernelStride() + rate.getDenominator() - 1) / rate.getDenominator();
                    applyOutputBufferExpansions(b, name, cast<DynamicBuffer>(outputs[i]), l);
                }
            }
        }
    }
}
