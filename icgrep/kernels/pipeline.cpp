/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "pipeline.h"
#include <kernels/toolchain.h>
#include <kernels/kernel.h>
#include <kernels/streamset.h>
#include <llvm/IR/Module.h>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>

using namespace kernel;
using namespace parabix;
using namespace llvm;

template <typename Value>
using StreamSetBufferMap = boost::container::flat_map<const StreamSetBuffer *, Value>;

template <typename Value>
using FlatSet = boost::container::flat_set<Value>;

Function * makeThreadFunction(const std::string & name, Module * const m) {
    LLVMContext & C = m->getContext();
    Type * const voidTy = Type::getVoidTy(C);
    PointerType * const int8PtrTy = Type::getInt8PtrTy(C);
    Function * const f = Function::Create(FunctionType::get(voidTy, {int8PtrTy}, false), Function::InternalLinkage, name, m);
    f->setCallingConv(CallingConv::C);
    f->arg_begin()->setName("input");
    return f;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateSegmentParallelPipeline
 *
 * Given a computation expressed as a logical pipeline of K kernels k0, k_1, ...k_(K-1)
 * operating over an input stream set S, a segment-parallel implementation divides the input
 * into segments and coordinates a set of T <= K threads to each process one segment at a time.
 * Let S_0, S_1, ... S_N be the segments of S.   Segments are assigned to threads in a round-robin
 * fashion such that processing of segment S_i by the full pipeline is carried out by thread i mod T.
 ** ------------------------------------------------------------------------------------------------------------- */
void generateSegmentParallelPipeline(IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels) {

    const unsigned n = kernels.size(); assert (n > 0);
    Module * const m = iBuilder->getModule();
    IntegerType * const sizeTy = iBuilder->getSizeTy();
    PointerType * const voidPtrTy = iBuilder->getVoidPtrTy();
    PointerType * const int8PtrTy = iBuilder->getInt8PtrTy();
    const unsigned threads = codegen::ThreadNum;
    Constant * nullVoidPtrVal = ConstantPointerNull::getNullValue(voidPtrTy);

    std::vector<Type *> structTypes;
    for (unsigned i = 0; i < n; i++) {
        kernels[i]->createInstance();
        structTypes.push_back(kernels[i]->getInstance()->getType());
    }
    StructType * const sharedStructType = StructType::get(m->getContext(), structTypes);
    StructType * const threadStructType = StructType::get(sharedStructType->getPointerTo(), sizeTy, nullptr);

    // -------------------------------------------------------------------------------------------------------------------------
    // MAKE SEGMENT PARALLEL PIPELINE THREAD
    // -------------------------------------------------------------------------------------------------------------------------
    const auto ip = iBuilder->saveIP();
    Function * const threadFunc = makeThreadFunction("sppt", m);

     // Create the basic blocks for the thread function.
    BasicBlock * entryBlock = BasicBlock::Create(iBuilder->getContext(), "entry", threadFunc);
    iBuilder->SetInsertPoint(entryBlock);
    Value * const input = &threadFunc->getArgumentList().front();
    Value * const threadStruct = iBuilder->CreatePointerCast(input, threadStructType->getPointerTo());
    Value * const sharedStatePtr = iBuilder->CreateLoad(iBuilder->CreateGEP(threadStruct, {iBuilder->getInt32(0), iBuilder->getInt32(0)}));
    Value * instance[n];
    for (unsigned k = 0; k < n; k++) {
        Value * ptr = iBuilder->CreateGEP(sharedStatePtr, {iBuilder->getInt32(0), iBuilder->getInt32(k)});
        instance[k] = iBuilder->CreateLoad(ptr);
    }
    Value * const segOffset = iBuilder->CreateLoad(iBuilder->CreateGEP(threadStruct, {iBuilder->getInt32(0), iBuilder->getInt32(1)}));

    BasicBlock * segmentLoop = BasicBlock::Create(iBuilder->getContext(), "segmentLoop", threadFunc);
    iBuilder->CreateBr(segmentLoop);

    iBuilder->SetInsertPoint(segmentLoop);
    PHINode * const segNo = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "segNo");
    segNo->addIncoming(segOffset, entryBlock);

    Value * doFinal = iBuilder->getFalse();
    Value * const nextSegNo = iBuilder->CreateAdd(segNo, iBuilder->getSize(1));

    BasicBlock * segmentWait = BasicBlock::Create(iBuilder->getContext(), kernels[0]->getName() + "Wait", threadFunc);

    BasicBlock * segmentLoopBody = nullptr;

    BasicBlock * const exitThreadBlock = BasicBlock::Create(iBuilder->getContext(), "exitThread", threadFunc);

    iBuilder->CreateBr(segmentWait);

    StreamSetBufferMap<Value *> producedPos;

    for (unsigned k = 0;;) {
        const auto & kernel = kernels[k];
        const unsigned waitIdx = codegen::DebugOptionIsSet(codegen::SerializeThreads) ? (n - 1) : k;

        segmentLoopBody = BasicBlock::Create(iBuilder->getContext(), kernel->getName() + "Do", threadFunc);

        iBuilder->SetInsertPoint(segmentWait);
        Value * const processedSegmentCount = kernels[waitIdx]->acquireLogicalSegmentNo(instance[waitIdx]);
        assert (processedSegmentCount->getType() == segNo->getType());
        Value * const ready = iBuilder->CreateICmpEQ(segNo, processedSegmentCount);

        if (kernel->hasNoTerminateAttribute()) {
            iBuilder->CreateCondBr(ready, segmentLoopBody, segmentWait);
        } else { // If the kernel was terminated in a previous segment then the pipeline is done.
            BasicBlock * completionTest = BasicBlock::Create(iBuilder->getContext(), kernel->getName() + "Completed", threadFunc, 0);
            BasicBlock * exitBlock = BasicBlock::Create(iBuilder->getContext(), kernel->getName() + "Exit", threadFunc, 0);
            iBuilder->CreateCondBr(ready, completionTest, segmentWait);
            iBuilder->SetInsertPoint(completionTest);
            Value * alreadyDone = kernel->getTerminationSignal(instance[k]);
            iBuilder->CreateCondBr(alreadyDone, exitBlock, segmentLoopBody);
            iBuilder->SetInsertPoint(exitBlock);
            // Ensure that the next thread will also exit.
            kernel->releaseLogicalSegmentNo(instance[k], nextSegNo);
            iBuilder->CreateBr(exitThreadBlock);
        }

        // Execute the kernel segment
        iBuilder->SetInsertPoint(segmentLoopBody);
        const auto & inputs = kernel->getStreamInputs();
        const auto & outputs = kernel->getStreamOutputs();
        std::vector<Value *> args = {instance[k], doFinal};
        for (unsigned i = 0; i < inputs.size(); ++i) {
            const auto f = producedPos.find(kernel->getStreamSetInputBuffer(i));
            if (LLVM_UNLIKELY(f == producedPos.end())) {
                report_fatal_error(kernel->getName() + " uses stream set " + inputs[i].name + " prior to its definition");
            }
            args.push_back(f->second);
        }
        for (unsigned i = 0; i < outputs.size(); ++i) {
            args.push_back(iBuilder->getSize(0));
        }
        CallInst * ci = kernel->createDoSegmentCall(args);
        // TODO: investigate whether this actually inlines the function call correctly despite being in a seperate module.
        ci->addAttribute(AttributeSet::FunctionIndex, Attribute::AlwaysInline);

        if (!kernel->hasNoTerminateAttribute()) {
            doFinal = iBuilder->CreateOr(doFinal, kernel->getTerminationSignal(instance[k]));
        }
        for (unsigned i = 0; i < outputs.size(); i++) {
            Value * const produced = kernel->getProducedItemCount(instance[k], outputs[i].name, doFinal);
            const StreamSetBuffer * const buf = kernel->getStreamSetOutputBuffer(i);
            assert (producedPos.count(buf) == 0);
            producedPos.emplace(buf, produced);
        }

        kernel->releaseLogicalSegmentNo(instance[k], nextSegNo);
        if (LLVM_UNLIKELY(++k == n)) {
            assert (segmentLoopBody);
            exitThreadBlock->moveAfter(segmentLoopBody);
            segNo->addIncoming(iBuilder->CreateAdd(segNo, iBuilder->getSize(threads)), segmentLoopBody);
            iBuilder->CreateCondBr(doFinal, exitThreadBlock, segmentLoop);

            iBuilder->SetInsertPoint(exitThreadBlock);
            iBuilder->CreatePThreadExitCall(nullVoidPtrVal);
            iBuilder->CreateRetVoid();
            break;
        } else {
            segmentWait = BasicBlock::Create(iBuilder->getContext(), kernels[k]->getName() + "Wait", threadFunc);
            iBuilder->CreateBr(segmentWait);
        }
    }

    // -------------------------------------------------------------------------------------------------------------------------
    iBuilder->restoreIP(ip);

    // -------------------------------------------------------------------------------------------------------------------------
    // MAKE SEGMENT PARALLEL PIPELINE DRIVER
    // -------------------------------------------------------------------------------------------------------------------------

    Type * const pthreadsTy = ArrayType::get(sizeTy, threads);
    AllocaInst * const pthreads = iBuilder->CreateAlloca(pthreadsTy);
    Value * threadIdPtr[threads];
    for (unsigned i = 0; i < threads; i++) {
        threadIdPtr[i] = iBuilder->CreateGEP(pthreads, {iBuilder->getInt32(0), iBuilder->getInt32(i)});
    }

    for (unsigned i = 0; i < n; i++) {
        kernels[i]->releaseLogicalSegmentNo(kernels[i]->getInstance(), iBuilder->getSize(0));
    }

    AllocaInst * const sharedStruct = iBuilder->CreateCacheAlignedAlloca(sharedStructType);
    for (unsigned i = 0; i < n; i++) {
        Value * ptr = iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(i)});
        iBuilder->CreateStore(kernels[i]->getInstance(), ptr);
    }

    for (unsigned i = 0; i < threads; i++) {
        AllocaInst * threadState = iBuilder->CreateAlloca(threadStructType);
        Value * const sharedStatePtr = iBuilder->CreateGEP(threadState, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
        iBuilder->CreateStore(sharedStruct, sharedStatePtr);
        Value * const segmentOffsetPtr = iBuilder->CreateGEP(threadState, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
        iBuilder->CreateStore(iBuilder->getSize(i), segmentOffsetPtr);
        iBuilder->CreatePThreadCreateCall(threadIdPtr[i], nullVoidPtrVal, threadFunc, threadState);
    }

    AllocaInst * const status = iBuilder->CreateAlloca(int8PtrTy);
    for (unsigned i = 0; i < threads; i++) {
        Value * threadId = iBuilder->CreateLoad(threadIdPtr[i]);
        iBuilder->CreatePThreadJoinCall(threadId, status);
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateParallelPipeline
 ** ------------------------------------------------------------------------------------------------------------- */
void generateParallelPipeline(IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels) {

    Module * const m = iBuilder->getModule();
    IntegerType * const sizeTy = iBuilder->getSizeTy();
    PointerType * const voidPtrTy = iBuilder->getVoidPtrTy();
    PointerType * const int8PtrTy = iBuilder->getInt8PtrTy();
    ConstantInt * bufferSegments = ConstantInt::get(sizeTy, codegen::BufferSegments - 1);
    ConstantInt * segmentItems = ConstantInt::get(sizeTy, codegen::SegmentSize * iBuilder->getBitBlockWidth());
    Constant * const nullVoidPtrVal = ConstantPointerNull::getNullValue(voidPtrTy);

    for (auto & k : kernels) {
        k->createInstance();
    }

    const unsigned n = kernels.size();

    Type * const pthreadsTy = ArrayType::get(sizeTy, n);
    AllocaInst * const pthreads = iBuilder->CreateAlloca(pthreadsTy);
    Value * threadIdPtr[n];
    for (unsigned i = 0; i < n; i++) {
        threadIdPtr[i] = iBuilder->CreateGEP(pthreads, {iBuilder->getInt32(0), iBuilder->getInt32(i)});
    }

    Type * structTypes[n];
    for (unsigned i = 0; i < n; i++) {
        structTypes[i] = kernels[i]->getInstance()->getType();
    }
    Type * const sharedStructType = StructType::get(m->getContext(), ArrayRef<Type *>{structTypes, n});
    AllocaInst * sharedStruct = iBuilder->CreateAlloca(sharedStructType);
    for (unsigned i = 0; i < n; i++) {
        Value * ptr = iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(i)});
        iBuilder->CreateStore(kernels[i]->getInstance(), ptr);
    }
    for (auto & kernel : kernels) {
        kernel->releaseLogicalSegmentNo(kernel->getInstance(), iBuilder->getSize(0));
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
                report_fatal_error(kernel->getName() + " redefines stream set " + outputs[j].name);
            }
            producingKernel.emplace(buf, id);
        }
        // and any inputs to the consuming kernels list
        for (unsigned j = 0; j < inputs.size(); ++j) {
            const StreamSetBuffer * const buf = kernel->getStreamSetInputBuffer(j);
            auto f = consumingKernels.find(buf);
            if (f == consumingKernels.end()) {
                if (LLVM_UNLIKELY(producingKernel.count(buf) == 0)) {
                    report_fatal_error(kernel->getName() + " uses stream set " + inputs[j].name + " prior to its definition");
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
    for (unsigned id = 0; id < n; id++) {
        const auto & kernel = kernels[id];
        const auto & inputs = kernel->getStreamInputs();
        const auto & outputs = kernel->getStreamOutputs();

        Function * const threadFunc = makeThreadFunction("ppt:" + kernel->getName(), m);

         // Create the basic blocks for the thread function.
        BasicBlock * entryBlock = BasicBlock::Create(iBuilder->getContext(), "entry", threadFunc);
        BasicBlock * outputCheckBlock = BasicBlock::Create(iBuilder->getContext(), "outputCheck", threadFunc);
        BasicBlock * inputCheckBlock = BasicBlock::Create(iBuilder->getContext(), "inputCheck", threadFunc);
        BasicBlock * doSegmentBlock = BasicBlock::Create(iBuilder->getContext(), "doSegment", threadFunc);
        BasicBlock * exitThreadBlock = BasicBlock::Create(iBuilder->getContext(), "exitThread", threadFunc);

        iBuilder->SetInsertPoint(entryBlock);

        Value * sharedStruct = iBuilder->CreateBitCast(&threadFunc->getArgumentList().front(), sharedStructType->getPointerTo());

        Value * instancePtrs[n];
        for (unsigned k = 0; k < n; k++) {
            Value * const ptr = iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(k)});
            instancePtrs[k] = iBuilder->CreateLoad(ptr);
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
            Value * consumerSegNo = kernels[k]->acquireLogicalSegmentNo(instancePtrs[k]);
            assert (consumerSegNo->getType() == segNo->getType());
            Value * consumedSegNo = iBuilder->CreateAdd(consumerSegNo, bufferSegments);
            outputWaitCond = iBuilder->CreateAnd(outputWaitCond, iBuilder->CreateICmpULE(segNo, consumedSegNo));
        }
        kernelSet.clear();
        iBuilder->CreateCondBr(outputWaitCond, inputCheckBlock, outputCheckBlock);

        // Check whether the input buffers have enough data for this kernel to begin
        iBuilder->SetInsertPoint(inputCheckBlock);
        for (const StreamSetBuffer * buf : kernel->getStreamSetInputBuffers()) {
            kernelSet.insert(producingKernel[buf]);
        }
        const auto m = kernelSet.size();
        Value * producerSegNo[m];

        Value * inputWaitCond = iBuilder->getTrue();
        if (m) {
            unsigned j = 0;
            for (unsigned k : kernelSet) {
                producerSegNo[j] = kernels[k]->acquireLogicalSegmentNo(instancePtrs[k]);
                assert (producerSegNo[j]->getType() == segNo->getType());
                inputWaitCond = iBuilder->CreateAnd(inputWaitCond, iBuilder->CreateICmpULT(segNo, producerSegNo[j++]));
            }
        }
        iBuilder->CreateCondBr(inputWaitCond, doSegmentBlock, inputCheckBlock);

        // Process the segment
        iBuilder->SetInsertPoint(doSegmentBlock);

        Value * const nextSegNo = iBuilder->CreateAdd(segNo, iBuilder->getSize(1));

        Value * terminated = nullptr;
        if (m == 0) {
            // if this kernel has no input streams, the kernel itself must decide when it terminates.
            terminated = kernel->getTerminationSignal(instancePtrs[id]);
        } else {
            // ... otherwise the kernel terminates only when it exhausts all of its input streams
            terminated = iBuilder->getTrue();
            unsigned j = 0;
            for (unsigned k : kernelSet) {
                terminated = iBuilder->CreateAnd(terminated, kernels[k]->getTerminationSignal(instancePtrs[k]));
                terminated = iBuilder->CreateAnd(terminated, iBuilder->CreateICmpEQ(nextSegNo, producerSegNo[j++]));
            }
            kernelSet.clear();
        }

        std::vector<Value *> args = {instancePtrs[id], terminated};
        args.insert(args.end(), inputs.size(), iBuilder->CreateMul(segmentItems, segNo));
        args.insert(args.end(), outputs.size(), iBuilder->getSize(0));

        kernel->createDoSegmentCall(args);
        segNo->addIncoming(nextSegNo, doSegmentBlock);
        kernel->releaseLogicalSegmentNo(instancePtrs[id], nextSegNo);

        iBuilder->CreateCondBr(terminated, exitThreadBlock, outputCheckBlock);

        iBuilder->SetInsertPoint(exitThreadBlock);
        iBuilder->CreatePThreadExitCall(nullVoidPtrVal);
        iBuilder->CreateRetVoid();

        thread_functions[id] = threadFunc;
    }

    iBuilder->restoreIP(ip);

    for (unsigned i = 0; i < n; i++) {
        iBuilder->CreatePThreadCreateCall(threadIdPtr[i], nullVoidPtrVal, thread_functions[i], sharedStruct);
    }

    AllocaInst * const status = iBuilder->CreateAlloca(int8PtrTy);
    for (unsigned i = 0; i < n; i++) {
        Value * threadId = iBuilder->CreateLoad(threadIdPtr[i]);
        iBuilder->CreatePThreadJoinCall(threadId, status);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generatePipelineLoop
 ** ------------------------------------------------------------------------------------------------------------- */
void generatePipelineLoop(IDISA::IDISA_Builder * iBuilder, const std::vector<KernelBuilder *> & kernels) {

    for (auto & k : kernels) {
        k->createInstance();
    }
    BasicBlock * entryBlock = iBuilder->GetInsertBlock();
    Function * main = entryBlock->getParent();

    // Create the basic blocks for the loop.
    BasicBlock * pipelineLoop = BasicBlock::Create(iBuilder->getContext(), "pipelineLoop", main);
    BasicBlock * pipelineExit = BasicBlock::Create(iBuilder->getContext(), "pipelineExit", main);

    StreamSetBufferMap<Value *> producedPos;
    StreamSetBufferMap<std::pair<PHINode *, Value *>> consumedPos;

    iBuilder->CreateBr(pipelineLoop);
    iBuilder->SetInsertPoint(pipelineLoop);

    // Build up the initial phi nodes for each of the consumed (minimum processed) positions
    for (auto & kernel : kernels) {
        const auto & outputs = kernel->getStreamOutputs();
        for (unsigned i = 0; i < outputs.size(); ++i) {
            const StreamSetBuffer * const buf = kernel->getStreamSetOutputBuffer(i);
            if (LLVM_UNLIKELY(consumedPos.count(buf) != 0)) {
                report_fatal_error(kernel->getName() + " redefines stream set " + outputs[i].name);
            }
            PHINode * phi = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
            phi->addIncoming(iBuilder->getSize(0), entryBlock);
            consumedPos.emplace(buf, std::make_pair(phi, nullptr));
        }
    }

    Value * terminated = iBuilder->getFalse();
    for (auto & kernel : kernels) {
        Value * const instance = kernel->getInstance();
        const auto & inputs = kernel->getStreamInputs();
        const auto & outputs = kernel->getStreamOutputs();
        std::vector<Value *> args = {instance, terminated};
        for (unsigned i = 0; i < inputs.size(); ++i) {
            const auto f = producedPos.find(kernel->getStreamSetInputBuffer(i));
            if (LLVM_UNLIKELY(f == producedPos.end())) {
                report_fatal_error(kernel->getName() + " uses stream set " + inputs[i].name + " prior to its definition");
            }
            args.push_back(f->second);
        }
        for (unsigned i = 0; i < outputs.size(); ++i) {
            const auto f = consumedPos.find(kernel->getStreamSetOutputBuffer(i));
            assert (f != consumedPos.end());
            args.push_back(std::get<0>(f->second));
        }
        kernel->createDoSegmentCall(args);
        if (!kernel->hasNoTerminateAttribute()) {
            terminated = iBuilder->CreateOr(terminated, kernel->getTerminationSignal(instance));
        }
        for (unsigned i = 0; i < outputs.size(); i++) {
            Value * const produced = kernel->getProducedItemCount(instance, outputs[i].name, terminated);
            const StreamSetBuffer * const buf = kernel->getStreamSetOutputBuffer(i);
            assert (producedPos.count(buf) == 0);
            producedPos.emplace(buf, produced);
        }
        for (unsigned i = 0; i < inputs.size(); i++) {
            Value * const processed = kernel->getProcessedItemCount(instance, inputs[i].name);
            const StreamSetBuffer * const buf = kernel->getStreamSetInputBuffer(i);
            const auto f = consumedPos.find(buf);
            assert (f != consumedPos.end());
            Value *& consumed = std::get<1>(f->second);
            if (consumed) {
                consumed = iBuilder->CreateSelect(iBuilder->CreateICmpULT(processed, consumed), processed, consumed);
            } else {
                consumed = processed;
            }
        }
        Value * const segNo = kernel->acquireLogicalSegmentNo(instance);
        kernel->releaseLogicalSegmentNo(instance, iBuilder->CreateAdd(segNo, iBuilder->getSize(1)));
    }
    // update the consumed position phi nodes with the last min processed count of each input stream
    for (const auto entry : consumedPos) {
        PHINode * const phi = std::get<0>(entry.second);
        Value * const value = std::get<1>(entry.second);
        phi->addIncoming(value ? value : phi, pipelineLoop);
    }
    iBuilder->CreateCondBr(terminated, pipelineExit, pipelineLoop);
    iBuilder->SetInsertPoint(pipelineExit);
}
