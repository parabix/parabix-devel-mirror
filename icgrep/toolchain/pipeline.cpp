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

bool requiresCopyBack(const Kernel * k, const ProcessingRate & rate);

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

    BasicBlock * const segmentLoop = BasicBlock::Create(b->getContext(), "segmentLoop", threadFunc);
    b->CreateBr(segmentLoop);

    b->SetInsertPoint(segmentLoop);
    PHINode * const segNo = b->CreatePHI(b->getSizeTy(), 2, "segNo");
    segNo->addIncoming(segOffset, entryBlock);

    BasicBlock * const exitThreadBlock = BasicBlock::Create(b->getContext(), "exitThread", threadFunc);

    StreamSetBufferMap<Value *> producedItemCount;
    StreamSetBufferMap<Value *> consumedItemCount;
    StreamSetBufferMap<Kernel *> lastUsedKernel;

    Value * cycleCountStart = nullptr;
    Value * cycleCountEnd = nullptr;
    if (DebugOptionIsSet(codegen::EnableCycleCounter)) {
        cycleCountStart = b->CreateReadCycleCounter();
    }

    Value * terminated = nullptr;

    const bool serialize = codegen::DebugOptionIsSet(codegen::SerializeThreads);

    for (Kernel * const kernel : kernels) {
        const auto & inputs = kernel->getStreamInputs();
        for (unsigned i = 0; i < inputs.size(); ++i) {
            const StreamSetBuffer * const buffer = kernel->getStreamSetInputBuffer(i);
            auto f = lastUsedKernel.find(buffer);
            if (f == lastUsedKernel.end()) {
                lastUsedKernel.emplace(buffer, kernel);
            } else {
                f->second = kernel;
            }
        }
    }

    for (unsigned k = 0; k < n; ++k) {

        const auto & kernel = kernels[k];

        BasicBlock * const kernelWait = BasicBlock::Create(b->getContext(), kernel->getName() + "Wait", threadFunc);

        b->CreateBr(kernelWait);

        BasicBlock * const kernelCheck = BasicBlock::Create(b->getContext(), kernel->getName() + "Check", threadFunc);

        BasicBlock * const kernelBody = BasicBlock::Create(b->getContext(), kernel->getName() + "Do", threadFunc);

        BasicBlock * const kernelEnd = BasicBlock::Create(b->getContext(), kernel->getName() + "End", threadFunc);

        b->SetInsertPoint(kernelWait);

        b->setKernel(kernels[serialize ? (n - 1) : k]);
        Value * const processedSegmentCount = b->acquireLogicalSegmentNo();
        b->setKernel(kernel);

        assert (processedSegmentCount->getType() == segNo->getType());
        Value * const ready = b->CreateICmpEQ(segNo, processedSegmentCount);        
        b->CreateCondBr(ready, kernelCheck, kernelWait);

        b->SetInsertPoint(kernelCheck);
        b->CreateUnlikelyCondBr(b->getTerminationSignal(), kernelEnd, kernelBody);

        // Execute the kernel segment
        b->SetInsertPoint(kernelBody);
        const auto & inputs = kernel->getStreamInputs();
        Value * const isFinal = b->CreateOr(terminated ? terminated : b->getFalse(), b->getTerminationSignal());
        std::vector<Value *> args = {kernel->getInstance(), isFinal};
        for (unsigned i = 0; i < inputs.size(); ++i) {
            const StreamSetBuffer * const buffer = kernel->getStreamSetInputBuffer(i);
            const auto f = producedItemCount.find(buffer);
            assert (f != producedItemCount.end());
            Value * const produced = f->second;
            args.push_back(produced);
            handleInsufficientData(b, produced, isFinal, kernelEnd, kernel, inputs[i], buffer);
        }

        b->setKernel(kernel);
        b->createDoSegmentCall(args);
        b->CreateBr(kernelEnd);

        b->SetInsertPoint(kernelEnd);

        Value * const finished = b->getTerminationSignal();
        if (terminated) { // all kernels must terminate
            terminated = b->CreateAnd(terminated, finished);
        } else {
            terminated = finished;
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

        for (auto i = lastUsedKernel.begin(); i != lastUsedKernel.end(); i++) {
            if (i->second == kernel) {
                const StreamSetBuffer * const buffer = i->first;
                Kernel * const producerKernel = buffer->getProducer();
                const auto & binding = producerKernel->getStreamOutput(buffer);
                if (LLVM_UNLIKELY(binding.getRate().isDerived())) {
                    continue;
                }
                auto f = consumedItemCount.find(buffer);
                if (f != consumedItemCount.end()) {
                    const Kernel* tempKernel = b->getKernel();
                    b->setKernel(producerKernel);
                    b->setConsumedItemCount(binding.getName(), f->second);
                    b->setKernel(tempKernel);
                }
            }
        }


        if (DebugOptionIsSet(codegen::EnableCycleCounter)) {
            cycleCountEnd = b->CreateReadCycleCounter();
            Value * counterPtr = b->getCycleCountPtr();
            b->CreateStore(b->CreateAdd(b->CreateLoad(counterPtr), b->CreateSub(cycleCountEnd, cycleCountStart)), counterPtr);
            cycleCountStart = cycleCountEnd;
        }

        b->releaseLogicalSegmentNo(b->CreateAdd(segNo, b->getSize(1)));
    }

    exitThreadBlock->moveAfter(b->GetInsertBlock());

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
    assert (codegen::ThreadNum > 0);
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
    
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        for (const Kernel * kernel : kernels) {
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
 * @brief generatePipelineLoop
 ** ------------------------------------------------------------------------------------------------------------- */
void generatePipelineLoop(const std::unique_ptr<KernelBuilder> & b, const std::vector<Kernel *> & kernels) {

    BasicBlock * entryBlock = b->GetInsertBlock();
    Function * main = entryBlock->getParent();

    // Create the basic blocks for the loop.
    BasicBlock * const pipelineLoop = BasicBlock::Create(b->getContext(), "pipelineLoop", main);
    BasicBlock * const pipelineExit = BasicBlock::Create(b->getContext(), "pipelineExit", main);

    StreamSetBufferMap<Value *> producedItemCount;
    StreamSetBufferMap<Value *> consumedItemCount;
    StreamSetBufferMap<Kernel *> lastUsedKernel;

    b->CreateBr(pipelineLoop);
    b->SetInsertPoint(pipelineLoop);
    
    Value * cycleCountStart = nullptr;
    Value * cycleCountEnd = nullptr;
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        cycleCountStart = b->CreateReadCycleCounter();
    }
    Value * terminated = nullptr;

    for (Kernel * const kernel : kernels) {
        const auto & inputs = kernel->getStreamInputs();
        for (unsigned i = 0; i < inputs.size(); ++i) {
            const StreamSetBuffer * const buffer = kernel->getStreamSetInputBuffer(i);
            auto f = lastUsedKernel.find(buffer);
            if (f == lastUsedKernel.end()) {
                lastUsedKernel.emplace(buffer, kernel);
            } else {
                f->second = kernel;
            }
        }
    }

    for (Kernel * const kernel : kernels) {

        b->setKernel(kernel);

        BasicBlock * const entry = b->GetInsertBlock();
        BasicBlock * const kernelCode = BasicBlock::Create(b->getContext(), kernel->getName(), main);
        BasicBlock * const kernelExit = BasicBlock::Create(b->getContext(), kernel->getName() + "_exit", main);

        b->CreateUnlikelyCondBr(b->getTerminationSignal(), kernelExit, kernelCode);

        b->SetInsertPoint(kernelCode);
        const auto & inputs = kernel->getStreamInputs();
        const auto & outputs = kernel->getStreamOutputs();

        Value * const isFinal = terminated ? terminated : b->getFalse();

        std::vector<Value *> args = {kernel->getInstance(), isFinal};

        const auto name = kernel->getName();
        for (unsigned i = 0; i < inputs.size(); ++i) {
            const StreamSetBuffer * const buffer = kernel->getStreamSetInputBuffer(i);
            const auto f = producedItemCount.find(buffer);
            if (LLVM_UNLIKELY(f == producedItemCount.end())) {
                report_fatal_error(kernel->getName() + " uses stream set " + inputs[i].getName() + " prior to its definition");
            }
            Value * const produced = f->second;
            args.push_back(produced);
            handleInsufficientData(b, produced, isFinal, pipelineLoop, kernel, inputs[i], buffer);
        }

        applyOutputBufferExpansions(b, kernel);

        b->createDoSegmentCall(args);

        BasicBlock * const kernelFinished = b->GetInsertBlock();
        Value * const finished = b->getTerminationSignal();
        b->CreateBr(kernelExit);

        b->SetInsertPoint(kernelExit);
        PHINode * const finishedPhi = b->CreatePHI(b->getInt1Ty(), 2);
        finishedPhi->addIncoming(b->getTrue(), entry);
        finishedPhi->addIncoming(finished, kernelFinished);
        if (terminated) { // All kernels must agree that we've terminated.
            terminated = b->CreateAnd(terminated, finishedPhi);
        } else {
            terminated = finishedPhi;
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

        for (auto i = lastUsedKernel.begin(); i != lastUsedKernel.end(); i++) {
            if (i->second == kernel) {
                const StreamSetBuffer * const buffer = i->first;
                Kernel * const producerKernel = buffer->getProducer();
                const auto & binding = producerKernel->getStreamOutput(buffer);
                if (LLVM_UNLIKELY(binding.getRate().isDerived())) {
                    continue;
                }
                auto f = consumedItemCount.find(buffer);
                if (f != consumedItemCount.end()) {
                    const Kernel* tempKernel = b->getKernel();
                    b->setKernel(producerKernel);
                    b->setConsumedItemCount(binding.getName(), f->second);
                    b->setKernel(tempKernel);
                }
            }
        }

        if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
            cycleCountEnd = b->CreateReadCycleCounter();
            Value * counterPtr = b->getCycleCountPtr();
            b->CreateStore(b->CreateAdd(b->CreateLoad(counterPtr), b->CreateSub(cycleCountEnd, cycleCountStart)), counterPtr);
            cycleCountStart = cycleCountEnd;
        }
//        Value * const segNo = b->acquireLogicalSegmentNo();
//        Value * nextSegNo = b->CreateAdd(segNo, b->getSize(1));
//        b->releaseLogicalSegmentNo(nextSegNo);
    }

    b->CreateCondBr(terminated, pipelineExit, pipelineLoop);

    pipelineExit->moveAfter(b->GetInsertBlock());

    b->SetInsertPoint(pipelineExit);

    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
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
    const auto consumedRate = consumer->getUpperBound(input.getRate()) * consumer->getStride();
    if (consumedRate > 0) {
        auto producedRate = producer->getLowerBound(output.getRate()) * producer->getStride();
        if (LLVM_UNLIKELY(input.hasLookahead())) {
            producedRate -= input.getLookahead();
        }
        if (LLVM_UNLIKELY(producedRate < consumedRate)) {
            const auto name = input.getName();
            BasicBlock * const sufficient = BasicBlock::Create(b->getContext(), name + "IsSufficient", b->GetInsertBlock()->getParent());
            Value * const processed = b->getProcessedItemCount(name);

            if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableAsserts))) {
                b->CreateAssert(b->CreateICmpULE(processed, produced), input.getName() + ": processed cannot exceed produced");
            }
            Value * const unread = b->CreateSub(produced, processed);
            Constant * const amount = ConstantInt::get(unread->getType(), ceiling(consumedRate));
            Value * const cond = b->CreateOr(b->CreateICmpUGE(unread, amount), final);
            b->CreateLikelyCondBr(cond, sufficient, insufficient);
            b->SetInsertPoint(sufficient);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresCopyBack
 ** ------------------------------------------------------------------------------------------------------------- */
bool requiresCopyBack(const Kernel * k, const ProcessingRate & rate) {
    if (rate.isBounded() || rate.isUnknown()) {
        return true;
    } else if (rate.isRelative()) {
        return requiresCopyBack(k, k->getBinding(rate.getReference()).getRate());
    }
    return false;
}
