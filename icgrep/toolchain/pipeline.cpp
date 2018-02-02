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
#include <boost/graph/adjacency_list.hpp>
#include <kernels/kernel_builder.h>

#include <llvm/Support/raw_ostream.h>

using namespace kernel;
using namespace parabix;
using namespace llvm;
using namespace boost;
using namespace boost::container;

using Port = Kernel::Port;

Function * makeThreadFunction(const std::unique_ptr<kernel::KernelBuilder> & b, const std::string & name) {
    Function * const f = Function::Create(FunctionType::get(b->getVoidTy(), {b->getVoidPtrTy()}, false), Function::InternalLinkage, name, b->getModule());
    f->setCallingConv(CallingConv::C);
    f->arg_begin()->setName("state");
    return f;
}

struct PipelineGenerator {

    template <typename Value>
    using StreamSetBufferMap = flat_map<const StreamSetBuffer *, Value>;

    using RateValue = ProcessingRate::RateValue;

    struct Channel {
        Channel() = default;
        Channel(const RateValue & rate, const StreamSetBuffer * const buffer)
        : rate(rate), buffer(buffer) { }

        RateValue               rate;
        const StreamSetBuffer * buffer;
    };

    using Graph = adjacency_list<vecS, vecS, bidirectionalS, const Kernel *, Channel, vecS>;

    using Map = flat_map<const Kernel *, Graph::vertex_descriptor>;

    void initialize(const std::vector<Kernel *> & kernels);

    Value * executeKernel(const std::unique_ptr<KernelBuilder> & b, const Kernel * const kernel, PHINode * const segNo, Value * const finished);

    void applyOutputBufferExpansions(const std::unique_ptr<KernelBuilder> & b, const Kernel * kernel);

    void updateProducedAndConsumedCounts(const std::unique_ptr<KernelBuilder> & b, const Kernel * kernel);

private:

    Graph   G;
    Map     M;

    StreamSetBufferMap<Value *>         producedItemCount;
    StreamSetBufferMap<Value *>         consumedItemCount;
    StreamSetBufferMap<const Kernel *>  lastConsumer;
};

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

    PipelineGenerator G;

    BasicBlock * const segmentLoop = b->CreateBasicBlock("segmentLoop");
    b->CreateBr(segmentLoop);

    b->SetInsertPoint(segmentLoop);
    G.initialize(kernels);
    PHINode * const segNo = b->CreatePHI(b->getSizeTy(), 2, "segNo");
    segNo->addIncoming(segOffset, entryBlock);
    Value * finished = nullptr;

    Value * cycleCountStart = nullptr;
    Value * cycleCountEnd = nullptr;
    if (DebugOptionIsSet(codegen::EnableCycleCounter)) {
        cycleCountStart = b->CreateReadCycleCounter();
    }

    const bool serialize = codegen::DebugOptionIsSet(codegen::SerializeThreads);

    for (unsigned k = 0; k < n; ++k) {

        const Kernel * const kernel = kernels[k];

        BasicBlock * const kernelWait = b->CreateBasicBlock(kernel->getName() + "Wait");
        b->CreateBr(kernelWait);

        b->SetInsertPoint(kernelWait);
        b->setKernel(kernels[serialize ? (n - 1) : k]);
        Value * const processedSegmentCount = b->acquireLogicalSegmentNo();
        b->setKernel(kernel);
        assert (processedSegmentCount->getType() == segNo->getType());
        Value * const ready = b->CreateICmpEQ(segNo, processedSegmentCount);

        BasicBlock * const kernelCheck = b->CreateBasicBlock(kernel->getName() + "Check");
        b->CreateCondBr(ready, kernelCheck, kernelWait);

        b->SetInsertPoint(kernelCheck);

        finished = G.executeKernel(b, kernel, segNo, finished);

        if (DebugOptionIsSet(codegen::EnableCycleCounter)) {
            cycleCountEnd = b->CreateReadCycleCounter();
            Value * counterPtr = b->getCycleCountPtr();
            b->CreateStore(b->CreateAdd(b->CreateLoad(counterPtr), b->CreateSub(cycleCountEnd, cycleCountStart)), counterPtr);
            cycleCountStart = cycleCountEnd;
        }

        b->releaseLogicalSegmentNo(b->CreateAdd(segNo, b->getSize(1)));
    }

    segNo->addIncoming(b->CreateAdd(segNo, b->getSize(codegen::ThreadNum)), b->GetInsertBlock());

    BasicBlock * const segmentExit = b->CreateBasicBlock("segmentExit");
    b->CreateUnlikelyCondBr(finished, segmentExit, segmentLoop);

    b->SetInsertPoint(segmentExit);

    // only call pthread_exit() within spawned threads; otherwise it'll be equivalent to calling exit() within the process
    BasicBlock * const exitThread = b->CreateBasicBlock("ExitThread");
    BasicBlock * const exitFunction = b->CreateBasicBlock("ExitProcessFunction");

    b->CreateCondBr(b->CreateIsNull(segOffset), exitFunction, exitThread);
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

    // Create the basic blocks for the loop.
    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const pipelineLoop = b->CreateBasicBlock("pipelineLoop");
    BasicBlock * const pipelineExit = b->CreateBasicBlock("pipelineExit");

    PipelineGenerator G;

    b->CreateBr(pipelineLoop);

    b->SetInsertPoint(pipelineLoop);
    G.initialize(kernels);
    PHINode * const segNo = b->CreatePHI(b->getSizeTy(), 2, "segNo");
    segNo->addIncoming(b->getSize(0), entryBlock);
    Value * finished = nullptr;

    Value * cycleCountStart = nullptr;
    Value * cycleCountEnd = nullptr;
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        cycleCountStart = b->CreateReadCycleCounter();
    }

    for (Kernel * const kernel : kernels) {

        b->setKernel(kernel);

        finished = G.executeKernel(b, kernel, segNo, finished);

        if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
            cycleCountEnd = b->CreateReadCycleCounter();
            Value * counterPtr = b->getCycleCountPtr();
            b->CreateStore(b->CreateAdd(b->CreateLoad(counterPtr), b->CreateSub(cycleCountEnd, cycleCountStart)), counterPtr);
            cycleCountStart = cycleCountEnd;
        }
    }

    segNo->addIncoming(b->CreateAdd(segNo, b->getSize(1)), b->GetInsertBlock());
    b->CreateCondBr(finished, pipelineExit, pipelineLoop);

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
 * @brief initialize
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineGenerator::initialize(const std::vector<Kernel *> & kernels) {

    // Our goal when building G is *not* to model the dataflow of our program but instead to
    // detetermine the minimum number of sufficient data tests needed to ensure each kernel has
    // enough data to progress.

    // For example, suppose we have kernels A, B and C, and that B has a fixed input and fixed
    // output rate. C also has a fixed input rate but A does *not* have a fixed output rate.
    // C must test whether it has enough input from B as B is not guaranteed to have enough
    // input from A. Moreover if C is depedent on B, C could be skipped entirely.

    // Note: we cannot simply test the output of A for both B and C. In a our data-parallel
    // pipeline A's state may change by the time we process C.

    for (const Kernel * const consumer : kernels) {
        const auto v = add_vertex(consumer, G);
        M.emplace(consumer, v);
        const auto & inputs = consumer->getStreamInputs();
        for (unsigned i = 0; i < inputs.size(); ++i) {

            const auto buffer = consumer->getStreamSetInputBuffer(i);
            const Kernel * const producer = buffer->getProducer();
            const Binding & output = producer->getStreamOutput(buffer);
            if (output.getRate().isRelative()) continue;

            const Binding & input = inputs[i];
            auto ub_in = consumer->getUpperBound(input.getRate()) * consumer->getStride();
            if (input.hasLookahead()) {
                ub_in += input.getLookahead();
            }

            const auto lb_out = producer->getLowerBound(output.getRate()) * producer->getStride();

            const auto rate = lb_out / ub_in;
            const auto f = M.find(producer); assert (f != M.end());
            const auto u = f->second;
            // If we have multiple inputs from the same kernel, we only need to consider the "slowest" one
            bool slowest = true;
            if (lb_out > 0) {
                for (const auto e : make_iterator_range(in_edges(v, G))) {
                    if (source(e, G) == u) {
                        Channel & p = G[e];
                        slowest = false;
                        if (rate < p.rate) {
                            p.rate = rate;
                            p.buffer = buffer;
                        }
                        break;
                    }
                }
            }
            if (slowest) {
                add_edge(u, v, Channel{rate, buffer}, G);
            }
        }
    }

    // Take a transitive closure of G but whenever we attempt to insert an edge into the closure
    // that already exists, check instead whether the rate of our proposed edge is <= the existing
    // edge's rate. If so, the data availability is transitively guaranteed.
    for (const auto u : make_iterator_range(vertices(G))) {
        for (auto ei : make_iterator_range(in_edges(u, G))) {
            const auto v = source(ei, G);
            const Channel & pu = G[ei];           
            for (auto ej : make_iterator_range(out_edges(u, G))) {                
                const auto w = target(ej, G);
                const auto ratio = RateValue(G[u]->getStride(), G[w]->getStride());
                const auto rate = pu.rate * ratio;
                bool insert = true;
                for (auto ek : make_iterator_range(in_edges(w, G))) {
                    if (source(ek, G) == v) {
                        Channel & pw = G[ek];
                        if (rate <= pw.rate && pw.rate > 0) {
                            pw.buffer = nullptr;
                        }
                        insert = false;
                        break;
                    }
                }
                if (insert) {
                    add_edge(v, w, Channel{rate, nullptr}, G);
                }
            }
        }
    }

    // remove any closure edges from G
    remove_edge_if([&](const Graph::edge_descriptor e) { return G[e].buffer == nullptr; }, G);

    // If a kernel has no 'necessary to check' inputs then we can remove every output with a rate >= 1 from G
    for (const auto u : make_iterator_range(vertices(G))) {
        if (in_degree(u, G) == 0) {
            remove_out_edge_if(u, [&](const Graph::edge_descriptor e) { return G[e].rate >= RateValue{1, 1}; }, G);
        }
    }

    // iterate through each kernel in order and determine which kernel last used a particular buffer
    for (Kernel * const kernel : kernels) {
        const auto & inputs = kernel->getStreamInputs();
        for (unsigned i = 0; i < inputs.size(); ++i) {
            lastConsumer[kernel->getStreamSetInputBuffer(i)] = kernel;
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief executeKernel
 ** ------------------------------------------------------------------------------------------------------------- */
Value *PipelineGenerator::executeKernel(const std::unique_ptr<KernelBuilder> & b, const Kernel * const kernel, PHINode * const segNo, Value * const finished) {

    const auto & inputs = kernel->getStreamInputs();

    std::vector<Value *> args(2 + inputs.size());

    const auto f = M.find(kernel); assert (f != M.end());
    const auto u = f->second;

    BasicBlock * const kernelEntry = b->GetInsertBlock();
    BasicBlock * const kernelCode = b->CreateBasicBlock(kernel->getName());
    BasicBlock * const kernelExit = b->CreateBasicBlock(kernel->getName() + "_exit");

    b->CreateUnlikelyCondBr(b->getTerminationSignal(), kernelExit, kernelCode);

    b->SetInsertPoint(kernelExit);
    PHINode * const terminated = b->CreatePHI(b->getInt1Ty(), 2);
    // Since our initial "isFinal" state is equal to what the first kernel's termination signal state
    terminated->addIncoming(finished ? finished : b->getTrue(), kernelEntry);
    Value * isFinal = finished ? finished : b->getFalse();

    b->SetInsertPoint(kernelCode);
    for (unsigned i = 0; i < inputs.size(); ++i) {

        const Binding & input = inputs[i];

        const StreamSetBuffer * const buffer = kernel->getStreamSetInputBuffer(i);

        const auto name = input.getName();

        const auto p = producedItemCount.find(buffer);
        if (LLVM_UNLIKELY(p == producedItemCount.end())) {
            report_fatal_error(kernel->getName() + " uses stream set " + name + " prior to its definition");
        }
        Value * const produced = p->second;
        const auto ub = kernel->getUpperBound(input.getRate()); assert (ub > 0);
        const auto strideLength = ceiling(ub * kernel->getStride()) ;
        Constant * const segmentLength = b->getSize(strideLength * codegen::SegmentSize);

        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            b->CreateAssert(b->CreateICmpULE(segmentLength, b->getCapacity(name)),
                            kernel->getName() + ": " + name + " upper bound of segment length exceeds buffer capacity");
        }

        Value * limit = nullptr;
        if (input.getRate().isFixed()) {
            // if the input is deferred, simply adding length to the processed item count may result in setting a limit
            // that is too low for. instead just calculate the limit of all fixed rates from the segment no.
            limit = b->CreateMul(b->CreateAdd(segNo, b->getSize(1)), segmentLength);
        } else {
            Value * const processed = b->getProcessedItemCount(name);
            limit = b->CreateAdd(processed, segmentLength);
        }

        // TODO: currently, if we produce the exact amount as our limit states, we will have to process one additional segment
        // before we can consider this kernel finished. We ought to be able to avoid doing in some cases but need to prove its
        // always safe to do so.

        Value * const consumingAll = b->CreateICmpULT(produced, limit);
        args[i + 2] = b->CreateSelect(consumingAll, produced, limit);
        isFinal = b->CreateAnd(isFinal, consumingAll);

        // Check for available input (if it's both computable and not guaranteed to be sufficient by the processing rates)
        for (auto e : make_iterator_range(in_edges(u, G))) {
            const auto p = G[e];
            if (p.buffer == buffer) {
                BasicBlock * const sufficient = b->CreateBasicBlock(name + "_hasSufficientData");

                Constant * const sl = b->getSize(strideLength);

                Value * remaining = nullptr;
                if (input.getRate().isFixed()) {
                    remaining = b->CreateMul(segNo, sl);
                } else {
                    remaining = b->getProcessedItemCount(name);
                }
                remaining = b->CreateSub(produced, remaining);

                Value * const hasSufficientData = b->CreateOr(b->CreateICmpUGE(remaining, sl), isFinal);
                terminated->addIncoming(b->getFalse(), b->GetInsertBlock());
                b->CreateLikelyCondBr(hasSufficientData, sufficient, kernelExit);
                b->SetInsertPoint(sufficient);
            }
        }
    }

    applyOutputBufferExpansions(b, kernel);

    args[0] = kernel->getInstance();
    args[1] = isFinal;

    b->createDoSegmentCall(args);

    if (inputs.empty() || kernel->canTerminateEarly()) {
        isFinal = b->CreateOr(isFinal, b->getTerminationSignal());
    }
    b->setTerminationSignal(isFinal);
//    b->CallPrintInt(kernel->getName() + "_finished", isFinal);
    BasicBlock * const kernelFinished = b->GetInsertBlock();
    kernelExit->moveAfter(kernelFinished);
    b->CreateBr(kernelExit);

    b->SetInsertPoint(kernelExit);
    terminated->addIncoming(isFinal, kernelFinished);

    updateProducedAndConsumedCounts(b, kernel);

    return terminated;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief applyOutputBufferExpansions
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineGenerator::applyOutputBufferExpansions(const std::unique_ptr<KernelBuilder> & b, const Kernel * k) {
    const auto & outputs = k->getStreamSetOutputBuffers();
    for (unsigned i = 0; i < outputs.size(); i++) {
        if (isa<DynamicBuffer>(outputs[i])) {
            const auto baseSize = ceiling(k->getUpperBound(k->getStreamOutput(i).getRate()) * k->getStride() * codegen::SegmentSize);
            if (LLVM_LIKELY(baseSize > 0)) {

                const auto & name = k->getStreamOutput(i).getName();

                BasicBlock * const doExpand = b->CreateBasicBlock(name + "Expand");
                BasicBlock * const nextBlock = b->GetInsertBlock()->getNextNode();
                doExpand->moveAfter(b->GetInsertBlock());
                BasicBlock * const bufferReady = b->CreateBasicBlock(name + "Ready");
                bufferReady->moveAfter(doExpand);
                if (nextBlock) nextBlock->moveAfter(bufferReady);

                Value * const produced = b->getProducedItemCount(name);
                Value * const consumed = b->getConsumedItemCount(name);
                Value * const required = b->CreateAdd(b->CreateSub(produced, consumed), b->getSize(2 * baseSize));

                b->CreateCondBr(b->CreateICmpUGT(required, b->getBufferedSize(name)), doExpand, bufferReady);
                b->SetInsertPoint(doExpand);

                b->doubleCapacity(name);
                // Ensure that capacity is sufficient by successive doubling, if necessary.
                b->CreateCondBr(b->CreateICmpUGT(required, b->getBufferedSize(name)), doExpand, bufferReady);

                b->SetInsertPoint(bufferReady);

            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateProducedAndConsumedCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineGenerator::updateProducedAndConsumedCounts(const std::unique_ptr<KernelBuilder> & b, const Kernel * kernel) {

    const auto & inputs = kernel->getStreamInputs();
    for (unsigned i = 0; i < inputs.size(); ++i) {
        Value * const processed = b->getProcessedItemCount(inputs[i].getName());

        const StreamSetBuffer * const buffer = kernel->getStreamSetInputBuffer(i);
        auto f = consumedItemCount.find(buffer);
        Value * consumed = processed;
        if (f == consumedItemCount.end()) {
            consumedItemCount.emplace(buffer, consumed);
        } else {
            consumed = b->CreateUMin(consumed, f->second);
            f->second = consumed;
        }

        // If this kernel is the last consumer of a input buffer, update the consumed count for that buffer.
        const auto c = lastConsumer.find(buffer);
        assert (c != lastConsumer.end());
        if (c->second == kernel) {
            Kernel * const producer = buffer->getProducer();
            const auto & output = producer->getStreamOutput(buffer);
            if (output.getRate().isRelative()) continue;
            b->setKernel(producer);

            b->setConsumedItemCount(output.getName(), consumed);
            b->setKernel(kernel);
        }
    }

    const auto & outputs = kernel->getStreamOutputs();
    for (unsigned i = 0; i < outputs.size(); ++i) {
        Value * const produced = b->getProducedItemCount(outputs[i].getName());
        const StreamSetBuffer * const buf = kernel->getStreamSetOutputBuffer(i);
        assert (producedItemCount.count(buf) == 0);
        producedItemCount.emplace(buf, produced);
    }

}


