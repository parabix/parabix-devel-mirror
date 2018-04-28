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
#include <boost/range/adaptor/reversed.hpp>
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>

using namespace kernel;
using namespace parabix;
using namespace llvm;
using namespace boost;
using namespace boost::container;

#define DISABLE_COPY_TO_OVERFLOW

using Port = Kernel::Port;

Function * makeThreadFunction(const std::unique_ptr<kernel::KernelBuilder> & b, const std::string & name) {
    Function * const f = Function::Create(FunctionType::get(b->getVoidTy(), {b->getVoidPtrTy()}, false), Function::InternalLinkage, name, b->getModule());
    f->setCallingConv(CallingConv::C);
    f->arg_begin()->setName("state");
    return f;
}

class PipelineGenerator {
public:

    template <typename Value>
    using StreamSetBufferMap = flat_map<const StreamSetBuffer *, Value>;

    using RateValue = ProcessingRate::RateValue;

    PipelineGenerator(const std::vector<Kernel *> & kernels)
    : kernels(kernels)
    , terminated(nullptr)
    , noMore(nullptr)
    , deadLockCounter(nullptr)
    , anyProgress(nullptr)
    , madeProgress(nullptr) {

    }

    struct Channel {
        Channel() = default;
        Channel(const RateValue & ratio, const StreamSetBuffer * const buffer = nullptr, const unsigned operand = 0)
        : ratio(ratio), buffer(buffer), operand(operand) { }

        RateValue               ratio;
        const StreamSetBuffer * buffer;
        unsigned                operand;
    };

    using ChannelGraph = adjacency_list<vecS, vecS, bidirectionalS, const Kernel *, Channel>;

    using DependencyGraph = adjacency_list<hash_setS, vecS, bidirectionalS, Value *>;

    using KernelMap = flat_map<const Kernel *, unsigned>;

    void initialize(const std::unique_ptr<KernelBuilder> & b, BasicBlock * const entryBlock);

    void execute(const std::unique_ptr<KernelBuilder> & b, const unsigned index);

    Value * finalize(const std::unique_ptr<KernelBuilder> & b);

protected:

    ChannelGraph makeInputGraph() const;

    ChannelGraph makeOutputGraph() const;

    DependencyGraph makeDependencyGraph() const;

    template<class VertexList>
    ChannelGraph pruneGraph(ChannelGraph && G, VertexList && V) const;

    void checkIfAllInputKernelsAreTerminated(const std::unique_ptr<KernelBuilder> & b, const unsigned index);

    void checkAvailableInputData(const std::unique_ptr<KernelBuilder> & b, const unsigned index);

    void checkAvailableOutputSpace(const std::unique_ptr<KernelBuilder> & b, const unsigned index);

    Value * getStrideLength(const std::unique_ptr<KernelBuilder> & b, const Kernel * const kernel, const Binding & binding);

    Value * callKernel(const std::unique_ptr<KernelBuilder> & b, const unsigned index);

    void applyOutputBufferExpansions(const std::unique_ptr<KernelBuilder> & b, const unsigned index);

    Value * getFullyProcessedItemCount(const std::unique_ptr<KernelBuilder> & b, const Binding & binding, Value * const final) const;

    void printGraph(const ChannelGraph & G) const;

    void printGraph(const DependencyGraph & G) const;

private:

    const std::vector<Kernel *> &       kernels;
    PHINode *                           terminated;

    Value *                             noMore;

    DependencyGraph                     dependencyGraph;
    ChannelGraph                        inputGraph;
    ChannelGraph                        outputGraph;

    BasicBlock *                        kernelFinished;

    PHINode *                           deadLockCounter;
    Value *                             anyProgress;
    PHINode *                           madeProgress;

    StreamSetBufferMap<Value *>         producedItemCount;
    StreamSetBufferMap<Value *>         consumedItemCount;
    StreamSetBufferMap<const Kernel *>  lastConsumer;
    flat_set<const StreamSetBuffer *>   isConsumedAtNonFixedRate;
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

    PipelineGenerator G(kernels);

    BasicBlock * const segmentLoop = b->CreateBasicBlock("segmentLoop");
    b->CreateBr(segmentLoop);

    b->SetInsertPoint(segmentLoop);    
    PHINode * const segNo = b->CreatePHI(b->getSizeTy(), 2, "segNo");
    segNo->addIncoming(segOffset, entryBlock);
    G.initialize(b, entryBlock);

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
        assert (processedSegmentCount->getType() == segNo->getType());
        Value * const ready = b->CreateICmpEQ(segNo, processedSegmentCount);

        BasicBlock * const kernelCheck = b->CreateBasicBlock(kernel->getName() + "Check");
        b->CreateCondBr(ready, kernelCheck, kernelWait);

        b->SetInsertPoint(kernelCheck);

        G.execute(b, k);

        b->releaseLogicalSegmentNo(b->CreateAdd(segNo, b->getSize(1)));

        if (DebugOptionIsSet(codegen::EnableCycleCounter)) {
            cycleCountEnd = b->CreateReadCycleCounter();
            Value * counterPtr = b->getCycleCountPtr();
            b->CreateStore(b->CreateAdd(b->CreateLoad(counterPtr), b->CreateSub(cycleCountEnd, cycleCountStart)), counterPtr);
            cycleCountStart = cycleCountEnd;
        }

    }

    segNo->addIncoming(b->CreateAdd(segNo, b->getSize(codegen::ThreadNum)), b->GetInsertBlock());
    Value * const finished = G.finalize(b);
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

    PipelineGenerator G(kernels);

    b->CreateBr(pipelineLoop);

    b->SetInsertPoint(pipelineLoop);    
    PHINode * const segNo = b->CreatePHI(b->getSizeTy(), 2, "segNo");
    segNo->addIncoming(b->getSize(0), entryBlock);
    G.initialize(b, entryBlock);

    Value * const nextSegNo = b->CreateAdd(segNo, b->getSize(1));

    Value * cycleCountStart = nullptr;
    Value * cycleCountEnd = nullptr;
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        cycleCountStart = b->CreateReadCycleCounter();
    }

    for (unsigned i = 0; i < kernels.size(); ++i) {

        G.execute(b, i);

        b->releaseLogicalSegmentNo(nextSegNo);

        if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
            cycleCountEnd = b->CreateReadCycleCounter();
            Value * counterPtr = b->getCycleCountPtr();
            b->CreateStore(b->CreateAdd(b->CreateLoad(counterPtr), b->CreateSub(cycleCountEnd, cycleCountStart)), counterPtr);
            cycleCountStart = cycleCountEnd;
        }
    }

    segNo->addIncoming(nextSegNo, b->GetInsertBlock());
    BasicBlock * const pipelineExit = b->CreateBasicBlock("pipelineExit");
    Value * const finished = G.finalize(b);
    b->CreateCondBr(finished, pipelineExit, pipelineLoop);

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
void PipelineGenerator::initialize(const std::unique_ptr<KernelBuilder> & b, BasicBlock * const entryBlock) {

    dependencyGraph = makeDependencyGraph();
    inputGraph = makeInputGraph();
    outputGraph = makeOutputGraph();

    for (Kernel * const kernel : kernels) {
        const auto & inputs = kernel->getStreamInputs();
        for (unsigned i = 0; i < inputs.size(); ++i) {
            const StreamSetBuffer * const buffer = kernel->getStreamSetInputBuffer(i);
            if (kernel->requiresCopyBack(inputs[i]) && !buffer->isUnbounded()) {
                if (LLVM_LIKELY(buffer->supportsCopyBack())) {
                    isConsumedAtNonFixedRate.insert(buffer);
                } else {
    //                std::string tmp;
    //                raw_string_ostream out(tmp);
    //                out << kernel->getName() << " : " << name << " must have an overflow";
    //                report_fatal_error(out.str());
                }
            }
        }
        // if this kernel consumes this buffer, update the last consumer
        for (const StreamSetBuffer * const buffer : kernel->getStreamSetInputBuffers()) {
            auto f = lastConsumer.find(buffer);
            assert (f != lastConsumer.end());
            f->second = kernel;
        }
        // incase some output is never consumed, make the kernel that produced it the initial "last consumer"
        for (const StreamSetBuffer * const buffer : kernel->getStreamSetOutputBuffers()) {
            assert (buffer->getProducer() == kernel);
            assert (lastConsumer.count(buffer) == 0);
            lastConsumer.emplace(buffer, kernel);
        }
    }

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        deadLockCounter = b->CreatePHI(b->getSizeTy(), 2, "deadLockCounter");
        deadLockCounter->addIncoming(b->getSize(0), entryBlock);
        anyProgress = b->getFalse();
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeTerminationGraph
 *
 * The input graph models whether a kernel could *consume* more data than may be produced by a preceeding kernel.
 ** ------------------------------------------------------------------------------------------------------------- */
PipelineGenerator::DependencyGraph PipelineGenerator::makeDependencyGraph() const {
    const auto n = kernels.size();
    DependencyGraph G(n);
    KernelMap M;
    // construct a kernel dependency graph
    for (unsigned v = 0; v < n; ++v) {
        const Kernel * const kernel = kernels[v];        
        for (const StreamSetBuffer * buf : kernel->getStreamSetInputBuffers()) {
            const auto f = M.find(buf->getProducer()); assert (f != M.end());
            add_edge(f->second, v, G);
        }
        M.emplace(kernel, v);
    }
    // generate a transitive closure
    for (unsigned u = 0; u < n; ++u) {
        for (auto e : make_iterator_range(in_edges(u, G))) {
            const auto s = source(e, G);
            for (auto f : make_iterator_range(out_edges(u, G))) {
                add_edge(s, target(f, G), G);
            }
        }
    }
    // then take the transitive reduction
    std::vector<unsigned> sources;
    for (unsigned u = n; u-- > 0; ) {
        if (in_degree(u, G) > 0 && out_degree(u, G) > 0) {
            for (auto e : make_iterator_range(in_edges(u, G))) {
                sources.push_back(source(e, G));
            }
            std::sort(sources.begin(), sources.end());
            for (auto e : make_iterator_range(out_edges(u, G))) {
                remove_in_edge_if(target(e, G), [&G, &sources](const DependencyGraph::edge_descriptor f) {
                    return std::binary_search(sources.begin(), sources.end(), source(f, G));
                }, G);
            }
            sources.clear();
        }
    }
    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeInputGraph
 *
 * The input graph models whether a kernel could *consume* more data than may be produced by a preceeding kernel.
 ** ------------------------------------------------------------------------------------------------------------- */
PipelineGenerator::ChannelGraph PipelineGenerator::makeInputGraph() const {
    const auto n = kernels.size();
    ChannelGraph G(n);
    KernelMap M;
    for (unsigned v = 0; v < n; ++v) {

        const Kernel * const consumer = kernels[v];
        G[v] = consumer;
        M.emplace(consumer, v);

        const auto & inputs = consumer->getStreamInputs();
        for (unsigned i = 0; i < inputs.size(); ++i) {

            const Binding & input = inputs[i];
            auto ub_in = consumer->getUpperBound(input.getRate()) * consumer->getStride() * codegen::SegmentSize; assert (ub_in > 0);
            if (input.hasLookahead()) {
                ub_in += input.getLookahead();
            }

            const auto buffer = consumer->getStreamSetInputBuffer(i);
            const Kernel * const producer = buffer->getProducer();
            const Binding & output = producer->getStreamOutput(buffer);
            const auto lb_out = producer->getLowerBound(output.getRate()) * producer->getStride() * codegen::SegmentSize;

            const auto min_oi_ratio = lb_out / ub_in;
            const auto f = M.find(producer); assert (f != M.end());
            const auto u = f->second;
            // If we have multiple inputs from the same kernel, we only need to consider the "slowest" one
            bool slowest = true;
            for (const auto e : make_iterator_range(in_edges(v, G))) {
                if (source(e, G) == u) {
                    const Channel & p = G[e];
                    if (min_oi_ratio > p.ratio) {
                        slowest = false;
                    } else if (min_oi_ratio < p.ratio) {
                        clear_in_edges(v, G);
                    }
                    break;
                }
            }
            if (slowest) {
                add_edge(u, v, Channel{min_oi_ratio, buffer, i}, G);
            }
        }
    }

    return pruneGraph(std::move(G), make_iterator_range(vertices(G)));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeOutputGraph
 *
 * The output graph models whether a kernel could *produce* more data than may be consumed by a subsequent kernel.
 ** ------------------------------------------------------------------------------------------------------------- */
PipelineGenerator::ChannelGraph PipelineGenerator::makeOutputGraph() const {
    const auto n = kernels.size();
    ChannelGraph G(n);
    KernelMap M;
    for (unsigned v = 0; v < n; ++v) {
        const Kernel * const consumer = kernels[v];
        G[v] = consumer;
        M.emplace(consumer, v);

        const auto & inputs = consumer->getStreamInputs();
        for (unsigned i = 0; i < inputs.size(); ++i) {
            const auto buffer = consumer->getStreamSetInputBuffer(i);
            if (isa<SourceBuffer>(buffer)) continue;
            const Kernel * const producer = buffer->getProducer();
            assert (consumer != producer);
            const Binding & output = producer->getStreamOutput(buffer);
            auto ub_out = producer->getUpperBound(output.getRate()) * producer->getStride() * codegen::SegmentSize;
            if (ub_out > 0) { // unknown output rates are handled by reallocating their buffers
                const Binding & input = inputs[i];
                if (input.hasLookahead()) {
                    const auto la = input.getLookahead();
                    if (LLVM_UNLIKELY(ub_out <= la)) {
                        llvm::report_fatal_error("lookahead exceeds segment size");
                    }
                    ub_out += la;
                }
                const auto lb_in = consumer->getLowerBound(input.getRate()) * consumer->getStride() * codegen::SegmentSize;
                const auto min_io_ratio = lb_in / ub_out;
                const auto f = M.find(producer); assert (f != M.end());
                const auto u = f->second;
                assert (v != u);
                assert (G[u] == producer);
                // If we have multiple inputs from the same kernel, we only need to consider the "fastest" one
                bool fastest = true;
                for (const auto e : make_iterator_range(in_edges(v, G))) {
                    if (source(e, G) == u) {
                        const Channel & p = G[e];
                        if (min_io_ratio > p.ratio) {
                            fastest = false;
                        } else if (min_io_ratio < p.ratio) {
                            clear_in_edges(v, G);
                        }
                        break;
                    }
                }
                if (fastest) {
                    add_edge(v, u, Channel{min_io_ratio, buffer, i}, G);
                }
            }
        }
    }

    return pruneGraph(std::move(G), boost::adaptors::reverse(make_iterator_range(vertices(G))));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief pruneGraph
 ** ------------------------------------------------------------------------------------------------------------- */
template<class VertexList>
inline PipelineGenerator::ChannelGraph PipelineGenerator::pruneGraph(ChannelGraph && G, VertexList && V) const {
    // Take a transitive closure of G but whenever we attempt to insert an edge into the closure
    // that already exists, check instead whether the rate of our proposed edge is <= the existing
    // edge's rate. If so, the data availability/consumption is transitively guaranteed.
    for (const auto u : V) {
        for (auto ei : make_iterator_range(in_edges(u, G))) {
            const auto v = source(ei, G);
            const Channel & ci = G[ei];
            for (auto ej : make_iterator_range(out_edges(u, G))) {
                const auto w = target(ej, G);
                // ci.rate = BOUND_RATIO(u, v) * (STRIDE(u) / STRIDE(v))
                const auto scaling = RateValue(G[v]->getStride(), G[w]->getStride());
                const auto rate = ci.ratio * scaling;
                bool insert = true;
                for (auto ek : make_iterator_range(in_edges(w, G))) {
                    // do we already have a vw edge?
                    if (source(ek, G) == v) {
                        Channel & ck = G[ek];
                        if (rate <= ck.ratio) {
                            ck.buffer = nullptr;
                        }
                        insert = false;
                    }
                }
                if (insert) {
                    add_edge(v, w, Channel{rate}, G);
                }
            }
        }
    }

    // remove any closure edges from G
    remove_edge_if([&G](const ChannelGraph::edge_descriptor e) { return G[e].buffer == nullptr; }, G);

    for (const auto u : V) {
        if (in_degree(u, G) == 0) {
            // If we do not need to check any of its inputs, we can avoid testing any output of a kernel
            // with a rate ratio of at least 1
            remove_out_edge_if(u, [&G](const ChannelGraph::edge_descriptor e) {
                return G[e].ratio >= RateValue{1, 1};
            }, G);
        }
    }

    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief executeKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineGenerator::execute(const std::unique_ptr<KernelBuilder> & b, const unsigned index) {

    const Kernel * const kernel = kernels[index];
    b->setKernel(kernel);

    const auto & inputs = kernel->getStreamInputs();
    const auto & outputs = kernel->getStreamOutputs();

    BasicBlock * const kernelEntry = b->GetInsertBlock();
    BasicBlock * const kernelCode = b->CreateBasicBlock(kernel->getName());
    kernelFinished = b->CreateBasicBlock(kernel->getName() + "Finished");
    BasicBlock * const kernelExit = b->CreateBasicBlock(kernel->getName() + "Exit");

    b->CreateUnlikelyCondBr(b->getTerminationSignal(), kernelExit, kernelCode);

    b->SetInsertPoint(kernelFinished);
    terminated = b->CreatePHI(b->getInt1Ty(), 2);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        madeProgress = b->CreatePHI(b->getInt1Ty(), 3);
    }
    b->SetInsertPoint(kernelExit);
    PHINode * const isFinal = b->CreatePHI(b->getInt1Ty(), 2, kernel->getName() + "_isFinal");
    isFinal->addIncoming(b->getTrue(), kernelEntry);
    isFinal->addIncoming(terminated, kernelFinished);

    PHINode * progress = nullptr;
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        progress = b->CreatePHI(b->getInt1Ty(), 2, kernel->getName() + "_anyProgress");
        progress->addIncoming(anyProgress, kernelEntry);
        progress->addIncoming(madeProgress, kernelFinished);
    }

    // Since it is possible that a sole consumer of some stream could terminate early, set the
    // initial consumed amount to the amount produced in this iteration.

    // First determine the priorConsumedItemCounts, making sure
    // that none of them are previous input buffers of this kernel!
    std::vector<Value *> priorConsumedItemCount(inputs.size());

    for (unsigned i = 0; i < inputs.size(); ++i) {
        const StreamSetBuffer * const buffer = kernel->getStreamSetInputBuffer(i);
        auto c = consumedItemCount.find(buffer);
        if (c == consumedItemCount.end()) {
            const auto p = producedItemCount.find(buffer);
            assert (p != producedItemCount.end());
            priorConsumedItemCount[i] = p->second;
        } else {
            priorConsumedItemCount[i] = c->second;
        }
    }

    std::vector<PHINode *> consumedItemCountPhi(inputs.size());

    for (unsigned i = 0; i < inputs.size(); ++i) {
        const StreamSetBuffer * const buffer = kernel->getStreamSetInputBuffer(i);
        PHINode * const consumedPhi = b->CreatePHI(b->getSizeTy(), 2);
        auto c = consumedItemCount.find(buffer);
        if (c == consumedItemCount.end()) {
            consumedItemCount.emplace(buffer, consumedPhi);
        } else {
            c->second = consumedPhi;
        }
        consumedPhi->addIncoming(priorConsumedItemCount[i], kernelEntry);
        consumedItemCountPhi[i] = consumedPhi;
    }

    b->SetInsertPoint(kernelCode);

    checkIfAllInputKernelsAreTerminated(b, index);

    checkAvailableInputData(b, index);

    checkAvailableOutputSpace(b, index);

    applyOutputBufferExpansions(b, index);

    Value * const finalStride = callKernel(b, index);

    // TODO: add in some checks to verify the kernel actually adheres to its rate

    BasicBlock * const kernelCodeExit = b->GetInsertBlock();
    terminated->addIncoming(finalStride, kernelCodeExit);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        madeProgress->addIncoming(b->getTrue(), kernelCodeExit);
        anyProgress = progress;
    }
    b->CreateBr(kernelFinished);

    b->SetInsertPoint(kernelFinished);

    // update the consumed item counts
    for (unsigned i = 0; i < inputs.size(); ++i) {
        const Binding & input = inputs[i];
        Value * const fullyProcessed = getFullyProcessedItemCount(b, input, terminated);
        Value * const consumed = b->CreateUMin(priorConsumedItemCount[i], fullyProcessed);
        consumedItemCountPhi[i]->addIncoming(consumed, kernelFinished);
    }
    b->CreateBr(kernelExit);

    kernelExit->moveAfter(kernelFinished);

    b->SetInsertPoint(kernelExit);

    for (unsigned i = 0; i < outputs.size(); ++i) {
        const Binding & output = outputs[i];
        Value * const produced = b->getProducedItemCount(output.getName());
        const StreamSetBuffer * const buffer = kernel->getStreamSetOutputBuffer(i);
        // if some stream has no consumer, set the consumed item count to the produced item count
        const auto c = lastConsumer.find(buffer);
        assert (c != lastConsumer.end());
        if (LLVM_UNLIKELY(c->second == kernel)) {
            assert (buffer->getProducer() == kernel);
            if (LLVM_UNLIKELY(output.getRate().isRelative())) {
                continue;
            }
            b->setConsumedItemCount(output.getName(), produced);
        } else { // otherwise record how many items were produced
            assert (producedItemCount.count(buffer) == 0);
            producedItemCount.emplace(buffer, produced);
        }

    }


    // TODO: if all consumers process the data at a fixed rate, we can just set the consumed item count
    // by the strideNo rather than tracking it.


    // If this kernel is the last consumer of a input buffer, update the consumed count for that buffer.
    // NOTE: unless we can prove that this kernel cannot terminate before any prior consumer, we cannot
    // put this code into the kernelFinished block.
    for (unsigned i = 0; i < inputs.size(); ++i) {
        const StreamSetBuffer * const buffer = kernel->getStreamSetInputBuffer(i);
        const auto c = lastConsumer.find(buffer);
        assert (c != lastConsumer.end());
        if (LLVM_LIKELY(c->second == kernel)) {
            Kernel * const producer = buffer->getProducer();
            assert (producer != kernel);
            const auto & output = producer->getStreamOutput(buffer);
            if (output.getRate().isRelative()) continue;
            b->setKernel(producer);
            if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
                Value * const alreadyConsumed = b->getConsumedItemCount(output.getName());
                b->CreateAssert(b->CreateICmpULE(alreadyConsumed, consumedItemCountPhi[i]),
                                producer->getName() + ": " + output.getName() + " consumed item count is not monotonically non-decreasing!");
            }
            b->setConsumedItemCount(output.getName(), consumedItemCountPhi[i]);
            b->setKernel(kernel);
        }
    }

    dependencyGraph[index] = isFinal;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkAvailableInputData
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineGenerator::checkIfAllInputKernelsAreTerminated(const std::unique_ptr<KernelBuilder> & b, const unsigned index) {
    const auto n = in_degree(index, dependencyGraph);
    if (LLVM_UNLIKELY(n == 0)) {
        noMore = b->getFalse();
    } else {
        noMore = b->getTrue();
        for (auto e : make_iterator_range(in_edges(index, dependencyGraph))) {
            const auto u = source(e, dependencyGraph);
            Value * const finished = dependencyGraph[u];
            //b->CallPrintInt("* " + kernels[u]->getName() + "_hasFinished", finished);
            noMore = b->CreateAnd(noMore, finished);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkAvailableInputData
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineGenerator::checkAvailableInputData(const std::unique_ptr<KernelBuilder> & b, const unsigned index) {
    const Kernel * const kernel = kernels[index];
    b->setKernel(kernel);
    for (auto e : make_iterator_range(in_edges(index, inputGraph))) {
        const Channel & c = inputGraph[e];
        const Binding & input = kernel->getStreamInput(c.operand);

        Value * requiredInput = getStrideLength(b, kernel, input);
        if (LLVM_UNLIKELY(input.hasLookahead())) {
            Constant * const lookahead = b->getSize(input.getLookahead());
            requiredInput = b->CreateAdd(requiredInput, lookahead);
        }
        const auto p = producedItemCount.find(c.buffer);
        assert (p != producedItemCount.end());
        Value * const produced = p->second;
        Value * const processed = b->getNonDeferredProcessedItemCount(input);
        Value * const unprocessed = b->CreateSub(produced, processed);
        Value * const hasEnough = b->CreateICmpUGE(unprocessed, requiredInput);
        Value * const check = b->CreateOr(hasEnough, noMore);
        terminated->addIncoming(b->getFalse(), b->GetInsertBlock());
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            madeProgress->addIncoming(anyProgress, b->GetInsertBlock());
        }
        BasicBlock * const hasSufficientInput = b->CreateBasicBlock(kernel->getName() + "_" + input.getName() + "_hasSufficientInput");
        b->CreateLikelyCondBr(check, hasSufficientInput, kernelFinished);
        b->SetInsertPoint(hasSufficientInput);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkAvailableOutputSpace
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineGenerator::checkAvailableOutputSpace(const std::unique_ptr<KernelBuilder> & b, const unsigned index) {
    const Kernel * const kernel = kernels[index];
    b->setKernel(kernel);
    for (auto e : make_iterator_range(in_edges(index, outputGraph))) {
        const Channel & c = outputGraph[e];
        assert (c.buffer->getProducer() == kernel);
        const Binding & output = kernel->getStreamOutput(c.buffer);

        Value * requiredSpace = getStrideLength(b, kernel, output);
        const auto & name = output.getName();
        Value * const produced = b->getNonDeferredProducedItemCount(output);
        Value * const consumed = b->getConsumedItemCount(name);
        Value * const unconsumed = b->CreateSub(produced, consumed);
        requiredSpace = b->CreateAdd(requiredSpace, unconsumed);
        Value * const capacity = b->getBufferedSize(name);
        Value * const check = b->CreateICmpULE(requiredSpace, capacity);
        terminated->addIncoming(b->getFalse(), b->GetInsertBlock());
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            madeProgress->addIncoming(anyProgress, b->GetInsertBlock());
        }
        BasicBlock * const hasOutputSpace = b->CreateBasicBlock(kernel->getName() + "_" + name + "_hasOutputSpace");
        b->CreateLikelyCondBr(check, hasOutputSpace, kernelFinished);
        b->SetInsertPoint(hasOutputSpace);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineGenerator::getStrideLength(const std::unique_ptr<KernelBuilder> & b, const Kernel * const kernel, const Binding & binding) {
    Value * strideLength = nullptr;
    const ProcessingRate & rate = binding.getRate();
    if (rate.isPopCount() || rate.isNegatedPopCount()) {
        Port refPort; unsigned refIndex;
        std::tie(refPort, refIndex) = kernel->getStreamPort(rate.getReference());
        const Binding & ref = kernel->getStreamInput(refIndex);
        Value * markers = b->loadInputStreamBlock(ref.getName(), b->getSize(0));
        if (rate.isNegatedPopCount()) {
            markers = b->CreateNot(markers);
        }
        strideLength = b->bitblock_popcount(markers);
    } else if (binding.hasAttribute(kernel::Attribute::KindId::AlwaysConsume)) {
        const auto lb = kernel->getLowerBound(rate);
        strideLength = b->getSize(ceiling(lb * kernel->getStride()));
    } else {
        const auto ub = kernel->getUpperBound(rate); assert (ub > 0);
        strideLength = b->getSize(ceiling(ub * kernel->getStride()));
    }
    return strideLength;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callKernel
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineGenerator::callKernel(const std::unique_ptr<KernelBuilder> & b, const unsigned index) {

    const Kernel * const kernel = kernels[index];
    b->setKernel(kernel);

    #ifndef DISABLE_COPY_TO_OVERFLOW
    // Store how many items we produced by this kernel in the prior iteration. We'll use this to determine when
    // to mirror the first K segments
    const auto & outputs = kernel->getStreamOutputs();
    const auto m = outputs.size();

    std::vector<Value *> initiallyProducedItemCount(m, nullptr);
    for (unsigned i = 0; i < m; ++i) {
        const Binding & output = outputs[i];
        const auto & name = output.getName();
        const StreamSetBuffer * const buffer = kernel->getOutputStreamSetBuffer(name);
        if (isConsumedAtNonFixedRate.count(buffer)) {
            initiallyProducedItemCount[i] = b->getProducedItemCount(name);
        }
    }
    #endif

    const auto & inputs = kernel->getStreamInputs();
    const auto n = inputs.size();
    std::vector<Value *> arguments(n + 2);

    Value * isFinal = noMore;
    for (unsigned i = 0; i < n; ++i) {
        const Binding & input = inputs[i];
        const StreamSetBuffer * const buffer = kernel->getStreamSetInputBuffer(i);

        const auto p = producedItemCount.find(buffer);
        assert (p != producedItemCount.end());
        Value * const produced = p->second;

        const ProcessingRate & rate = input.getRate();
        if (rate.isPopCount()) {
            arguments[i + 2] = produced;
        } else {
            const unsigned strideSize = ceiling(kernel->getUpperBound(rate) * kernel->getStride());
            Value * const processed = b->getNonDeferredProcessedItemCount(input);
            Value * const limit = b->CreateAdd(processed, b->getSize(strideSize * codegen::SegmentSize));
            Value * const partial = b->CreateICmpULT(produced, limit);
            arguments[i + 2] = b->CreateSelect(partial, produced, limit);
            isFinal = b->CreateAnd(isFinal, partial);
        }
    }

    // TODO: pass in a strideNo for fixed rate streams to allow the kernel to calculate the current avail,
    // processed, and produced counts

    arguments[0] = kernel->getInstance();
    arguments[1] = isFinal;

    b->createDoSegmentCall(arguments);

    #ifndef DISABLE_COPY_TO_OVERFLOW
    // For each buffer with an overflow region of K blocks, overwrite the overflow with the first K blocks of
    // data to ensure that even if this stream is produced at a fixed rate but consumed at a bounded rate,
    // every kernel has a consistent view of the stream data.
    for (unsigned i = 0; i < m; ++i) {
        const Binding & output = outputs[i];
        const auto & name = output.getName();
        if (initiallyProducedItemCount[i]) {
            Value * const bufferSize = b->getBufferedSize(name);
            Value * const prior = initiallyProducedItemCount[i];
            Value * const offset = b->CreateURem(prior, bufferSize);
            Value * const produced = b->getNonDeferredProducedItemCount(output);
            Value * const buffered = b->CreateAdd(offset, b->CreateSub(produced, prior));
            BasicBlock * const copyBack = b->CreateBasicBlock(name + "MirrorOverflow");
            BasicBlock * const done = b->CreateBasicBlock(name + "MirrorOverflowDone");
            b->CreateCondBr(b->CreateICmpUGT(buffered, bufferSize), copyBack, done);
            b->SetInsertPoint(copyBack);
            b->CreateCopyToOverflow(name);
            b->CreateBr(done);
            b->SetInsertPoint(done);
        }
    }
    #endif

    return b->getTerminationSignal();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief applyOutputBufferExpansions
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineGenerator::applyOutputBufferExpansions(const std::unique_ptr<KernelBuilder> & b, const unsigned index) {
    const Kernel * const kernel = kernels[index];
    const auto & outputs = kernel->getStreamSetOutputBuffers();
    for (unsigned i = 0; i < outputs.size(); i++) {
        if (isa<DynamicBuffer>(outputs[i])) {


            const auto baseSize = ceiling(kernel->getUpperBound(kernel->getStreamOutput(i).getRate()) * kernel->getStride() * codegen::SegmentSize);
            if (LLVM_LIKELY(baseSize > 0)) {

                const auto & name = kernel->getStreamOutput(i).getName();

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
 * @brief getFullyProcessedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineGenerator::getFullyProcessedItemCount(const std::unique_ptr<KernelBuilder> & b, const Binding & input, Value * const isFinal) const {
    Value * const processed = b->getProcessedItemCount(input.getName());
    if (LLVM_UNLIKELY(input.hasAttribute(kernel::Attribute::KindId::BlockSize))) {
        // If the input rate has a block size attribute then --- for the purpose of determining how many
        // items have been consumed --- we consider a stream set to be fully processed when an entire
        // block (stride?) has been processed.
        Constant * const BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());
        Value * const partial = b->CreateAnd(processed, ConstantExpr::getNeg(BLOCK_WIDTH));
        return b->CreateSelect(isFinal, processed, partial);
    }
    return processed;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalize
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineGenerator::finalize(const std::unique_ptr<KernelBuilder> & b) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        ConstantInt * const ZERO = b->getSize(0);
        ConstantInt * const ONE = b->getSize(1);
        ConstantInt * const TWO = b->getSize(2);
        Value * const count = b->CreateSelect(anyProgress, ZERO, b->CreateAdd(deadLockCounter, ONE));
        b->CreateAssert(b->CreateICmpNE(count, TWO), "Dead lock detected: pipeline could not progress after two iterations");
        deadLockCounter->addIncoming(count, b->GetInsertBlock());
    }
    // return whether each sink has terminated
    Value * final = b->getTrue();
    for (const auto u : make_iterator_range(vertices(dependencyGraph))) {
        if (out_degree(u, dependencyGraph) == 0) {
            final = b->CreateAnd(final, dependencyGraph[u]);
        }
    }
    return final;

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineGenerator::printGraph(const ChannelGraph & G) const {

    auto & out = errs();

    out << "digraph G {\n";
    for (auto u : make_iterator_range(vertices(G))) {
        assert (G[u] == kernels[u]);
        if (in_degree(u, G) > 0 || out_degree(u, G) > 0) {
            out << "v" << u << " [label=\"" << u << ": "
                << G[u]->getName() << "\"];\n";
        }
    }

    for (auto e : make_iterator_range(edges(G))) {
        const Channel & c = G[e];
        const auto s = source(e, G);
        const auto t = target(e, G);

        out << "v" << s << " -> v" << t
            << " [label=\""



            << c.ratio.numerator() << " / " << c.ratio.denominator()
            << "\"];\n";
    }

    out << "}\n\n";
    out.flush();

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineGenerator::printGraph(const DependencyGraph & G) const {

    auto & out = errs();

    out << "digraph G {\n";
    for (auto u : make_iterator_range(vertices(G))) {
            out << "v" << u << " [label=\"" << u << ": "
                << kernels[u]->getName() << "\"];\n";
    }

    for (auto e : make_iterator_range(edges(G))) {
        const auto s = source(e, G);
        const auto t = target(e, G);
        out << "v" << s << " -> v" << t << ";\n";
    }

    out << "}\n\n";
    out.flush();

}
