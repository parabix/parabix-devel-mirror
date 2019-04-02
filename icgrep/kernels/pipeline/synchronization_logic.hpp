#ifndef SYNCHRONIZATION_LOGIC_HPP
#define SYNCHRONIZATION_LOGIC_HPP

#include "pipeline_compiler.hpp"

// Suppose T1 and T2 are two pipeline threads where all segment processing
// of kernel Ki in T1 logically happens before Ki in T2.

// Any stateless kernel (or kernel marked as internally synchronized) with
// countable input rates that is not a source, sink or side-effecting can
// be executed in parallel once we've calculated the "future" item count
// position(s). However, T2 may finish before T1 and a Kj>i in T2 could
// attempt to consume unfinished data from T1. So we must ensure that T1
// is always completely finished before T2 may execute Kj.

// For any such kernel, we require two counters. The first marks that T1
// has computed T2's initial processed item counts. The second informs T2
// when T1 has finished writing to its output streams. T2 may begin p
// rocessing once it acquires the first lock but cannot write its output
// until acquiring the second.

// If each stride of output of Ki cannot be guaranteed to be written on
// a cache-aligned boundary, regardless of input state, a temporary output
// buffer is required. After acquiring the second lock, the data
// must be copied to the shared stream set. To minimize redundant copies,
// if at the point of executing Ki,
// we require an additional lock that indicates whether some kernel "owns"
// the actual stream set.

// Even though T1 and T2 process a segment per call, a segment may require
// several iterations (due to buffer contraints, final stride processing,
// etc.) Thus to execute a stateful internally synchronized kernel, we must
// hold both buffer locks until reaching the last partial segment.

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief acquireBufferSetLock
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::acquireLock(BuilderRef b, const std::string & lockName) const {
    Value * const ptr = b->getScalarFieldPtr(lockName);
    const auto prefix = makeKernelName(mKernelIndex);
    BasicBlock * const wait = b->CreateBasicBlock(prefix + "_acquire" + lockName);
    BasicBlock * const acquired = b->CreateBasicBlock(prefix + "_acquired" + lockName);
    b->CreateCondBr(b->CreateICmpEQ(b->CreateAtomicLoadAcquire(ptr), mSegNo), acquired, wait);
    b->SetInsertPoint(wait);
    b->CreatePThreadYield();
    b->CreateCondBr(b->CreateICmpEQ(b->CreateAtomicLoadAcquire(ptr), mSegNo), acquired, wait);
    b->SetInsertPoint(acquired);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief acquireItemCountLock
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::acquireItemCountLock(BuilderRef b) const {
    // acquireLock(b, ITEM_COUNT_LOCK);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeBufferSetGraph
 ** ------------------------------------------------------------------------------------------------------------- */
BufferSetGraph PipelineCompiler::makeBufferSetGraph() const {

    // We need to account for the pipeline I/O individually but each set of output
    // from an internal kernel can be considered together.

    SmallVector<unsigned, 64> M(num_vertices(mBufferGraph) - PipelineOutput, 0);
    const auto m  = PipelineOutput + 1U;
    auto n = m;
    for (const auto & e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
        M[target(e, mBufferGraph) - m] = n++;
    }
    for (const auto & e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        M[source(e, mBufferGraph) - m] = n++;
    }
    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        if (LLVM_UNLIKELY(out_degree(i, mBufferGraph) == 0)) {
            continue;
        }
        for (const auto & e : make_iterator_range(out_edges(i, mBufferGraph))) {
            auto & Mi = M[target(e, mBufferGraph) - m];
            if (LLVM_LIKELY(Mi == 0)) {
                Mi = n;
            }
        }
        ++n;
    }

    BufferSetGraph G(n);

    for (auto i = PipelineInput; i <= PipelineOutput; ++i) {
        for (const auto & e : make_iterator_range(in_edges(i, mBufferGraph))) {
            add_edge(M[source(e, mBufferGraph) - m], i, G);
        }
        for (const auto & e : make_iterator_range(out_edges(i, mBufferGraph))) {
            add_edge(i, M[target(e, mBufferGraph) - m], G);
        }
    }

    transitive_reduction_dag(G);

    unsigned lock = 0;
    for (const auto & e : make_iterator_range(edges(G))) {
        G[e] = lock++;
    }

    return G;

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief acquireCurrentSegment
 *
 * Before the segment is processed, this loads the segment number of the kernel state and ensures the previous
 * segment is complete (by checking that the acquired segment number is equal to the desired segment number).
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::acquireCurrentSegment(BuilderRef b) {
    if (LLVM_LIKELY(requiresSynchronization(mKernelIndex))) {

        b->setKernel(mPipelineKernel);
        const auto prefix = makeKernelName(mKernelIndex);
        const auto serialize = codegen::DebugOptionIsSet(codegen::SerializeThreads);
        const unsigned waitingOnIdx = serialize ? LastKernel : mKernelIndex;
        const auto waitingOn = makeKernelName(waitingOnIdx);
        Value * const waitingOnPtr = b->getScalarFieldPtr(waitingOn + LOGICAL_SEGMENT_SUFFIX);
        BasicBlock * const kernelWait = b->CreateBasicBlock(prefix + "Wait", mPipelineEnd);
        b->CreateBr(kernelWait);

        b->SetInsertPoint(kernelWait);
        Value * const currentSegNo = b->CreateAtomicLoadAcquire(waitingOnPtr);
        Value * const ready = b->CreateICmpEQ(mSegNo, currentSegNo);
        BasicBlock * const kernelStart = b->CreateBasicBlock(prefix + "Start", mPipelineEnd);
        b->CreateCondBr(ready, kernelStart, kernelWait);

        b->SetInsertPoint(kernelStart);
        b->setKernel(mKernel);
        updateCycleCounter(b, CycleCounter::INITIAL, CycleCounter::AFTER_SYNCHRONIZATION);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief releaseCurrentSegment
 *
 * After executing the kernel, the segment number must be incremented to release the kernel for the next thread.
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::releaseCurrentSegment(BuilderRef b) {
    if (LLVM_LIKELY(requiresSynchronization(mKernelIndex))) {
        b->setKernel(mPipelineKernel);
        Value * const nextSegNo = b->CreateAdd(mSegNo, b->getSize(1));
        const auto prefix = makeKernelName(mKernelIndex);
        Value * const waitingOnPtr = b->getScalarFieldPtr(prefix + LOGICAL_SEGMENT_SUFFIX);
        b->CreateAtomicStoreRelease(nextSegNo, waitingOnPtr);
        b->setKernel(mKernel);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresSynchronization
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::requiresSynchronization(const unsigned kernelIndex) const {
    const Kernel * const kernel = getKernel(kernelIndex);
    if (kernel->hasAttribute(AttrId::InternallySynchronized)) {
        return false;
    }
    // TODO: Not quite ready yet: we need a function to calculate how many items
    // will be processed/produced by the i-th execution of this kernel based
    // strictly on the number of items produced by the (i-1)-th and the i-th
    // segment for each input to this kernel. Moreover, if we have static buffers,
    // we must statically know how many items will be consumed by any segment
    // based only only the value of i and/or the above-mentioned information.
    return true;
#if 0
    if (LLVM_LIKELY(kernel->isStateful())) {
        return true;
    }
    const auto numOfInputs = in_degree(kernelIndex, mBufferGraph);
    for (unsigned i = 0; i < numOfInputs; i++) {
        const Binding & input = getInputBinding(kernelIndex, i);
        if (!input.getRate().isFixed()) {
            return true;
        }
    }
    const auto numOfOutputs = out_degree(kernelIndex, mBufferGraph);
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const Binding & output = getOutputBinding(kernelIndex, i);
        if (!output.getRate().isFixed()) {
            return true;
        }
    }
    return false;
#endif
}



/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isKernelDataParallel
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::isKernelDataParallel(const unsigned kernel) const {

    // Any stateless kernel (or kernel marked as internally synchronized) with
    // countable input rates that is not a source, sink or side-effecting may
    // be executed in parallel.

    // NOTE: if the output of a kernel cannot be statically guaranteed to be
    // written on cache-aligned boundaries, regardless of input state, a
    // temporary buffer is required.

    const Kernel * const K = getKernel(kernel);
    if (LLVM_LIKELY(K->isStateful() && !K->hasAttribute(AttrId::InternallySynchronized))) {
        return false;
    }
    if (LLVM_UNLIKELY(in_degree(kernel, mBufferGraph) == 0)) {
        return false;
    }
    if (LLVM_UNLIKELY(out_degree(kernel, mBufferGraph) == 0)) {
        return false;
    }
    if (LLVM_UNLIKELY(K->hasAttribute(AttrId::SideEffecting))) {
        return false;
    }
    for (const auto & e : make_iterator_range(in_edges(kernel, mBufferGraph))) {
        const BufferRateData & rd = mBufferGraph[e];
        if (!isCountable(rd.Binding)) {
            return false;
        }
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isOutputCacheAligned
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::isOutputCacheAligned(BuilderRef b, const unsigned kernel, const unsigned outputPort) const {

    // If the output of a kernel cannot be statically guaranteed to be written on cache-aligned
    // boundaries, regardless of input state, a temporary buffer is required.

    const Binding & output = getOutputBinding(kernel, outputPort);
    const ProcessingRate & rate = output.getRate();
    if (rate.isFixed()) {
        const Kernel * const K = getKernel(kernel);
        const auto strideSize = (K->getStride() * output.getFieldWidth() * output.getNumElements());
        return (strideSize % (b->getCacheAlignment() * 8)) == 0;
    }
    return false;
}

}

#endif // SYNCHRONIZATION_LOGIC_HPP
