#ifndef CONSUMER_LOGIC_HPP
#define CONSUMER_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeConsumedItemCount(BuilderRef b, const unsigned bufferVertex, Value * const produced) {
    // If this stream has no consumers, immediately store the consumed item count.
    if (LLVM_UNLIKELY(in_degree(bufferVertex, mConsumerGraph) == 0)) {
        return;
    }
    if (LLVM_UNLIKELY(out_degree(bufferVertex, mConsumerGraph) != 0)) {
        ConsumerNode & cn = mConsumerGraph[bufferVertex];
        assert (cn.Consumed == nullptr);
        cn.Consumed = produced;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createConsumedPhiNodes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::createConsumedPhiNodes(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(mKernelIndex, mConsumerGraph))) {
        if (LLVM_UNLIKELY(mConsumerGraph[e] == FAKE_CONSUMER)) continue;
        const auto bufferVertex = source(e, mConsumerGraph);
        ConsumerNode & cn = mConsumerGraph[bufferVertex]; assert (cn.Consumed);
        PHINode * const consumedPhi = b->CreatePHI(b->getSizeTy(), 2);
        consumedPhi->addIncoming(cn.Consumed, mKernelEntry);
        cn.PhiNode = consumedPhi;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeMinimumConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::computeMinimumConsumedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(mKernelIndex, mConsumerGraph))) {
        if (LLVM_UNLIKELY(mConsumerGraph[e] == FAKE_CONSUMER)) continue;
        Value * const processed = mFullyProcessedItemCount[mConsumerGraph[e]];
        const auto bufferVertex = source(e, mConsumerGraph);
        ConsumerNode & cn = mConsumerGraph[bufferVertex]; assert (cn.Consumed);
        cn.Consumed = b->CreateUMin(cn.Consumed, processed);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeFinalConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeFinalConsumedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(mKernelIndex, mConsumerGraph))) {
        const auto bufferVertex = source(e, mConsumerGraph);
        ConsumerNode & cn = mConsumerGraph[bufferVertex]; assert (cn.Consumed);
        if (LLVM_LIKELY(mConsumerGraph[e] != FAKE_CONSUMER)) {
            cn.PhiNode->addIncoming(cn.Consumed, mKernelLoopExitPhiCatch);
            cn.Consumed = cn.PhiNode;
        }
        // Is this kernel the last consumer? If so, store the consumed count
        if (out_degree(bufferVertex, mConsumerGraph) == 1) {
            setConsumedItemCount(b, bufferVertex, cn.Consumed);
        }
    }
    clear_in_edges(mKernelIndex, mConsumerGraph);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addConsumerKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addConsumerKernelProperties(BuilderRef b, const unsigned kernelIndex) {
    IntegerType * const sizeTy = b->getSizeTy();
    const Kernel * const kernel = mPipeline[kernelIndex];
    const auto numOfOutputs = kernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const Binding & output = kernel->getOutputStreamSetBinding(i);
        const auto prefix = makeBufferName(kernelIndex, output);
        const auto bufferVertex = getOutputBufferVertex(kernelIndex, i);
        // If the out-degree for this buffer is zero, then either the stream has no consumers
        // or we've proven that its consumption rate is identical to its production rate.
        if (out_degree(bufferVertex, mConsumerGraph) != 0) {
            mPipelineKernel->addInternalScalar(sizeTy, prefix + CONSUMED_ITEM_COUNT_SUFFIX);
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getConsumedItemCount(BuilderRef b, const unsigned outputPort) {
    Value * consumed = nullptr;
    const auto bufferVertex = getOutputBufferVertex(outputPort);
    if (LLVM_UNLIKELY(out_degree(bufferVertex, mConsumerGraph) == 0)) {
        // This stream either has no consumers or we've proven that its consumption rate
        // is identical to its production rate.
        consumed = mInitiallyProducedItemCount[outputPort];
    } else {
        const BufferNode & bn = mBufferGraph[bufferVertex];
        if (LLVM_UNLIKELY(bn.Type == BufferType::External)) {
            consumed = b->getSize(0);
        } else {
            b->setKernel(mPipelineKernel);
            const Binding & output = mKernel->getOutputStreamSetBinding(outputPort);
            const auto prefix = makeBufferName(mKernelIndex, output);
            consumed = b->getScalarField(prefix + CONSUMED_ITEM_COUNT_SUFFIX);
            b->setKernel(mKernel);
        }
    }
    return consumed;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::readConsumedItemCounts(BuilderRef b) {
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; i++) {
        mConsumedItemCount[i] = getConsumedItemCount(b, i);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::setConsumedItemCount(BuilderRef b, const unsigned bufferVertex, Value * const consumed) const {
    const BufferNode & bn = mBufferGraph[bufferVertex];
    if (LLVM_LIKELY(bn.Type == BufferType::External)) {
        return;
    }
    const auto pe = in_edge(bufferVertex, mConsumerGraph);
    const auto producerVertex = source(pe, mConsumerGraph);
    const Kernel * const producer = mPipeline[producerVertex];
    const auto outputPort = mConsumerGraph[pe];
    const Binding & output = producer->getOutputStreamSetBinding(outputPort);
    const auto prefix = makeBufferName(producerVertex, output);
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(prefix + CONSUMED_ITEM_COUNT_SUFFIX, consumed);
    #endif
    b->setKernel(mPipelineKernel);
    b->setScalarField(prefix + CONSUMED_ITEM_COUNT_SUFFIX, consumed);
    b->setKernel(mKernel);
}

}

#endif // CONSUMER_LOGIC_HPP
