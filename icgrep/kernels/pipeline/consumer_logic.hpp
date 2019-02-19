#ifndef CONSUMER_LOGIC_HPP
#define CONSUMER_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeConsumedItemCount(const unsigned bufferVertex, Value * const produced) {
    ConsumerNode & cn = mConsumerGraph[bufferVertex];
    cn.Consumed = produced;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createConsumedPhiNodes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::createConsumedPhiNodes(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(mKernelIndex, mConsumerGraph))) {
        if (LLVM_UNLIKELY(mConsumerGraph[e] == FAKE_CONSUMER)) continue;
        const auto bufferVertex = source(e, mConsumerGraph);
        ConsumerNode & cn = mConsumerGraph[bufferVertex];
        if (LLVM_LIKELY(cn.PhiNode == nullptr)) {
            PHINode * const consumedPhi = b->CreatePHI(b->getSizeTy(), 2, "consumed." + std::to_string(bufferVertex) + ".");
            consumedPhi->addIncoming(cn.Consumed, mKernelEntry);
            cn.PhiNode = consumedPhi;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeMinimumConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::computeMinimumConsumedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(mKernelIndex, mConsumerGraph))) {
        if (LLVM_UNLIKELY(mConsumerGraph[e] == FAKE_CONSUMER)) continue;
        const auto inputPort = InputPort(mConsumerGraph[e]);
        Value * const processed = mFullyProcessedItemCount[inputPort];
        const auto bufferVertex = source(e, mConsumerGraph);
        ConsumerNode & cn = mConsumerGraph[bufferVertex]; assert (cn.Consumed);
        cn.Consumed = b->CreateUMin(cn.Consumed, processed);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addConsumerKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addConsumerKernelProperties(BuilderRef b, const unsigned kernelIndex) {
    IntegerType * const sizeTy = b->getSizeTy();
    const Kernel * const kernel = mPipeline[kernelIndex];
    for (const auto & e : make_iterator_range(out_edges(kernelIndex, mConsumerGraph))) {
        const auto bufferVertex = target(e, mConsumerGraph);
        // If the out-degree for this buffer is zero, then we've proven that its consumption rate
        // is identical to its production rate.
        const auto comsumedItemCountMatchesProducedItemCount = (out_degree(bufferVertex, mConsumerGraph) == 0);
        const auto isPipelineInput = (kernelIndex == PipelineInput);
        if (LLVM_UNLIKELY(comsumedItemCountMatchesProducedItemCount && !isPipelineInput)) {
            continue;
        }
        const Binding & binding = getBinding(kernel, mConsumerGraph[e]);
        const auto prefix = makeBufferName(kernelIndex, binding);
//        if (LLVM_UNLIKELY(isPipelineInput)) {
//            mPipelineKernel->addLocalScalar(sizeTy, prefix + CONSUMED_ITEM_COUNT_SUFFIX);
//        } else {
            mPipelineKernel->addInternalScalar(sizeTy, prefix + CONSUMED_ITEM_COUNT_SUFFIX);
//        }

    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::readConsumedItemCounts(BuilderRef b) {
    b->setKernel(mPipelineKernel);
    for (const auto & e : make_iterator_range(out_edges(mKernelIndex, mConsumerGraph))) {
        const auto port = OutputPort(mConsumerGraph[e]);
        const auto bufferVertex = target(e, mConsumerGraph);
        Value * consumed = nullptr;
        if (LLVM_UNLIKELY(out_degree(bufferVertex, mConsumerGraph) == 0)) {
            // This stream either has no consumers or we've proven that its consumption rate
            // is identical to its production rate.
            consumed = mInitiallyProducedItemCount[port];
        } else {
            const Binding & output = mKernel->getOutputStreamSetBinding(port);
            const auto prefix = makeBufferName(mKernelIndex, output);
            consumed = b->getScalarField(prefix + CONSUMED_ITEM_COUNT_SUFFIX);
        }
        mConsumedItemCount[port] = consumed;
        mPriorConsumedItemCount[getBufferIndex(bufferVertex)] = consumed;
    }
    b->setKernel(mKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeFinalConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeFinalConsumedItemCounts(BuilderRef b) {

    flat_set<unsigned> buffers;
    buffers.reserve(in_degree(mKernelIndex, mConsumerGraph));

    for (const auto e : make_iterator_range(in_edges(mKernelIndex, mConsumerGraph))) {
        const auto buffer = source(e, mConsumerGraph);
        buffers.insert(buffer);
        if (LLVM_UNLIKELY(mConsumerGraph[e] == FAKE_CONSUMER)) continue;
        ConsumerNode & cn = mConsumerGraph[buffer];
        if (LLVM_LIKELY(cn.PhiNode != nullptr)) {
            cn.PhiNode->addIncoming(cn.Consumed, mKernelLoopExitPhiCatch);
            cn.Consumed = cn.PhiNode;
            cn.PhiNode = nullptr;
        }
    }
    clear_in_edges(mKernelIndex, mConsumerGraph);

    b->setKernel(mPipelineKernel);
    // check to see if we've fully finished processing any stream
    for (const auto buffer : buffers) {
        if (out_degree(buffer, mConsumerGraph) == 0) {
            ConsumerNode & cn = mConsumerGraph[buffer];
            setConsumedItemCount(b, buffer, cn.Consumed);
        }
    }
    b->setKernel(mKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::setConsumedItemCount(BuilderRef b, const unsigned buffer, not_null<Value *> consumed) const {
    const auto pe = in_edge(buffer, mConsumerGraph);
    const auto producerVertex = source(pe, mConsumerGraph);
    const Kernel * const producer = mPipeline[producerVertex];
    const Binding & binding = getBinding(producer, mConsumerGraph[pe]);
    const auto prefix = makeBufferName(producerVertex, binding);
    b->setScalarField(prefix + CONSUMED_ITEM_COUNT_SUFFIX, consumed);
}

}

#endif // CONSUMER_LOGIC_HPP
