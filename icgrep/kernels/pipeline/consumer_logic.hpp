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
    if (LLVM_UNLIKELY(out_degree(bufferVertex, mConsumerGraph) == 0)) {
        setConsumedItemCount(b, bufferVertex, produced);
    } else { // otherwise set the initial consumed amount to the produced amount
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
 * @brief getConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getConsumedItemCount(BuilderRef b, const unsigned outputPort) const {
    // TODO: if we can prove that this stream is always fully consumed, its consumed count
    // is its produced count.
    assert (mPipeline[mKernelIndex] == mKernel);
    assert (b->getKernel() == mKernel);
    const Binding & output = mKernel->getOutputStreamSetBinding(outputPort);
    Value * consumed = nullptr;
    const BufferNode & bn = mBufferGraph[getOutputBufferVertex(mKernelIndex, outputPort)];
    if (LLVM_LIKELY(bn.Type == BufferType::Internal)) {
        b->setKernel(mPipelineKernel);
        consumed = b->getScalarField(makeBufferName(mKernelIndex, output) + CONSUMED_ITEM_COUNT_SUFFIX);
        b->setKernel(mKernel);
    } else if (bn.Type == BufferType::Managed) {
        b->setKernel(mKernel);
        consumed = b->getScalarField(output.getName() + CONSUMED_ITEM_COUNT_SUFFIX);
    } else if (bn.Type == BufferType::External) {
        consumed = b->getSize(0);
    }
    assert (consumed);
    return consumed;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::setConsumedItemCount(BuilderRef b, const unsigned bufferVertex, Value * const consumed) const {
    const auto pe = in_edge(bufferVertex, mConsumerGraph);
    const auto producerVertex = source(pe, mConsumerGraph);
    const Kernel * const producer = mBufferGraph[producerVertex].Kernel;
    assert (producer->getHandle());
    const Binding & output = producer->getOutputStreamSetBinding(mConsumerGraph[pe]);
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(producerVertex, output);
    b->CallPrintInt(prefix + CONSUMED_ITEM_COUNT_SUFFIX, consumed);
    #endif
    if (LLVM_UNLIKELY(storedInNestedKernel(output))) {
        b->setKernel(producer);
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
            b->CreateMProtect(producer->getHandle(), CBuilder::Protect::WRITE);
        }
        b->setScalarField(output.getName() + CONSUMED_ITEM_COUNT_SUFFIX, consumed);
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
            b->CreateMProtect(producer->getHandle(), CBuilder::Protect::READ);
        }
    } else {
        b->setKernel(mPipelineKernel);
        b->setScalarField(makeBufferName(producerVertex, output) + CONSUMED_ITEM_COUNT_SUFFIX, consumed);
    }
    b->setKernel(mKernel);
}

}

#endif // CONSUMER_LOGIC_HPP
