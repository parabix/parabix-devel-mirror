#ifndef CONSUMER_LOGIC_HPP
#define CONSUMER_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

namespace {

const auto FAKE = std::numeric_limits<unsigned>::max();

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief minimumConsumed
 ** ------------------------------------------------------------------------------------------------------------- */
inline LLVM_READNONE RateValue minimumConsumed(const Kernel * const kernel, const Binding & binding) {
    if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::Deferred))) {
        return RateValue{0};
    }
    return lowerBound(kernel, binding);
}

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeConsumerGraph
 *
 * Copy the buffer graph but amalgamate any multi-edges into a single one
 ** ------------------------------------------------------------------------------------------------------------- */
ConsumerGraph PipelineCompiler::makeConsumerGraph()  const {

    const auto lastBuffer = num_vertices(mBufferGraph);
    ConsumerGraph G(lastBuffer);
    const auto numOfKernels = mPipeline.size();
    const auto firstBuffer = numOfKernels + 1;

    #warning TODO: ConsumerGraph assumes the dataflow is transitively bounded by the same initial source

    #warning REVISIT: ConsumerGraph is not optimal for handling relative rate inputs

    std::vector<std::pair<unsigned, unsigned>> consumers; // kernel, portIndex

    for (auto bufferVertex = firstBuffer; bufferVertex < lastBuffer; ++bufferVertex) {

        if (LLVM_UNLIKELY(isPipelineIO(mBufferGraph[bufferVertex].buffer))) {
            continue;
        }

        // copy the producing edge
        const auto pe = in_edge(bufferVertex, mBufferGraph);
        add_edge(source(pe, mBufferGraph), bufferVertex, mBufferGraph[pe].port, G);

        // collect the consumers of the i-th buffer
        consumers.clear();
        for (const auto e : make_iterator_range(out_edges(bufferVertex, mBufferGraph))) {
            consumers.emplace_back(target(e, mBufferGraph), mBufferGraph[e].port);
        }

        // If the minimum input rate of the j-th consumer is greater than or equal to the maximum input
        // rate of the k-th consumer, we do not need to test the j-th consumer. This also ensures that
        // for each *FIXED* rate stream, keep only the minimum such rate. However, we may need to insert
        // a "fake" edge to mark the last consumer otherwise we'll set it too soon.

        if (LLVM_LIKELY(consumers.size() > 1)) {
            std::sort(consumers.begin(), consumers.end());

            const auto finalConsumer = consumers.back().first;

            for (auto j = consumers.begin() + 1; j != consumers.end(); ) {

                const Kernel * const kernel_j = mPipeline[j->first];
                const Binding & input_j = kernel_j->getInputStreamSetBinding(j->second);
                const auto lb_j = minimumConsumed(kernel_j, input_j);

                for (auto k = consumers.begin(); k != j; ++k) {
                    const Kernel * const kernel_k = mPipeline[k->first];
                    const Binding & input_k = kernel_k->getInputStreamSetBinding(k->second);
                    const auto ub_k = upperBound(kernel_k, input_k);
                    if (LLVM_UNLIKELY(lb_j >= ub_k)) {
                        j = consumers.erase(j);
                        goto next;
                    }
                }

                for (auto k = j + 1; k != consumers.end(); ++k) {
                    const Kernel * const kernel_k = mPipeline[k->first];
                    const Binding & input_k = kernel_k->getInputStreamSetBinding(k->second);
                    const auto ub_k = upperBound(kernel_k, input_k);
                    if (LLVM_UNLIKELY(lb_j >= ub_k)) {
                        j = consumers.erase(j);
                        goto next;
                    }
                }

                ++j;
next:           continue;
            }
            if (LLVM_UNLIKELY(consumers.back().first != finalConsumer)) {
                consumers.emplace_back(finalConsumer, FAKE);
            }
        }

        for (const auto & consumer : consumers) {
            add_edge(bufferVertex, consumer.first, consumer.second, G);
        }
    }

    return G;
}

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
        ConsumerNode & cn = mConsumerGraph[bufferVertex]; assert (cn.consumed == nullptr);
        cn.consumed = produced;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createConsumedPhiNodes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::createConsumedPhiNodes(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(mKernelIndex, mConsumerGraph))) {
        if (LLVM_UNLIKELY(mConsumerGraph[e] == FAKE)) continue;
        const auto bufferVertex = source(e, mConsumerGraph);
        ConsumerNode & cn = mConsumerGraph[bufferVertex]; assert (cn.consumed);
        PHINode * const consumedPhi = b->CreatePHI(b->getSizeTy(), 2);
        consumedPhi->addIncoming(cn.consumed, mKernelEntry);
        cn.phiNode = consumedPhi;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeMinimumConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::computeMinimumConsumedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(mKernelIndex, mConsumerGraph))) {
        if (LLVM_UNLIKELY(mConsumerGraph[e] == FAKE)) continue;
        const Binding & input = mKernel->getInputStreamSetBinding(mConsumerGraph[e]);
        Value * const processed = getFullyProcessedItemCount(b, input);
        const auto bufferVertex = source(e, mConsumerGraph);
        ConsumerNode & cn = mConsumerGraph[bufferVertex]; assert (cn.consumed);
        cn.consumed = b->CreateUMin(cn.consumed, processed);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeFinalConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeFinalConsumedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(mKernelIndex, mConsumerGraph))) {
        const auto bufferVertex = source(e, mConsumerGraph);
        ConsumerNode & cn = mConsumerGraph[bufferVertex]; assert (cn.consumed);
        if (LLVM_LIKELY(mConsumerGraph[e] != FAKE)) {
            cn.phiNode->addIncoming(cn.consumed, mKernelLoopExitPhiCatch);
            cn.consumed = cn.phiNode;
        }
        // Is this kernel the last consumer? If so, store the consumed count
        if (out_degree(bufferVertex, mConsumerGraph) == 1) {
            setConsumedItemCount(b, bufferVertex, cn.consumed);
        }
    }
    clear_in_edges(mKernelIndex, mConsumerGraph);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getConsumedItemCount(BuilderRef b, const unsigned index) const {
    assert (mPipeline[mKernelIndex] == mKernel);
    assert (b->getKernel() == mKernel);
    const Binding & output = mKernel->getOutputStreamSetBinding(index);
    StreamSetBuffer * const buffer = getOutputBuffer(index);
    Value * consumed = nullptr;
    if (LLVM_UNLIKELY(isa<ExternalBuffer>(buffer))) {
        if (LLVM_UNLIKELY(isPipelineIO(buffer))) {
            consumed = b->getSize(0);
        } else {
            consumed = b->getScalarField(output.getName() + CONSUMED_ITEM_COUNT_SUFFIX);
        }
    } else {
        b->setKernel(mPipelineKernel);
        consumed = b->getScalarField(makeBufferName(mKernelIndex, output) + CONSUMED_ITEM_COUNT_SUFFIX);
        b->setKernel(mKernel);
    }
    return consumed;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::setConsumedItemCount(BuilderRef b, const unsigned bufferVertex, Value * const consumed) const {
    const auto pe = in_edge(bufferVertex, mConsumerGraph);
    const auto producerVertex = source(pe, mConsumerGraph);
    const Kernel * const producer = mBufferGraph[producerVertex].kernel;
    assert (producer->getHandle());
    const Binding & output = producer->getOutputStreamSetBinding(mConsumerGraph[pe]);
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
