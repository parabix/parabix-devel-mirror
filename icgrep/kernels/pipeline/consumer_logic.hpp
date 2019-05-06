#ifndef CONSUMER_LOGIC_HPP
#define CONSUMER_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {


namespace {

#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief minimumConsumed
 ** ------------------------------------------------------------------------------------------------------------- */
inline LLVM_READNONE RateValue minimumConsumed(const Kernel * const kernel, const Binding & binding) {
    if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::Deferred))) {
        return RateValue{0};
    }
    return lowerBound(kernel, binding);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief maximumConsumed
 ** ------------------------------------------------------------------------------------------------------------- */
inline LLVM_READNONE RateValue maximumConsumed(const Kernel * const kernel, const Binding & binding) {
    auto ub = upperBound(kernel, binding);
    if (binding.hasLookahead()) {
        ub += binding.getLookahead();
    }
    return ub;
}

#endif

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeConsumerGraph
 *
 * Copy the buffer graph but amalgamate any multi-edges into a single one
 ** ------------------------------------------------------------------------------------------------------------- */
ConsumerGraph PipelineCompiler::makeConsumerGraph()  const {

    const auto firstBuffer = PipelineOutput + 1;
    const auto lastBuffer = num_vertices(mBufferGraph);
    ConsumerGraph G(lastBuffer);

#if 0

    #warning TODO: ConsumerGraph assumes the dataflow is transitively bounded by the same initial source

    #warning REVISIT: ConsumerGraph is not optimal for handling relative rate inputs

    struct ConsumerData {
        unsigned Kernel{0};
        unsigned Port{0};
        RateValue Minimum{0};
        RateValue Maximum{0};

        inline bool operator < (const ConsumerData & other) const {
            return (Kernel < other.Kernel) || (Port < other.Port);
        }
    };

    std::vector<ConsumerData> consumers;
#endif

    for (auto bufferVertex = firstBuffer; bufferVertex < lastBuffer; ++bufferVertex) {
        // copy the producing edge
        const auto pe = in_edge(bufferVertex, mBufferGraph);
        const BufferRateData & br = mBufferGraph[pe];
        add_edge(source(pe, mBufferGraph), bufferVertex, ConsumerEdge{br.Port, 0}, G);
        unsigned index = 0;
        for (const auto & ce : make_iterator_range(out_edges(bufferVertex, mBufferGraph))) {
            const BufferRateData & br = mBufferGraph[ce];
            add_edge(bufferVertex, target(ce, mBufferGraph), ConsumerEdge{br.Port, ++index}, G);
        }

#if 0

        // collect the consumers of the i-th buffer
        consumers.clear();
        for (const auto e : make_iterator_range(out_edges(bufferVertex, mBufferGraph))) {
            ConsumerData cd;
            cd.Kernel = target(e, mBufferGraph);
            const BufferNode & bn = mBufferGraph[cd.Kernel];
            const BufferRateData & rd = mBufferGraph[e];
            cd.Port = rd.Port;
            const Kernel * const kernel = mPipeline[cd.Kernel];
            const Binding & input = kernel->getInputStreamSetBinding(cd.Port); // <-- fix!
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::Deferred))) {
                cd.Minimum = RateValue{0};
            } else {
                cd.Minimum = bn.Lower * rd.Minimum;
            }
            cd.Maximum = bn.Upper * rd.Maximum;
            if (LLVM_UNLIKELY(input.hasLookahead())) {
                cd.Maximum += input.getLookahead();
            }
            consumers.emplace_back(cd);
        }

        // If the minimum input rate of the j-th consumer is greater than or equal to the maximum input
        // rate of the k-th consumer, we do not need to test the j-th consumer. This also ensures that
        // for each *FIXED* rate stream, keep only the minimum such rate. However, we may need to insert
        // a "fake" edge to mark the last consumer otherwise we'll set it too soon.

        // NOTE: here we need to consider the impact of lookahead on the use of a buffer since it may
        // limit how much work we can perform when nearing the end of the buffer.

        // TODO: this takes too narrow of a view of the problem. By considering a buffer's consumers
        // in isolation, it does not take into account that a particular kernel may be executed fewer
        // times than another because of I/O constraints independent of the buffer we're considering.
        // Essentially, to make this optimization safe we need to prove that if a consumer has performed
        // k strides, all other consumers performed k.

        if (LLVM_LIKELY(consumers.size() > 1)) {

            std::sort(consumers.begin(), consumers.end());

            const auto finalConsumer = consumers.back().first;

            for (auto j = consumers.begin() + 1; j != consumers.end(); ) {

                const ConsumerData & Cj = *j;
                for (auto k = consumers.begin(); k != j; ++k) {
                    const ConsumerData & Ck = *k;
                    if (LLVM_UNLIKELY(Cj.Minimum >= Ck.Maximum)) {
                        j = consumers.erase(j);
                        goto next;
                    }
                }

                for (auto k = j + 1; k != consumers.end(); ++k) {
                    const ConsumerData & Ck = *k;
                    if (LLVM_UNLIKELY(Cj.Minimum >= Ck.Maximum)) {
                        j = consumers.erase(j);
                        goto next;
                    }
                }

                ++j;
next:           continue;
            }
            if (LLVM_UNLIKELY(consumers.back().first != finalConsumer)) {
                consumers.emplace_back(finalConsumer, FAKE_CONSUMER);
            }
        }

        for (const auto & consumer : consumers) {
            add_edge(bufferVertex, consumer.first, consumer.second, G);
        }
#endif

    }

    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addConsumerKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addConsumerKernelProperties(BuilderRef b, const unsigned kernel) {
    IntegerType * const sizeTy = b->getSizeTy();
    for (const auto & e : make_iterator_range(out_edges(kernel, mBufferGraph))) {
        const auto buffer = target(e, mBufferGraph);
        // If the out-degree for this buffer is zero, then we've proven that its consumption rate
        // is identical to its production rate.

        const auto numOfConsumers = out_degree(buffer, mConsumerGraph);
        const auto consumedItemCountMatchesProducedItemCount = (numOfConsumers == 0);
        if (LLVM_UNLIKELY(consumedItemCountMatchesProducedItemCount && (kernel != PipelineInput))) {
            continue;
        }
        const BufferRateData & rd = mBufferGraph[e];
        assert (rd.Port.Type == PortType::Output);
        const auto name = makeBufferName(kernel, rd.Port) + CONSUMED_ITEM_COUNT_SUFFIX;

        Type * countTy = sizeTy;
        if (LLVM_UNLIKELY(mTrackIndividualConsumedItemCounts)) {
            countTy = ArrayType::get(sizeTy, numOfConsumers + 1);
        }
        if (LLVM_UNLIKELY(kernel == PipelineInput)) {
            mPipelineKernel->addNonPersistentScalar(countTy, name);
        } else {
            mPipelineKernel->addInternalScalar(countTy, name);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeConsumedItemCount(BuilderRef b, const unsigned outputPort, Value * const produced) {
    Value * initiallyConsumed = produced;
    const Binding & binding = getOutputBinding(outputPort);
    if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::LookBehind))) {
        const Attribute & attr = binding.findAttribute(AttrId::LookBehind);
        Constant * const lookBehind = b->getSize(attr.amount());
        Value * consumed = b->CreateSub(initiallyConsumed, lookBehind);
        Value * const satisfies = b->CreateICmpUGT(initiallyConsumed, lookBehind);
        initiallyConsumed = b->CreateSelect(satisfies, consumed, b->getSize(0));
    }
    ConsumerNode & cn = mConsumerGraph[getOutputBufferVertex(outputPort)];
    cn.Consumed = initiallyConsumed;
    cn.Encountered = 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::readConsumedItemCounts(BuilderRef b) {
    b->setKernel(mPipelineKernel);
    for (const auto & e : make_iterator_range(out_edges(mKernelIndex, mConsumerGraph))) {
        const ConsumerEdge & c = mConsumerGraph[e];
        const auto port = c.Port;
        const auto bufferVertex = target(e, mConsumerGraph);
        Value * consumed = nullptr;
        if (LLVM_UNLIKELY(out_degree(bufferVertex, mConsumerGraph) == 0)) {
            // This stream either has no consumers or we've proven that its consumption rate
            // is identical to its production rate.
            consumed = mInitiallyProducedItemCount[port];
        } else {
            const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, port});
            if (LLVM_UNLIKELY(mTrackIndividualConsumedItemCounts)) {
                Value * const ptr = b->getScalarFieldPtr(prefix + CONSUMED_ITEM_COUNT_SUFFIX);
                Constant * const ZERO = b->getInt32(0);
                consumed = b->CreateLoad(b->CreateGEP(ptr, { ZERO, ZERO } ));
            } else {
                consumed = b->getScalarField(prefix + CONSUMED_ITEM_COUNT_SUFFIX);
            }
        }
        mConsumedItemCount[port] = consumed;
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, port});
        b->CallPrintInt(prefix + "_consumed", consumed);
//        Value * const unconsumedDelta = b->CreateSub(mInitiallyProducedItemCount[port], consumed);
//        b->CallPrintInt(prefix + "_unconsumedΔ", unconsumedDelta);
        #endif
    }
    b->setKernel(mKernel);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createConsumedPhiNodes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::createConsumedPhiNodes(BuilderRef b) {
    IntegerType * const sizeTy = b->getSizeTy();
    for (const auto & e : make_iterator_range(in_edges(mKernelIndex, mConsumerGraph))) {
        //if (LLVM_UNLIKELY(mConsumerGraph[e] == FAKE_CONSUMER)) continue;
        const auto buffer = source(e, mConsumerGraph);
        ConsumerNode & cn = mConsumerGraph[buffer];
        if (LLVM_LIKELY(cn.PhiNode == nullptr)) {
            const ConsumerEdge & c = mConsumerGraph[e];
            const auto inputPort = c.Port;
            const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Input, inputPort});
            PHINode * const consumedPhi = b->CreatePHI(sizeTy, 2, prefix + "_consumed");
            consumedPhi->addIncoming(cn.Consumed, mKernelEntry);
            cn.PhiNode = consumedPhi;
            cn.Encountered++;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeMinimumConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::computeMinimumConsumedItemCounts(BuilderRef b) {
    for (const auto & e : make_iterator_range(in_edges(mKernelIndex, mConsumerGraph))) {
        //if (LLVM_UNLIKELY(mConsumerGraph[e] == FAKE_CONSUMER)) continue;
        const ConsumerEdge & c = mConsumerGraph[e];
        Value * processed = mFullyProcessedItemCount[c.Port];
        // To support the lookbehind attribute, we need to withhold the items from
        // our consumed count and rely on the initial buffer underflow to access any
        // items before the start of the physical buffer.
        const Binding & input = getBinding(StreamPort{PortType::Input, c.Port});
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::LookBehind))) {
            const auto & lookBehind = input.findAttribute(AttrId::LookBehind);
            ConstantInt * const amount = b->getSize(lookBehind.amount());
            ConstantInt * const ZERO = b->getSize(0);
            Value * const safe = b->CreateICmpULT(processed, amount);
            processed = b->CreateSelect(safe, b->CreateSub(processed, amount), ZERO);
        }
        const auto buffer = source(e, mConsumerGraph);
        ConsumerNode & cn = mConsumerGraph[buffer]; assert (cn.Consumed);
        if (LLVM_UNLIKELY(mTrackIndividualConsumedItemCounts)) {
            const ConsumerEdge & c = mConsumerGraph[e]; assert (c.Index > 0);
            setConsumedItemCount(b, buffer, processed, c.Index);
        }
        cn.Consumed = b->CreateUMin(cn.Consumed, processed);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeFinalConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeFinalConsumedItemCounts(BuilderRef b) {

    for (const auto e : make_iterator_range(in_edges(mKernelIndex, mConsumerGraph))) {
        const auto buffer = source(e, mConsumerGraph);
        //if (LLVM_UNLIKELY(mConsumerGraph[e] != FAKE_CONSUMER)) {
        ConsumerNode & cn = mConsumerGraph[buffer];
        if (LLVM_LIKELY(cn.PhiNode != nullptr)) {
            cn.PhiNode->addIncoming(cn.Consumed, mKernelLoopExitPhiCatch);
            cn.Consumed = cn.PhiNode;
            cn.PhiNode = nullptr;
        }
        // check to see if we've fully finished processing any stream
        if (out_degree(buffer, mConsumerGraph) == cn.Encountered) {
            setConsumedItemCount(b, buffer, cn.Consumed, 0);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::setConsumedItemCount(BuilderRef b, const unsigned buffer, not_null<Value *> consumed, const unsigned slot) const {
    const auto pe = in_edge(buffer, mBufferGraph);
    const auto producerVertex = source(pe, mBufferGraph);
    const BufferRateData & rd = mBufferGraph[pe];
    const auto prefix = makeBufferName(producerVertex, rd.Port);
    b->setKernel(mPipelineKernel);
    Value * ptr = b->getScalarFieldPtr(prefix + CONSUMED_ITEM_COUNT_SUFFIX);
    b->setKernel(mKernel);
    if (LLVM_UNLIKELY(mTrackIndividualConsumedItemCounts)) {
        ptr = b->CreateGEP(ptr, { b->getInt32(0), b->getInt32(slot) });
    }
    b->CreateStore(consumed, ptr);
}

}

#endif // CONSUMER_LOGIC_HPP
