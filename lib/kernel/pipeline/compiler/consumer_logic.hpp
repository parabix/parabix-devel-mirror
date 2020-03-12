#ifndef CONSUMER_LOGIC_HPP
#define CONSUMER_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addConsumerKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addConsumerKernelProperties(BuilderRef b, const unsigned producer) {
    IntegerType * const sizeTy = b->getSizeTy();
    for (const auto e : make_iterator_range(out_edges(producer, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        // If the out-degree for this buffer is zero, then we've proven that its consumption rate
        // is identical to its production rate.
        const auto numOfIndependentConsumers = out_degree(streamSet, mConsumerGraph);
        if (LLVM_UNLIKELY(numOfIndependentConsumers == 0 && producer != PipelineInput)) {
            continue;
        }
        const BufferNode & bn = mBufferGraph[streamSet];
        const BufferRateData & rd = mBufferGraph[e];
        assert (rd.Port.Type == PortType::Output);
        const auto name = makeBufferName(producer, rd.Port) + CONSUMED_ITEM_COUNT_SUFFIX;

        // If we're tracing the consumer item counts, we need to store one for each
        // (non-nested) consumer. Any nested consumers will have their own trace.
        Type * countTy = sizeTy;
        if (LLVM_UNLIKELY(mTraceIndividualConsumedItemCounts)) {
            countTy = ArrayType::get(sizeTy, numOfIndependentConsumers + 1);
        }
        if (LLVM_LIKELY(bn.isOwned() || bn.isInternal())) {
            mTarget->addInternalScalar(countTy, name);
        } else {
            mTarget->addNonPersistentScalar(countTy, name);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeConsumedItemCount(BuilderRef b, const StreamSetPort outputPort, Value * const produced) {
    Value * initiallyConsumed = produced;
    const Binding & binding = getOutputBinding(outputPort);
    if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::LookBehind))) {
        const Attribute & attr = binding.findAttribute(AttrId::LookBehind);
        Constant * const lookBehind = b->getSize(attr.amount());
        Value * consumed = b->CreateSub(initiallyConsumed, lookBehind);
        Value * const satisfies = b->CreateICmpUGT(initiallyConsumed, lookBehind);
        initiallyConsumed = b->CreateSelect(satisfies, consumed, b->getSize(0));
    }
    const auto streamSet = getOutputBufferVertex(outputPort);
    const ConsumerNode & cn = mConsumerGraph[streamSet];
    cn.Consumed = initiallyConsumed;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::readConsumedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(mKernelId, mConsumerGraph))) {
        const auto streamSet = target(e, mConsumerGraph);
        Value * consumed = readConsumedItemCount(b, streamSet);
        mConsumedItemCount[streamSet] = consumed; assert (consumed);
        #ifdef PRINT_DEBUG_MESSAGES
        const ConsumerEdge & c = mConsumerGraph[e];
        const StreamSetPort port{PortType::Output, c.Port};
        const auto prefix = makeBufferName(mKernelId, port);
        debugPrint(b, prefix + "_consumed = %" PRIu64, consumed);
        #endif
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::readConsumedItemCount(BuilderRef b, const size_t streamSet) {
    const auto n = out_degree(streamSet, mConsumerGraph);
    assert (n > 0);
    if (LLVM_UNLIKELY(n == 1)) {
        const auto e = out_edge(streamSet, mConsumerGraph);
        const ConsumerEdge & c = mConsumerGraph[e];
        if ((c.Flags & ConsumerEdge::UpdatePhi) == 0) {
            // This stream either has no consumers or we've proven that
            // its consumption rate is identical to its production rate.
            return mInitiallyProducedItemCount[streamSet];
        }
    }

    const auto e = in_edge(streamSet, mConsumerGraph);
    const ConsumerEdge & c = mConsumerGraph[e];
    const auto producer = source(e, mConsumerGraph);
    const StreamSetPort port{PortType::Output, c.Port};
    const auto prefix = makeBufferName(producer, port);
    Value * ptr = b->getScalarFieldPtr(prefix + CONSUMED_ITEM_COUNT_SUFFIX);
    if (LLVM_UNLIKELY(mTraceIndividualConsumedItemCounts)) {
        Constant * const ZERO = b->getInt32(0);
        ptr = b->CreateInBoundsGEP(ptr, { ZERO, ZERO } );
    }
    return b->CreateLoad(ptr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createConsumedPhiNodes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::createConsumedPhiNodes(BuilderRef b) {
    IntegerType * const sizeTy = b->getSizeTy();
    for (const auto e : make_iterator_range(in_edges(mKernelId, mConsumerGraph))) {
        const ConsumerEdge & c = mConsumerGraph[e];
        if (c.Flags & ConsumerEdge::UpdatePhi) {
            const auto streamSet = source(e, mConsumerGraph);
            const ConsumerNode & cn = mConsumerGraph[streamSet];
            if (LLVM_LIKELY(cn.PhiNode == nullptr)) {
                const ConsumerEdge & c = mConsumerGraph[e];
                const StreamSetPort port(PortType::Input, c.Port);
                const auto prefix = makeBufferName(mKernelId, port);
                PHINode * const consumedPhi = b->CreatePHI(sizeTy, 2, prefix + "_consumed");
                assert (cn.Consumed);
//                if (mIsPartitionRoot) {
//                    consumedPhi->addIncoming(cn.Consumed, mKernelInitiallyTerminatedPhiCatch);
//                }
                cn.PhiNode = consumedPhi;
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeMinimumConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::computeMinimumConsumedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(mKernelId, mConsumerGraph))) {
        const ConsumerEdge & c = mConsumerGraph[e];
        if (c.Flags & ConsumerEdge::UpdatePhi) {
            const StreamSetPort port(PortType::Input, c.Port);
            Value * processed = mFullyProcessedItemCount(port);
            // To support the lookbehind attribute, we need to withhold the items from
            // our consumed count and rely on the initial buffer underflow to access any
            // items before the start of the physical buffer.
            const Binding & input = getBinding(port);
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::LookBehind))) {
                const auto & lookBehind = input.findAttribute(AttrId::LookBehind);
                ConstantInt * const amount = b->getSize(lookBehind.amount());
                processed = b->CreateSaturatingSub(processed, amount);
            }
            const auto streamSet = source(e, mConsumerGraph);
            assert (streamSet >= FirstStreamSet && streamSet <= LastStreamSet);
            const ConsumerNode & cn = mConsumerGraph[streamSet]; assert (cn.Consumed);
            if (LLVM_UNLIKELY(mTraceIndividualConsumedItemCounts)) {
                const ConsumerEdge & c = mConsumerGraph[e]; assert (c.Index > 0);
                setConsumedItemCount(b, streamSet, processed, c.Index);
            }
            assert (cn.Consumed);
            cn.Consumed = b->CreateUMin(cn.Consumed, processed);

        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeFinalConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeFinalConsumedItemCounts(BuilderRef b) {

    for (const auto e : make_iterator_range(in_edges(mKernelId, mConsumerGraph))) {
        const ConsumerEdge & c = mConsumerGraph[e];
        if (c.Flags & ConsumerEdge::UpdatePhi) {
            const auto streamSet = source(e, mConsumerGraph);
            const ConsumerNode & cn = mConsumerGraph[streamSet];
            if (LLVM_LIKELY(cn.PhiNode != nullptr)) {
                cn.PhiNode->addIncoming(cn.Consumed, mKernelLoopExitPhiCatch);
                cn.Consumed = cn.PhiNode;
                cn.PhiNode = nullptr;
            }
            // check to see if we've fully finished processing any stream
            if (c.Flags & ConsumerEdge::WriteFinalCount) {
                #ifdef PRINT_DEBUG_MESSAGES
                const auto output = in_edge(streamSet, mBufferGraph);
                const BufferRateData & br = mBufferGraph[output];
                const auto producer = source(output, mBufferGraph);
                const auto prefix = makeBufferName(producer, br.Port);
                debugPrint(b, " * writing " + prefix + "_consumed = %" PRIu64, cn.Consumed);
                #endif
                setConsumedItemCount(b, streamSet, cn.Consumed, 0);
            }

        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::setConsumedItemCount(BuilderRef b, const size_t streamSet, not_null<Value *> consumed, const unsigned slot) const {
    const auto pe = in_edge(streamSet, mBufferGraph);
    const auto producer = source(pe, mBufferGraph);
    const BufferRateData & rd = mBufferGraph[pe];
    const auto prefix = makeBufferName(producer, rd.Port);
    Value * ptr = b->getScalarFieldPtr(prefix + CONSUMED_ITEM_COUNT_SUFFIX);
    if (LLVM_UNLIKELY(mTraceIndividualConsumedItemCounts)) {
        ptr = b->CreateInBoundsGEP(ptr, { b->getInt32(0), b->getInt32(slot) });
    }
    if (LLVM_UNLIKELY(mCheckAssertions)) {
        Value * const prior = b->CreateLoad(ptr);
        const Binding & output = rd.Binding;
        // TODO: cross reference which slot the traced count is for?
        b->CreateAssert(b->CreateICmpULE(prior, consumed),
                        "%s.%s: consumed item count is not monotonically nondecreasing "
                        "(prior %" PRIu64 " > current %" PRIu64 ")",
                        mKernelAssertionName,
                        b->GetString(output.getName()),
                        prior, consumed);
    }
    b->CreateStore(consumed, ptr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readExternalConsumerItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::readExternalConsumerItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        const auto buffer = source(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[buffer];
        if (LLVM_LIKELY(bn.isOwned())) {
            const auto p = in_edge(buffer, mBufferGraph);
            const BufferRateData & rd = mBufferGraph[p];
            Value * const consumed = getConsumedOutputItems(rd.Port.Number); assert (consumed);
            const auto producer = source(p, mBufferGraph);
            const auto name = makeBufferName(producer, rd.Port);
            b->setScalarField(name + CONSUMED_ITEM_COUNT_SUFFIX, consumed);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief reportExternalConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeExternalConsumedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferRateData & rd = mBufferGraph[e];
        Value * const ptr = getProcessedInputItemsPtr(rd.Port.Number);
        const ConsumerNode & cn = mConsumerGraph[streamSet];
        b->CreateStore(cn.Consumed, ptr);
    }
}

}

#endif // CONSUMER_LOGIC_HPP
