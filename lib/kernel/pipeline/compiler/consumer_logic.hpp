#ifndef CONSUMER_LOGIC_HPP
#define CONSUMER_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addConsumerKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addConsumerKernelProperties(BuilderRef b, const unsigned producer) {   
    if (producer != PipelineInput || mTraceIndividualConsumedItemCounts) {

        IntegerType * const sizeTy = b->getSizeTy();

        for (const auto e : make_iterator_range(out_edges(producer, mBufferGraph))) {
            const auto streamSet = target(e, mBufferGraph);
            const BufferNode & bn = mBufferGraph[streamSet];
            // If the out-degree for this buffer is zero, then we've proven that its consumption rate
            // is identical to its production rate.
            const auto numOfIndependentConsumers = out_degree(streamSet, mConsumerGraph);
            if (LLVM_UNLIKELY(numOfIndependentConsumers != 0)) {
                const BufferPort & rd = mBufferGraph[e];
                assert (rd.Port.Type == PortType::Output);
                const auto prefix = makeBufferName(producer, rd.Port);
                const auto name = prefix + CONSUMED_ITEM_COUNT_SUFFIX;

                // If we're tracing the consumer item counts, we need to store one for each
                // (non-nested) consumer. Any nested consumers will have their own trace.
                Type * countTy = sizeTy;
                if (LLVM_UNLIKELY(mTraceIndividualConsumedItemCounts)) {
                    countTy = ArrayType::get(sizeTy, numOfIndependentConsumers + 1);
                }
                if (LLVM_LIKELY(bn.isOwned() || bn.isInternal() || mTraceIndividualConsumedItemCounts)) {
                    mTarget->addInternalScalar(countTy, name, producer);
                } else {
                    mTarget->addNonPersistentScalar(countTy, name);
                }
            }
        }
    }


}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::readConsumedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(mKernelId, mConsumerGraph))) {
        const auto streamSet = target(e, mConsumerGraph);
        Value * consumed = readConsumedItemCount(b, streamSet);
        mInitialConsumedItemCount[streamSet] = consumed; assert (consumed);
        #ifdef PRINT_DEBUG_MESSAGES
        const ConsumerEdge & c = mConsumerGraph[e];
        const StreamSetPort port{PortType::Output, c.Port};
        const auto prefix = makeBufferName(mKernelId, port);
        debugPrint(b, prefix + "_consumed = %" PRIu64, consumed);
        #endif
        if (LLVM_UNLIKELY(CheckAssertions)) {
            Value * const produced = mInitiallyProducedItemCount[streamSet];
            Value * const valid = b->CreateOr(b->CreateICmpULE(consumed, produced), mInitiallyTerminated);
            constexpr auto msg =
                "Consumed item count (%" PRId64 ") of %s.%s exceeded its produced item count (%" PRId64 ").";
            const ConsumerEdge & c = mConsumerGraph[e];
            const StreamSetPort port{PortType::Output, c.Port};
            Constant * const bindingName = b->GetString(getBinding(mKernelId, port).getName());
            b->CreateAssert(valid, msg,
                consumed, mCurrentKernelName, bindingName, produced);
        }
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readExternalConsumerItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::readExternalConsumerItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        const auto streamSet = source(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[streamSet];
        if (LLVM_LIKELY(bn.isOwned())) {
            const BufferPort & externalPort = mBufferGraph[e];
            Value * const consumed = getConsumedOutputItems(externalPort.Port.Number); assert (consumed);
            const auto numOfIndependentConsumers = out_degree(streamSet, mConsumerGraph);
            const auto producer = parent(streamSet, mBufferGraph);
            if (LLVM_UNLIKELY((numOfIndependentConsumers != 0) || (producer == PipelineInput))) {
                setConsumedItemCount(b, streamSet, consumed, 0);
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::readConsumedItemCount(BuilderRef b, const size_t streamSet, const bool useFinalCount) {

    if (out_degree(streamSet, mConsumerGraph) == 0) {
        // This stream either has no consumers or we've proven that
        // its consumption rate is identical to its production rate.
        Value * produced = nullptr;
        if (useFinalCount) {
            produced = mLocallyAvailableItems[streamSet];
        } else {
            produced = mInitiallyProducedItemCount[streamSet];
        }
        const auto e = in_edge(streamSet, mBufferGraph);
        const BufferPort & port = mBufferGraph[e];
        if (LLVM_UNLIKELY(produced == nullptr)) {
            const auto producer = source(e, mBufferGraph);
            const auto prefix = makeBufferName(producer, port.Port);
            if (LLVM_UNLIKELY(port.IsDeferred)) {
                produced = b->getScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX);
            } else {
                produced = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
            }
        }
        auto delayOrLookBehind = std::max(port.Delay, port.LookBehind);
        for (const auto e : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
            const BufferPort & br = mBufferGraph[e];
            const auto d = std::max(br.Delay, br.LookBehind);
            delayOrLookBehind = std::max(delayOrLookBehind, d);
        }
        if (delayOrLookBehind) {
            produced = b->CreateSaturatingSub(produced, b->getSize(delayOrLookBehind));
        }
        return produced;
    }

    const auto e = in_edge(streamSet, mConsumerGraph);
    const ConsumerEdge & c = mConsumerGraph[e];
    const auto producer = source(e, mConsumerGraph);
    if (LLVM_LIKELY(producer != PipelineInput || mTraceIndividualConsumedItemCounts)) {

        const StreamSetPort port{PortType::Output, c.Port};
        const auto prefix = makeBufferName(producer, port);
        Value * ptr = b->getScalarFieldPtr(prefix + CONSUMED_ITEM_COUNT_SUFFIX);
        if (LLVM_UNLIKELY(mTraceIndividualConsumedItemCounts)) {
            Constant * const ZERO = b->getInt32(0);
            ptr = b->CreateInBoundsGEP(ptr, { ZERO, ZERO } );
        }
        return b->CreateLoad(ptr);


    } else {
        return b->CreateLoad(getProcessedInputItemsPtr(c.Port));
    }


}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeConsumedItemCount(BuilderRef b, const StreamSetPort outputPort, Value * const produced) {
    Value * initiallyConsumed = produced;
    const auto output = getOutput(mKernelId, outputPort);
    const BufferPort & br = mBufferGraph[output];
    if (br.LookBehind || br.Delay) {
        const auto delayOrLookBehind = std::max(br.Delay, br.LookBehind);
        initiallyConsumed = b->CreateSaturatingSub(produced, b->getSize(delayOrLookBehind));
    }
    const auto streamSet = target(output, mBufferGraph);
    const ConsumerNode & cn = mConsumerGraph[streamSet];
    cn.Consumed = initiallyConsumed;


    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelId, outputPort);
    debugPrint(b, prefix + " -> " + prefix + "_initiallyConsumed = %" PRIu64, initiallyConsumed);
    #endif

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
                cn.PhiNode = consumedPhi;
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief phiOutConsumedItemCountsAfterInitiallyTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::phiOutConsumedItemCountsAfterInitiallyTerminated(BuilderRef /* b */) {
    for (const auto e : make_iterator_range(in_edges(mKernelId, mConsumerGraph))) {
        const ConsumerEdge & c = mConsumerGraph[e];
        if (c.Flags & ConsumerEdge::UpdatePhi) {
            const auto streamSet = source(e, mConsumerGraph);
            const ConsumerNode & cn = mConsumerGraph[streamSet];
            cn.PhiNode->addIncoming(mInitialConsumedItemCount[streamSet], mKernelInitiallyTerminatedExit);
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
            Value * processed = mFullyProcessedItemCount[port];
            // To support the lookbehind attribute, we need to withhold the items from
            // our consumed count and rely on the initial buffer underflow to access any
            // items before the start of the physical buffer.
            const auto input = getInput(mKernelId, port);
            const BufferPort & br = mBufferGraph[input];
            if (LLVM_UNLIKELY(br.LookBehind != 0)) {
                ConstantInt * const amount = b->getSize(br.LookBehind);
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
            const auto output = in_edge(streamSet, mBufferGraph);
            const auto producer = source(output, mBufferGraph);
            const auto prodPrefix = makeBufferName(producer, mBufferGraph[output].Port);

            cn.Consumed = b->CreateUMin(cn.Consumed, processed, prodPrefix + "_minConsumed");

            #ifdef PRINT_DEBUG_MESSAGES
            const auto consPrefix = makeBufferName(mKernelId, port);
            debugPrint(b, consPrefix + " -> " + prodPrefix + "_consumed' = %" PRIu64, cn.Consumed);
            #endif
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeFinalConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeConsumedItemCounts(BuilderRef b) {

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
            if (c.Flags & ConsumerEdge::WriteConsumedCount) {
                #ifdef PRINT_DEBUG_MESSAGES
                const auto output = in_edge(streamSet, mBufferGraph);
                const BufferPort & br = mBufferGraph[output];
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
    const BufferPort & rd = mBufferGraph[pe];
    Value * ptr = nullptr;
    if (LLVM_LIKELY(producer != PipelineInput || mTraceIndividualConsumedItemCounts)) {
        const auto prefix = makeBufferName(producer, rd.Port);
        ptr = b->getScalarFieldPtr(prefix + CONSUMED_ITEM_COUNT_SUFFIX);
        if (LLVM_UNLIKELY(mTraceIndividualConsumedItemCounts)) {
            ptr = b->CreateInBoundsGEP(ptr, { b->getInt32(0), b->getInt32(slot) });
        }
        if (LLVM_UNLIKELY(CheckAssertions)) {
            Value * const prior = b->CreateLoad(ptr);
            const Binding & output = rd.Binding;
            // TODO: cross reference which slot the traced count is for?

            Constant * const bindingName = b->GetString(output.getName());

            assert (mCurrentKernelName);

            b->CreateAssert(b->CreateICmpULE(prior, consumed),
                            "%s.%s: consumed item count is not monotonically nondecreasing "
                            "(prior %" PRIu64 " > current %" PRIu64 ")",
                            mCurrentKernelName, bindingName,
                            prior, consumed);

            const BufferNode & bn = mBufferGraph[streamSet];
            if (bn.Locality == BufferLocality::ThreadLocal) {
                Value * const produced = mLocallyAvailableItems[streamSet]; assert (produced);
                // NOTE: static linear buffers are assumed to be threadlocal.
                Value * const fullyConsumed = b->CreateICmpEQ(produced, consumed);
                Constant * const fatal = getTerminationSignal(b, TerminationSignal::Fatal);
                Value * const fatalError = b->CreateICmpEQ(mTerminatedAtLoopExitPhi, fatal);
                Value * const valid = b->CreateOr(fullyConsumed, fatalError);

                b->CreateAssert(valid,
                                "%s.%s: local available item count (%" PRId64 ") does not match "
                                "its consumed item count (%" PRId64 ")",
                                mCurrentKernelName, bindingName,
                                produced, consumed);
            }

        }
    } else {
        ptr = getProcessedInputItemsPtr(rd.Port.Number);
    }

    b->CreateStore(consumed, ptr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializePipelineInputConsumedPhiNodes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializePipelineInputConsumedPhiNodes(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferPort & br = mBufferGraph[e];
        mInitialConsumedItemCount[streamSet] = getAvailableInputItems(br.Port.Number);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief reportExternalConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeExternalConsumedItemCounts(BuilderRef b) {
//    for (const auto e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
//        const auto streamSet = target(e, mBufferGraph);
//        const BufferPort & rd = mBufferGraph[e];
//        Value * const ptr = getProcessedInputItemsPtr(rd.Port.Number);
//        Value * const consumed = mInitialConsumedItemCount[streamSet]; assert (consumed);
//        b->CreateStore(consumed, ptr);
//    }
}

}

#endif // CONSUMER_LOGIC_HPP
