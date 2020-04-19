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
        if ((numOfIndependentConsumers > 0) || (producer == PipelineInput)) {
            const BufferNode & bn = mBufferGraph[streamSet];
            const BufferRateData & rd = mBufferGraph[e];
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

//        if (CheckAssertions) {
//            const auto prefix = std::to_string(streamSet);
//            const auto name = prefix + DEBUG_CONSUMED_ITEM_COUNT_SUFFIX;
//            mTarget->addNonPersistentScalar(sizeTy, name);
//        }
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
//        const auto e = in_edge(streamSet, mBufferGraph);
//        const BufferRateData & br = mBufferGraph[e];
//        auto delayOrLookBehind = std::max(br.Delay, br.LookBehind);
//        for (const auto e : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
//            const BufferRateData & br = mBufferGraph[e];
//            const auto d = std::max(br.Delay, br.LookBehind);
//            delayOrLookBehind = std::max(delayOrLookBehind, d);
//        }
//        if (delayOrLookBehind) {
//            produced = b->CreateSaturatingSub(produced, b->getSize(delayOrLookBehind));
//        }
        return produced;
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
 * @brief initializeConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeConsumedItemCount(BuilderRef b, const StreamSetPort outputPort, Value * const produced) {
    Value * initiallyConsumed = produced;
    const auto output = getOutput(mKernelId, outputPort);
    const BufferRateData & br = mBufferGraph[output];
    if (br.LookBehind) {
        Constant * const lookBehind = b->getSize(br.LookBehind);
        Value * consumed = b->CreateSub(initiallyConsumed, lookBehind);
        Value * const satisfies = b->CreateICmpUGT(initiallyConsumed, lookBehind);
        initiallyConsumed = b->CreateSelect(satisfies, consumed, b->getSize(0));
    }
    const auto streamSet = target(output, mBufferGraph);
    const ConsumerNode & cn = mConsumerGraph[streamSet];
    cn.Consumed = initiallyConsumed;

//    if (CheckAssertions) {
//        const auto prefix = std::to_string(streamSet);
//        const auto name = prefix + DEBUG_CONSUMED_ITEM_COUNT_SUFFIX;
//        Value * const ptr = b->getScalarFieldPtr(name);
//        #ifdef PRINT_DEBUG_MESSAGES
//        const auto output = in_edge(streamSet, mBufferGraph);
//        const auto producer = source(output, mBufferGraph);
//        const auto prefix2 = makeBufferName(producer, mBufferGraph[output].Port);
//        debugPrint(b, prefix2 + "_expected_consumed = %" PRIu64 " (%" PRIx64 ")", initiallyConsumed, ptr);
//        #endif
//        b->CreateStore(initiallyConsumed, ptr);
//    }

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
            const auto input = getInput(mKernelId, port);
            const BufferRateData & br = mBufferGraph[input];
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
            cn.Consumed = b->CreateUMin(cn.Consumed, processed);
        }
    }

//    // When checking assertions, keep an absolute running tally of how many items
//    // are actually consumed to verify the final number.
//    if (CheckAssertions) {
//        for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
//            const BufferRateData & br = mBufferGraph[e];
//            const auto streamSet = source(e, mBufferGraph);
//            const auto prefix = std::to_string(streamSet);
//            const auto name = prefix + DEBUG_CONSUMED_ITEM_COUNT_SUFFIX;
//            Value * ptr = b->getScalarFieldPtr(prefix + DEBUG_CONSUMED_ITEM_COUNT_SUFFIX);
//            Value * current = b->CreateLoad(ptr);
//            Value * processed = mFullyProcessedItemCount(br.Port);
//            if (LLVM_UNLIKELY(br.LookBehind != 0)) {
//                ConstantInt * const amount = b->getSize(br.LookBehind);
//                processed = b->CreateSaturatingSub(processed, amount);
//            }
//            current = b->CreateUMin(current, processed);
//            #ifdef PRINT_DEBUG_MESSAGES
//            const auto output = in_edge(streamSet, mBufferGraph);
//            const auto producer = source(output, mBufferGraph);
//            const auto prefix2 = makeBufferName(producer, mBufferGraph[output].Port);
//            debugPrint(b, prefix2 + "_expected_consumed' = %" PRIu64 " (%" PRIx64 ")", current, ptr);
//            #endif
//            b->CreateStore(current, ptr);
//        }
//    }

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
                const BufferRateData & br = mBufferGraph[output];
                const auto producer = source(output, mBufferGraph);
                const auto prefix = makeBufferName(producer, br.Port);
                debugPrint(b, " * writing " + prefix + "_consumed = %" PRIu64, cn.Consumed);
                #endif
                setConsumedItemCount(b, streamSet, cn.Consumed, 0);
            }

        }
    }

//    // When checking assertions, keep an absolute running tally of how many items
//    // are actually consumed to verify the final number.
//    if (CheckAssertions) {
//        flat_set<unsigned> inputStreamSets;
//        inputStreamSets.reserve(in_degree(mKernelId, mBufferGraph));

//        for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
//            const auto streamSet = source(e, mBufferGraph);
//            auto lastConsumer = mKernelId;
//            for (const auto e : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
//                const auto consumer = target(e, mBufferGraph);
//                lastConsumer = std::max<unsigned>(lastConsumer, consumer);
//            }
//            if (lastConsumer == mKernelId) {
//                inputStreamSets.insert(streamSet);
//            }
//        }

//        const auto msg = "Expected to consume %" PRIu64 " items of %s.%s but instead consumed %" PRIu64;

//        for (const auto streamSet : inputStreamSets) {
//            const auto prefix = std::to_string(streamSet);
//            Value * const current = b->getScalarField(prefix + DEBUG_CONSUMED_ITEM_COUNT_SUFFIX);
//            Value * const consumed = readConsumedItemCount(b, streamSet, true);
//            Value * const valid = b->CreateICmpEQ(current, consumed);
//            const auto output = in_edge(streamSet, mBufferGraph);
//            const auto producer = source(output, mBufferGraph);
//            const BufferRateData & br = mBufferGraph[output];
//            const Binding & binding = br.Binding;
//            b->CreateAssert(valid, msg, current, mKernelName[producer], b->GetString(binding.getName()), consumed);
//        }

//    }

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
    if (LLVM_UNLIKELY(CheckAssertions)) {
        Value * const prior = b->CreateLoad(ptr);
        const Binding & output = rd.Binding;
        // TODO: cross reference which slot the traced count is for?

        Constant * const bindingName = b->GetString(output.getName());

        b->CreateAssert(b->CreateICmpULE(prior, consumed),
                        "%s.%s: consumed item count is not monotonically nondecreasing "
                        "(prior %" PRIu64 " > current %" PRIu64 ")",
                        mCurrentKernelName, bindingName,
                        prior, consumed);

        const BufferNode & bn = mBufferGraph[streamSet];
        if (!bn.NonLocal) {
            Value * const produced = mLocallyAvailableItems[streamSet];
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



    b->CreateStore(consumed, ptr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readExternalConsumerItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::readExternalConsumerItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        const auto streamSet = source(e, mBufferGraph);



        const BufferNode & bn = mBufferGraph[streamSet];
        if (LLVM_LIKELY(bn.isOwned())) {
            const BufferRateData & externalPort = mBufferGraph[e];
            Value * const consumed = getConsumedOutputItems(externalPort.Port.Number); assert (consumed);

            const auto p = in_edge(streamSet, mBufferGraph);
            const auto producer = source(p, mBufferGraph);

            const auto numOfIndependentConsumers = out_degree(streamSet, mConsumerGraph);
            if (LLVM_UNLIKELY((numOfIndependentConsumers == 0) && (producer != PipelineInput))) {
                continue;
            }

            const BufferRateData & internalPort = mBufferGraph[p];
            const auto name = makeBufferName(producer, internalPort.Port);
            b->setScalarField(name + CONSUMED_ITEM_COUNT_SUFFIX, consumed);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializePipelineInputConsumedPhiNodes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializePipelineInputConsumedPhiNodes(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferRateData & br = mBufferGraph[e];
        const auto prefix = makeBufferName(PipelineInput, br.Port);
        PHINode * const phi = b->CreatePHI(b->getSizeTy(), 2, prefix + "_consumedItemCount");
        mExternalConsumedItemsPhi[streamSet] = phi;
        mInitialConsumedItemCount[streamSet] = getAvailableInputItems(br.Port.Number);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updatePipelineInputConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::updatePipelineInputConsumedItemCounts(BuilderRef /* b */, BasicBlock * const exit) {
    for (const auto e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        PHINode * const phi = mExternalConsumedItemsPhi[streamSet]; assert (phi);
        Value * const consumed = mInitialConsumedItemCount[streamSet]; assert (consumed);
        phi->addIncoming(consumed, exit);
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
        Value * const consumed = mExternalConsumedItemsPhi[streamSet]; assert (consumed);
        b->CreateStore(consumed, ptr);
    }
}

}

#endif // CONSUMER_LOGIC_HPP
