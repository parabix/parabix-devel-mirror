#ifndef CONSUMER_LOGIC_HPP
#define CONSUMER_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addConsumerKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addConsumerKernelProperties(BuilderRef b, const unsigned producer) {   
    //if (producer != PipelineInput || mTraceIndividualConsumedItemCounts) {

        IntegerType * const sizeTy = b->getSizeTy();

        for (const auto e : make_iterator_range(out_edges(producer, mBufferGraph))) {
            const auto streamSet = target(e, mBufferGraph);

            // If we have a buffer with only external consumers, we do not need to maintain the
            // state for it.

            bool atLeastOneInternalConsumer = false;
            for (const auto e : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
                const auto consumer = target(e, mBufferGraph);
                if (consumer != PipelineOutput) {
                    atLeastOneInternalConsumer = true;
                    break;
                }
            }

            if (LLVM_LIKELY(atLeastOneInternalConsumer)) {

                // If the out-degree for this buffer is zero, then we've proven that its consumption rate
                // is identical to its production rate.
                const auto numOfIndependentConsumers = out_degree(streamSet, mConsumerGraph);
                assert (numOfIndependentConsumers <= out_degree(streamSet, mBufferGraph));

                const BufferNode & bn = mBufferGraph[streamSet];
                if (LLVM_UNLIKELY(numOfIndependentConsumers != 0 || bn.isExternal())) {

                    if (LLVM_LIKELY(bn.isOwned() || bn.isInternal() || mTraceIndividualConsumedItemCounts)) {
                        // If we're tracing the consumer item counts, we need to store one for each
                        // (non-nested) consumer. Any nested consumers will have their own trace.
                        Type * countTy = sizeTy;
                        if (LLVM_UNLIKELY(mTraceIndividualConsumedItemCounts)) {
                            countTy = ArrayType::get(sizeTy, numOfIndependentConsumers + 1);
                        }

                        const BufferPort & rd = mBufferGraph[e];
                        assert (rd.Port.Type == PortType::Output);
                        const auto prefix = makeBufferName(producer, rd.Port);
                        if (numOfIndependentConsumers > 0 && atLeastOneInternalConsumer) {
                            mTarget->addInternalScalar(countTy, prefix + CONSUMED_ITEM_COUNT_SUFFIX, producer);
                        } else {
                            mTarget->addNonPersistentScalar(countTy, prefix + CONSUMED_ITEM_COUNT_SUFFIX);
                        }

                    }
                }

            }

        }
    //}
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
 * @brief readExternalConsumerItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::readExternalConsumerItemCounts(BuilderRef b) {
//    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
//        const auto streamSet = source(e, mBufferGraph);
//        const BufferNode & bn = mBufferGraph[streamSet];
//        if (LLVM_LIKELY(bn.isOwned() || bn.isShared())) {
//            const BufferPort & externalPort = mBufferGraph[e];
//            Value * const consumed = getConsumedOutputItems(externalPort.Port.Number); assert (consumed);



//            mInitialConsumedItemCount[streamSet] = consumed;
//            const auto numOfIndependentConsumers = out_degree(streamSet, mConsumerGraph);
//            const auto producer = parent(streamSet, mBufferGraph);
//            if (LLVM_UNLIKELY((numOfIndependentConsumers != 0) || (producer == PipelineInput))) {
//                setConsumedItemCount(b, streamSet, consumed, 0);
//            }
//        }
//    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::readConsumedItemCount(BuilderRef b, const size_t streamSet, const bool useFinalCount) {

    Value * consumed = nullptr;

    const BufferNode & bn = mBufferGraph[streamSet];

    if (out_degree(streamSet, mConsumerGraph) == 0) {

        if (LLVM_LIKELY(bn.isInternal())) {

            // This stream either has no consumers or we've proven that
            // its consumption rate is identical to its production rate.
            if (useFinalCount) {
                consumed = mLocallyAvailableItems[streamSet];
            } else {
                consumed = mInitiallyProducedItemCount[streamSet];
            }
            const auto e = in_edge(streamSet, mBufferGraph);
            const BufferPort & port = mBufferGraph[e];
            if (LLVM_UNLIKELY(consumed == nullptr)) {
                const auto producer = source(e, mBufferGraph);
                const auto prefix = makeBufferName(producer, port.Port);
                if (LLVM_UNLIKELY(port.IsDeferred)) {
                    consumed = b->getScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX);
                } else {
                    consumed = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
                }
            }
            auto delayOrLookBehind = std::max(port.Delay, port.LookBehind);
            for (const auto e : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
                const BufferPort & br = mBufferGraph[e];
                const auto d = std::max(br.Delay, br.LookBehind);
                delayOrLookBehind = std::max(delayOrLookBehind, d);
            }
            if (delayOrLookBehind) {
                consumed = b->CreateSaturatingSub(consumed, b->getSize(delayOrLookBehind));
            }

        }


    } else {

        const auto e = in_edge(streamSet, mConsumerGraph);
        const ConsumerEdge & c = mConsumerGraph[e];
        const auto producer = source(e, mConsumerGraph);
        Value * consumedPtr = nullptr;
        if (LLVM_LIKELY(producer != PipelineInput || mTraceIndividualConsumedItemCounts)) {
            const StreamSetPort port{PortType::Output, c.Port};
            const auto prefix = makeBufferName(producer, port);
            consumedPtr = b->getScalarFieldPtr(prefix + CONSUMED_ITEM_COUNT_SUFFIX);
            if (LLVM_UNLIKELY(mTraceIndividualConsumedItemCounts)) {
                Constant * const ZERO = b->getInt32(0);
                consumedPtr = b->CreateInBoundsGEP(consumedPtr, { ZERO, ZERO } );
            }
        } else {
            consumedPtr = getProcessedInputItemsPtr(c.Port);
        }
        consumed = b->CreateLoad(consumedPtr);
    }

    //if (LLVM_UNLIKELY(bn.isExternal())) {

        bool foundAny = false;

        for (const auto e : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
            if (target(e, mBufferGraph) == PipelineOutput) {
                const BufferPort & externalPort = mBufferGraph[e];
                Value * const external = getConsumedOutputItems(externalPort.Port.Number); assert (external);

                const Binding & binding = externalPort.Binding;
                //b->CallPrintInt(binding.getName() + "_externalConsumed", external);

                consumed = b->CreateUMin(consumed, external);
                foundAny = true;
                // break;
            }
        }

        assert (foundAny ^ bn.isInternal());

    //}

    return consumed;
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
            debugPrint(b, "* update " + consPrefix + " -> " + prodPrefix + "_consumed' = %" PRIu64, cn.Consumed);
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
        if (c.Flags) {
            const auto streamSet = source(e, mConsumerGraph);
            const ConsumerNode & cn = mConsumerGraph[streamSet];
            if (c.Flags & ConsumerEdge::UpdatePhi) {
                if (LLVM_LIKELY(cn.PhiNode != nullptr)) {
                    cn.PhiNode->addIncoming(cn.Consumed, mKernelLoopExitPhiCatch);
                    cn.Consumed = cn.PhiNode;
                    cn.PhiNode = nullptr;
                }
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
    const auto output = in_edge(streamSet, mBufferGraph);
    const auto producer = source(output, mBufferGraph);
    const BufferPort & outputPort = mBufferGraph[output];
    Value * ptr = nullptr;
    if (LLVM_LIKELY(producer != PipelineInput || slot != 0 || mTraceIndividualConsumedItemCounts)) {
        const auto prefix = makeBufferName(producer, outputPort.Port);
        ptr = b->getScalarFieldPtr(prefix + CONSUMED_ITEM_COUNT_SUFFIX);
        if (LLVM_UNLIKELY(mTraceIndividualConsumedItemCounts)) {
            ptr = b->CreateInBoundsGEP(ptr, { b->getInt32(0), b->getInt32(slot) });
        }
        if (LLVM_UNLIKELY(CheckAssertions)) {
            Value * const prior = b->CreateLoad(ptr);
            const Binding & output = outputPort.Binding;
            // TODO: cross reference which slot the traced count is for?

            Constant * const bindingName = b->GetString(output.getName());

            assert (mCurrentKernelName);

            b->CreateAssert(b->CreateICmpULE(prior, consumed),
                            "%s.%s: consumed item count is not monotonically nondecreasing "
                            "(prior %" PRIu64 " > current %" PRIu64 ")",
                            mCurrentKernelName, bindingName,
                            prior, consumed);

            const BufferNode & bn = mBufferGraph[streamSet];
            Value * const produced = mLocallyAvailableItems[streamSet]; assert (produced);
            if (bn.NonLocal) {
                Value * const consumedLessThanProduced = b->CreateICmpULE(consumed, produced);
                Constant * const none = getTerminationSignal(b, TerminationSignal::None);
                Value * const terminated = b->CreateICmpNE(mTerminatedAtLoopExitPhi, none);
                Value * const valid = b->CreateOr(consumedLessThanProduced, terminated);


                b->CreateAssert(valid,
                                "%s.%s: consumed item count (%" PRId64 ") exceeds "
                                "produced item count (%" PRId64 ")",
                                mCurrentKernelName, bindingName,
                                consumed, produced);

            } else {
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
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializePipelineInputConsumedPhiNodes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializePipelineInputConsumedPhiNodes(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferPort & br = mBufferGraph[e];
        const auto portNum = br.Port.Number;
        Value * const avail = getAvailableInputItems(portNum);
        mInitialConsumedItemCount[streamSet] = avail;
        // If we have an unused external input, set the value immediately.
        if (out_degree(streamSet, mBufferGraph) == 0) {
            Value * const externalPtr = getProcessedInputItemsPtr(portNum);
            b->CreateStore(avail, externalPtr);
        }
    }
}

}

#endif // CONSUMER_LOGIC_HPP
