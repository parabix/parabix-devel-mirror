#ifndef PARTITION_PROCESSING_LOGIC_HPP
#define PARTITION_PROCESSING_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePartitionEntryPoints
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::makePartitionEntryPoints(BuilderRef b) {    
    mPipelineLoop = b->CreateBasicBlock("PipelineLoop");

    // the zeroth partition may be a fake one if this pipeline has I/O
    const auto firstPartition = KernelPartitionId[FirstKernel];
    for (auto i = firstPartition; i < PartitionCount; ++i) {
        mPartitionEntryPoint[i] = b->CreateBasicBlock("Partition" + std::to_string(i));
    }    
    mPipelineEnd = b->CreateBasicBlock("PipelineEnd");
    mPartitionEntryPoint[PartitionCount] = mPipelineEnd;

    const auto ip = b->saveIP();
    IntegerType * const boolTy = b->getInt1Ty();
    IntegerType * const sizeTy = b->getInt64Ty();
    for (auto i = firstPartition + 1U; i < PartitionCount; ++i) {
        b->SetInsertPoint(mPartitionEntryPoint[i]);
        assert (mPartitionEntryPoint[i]->getFirstNonPHI() == nullptr);
        mPartitionPipelineProgressPhi[i] = b->CreatePHI(boolTy, PartitionCount, std::to_string(i) + ".pipelineProgress");
        mExhaustedPipelineInputAtPartitionEntry[i] = b->CreatePHI(boolTy, PartitionCount, std::to_string(i) + ".exhaustedInput");
        if (LLVM_UNLIKELY(EnableCycleCounter)) {
            mPartitionStartTimePhi[i] = b->CreatePHI(sizeTy, PartitionCount, std::to_string(i) + ".startTimeCycleCounter");
        }
    }
    initializePipelineInputConsumedPhiNodes(b);
    b->restoreIP(ip);


}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief branchToInitialPartition
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::branchToInitialPartition(BuilderRef b) {
    const auto firstPartition = KernelPartitionId[FirstKernel];
    BasicBlock * const entry = mPartitionEntryPoint[firstPartition];
    b->CreateBr(entry);

    b->SetInsertPoint(entry);
    mCurrentPartitionId = -1U;
    setActiveKernel(b, FirstKernel, true);

    mKernelStartTime = startCycleCounter(b);
    acquireSynchronizationLock(b, FirstKernel);
    updateInternalPipelineItemCount(b);
    updateCycleCounter(b, FirstKernel, mKernelStartTime, CycleCounter::KERNEL_SYNCHRONIZATION);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPartitionExitPoint
 ** ------------------------------------------------------------------------------------------------------------- */
inline BasicBlock * PipelineCompiler::getPartitionExitPoint(BuilderRef /* b */) {
    assert (mKernelId >= FirstKernel && mKernelId <= PipelineOutput);
    const auto partitionId = KernelPartitionId[mKernelId];
    assert ("partition exit cannot loop to current entry!" &&
            mPartitionEntryPoint[partitionId] != mPartitionEntryPoint[partitionId + 1]);
    return mPartitionEntryPoint[partitionId + 1];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForPartitionEntry
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::checkForPartitionEntry(BuilderRef b) {
    assert (mKernelId >= FirstKernel && mKernelId <= LastKernel);    
    mNextPartitionWithPotentialInput = nullptr;
    mIsPartitionRoot = false;
    const auto partitionId = KernelPartitionId[mKernelId];
    if (partitionId != mCurrentPartitionId) {
        mCurrentPartitionId = partitionId;
        const auto jumpIdx = mPartitionJumpIndex[partitionId];
        assert (partitionId != jumpIdx);
        mNextPartitionWithPotentialInput = mPartitionEntryPoint[jumpIdx];
        mIsPartitionRoot = true;
        identifyPartitionKernelRange();
        determinePartitionStrideRates();
        #ifdef PRINT_DEBUG_MESSAGES
        debugPrint(b, "  *** entering partition %" PRIu64, b->getSize(mCurrentPartitionId));
        #endif
    }
    assert (KernelPartitionId[mKernelId - 1U] <= mCurrentPartitionId);
    assert ((KernelPartitionId[mKernelId - 1U] + 1U) >= mCurrentPartitionId);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyPartitionKernelRange
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::identifyPartitionKernelRange() {
    FirstKernelInPartition = mKernelId;
    for (auto i = mKernelId + 1U; i <= PipelineOutput; ++i) {
        if (KernelPartitionId[i] != mCurrentPartitionId) {
            LastKernelInPartition = i - 1U;
            break;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determinePartitionStrideRates
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::determinePartitionStrideRates() {
    const auto max = MaximumNumOfStrides[FirstKernelInPartition];
    MaxPartitionStrideRate = max;
    for (auto i = FirstKernelInPartition + 1U; i <= LastKernelInPartition; ++i) {
        const auto max = MaximumNumOfStrides[i];
        MaxPartitionStrideRate = std::max<unsigned>(MaxPartitionStrideRate, max);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writePartitionEntryIOGuard
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writePartitionEntryIOGuard(BuilderRef b) {
    b->SetInsertPoint(mCurrentPartitionEntryGuard);
    Value * const processPartition = calculatePartitionSegmentLength(b);
    detemineMaximumNumberOfStrides(b);
    b->CreateLikelyCondBr(processPartition, mKernelLoopEntry, mKernelJumpToNextUsefulPartition);

    BasicBlock * const exitBlock = b->GetInsertBlock();
    if (mExhaustedInputAtJumpPhi) {
        mExhaustedInputAtJumpPhi->addIncoming(mExhaustedInput, exitBlock);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculatePartitionSegmentLength
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::calculatePartitionSegmentLength(BuilderRef b) {

    const auto minFirst = MinimumNumOfStrides[FirstKernelInPartition];
    const auto maxFirst = MaximumNumOfStrides[FirstKernelInPartition];
    assert ((maxFirst % minFirst) == 0);
    const auto maxNumOfStrides = maxFirst / minFirst;

    ConstantInt * const maxPartitionStrides = b->getSize(maxNumOfStrides);

    Value * availPartitionStrides = maxPartitionStrides;

    Value * anyClosed = nullptr;

    for (const auto e : make_iterator_range(in_edges(mCurrentPartitionId, mPartitionIOGraph))) {

        const auto i = source(e, mPartitionIOGraph);
        assert (i >= PartitionCount);
        const auto streamSet = mPartitionIOGraph[i];

        assert (FirstStreamSet <= streamSet && streamSet <= LastStreamSet);

        const PartitionIOData & IO = mPartitionIOGraph[e];

        assert (FirstKernelInPartition <= IO.Kernel && IO.Kernel <= LastKernelInPartition);
        assert (maxNumOfStrides == (MaximumNumOfStrides[IO.Kernel] / MinimumNumOfStrides[IO.Kernel]));

        const BufferPort & port = IO.Port;
        assert (port.Port.Type == PortType::Input);
        Value * const avail = subtractLookahead(b, port, mLocallyAvailableItems[streamSet]);
        const auto prefix = makeBufferName(IO.Kernel, port.Port);
        Value * const processed = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
        Value * const unprocessed = b->CreateSub(avail, processed);

        Value * segmentStrides = nullptr;

        const Binding & binding = port.Binding;
        const ProcessingRate & rate = binding.getRate();
        switch (rate.getKind()) {
            case RateId::Fixed:
            case RateId::Bounded:
                BEGIN_SCOPED_REGION
                Value * const strideLength = calculateStrideLength(b, port);
                ConstantInt * const numOfStrides = b->getSize(MinimumNumOfStrides[IO.Kernel]);
                Value * const segmentLength = b->CreateMul(strideLength, numOfStrides);
                segmentStrides = b->CreateUDiv(unprocessed, segmentLength);
                END_SCOPED_REGION
                break;
            case RateId::PartialSum:
                BEGIN_SCOPED_REGION
                const auto ref = getReference(IO.Kernel, port.Port);
                assert (ref.Type == PortType::Input);
                const StreamSetBuffer * const buffer = getInputBuffer(IO.Kernel, ref);
                Constant * const ZERO = b->getSize(0);
                const auto step = MinimumNumOfStrides[IO.Kernel];
                const auto partialSumPrefix = makeBufferName(IO.Kernel, ref);
                Value * partialSumStrides = ZERO;
                Value * const processed = b->getScalarField(partialSumPrefix + ITEM_COUNT_SUFFIX);
                for (unsigned i = 1; i <= maxNumOfStrides; ++i) {
                    Value * const offset = b->getSize(i * step - 1);
                    Value * const position = b->CreateAdd(processed, offset);
                    Value * const requiredPtr = buffer->getRawItemPointer(b, ZERO, position);
                    Value * const required = b->CreateLoad(requiredPtr);
                    Value * const sufficent = b->CreateZExt(b->CreateICmpUGE(avail, required), b->getSizeTy());
                    partialSumStrides = b->CreateAdd(partialSumStrides, sufficent);
                }
                segmentStrides = partialSumStrides;
                END_SCOPED_REGION
                break;
            case RateId::Greedy:
                BEGIN_SCOPED_REGION
                Value * const required = b->getSize(floor(rate.getLowerBound()));
                Value * const sufficient = b->CreateZExt(b->CreateICmpUGE(unprocessed, required), b->getSizeTy());
                segmentStrides = b->CreateMul(sufficient, maxPartitionStrides);
                END_SCOPED_REGION
                break;
            case RateId::Relative:
                report_fatal_error("Relative rates should not be used to calculate partition segment length");
            default:
                break;
        }

        Value * const closed = isClosed(b, streamSet);
        assert (closed);
        if (anyClosed) {
            anyClosed = b->CreateOr(anyClosed, closed);
        } else {
            anyClosed = closed;
        }

        #ifndef DISABLE_ZERO_EXTEND
        if (LLVM_UNLIKELY(port.IsZeroExtended)) {
            // To zero-extend an input stream, we must first exhaust all input for this stream before
            // switching to a "zeroed buffer". The size of the buffer will be determined by the final
            // number of non-zero-extended strides.

            // NOTE: the producer of this stream will zero out all data after its last produced item
            // that can be read by a single iteration of any consuming kernel.
            segmentStrides = b->CreateSelect(closed, segmentStrides, maxPartitionStrides);
        }
        #endif
        availPartitionStrides = b->CreateUMin(availPartitionStrides, segmentStrides);
    }

    mPartitionSegmentLength = availPartitionStrides;

    b->CallPrintInt("mPartitionSegmentLength", mPartitionSegmentLength);

    if (anyClosed) {
        Value * const hasFullSegment = b->CreateICmpNE(availPartitionStrides, b->getSize(0));
        return b->CreateOr(anyClosed, hasFullSegment);
    } else {
        return b->getTrue();
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadLastGoodVirtualBaseAddressesOfUnownedBuffersInCurrentPartition
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::loadLastGoodVirtualBaseAddressesOfUnownedBuffersInPartition(BuilderRef b) const {
    for (auto i = mKernelId; i <= LastKernel; ++i) {
        if (KernelPartitionId[i] != mCurrentPartitionId) {
            break;
        }
        loadLastGoodVirtualBaseAddressesOfUnownedBuffers(b, i);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadLastGoodVirtualBaseAddressesOfUnownedBuffersInCurrentPartition
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::phiOutPartitionItemCounts(BuilderRef b, const unsigned kernel,
                                                 const unsigned targetPartitionId,
                                                 const bool fromInitiallyTerminated,
                                                 BasicBlock * const entryPoint) {

    BasicBlock * const exitPoint = b->GetInsertBlock();

    struct PhiData {
        unsigned    StreamSet;
        Value *     ItemCount;
        PhiData(unsigned streamSet, Value * itemCount) : StreamSet(streamSet), ItemCount(itemCount) { }
    };

    using PhiDataSet = SmallVector<PhiData, 16>;

    PhiDataSet producedSet;
    PhiDataSet consumedSet;

    for (const auto e : make_iterator_range(out_edges(kernel, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);

        // When jumping out of a partition to some subsequent one, we may have to
        // phi-out some of the produced item counts. We 3 cases to consider:

        // (1) if we've executed the kernel, we use the fully produced item count.
        // (2) if producer is the current kernel, we use the already produced phi node.
        // (3) if we have yet to execute (and will be jumping over) the kernel, load
        // the prior produced count.


        const BufferPort & br = mBufferGraph[e];
        // Select/compute/load the appropriate produced item count
        Value * produced = nullptr;

        if (kernel < mKernelId) {
            produced = mLocallyAvailableItems[streamSet];
        } else {
            if (kernel == mKernelId) {

                if (LLVM_UNLIKELY(br.IsDeferred)) {
                    produced = mInitiallyProducedDeferredItemCount[streamSet];
                } else {
                    produced = mInitiallyProducedItemCount[streamSet];
                }

                if (fromInitiallyTerminated) {
//                    if (LLVM_UNLIKELY(br.IsDeferred)) {
//                        produced = mInitiallyProducedDeferredItemCount[streamSet];
//                    } else {
//                        produced = mInitiallyProducedItemCount[streamSet];
//                    }
                } else {

                    Value * alreadyProduced = produced;
//                    if (LLVM_UNLIKELY(br.IsDeferred)) {
//                        alreadyProduced = mAlreadyProducedDeferredPhi[br.Port];
//                    } else {
//                        alreadyProduced = mAlreadyProducedPhi[br.Port];
//                    }
//                    assert (alreadyProduced);

//                    produced = alreadyProduced;

                    const auto nextPartitionId = mCurrentPartitionId + 1U;
                    const auto jumpId = mPartitionJumpIndex[mCurrentPartitionId];

                    assert (nextPartitionId <= jumpId);

                    if (LLVM_UNLIKELY(nextPartitionId == jumpId)) {
                        Value * initiallyProduced = nullptr;
                        if (LLVM_UNLIKELY(br.IsDeferred)) {
                            initiallyProduced = mInitiallyProducedDeferredItemCount[streamSet];
                        } else {
                            initiallyProduced = mInitiallyProducedItemCount[streamSet];
                        }
                        assert (initiallyProduced);

                        const auto ip = b->saveIP();
                        b->SetInsertPoint(entryPoint, entryPoint->begin());
                        PHINode * const phi = b->CreatePHI(b->getSizeTy(), 3);
                        assert ((mKernelInitiallyTerminatedExit == nullptr) ^ (mKernelInitiallyTerminated != nullptr));
                        assert (mCurrentPartitionEntryGuard);
                        phi->addIncoming(initiallyProduced, mCurrentPartitionEntryGuard);
                        if (mKernelInitiallyTerminatedExit) {
                            phi->addIncoming(initiallyProduced, mKernelInitiallyTerminatedExit);
                        }
//                        if (mKernelInsufficientInput) {
//                            phi->addIncoming(alreadyProduced, mKernelInsufficientInput);
//                        }
                        produced = phi;
                        b->restoreIP(ip);                       
                    }

                }

//                assert (mTerminatedInitially);
//                Value * const termSignal = b->CreateTrunc(mTerminatedInitially, b->getInt1Ty(), "termSignal");
//                produced = computeFullyProducedItemCount(b, kernel, br.Port, produced, termSignal);
            } else {
                if (fromInitiallyTerminated) {
                    const auto prefix = makeBufferName(kernel, br.Port);
                    if (LLVM_UNLIKELY(br.IsDeferred)) {
                        produced = b->getScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX);
                    } else {
                        produced = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
                    }
                } else {
                    produced = mInitiallyAvailableItemsPhi[streamSet];
                }
            }
        }

        #ifdef PRINT_DEBUG_MESSAGES
        SmallVector<char, 256> tmp;
        raw_svector_ostream out(tmp);
        out << makeKernelName(mKernelId) << " -> " <<
               makeBufferName(kernel, br.Port) << "_avail = %" PRIu64;
        debugPrint(b, out.str(), produced);
        #endif

        producedSet.emplace_back(streamSet, produced);

        bool prepareConsumedPhi = false;
        for (const auto f : make_iterator_range(out_edges(streamSet, mConsumerGraph))) {
            const auto consumer = target(f, mConsumerGraph);
            const auto p = KernelPartitionId[consumer];
            if (p >= targetPartitionId) {
                prepareConsumedPhi = true;
                break;
            }
        }

        if (prepareConsumedPhi) {
            Value * consumed = nullptr;
            if (kernel >= mKernelId) {
                consumed = readConsumedItemCount(b, streamSet);
                assert (consumed);
            } else {
                consumed = mInitialConsumedItemCount[streamSet];
                assert (consumed);
            }            

            #ifdef PRINT_DEBUG_MESSAGES
            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            out << makeKernelName(mKernelId) << " -> " <<
                   makeBufferName(kernel, br.Port) << "_consumed = %" PRIu64;
            debugPrint(b, out.str(), consumed);
            #endif

            consumedSet.emplace_back(streamSet, consumed);
        }
    }

    auto phiOut = [&](const PhiDataSet & set, PartitionPhiNodeTable & tbl, const StringRef prefix) {

        for (const PhiData & item : set) {
            PHINode *& phi = tbl[targetPartitionId][item.StreamSet - FirstStreamSet];
            if (phi == nullptr) {
                BasicBlock * const entryPoint = mPartitionEntryPoint[targetPartitionId];
                assert (entryPoint->getFirstNonPHI() == nullptr);
                const auto expected = in_degree(targetPartitionId, mPartitionJumpTree) + 2;

                SmallVector<char, 256> tmp;
                raw_svector_ostream nm(tmp);
                nm << prefix << "_" << item.StreamSet << "@" << targetPartitionId;

                phi = PHINode::Create(b->getSizeTy(), expected, nm.str(), entryPoint);
                assert (tbl[targetPartitionId][item.StreamSet - FirstStreamSet] == phi);
            }
            phi->addIncoming(item.ItemCount, exitPoint);
        }

    };

    phiOut(producedSet, mPartitionProducedItemCountPhi, "partitionProduced");
    phiOut(consumedSet, mPartitionConsumedItemCountPhi, "partitionConsumed");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief phiOutPartitionStatusFlags
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::phiOutPartitionStatusFlags(BuilderRef b, const unsigned targetPartitionId,
                                                  const bool /* fromKernelEntry */,
                                                  BasicBlock * const /* entryPoint */) {

    BasicBlock * const exitPoint = b->GetInsertBlock();

    auto findOrAddPhi = [&](PartitionPhiNodeTable & tbl, const unsigned partitionId, const StringRef prefix) -> PHINode * {
        PHINode *& phi = tbl[targetPartitionId][partitionId];
        assert (targetPartitionId != partitionId);
        if (phi == nullptr) {
            BasicBlock * const entryPoint = mPartitionEntryPoint[targetPartitionId];
            assert (entryPoint->getFirstNonPHI() == nullptr);
            const auto expected = in_degree(targetPartitionId, mPartitionJumpTree) + 2;

            SmallVector<char, 256> tmp;
            raw_svector_ostream nm(tmp);
            nm << prefix << "_" << partitionId << "@" << targetPartitionId;

            phi = PHINode::Create(b->getSizeTy(), expected, nm.str(), entryPoint);

            assert (tbl[targetPartitionId][partitionId] == phi);
        }
        return phi;
    };

    const auto firstPartition = KernelPartitionId[FirstKernel];

    for (auto partitionId = firstPartition; partitionId != targetPartitionId; ++partitionId) {
        PHINode * const termPhi = findOrAddPhi(mPartitionTerminationSignalPhi, partitionId, "partitionTerminationSignalPhi");
        Value * term = nullptr;
        if (partitionId < mCurrentPartitionId) {
            term = mPartitionTerminationSignal[partitionId]; assert (term);
        } else {
            term = readTerminationSignal(b, partitionId); assert (term);
        }
        termPhi->addIncoming(term, exitPoint);
    }

    PHINode * const progressPhi = mPartitionPipelineProgressPhi[targetPartitionId]; assert (progressPhi);
    Value * const progress = mPipelineProgress; // fromKernelEntry ? mPipelineProgress : mAlreadyProgressedPhi;
    progressPhi->addIncoming(progress, exitPoint);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief releaseAllSynchronizationLocksUntil
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::acquireAndReleaseAllSynchronizationLocksUntil(BuilderRef b, const unsigned partitionId) {
    // Find the first kernel in the partition we're jumping to and acquire the LSN then release
    // all of the kernels we skipped over. However, to safely jump to a partition, we need to
    // know how many items were processed by any consumers of the kernels up to the target
    // kernel; otherwise we run the risk of mangling the buffer state. For safety, wait until we
    // can acquire the last consumer's lock but only release the locks that we end up skipping.

    auto firstKernelInTargetPartition = mKernelId;
    auto lastConsumer = mKernelId;
    for (; firstKernelInTargetPartition <= LastKernel; ++firstKernelInTargetPartition) {
        for (const auto e : make_iterator_range(out_edges(firstKernelInTargetPartition, mBufferGraph))) {
            const auto streamSet = target(e, mBufferGraph);
            for (const auto e : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
                const auto consumer = target(e, mBufferGraph);
                lastConsumer = std::max<unsigned>(lastConsumer, consumer);
            }
        }
        if (KernelPartitionId[firstKernelInTargetPartition] == partitionId) {
            break;
        }
    }
    assert (firstKernelInTargetPartition > mKernelId);    
    lastConsumer = std::min(lastConsumer, LastKernel);

    // TODO: experiment with a mutex lock here.
    Value * const startTime = startCycleCounter(b);
    acquireSynchronizationLock(b, lastConsumer);
    updateCycleCounter(b, mKernelId, startTime, CycleCounter::PARTITION_JUMP_SYNCHRONIZATION);
    for (auto kernel = mKernelId; kernel < firstKernelInTargetPartition; ++kernel) {
        releaseSynchronizationLock(b, kernel);
    }

    return startTime;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeInitiallyTerminatedPartitionExit
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeInitiallyTerminatedPartitionExit(BuilderRef b) {

    b->SetInsertPoint(mKernelInitiallyTerminated);

    loadLastGoodVirtualBaseAddressesOfUnownedBuffersInPartition(b);

    // create a temporary basic block to act a constant exit block of any phi nodes;
    // once we've determined the actual exit block of this portion of the program,
    // replace the phi catch with it.

    if (mIsPartitionRoot) {

        #ifdef INITIALLY_TERMINATED_KERNELS_JUMP_TO_NEXT_PARTITION
        const auto nextPartitionId = mCurrentPartitionId + 1U;
        const auto jumpId = mPartitionJumpIndex[mCurrentPartitionId];

        assert (nextPartitionId <= jumpId);       
        if (LLVM_LIKELY(nextPartitionId != jumpId)) {

            Value * const startTime = acquireAndReleaseAllSynchronizationLocksUntil(b, nextPartitionId);
            mKernelInitiallyTerminatedExit = b->GetInsertBlock();

            if (LLVM_UNLIKELY(EnableCycleCounter)) {
                mPartitionStartTimePhi[nextPartitionId]->addIncoming(startTime, mKernelInitiallyTerminatedExit);
            }

            PHINode * const exhaustedInputPhi = mExhaustedPipelineInputAtPartitionEntry[nextPartitionId];
            exhaustedInputPhi->addIncoming(mExhaustedInput, mKernelInitiallyTerminatedExit);

            for (auto kernel = PipelineInput; kernel <= mKernelId; ++kernel) {
                phiOutPartitionItemCounts(b, kernel, nextPartitionId, true, mKernelInitiallyTerminated);
            }
            for (auto kernel = mKernelId + 1U; kernel <= LastKernel; ++kernel) {
                if (KernelPartitionId[kernel] != mCurrentPartitionId) {
                    break;
                }
                phiOutPartitionItemCounts(b, kernel, nextPartitionId, true, mKernelInitiallyTerminated);
            }
            phiOutPartitionStatusFlags(b, nextPartitionId, true, mKernelInitiallyTerminated);

            updateCycleCounter(b, mKernelId, mKernelStartTime, CycleCounter::TOTAL_TIME);

            b->CreateBr(mNextPartitionEntryPoint);

        } else {
        #endif
            mKernelInitiallyTerminatedExit = b->GetInsertBlock();
            b->CreateBr(mKernelJumpToNextUsefulPartition);
            if (mExhaustedInputAtJumpPhi) {
                mExhaustedInputAtJumpPhi->addIncoming(mExhaustedInput, mKernelInitiallyTerminatedExit);
            }
        #ifdef INITIALLY_TERMINATED_KERNELS_JUMP_TO_NEXT_PARTITION
        }
        #endif

    } else { // if (!mIsPartitionRoot) {

        mKernelInitiallyTerminatedExit = b->GetInsertBlock();
        updateKernelExitPhisAfterInitiallyTerminated(b);
        b->CreateBr(mKernelExit);

    }

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeOnInitialTerminationJumpToNextPartitionToCheck
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeJumpToNextPartition(BuilderRef b) {

    const auto nextPartitionId = mPartitionJumpIndex[mCurrentPartitionId];
    assert (mCurrentPartitionId < nextPartitionId);

    Value * const startTime = acquireAndReleaseAllSynchronizationLocksUntil(b, nextPartitionId);
    BasicBlock * const exitBlock = b->GetInsertBlock();
    if (LLVM_UNLIKELY(EnableCycleCounter)) {
        mPartitionStartTimePhi[nextPartitionId]->addIncoming(startTime, exitBlock);
    }

    PHINode * const exhaustedInputPhi = mExhaustedPipelineInputAtPartitionEntry[nextPartitionId];
    if (exhaustedInputPhi) {       
        Value * const exhausted = mIsBounded ? mExhaustedInputAtJumpPhi : mExhaustedInput;
        exhaustedInputPhi->addIncoming(exhausted, exitBlock); assert (exhausted);
    }
    for (auto kernel = PipelineInput; kernel <= mKernelId; ++kernel) {
        phiOutPartitionItemCounts(b, kernel, nextPartitionId, false, mKernelJumpToNextUsefulPartition);
    }
    // NOTE: break condition differs from "writeInitiallyTerminatedPartitionExit"
    for (auto kernel = mKernelId + 1U; kernel <= LastKernel; ++kernel) {
        if (mPartitionJumpIndex[KernelPartitionId[kernel]] > nextPartitionId) {
            break;
        }
        phiOutPartitionItemCounts(b, kernel, nextPartitionId, false, mKernelJumpToNextUsefulPartition);
    }
    phiOutPartitionStatusFlags(b, nextPartitionId, false, mKernelJumpToNextUsefulPartition);

    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, "** " + makeKernelName(mKernelId) + ".jumping = %" PRIu64, mSegNo);
    #endif

    updateCycleCounter(b, mKernelId, mKernelStartTime, CycleCounter::TOTAL_TIME);

    assert (mNextPartitionWithPotentialInput);

    b->CreateBr(mNextPartitionWithPotentialInput);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForPartitionExit
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::checkForPartitionExit(BuilderRef b) {

    assert (mKernelId >= FirstKernel && mKernelId <= LastKernel);

    releaseSynchronizationLock(b, mKernelId);

    const auto nextKernel = mKernelId + 1U;
    if (LLVM_LIKELY(nextKernel < PipelineOutput)) {
        mKernelStartTime = startCycleCounter(b);
        acquireSynchronizationLock(b, nextKernel);
        updateCycleCounter(b, nextKernel, mKernelStartTime, CycleCounter::KERNEL_SYNCHRONIZATION);
    }

    const auto nextPartitionId = KernelPartitionId[nextKernel];

    assert (nextKernel != PipelineOutput || (nextPartitionId != mCurrentPartitionId));

    if (nextPartitionId != mCurrentPartitionId) {
        assert (mCurrentPartitionId < nextPartitionId);
        assert (nextPartitionId <= PartitionCount);
        BasicBlock * const exitBlock = b->GetInsertBlock();
        #ifdef PRINT_DEBUG_MESSAGES
        debugPrint(b, "  *** exiting partition %" PRIu64, b->getSize(mCurrentPartitionId));
        #endif
        b->CreateBr(mNextPartitionEntryPoint);

        b->SetInsertPoint(mNextPartitionEntryPoint);
        PHINode * const progressPhi = mPartitionPipelineProgressPhi[nextPartitionId];
        progressPhi->addIncoming(mPipelineProgress, exitBlock);
        mPipelineProgress = progressPhi;
        // Since there may be multiple paths into this kernel, phi out the start time
        // for each path.
        if (LLVM_UNLIKELY(EnableCycleCounter)) {
            mPartitionStartTimePhi[nextPartitionId]->addIncoming(mKernelStartTime, exitBlock);
            mKernelStartTime = mPartitionStartTimePhi[nextPartitionId];
        }

        PHINode * const exhaustedInputPhi = mExhaustedPipelineInputAtPartitionEntry[nextPartitionId];
        if (exhaustedInputPhi) {
            exhaustedInputPhi->addIncoming(mExhaustedInput, exitBlock);
            mExhaustedInput = exhaustedInputPhi;
        }

        const auto n = LastStreamSet - FirstStreamSet + 1U;

        for (unsigned i = 0; i != n; ++i) {
            PHINode * const phi = mPartitionProducedItemCountPhi[nextPartitionId][i];
            if (phi) {
                const auto streamSet = FirstStreamSet + i;
                phi->addIncoming(mLocallyAvailableItems[streamSet], exitBlock);
                mLocallyAvailableItems[streamSet] = phi;
            }
        }

        for (unsigned i = 0; i != n; ++i) {
            PHINode * const phi = mPartitionConsumedItemCountPhi[nextPartitionId][i];
            if (phi) {
                const auto streamSet = FirstStreamSet + i;
                const ConsumerNode & cn = mConsumerGraph[streamSet];
                assert (cn.Consumed);
                phi->addIncoming(cn.Consumed, exitBlock);
                cn.Consumed = phi;
                assert (cn.PhiNode == nullptr);
                // The consumed phi node propagates the *initial* consumed item count for
                // each item to reflect the fact we've skipped some kernel. However, since
                // we might skip the actual producer, we need to constantly update the
                // initial item count value to construct a legal program. Despite this
                // the initial consumed item count should be considered as fixed value
                // per pipeline iteration.
                mInitialConsumedItemCount[streamSet] = phi;
            }
        }

        for (unsigned i = 0; i != nextPartitionId; ++i) {
            PHINode * const termPhi = mPartitionTerminationSignalPhi[nextPartitionId][i];
            if (termPhi) {
                termPhi->addIncoming(mPartitionTerminationSignal[i], exitBlock);
                mPartitionTerminationSignal[i] = termPhi;
            }
        }

    }
}

#if 0
/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setPartitionVariablesToPipelineEnd
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::setPartitionVariablesToPipelineEnd(BuilderRef /* b */) {

    const auto nextPartitionId = KernelPartitionId[LastKernel];

    mExhaustedInput = mExhaustedPipelineInputAtPartitionEntry[nextPartitionId];

    const auto n = LastStreamSet - FirstStreamSet + 1U;

    for (unsigned i = 0; i != n; ++i) {
        PHINode * const phi = mPartitionProducedItemCountPhi[nextPartitionId][i];
        const auto streamSet = FirstStreamSet + i;
        mLocallyAvailableItems[streamSet] = phi;
    }

    for (unsigned i = 0; i != n; ++i) {
        PHINode * const phi = mPartitionConsumedItemCountPhi[nextPartitionId][i];
        const auto streamSet = FirstStreamSet + i;
        mInitialConsumedItemCount[streamSet] = phi;
    }

    for (unsigned i = 0; i != nextPartitionId; ++i) {
        mPartitionTerminationSignal[i] = mPartitionTerminationSignalPhi[nextPartitionId][i];
    }

}
#endif

}

#endif
