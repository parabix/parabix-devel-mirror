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
    for (unsigned i = firstPartition; i < PartitionCount; ++i) {
        mPartitionEntryPoint[i] = b->CreateBasicBlock("Partition" + std::to_string(i));
    }

    const auto ip = b->saveIP();
    IntegerType * const boolTy = b->getInt1Ty();
    for (auto i = firstPartition + 1U; i < PartitionCount; ++i) {
        b->SetInsertPoint(mPartitionEntryPoint[i]);
        assert (mPartitionEntryPoint[i]->getFirstNonPHI() == nullptr);
        mPartitionPipelineProgressPhi[i] = b->CreatePHI(boolTy, PartitionCount, std::to_string(i) + ".pipelineProgress");
        mExhaustedPipelineInputAtPartitionEntry[i] = b->CreatePHI(boolTy, PartitionCount, std::to_string(i) + ".exhaustedInput");
    }
    initializePipelineInputConsumedPhiNodes(b);
    b->restoreIP(ip);

    mPipelineEnd = b->CreateBasicBlock("PipelineEnd");
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
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPartitionExitPoint
 ** ------------------------------------------------------------------------------------------------------------- */
inline BasicBlock * PipelineCompiler::getPartitionExitPoint(BuilderRef b) {
    assert (mKernelId >= FirstKernel && mKernelId <= LastKernel);
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
        identifyPartitionKernelRange();
        determinePartitionStrideRates();

        const auto jumpIdx = mPartitionJumpIndex[partitionId];
        assert (partitionId != jumpIdx);
        mNextPartitionWithPotentialInput = mPartitionEntryPoint[jumpIdx];        
        mIsPartitionRoot = true;
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
    const auto & max = MaximumNumOfStrides[FirstKernelInPartition];
    MaxPartitionStrideRate = max;
    for (auto i = FirstKernelInPartition + 1U; i <= LastKernelInPartition; ++i) {
        const auto & max = MaximumNumOfStrides[i];
        MaxPartitionStrideRate = std::max(MaxPartitionStrideRate, max);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadLastGoodVirtualBaseAddressesOfUnownedBuffersInCurrentPartition
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::loadLastGoodVirtualBaseAddressesOfUnownedBuffersInPartition(BuilderRef b) const {
    for (auto i = FirstKernelInPartition; i <= LastKernel; ++i) {
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


        const BufferRateData & br = mBufferGraph[e];
        // Select/compute/load the appropriate produced item count
        Value * produced = nullptr;

        #ifdef PRINT_DEBUG_MESSAGES
        SmallVector<char, 256> tmp;
        raw_svector_ostream out(tmp);
        out << makeKernelName(mKernelId) << " -> " << makeBufferName(kernel, br.Port);
        out << "_avail";
        #endif

        if (kernel < mKernelId) {
            produced = mLocallyAvailableItems[streamSet];

            #ifdef PRINT_DEBUG_MESSAGES
            out << '0';
            #endif

        } else {
            if (kernel == mKernelId) {
                if (fromInitiallyTerminated) {
                    if (LLVM_UNLIKELY(br.IsDeferred)) {
                        produced = mInitiallyProducedDeferredItemCount[streamSet];
                    } else {
                        produced = mInitiallyProducedItemCount[streamSet];
                    }
                    #ifdef PRINT_DEBUG_MESSAGES
                    out << '1';
                    #endif
                } else {

                    Value * alreadyProduced = nullptr;
                    if (LLVM_UNLIKELY(br.IsDeferred)) {
                        alreadyProduced = mAlreadyProducedDeferredPhi[br.Port];
                    } else {
                        alreadyProduced = mAlreadyProducedPhi[br.Port];
                    }
                    assert (alreadyProduced);

                    produced = alreadyProduced;

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
                        PHINode * const phi = b->CreatePHI(b->getSizeTy(), 2);
                        assert ((mKernelInitiallyTerminatedExit == nullptr) ^ (mKernelInitiallyTerminated != nullptr));
                        if (mKernelInitiallyTerminatedExit) {
                            phi->addIncoming(initiallyProduced, mKernelInitiallyTerminatedExit);
                        }
                        if (mKernelInsufficientInput) {
                            phi->addIncoming(alreadyProduced, mKernelInsufficientInput);
                        }
                        produced = phi;
                        b->restoreIP(ip);                       
                    }

                }
                produced = computeFullyProducedItemCount(b, kernel, br.Port, produced, mTerminatedInitially);
                #ifdef PRINT_DEBUG_MESSAGES
                out << '2';
                #endif
            } else {
                if (fromInitiallyTerminated) {
//                    const auto ip = b->saveIP();
//                    b->SetInsertPoint(phiCatchBlock);
                    const auto prefix = makeBufferName(kernel, br.Port);
                    if (LLVM_UNLIKELY(br.IsDeferred)) {
                        produced = b->getScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX);
                    } else {
                        produced = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
                    }
//                    b->restoreIP(ip);
                    #ifdef PRINT_DEBUG_MESSAGES
                    out << '3';
                    #endif
                } else {
                    produced = mInitiallyAvailableItemsPhi[streamSet];
                    #ifdef PRINT_DEBUG_MESSAGES
                    out << '4';
                    #endif
                }
            }
        }

        #ifdef PRINT_DEBUG_MESSAGES
        out << " = %" PRIu64;
//        const auto ip = b->saveIP();
//        b->SetInsertPoint(phiCatchBlock);
        debugPrint(b, out.str(), produced);
//        b->restoreIP(ip);
        #endif

        producedSet.emplace_back(streamSet, produced);

        bool prepareConsumedPhi = false;
        for (const auto f : make_iterator_range(out_edges(streamSet, mConsumerGraph))) {
            const ConsumerEdge & c = mConsumerGraph[f];
            assert (c.Flags != ConsumerEdge::None);
            const auto consumer = target(f, mConsumerGraph);
            const auto p = KernelPartitionId[consumer];
            if (p >= targetPartitionId) {
                prepareConsumedPhi = true;
                break;
            }
        }
        if (prepareConsumedPhi) {

            #ifdef PRINT_DEBUG_MESSAGES
            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            out << makeKernelName(mKernelId) << " -> " << makeBufferName(kernel, br.Port);
            out << "_consumed";
            #endif

            Value * consumed = nullptr;
            if (kernel > mKernelId) {
                consumed = readConsumedItemCount(b, streamSet);
                #ifdef PRINT_DEBUG_MESSAGES
                out << '0';
                #endif
            } else {
                consumed = mInitialConsumedItemCount[streamSet];
                #ifdef PRINT_DEBUG_MESSAGES
                out << '1';
                #endif
            }
            assert (consumed);

            #ifdef PRINT_DEBUG_MESSAGES
            out << " = %" PRIu64;
//            const auto ip = b->saveIP();
//            b->SetInsertPoint(phiCatchBlock);
            debugPrint(b, out.str(), consumed);
//            b->restoreIP(ip);
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
void PipelineCompiler::phiOutPartitionStatusFlags(BuilderRef b, const unsigned targetPartitionId, const bool fromKernelEntry,
                                                  BasicBlock * const entryPoint) {

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

    for (auto partitionId = 0U; partitionId != targetPartitionId; ++partitionId) {
        PHINode * const termPhi = findOrAddPhi(mPartitionTerminationSignalPhi, partitionId, "partitionTerminationSignalPhi");
        Value * term = nullptr;
        if (partitionId <= mCurrentPartitionId) {
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
void PipelineCompiler::acquireAndReleaseAllSynchronizationLocksUntil(BuilderRef b, const unsigned partitionId) {
    // Find the first kernel in the partition we're jumping to and acquire the LSN
    // then release all of the kernels we skipped over.
    auto firstKernelInNextPartition = mKernelId + 1U;
    for (; firstKernelInNextPartition <= LastKernel; ++firstKernelInNextPartition) {
        if (KernelPartitionId[firstKernelInNextPartition] == partitionId) {
            break;
        }
    }

    // TODO: experiment with a mutex lock here.
    const auto lockingKernel = std::min(firstKernelInNextPartition, LastKernel);
    assert ((firstKernelInNextPartition == lockingKernel) || (firstKernelInNextPartition == PipelineOutput));
    acquireSynchronizationLock(b, lockingKernel, CycleCounter::INITIAL);
    for (auto kernel = mKernelId; kernel < firstKernelInNextPartition; ++kernel) {
        releaseSynchronizationLock(b, kernel);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeInitiallyTerminatedPartitionExit
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeInitiallyTerminatedPartitionExit(BuilderRef b) {

    #ifdef INITIALLY_TERMINATED_KERNELS_JUMP_TO_NEXT_PARTITION

    // create a temporary basic block to act a constant exit block of any phi nodes;
    // once we've determined the actual exit block of this portion of the program,
    // replace the phi catch with it.

    const auto nextPartitionId = mCurrentPartitionId + 1U;
    const auto jumpId = mPartitionJumpIndex[mCurrentPartitionId];

    assert (nextPartitionId <= jumpId);

    if (LLVM_LIKELY(nextPartitionId != jumpId)) {

        acquireAndReleaseAllSynchronizationLocksUntil(b, nextPartitionId);
        loadLastGoodVirtualBaseAddressesOfUnownedBuffersInPartition(b);
        mKernelInitiallyTerminatedExit = b->GetInsertBlock();

        PHINode * const exhaustedInputPhi = mExhaustedPipelineInputAtPartitionEntry[nextPartitionId];
        exhaustedInputPhi->addIncoming(mExhaustedInput, mKernelInitiallyTerminatedExit);

        for (auto kernel = FirstKernel; kernel <= mKernelId; ++kernel) {
            phiOutPartitionItemCounts(b, kernel, nextPartitionId, true, mKernelInitiallyTerminated);
        }
        for (auto kernel = mKernelId + 1U; kernel <= LastKernel; ++kernel) {
            if (KernelPartitionId[kernel] != mCurrentPartitionId) {
                break;
            }
            phiOutPartitionItemCounts(b, kernel, nextPartitionId, true, mKernelInitiallyTerminated);
        }

        phiOutPartitionStatusFlags(b, nextPartitionId, true, mKernelInitiallyTerminated);
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
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeOnInitialTerminationJumpToNextPartitionToCheck
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeJumpToNextPartition(BuilderRef b) {

    const auto nextPartitionId = mPartitionJumpIndex[mCurrentPartitionId];
    assert (mCurrentPartitionId < nextPartitionId);

    acquireAndReleaseAllSynchronizationLocksUntil(b, nextPartitionId);

    BasicBlock * const exitBlock = b->GetInsertBlock(); // b->CreateBasicBlock();
    for (auto kernel = FirstKernel; kernel <= mKernelId; ++kernel) {
        phiOutPartitionItemCounts(b, kernel, nextPartitionId, false, mKernelJumpToNextUsefulPartition);
    }
    for (auto kernel = mKernelId + 1U; kernel <= LastKernel; ++kernel) {
        if (mPartitionJumpIndex[KernelPartitionId[kernel]] > nextPartitionId) {
            break;
        }
        phiOutPartitionItemCounts(b, kernel, nextPartitionId, false, mKernelJumpToNextUsefulPartition);
    }
    phiOutPartitionStatusFlags(b, nextPartitionId, false, mKernelJumpToNextUsefulPartition);
    PHINode * const exhaustedInputPhi = mExhaustedPipelineInputAtPartitionEntry[nextPartitionId];
    if (exhaustedInputPhi) {       
        Value * const exhausted = mIsBounded ? mExhaustedInputAtJumpPhi : mExhaustedInput;
        exhaustedInputPhi->addIncoming(exhausted, exitBlock); assert (exhausted);
    }
    // if we jump to the end of the pipeline, phi out the consumed item counts.
    if (LLVM_UNLIKELY(nextPartitionId == (PartitionCount - 1U))) {
        updatePipelineInputConsumedItemCounts(b, exitBlock);
    }
//    replacePhiCatchWithCurrentBlock(b, exitBlock, mNextPartitionWithPotentialInput);

    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, "** " + makeKernelName(mKernelId) + ".jumping = %" PRIu64, mSegNo);
    #endif

    b->CreateBr(mNextPartitionWithPotentialInput);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForPartitionExit
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::checkForPartitionExit(BuilderRef b) {

    assert (mKernelId >= FirstKernel && mKernelId <= LastKernel);

    releaseSynchronizationLock(b, mKernelId);

    const auto nextKernel = mKernelId + 1U;
    if (nextKernel != PipelineOutput) {
        acquireSynchronizationLock(b, nextKernel, CycleCounter::INITIAL);
    }

    const auto nextPartitionId = KernelPartitionId[nextKernel];

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
                // The consumed phi node propagates the *initial* consumed item count for
                // each item to reflect the fact we've skipped some kernel. However, since
                // we might skip the actual producer, we need to constantly update the
                // initial item count value to construct a legal program. Despite this
                // the initial consumed item count should be considered as fixed value
                // per pipeline iteration.
                mInitialConsumedItemCount[streamSet] = phi;
            }
        }

        // if we jump to the end of the pipeline, phi out the consumed item counts.
        if (LLVM_UNLIKELY(nextPartitionId == (PartitionCount - 1U))) {
            updatePipelineInputConsumedItemCounts(b, exitBlock);
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

}

#endif
