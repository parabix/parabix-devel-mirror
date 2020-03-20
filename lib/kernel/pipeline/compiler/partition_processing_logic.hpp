#ifndef PARTITION_PROCESSING_LOGIC_HPP
#define PARTITION_PROCESSING_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

//const static std::string PARTITION_LOGICAL_SEGMENT_NUMBER = ".PLS";
//const static std::string PARTITION_ITEM_COUNT_SUFFIX = ".PIC";
//const static std::string PARTITION_FIXED_RATE_SUFFIX = ".PFR";

inline std::string getPartitionSegmentNumberName(const size_t partitionId) {
    return std::to_string(partitionId) + PARTITION_LOGICAL_SEGMENT_NUMBER;
}

inline std::string getPartitionFixedRateName(const size_t partitionId) {
    return std::to_string(partitionId) + PARTITION_FIXED_RATE_SUFFIX;
}

inline std::string getPartitionPortName(const size_t partitionId, const PartitioningGraphEdge & e) {
    std::string tmp;
    raw_string_ostream out(tmp);
    out << partitionId << '.'
        << e.Kernel <<
        ((e.Port.Type == PortType::Input) ? 'I' : 'O') <<
        e.Port.Number <<
        PARTITION_ITEM_COUNT_SUFFIX;
    out.flush();
    return tmp;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addPartitionItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addPartitionInputItemCounts(BuilderRef b, const size_t partitionId) const {
    IntegerType * const int64Ty = b->getInt64Ty();
    mTarget->addInternalScalar(int64Ty, getPartitionSegmentNumberName(partitionId));

    bool hasFixedRate = false;
    for (unsigned i = FirstKernel, partitionSize = 0; i <= LastKernel; ++i) {
        if (KernelPartitionId[i] == partitionId) {
            ++partitionSize;
            if (partitionSize > 1) {
                hasFixedRate = true;
                break;
            }
        }
    }

    for (const auto e : make_iterator_range(out_edges(partitionId, mPartitioningGraph))) {
        const auto v = target(e, mPartitioningGraph);
        const PartitioningGraphNode & node = mPartitioningGraph[v];
        if (node.Type == PartitioningGraphNode::Fixed) {
            hasFixedRate = true;
            break;
        }
    }

    if (LLVM_LIKELY(hasFixedRate)) {
        mTarget->addInternalScalar(int64Ty, getPartitionFixedRateName(partitionId));
    }
    for (const auto e : make_iterator_range(in_edges(partitionId, mPartitioningGraph))) {
        mTarget->addInternalScalar(int64Ty, getPartitionPortName(partitionId, mPartitioningGraph[e]));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePartitionEntryPoints
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::makePartitionEntryPoints(BuilderRef b) {    
    mPipelineLoop = b->CreateBasicBlock("PipelineLoop");
    mPartitionEntryPoint.resize(PartitionCount);

    for (unsigned i = 0; i < PartitionCount; ++i) {
        mPartitionEntryPoint[i] = b->CreateBasicBlock("Partition" + std::to_string(i));
    }

    const auto ip = b->saveIP();
    IntegerType * const boolTy = b->getInt1Ty();
    for (unsigned i = 1; i < PartitionCount; ++i) {
        b->SetInsertPoint(mPartitionEntryPoint[i]);
        assert (mPartitionEntryPoint[i]->getFirstNonPHI() == nullptr);
        mPartitionPipelineProgressPhi[i] = b->CreatePHI(boolTy, PartitionCount, std::to_string(i) + ".pipelineProgress");
        mExhaustedPipelineInputAtPartitionEntry[i] = b->CreatePHI(boolTy, PartitionCount, std::to_string(i) + ".exhaustedInput");
    }
    b->restoreIP(ip);

    mPipelineEnd = b->CreateBasicBlock("PipelineEnd");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief branchToInitialPartition
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::branchToInitialPartition(BuilderRef b) {
    assert (KernelPartitionId[FirstKernel] == 0);
    BasicBlock * const entry = mPartitionEntryPoint[0];
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
    return mPartitionEntryPoint[partitionId + 1];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkInputDataOnPartitionEntry
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::checkForPartitionEntry(BuilderRef b) {
    assert (mKernelId >= FirstKernel && mKernelId <= LastKernel);
    mNextPartitionWithPotentialInput = nullptr;
    mIsPartitionRoot = false;
    const auto partitionId = KernelPartitionId[mKernelId];
    if (partitionId != mCurrentPartitionId) {
        mPartitionRootKernelId = mKernelId;
        mCurrentPartitionId = partitionId;
        const auto jumpIdx = mPartitionJumpIndex[partitionId];
        mNextPartitionWithPotentialInput = mPartitionEntryPoint[jumpIdx];
        mIsPartitionRoot = true;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadLastGoodVirtualBaseAddressesOfUnownedBuffersInCurrentPartition
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::loadLastGoodVirtualBaseAddressesOfUnownedBuffersInPartition(BuilderRef b) const {
    for (auto i = mPartitionRootKernelId; i <= LastKernel; ++i) {
        if (KernelPartitionId[i] != mCurrentPartitionId) {
            break;
        }
        loadLastGoodVirtualBaseAddressesOfUnownedBuffers(b, i);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadLastGoodVirtualBaseAddressesOfUnownedBuffersInCurrentPartition
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::phiOutPartitionItemCounts(BuilderRef b, const unsigned kernel, const unsigned targetPartitionId, const bool fromKernelEntry, BasicBlock * const exitBlock) {

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
        if (kernel < mKernelId) {
            produced = mLocallyAvailableItems[streamSet];
        } else {
            const Binding & output = br.Binding;
            const auto deferred = output.isDeferred();
            if (kernel == mKernelId) {
                if (fromKernelEntry) {
                    if (LLVM_UNLIKELY(deferred)) {
                        produced = mInitiallyProducedDeferredItemCount[streamSet];
                    } else {
                        produced = mInitiallyProducedItemCount[streamSet];
                    }
                } else {
                    Value * initiallyProduced;
                    Value * alreadyProduced;
                    if (LLVM_UNLIKELY(deferred)) {
                        initiallyProduced = mInitiallyProducedDeferredItemCount[streamSet];
                        alreadyProduced = mAlreadyProducedDeferredPhi(br.Port);
                    } else {
                        initiallyProduced = mInitiallyProducedItemCount[streamSet];
                        alreadyProduced = mAlreadyProducedPhi(br.Port);
                    }
                    PHINode * const phi = b->CreatePHI(b->getSizeTy(), 2);
                    if (mKernelInitiallyTerminatedExit) {
                        phi->addIncoming(initiallyProduced, mKernelInitiallyTerminatedExit);
                    }
                    if (mKernelInsufficientInputExit) {
                        phi->addIncoming(alreadyProduced, mKernelInsufficientInputExit);
                    }
                    produced = phi;
                }
                produced = computeFullyProducedItemCount(b, kernel, br.Port, produced, mTerminatedInitially);
            } else {
                produced = mInitiallyAvailableItemsPhi[streamSet];
            }
        }
        producedSet.emplace_back(streamSet, produced);

        bool prepareConsumedPhi = false;
        for (const auto f : make_iterator_range(out_edges(streamSet, mConsumerGraph))) {
            const ConsumerEdge & c = mConsumerGraph[f];
            if (c.Flags == ConsumerEdge::None) continue;
            const auto consumer = target(f, mConsumerGraph);
            const auto p = KernelPartitionId[consumer];
            if (p >= targetPartitionId) {
                prepareConsumedPhi = true;
                break;
            }
        }
        if (prepareConsumedPhi) {
            Value * consumed = nullptr;
            if (kernel <= mKernelId) {
                consumed = mInitialConsumedItemCount[streamSet];
            } else {
                consumed = readConsumedItemCount(b, streamSet);
            }
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
            phi->addIncoming(item.ItemCount, exitBlock);
        }

    };

    phiOut(producedSet, mPartitionProducedItemCountPhi, "partitionProduced");
    phiOut(consumedSet, mPartitionConsumedItemCountPhi, "partitionConsumed");

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief phiOutPartitionStatusFlags
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::phiOutPartitionStatusFlags(BuilderRef b, const unsigned targetPartitionId, const bool fromKernelEntry, BasicBlock * const exitBlock) {

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
            term = mPartitionTerminationSignal[partitionId];
        } else {
            term = readTerminationSignal(b, partitionId);
        }
        termPhi->addIncoming(term, exitBlock);
    }

    PHINode * const progressPhi = mPartitionPipelineProgressPhi[targetPartitionId];
    Value * const progress = fromKernelEntry ? mPipelineProgress : b->getTrue();
    progressPhi->addIncoming(progress, exitBlock);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeInitiallyTerminatedPartitionExit
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeInitiallyTerminatedPartitionExit(BuilderRef b) {

    loadLastGoodVirtualBaseAddressesOfUnownedBuffersInPartition(b);
    BasicBlock * const exitBlock = b->GetInsertBlock();
    mExhaustedInputAtJumpPhi->addIncoming(mExhaustedInput, exitBlock);
    mKernelInitiallyTerminatedExit = exitBlock;
    b->CreateBr(mKernelJumpToNextUsefulPartition);

#if 0

    // create a temporary basic block to act a constant exit block of any phi nodes;
    // once we've determined the actual exit block of this portion of the program,
    // replace the phi catch with it.
    BasicBlock * phiCatchBlock = b->CreateBasicBlock();

    const auto nextPartitionId = mCurrentPartitionId + 1U;

    PHINode * const exhaustedInputPhi = mExhaustedPipelineInputAtPartitionEntry[nextPartitionId];
    exhaustedInputPhi->addIncoming(mExhaustedInput, phiCatchBlock);

    for (auto kernel = FirstKernel; kernel <= mKernelId; ++kernel) {
        phiOutPartitionItemCounts(b, kernel, nextPartitionId, true, phiCatchBlock);
    }

    for (auto kernel = mKernelId + 1U; kernel <= LastKernel; ++kernel) {
        if (KernelPartitionId[kernel] != mCurrentPartitionId) {
            break;
        }
        phiOutPartitionItemCounts(b, kernel, nextPartitionId, true, phiCatchBlock);
    }

    phiOutPartitionStatusFlags(b, nextPartitionId, true, phiCatchBlock);

    releaseSynchronizationLock(b, mKernelId, LockType::Segment);

    replacePhiCatchWithCurrentBlock(b, phiCatchBlock, mNextPartitionEntryPoint);

    b->CreateBr(mNextPartitionEntryPoint);

#endif
}



/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeOnInitialTerminationJumpToNextPartitionToCheck
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeOnInitialTerminationJumpToNextPartitionToCheck(BuilderRef b) {

    const auto nextPartitionId = mPartitionJumpIndex[mCurrentPartitionId];
    assert (mCurrentPartitionId < nextPartitionId);

    BasicBlock * exitBlock = b->CreateBasicBlock();
    for (auto kernel = FirstKernel; kernel <= mKernelId; ++kernel) {
        phiOutPartitionItemCounts(b, kernel, nextPartitionId, false, exitBlock);
    }
    for (auto kernel = mKernelId + 1U; kernel <= LastKernel; ++kernel) {
        if (mPartitionJumpIndex[KernelPartitionId[kernel]] > nextPartitionId) {
            break;
        }
        phiOutPartitionItemCounts(b, kernel, nextPartitionId, false, exitBlock);
    }

    phiOutPartitionStatusFlags(b, nextPartitionId, false, exitBlock);
    PHINode * const exhaustedInputPhi = mExhaustedPipelineInputAtPartitionEntry[nextPartitionId];
    if (exhaustedInputPhi) {
        Value * const exhausted = mIsBounded ? mExhaustedInputAtJumpPhi : mExhaustedInput;
        exhaustedInputPhi->addIncoming(exhausted, exitBlock); assert (exhausted);
    }

    // Find the first kernel in the partition we're jumping to and acquire the LSN
    // then release all of the kernels we skipped over.
    auto firstKernelInNextPartition = mKernelId + 1U;
    for (; firstKernelInNextPartition <= LastKernel; ++firstKernelInNextPartition) {
        if (KernelPartitionId[firstKernelInNextPartition] == nextPartitionId) {
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

    replacePhiCatchWithCurrentBlock(b, exitBlock, mNextPartitionWithPotentialInput);

    b->CreateBr(mNextPartitionWithPotentialInput);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForPartitionExit
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::checkForPartitionExit(BuilderRef b) {

    assert (mKernelId >= FirstKernel && mKernelId <= LastKernel);
    const auto nextPartitionId = KernelPartitionId[mKernelId + 1];

    mPartitionTerminationSignal[mCurrentPartitionId] = mTerminatedAtExitPhi;

    if (nextPartitionId != mCurrentPartitionId) {
        assert (mCurrentPartitionId < nextPartitionId);
        assert (nextPartitionId < PartitionCount);
        BasicBlock * const exitBlock = b->GetInsertBlock();
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
                const ConsumerNode & cn = mConsumerGraph[streamSet]; assert (cn.Consumed);
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

        for (unsigned i = 0; i != nextPartitionId; ++i) {
            PHINode * const termPhi = mPartitionTerminationSignalPhi[nextPartitionId][i];
            if (termPhi) {
                termPhi->addIncoming(mPartitionTerminationSignal[i], exitBlock);
                mPartitionTerminationSignal[i] = termPhi;
            }
        }


    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setCurrentPartitionTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::setCurrentPartitionTerminationSignal(Value * const signal) {
    const auto partitionId = KernelPartitionId[mKernelId];
    mPartitionTerminationSignal[partitionId] = signal;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getCurrentPartitionTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getCurrentPartitionTerminationSignal() const {
    const auto partitionId = KernelPartitionId[mKernelId];
    return mPartitionTerminationSignal[partitionId];
}

}

#endif
