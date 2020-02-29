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
    mPartitionEntryPoint.resize(PartitionCount + 1);

    mPartitionJumpPoint.resize(PartitionCount);
    mPartitionExitPoint.resize(PartitionCount);

    for (unsigned i = 0; i <= PartitionCount; ++i) {
        mPartitionEntryPoint[i] = b->CreateBasicBlock("Partition" + std::to_string(i));
    }

    mPipelineProgressAtPartitionExit.resize(PartitionCount + 1, nullptr);

    mPartitionTerminationSignalPhi.resize(PartitionCount, nullptr);
    mPartitionTerminationSignal.resize(PartitionCount, nullptr);
    mPartitionTerminationSignalAtJumpExit.resize(PartitionCount, nullptr);

    const auto ip = b->saveIP();

    IntegerType * const boolTy = b->getInt1Ty();
    for (unsigned i = 1; i <= PartitionCount; ++i) {
        b->SetInsertPoint(mPartitionEntryPoint[i]);
        mPipelineProgressAtPartitionExit[i] = b->CreatePHI(boolTy, PartitionCount, std::to_string(i) + ".pipelineProgress");
    }
#if 0
    IntegerType * const sizeTy = b->getSizeTy();
    for (const auto e : make_iterator_range(edges(mPartitionJumpGraph))) {
        const auto j = target(e, mPartitionJumpGraph);
        b->SetInsertPoint(mPartitionEntryPoint[j]);
        const auto i = source(e, mPartitionJumpGraph);
//        PHINode * const phi = b->CreatePHI(sizeTy, in_degree(j, mPartitionJumpGraph), std::to_string(i) + ".terminationSignal");
//        mPartitionJumpGraph[e].Phi = phi;
    }

//    for (unsigned i = 0; i < PartitionCount; ++i) {
//        const auto j = mPartitionJumpIndex[i];
//        b->SetInsertPoint(mPartitionEntryPoint[j]);
//        mPartitionTerminationSignalJumpCombinePhi[i] = b->CreatePHI(sizeTy, PartitionCount, "terminationSignal" + std::to_string(i));
//    }
#endif
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
inline void PipelineCompiler::checkPartitionEntry(BuilderRef b) {
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


        const auto n = in_degree(mCurrentPartitionId, mPartitionJumpTree);


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
 * @brief jumpToNextPartition
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::jumpToNextPartition(BuilderRef b) {



}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeOnInitialTerminationJumpToNextPartitionToCheck
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeOnInitialTerminationJumpToNextPartitionToCheck(BuilderRef b) {
    if (mKernelIsInternallySynchronized) {
        releaseSynchronizationLock(b, LockType::ItemCheck);
        startCycleCounter(b, CycleCounter::BEFORE_SYNCHRONIZATION);
        acquireSynchronizationLock(b, LockType::Segment, CycleCounter::BEFORE_SYNCHRONIZATION);
    }
    releaseSynchronizationLock(b, LockType::Segment);

    const auto jumpId = mPartitionJumpIndex[mCurrentPartitionId];

//    auto lastKernel = mKernel + 1;
//    for (; lastKernel <= LastKernel; ++lastKernel) {
//        if (mPartitionJumpIndex[lastKernel] != jumpId) {
//            break;
//        }
//        readProducedItemCounts(b, lastKernel);
//    }

    BasicBlock * const exitBlock = b->GetInsertBlock();

    mPartitionJumpPoint[mCurrentPartitionId] = exitBlock;
    mPartitionTerminationSignalAtJumpExit[mCurrentPartitionId] = mTerminatedInitially;

    PHINode * const p = mPipelineProgressAtPartitionExit[jumpId]; assert (p);
    p->addIncoming(mPipelineProgress, exitBlock);

    // Finally jump to our next partition that may have input
    b->CreateBr(mNextPartitionWithPotentialInput);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForPartitionExit
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::checkForPartitionExit(BuilderRef b) {
    assert (mKernelId >= FirstKernel && mKernelId <= LastKernel);
    const auto nextPartitionId = KernelPartitionId[mKernelId + 1];

    if (nextPartitionId != mCurrentPartitionId) {
        assert (mCurrentPartitionId < nextPartitionId);
        assert (nextPartitionId <= PartitionCount);
        BasicBlock * const nextPartition = mPartitionEntryPoint[nextPartitionId];
        BasicBlock * const exitBlock = b->GetInsertBlock();
        mPartitionExitPoint[mCurrentPartitionId] = exitBlock;
        b->CreateBr(nextPartition);

        b->SetInsertPoint(nextPartition);
        PHINode * const p = mPipelineProgressAtPartitionExit[nextPartitionId]; assert (p);
        p->addIncoming(mHasProgressedPhi, exitBlock);
        assert (mTerminatedAtExitPhi);
        mPipelineProgress = p;

        // When entering here, mPartitionTerminationSignal[mCurrentPartitionId] will pointer to the initially
        // terminated state. Update it to reflect the state upon exiting this partition. Depending on which
        // partitions jump into the next partition, we may end up immediately phi-ing it out. Thus this must
        // be done before the subsequent phi-out loop.
        mPartitionTerminationSignal[mCurrentPartitionId] = mTerminatedAtExitPhi;


        const auto n = in_degree(nextPartitionId, mPartitionJumpTree);

        if (n) {

            using InEdgeIterator = graph_traits<PartitionJumpTree>::in_edge_iterator;

            IntegerType * const sizeTy = b->getSizeTy();

            InEdgeIterator begin, end;
            std::tie(begin, end) = in_edges(nextPartitionId, mPartitionJumpTree);

            unsigned prior_i = 0;
            for (auto ei = begin; ei != end; ++ei) {

                const auto i = source(*ei, mPartitionJumpTree);
                assert (prior_i <= i);
                prior_i = i;

                std::function<void(size_t)> phiOutPredecessors = [&](const size_t k) {

                    // Recurse to the leaves of the tree first to ensure we properly phi-out
                    // all of the nested partitions
                    for (const auto e : make_iterator_range(in_edges(k, mPartitionJumpTree))) {
                        phiOutPredecessors(source(e, mPartitionJumpTree));
                    }

                    PHINode * const p = b->CreatePHI(sizeTy, n + 1, std::to_string(k) + ".terminationSignal");

                    // If we're phi-ing out a value of a partition when when it was
                    // "jumped over", we inherit the termination signal of the prior
                    // partition.
                    for (auto ej = begin; ej != ei; ++ej) {
                        const auto j = source(*ej, mPartitionJumpTree);
                        p->addIncoming(mPartitionTerminationSignalAtJumpExit[j], mPartitionJumpPoint[j]);
                    }

                    // NOTE: "i" is intentional as i is the value from the outer for loop.
                    p->addIncoming(mPartitionTerminationSignalAtJumpExit[i], mPartitionJumpPoint[i]);

                    // If we have successfully executed the partition, we use its actual
                    // termination signal.
                    Value * const termSignal = mPartitionTerminationSignal[k];
                    for (auto ej = ei + 1; ej != end; ++ej) {
                        const auto j = source(*ej, mPartitionJumpTree);
                        p->addIncoming(termSignal, mPartitionJumpPoint[j]);
                    }
                    // Finally phi-out the "successful" signal
                    p->addIncoming(termSignal, exitBlock);
                    // mPartitionTerminationSignalAtJumpExit[k] = p;
                    mPartitionTerminationSignal[k] = p;
                };

                phiOutPredecessors(i);

            }
        }

    }
}

}

#endif
