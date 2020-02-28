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
    mPartitionTerminationAtJumpExitSignal.resize(PartitionCount, nullptr);

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
    mPartitionTerminationAtJumpExitSignal[mCurrentPartitionId] = mTerminatedInitially;

    PHINode * const p = mPipelineProgressAtPartitionExit[jumpId]; assert (p);
    p->addIncoming(mPipelineProgress, exitBlock);

#if 0

    // pass any previously resolved termination states
    for (const auto e : make_iterator_range(in_edges(jumpId, mPartitionJumpGraph))) {
        PHINode * const p = mPartitionJumpGraph[e].Phi; assert (p);
        // If we have not yet executed the partition, assume it inherits this
        // partition's termination signal.
        Value * val = nullptr;
        const auto i = source(e, mPartitionJumpGraph);
        if (i < mCurrentPartitionId) {
            val = mPartitionTerminationSignal[i]; assert (val);
        } else {
            val = mTerminatedInitially;
        }
        p->addIncoming(val, exitBlock);


//        const auto numOfOutputs = getNumOfStreamOutputs(mKernelId);
//        for (unsigned i = 0; i < numOfOutputs; ++i) {
//            const auto port = StreamSetPort{ PortType::Output, i };
//            const auto prefix = makeBufferName(mKernelId, port);
//            PHINode * const fullyProduced = b->CreatePHI(sizeTy, 2, prefix + "_fullyProducedAtKernelExit");
//            mFullyProducedItemCount(port) = fullyProduced;
//        }

    }

#endif

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
        // When entering here, mPartitionTerminationSignal[mCurrentPartitionId] will pointer to the initially
        // terminated state. Update it to reflect the state upon exiting this partition. Depending on which
        // partitions jump into the next partition, we may end up immediately phi-ing it out. Thus this must
        // be done before the subsequent phi-out loop.
        mPartitionTerminationSignal[mCurrentPartitionId] = mTerminatedAtExitPhi;
        mPartitionExitPoint[mCurrentPartitionId] = exitBlock;
        b->CreateBr(nextPartition);

        b->SetInsertPoint(nextPartition);
        PHINode * const p = mPipelineProgressAtPartitionExit[nextPartitionId]; assert (p);
        p->addIncoming(mHasProgressedPhi, exitBlock);
        assert (mTerminatedAtExitPhi);
        mPipelineProgress = p;


        const auto n = in_degree(nextPartitionId, mPartitionJumpGraph);

        if (n) {

            std::vector<size_t> O;
            O.reserve(n);
            for (const auto e : make_iterator_range(in_edges(nextPartitionId, mPartitionJumpGraph))) {
                O.push_back(source(e, mBufferGraph));
            }
            std::sort(O.begin(), O.end());

            std::vector<Value *> P(n);
            std::vector<Value *> V(n);

            IntegerType * const sizeTy = b->getSizeTy();
            for (unsigned i = 0; i != n; ++i) {
                const auto k = O[i];
                PHINode * const p = b->CreatePHI(sizeTy, n + 1, std::to_string(k) + ".terminationSignal");
                for (unsigned j = i; j != n; ++j) {
                    V[j] = mPartitionTerminationAtJumpExitSignal[O[j]];
                }
                for (unsigned j = 0; j != n; ++j) {
                    p->addIncoming(V[j], mPartitionJumpPoint[O[j]]);
                }
                V[i] = mPartitionTerminationSignal[k];
                p->addIncoming(V[i], exitBlock);
                P[i] = p;
            }

            for (unsigned i = 0; i != n; ++i) {
                mPartitionTerminationSignal[O[i]] = P[i];
            }

        }



#if 0

        // When entering here, mPartitionTerminationSignal[mCurrentPartitionId] will pointer to the initially
        // terminated state. Update it to reflect the state upon exiting this partition. Depending on which
        // partitions jump into the next partition, we may end up immediately phi-ing it out. Thus this must
        // be done before the subsequent phi-out loop.
        mPartitionTerminationSignal[mCurrentPartitionId] = mTerminatedAtExitPhi;

        for (const auto e : make_iterator_range(in_edges(nextPartitionId, mPartitionJumpGraph))) {
            PHINode * const p = mPartitionJumpGraph[e].Phi; assert (p);
            const auto i = source(e, mPartitionJumpGraph);
            Value * const v = mPartitionTerminationSignal[i]; assert(v);
            assert (p != v);
            p->addIncoming(v, exitBlock);
            mPartitionTerminationSignal[i] = p;




        }

#endif

    }
}

}

#endif
