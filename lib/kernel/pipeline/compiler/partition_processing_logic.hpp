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

    mPartitionJumpExitPoint.resize(PartitionCount);
    mPartitionExitPoint.resize(PartitionCount);

    for (unsigned i = 0; i < PartitionCount; ++i) {
        mPartitionEntryPoint[i] = b->CreateBasicBlock("Partition" + std::to_string(i));
    }

    mPipelineProgressAtPartitionExit.resize(PartitionCount, nullptr);

    mPartitionTerminationSignal.resize(PartitionCount, nullptr);
    mPartitionTerminationSignalAtJumpExit.resize(PartitionCount, nullptr);

    mExhaustedPipelineInputAtPartitionEntry.resize(PartitionCount, nullptr);

    const auto ip = b->saveIP();
    IntegerType * const boolTy = b->getInt1Ty();
    for (unsigned i = 1; i < PartitionCount; ++i) {
        b->SetInsertPoint(mPartitionEntryPoint[i]);
        mPipelineProgressAtPartitionExit[i] = b->CreatePHI(boolTy, PartitionCount, std::to_string(i) + ".pipelineProgress");
        // if we have some partition input and at least one partition jumps into this one,
        // create a phi node to store whether we've exhausted the pipeline's input data
        if (in_degree(i, mPartitionJumpTree) > 0) {
            mExhaustedPipelineInputAtPartitionEntry[i] = b->CreatePHI(boolTy, PartitionCount, std::to_string(i) + ".exhaustedInput");
        }
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

    // Create our branch to our next partition that may have input early so that we always have
    // and insertion point later to introduce any new phi-d variables. We move it to the end of
    // the block afterwards.
    BranchInst * const br = b->CreateBr(mNextPartitionWithPotentialInput);

    const auto jumpId = mPartitionJumpIndex[mCurrentPartitionId];

    // When jumping out of a partition to some subsequent one, we may have to
    // phi-out some of the produced item counts. We 3 cases to consider:

    // (1) if we've executed the kernel, we use the fully produced item count.
    // (2) if producer is the current kernel, we use the already produced phi node.
    // (3) if we have yet to execute (and will be jumping over) the kernel, load
    // the prior produced count.

    Instruction * const insertPoint = b->GetInsertBlock()->getFirstNonPHI(); assert (insertPoint);

    for (auto kernel = FirstKernel; kernel <= LastKernel; ++kernel) {
        if ((kernel < mKernelId) || mPartitionJumpIndex[KernelPartitionId[kernel]] == jumpId) {

            for (const auto e : make_iterator_range(out_edges(kernel, mBufferGraph))) {
                const auto streamSet = target(e, mBufferGraph);

                // If the consumer of this stream set is dominated by the jump target,
                // we'll need to store the produced item count for later use.
                bool prepareProducedPhi = false;
                for (const auto f : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
                    const auto consumer = target(f, mBufferGraph);
                    const auto p = KernelPartitionId[consumer];
                    if (p >= jumpId) {
                        prepareProducedPhi = true;
                        break;
                    }
                }
                if (prepareProducedPhi) {
                    const BufferRateData & br = mBufferGraph[e];
                    // Select/compute/load the appropriate produced item count
                    Value * produced = nullptr;
                    if (kernel < mKernelId) {
                        produced = mLocallyAvailableItems[getBufferIndex(streamSet)];
                    } else {
                        const Binding & output = br.Binding;
                        const auto deferred = output.isDeferred();
                        if (kernel == mKernelId) {

                            Value * intermediateProducedPhi = nullptr;
                            Value * initialProducedPhi = nullptr;
                            if (LLVM_UNLIKELY(deferred)) {
                                intermediateProducedPhi = mAlreadyProducedDeferredPhi(br.Port);
                                initialProducedPhi = mInitiallyProducedDeferredItemCount(br.Port);
                            } else {
                                intermediateProducedPhi = mAlreadyProducedPhi(br.Port);
                                initialProducedPhi = mInitiallyProducedItemCount[streamSet];
                            }

                            IntegerType * const sizeTy = b->getSizeTy();
                            PHINode * const producedPhi = PHINode::Create(sizeTy, 2, "", insertPoint);
                            if (mKernelInsufficientInputExit) {
                                producedPhi->addIncoming(intermediateProducedPhi, mKernelInsufficientInputExit);
                            }
                            if (mKernelInitiallyTerminatedExit) {
                                producedPhi->addIncoming(initialProducedPhi, mKernelInitiallyTerminatedExit);
                            }
                            produced = producedPhi;
                        } else {
                            const auto prefix = makeBufferName(kernel, br.Port);
                            if (LLVM_UNLIKELY(deferred)) {
                                produced = b->getScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX);
                            } else {
                                produced = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
                            }
                        }
                        produced = computeFullyProducedItemCount(b, kernel, br.Port, produced, mTerminatedInitially);                        
                    }
                    // Store the produced item count at the point we're jumping out of the partition
                    mPartitionProducedItemCountAtJumpExit.emplace(std::make_pair(streamSet, mCurrentPartitionId), produced);
                }

                bool prepareConsumedPhi = false;
                for (const auto f : make_iterator_range(out_edges(streamSet, mConsumerGraph))) {
                    const ConsumerEdge & c = mConsumerGraph[f];
                    if (c.Flags == ConsumerEdge::None) continue;
                    const auto consumer = target(f, mConsumerGraph);
                    const auto p = KernelPartitionId[consumer];
                    if (p >= jumpId) {
                        prepareConsumedPhi = true;
                        break;
                    }
                }
                if (prepareConsumedPhi) {
                    Value * consumed = nullptr;
                    if (kernel <= mKernelId) {
                        consumed = mConsumedItemCount[streamSet];
                    } else {
                        consumed = readConsumedItemCount(b, streamSet);
                    }
                    // Store the produced item count at the point we're jumping out of the partition
                    mPartitionConsumedItemCountAtJumpExit.emplace(std::make_pair(streamSet, mCurrentPartitionId), consumed);
                }
            }
        }
    }

    BasicBlock * const exitBlock = b->GetInsertBlock();

    mPartitionJumpExitPoint[mCurrentPartitionId] = exitBlock;
    mPartitionTerminationSignalAtJumpExit[mCurrentPartitionId] = mTerminatedInitially;

    PHINode * const p = mPipelineProgressAtPartitionExit[jumpId]; assert (p);
    p->addIncoming(mPipelineProgress, exitBlock);

    PHINode * const exhaustedInputPhi = mExhaustedPipelineInputAtPartitionEntry[jumpId];
    if (exhaustedInputPhi) {
        IntegerType * const boolTy = b->getInt1Ty();
        PHINode * const phi = PHINode::Create(boolTy, 2, "", insertPoint);
        if (mKernelInsufficientInputExit) {
            phi->addIncoming(mExhaustedPipelineInputPhi, mKernelInsufficientInputExit);
        }
        if (mKernelInitiallyTerminatedExit) {
            phi->addIncoming(mExhaustedInput, mKernelInitiallyTerminatedExit);
        }
        exhaustedInputPhi->addIncoming(phi, exitBlock);
    }

    // Move the branch to the end of the block
    Instruction * const last = &exitBlock->back();
    if (br != last) {
        br->moveAfter(last);
    }

}




/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForPartitionExit
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::checkForPartitionExit(BuilderRef b) {
    using InEdgeIterator = graph_traits<PartitionJumpTree>::in_edge_iterator;

    assert (mKernelId >= FirstKernel && mKernelId <= LastKernel);
    const auto nextPartitionId = KernelPartitionId[mKernelId + 1];

    if (nextPartitionId != mCurrentPartitionId) {
        assert (mCurrentPartitionId < nextPartitionId);
        assert (nextPartitionId < PartitionCount);
        BasicBlock * const nextPartition = mPartitionEntryPoint[nextPartitionId];
        BasicBlock * const exitBlock = b->GetInsertBlock();
        mPartitionExitPoint[mCurrentPartitionId] = exitBlock;
        b->CreateBr(nextPartition);

        b->SetInsertPoint(nextPartition);
        PHINode * const p = mPipelineProgressAtPartitionExit[nextPartitionId]; assert (p);
        p->addIncoming(mAnyProgressedAtLoopExitPhi, exitBlock);
        assert (mTerminatedAtExitPhi);
        mPipelineProgress = p;

        PHINode * const exhaustedInputPhi = mExhaustedPipelineInputAtPartitionEntry[nextPartitionId];
        if (exhaustedInputPhi) {
            exhaustedInputPhi->addIncoming(mExhaustedInput, exitBlock); assert (mExhaustedInput);
            mExhaustedInput = exhaustedInputPhi;
        }

        // When entering here, mPartitionTerminationSignal[mCurrentPartitionId] will pointer to the initially
        // terminated state. Update it to reflect the state upon exiting this partition. Depending on which
        // partitions jump into the next partition, we may end up immediately phi-ing it out. Thus this must
        // be done before the subsequent phi-out loop.
        mPartitionTerminationSignal[mCurrentPartitionId] = mTerminatedAtExitPhi;

        const auto n = in_degree(nextPartitionId, mPartitionJumpTree);

        if (n) {

            // Gather all of the incoming non-local streams for any dominated partitions
            flat_set<unsigned> streamSets;
            const auto nextJumpId = mPartitionJumpIndex[nextPartitionId];
            for (auto k = (mKernelId + 1); k <= LastKernel; ++k) {
                if (KernelPartitionId[k] == nextJumpId) {
                    break;
                }
                for (const auto e : make_iterator_range(in_edges(k, mBufferGraph))) {
                    const auto streamSet = source(e, mBufferGraph);
                    const auto producer = parent(streamSet, mBufferGraph);
                    const auto partitionId = KernelPartitionId[producer];
                    // Since every partition is numbered in order of execution, if the
                    // producer's partition is < the partition after the current kernel,
                    // then we need to phi it out.
                    if (partitionId < nextPartitionId) {
                        streamSets.emplace(streamSet);
                    }
                }
            }


            IntegerType * const sizeTy = b->getSizeTy();
            InEdgeIterator begin, end;
            std::tie(begin, end) = in_edges(nextPartitionId, mPartitionJumpTree);

            // Phi-out all of the produced item counts
            for (const auto streamSet : streamSets) {

                PHINode * const producedPhi = b->CreatePHI(sizeTy, n + 1);
                for (auto ej = begin; ej != end; ++ej) {
                    const auto j = source(*ej, mPartitionJumpTree);
                    const auto f = mPartitionProducedItemCountAtJumpExit.find(std::make_pair(streamSet, j));
                    assert (f != mPartitionProducedItemCountAtJumpExit.end());
                    Value * const incomingValue = f->second;
                    BasicBlock * const exit = mPartitionJumpExitPoint[j];
                    producedPhi->addIncoming(incomingValue, exit);
                }

                const auto k = getBufferIndex(streamSet);
                Value * const avail = mLocallyAvailableItems[k];
                producedPhi->addIncoming(avail, exitBlock);
                mLocallyAvailableItems[k] = producedPhi;
            }

            streamSets.clear();

            for (auto k = (mKernelId + 1); k <= LastKernel; ++k) {
                if (KernelPartitionId[k] == nextJumpId) {
                    break;
                }
                for (const auto e : make_iterator_range(in_edges(k, mConsumerGraph))) {
                    const ConsumerEdge & c = mConsumerGraph[e];
                    if (c.Flags == ConsumerEdge::None) continue;
                    const auto streamSet = source(e, mConsumerGraph);
                    const auto producer = parent(streamSet, mConsumerGraph);
                    const auto partitionId = KernelPartitionId[producer];
                    // Since every partition is numbered in order of execution, if the
                    // producer's partition is < the partition after the current kernel,
                    // then we need to phi it out.
                    if (partitionId < nextPartitionId) {
                        streamSets.emplace(streamSet);
                    }
                }
            }

            // Phi-out all of the consumed item counts
            for (const auto streamSet : streamSets) {
                PHINode * const producedPhi = b->CreatePHI(sizeTy, n + 1);
                for (auto ej = begin; ej != end; ++ej) {
                    const auto j = source(*ej, mPartitionJumpTree);
                    const auto f = mPartitionConsumedItemCountAtJumpExit.find(std::make_pair(streamSet, j));
                    assert (f != mPartitionConsumedItemCountAtJumpExit.end());
                    Value * const incomingValue = f->second;
                    BasicBlock * const exit = mPartitionJumpExitPoint[j];
                    producedPhi->addIncoming(incomingValue, exit);
                }
                const ConsumerNode & cn = mConsumerGraph[streamSet];
                producedPhi->addIncoming(cn.Consumed, exitBlock);
                cn.Consumed = producedPhi;
            }

            // Phi out the termination signals
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


                    PHINode * const termSignalPhi = b->CreatePHI(sizeTy, n + 1, std::to_string(k) + ".terminationSignal");
                    // If we're phi-ing out a value of a partition when when it was
                    // "jumped over", we inherit the termination signal of the prior
                    // partition.
                    for (auto ej = begin; ej != ei; ++ej) {
                        const auto j = source(*ej, mPartitionJumpTree);
                        BasicBlock * const exit = mPartitionJumpExitPoint[j];
                        termSignalPhi->addIncoming(mPartitionTerminationSignalAtJumpExit[j], exit);

                    }

                    // NOTE: "i" is intentional as i is the value from the outer for loop.
                    termSignalPhi->addIncoming(mPartitionTerminationSignalAtJumpExit[i], mPartitionJumpExitPoint[i]);

                    // If we have successfully executed the partition, we use its actual
                    // termination signal.
                    Value * const termSignal = mPartitionTerminationSignal[k];
                    for (auto ej = ei + 1; ej != end; ++ej) {
                        const auto j = source(*ej, mPartitionJumpTree);
                        termSignalPhi->addIncoming(termSignal, mPartitionJumpExitPoint[j]);
                    }
                    // Finally phi-out the "successful" signal
                    termSignalPhi->addIncoming(termSignal, exitBlock);
                    mPartitionTerminationSignal[k] = termSignalPhi;
                };

                phiOutPredecessors(i);

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
