#include "pipeline_compiler.hpp"
#include "kernel_logic.hpp"
#include "kernel_io_calculation_logic.hpp"
#include "kernel_execution_logic.hpp"
#include "kernel_family_logic.hpp"

// TODO: if we have multiple copies of the same type of kernel executing sequentially, we could avoid
// generating an "execution call" for each and instead pass in different handles/item counts. This
// could improve I-Cache utilization.

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief start
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::start(BuilderRef b) {

    makePartitionEntryPoints(b);

    if (mCheckAssertions) {
        mRethrowException = b->WriteDefaultRethrowBlock();
    }
    #ifdef PRINT_DEBUG_MESSAGES
    debugInit(b);
    if (ExternallySynchronized) {
        debugPrint(b, "------------------------------------------------- START %" PRIx64, getHandle());
    } else {
        debugPrint(b, "================================================= START %" PRIx64, getHandle());
    }
    #endif
    loadInternalStreamSetHandles(b);
    readExternalConsumerItemCounts(b);
    mKernel = nullptr;
    mKernelId = 0;
    BasicBlock * const entryBlock = b->GetInsertBlock();
    b->CreateBr(mPipelineLoop);

    b->SetInsertPoint(mPipelineLoop);
    mMadeProgressInLastSegment = b->CreatePHI(b->getInt1Ty(), 2, "madeProgressInLastSegment");
    mMadeProgressInLastSegment->addIncoming(b->getTrue(), entryBlock);
    Constant * const i1_FALSE = b->getFalse();
    mPipelineProgress = i1_FALSE;
    mExhaustedInput = i1_FALSE;
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = mTarget->getName();
    #endif
    mSegNo = mExternalSegNo;
    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, prefix + " +++ NUM OF STRIDES %" PRIu64 "+++", mNumOfStrides);
    debugPrint(b, prefix + " +++ IS FINAL %" PRIu8 "+++", mIsFinal);
    #endif
    branchToInitialPartition(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief executeRegularKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::executeKernel(BuilderRef b) {

    mNumOfAddressableItemCount = 0;
    mNumOfVirtualBaseAddresses = 0;
    mNumOfTruncatedInputBuffers = 0;

    mHasZeroExtendedInput = nullptr;
    mZeroExtendBufferPhi = nullptr;
    mExhaustedPipelineInputAtExit = mExhaustedInput;

    assert (mKernelId >= FirstKernel);
    assert (mKernelId <= LastKernel);

    BasicBlock * const partitionExit = getPartitionExitPoint(b);

    identifyPipelineInputs();

    mMaximumNumOfStrides = ceiling(MaximumNumOfStrides[mKernelId]);

    checkPartitionEntry(b);

    const auto prefix = makeKernelName(mKernelId);

    /// -------------------------------------------------------------------------------------
    /// KERNEL ENTRY
    /// -------------------------------------------------------------------------------------

    if (!mKernelIsInternallySynchronized) {
        // By using an atomic fetch/add here, we gain the ability to dynamically add or
        // remove threads while still using the segment pipeline parallelism model.
        Value * const segNoPtr = b->getScalarFieldPtr(prefix + NEXT_LOGICAL_SEGMENT_SUFFIX);
        mSegNo = b->CreateAtomicFetchAndAdd(b->getSize(1), segNoPtr);
    }


    const auto initialLock = mKernelIsInternallySynchronized ? LockType::ItemCheck : LockType::Segment;
    acquireSynchronizationLock(b, initialLock, CycleCounter::INITIAL);
    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, "+++ " + prefix + "_segNo = %" PRIu64, mSegNo);
    #endif

    mKernelCanTerminateEarly = mKernel->canSetTerminateSignal();
    mHasExplicitFinalPartialStride = mKernel->requiresExplicitPartialFinalStride();
    mMayHaveNonLinearIO = mayHaveNonLinearIO(mKernelId);
    mKernelIsInternallySynchronized = supportsInternalSynchronization();
    assert ("non-partition-root kernel can terminate early?" && (!mKernelCanTerminateEarly || mIsPartitionRoot));
    mIsBounded = isBounded();

    mKernelLoopEntry = b->CreateBasicBlock(prefix + "_loopEntry", partitionExit);
    mKernelCheckOutputSpace = b->CreateBasicBlock(prefix + "_checkOutputSpace", partitionExit);
    mKernelLoopCall = b->CreateBasicBlock(prefix + "_executeKernel", partitionExit);
    mKernelCompletionCheck = b->CreateBasicBlock(prefix + "_normalCompletionCheck", partitionExit);
    mKernelInsufficientIOExit = b->CreateBasicBlock(prefix + "_insufficientIOExit", partitionExit);
    mKernelTerminated = nullptr;
    mKernelInitiallyTerminated = nullptr;
    mKernelInitiallyTerminatedPhiCatch = nullptr;
    if (mIsPartitionRoot) {
        mKernelTerminated = b->CreateBasicBlock(prefix + "_terminated", partitionExit);
        mKernelInitiallyTerminated = b->CreateBasicBlock(prefix + "_initiallyTerminated", partitionExit);
        mKernelInitPartitionJump = b->CreateBasicBlock(prefix + "_initPartitionJump", partitionExit);
    }
    mKernelLoopExit = b->CreateBasicBlock(prefix + "_loopExit", partitionExit);
    // The phi catch simplifies compilation logic by "forward declaring" the loop exit point.
    // Subsequent optimization phases will collapse it into the correct exit block.
    mKernelLoopExitPhiCatch = b->CreateBasicBlock(prefix + "_kernelExitPhiCatch", partitionExit);
    mKernelExit = b->CreateBasicBlock(prefix + "_kernelExit", partitionExit);


    readInitialItemCounts(b);
    readConsumedItemCounts(b);
    prepareLinearBuffers(b);

    incrementNumberOfSegmentsCounter(b);
    recordUnconsumedItemCounts(b);

    mKernelEntry = b->GetInsertBlock();

    if (mIsPartitionRoot) {
        Value * const terminated = initiallyTerminated(b);
        b->CreateUnlikelyCondBr(terminated, mKernelInitiallyTerminated, mKernelLoopEntry);
    } else {
        b->CreateBr(mKernelLoopEntry);
    }

    /// -------------------------------------------------------------------------------------
    /// PHI NODE INITIALIZATION
    /// -------------------------------------------------------------------------------------

    // Set up some PHI nodes early to simplify accumulating their incoming values.
    initializeKernelLoopEntryPhis(b);
    initializeKernelCheckOutputSpacePhis(b);
    if (mIsPartitionRoot) {
        initializeKernelTerminatedPhis(b);
    }
    initializeKernelInsufficientIOExitPhis(b);
    initializeKernelLoopExitPhis(b);
    initializeKernelExitPhis(b);

    /// -------------------------------------------------------------------------------------
    /// KERNEL LOOP ENTRY
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopEntry);
    determineNumOfLinearStrides(b);

    /// -------------------------------------------------------------------------------------
    /// KERNEL CALL
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopCall);
    writeLookBehindLogic(b);
    writeKernelCall(b);
    writeCopyBackLogic(b);
    writeLookBehindReflectionLogic(b);

    /// -------------------------------------------------------------------------------------
    /// KERNEL EXPLICIT TERMINATION CHECK
    /// -------------------------------------------------------------------------------------

    if (mIsPartitionRoot && mKernelCanTerminateEarly) {

        Value * const aborted = b->CreateIsNotNull(mTerminatedExplicitly);
        BasicBlock * const explicitTermination =
            b->CreateBasicBlock(prefix + "_explicitTermination", mKernelCompletionCheck);

        b->CreateUnlikelyCondBr(aborted, explicitTermination, mKernelCompletionCheck);

        b->SetInsertPoint(explicitTermination);
        // If the kernel explicitly terminates, it must set its processed/produced item counts.
        // Otherwise, the pipeline will update any countable rates, even upon termination.
        readCountableItemCountsAfterAbnormalTermination(b);
        signalAbnormalTermination(b);
        b->CreateBr(mKernelTerminated);

    } else { // kernel cannot terminate early

        b->CreateBr(mKernelCompletionCheck);
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL NORMAL COMPLETION CHECK
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelCompletionCheck);
    normalCompletionCheck(b);

    /// -------------------------------------------------------------------------------------
    /// KERNEL TERMINATED
    /// -------------------------------------------------------------------------------------

    if (LLVM_UNLIKELY(mIsPartitionRoot)) {
        b->SetInsertPoint(mKernelTerminated);
        writeTerminationSignal(b, mTerminatedSignalPhi);
        clearUnwrittenOutputData(b);
        updatePhisAfterTermination(b);
        b->CreateBr(mKernelLoopExit);
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL INSUFFICIENT IO EXIT
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelInsufficientIOExit);
    writeInsufficientIOExit(b);

    /// -------------------------------------------------------------------------------------
    /// KERNEL LOOP EXIT
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopExit);
    updateTerminationSignal(mTerminatedAtLoopExitPhi);
    if (LLVM_LIKELY(!mKernelIsInternallySynchronized)) {
        writeUpdatedItemCounts(b, ItemCountSource::UpdatedItemCountsFromLoopExit);
    }
    computeFullyProcessedItemCounts(b);
    computeMinimumConsumedItemCounts(b);
    writeLookAheadLogic(b);
    computeFullyProducedItemCounts(b);
    replacePhiCatchBlocksWith(mKernelLoopExitPhiCatch, b->GetInsertBlock());
    b->CreateBr(mKernelExit);

    /// -------------------------------------------------------------------------------------
    /// KERNEL INITIALLY TERMINATED EXIT
    /// -------------------------------------------------------------------------------------

    if (mIsPartitionRoot) {
        b->SetInsertPoint(mKernelInitiallyTerminated);
        #ifdef PRINT_DEBUG_MESSAGES
        debugPrint(b, "* " + prefix + ".initiallyTerminated = %" PRIu64, mSegNo);
        #endif
        loadLastGoodVirtualBaseAddressesOfUnownedBuffersInPartition(b);
        b->CreateBr(mKernelInitPartitionJump);
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL PREPARE FOR PARTITION JUMP
    /// -------------------------------------------------------------------------------------

    if (mIsPartitionRoot) {
        b->SetInsertPoint(mKernelInitPartitionJump);
        writeInitParitionJump(b);
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL EXIT (CONTINUED)
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelExit);
    updateTerminationSignal(mTerminatedAtExitPhi);
    writeFinalConsumedItemCounts(b);
    recordFinalProducedItemCounts(b);
    recordStridesPerSegment(b);
    recordProducedItemCountDeltas(b);
    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, "* " + prefix + ".madeProgress = %" PRIu8, mPipelineProgress);
    #endif
    // chain the progress state so that the next one carries on from this one
    mExhaustedInput = mExhaustedPipelineInputAtExit;
    if (mIsPartitionRoot) {
        mNumOfPartitionStrides = mTotalNumOfStridesAtExitPhi;
    }
    releaseSynchronizationLock(b, LockType::Segment);
    checkForPartitionExit(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief replacePhiCatchBlocksWith
 *
 * replace the phi catch with the actual exit blocks
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::replacePhiCatchBlocksWith(BasicBlock *& from, BasicBlock * const to) {
    // NOTE: not all versions of LLVM seem to have BasicBlock::replacePhiUsesWith or PHINode::replaceIncomingBlockWith.
    // This code could be made to use those instead.
    for (Instruction & inst : *mKernelExit) {
        PHINode & pn = cast<PHINode>(inst);
        for (unsigned i = 0; i != pn.getNumIncomingValues(); ++i) {
            if (pn.getIncomingBlock(i) == from) {
                pn.setIncomingBlock(i, to);
            }
        }
    }
    from->eraseFromParent();
    from = to;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief normalCompletionCheck
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::normalCompletionCheck(BuilderRef b) {

    BasicBlock * const entryBlock = b->GetInsertBlock();

    const auto numOfInputs = getNumOfStreamInputs(mKernelId);
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelId);

    ConstantInt * const i1_TRUE = b->getTrue();

    if (mIsBounded) {

        if (mIsPartitionRoot) {

            for (unsigned i = 0; i < numOfInputs; ++i) {
                const auto port = StreamSetPort{ PortType::Input, i };
                mFinalProcessedPhi(port)->addIncoming(mProcessedItemCount(port), entryBlock);
            }

            for (unsigned i = 0; i < numOfOutputs; ++i) {
                const auto port = StreamSetPort{ PortType::Output, i };
                mFinalProducedPhi(port)->addIncoming(mProducedItemCount(port), entryBlock);
            }

            mTerminatedSignalPhi->addIncoming(mIsFinalInvocationPhi, entryBlock);

        }

        Value * const notFinal = b->CreateIsNull(mIsFinalInvocationPhi);

        if (mMayHaveNonLinearIO) {

            for (unsigned i = 0; i < numOfInputs; ++i) {
                const auto port = StreamSetPort{ PortType::Input, i };
                mAlreadyProcessedPhi(port)->addIncoming(mProcessedItemCount(port), entryBlock);
                if (mAlreadyProcessedDeferredPhi(port)) {
                    mAlreadyProcessedDeferredPhi(port)->addIncoming(mProcessedDeferredItemCount(port), entryBlock);
                }
            }

            for (unsigned i = 0; i < numOfOutputs; ++i) {
                const auto port = StreamSetPort{ PortType::Output, i };
                mAlreadyProducedPhi(port)->addIncoming(mProducedItemCount(port), entryBlock);
                if (mAlreadyProducedDeferredPhi(port)) {
                    mAlreadyProducedDeferredPhi(port)->addIncoming(mProducedDeferredItemCount(port), entryBlock);
                }
            }

            BasicBlock * const exitBlock = mIsPartitionRoot ? mKernelTerminated : mKernelLoopExit; assert (exitBlock);

            b->CreateCondBr(notFinal, mKernelLoopEntry, exitBlock);

            mAlreadyProgressedPhi->addIncoming(i1_TRUE, entryBlock);
            mExecutedAtLeastOncePhi->addIncoming(i1_TRUE, entryBlock);
            mCurrentNumOfStrides->addIncoming(mUpdatedNumOfStrides, entryBlock);

            // If we observe termination, return rather than phi-ing out the values for the
            // loop exit block. They will be handled in the termination block.
            if (mIsPartitionRoot) {
                return;
            }

        } else if (mIsPartitionRoot) {
            b->CreateLikelyCondBr(notFinal, mKernelLoopExit, mKernelTerminated);
        } else {
            b->CreateBr(mKernelLoopExit);
        }
    } else { // just exit the loop
        b->CreateBr(mKernelLoopExit);
    }

    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto port = StreamSetPort{ PortType::Input, i };
        mUpdatedProcessedPhi(port)->addIncoming(mProcessedItemCount(port), entryBlock);
        if (mUpdatedProcessedDeferredPhi(port)) {
            mUpdatedProcessedDeferredPhi(port)->addIncoming(mProcessedDeferredItemCount(port), entryBlock);
        }
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto port = StreamSetPort{ PortType::Output, i };
        mUpdatedProducedPhi(port)->addIncoming(mProducedItemCount(port), entryBlock);
        if (mUpdatedProducedDeferredPhi(port)) {
            mUpdatedProducedDeferredPhi(port)->addIncoming(mProducedDeferredItemCount(port), entryBlock);
        }
    }

    mTerminatedAtLoopExitPhi->addIncoming(mIsFinalInvocationPhi, entryBlock);
    mHasProgressedPhi->addIncoming(i1_TRUE, entryBlock);
    if (mMayHaveNonLinearIO) {
        mTotalNumOfStrides->addIncoming(mUpdatedNumOfStrides, entryBlock);
    } else {
        mTotalNumOfStrides->addIncoming(mNumOfLinearStrides, entryBlock);
    }
    if (mExhaustedPipelineInputAtLoopExitPhi) {
        mExhaustedPipelineInputAtLoopExitPhi->addIncoming(mExhaustedInput, entryBlock);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelLoopEntryPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelLoopEntryPhis(BuilderRef b) {
    IntegerType * const sizeTy = b->getSizeTy();
    IntegerType * const boolTy = b->getInt1Ty();
    b->SetInsertPoint(mKernelLoopEntry);
    const auto numOfInputs = getNumOfStreamInputs(mKernelId);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto port = StreamSetPort{PortType::Input, i};
        const auto prefix = makeBufferName(mKernelId, port);
        mAlreadyProcessedPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProcessed");
        mAlreadyProcessedPhi(port)->addIncoming(mInitiallyProcessedItemCount(port), mKernelEntry);
        if (mInitiallyProcessedDeferredItemCount(port)) {
            mAlreadyProcessedDeferredPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProcessedDeferred");
            mAlreadyProcessedDeferredPhi(port)->addIncoming(mInitiallyProcessedDeferredItemCount(port), mKernelEntry);
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelId);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto port = StreamSetPort{PortType::Output, i};
        const auto prefix = makeBufferName(mKernelId, port);
        mAlreadyProducedPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProduced");
        mAlreadyProducedPhi(port)->addIncoming(mInitiallyProducedItemCount(port), mKernelEntry);
        if (mInitiallyProducedDeferredItemCount(port)) {
            mAlreadyProducedDeferredPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProducedDeferred");
            mAlreadyProducedDeferredPhi(port)->addIncoming(mInitiallyProducedDeferredItemCount(port), mKernelEntry);
        }
    }
    const auto prefix = makeKernelName(mKernelId);
    mAlreadyProgressedPhi = b->CreatePHI(boolTy, 2, prefix + "_madeProgress");
    mAlreadyProgressedPhi->addIncoming(mPipelineProgress, mKernelEntry);
    if (mMayHaveNonLinearIO) {
        // Since we may loop and call the kernel again, we want to mark that we've progressed
        // if we execute any kernel even if we could not complete a full segment.
        mExecutedAtLeastOncePhi = b->CreatePHI(boolTy, 2, prefix + "_executedAtLeastOnce");
        mExecutedAtLeastOncePhi->addIncoming(b->getFalse(), mKernelEntry);
        mCurrentNumOfStrides = b->CreatePHI(sizeTy, 2, prefix + "_currentNumOfStrides");
        mCurrentNumOfStrides->addIncoming(b->getSize(0), mKernelEntry);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelCheckOutputSpacePhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelCheckOutputSpacePhis(BuilderRef b) {
    b->SetInsertPoint(mKernelCheckOutputSpace);
    const auto numOfInputs = getNumOfStreamInputs(mKernelId);
    Type * const sizeTy = b->getSizeTy();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto inputPort = StreamSetPort{PortType::Input, i};
        const auto prefix = makeBufferName(mKernelId, inputPort);
        mLinearInputItemsPhi(inputPort) = b->CreatePHI(sizeTy, 2, prefix + "_linearlyAccessible");
        Type * const bufferTy = getInputBuffer(inputPort)->getPointerType();
        mInputVirtualBaseAddressPhi(inputPort) = b->CreatePHI(bufferTy, 2, prefix + "_baseAddress");
    }

    const auto numOfOutputs = getNumOfStreamOutputs(mKernelId);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto outputPort = StreamSetPort{PortType::Output, i};
        const auto prefix = makeBufferName(mKernelId, outputPort);
        mLinearOutputItemsPhi(outputPort) = b->CreatePHI(sizeTy, 2, prefix + "_linearlyWritable");
    }
    mFixedRateFactorPhi = nullptr;
    const auto prefix = makeKernelName(mKernelId);
    if (LLVM_LIKELY(mKernel->hasFixedRateInput())) {
        mFixedRateFactorPhi = b->CreatePHI(sizeTy, 2, prefix + "_fixedRateFactor");
    }
    if (mHasExplicitFinalPartialStride) {
        mReportedNumOfStridesPhi = nullptr;
    } else {
        mReportedNumOfStridesPhi = b->CreatePHI(sizeTy, 2, prefix + "_reportedNumOfLinearStrides");
    }
    mIsFinalInvocationPhi = b->CreatePHI(sizeTy, 2, prefix + "_isFinalPhi");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelTerminatedPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelTerminatedPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelTerminated);
    Type * const sizeTy = b->getSizeTy();
    const auto prefix = makeKernelName(mKernelId);
    mTerminatedSignalPhi = b->CreatePHI(sizeTy, 2, prefix + "_terminatedSignal");
    const auto numOfInputs = getNumOfStreamInputs(mKernelId);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto port = StreamSetPort{ PortType::Input, i };
        const auto prefix = makeBufferName(mKernelId, port);
        mFinalProcessedPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_finalProcessed");
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelId);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto port = StreamSetPort{ PortType::Output, i };
        const auto prefix = makeBufferName(mKernelId, port);
        mFinalProducedPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_finalProduced");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelInsufficientIOExitPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelInsufficientIOExitPhis(BuilderRef b) {
    mExhaustedPipelineInputPhi = nullptr;
    if (LLVM_UNLIKELY(mHasPipelineInput.any())) {
        b->SetInsertPoint(mKernelInsufficientIOExit);
        const auto prefix = makeKernelName(mKernelId);
        IntegerType * const boolTy = b->getInt1Ty();
        mExhaustedPipelineInputPhi = b->CreatePHI(boolTy, 2, prefix + "_exhaustedInput");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelLoopExitPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelLoopExitPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelLoopExit);
    const auto prefix = makeKernelName(mKernelId);
    IntegerType * const sizeTy = b->getSizeTy();
    IntegerType * const boolTy = b->getInt1Ty();
    const auto numOfInputs = getNumOfStreamInputs(mKernelId);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto port = StreamSetPort{ PortType::Input, i };
        const auto prefix = makeBufferName(mKernelId, port);
        mUpdatedProcessedPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessedAtLoopExit");
        if (mAlreadyProcessedDeferredPhi(port)) {
            mUpdatedProcessedDeferredPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessedDeferredAtLoopExit");
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelId);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto port = StreamSetPort{ PortType::Output, i };
        const auto prefix = makeBufferName(mKernelId, port);
        mUpdatedProducedPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_updatedProducedAtLoopExit");
        if (mAlreadyProducedDeferredPhi(port)) {
            mUpdatedProducedDeferredPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessedDeferredAtLoopExit");
        }
    }
    mTerminatedAtLoopExitPhi = b->CreatePHI(sizeTy, 2, prefix + "_terminatedAtLoopExit");    
    mHasProgressedPhi = b->CreatePHI(boolTy, 2, prefix + "_anyProgressAtLoopExit");
    mTotalNumOfStrides = b->CreatePHI(sizeTy, 2, prefix + "_totalNumOfStridesAtLoopExit");
    if (mExhaustedPipelineInputPhi) {
        mExhaustedPipelineInputAtLoopExitPhi = b->CreatePHI(boolTy, 2, prefix + "_exhaustedInputAtLoopExit");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeInsufficientIOExit
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeInsufficientIOExit(BuilderRef b) {    

    // A partition root will always have an insufficient I/O check since they control how many strides the
    // other kernels in the partition will execute. If a kernel has non-linear I/O, however, we need to test
    // whether we've finished executing.

    BasicBlock * const exitBlock = b->GetInsertBlock();

    if (!mIsPartitionRoot || mMayHaveNonLinearIO) {

        mTerminatedAtLoopExitPhi->addIncoming(mTerminatedInitially, exitBlock);
        const auto numOfInputs = getNumOfStreamInputs(mKernelId);
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const auto port = StreamSetPort{ PortType::Input, i };
            mUpdatedProcessedPhi(port)->addIncoming(mAlreadyProcessedPhi(port), exitBlock);
            if (mAlreadyProcessedDeferredPhi(port)) {
                mUpdatedProcessedDeferredPhi(port)->addIncoming(mAlreadyProcessedDeferredPhi(port), exitBlock);
            }
        }
        const auto numOfOutputs = getNumOfStreamOutputs(mKernelId);
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            const auto port = StreamSetPort{ PortType::Output, i };
            mUpdatedProducedPhi(port)->addIncoming(mAlreadyProducedPhi(port), exitBlock);
            if (mAlreadyProducedDeferredPhi(port)) {
                mUpdatedProducedDeferredPhi(port)->addIncoming(mAlreadyProducedDeferredPhi(port), exitBlock);
            }
        }
        if (mMayHaveNonLinearIO) {
            mTotalNumOfStrides->addIncoming(mCurrentNumOfStrides, exitBlock);
        } else {
            mTotalNumOfStrides->addIncoming(b->getSize(0), exitBlock);
        }
        if (mExhaustedPipelineInputAtLoopExitPhi) {
            mExhaustedPipelineInputAtLoopExitPhi->addIncoming(mExhaustedPipelineInputPhi, exitBlock);
        }

    }

    if (mMayHaveNonLinearIO) {
        assert (mAlreadyProgressedPhi);
        mHasProgressedPhi->addIncoming(mAlreadyProgressedPhi, exitBlock);
    } else {
        mHasProgressedPhi->addIncoming(mPipelineProgress, exitBlock);
    }

    if (mIsPartitionRoot) {
        assert (mNextPartitionWithPotentialInput);
        assert (mKernelInitPartitionJump);
        if (mMayHaveNonLinearIO) {
            b->CreateLikelyCondBr(mExecutedAtLeastOncePhi, mKernelLoopExit, mKernelInitPartitionJump);
        } else {
            b->CreateBr(mKernelInitPartitionJump);
        }
    } else { // if this is not a partition root, we know it
        b->CreateBr(mKernelLoopExit);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelExitPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelExitPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelExit);
    const auto prefix = makeKernelName(mKernelId);
    IntegerType * const sizeTy = b->getSizeTy();
    IntegerType * const boolTy = b->getInt1Ty();
    mTerminatedAtExitPhi = b->CreatePHI(sizeTy, 2, prefix + "_terminatedAtKernelExit");
    mTerminatedAtExitPhi->addIncoming(mTerminatedAtLoopExitPhi, mKernelLoopExitPhiCatch);
    mTotalNumOfStridesAtExitPhi = b->CreatePHI(sizeTy, 2, prefix + "_totalNumOfStridesAtExit");
    mTotalNumOfStridesAtExitPhi->addIncoming(mTotalNumOfStrides, mKernelLoopExitPhiCatch);

    createConsumedPhiNodes(b);

    const auto numOfOutputs = getNumOfStreamOutputs(mKernelId);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto port = StreamSetPort{ PortType::Output, i };
        const auto prefix = makeBufferName(mKernelId, port);
        PHINode * const fullyProduced = b->CreatePHI(sizeTy, 2, prefix + "_fullyProducedAtKernelExit");
        mFullyProducedItemCount(port) = fullyProduced;
    }

    if (LLVM_UNLIKELY(mExhaustedPipelineInputPhi)) {
        PHINode * const phi = b->CreatePHI(boolTy, 2, prefix + "_exhaustedPipelineInputAtKernelExit");
        phi->addIncoming(mExhaustedPipelineInputAtLoopExitPhi, mKernelLoopExitPhiCatch);
        mExhaustedPipelineInputAtExit = phi;
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updatePhiCountAfterTermination
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::updatePhisAfterTermination(BuilderRef b) {
    BasicBlock * const exitBlock = b->GetInsertBlock();
    mTerminatedAtLoopExitPhi->addIncoming(mTerminatedSignalPhi, exitBlock);
    mHasProgressedPhi->addIncoming(b->getTrue(), exitBlock);
    if (mExhaustedPipelineInputAtLoopExitPhi) {
        mExhaustedPipelineInputAtLoopExitPhi->addIncoming(mExhaustedInput, exitBlock);
    }
    if (mMayHaveNonLinearIO) {
        mTotalNumOfStrides->addIncoming(mCurrentNumOfStrides, exitBlock);
    } else {
        mTotalNumOfStrides->addIncoming(mNumOfLinearStrides, exitBlock);
    }
    const auto numOfInputs = getNumOfStreamInputs(mKernelId);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto port = StreamSetPort{ PortType::Input, i };
        Value * const totalCount = getLocallyAvailableItemCount(b, port);
        mUpdatedProcessedPhi(port)->addIncoming(totalCount, exitBlock);
        if (mUpdatedProcessedDeferredPhi(port)) {
            mUpdatedProcessedDeferredPhi(port)->addIncoming(totalCount, exitBlock);
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelId);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto port = StreamSetPort{ PortType::Output, i };
        mUpdatedProducedPhi(port)->addIncoming(mFinalProducedPhi(port), exitBlock);
        if (mUpdatedProducedDeferredPhi(port)) {
            mUpdatedProducedDeferredPhi(port)->addIncoming(mFinalProducedPhi(port), exitBlock);
        }
    }   
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief end
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::end(BuilderRef b) {

    // A pipeline will end for one or two reasons:

    // 1) Process has *halted* due to insufficient pipeline I/O.

    // 2) All pipeline sinks have terminated (i.e., any kernel that writes
    // to a pipeline output, is marked as having a side-effect, or produces
    // an input for some call in which no dependent kernels is a pipeline
    // sink).

    // TODO: if we determine that all of the pipeline I/O is consumed in one invocation of the
    // pipeline, we can avoid testing at the end whether its terminated.

    Value * terminated = nullptr;

    if (ExternallySynchronized) {
        b->CreateBr(mPipelineEnd);
    } else {

        #ifdef PRINT_DEBUG_MESSAGES
        debugPrint(b, mTarget->getName() + "+++ pipeline end %" PRIu64 " +++", mSegNo);
        #endif

        terminated = hasPipelineTerminated(b);
        Value * done = b->CreateIsNotNull(terminated);

        if (LLVM_UNLIKELY(mCheckAssertions)) {
            Value * const progressedOrFinished = b->CreateOr(mPipelineProgress, done);
            Value * const live = b->CreateOr(mMadeProgressInLastSegment, progressedOrFinished);
            b->CreateAssert(live, "Dead lock detected: pipeline could not progress after two iterations");
        }
        if (mExhaustedInput) {
            done = b->CreateOr(done, mExhaustedInput);
        }
        BasicBlock * const exitBlock = b->GetInsertBlock();
        mMadeProgressInLastSegment->addIncoming(mPipelineProgress, exitBlock);
        b->CreateUnlikelyCondBr(done, mPipelineEnd, mPipelineLoop);
    }
    b->SetInsertPoint(mPipelineEnd);


    writeExternalConsumedItemCounts(b);
    writeExternalProducedItemCounts(b);
    if (mCurrentThreadTerminationSignalPtr) {
        assert ("externally synchronized kernels cannot have a termination signal" && terminated);
        b->CreateStore(terminated, mCurrentThreadTerminationSignalPtr);
    }
    // free any truncated input buffers
    for (Value * const bufferPtr : mTruncatedInputBuffer) {
        b->CreateFree(b->CreateLoad(bufferPtr));
    }
    #ifdef PRINT_DEBUG_MESSAGES
    if (ExternallySynchronized) {
        debugPrint(b, "------------------------------------------------- END %" PRIx64, getHandle());
    } else {
        debugPrint(b, "================================================= END %" PRIx64, getHandle());
    }
    debugClose(b);
    #endif
    if (LLVM_UNLIKELY(canSetTerminateSignal())) {
        Constant * const unterminated = b->getSize(KernelBuilder::TerminationCode::None);
        Constant * const terminated = b->getSize(KernelBuilder::TerminationCode::Terminated);
        Value * const retVal = b->CreateSelect(mPipelineProgress, unterminated, terminated);
        b->setTerminationSignal(retVal);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeExternalProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeExternalProducedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        const BufferRateData & external = mBufferGraph[e];
        const auto buffer = source(e, mBufferGraph);
        const auto pe = in_edge(buffer, mBufferGraph);
        const BufferRateData & internal = mBufferGraph[pe];
        const auto producer = source(pe, mBufferGraph);
        Value * const ptr = getProducedOutputItemsPtr(external.Port.Number);
        const auto prefix = makeBufferName(producer, internal.Port);
        Value * const produced = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
        b->CreateStore(produced, ptr);
    }
}

}
