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

    // Create the basic blocks for the loop.
    mPipelineLoop = b->CreateBasicBlock("pipelineLoop");
    mPipelineEnd = b->CreateBasicBlock("pipelineEnd");
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
    mKernelIndex = 0;
    BasicBlock * const entryBlock = b->GetInsertBlock();
    b->CreateBr(mPipelineLoop);

    b->SetInsertPoint(mPipelineLoop);
    mMadeProgressInLastSegment = b->CreatePHI(b->getInt1Ty(), 2);
    mMadeProgressInLastSegment->addIncoming(b->getTrue(), entryBlock);
    mPipelineProgress = b->getFalse();
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = mTarget->getName();
    #endif
    mSegNo = mExternalSegNo;
    mHalted = b->getFalse();
    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, prefix + " +++ NUM OF STRIDES %" PRIu64 "+++", mNumOfStrides);
    debugPrint(b, prefix + " +++ IS FINAL %" PRIu8 "+++", mIsFinal);
    #endif
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief executeRegularKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::executeKernel(BuilderRef b) {

    resetMemoizedFields();
    mKernelCanTerminateEarly = mKernel->canSetTerminateSignal();
    mKernelIsInternallySynchronized = mKernel->hasAttribute(AttrId::InternallySynchronized);
    mKernelHasAnExplicitFinalPartialStride = Kernel::requiresExplicitPartialFinalStride(mKernel);
    mMaximumNumOfStrides = ceiling(MaximumNumOfStrides[mKernelIndex]);

    const auto prefix = makeKernelName(mKernelIndex);
    mKernelLoopEntry = b->CreateBasicBlock(prefix + "_loopEntry", mPipelineEnd);
    mKernelLoopCall = b->CreateBasicBlock(prefix + "_executeKernel", mPipelineEnd);
    mKernelTerminationCheck = b->CreateBasicBlock(prefix + "_normalTerminationCheck", mPipelineEnd);
    mKernelTerminated = b->CreateBasicBlock(prefix + "_terminated", mPipelineEnd);
    mKernelInsufficientIOExit = b->CreateBasicBlock(prefix + "_insufficientIOExit", mPipelineEnd);
    mKernelLoopExit = b->CreateBasicBlock(prefix + "_loopExit", mPipelineEnd);
    // The phi catch simplifies compilation logic by "forward declaring" the loop exit point.
    // Subsequent optimization phases will collapse it into the correct exit block.
    mKernelLoopExitPhiCatch = b->CreateBasicBlock(prefix + "_kernelExitPhiCatch", mPipelineEnd);
    mKernelInitiallyTerminated = b->CreateBasicBlock(prefix + "_initiallyTerminated", mPipelineEnd);
    mKernelInitiallyTerminatedPhiCatch = b->CreateBasicBlock(prefix + "_initiallyTerminatedPhiCatch", mPipelineEnd);
    mKernelExit = b->CreateBasicBlock(prefix + "_kernelExit", mPipelineEnd);

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
    readInitialItemCounts(b);
    readConsumedItemCounts(b);
    incrementNumberOfSegmentsCounter(b);
    recordUnconsumedItemCounts(b);
    Value * const terminated = initiallyTerminated(b);
    mKernelEntry = b->GetInsertBlock();
    b->CreateUnlikelyCondBr(terminated, mKernelInitiallyTerminated, mKernelLoopEntry);

    /// -------------------------------------------------------------------------------------
    /// PHI NODE INITIALIZATION
    /// -------------------------------------------------------------------------------------

    // Set up some PHI nodes early to simplify accumulating their incoming values.
    initializeKernelLoopEntryPhis(b);
    initializeKernelCallPhis(b);
    initializeKernelTerminatedPhis(b);
    initializeKernelInsufficientIOExitPhis(b);
    initializeKernelLoopExitPhis(b);
    initializeKernelExitPhis(b);

    /// -------------------------------------------------------------------------------------
    /// KERNEL LOOP ENTRY
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopEntry);
    determineNumOfLinearStrides(b);
    prepareLocalZeroExtendSpace(b);
    calculateItemCounts(b);

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

    if (mKernelCanTerminateEarly) {

        Value * const aborted = b->CreateIsNotNull(mTerminatedExplicitly);
        BasicBlock * const explicitTermination =
            b->CreateBasicBlock(prefix + "_explicitTermination", mKernelTerminationCheck);

        b->CreateUnlikelyCondBr(aborted, explicitTermination, mKernelTerminationCheck);

        b->SetInsertPoint(explicitTermination);
        // If the kernel explicitly terminates, it must set its processed/produced item counts.
        // Otherwise, the pipeline will update any countable rates, even upon termination.
        readCountableItemCountsAfterAbnormalTermination(b);
        signalAbnormalTermination(b);
        b->CreateBr(mKernelTerminated);

    } else { // kernel cannot terminate early

        b->CreateBr(mKernelTerminationCheck);
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL NORMAL TERMINATION CHECK
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelTerminationCheck);
    normalCompletionCheck(b);

    /// -------------------------------------------------------------------------------------
    /// KERNEL TERMINATED
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelTerminated);
    writeTerminationSignal(b, mTerminatedSignalPhi);
    clearUnwrittenOutputData(b);
    updatePhisAfterTermination(b);
    b->CreateBr(mKernelLoopExit);

    /// -------------------------------------------------------------------------------------
    /// KERNEL INSUFFICIENT IO EXIT
    /// -------------------------------------------------------------------------------------

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
    BasicBlock * const loopExit = b->GetInsertBlock(); assert (loopExit);
    b->CreateBr(mKernelExit);

    /// -------------------------------------------------------------------------------------
    /// KERNEL INITIALLY TERMINATED EXIT
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelInitiallyTerminated);
    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, "* " + prefix + ".initiallyTerminated = %" PRIu64, mSegNo);
    #endif
    if (mKernelIsInternallySynchronized) {
        releaseSynchronizationLock(b, LockType::ItemCheck);
        startCycleCounter(b, CycleCounter::BEFORE_SYNCHRONIZATION);
        acquireSynchronizationLock(b, LockType::Segment, CycleCounter::BEFORE_SYNCHRONIZATION);
    }
    loadLastGoodVirtualBaseAddressesOfUnownedBuffers(b);
    BasicBlock * const initiallyTerminatedExit = b->GetInsertBlock(); assert (initiallyTerminatedExit);
    b->CreateBr(mKernelExit);

    /// -------------------------------------------------------------------------------------
    /// KERNEL EXIT (CONTINUED)
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelExit);
    replacePhiCatchBlocksWith(loopExit, initiallyTerminatedExit);
    updateTerminationSignal(mTerminatedAtExitPhi);
    writeFinalConsumedItemCounts(b);
    recordFinalProducedItemCounts(b);
    recordStridesPerSegment(b);
    recordProducedItemCountDeltas(b);
    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, "* " + prefix + ".madeProgress = %" PRIu8, mPipelineProgress);
    #endif
    // chain the progress state so that the next one carries on from this one
    mHalted = mHaltedPhi;
    mPipelineProgress = mNextPipelineProgress;
    releaseSynchronizationLock(b, LockType::Segment);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief replacePhiCatchBlocksWith
 *
 * replace the phi catch with the actual exit blocks
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::replacePhiCatchBlocksWith(BasicBlock * const loopExit, BasicBlock * const initiallyTerminatedExit) {
    // NOTE: not all versions of LLVM seem to have BasicBlock::replacePhiUsesWith or PHINode::replaceIncomingBlockWith.
    // This code could be made to use those instead.
    assert (loopExit);
    assert (initiallyTerminatedExit);
    for (Instruction & inst : *mKernelExit) {
        PHINode & pn = cast<PHINode>(inst);
        for (unsigned i = 0; i != pn.getNumIncomingValues(); ++i) {
            if (pn.getIncomingBlock(i) == mKernelLoopExitPhiCatch) {
                pn.setIncomingBlock(i, loopExit);
            } else if (pn.getIncomingBlock(i) == mKernelInitiallyTerminatedPhiCatch) {
                pn.setIncomingBlock(i, initiallyTerminatedExit);
            }
        }
    }
    mKernelLoopExitPhiCatch->eraseFromParent();
    mKernelLoopExitPhiCatch = loopExit;
    mKernelInitiallyTerminatedPhiCatch->eraseFromParent();
    mKernelInitiallyTerminatedPhiCatch = initiallyTerminatedExit;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief normalCompletionCheck
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::normalCompletionCheck(BuilderRef b) {

    BasicBlock * const entryBlock = b->GetInsertBlock();

    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);

    ConstantInt * const i1_TRUE = b->getTrue();

    if (mKernelIsInternallySynchronized) {

        for (unsigned i = 0; i < numOfInputs; ++i) {
            assert (mProcessedItemCount[i]);
            mFinalProcessedPhi[i]->addIncoming(mProcessedItemCount[i], entryBlock);
        }
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            assert (mProducedItemCount[i]);
            mFinalProducedPhi[i]->addIncoming(mProducedItemCount[i], entryBlock);
        }
        Value * const notFinal = b->CreateIsNull(mIsFinalInvocationPhi);
        b->CreateLikelyCondBr(notFinal, mKernelLoopExit, mKernelTerminated);
        mTerminatedSignalPhi->addIncoming(mIsFinalInvocationPhi, entryBlock);

    } else if (mBoundedKernel) {

        const auto prefix = makeKernelName(mKernelIndex);
        BasicBlock * const ioBoundsCheck = b->CreateBasicBlock(prefix + "_boundsCheck", mKernelTerminated);

        for (unsigned i = 0; i < numOfInputs; ++i) {
            assert (mProcessedItemCount[i]);
            mAlreadyProcessedPhi[i]->addIncoming(mProcessedItemCount[i], ioBoundsCheck);
            if (mAlreadyProcessedDeferredPhi[i]) {
                assert (mProcessedDeferredItemCount[i]);
                mAlreadyProcessedDeferredPhi[i]->addIncoming(mProcessedDeferredItemCount[i], ioBoundsCheck);
            }
            mFinalProcessedPhi[i]->addIncoming(mProcessedItemCount[i], entryBlock);
        }

        for (unsigned i = 0; i < numOfOutputs; ++i) {
            assert (mProducedItemCount[i]);
            mAlreadyProducedPhi[i]->addIncoming(mProducedItemCount[i], ioBoundsCheck);
            if (mAlreadyProducedDeferredPhi[i]) {
                assert (mProducedDeferredItemCount[i]);
                mAlreadyProducedDeferredPhi[i]->addIncoming(mProducedDeferredItemCount[i], ioBoundsCheck);
            }
            mFinalProducedPhi[i]->addIncoming(mProducedItemCount[i], entryBlock);
        }

        Value * const notFinal = b->CreateIsNull(mIsFinalInvocationPhi);
        b->CreateLikelyCondBr(notFinal, ioBoundsCheck, mKernelTerminated);

        mTerminatedSignalPhi->addIncoming(mIsFinalInvocationPhi, entryBlock);

        b->SetInsertPoint(ioBoundsCheck);

        // bound the number of strides by the maximum expected
        Constant * const maxStrides = b->getSize(mMaximumNumOfStrides);
        Value * const done = b->CreateICmpEQ(mUpdatedNumOfStrides, maxStrides);
        b->CreateCondBr(done, mKernelLoopExit, mKernelLoopEntry);

        mAlreadyProgressedPhi->addIncoming(i1_TRUE, ioBoundsCheck);
        mExecutedAtLeastOncePhi->addIncoming(i1_TRUE, ioBoundsCheck);
        mCurrentNumOfStrides->addIncoming(mUpdatedNumOfStrides, ioBoundsCheck);

    } else { // just exit the loop
        b->CreateBr(mKernelLoopExit);
    }

    BasicBlock * const exitBlock = b->GetInsertBlock();

    for (unsigned i = 0; i < numOfInputs; ++i) {
        assert (mProcessedItemCount[i]);
        mUpdatedProcessedPhi[i]->addIncoming(mProcessedItemCount[i], exitBlock);
        if (mUpdatedProcessedDeferredPhi[i]) {
            assert (mProcessedDeferredItemCount[i]);
            mUpdatedProcessedDeferredPhi[i]->addIncoming(mProcessedDeferredItemCount[i], exitBlock);
        }
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        assert (mProducedItemCount[i]);
        mUpdatedProducedPhi[i]->addIncoming(mProducedItemCount[i], exitBlock);
        if (mUpdatedProducedDeferredPhi[i]) {
            assert (mProducedDeferredItemCount[i]);
            mUpdatedProducedDeferredPhi[i]->addIncoming(mProducedDeferredItemCount[i], exitBlock);
        }
    }

    mTerminatedAtLoopExitPhi->addIncoming(mIsFinalInvocationPhi, exitBlock);
    mHasProgressedPhi->addIncoming(i1_TRUE, exitBlock);
    mHaltingPhi->addIncoming(mHalted, exitBlock);
    mTotalNumOfStrides->addIncoming(mUpdatedNumOfStrides, exitBlock);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelLoopEntryPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelLoopEntryPhis(BuilderRef b) {
    IntegerType * const sizeTy = b->getSizeTy();
    IntegerType * const boolTy = b->getInt1Ty();
    b->SetInsertPoint(mKernelLoopEntry);
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto prefix = makeBufferName(mKernelIndex, StreamSetPort{PortType::Input, i});
        mAlreadyProcessedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProcessed");
        mAlreadyProcessedPhi[i]->addIncoming(mInitiallyProcessedItemCount[i], mKernelEntry);
        if (mInitiallyProcessedDeferredItemCount[i]) {
            mAlreadyProcessedDeferredPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProcessedDeferred");
            mAlreadyProcessedDeferredPhi[i]->addIncoming(mInitiallyProcessedDeferredItemCount[i], mKernelEntry);
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto prefix = makeBufferName(mKernelIndex, StreamSetPort{PortType::Output, i});
        mAlreadyProducedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProduced");
        mAlreadyProducedPhi[i]->addIncoming(mInitiallyProducedItemCount[i], mKernelEntry);
        if (mInitiallyProducedDeferredItemCount[i]) {
            mAlreadyProducedDeferredPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProducedDeferred");
            mAlreadyProducedDeferredPhi[i]->addIncoming(mInitiallyProducedDeferredItemCount[i], mKernelEntry);
        }
    }
    // Since we may loop and call the kernel again, we want to mark that we've progressed
    // if we execute any kernel even if we could not complete a full segment.
    const auto prefix = makeKernelName(mKernelIndex);
    mAlreadyProgressedPhi = b->CreatePHI(boolTy, 2, prefix + "_madeProgress");
    mAlreadyProgressedPhi->addIncoming(mPipelineProgress, mKernelEntry);
    mExecutedAtLeastOncePhi = b->CreatePHI(boolTy, 2, prefix + "_executedAtLeastOnce");
    mExecutedAtLeastOncePhi->addIncoming(b->getFalse(), mKernelEntry);
    mCurrentNumOfStrides = b->CreatePHI(sizeTy, 2, prefix + "_currentNumOfStrides");
    mCurrentNumOfStrides->addIncoming(b->getSize(0), mKernelEntry);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelCallPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelCallPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelLoopCall);
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    Type * const sizeTy = b->getSizeTy();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto inputPort = StreamSetPort{PortType::Input, i};
        const auto prefix = makeBufferName(mKernelIndex, inputPort);
        mLinearInputItemsPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_linearlyAccessible");
        Type * const bufferTy = getInputBuffer(inputPort)->getPointerType();
        mInputEpochPhi[i] = b->CreatePHI(bufferTy, 2, prefix + "_baseAddress");
    }

    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto outputPort = StreamSetPort{PortType::Output, i};
        const auto prefix = makeBufferName(mKernelIndex, outputPort);
        mLinearOutputItemsPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_linearlyWritable");
    }
    mFixedRateFactorPhi = nullptr;
    const auto prefix = makeKernelName(mKernelIndex);
    if (LLVM_LIKELY(hasFixedRateLCM())) {
        mFixedRateFactorPhi = b->CreatePHI(sizeTy, 2, prefix + "_fixedRateFactor");
    }
    mIsFinalInvocationPhi = b->CreatePHI(sizeTy, 2, prefix + "_isFinalPhi");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelTerminatedPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelTerminatedPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelTerminated);
    Type * const sizeTy = b->getSizeTy();
    mTerminatedSignalPhi = b->CreatePHI(sizeTy, 2);
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto prefix = makeBufferName(mKernelIndex, StreamSetPort{PortType::Input, i});
        mFinalProcessedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_finalProcessed");
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto prefix = makeBufferName(mKernelIndex, StreamSetPort{PortType::Output, i});
        mFinalProducedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_finalProduced");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelInsufficientIOExitPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelInsufficientIOExitPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelInsufficientIOExit);
    const auto prefix = makeKernelName(mKernelIndex);
    IntegerType * const boolTy = b->getInt1Ty();
    mInsufficientIOHaltingPhi = b->CreatePHI(boolTy, 2, prefix + "_insufficientIOHalting");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelLoopExitPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelLoopExitPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelLoopExit);
    const auto prefix = makeKernelName(mKernelIndex);
    IntegerType * const sizeTy = b->getSizeTy();
    IntegerType * const boolTy = b->getInt1Ty();
    mTerminatedAtLoopExitPhi = b->CreatePHI(sizeTy, 2, prefix + "_terminatedAtLoopExit");
    mHasProgressedPhi = b->CreatePHI(boolTy, 2, prefix + "_anyProgressAtLoopExit");
    mHaltingPhi = b->CreatePHI(boolTy, 2, prefix + "_haltingAtLoopExit");
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto prefix = makeBufferName(mKernelIndex, StreamSetPort{PortType::Input, i});
        mUpdatedProcessedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessedAtLoopExit");
        if (mAlreadyProcessedDeferredPhi[i]) {
            mUpdatedProcessedDeferredPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessedDeferredAtLoopExit");
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto prefix = makeBufferName(mKernelIndex, StreamSetPort{PortType::Output, i});
        mUpdatedProducedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_updatedProducedAtLoopExit");
        if (mAlreadyProducedDeferredPhi[i]) {
            mUpdatedProducedDeferredPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessedDeferredAtLoopExit");
        }
    }
    mTotalNumOfStrides = b->CreatePHI(sizeTy, 2, prefix + "_totalNumOfStridesAtLoopExit");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeInsufficientIOExit
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeInsufficientIOExit(BuilderRef b) {
    if (LLVM_UNLIKELY(mInsufficientIOHaltingPhi->getNumIncomingValues() == 0)) {
        mKernelInsufficientIOExit->eraseFromParent();
    } else {
        b->SetInsertPoint(mKernelInsufficientIOExit);
        if (mKernelIsInternallySynchronized) {
            releaseSynchronizationLock(b, LockType::ItemCheck);
            startCycleCounter(b, CycleCounter::BEFORE_SYNCHRONIZATION);
            acquireSynchronizationLock(b, LockType::Segment, CycleCounter::BEFORE_SYNCHRONIZATION);
        }
        BasicBlock * const exitBlock = b->GetInsertBlock();
        mTerminatedAtLoopExitPhi->addIncoming(mTerminatedInitially, exitBlock);
        mHasProgressedPhi->addIncoming(mAlreadyProgressedPhi, exitBlock);
        mHaltingPhi->addIncoming(mInsufficientIOHaltingPhi, exitBlock);
        const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
        for (unsigned i = 0; i < numOfInputs; ++i) {
            mUpdatedProcessedPhi[i]->addIncoming(mAlreadyProcessedPhi[i], exitBlock);
            if (mAlreadyProcessedDeferredPhi[i]) {
                mUpdatedProcessedDeferredPhi[i]->addIncoming(mAlreadyProcessedDeferredPhi[i], exitBlock);
            }
        }
        const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            mUpdatedProducedPhi[i]->addIncoming(mAlreadyProducedPhi[i], exitBlock);
            if (mAlreadyProducedDeferredPhi[i]) {
                mUpdatedProducedDeferredPhi[i]->addIncoming(mAlreadyProducedDeferredPhi[i], exitBlock);
            }
        }
        mTotalNumOfStrides->addIncoming(mCurrentNumOfStrides, exitBlock);
        b->CreateBr(mKernelLoopExit);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelExitPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelExitPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelExit);
    const auto prefix = makeKernelName(mKernelIndex);
    IntegerType * const sizeTy = b->getSizeTy();
    mTerminatedAtExitPhi = b->CreatePHI(sizeTy, 2, prefix + "_terminatedAtKernelExit");
    mTerminatedAtExitPhi->addIncoming(mTerminatedInitially, mKernelInitiallyTerminatedPhiCatch);
    mTerminatedAtExitPhi->addIncoming(mTerminatedAtLoopExitPhi, mKernelLoopExitPhiCatch);

    mTotalNumOfStridesAtExitPhi = b->CreatePHI(sizeTy, 2);
    mTotalNumOfStridesAtExitPhi->addIncoming(b->getSize(0), mKernelInitiallyTerminatedPhiCatch);
    mTotalNumOfStridesAtExitPhi->addIncoming(mTotalNumOfStrides, mKernelLoopExitPhiCatch);

    IntegerType * const boolTy = b->getInt1Ty();
    mHaltedPhi = b->CreatePHI(boolTy, 2, prefix + "_haltedAtKernelExit");
    mHaltedPhi->addIncoming(mHalted, mKernelInitiallyTerminatedPhiCatch);
    mHaltedPhi->addIncoming(mHaltingPhi, mKernelLoopExitPhiCatch);

    PHINode * const pipelineProgress = b->CreatePHI(boolTy, 2, prefix + "_pipelineProgressAtKernelExit");
    pipelineProgress->addIncoming(mPipelineProgress, mKernelInitiallyTerminatedPhiCatch);
    pipelineProgress->addIncoming(mHasProgressedPhi, mKernelLoopExitPhiCatch);
    mNextPipelineProgress = pipelineProgress;

    createConsumedPhiNodes(b);

    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto prefix = makeBufferName(mKernelIndex, StreamSetPort{PortType::Output, i});
        PHINode * const fullyProduced = b->CreatePHI(sizeTy, 2, prefix + "_fullyProducedAtKernelExit");
        fullyProduced->addIncoming(mInitiallyProducedItemCount[i], mKernelInitiallyTerminatedPhiCatch);
        mFullyProducedItemCount[i] = fullyProduced;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updatePhiCountAfterTermination
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::updatePhisAfterTermination(BuilderRef b) {
    BasicBlock * const exitBlock = b->GetInsertBlock();
    mTerminatedAtLoopExitPhi->addIncoming(mTerminatedSignalPhi, exitBlock);
    mHasProgressedPhi->addIncoming(b->getTrue(), exitBlock);
    mHaltingPhi->addIncoming(mHalted, exitBlock);
    mTotalNumOfStrides->addIncoming(mCurrentNumOfStrides, exitBlock);
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        Value * const totalCount = getLocallyAvailableItemCount(b, StreamSetPort{PortType::Input, i});
        mUpdatedProcessedPhi[i]->addIncoming(totalCount, exitBlock);
        if (mUpdatedProcessedDeferredPhi[i]) {
            mUpdatedProcessedDeferredPhi[i]->addIncoming(totalCount, exitBlock);
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        mUpdatedProducedPhi[i]->addIncoming(mFinalProducedPhi[i], exitBlock);
        if (mUpdatedProducedDeferredPhi[i]) {
            mUpdatedProducedDeferredPhi[i]->addIncoming(mFinalProducedPhi[i], exitBlock);
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

    Value * const terminated = hasPipelineTerminated(b);
    // TODO: remove halted check? see if editd still requires it.
    Value * done = b->CreateOr(mHalted, b->CreateIsNotNull(terminated));
    Value * const progressedOrFinished = b->CreateOr(mPipelineProgress, done);
    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, mTarget->getName() + "+++ pipeline end %" PRIu64 " +++", mSegNo);
    #endif

    if (LLVM_UNLIKELY(mCheckAssertions)) {
        b->CreateAssert(b->CreateOr(mMadeProgressInLastSegment, progressedOrFinished),
            "Dead lock detected: pipeline could not progress after two iterations");
    }

    BasicBlock * const exitBlock = b->GetInsertBlock();
    mMadeProgressInLastSegment->addIncoming(progressedOrFinished, exitBlock);
    if (ExternallySynchronized) {
        done = b->getTrue();
    }
    b->CreateUnlikelyCondBr(done, mPipelineEnd, mPipelineLoop);

    b->SetInsertPoint(mPipelineEnd);
    writeExternalConsumedItemCounts(b);
    writeExternalProducedItemCounts(b);
    if (mCurrentThreadTerminationSignalPtr) {
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
        Value * const ptr = getProducedOutputItemsPtr(external.outputPort());
        const auto prefix = makeBufferName(producer, internal.Port);
        Value * const produced = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
        b->CreateStore(produced, ptr);
    }
}

}
