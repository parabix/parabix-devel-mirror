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
    makePartitionEntryPoints(b);
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
    branchToInitialPartition(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief executeRegularKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::executeKernel(BuilderRef b) {

    mNumOfAddressableItemCount = 0;
    mNumOfVirtualBaseAddresses = 0;

    mHasZeroExtendedInput = nullptr;

    assert (mKernelIndex >= FirstKernel);
    assert (mKernelIndex <= LastKernel);

    BasicBlock * const partitionExit = getPartitionExitPoint(b);

    mKernelCanTerminateEarly = mKernel->canSetTerminateSignal();
    mKernelIsInternallySynchronized = mKernel->hasAttribute(AttrId::InternallySynchronized);
    mHasExplicitFinalPartialStride = mKernel->requiresExplicitPartialFinalStride();
    mMaximumNumOfStrides = ceiling(MaximumNumOfStrides[mKernelIndex]);
    mMayHaveNonLinearIO = mHasExplicitFinalPartialStride || mayHaveNonLinearIO(mKernelIndex);

    const auto prefix = makeKernelName(mKernelIndex);
    mKernelLoopEntry = b->CreateBasicBlock(prefix + "_loopEntry", partitionExit);
    mKernelCheckOutputSpace = b->CreateBasicBlock(prefix + "_checkOutputSpace", partitionExit);
    mKernelLoopCall = b->CreateBasicBlock(prefix + "_executeKernel", partitionExit);
    mKernelTerminationCheck = b->CreateBasicBlock(prefix + "_normalTerminationCheck", partitionExit);
    mKernelTerminated = b->CreateBasicBlock(prefix + "_terminated", partitionExit);
    mKernelInsufficientIOExit = b->CreateBasicBlock(prefix + "_insufficientIOExit", partitionExit);
    mKernelLoopExit = b->CreateBasicBlock(prefix + "_loopExit", partitionExit);
    // The phi catch simplifies compilation logic by "forward declaring" the loop exit point.
    // Subsequent optimization phases will collapse it into the correct exit block.
    mKernelLoopExitPhiCatch = b->CreateBasicBlock(prefix + "_kernelExitPhiCatch", partitionExit);
    mKernelInitiallyTerminated = b->CreateBasicBlock(prefix + "_initiallyTerminated", partitionExit);
    mKernelInitiallyTerminatedPhiCatch = b->CreateBasicBlock(prefix + "_initiallyTerminatedPhiCatch", partitionExit);
    mKernelExit = b->CreateBasicBlock(prefix + "_kernelExit", partitionExit);

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

    checkInputDataOnPartitionEntry(b);

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
    initializeKernelCheckOutputSpacePhis(b);
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
    // if (LLVM_UNLIKELY(mCheckAssertions)) {
        validateSegmentExecution(b);
    // }
    releaseSynchronizationLock(b, LockType::Segment);

    checkForPartitionExit(b);
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
 * @brief validateSegmentExecution
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::validateSegmentExecution(BuilderRef b) const {

    ConstantInt * lb = b->getSize(floor(MinimumNumOfStrides[mKernelIndex]));
    ConstantInt * ub = b->getSize(ceiling(MaximumNumOfStrides[mKernelIndex]));
    Value * const notTooFew = b->CreateICmpUGE(mTotalNumOfStridesAtExitPhi, lb);
    Value * const terminated = b->CreateIsNotNull(mTerminatedAtExitPhi);
    Value * const notTooFew2 = b->CreateOr(terminated, notTooFew);

    Value * const notTooMany = b->CreateICmpULE(mTotalNumOfStridesAtExitPhi, ub);
    Value * const withinRange = b->CreateAnd(notTooFew2, notTooMany);

    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, "* terminated = %" PRIu8, b->CreateZExtOrTrunc(terminated, b->getInt8Ty()));
    #endif

    //%s:
    b->CreateAssert(withinRange, " processed %" PRIu64 " strides but expected [%" PRIu64 ",%" PRIu64 "]",
                    // mKernelAssertionName,
                    mTotalNumOfStridesAtExitPhi,
                    lb, ub);

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
			const auto port = StreamSetPort{ PortType::Input, i };
            mFinalProcessedPhi(port)->addIncoming(mProcessedItemCount(port), entryBlock);
        }
        for (unsigned i = 0; i < numOfOutputs; ++i) {
			const auto port = StreamSetPort{ PortType::Output, i };
            mFinalProducedPhi(port)->addIncoming(mProducedItemCount(port), entryBlock);
        }
        Value * const notFinal = b->CreateIsNull(mIsFinalInvocationPhi);
        b->CreateLikelyCondBr(notFinal, mKernelLoopExit, mKernelTerminated);
        mTerminatedSignalPhi->addIncoming(mIsFinalInvocationPhi, entryBlock);

    } else if (mIsBounded) {

        for (unsigned i = 0; i < numOfInputs; ++i) {
            const auto port = StreamSetPort{ PortType::Input, i };
            mFinalProcessedPhi(port)->addIncoming(mProcessedItemCount(port), entryBlock);
        }

        for (unsigned i = 0; i < numOfOutputs; ++i) {
            const auto port = StreamSetPort{ PortType::Output, i };
            mFinalProducedPhi(port)->addIncoming(mProducedItemCount(port), entryBlock);
        }

        mTerminatedSignalPhi->addIncoming(mIsFinalInvocationPhi, entryBlock);

        Value * const notFinal = b->CreateIsNull(mIsFinalInvocationPhi);

        if (mMayHaveNonLinearIO) {
            const auto prefix = makeKernelName(mKernelIndex);
            BasicBlock * const ioBoundsCheck = b->CreateBasicBlock(prefix + "_boundsCheck", mKernelTerminated);

            for (unsigned i = 0; i < numOfInputs; ++i) {
                const auto port = StreamSetPort{ PortType::Input, i };
                mAlreadyProcessedPhi(port)->addIncoming(mProcessedItemCount(port), ioBoundsCheck);
                if (mAlreadyProcessedDeferredPhi(port)) {
                    mAlreadyProcessedDeferredPhi(port)->addIncoming(mProcessedDeferredItemCount(port), ioBoundsCheck);
                }
            }

            for (unsigned i = 0; i < numOfOutputs; ++i) {
                const auto port = StreamSetPort{ PortType::Output, i };
                mAlreadyProducedPhi(port)->addIncoming(mProducedItemCount(port), ioBoundsCheck);
                if (mAlreadyProducedDeferredPhi(port)) {
                    mAlreadyProducedDeferredPhi(port)->addIncoming(mProducedDeferredItemCount(port), ioBoundsCheck);
                }
            }

            b->CreateLikelyCondBr(notFinal, ioBoundsCheck, mKernelTerminated);

            b->SetInsertPoint(ioBoundsCheck);

            // bound the number of strides by the maximum expected
            Constant * const maxStrides = b->getSize(mMaximumNumOfStrides);
            Value * const done = b->CreateICmpEQ(mUpdatedNumOfStrides, maxStrides);
            b->CreateCondBr(done, mKernelLoopExit, mKernelLoopEntry);

            mAlreadyProgressedPhi->addIncoming(i1_TRUE, ioBoundsCheck);
            mExecutedAtLeastOncePhi->addIncoming(i1_TRUE, ioBoundsCheck);
            mCurrentNumOfStrides->addIncoming(mUpdatedNumOfStrides, ioBoundsCheck);

        } else {
            b->CreateLikelyCondBr(notFinal, mKernelLoopExit, mKernelTerminated);
        }
    } else { // just exit the loop
        b->CreateBr(mKernelLoopExit);
    }

    BasicBlock * const exitBlock = b->GetInsertBlock();

    for (unsigned i = 0; i < numOfInputs; ++i) {
		const auto port = StreamSetPort{ PortType::Input, i };
		mUpdatedProcessedPhi(port)->addIncoming(mProcessedItemCount(port), exitBlock);
        if (mUpdatedProcessedDeferredPhi(port)) {
            mUpdatedProcessedDeferredPhi(port)->addIncoming(mProcessedDeferredItemCount(port), exitBlock);
        }
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
		const auto port = StreamSetPort{ PortType::Output, i };
        mUpdatedProducedPhi(port)->addIncoming(mProducedItemCount(port), exitBlock);
        if (mUpdatedProducedDeferredPhi(port)) {
            mUpdatedProducedDeferredPhi(port)->addIncoming(mProducedDeferredItemCount(port), exitBlock);
        }
    }

    mTerminatedAtLoopExitPhi->addIncoming(mIsFinalInvocationPhi, exitBlock);
    mHaltingPhi->addIncoming(mHalted, exitBlock);
    mHasProgressedPhi->addIncoming(i1_TRUE, exitBlock);
    if (mMayHaveNonLinearIO) {
        mTotalNumOfStrides->addIncoming(mUpdatedNumOfStrides, exitBlock);
    } else {
        mTotalNumOfStrides->addIncoming(mNumOfLinearStrides, exitBlock);
    }
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
        const auto port = StreamSetPort{PortType::Input, i};
        const auto prefix = makeBufferName(mKernelIndex, port);
        mAlreadyProcessedPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProcessed");
        mAlreadyProcessedPhi(port)->addIncoming(mInitiallyProcessedItemCount(mKernelIndex, port), mKernelEntry);
        if (mInitiallyProcessedDeferredItemCount(port)) {
            mAlreadyProcessedDeferredPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProcessedDeferred");
            mAlreadyProcessedDeferredPhi(port)->addIncoming(mInitiallyProcessedDeferredItemCount(mKernelIndex, port), mKernelEntry);
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto port = StreamSetPort{PortType::Output, i};
        const auto prefix = makeBufferName(mKernelIndex, port);
        mAlreadyProducedPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProduced");
        mAlreadyProducedPhi(port)->addIncoming(mInitiallyProducedItemCount(port), mKernelEntry);
        if (mInitiallyProducedDeferredItemCount(mKernelIndex, port)) {
            mAlreadyProducedDeferredPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProducedDeferred");
            mAlreadyProducedDeferredPhi(mKernelIndex, port)->addIncoming(mInitiallyProducedDeferredItemCount(mKernelIndex, port), mKernelEntry);
        }
    }

    if (mMayHaveNonLinearIO) {
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

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelCheckOutputSpacePhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelCheckOutputSpacePhis(BuilderRef b) {
    b->SetInsertPoint(mKernelCheckOutputSpace);
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    Type * const sizeTy = b->getSizeTy();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto inputPort = StreamSetPort{PortType::Input, i};
        const auto prefix = makeBufferName(mKernelIndex, inputPort);
        mLinearInputItemsPhi(inputPort) = b->CreatePHI(sizeTy, 2, prefix + "_linearlyAccessible");
        Type * const bufferTy = getInputBuffer(inputPort)->getPointerType();
        mInputEpochPhi(inputPort) = b->CreatePHI(bufferTy, 2, prefix + "_baseAddress");
    }

    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto outputPort = StreamSetPort{PortType::Output, i};
        const auto prefix = makeBufferName(mKernelIndex, outputPort);
        mLinearOutputItemsPhi(outputPort) = b->CreatePHI(sizeTy, 2, prefix + "_linearlyWritable");
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
    const auto prefix = makeKernelName(mKernelIndex);
    mTerminatedSignalPhi = b->CreatePHI(sizeTy, 2, prefix + "_terminatedSignal");
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; ++i) {
		const auto port = StreamSetPort{ PortType::Input, i };
        const auto prefix = makeBufferName(mKernelIndex, port);
        mFinalProcessedPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_finalProcessed");
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
		const auto port = StreamSetPort{ PortType::Output, i };
        const auto prefix = makeBufferName(mKernelIndex, port);
        mFinalProducedPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_finalProduced");
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
		const auto port = StreamSetPort{ PortType::Input, i };
        const auto prefix = makeBufferName(mKernelIndex, port);
        mUpdatedProcessedPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessedAtLoopExit");
        if (mAlreadyProcessedDeferredPhi(port)) {
            mUpdatedProcessedDeferredPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessedDeferredAtLoopExit");
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
		const auto port = StreamSetPort{ PortType::Output, i };
        const auto prefix = makeBufferName(mKernelIndex, port);
        mUpdatedProducedPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_updatedProducedAtLoopExit");
        if (mAlreadyProducedDeferredPhi(port)) {
            mUpdatedProducedDeferredPhi(port) = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessedDeferredAtLoopExit");
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
        mHaltingPhi->addIncoming(mInsufficientIOHaltingPhi, exitBlock);
        const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
        for (unsigned i = 0; i < numOfInputs; ++i) {
			const auto port = StreamSetPort{ PortType::Input, i };
            mUpdatedProcessedPhi(port)->addIncoming(mAlreadyProcessedPhi(port), exitBlock);
            if (mAlreadyProcessedDeferredPhi(port)) {
                mUpdatedProcessedDeferredPhi(port)->addIncoming(mAlreadyProcessedDeferredPhi(port), exitBlock);
            }
        }
        const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
        for (unsigned i = 0; i < numOfOutputs; ++i) {
			const auto port = StreamSetPort{ PortType::Output, i };
            mUpdatedProducedPhi(port)->addIncoming(mAlreadyProducedPhi(port), exitBlock);
            if (mAlreadyProducedDeferredPhi(port)) {
                mUpdatedProducedDeferredPhi(port)->addIncoming(mAlreadyProducedDeferredPhi(port), exitBlock);
            }
        }
        if (mMayHaveNonLinearIO) {
            mTotalNumOfStrides->addIncoming(mCurrentNumOfStrides, exitBlock);
            mHasProgressedPhi->addIncoming(mAlreadyProgressedPhi, exitBlock);
        } else {
            mTotalNumOfStrides->addIncoming(b->getSize(0), exitBlock);
            mHasProgressedPhi->addIncoming(mPipelineProgress, exitBlock);
        }
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
    mTotalNumOfStridesAtExitPhi = b->CreatePHI(sizeTy, 2, prefix + "_totalNumOfStridesAtExit");
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
		const auto port = StreamSetPort{ PortType::Output, i };
        const auto prefix = makeBufferName(mKernelIndex, port);
        PHINode * const fullyProduced = b->CreatePHI(sizeTy, 2, prefix + "_fullyProducedAtKernelExit");
        fullyProduced->addIncoming(mInitiallyProducedItemCount(port), mKernelInitiallyTerminatedPhiCatch);
        mFullyProducedItemCount(port) = fullyProduced;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updatePhiCountAfterTermination
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::updatePhisAfterTermination(BuilderRef b) {
    BasicBlock * const exitBlock = b->GetInsertBlock();
    mTerminatedAtLoopExitPhi->addIncoming(mTerminatedSignalPhi, exitBlock);
    mHaltingPhi->addIncoming(mHalted, exitBlock);
    mHasProgressedPhi->addIncoming(b->getTrue(), exitBlock);
    if (mMayHaveNonLinearIO) {
        mTotalNumOfStrides->addIncoming(mCurrentNumOfStrides, exitBlock);
    } else {
        mTotalNumOfStrides->addIncoming(mNumOfLinearStrides, exitBlock);
    }
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; ++i) {
		const auto port = StreamSetPort{ PortType::Input, i };
        Value * const totalCount = getLocallyAvailableItemCount(b, port);
        mUpdatedProcessedPhi(port)->addIncoming(totalCount, exitBlock);
        if (mUpdatedProcessedDeferredPhi(port)) {
            mUpdatedProcessedDeferredPhi(port)->addIncoming(totalCount, exitBlock);
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
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
