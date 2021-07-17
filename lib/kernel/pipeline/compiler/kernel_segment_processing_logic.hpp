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

    mCurrentKernelName = mKernelName[PipelineInput];

    makePartitionEntryPoints(b);

    if (CheckAssertions) {
        mRethrowException = b->WriteDefaultRethrowBlock();
    }

    #ifdef PRINT_DEBUG_MESSAGES
    debugInit(b);
    if (ExternallySynchronized) {
        debugPrint(b, "------------------------------------------------- START %" PRIx64, getHandle());
    } else {
        debugPrint(b, "================================================= START %" PRIx64, getHandle());
    }
    const auto prefix = mTarget->getName();
    if (mNumOfStrides) {
        debugPrint(b, prefix + " +++ NUM OF STRIDES %" PRIu64 "+++", mNumOfStrides);
    }
    debugPrint(b, prefix + " +++ IS FINAL %" PRIu8 "+++", mIsFinal);
    #endif

    #ifdef ENABLE_PAPI
    createEventSetAndStartPAPI(b);
    #endif

    mExpectedNumOfStridesMultiplier = b->getScalarField(EXPECTED_NUM_OF_STRIDES_MULTIPLIER);
    if (LLVM_LIKELY(RequiredThreadLocalStreamSetMemory > 0)) {
        mThreadLocalStreamSetBaseAddress = b->getScalarField(BASE_THREAD_LOCAL_STREAMSET_MEMORY);
    }

    loadExternalStreamSetHandles(b);
    loadInternalStreamSetHandles(b, true);
    loadInternalStreamSetHandles(b, false);
    readExternalConsumerItemCounts(b);
    initializePipelineInputTerminationSignal(b);
    identifyAllInternallySynchronizedKernels();

    mKernel = nullptr;
    mKernelId = 0;
    readFirstSegmentNumber(b);
    BasicBlock * const entryBlock = b->GetInsertBlock();
    b->CreateBr(mPipelineLoop);

    b->SetInsertPoint(mPipelineLoop);
    mMadeProgressInLastSegment = b->CreatePHI(b->getInt1Ty(), 2, "madeProgressInLastSegment");
    mMadeProgressInLastSegment->addIncoming(b->getTrue(), entryBlock);
    initializeLocallyAvailableItemCounts(b, entryBlock);
    Constant * const i1_FALSE = b->getFalse();
    mPipelineProgress = i1_FALSE;
    mExhaustedInput = i1_FALSE;
    obtainCurrentSegmentNumber(b, entryBlock);

    branchToInitialPartition(b);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief executeKernel
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::executeKernel(BuilderRef b) {

    clearInternalStateForCurrentKernel();

    checkForPartitionEntry(b);

    mFixedRateLCM = getLCMOfFixedRateInputs(mKernel);
    mKernelIsInternallySynchronized = mKernel->hasAttribute(AttrId::InternallySynchronized);
    mKernelCanTerminateEarly = mKernel->canSetTerminateSignal();
    mNextPartitionEntryPoint = getPartitionExitPoint(b);
    assert (mNextPartitionEntryPoint);

    mExhaustedPipelineInputAtExit = mExhaustedInput;



    identifyPipelineInputs(mKernelId);

    if (RequiresSynchronization[mKernelId]) {
        mIsBounded = isBounded();
        mHasExplicitFinalPartialStride = requiresExplicitFinalStride();
        mCheckInputChannels = false;
        for (const auto input : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
            const BufferPort & port = mBufferGraph[input];
            if (port.CanModifySegmentLength) {
                mCheckInputChannels = true;
                break;
            }
        }

        mMayLoopToEntry = false;

        if (in_degree(mKernelId, mBufferGraph) > 0) {
            if (mHasExplicitFinalPartialStride || (mCheckInputChannels && hasAtLeastOneNonGreedyInput())) {
                mMayLoopToEntry = true;
            } else {
                for (const auto output : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
                    const BufferPort & port = mBufferGraph[output];
                    if (port.CanModifySegmentLength) {
                        mMayLoopToEntry = true;
                        break;
                    }
                }
            }
        }
    } else {
        mHasExplicitFinalPartialStride = false;
        mIsBounded = false;
        mCheckInputChannels = false;
        mMayLoopToEntry = false;
    }

    const auto nextPartitionId = mCurrentPartitionId + 1U;
    const auto jumpId = PartitionJumpTargetId[mCurrentPartitionId];
    const auto canJumpToAnotherPartition = mIsPartitionRoot && (mIsBounded || nextPartitionId == jumpId);
    const auto handleNoUpdateExit = mIsPartitionRoot; // || !canJumpToAnotherPartition;

    const auto prefix = makeKernelName(mKernelId);

    // TODO: if a kernel has circular buffers and the produced/consumption rate is not synchronous
    // and the GCD of the stride step length of the producer/consumer is 1 but the stride step length
    // of the consumer is > 1, we may get a scenario in which the partition root needs to check the
    // raw produced item counts rather than the accessible ones to determine the segment length.
    // We could bypass this by having a larger overflow region but doing so would cause us to memcpy
    // more data than necessary.

    /// -------------------------------------------------------------------------------------
    /// BASIC BLOCK CONSTRUCTION
    /// -------------------------------------------------------------------------------------

    mKernelLoopEntry = b->CreateBasicBlock(prefix + "_loopEntry", mNextPartitionEntryPoint);
    mKernelCheckOutputSpace = nullptr;
    if (RequiresSynchronization[mKernelId]) {
        mKernelCheckOutputSpace = b->CreateBasicBlock(prefix + "_checkOutputSpace", mNextPartitionEntryPoint);
    }
    mKernelLoopCall = b->CreateBasicBlock(prefix + "_executeKernel", mNextPartitionEntryPoint);
    mKernelCompletionCheck = b->CreateBasicBlock(prefix + "_normalCompletionCheck", mNextPartitionEntryPoint);
    if (mCheckInputChannels) {
        mKernelInsufficientInput = b->CreateBasicBlock(prefix + "_insufficientInput", mNextPartitionEntryPoint);
    }
    if (handleNoUpdateExit) {
        mKernelInitiallyTerminated = b->CreateBasicBlock(prefix + "_initiallyTerminated", mNextPartitionEntryPoint);
    }
    if (canJumpToAnotherPartition) {
        SmallVector<char, 256> tmp;
        raw_svector_ostream nm(tmp);
        nm << prefix << "_jumpFromPartition_" << mCurrentPartitionId
           << "_to_" << PartitionJumpTargetId[mCurrentPartitionId];
        mKernelJumpToNextUsefulPartition = b->CreateBasicBlock(nm.str(), mNextPartitionEntryPoint);
    } else {
        mKernelJumpToNextUsefulPartition = mKernelInitiallyTerminated;
    }

    mKernelTerminated = b->CreateBasicBlock(prefix + "_terminated", mNextPartitionEntryPoint);
    mKernelLoopExit = b->CreateBasicBlock(prefix + "_loopExit", mNextPartitionEntryPoint);
    // The phi catch simplifies compilation logic by "forward declaring" the loop exit point.
    // Subsequent optimization phases will collapse it into the correct exit block.
    mKernelLoopExitPhiCatch = b->CreateBasicBlock(prefix + "_kernelExitPhiCatch", mNextPartitionEntryPoint);
    mKernelExit = b->CreateBasicBlock(prefix + "_kernelExit", mNextPartitionEntryPoint);

    /// -------------------------------------------------------------------------------------
    /// KERNEL / PARTITION ENTRY BLOCK
    /// -------------------------------------------------------------------------------------

    verifyCurrentSynchronizationLock(b);
    checkIfKernelIsAlreadyTerminated(b);
    readProcessedItemCounts(b);
    readProducedItemCounts(b);
    readConsumedItemCounts(b);

    prepareLinearThreadLocalOutputBuffers(b);

    incrementNumberOfSegmentsCounter(b);
    recordUnconsumedItemCounts(b);

    mFinalPartialStrideFixedRateRemainderPhi = nullptr;

    detemineMaximumNumberOfStrides(b);

    if (mIsPartitionRoot) {
        b->CreateUnlikelyCondBr(mInitiallyTerminated, mKernelInitiallyTerminated, mKernelLoopEntry);
    } else {
        b->CreateBr(mKernelLoopEntry);
    }
    mKernelLoopStart = b->GetInsertBlock();

    /// -------------------------------------------------------------------------------------
    /// PHI NODE INITIALIZATION
    /// -------------------------------------------------------------------------------------

    // Set up some PHI nodes early to simplify accumulating their incoming values.
    initializeKernelLoopEntryPhis(b);
    if (RequiresSynchronization[mKernelId]) {
        initializeKernelCheckOutputSpacePhis(b);
    }
    if (canJumpToAnotherPartition) {
        initializeJumpToNextUsefulPartitionPhis(b);
    }
    if (mCheckInputChannels) {
        initializeKernelInsufficientIOExitPhis(b);
    }
    initializeKernelTerminatedPhis(b);
    initializeKernelLoopExitPhis(b);
    initializeKernelExitPhis(b);

    /// -------------------------------------------------------------------------------------
    /// KERNEL LOOP ENTRY
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopEntry);
    if (RequiresSynchronization[mKernelId]) {
        determineNumOfLinearStrides(b);
        mIsFinalInvocation = mIsFinalInvocationPhi;
    } else {
        mUpdatedNumOfStrides = mMaximumNumOfStrides;
        mIsFinalInvocation = checkIfInputIsExhausted(b, InputExhaustionReturnType::Disjunction);
    }

    // When tracing blocking I/O, test all I/O streams but do not execute the
    // kernel if any stream is insufficient.
    if (mCheckInputChannels && TraceIO) {
        b->CreateUnlikelyCondBr(mBranchToLoopExit, mKernelInsufficientInput, mKernelLoopCall);
        BasicBlock * const exitBlock = b->GetInsertBlock();
        mExhaustedPipelineInputPhi->addIncoming(mExhaustedInput, exitBlock);
    } else {
        b->CreateBr(mKernelLoopCall);
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL CALL
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopCall);
    writeLookBehindLogic(b);
    writeKernelCall(b);
    writeCopyBackLogic(b);
    writeDelayReflectionLogic(b);

    /// -------------------------------------------------------------------------------------
    /// KERNEL EXPLICIT TERMINATION CHECK
    /// -------------------------------------------------------------------------------------

    if (mKernelCanTerminateEarly) {

        Value * const aborted = b->CreateIsNotNull(mTerminatedExplicitly);
        BasicBlock * const explicitTermination =
            b->CreateBasicBlock(prefix + "_explicitTermination", mKernelCompletionCheck);
        b->CreateUnlikelyCondBr(aborted, explicitTermination, mKernelCompletionCheck);

        b->SetInsertPoint(explicitTermination);
        // If the kernel explicitly terminates, it must set its processed/produced item counts.
        // Otherwise, the pipeline will update any countable rates, even upon termination.
        readCountableItemCountsAfterAbnormalTermination(b);
        #warning review this
        // TODO: We could have a *fixed-rate* source kernel be a partition root but will need to
        // calculate how many items are the stride "remainder" here.
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

    b->SetInsertPoint(mKernelTerminated);
    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, "** " + prefix + ".terminated at segment %" PRIu64, mSegNo);
    #endif
    writeTerminationSignal(b, mTerminatedSignalPhi);
    informInputKernelsOfTermination(b);
    clearUnwrittenOutputData(b);
    updatePhisAfterTermination(b);
    b->CreateBr(mKernelLoopExit);

    /// -------------------------------------------------------------------------------------
    /// KERNEL INSUFFICIENT IO EXIT
    /// -------------------------------------------------------------------------------------

    if (mCheckInputChannels) {
        writeInsufficientIOExit(b);
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL LOOP EXIT
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopExit);
    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, "** " + prefix + ".loopExit = %" PRIu64, mSegNo);
    #endif
    writeUpdatedItemCounts(b);
    assert (mTerminatedAtLoopExitPhi);
    Constant * const unterminated = getTerminationSignal(b, TerminationSignal::None);
    Value * const terminated = b->CreateICmpNE(mTerminatedAtLoopExitPhi, unterminated);
    computeFullyProcessedItemCounts(b, terminated);
    computeMinimumConsumedItemCounts(b);
    writeLookAheadLogic(b);
    computeFullyProducedItemCounts(b, terminated);
    replacePhiCatchWithCurrentBlock(b, mKernelLoopExitPhiCatch, mKernelExit);
    b->CreateBr(mKernelExit);

    /// -------------------------------------------------------------------------------------
    /// KERNEL INITIALLY TERMINATED EXIT
    /// -------------------------------------------------------------------------------------

    if (handleNoUpdateExit) {
        writeInitiallyTerminatedPartitionExit(b);
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL PREPARE FOR PARTITION JUMP
    /// -------------------------------------------------------------------------------------

    if (canJumpToAnotherPartition) {
        writeJumpToNextPartition(b);
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL EXIT
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelExit);
    writeConsumedItemCounts(b);
    recordFinalProducedItemCounts(b);
    recordStridesPerSegment(b);
    recordProducedItemCountDeltas(b);
    setCurrentTerminationSignal(b, mTerminatedAtExitPhi);

    // chain the progress state so that the next one carries on from this one
    mExhaustedInput = mExhaustedPipelineInputAtExit;
    mPipelineProgress = mAnyProgressedAtExitPhi;
    if (mIsPartitionRoot) {
        mNumOfPartitionStrides = mTotalNumOfStridesAtExitPhi;
        mFinalPartitionSegment = mFinalPartitionSegmentAtExitPhi;
    }

    updateCycleCounter(b, mKernelId, mKernelStartTime, CycleCounter::TOTAL_TIME);
    #ifdef ENABLE_PAPI
    accumPAPIMeasurementWithoutReset(b, PAPIReadInitialMeasurementArray, mKernelId, PAPIKernelCounter::PAPI_KERNEL_TOTAL);
    #endif

    if (LLVM_UNLIKELY(CheckAssertions)) {        
        verifyPostInvocationTerminationSignal(b);
    }

    checkForPartitionExit(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief normalCompletionCheck
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::normalCompletionCheck(BuilderRef b) {

    ConstantInt * const i1_TRUE = b->getTrue();

    if (LLVM_LIKELY(mMayLoopToEntry)) {

        Value * const loopAgain = hasMoreInput(b);

        BasicBlock * const exitBlockAfterLoopAgainTest = b->GetInsertBlock();

        for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
            const auto port = mBufferGraph[e].Port;
            mAlreadyProcessedPhi[port]->addIncoming(mProcessedItemCount[port], exitBlockAfterLoopAgainTest);
            if (mAlreadyProcessedDeferredPhi[port]) {
                mAlreadyProcessedDeferredPhi[port]->addIncoming(mProcessedDeferredItemCount[port], exitBlockAfterLoopAgainTest);
            }
        }

        for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
            const auto port = mBufferGraph[e].Port;
            mAlreadyProducedPhi[port]->addIncoming(mProducedItemCount[port], exitBlockAfterLoopAgainTest);
            if (mAlreadyProducedDeferredPhi[port]) {
                mAlreadyProducedDeferredPhi[port]->addIncoming(mProducedDeferredItemCount[port], exitBlockAfterLoopAgainTest);
            }
        }

        mAlreadyProgressedPhi->addIncoming(i1_TRUE, exitBlockAfterLoopAgainTest);
        if (mMayLoopToEntry) {
            mExecutedAtLeastOnceAtLoopEntryPhi->addIncoming(i1_TRUE, exitBlockAfterLoopAgainTest);
            mCurrentNumOfStridesAtLoopEntryPhi->addIncoming(mUpdatedNumOfStrides, exitBlockAfterLoopAgainTest);
        }

        const auto prefix = makeKernelName(mKernelId);
        BasicBlock * const isFinalCheck = b->CreateBasicBlock(prefix + "_isFinalCheck", mKernelTerminated);
        b->CreateUnlikelyCondBr(loopAgain, mKernelLoopEntry, isFinalCheck);

        b->SetInsertPoint(isFinalCheck);
    }

    Value * terminationSignal = nullptr;
    if (LLVM_UNLIKELY(mKernelIsInternallySynchronized)) {
        Constant * const completed = getTerminationSignal(b, TerminationSignal::Completed);
        Constant * const unterminated = getTerminationSignal(b, TerminationSignal::None);
        terminationSignal = b->CreateSelect(mIsFinalInvocation, completed, unterminated);
    } else {
        terminationSignal = mIsFinalInvocationPhi;
    }

    assert (terminationSignal);

    BasicBlock * const exitBlock = b->GetInsertBlock();

    // update KernelTerminated phi nodes
    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto port = mBufferGraph[e].Port;
        mProducedAtTerminationPhi[port]->addIncoming(mProducedItemCount[port], exitBlock);
    }
    mTerminatedSignalPhi->addIncoming(terminationSignal, exitBlock);

    if (mTotalNumOfStridesAtLoopExitPhi) {
        Value * updatedNumOfStrides = mUpdatedNumOfStrides; assert (mUpdatedNumOfStrides);
        if (mIsPartitionRoot) {
            updatedNumOfStrides = b->CreateMulRational(updatedNumOfStrides, mPartitionStrideRateScalingFactor);
        }
        mTotalNumOfStridesAtLoopExitPhi->addIncoming(updatedNumOfStrides, exitBlock);
    }

    Value * const isFinal = b->CreateIsNotNull(terminationSignal);
    if (mIsPartitionRoot) {
        mFinalPartitionSegmentAtLoopExitPhi->addIncoming(b->getFalse(), exitBlock);
    }
    b->CreateUnlikelyCondBr(isFinal, mKernelTerminated, mKernelLoopExit);

    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const auto port = mBufferGraph[e].Port;
        mUpdatedProcessedPhi[port]->addIncoming(mProcessedItemCount[port], exitBlock);
        if (mUpdatedProcessedDeferredPhi[port]) {
            mUpdatedProcessedDeferredPhi[port]->addIncoming(mProcessedDeferredItemCount[port], exitBlock);
        }
    }
    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto port = mBufferGraph[e].Port;
        mUpdatedProducedPhi[port]->addIncoming(mProducedItemCount[port], exitBlock);
        if (mUpdatedProducedDeferredPhi[port]) {
            mUpdatedProducedDeferredPhi[port]->addIncoming(mProducedDeferredItemCount[port], exitBlock);
        }
    }
    mTerminatedAtLoopExitPhi->addIncoming(terminationSignal, exitBlock);
    mAnyProgressedAtLoopExitPhi->addIncoming(i1_TRUE, exitBlock);
    mExhaustedPipelineInputAtLoopExitPhi->addIncoming(mExhaustedInput, exitBlock);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelLoopEntryPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelLoopEntryPhis(BuilderRef b) {
    IntegerType * const sizeTy = b->getSizeTy();
    IntegerType * const boolTy = b->getInt1Ty();

    assert ("kernel loop start must be created before initializing loop entry phi nodes" && mKernelLoopStart);

    b->SetInsertPoint(mKernelLoopEntry);

    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const BufferPort & br = mBufferGraph[e];
        const auto port = br.Port;
        const auto prefix = makeBufferName(mKernelId, port);
        mAlreadyProcessedPhi[port] = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProcessed");
        assert (mInitiallyProcessedItemCount[port]);
        mAlreadyProcessedPhi[port]->addIncoming(mInitiallyProcessedItemCount[port], mKernelLoopStart);
        Value * const value = mInitiallyProcessedDeferredItemCount[port];
        if (value) {
            PHINode * const phi = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProcessedDeferred");
            assert (phi);
            phi->addIncoming(value, mKernelLoopStart);
            mAlreadyProcessedDeferredPhi[port] = phi;
        }
    }

    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const BufferPort & br = mBufferGraph[e];
        const auto port = br.Port;
        const auto prefix = makeBufferName(mKernelId, port);
        const auto streamSet = target(e, mBufferGraph);
        mAlreadyProducedPhi[port] = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProduced");
        assert (mInitiallyProducedItemCount[streamSet]);
        mAlreadyProducedPhi[port]->addIncoming(mInitiallyProducedItemCount[streamSet], mKernelLoopStart);
        if (mInitiallyProducedDeferredItemCount[streamSet]) {
            mAlreadyProducedDeferredPhi[port] = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProducedDeferred");
            mAlreadyProducedDeferredPhi[port]->addIncoming(mInitiallyProducedDeferredItemCount[streamSet], mKernelLoopStart);
        }
    }
    const auto prefix = makeKernelName(mKernelId);
    mAlreadyProgressedPhi = b->CreatePHI(boolTy, 2, prefix + "_madeProgress");
    assert (mPipelineProgress);
    mAlreadyProgressedPhi->addIncoming(mPipelineProgress, mKernelLoopStart);

    if (mMayLoopToEntry) {
        // Since we may loop and call the kernel again, we want to mark that we've progressed
        // if we execute any kernel even if we could not complete a full segment.
        mExecutedAtLeastOnceAtLoopEntryPhi = b->CreatePHI(boolTy, 2, prefix + "_executedAtLeastOnce");
        mExecutedAtLeastOnceAtLoopEntryPhi->addIncoming(b->getFalse(), mKernelLoopStart);
        mCurrentNumOfStridesAtLoopEntryPhi = b->CreatePHI(sizeTy, 2, prefix + "_currentNumOfStrides");
        mCurrentNumOfStridesAtLoopEntryPhi->addIncoming(b->getSize(0), mKernelLoopStart);
    } else {
        mExecutedAtLeastOnceAtLoopEntryPhi = nullptr;
        mCurrentNumOfStridesAtLoopEntryPhi = nullptr;
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelCheckOutputSpacePhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelCheckOutputSpacePhis(BuilderRef b) {
    b->SetInsertPoint(mKernelCheckOutputSpace);
    IntegerType * const sizeTy = b->getSizeTy();
    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const auto inputPort = mBufferGraph[e].Port;
        const auto prefix = makeBufferName(mKernelId, inputPort);
        mLinearInputItemsPhi[inputPort] = b->CreatePHI(sizeTy, 2, prefix + "_linearlyAccessible");
        Type * const bufferTy = getInputBuffer(inputPort)->getPointerType();
        mInputVirtualBaseAddressPhi[inputPort] = b->CreatePHI(bufferTy, 2, prefix + "_baseAddress");
    }
    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto outputPort = mBufferGraph[e].Port;
        const auto prefix = makeBufferName(mKernelId, outputPort);
        mLinearOutputItemsPhi[outputPort] = b->CreatePHI(sizeTy, 2, prefix + "_linearlyWritable");
    }
    const auto prefix = makeKernelName(mKernelId);
    mNumOfLinearStridesPhi = b->CreatePHI(sizeTy, 2, prefix + "_numOfLinearStridesPhi");
    if (LLVM_LIKELY(mKernel->hasFixedRateInput())) {
        mFixedRateFactorPhi = b->CreatePHI(sizeTy, 2, prefix + "_fixedRateFactorPhi");
    }
    mIsFinalInvocationPhi = b->CreatePHI(sizeTy, 2, prefix + "_isFinalPhi");
    if (mIsPartitionRoot) {
        mFinalPartialStrideFixedRateRemainderPhi = b->CreatePHI(sizeTy, 2, prefix + "_partialPartitionStridesPhi");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelTerminatedPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelTerminatedPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelTerminated);
    Type * const sizeTy = b->getSizeTy();
    const auto prefix = makeKernelName(mKernelId);
    mTerminatedSignalPhi = b->CreatePHI(sizeTy, 2, prefix + "_terminatedSignal");

    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto outputPort = mBufferGraph[e].Port;
        const auto prefix = makeBufferName(mKernelId, outputPort);
        mProducedAtTerminationPhi[outputPort] = b->CreatePHI(sizeTy, 2, prefix + "_finalProduced");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelTerminatedPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeJumpToNextUsefulPartitionPhis(BuilderRef b) {
    assert (mKernelJumpToNextUsefulPartition);
    b->SetInsertPoint(mKernelJumpToNextUsefulPartition);
    const auto prefix = makeKernelName(mKernelId);
    Type * const boolTy = b->getInt1Ty();
    mExhaustedInputAtJumpPhi = b->CreatePHI(boolTy, 2, prefix + "_exhaustedInputAtJumpPhi");
}



/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelInsufficientIOExitPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelInsufficientIOExitPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelInsufficientInput);
    const auto prefix = makeKernelName(mKernelId);
    IntegerType * const boolTy = b->getInt1Ty();
    mExhaustedPipelineInputPhi = b->CreatePHI(boolTy, 2, prefix + "_exhaustedInput");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelLoopExitPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelLoopExitPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelLoopExit);
    const auto prefix = makeKernelName(mKernelId);
    IntegerType * const sizeTy = b->getSizeTy();
    IntegerType * const boolTy = b->getInt1Ty();
    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const auto port = mBufferGraph[e].Port;
        const auto prefix = makeBufferName(mKernelId, port);
        mUpdatedProcessedPhi[port] = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessedAtLoopExit");
        if (mAlreadyProcessedDeferredPhi[port]) {
            mUpdatedProcessedDeferredPhi[port] = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessedDeferredAtLoopExit");
        }
    }
    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto port = mBufferGraph[e].Port;
        const auto prefix = makeBufferName(mKernelId, port);
        mUpdatedProducedPhi[port] = b->CreatePHI(sizeTy, 2, prefix + "_updatedProducedAtLoopExit");
        if (mAlreadyProducedDeferredPhi[port]) {
            mUpdatedProducedDeferredPhi[port] = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessedDeferredAtLoopExit");
        }
    }
    mTerminatedAtLoopExitPhi = b->CreatePHI(sizeTy, 2, prefix + "_terminatedAtLoopExit");
    mAnyProgressedAtLoopExitPhi = b->CreatePHI(boolTy, 2, prefix + "_anyProgressAtLoopExit");
    if (mIsPartitionRoot && mKernelIsInternallySynchronized) {
        mTotalNumOfStridesAtLoopExitPhi = nullptr;
    } else {
        mTotalNumOfStridesAtLoopExitPhi = b->CreatePHI(sizeTy, 2, prefix + "_totalNumOfStridesAtLoopExit");
    }
    mExhaustedPipelineInputAtLoopExitPhi = b->CreatePHI(boolTy, 2, prefix + "_exhaustedInputAtLoopExit");
    mFinalPartitionSegmentAtLoopExitPhi = nullptr;
    if (mIsPartitionRoot) {
        mFinalPartitionSegmentAtLoopExitPhi = b->CreatePHI(boolTy, 2, prefix + "_finalPartitionSegmentAtLoopExitPhi");
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeInsufficientIOExit
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeInsufficientIOExit(BuilderRef b) {

    // A partition root will always have an insufficient I/O check since they control how many strides the
    // other kernels in the partition will execute. If a kernel has non-linear I/O, however, we need to test
    // whether we've finished executing.

    b->SetInsertPoint(mKernelInsufficientInput);

    BasicBlock * const exitBlock = b->GetInsertBlock();

    if (mMayLoopToEntry) {
        for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
            const auto port = mBufferGraph[e].Port;
            mUpdatedProcessedPhi[port]->addIncoming(mAlreadyProcessedPhi[port], exitBlock);
            if (mAlreadyProcessedDeferredPhi[port]) {
                mUpdatedProcessedDeferredPhi[port]->addIncoming(mAlreadyProcessedDeferredPhi[port], exitBlock);
            }
        }

        for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
            const auto port = mBufferGraph[e].Port;
            mUpdatedProducedPhi[port]->addIncoming(mAlreadyProducedPhi[port], exitBlock);
            if (mAlreadyProducedDeferredPhi[port]) {
                mUpdatedProducedDeferredPhi[port]->addIncoming(mAlreadyProducedDeferredPhi[port], exitBlock);
            }
        }
    }

    if (mMayLoopToEntry || !mIsPartitionRoot) {
        if (mTotalNumOfStridesAtLoopExitPhi) {
            Value * currentNumOfStrides;
            if (mMayLoopToEntry) {
                currentNumOfStrides = mCurrentNumOfStridesAtLoopEntryPhi;
            } else {
                currentNumOfStrides = b->getSize(0);
            }
            mTotalNumOfStridesAtLoopExitPhi->addIncoming(currentNumOfStrides, exitBlock);
        }
        assert (mExhaustedPipelineInputPhi);
        mExhaustedPipelineInputAtLoopExitPhi->addIncoming(mExhaustedPipelineInputPhi, exitBlock);
        assert (mAlreadyProgressedPhi);
        mAnyProgressedAtLoopExitPhi->addIncoming(mAlreadyProgressedPhi, exitBlock);
        mTerminatedAtLoopExitPhi->addIncoming(mInitialTerminationSignal, exitBlock);
    }

    if (mIsPartitionRoot) {
        assert (mInitialTerminationSignal);
        if (mExhaustedInputAtJumpPhi) {
            mExhaustedInputAtJumpPhi->addIncoming(mExhaustedPipelineInputPhi, exitBlock);
        }
        if (mMayLoopToEntry) {
            mFinalPartitionSegmentAtLoopExitPhi->addIncoming(b->getFalse(), exitBlock);
            b->CreateLikelyCondBr(mExecutedAtLeastOnceAtLoopEntryPhi, mKernelLoopExit, mKernelJumpToNextUsefulPartition);
        } else {
            b->CreateBr(mKernelJumpToNextUsefulPartition);
        }
    } else {
        // if this is not a partition root, it is not responsible for determining
        // whether the partition is out of input
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
    if (mIsPartitionRoot && mKernelIsInternallySynchronized) {
        mTotalNumOfStridesAtExitPhi = nullptr;
    } else {
        mTotalNumOfStridesAtExitPhi = b->CreatePHI(sizeTy, 2, prefix + "_totalNumOfStridesAtExit");
        mTotalNumOfStridesAtExitPhi->addIncoming(mTotalNumOfStridesAtLoopExitPhi, mKernelLoopExitPhiCatch);
    }
    createConsumedPhiNodes(b);

    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto port = mBufferGraph[e].Port;
        const auto prefix = makeBufferName(mKernelId, port);
        PHINode * const fullyProduced = b->CreatePHI(sizeTy, 2, prefix + "_fullyProducedAtKernelExit");
        mFullyProducedItemCount[port] = fullyProduced;
    }

    PHINode * const progress = b->CreatePHI(boolTy, 2, prefix + "_anyProgressAtKernelExit");
    progress->addIncoming(mAnyProgressedAtLoopExitPhi, mKernelLoopExitPhiCatch);
    mAnyProgressedAtExitPhi = progress;

    PHINode * const exhausted = b->CreatePHI(boolTy, 2, prefix + "_exhaustedPipelineInputAtKernelExit");
    exhausted->addIncoming(mExhaustedPipelineInputAtLoopExitPhi, mKernelLoopExitPhiCatch);
    mExhaustedPipelineInputAtExit = exhausted;

    if (mIsPartitionRoot) {
        mFinalPartitionSegmentAtExitPhi = b->CreatePHI(boolTy, 2, prefix + "_anyProgressAtKernelExit");
        mFinalPartitionSegmentAtExitPhi->addIncoming(mFinalPartitionSegmentAtLoopExitPhi, mKernelLoopExitPhiCatch);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateKernelExitPhisAfterInitiallyTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::updateKernelExitPhisAfterInitiallyTerminated(BuilderRef b) {
    Constant * const completed = getTerminationSignal(b, TerminationSignal::Completed);
    mTerminatedAtExitPhi->addIncoming(completed, mKernelInitiallyTerminatedExit);
    if (mTotalNumOfStridesAtExitPhi) {
        ConstantInt * const sz_ZERO = b->getSize(0);
        mTotalNumOfStridesAtExitPhi->addIncoming(sz_ZERO, mKernelInitiallyTerminatedExit);
    }

    phiOutConsumedItemCountsAfterInitiallyTerminated(b);

    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const auto port = mBufferGraph[e].Port;
        mFullyProducedItemCount[port]->addIncoming(mInitiallyProducedItemCount[streamSet], mKernelInitiallyTerminatedExit);
    }

    mAnyProgressedAtExitPhi->addIncoming(mPipelineProgress, mKernelInitiallyTerminatedExit);
    cast<PHINode>(mExhaustedPipelineInputAtExit)->addIncoming(mExhaustedInput, mKernelInitiallyTerminatedExit);
    if (mIsPartitionRoot) {
        mFinalPartitionSegmentAtExitPhi->addIncoming(b->getTrue(), mKernelInitiallyTerminatedExit);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updatePhiCountAfterTermination
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::updatePhisAfterTermination(BuilderRef b) {
    BasicBlock * const exitBlock = b->GetInsertBlock();
    mTerminatedAtLoopExitPhi->addIncoming(mTerminatedSignalPhi, exitBlock);
    mAnyProgressedAtLoopExitPhi->addIncoming(b->getTrue(), exitBlock);
    mExhaustedPipelineInputAtLoopExitPhi->addIncoming(mExhaustedInput, exitBlock);
    if (mTotalNumOfStridesAtLoopExitPhi) {
        Value * finalNumOfStrides = mUpdatedNumOfStrides; assert (mUpdatedNumOfStrides);
        if (mIsPartitionRoot) {
            if (mFinalPartialStrideFixedRateRemainderPhi) {
                const Rational fixedRateFactor = mFixedRateLCM * Rational{mKernel->getStride()};
                Value * fixedRateItems = b->CreateMulRational(finalNumOfStrides, fixedRateFactor);
                fixedRateItems = b->CreateAdd(fixedRateItems, mFinalPartialStrideFixedRateRemainderPhi);
                finalNumOfStrides = b->CreateMulRational(fixedRateItems, mPartitionStrideRateScalingFactor / fixedRateFactor);
            } else {
                finalNumOfStrides = b->CreateMulRational(finalNumOfStrides, mPartitionStrideRateScalingFactor);
            }
        }
        mTotalNumOfStridesAtLoopExitPhi->addIncoming(finalNumOfStrides, exitBlock);
    }
    if (mIsPartitionRoot) {
        mFinalPartitionSegmentAtLoopExitPhi->addIncoming(b->getTrue(), exitBlock);
    }
    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const auto port = mBufferGraph[e].Port;
        Value * const totalCount = getLocallyAvailableItemCount(b, port);
        mUpdatedProcessedPhi[port]->addIncoming(totalCount, exitBlock);
        if (mUpdatedProcessedDeferredPhi[port]) {
            mUpdatedProcessedDeferredPhi[port]->addIncoming(totalCount, exitBlock);
        }
    }

    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto port = mBufferGraph[e].Port;
        Value * const produced = mProducedAtTerminationPhi[port];

        #ifdef PRINT_DEBUG_MESSAGES
        debugPrint(b, makeBufferName(mKernelId, port) + "_producedAtTermination = %" PRIu64, produced);
        #endif

        mUpdatedProducedPhi[port]->addIncoming(produced, exitBlock);
        if (mUpdatedProducedDeferredPhi[port]) {
            mUpdatedProducedDeferredPhi[port]->addIncoming(produced, exitBlock);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief end
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::end(BuilderRef b) {



    // A pipeline will end for one or two reasons:

    // 1) Process has *halted* due to insufficient external I/O.

    // 2) All pipeline sinks have terminated (i.e., any kernel that writes
    // to a pipeline output, is marked as having a side-effect, or produces
    // an input for some call in which no dependent kernels is a pipeline
    // sink).

    // TODO: if we determine that all of the pipeline I/O is consumed in one invocation of the
    // pipeline, we can avoid testing at the end whether its terminated.

    Value * terminated = nullptr;
    if (ExternallySynchronized) {
        if (mCurrentThreadTerminationSignalPtr) {
            terminated = hasPipelineTerminated(b);
        }
        b->CreateBr(mPipelineEnd);
    } else {

        terminated = hasPipelineTerminated(b);

        Value * const done = b->CreateIsNotNull(terminated);

        if (LLVM_UNLIKELY(CheckAssertions)) {
            Value * const progressedOrFinished = b->CreateOr(mPipelineProgress, done);
            Value * const live = b->CreateOr(mMadeProgressInLastSegment, progressedOrFinished);
            b->CreateAssert(live, "Dead lock detected: pipeline could not progress after two iterations");
        }
        BasicBlock * const exitBlock = b->GetInsertBlock();
        mMadeProgressInLastSegment->addIncoming(mPipelineProgress, exitBlock);
        updateLocallyAvailableItemCounts(b, exitBlock);
        incrementCurrentSegNo(b, exitBlock);
        b->CreateUnlikelyCondBr(done, mPipelineEnd, mPipelineLoop);
    }
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

    #ifdef ENABLE_PAPI
    stopPAPIAndDestroyEventSet(b);
    #endif

    if (LLVM_UNLIKELY(canSetTerminateSignal())) {
        Constant * const unterminated = b->getSize(KernelBuilder::TerminationCode::None);
        Constant * const terminated = b->getSize(KernelBuilder::TerminationCode::Terminated);
        Value * const retVal = b->CreateSelect(mPipelineProgress, unterminated, terminated);
        b->setTerminationSignal(retVal);
    }

    mExpectedNumOfStridesMultiplier = nullptr;
    mThreadLocalStreamSetBaseAddress = nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeExternalProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeExternalProducedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        const BufferPort & external = mBufferGraph[e];
        const auto streamSet = source(e, mBufferGraph);
        Value * const ptr = getProducedOutputItemsPtr(external.Port.Number);
        b->CreateStore(mLocallyAvailableItems[streamSet], ptr);
    }
}

}
