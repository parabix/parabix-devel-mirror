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

    if (LLVM_LIKELY(RequiredThreadLocalStreamSetMemory > 0)) {
        mExpectedNumOfStridesMultiplier = b->getScalarField(EXPECTED_NUM_OF_STRIDES_MULTIPLIER);
        mThreadLocalStreamSetBaseAddress = b->getScalarField(BASE_THREAD_LOCAL_STREAMSET_MEMORY);
    }

    loadInternalStreamSetHandles(b, true);
    loadInternalStreamSetHandles(b, false);
    readExternalConsumerItemCounts(b);
    initializePipelineInputTerminationSignal(b);
    identifyAllInternallySynchronizedKernels();

    mKernel = nullptr;
    mKernelId = 0;

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
    identifyLocalPortIds(mKernelId);

    const auto kernelRequiresSynchronization = RequiresSynchronization[mKernelId];

    if (kernelRequiresSynchronization) {
        mMayHaveNonLinearIO = mayHaveNonLinearIO(mKernelId);
        mIsBounded = isBounded();
        mHasExplicitFinalPartialStride = requiresExplicitFinalStride();
        const auto nonSourceKernel = in_degree(mKernelId, mBufferGraph) > 0;

        mMayLoopToEntry = nonSourceKernel && (mMayHaveNonLinearIO || mHasExplicitFinalPartialStride || ExternallySynchronized);
//        if (mMayLoopToEntry) {
//            for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
//                const BufferPort & bp = mBufferGraph[e];
//                const Binding & binding = bp.Binding;
//                // any kernel with a greedy input rate must complete its work in a single invocation.
//                if (binding.getRate().isGreedy()) {
//                    mMayLoopToEntry = false;
//                    break;
//                }
//            }
//        }

        #ifdef CHECK_EVERY_IO_PORT
        mCheckIO = nonSourceKernel;
        #else
        mCheckIO = nonSourceKernel && (mIsPartitionRoot || mMayHaveNonLinearIO || TraceIO || hasExternalIO(mKernelId));
        #endif
    } else {
        mHasExplicitFinalPartialStride = false;
        mMayHaveNonLinearIO = false;
        mMayLoopToEntry = false;
        mIsBounded = false;
        mCheckIO = false;
    }


    #ifdef INITIALLY_TERMINATED_KERNELS_JUMP_TO_NEXT_PARTITION
    const auto nextPartitionId = mCurrentPartitionId + 1U;
    const auto jumpId = mPartitionJumpIndex[mCurrentPartitionId];
    const auto canJumpToAnotherPartition = mIsPartitionRoot && (mIsBounded || nextPartitionId == jumpId);
    const auto handleNoUpdateExit = mIsPartitionRoot || !canJumpToAnotherPartition;
    #else
    const auto canJumpToAnotherPartition = mIsPartitionRoot;
    const auto handleNoUpdateExit = mCheckIO;
    #endif

    const auto prefix = makeKernelName(mKernelId);

    /// -------------------------------------------------------------------------------------
    /// BASIC BLOCK CONSTRUCTION
    /// -------------------------------------------------------------------------------------

    mKernelLoopEntry = b->CreateBasicBlock(prefix + "_loopEntry", mNextPartitionEntryPoint);
    mKernelCheckOutputSpace = nullptr;
    if (kernelRequiresSynchronization) {
        mKernelCheckOutputSpace = b->CreateBasicBlock(prefix + "_checkOutputSpace", mNextPartitionEntryPoint);
    }
    mKernelLoopCall = b->CreateBasicBlock(prefix + "_executeKernel", mNextPartitionEntryPoint);
    mKernelCompletionCheck = b->CreateBasicBlock(prefix + "_normalCompletionCheck", mNextPartitionEntryPoint);
    if (mCheckIO) {
        mKernelInsufficientInput = b->CreateBasicBlock(prefix + "_insufficientInput", mNextPartitionEntryPoint);
    }
    if (handleNoUpdateExit) {
        mKernelInitiallyTerminated = b->CreateBasicBlock(prefix + "_initiallyTerminated", mNextPartitionEntryPoint);
    }
    if (canJumpToAnotherPartition) {
        SmallVector<char, 256> tmp;
        raw_svector_ostream nm(tmp);
        nm << prefix << "_jumpFromPartition_" << mCurrentPartitionId
           << "_to_" << mPartitionJumpIndex[mCurrentPartitionId];
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

    checkIfKernelIsAlreadyTerminated(b);
    readProcessedItemCounts(b);
    readProducedItemCounts(b);
    readConsumedItemCounts(b);

    prepareLinearThreadLocalBuffers(b);

    incrementNumberOfSegmentsCounter(b);
    recordUnconsumedItemCounts(b);

    if (mIsPartitionRoot) {
        mFinalPartialStrideFixedRateRemainderPhi = nullptr;
        mCurrentPartitionEntryGuard = b->CreateBasicBlock(prefix + "_partitionEntry", mKernelLoopEntry);
        b->CreateUnlikelyCondBr(mInitiallyTerminated, mKernelInitiallyTerminated, mCurrentPartitionEntryGuard);
        mKernelLoopStart = mCurrentPartitionEntryGuard;
    } else {
        detemineMaximumNumberOfStrides(b);
        b->CreateBr(mKernelLoopEntry);
        mKernelLoopStart = b->GetInsertBlock();
    }

    /// -------------------------------------------------------------------------------------
    /// PHI NODE INITIALIZATION
    /// -------------------------------------------------------------------------------------

    // Set up some PHI nodes early to simplify accumulating their incoming values.
    initializeKernelLoopEntryPhis(b);
    if (kernelRequiresSynchronization) {
        initializeKernelCheckOutputSpacePhis(b);
    }
    if (canJumpToAnotherPartition) {
        initializeJumpToNextUsefulPartitionPhis(b);
    }
    if (mCheckIO) {
        initializeKernelInsufficientIOExitPhis(b);
    }
    initializeKernelTerminatedPhis(b);
    initializeKernelLoopExitPhis(b);
    initializeKernelExitPhis(b);

    /// -------------------------------------------------------------------------------------
    /// PARTITION INPUT CHECK GUARD ENTRY
    /// -------------------------------------------------------------------------------------
    if (mIsPartitionRoot) {
        writePartitionEntryIOGuard(b);
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL LOOP ENTRY
    /// -------------------------------------------------------------------------------------

    #warning fix this to determine the initial (non-linear) number of strides

    b->SetInsertPoint(mKernelLoopEntry);
    if (kernelRequiresSynchronization) {
        determineNumOfLinearStrides(b);
    } else {
        determineIsFinal(b);
        // FIXME: temporary change to minimize changes to PHI nodes
        mUpdatedNumOfStrides = mMaximumNumOfStrides;
    }

    // When tracing blocking I/O, test all I/O streams but do not execute the
    // kernel if any stream is insufficient.
    if (mCheckIO && TraceIO) {
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
    if (kernelRequiresSynchronization) {
        writeLookBehindLogic(b);
    }
    writeKernelCall(b);
    if (kernelRequiresSynchronization) {
        writeCopyBackLogic(b);
        // writeDelayReflectionLogic(b);
    }

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
    if (kernelRequiresSynchronization) {
        clearUnwrittenOutputData(b);
    }
    updatePhisAfterTermination(b);
    b->CreateBr(mKernelLoopExit);

    /// -------------------------------------------------------------------------------------
    /// KERNEL INSUFFICIENT IO EXIT
    /// -------------------------------------------------------------------------------------

    if (mCheckIO) {
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
    if (kernelRequiresSynchronization) {
        writeLookAheadLogic(b);
    }
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
        b->SetInsertPoint(mKernelJumpToNextUsefulPartition);
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
        assert (mTotalNumOfStridesAtExitPhi);
        mNumOfPartitionStrides = mTotalNumOfStridesAtExitPhi;
    }

    updateCycleCounter(b, mKernelId, mKernelStartTime, CycleCounter::TOTAL_TIME);

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

        BasicBlock * const entryBlock = b->GetInsertBlock();

        for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
            const auto port = mBufferGraph[e].Port;
            mAlreadyProcessedPhi[port]->addIncoming(mProcessedItemCount[port], entryBlock);
            if (mAlreadyProcessedDeferredPhi[port]) {
                mAlreadyProcessedDeferredPhi[port]->addIncoming(mProcessedDeferredItemCount[port], entryBlock);
            }
        }

        for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
            const auto port = mBufferGraph[e].Port;
            mAlreadyProducedPhi[port]->addIncoming(mProducedItemCount[port], entryBlock);
            if (mAlreadyProducedDeferredPhi[port]) {
                mAlreadyProducedDeferredPhi[port]->addIncoming(mProducedDeferredItemCount[port], entryBlock);
            }
        }

        mAlreadyProgressedPhi->addIncoming(i1_TRUE, entryBlock);
        mExecutedAtLeastOnceAtLoopEntryPhi->addIncoming(i1_TRUE, entryBlock);
        if (mCurrentNumOfStridesAtLoopEntryPhi) {
            mCurrentNumOfStridesAtLoopEntryPhi->addIncoming(mUpdatedNumOfStrides, entryBlock);
        }

        const auto prefix = makeKernelName(mKernelId);
        BasicBlock * const isFinalCheck = b->CreateBasicBlock(prefix + "_isFinalCheck", mKernelTerminated);
        b->CreateCondBr(loopAgain, mKernelLoopEntry, isFinalCheck);

        b->SetInsertPoint(isFinalCheck);
    }

    Value * terminationSignal = mIsFinalInvocationPhi;
    if (mKernelIsInternallySynchronized) {
        Constant * const completed = getTerminationSignal(b, TerminationSignal::Completed);
        Constant * const unterminated = getTerminationSignal(b, TerminationSignal::None);
        terminationSignal = b->CreateSelect(mKernelIsFinal, completed, unterminated);
    }

    assert (terminationSignal);

    BasicBlock * const exitBlock = b->GetInsertBlock();

    // update KernelTerminated phi nodes
    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto port = mBufferGraph[e].Port;
        mProducedAtTerminationPhi[port]->addIncoming(mProducedItemCount[port], exitBlock);
    }
    mTerminatedSignalPhi->addIncoming(terminationSignal, exitBlock);
//    if (mIsPartitionRoot) {
//        Value * incomingValue = mFinalPartialStrideFixedRateRemainderPhi;
//        if (incomingValue == nullptr) {
//            incomingValue = b->getSize(0);
//        }
//        mFinalPartialStrideFixedRateRemainderAtLoopExitPhi->addIncoming(incomingValue, exitBlock);
//    }

    b->CreateUnlikelyCondBr(mKernelIsFinal, mKernelTerminated, mKernelLoopExit);

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
    if (mTotalNumOfStridesAtLoopExitPhi) {
        mTotalNumOfStridesAtLoopExitPhi->addIncoming(mUpdatedNumOfStrides, exitBlock);
    }
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
    mFixedRateFactorPhi = nullptr;
    const auto prefix = makeKernelName(mKernelId);
    if (LLVM_LIKELY(mKernel->hasFixedRateInput())) {
        mFixedRateFactorPhi = b->CreatePHI(sizeTy, 2, prefix + "_fixedRateFactor");
    }
    mIsFinalInvocationPhi = b->CreatePHI(sizeTy, 2, prefix + "_isFinalPhi");
    mFinalPartialStrideFixedRateRemainderPhi = nullptr;
    if (mIsPartitionRoot) {
        mFinalPartialStrideFixedRateRemainderPhi = b->CreatePHI(sizeTy, 2, prefix + "_partialPartitionStrides");
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
//    if (mIsPartitionRoot) {
//        mFinalPartialStrideFixedRateRemainderAtLoopExitPhi = b->CreatePHI(sizeTy, 2, prefix + "_partialPartitionStridesAtLoopExit");
//    }
}


#if 0
/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeInsufficientIOExit
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeInsufficientIOExit(BuilderRef b) {

    // A partition root will always have an insufficient I/O check since they control how many strides the
    // other kernels in the partition will execute. If a kernel has non-linear I/O, however, we need to test
    // whether we've finished executing.

    BasicBlock * const exitBlock = b->GetInsertBlock();

    ConstantInt * const ZERO = b->getSize(0);

    if (!mIsPartitionRoot || mMayLoopToEntry) {

        assert (mTerminatedInitially);

        mTerminatedAtLoopExitPhi->addIncoming(mTerminatedInitially, exitBlock);
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

        if (mTotalNumOfStridesAtLoopExitPhi) {
            Value * currentNumOfStrides;
            if (mMayLoopToEntry) {
                currentNumOfStrides = mCurrentNumOfStridesAtLoopEntryPhi;
            } else {
                currentNumOfStrides = ZERO;
            }
            mTotalNumOfStridesAtLoopExitPhi->addIncoming(currentNumOfStrides, exitBlock);
        }
        assert (mExhaustedPipelineInputPhi);
        mExhaustedPipelineInputAtLoopExitPhi->addIncoming(mExhaustedPipelineInputPhi, exitBlock);
    }

    if (mMayLoopToEntry) {
        assert (mAlreadyProgressedPhi);
        mAnyProgressedAtLoopExitPhi->addIncoming(mAlreadyProgressedPhi, exitBlock);
    }

    if (mIsPartitionRoot) {
        assert (mKernelJumpToNextUsefulPartition);
        assert (mNextPartitionWithPotentialInput);
        if (mExhaustedInputAtJumpPhi) {
            mExhaustedInputAtJumpPhi->addIncoming(mExhaustedPipelineInputPhi, exitBlock);
        }
        if (mMayLoopToEntry) {
            if (mIsPartitionRoot) {
                mPartialPartitionStridesAtLoopExitPhi->addIncoming(ZERO, exitBlock);
            }
            if (CheckAssertions) {
                const auto prefix = makeKernelName(mKernelId);
                b->CreateAssert(mExecutedAtLeastOnceAtLoopEntryPhi, prefix + " failed to execute a single stride");
            }
            b->CreateBr(mKernelLoopExit);
            // b->CreateLikelyCondBr(mExecutedAtLeastOnceAtLoopEntryPhi, mKernelLoopExit, mKernelJumpToNextUsefulPartition);
        } else {
            b->CreateBr(mKernelJumpToNextUsefulPartition);
        }
    } else {
        // if this is not a partition root, it is not responsible for determining
        // whether the partition is out of input
        b->CreateBr(mKernelLoopExit);
    }
}
#endif


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeInsufficientIOExit
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeInsufficientIOExit(BuilderRef b) {

    // A partition root will always have an insufficient I/O check since they control how many strides the
    // other kernels in the partition will execute. If a kernel has non-linear I/O, however, we need to test
    // whether we've finished executing.

    b->SetInsertPoint(mKernelInsufficientInput);

    BasicBlock * const exitBlock = b->GetInsertBlock();

    ConstantInt * const ZERO = b->getSize(0);

    if (mMayLoopToEntry) {

        assert (mInitialTerminationSignal);

        mTerminatedAtLoopExitPhi->addIncoming(mInitialTerminationSignal, exitBlock);
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

        if (mTotalNumOfStridesAtLoopExitPhi) {
            Value * currentNumOfStrides;
            if (mMayLoopToEntry) {
                currentNumOfStrides = mCurrentNumOfStridesAtLoopEntryPhi;
            } else {
                currentNumOfStrides = ZERO;
            }
            mTotalNumOfStridesAtLoopExitPhi->addIncoming(currentNumOfStrides, exitBlock);
        }
        assert (mExhaustedPipelineInputPhi);
        mExhaustedPipelineInputAtLoopExitPhi->addIncoming(mExhaustedPipelineInputPhi, exitBlock);
        assert (mAlreadyProgressedPhi);
        mAnyProgressedAtLoopExitPhi->addIncoming(mAlreadyProgressedPhi, exitBlock);
    }

    if (mIsPartitionRoot) {
//        assert (mKernelJumpToNextUsefulPartition);
//        assert (mNextPartitionWithPotentialInput);
//        if (mExhaustedInputAtJumpPhi) {
//            mExhaustedInputAtJumpPhi->addIncoming(mExhaustedPipelineInputPhi, exitBlock);
//        }
//        mFinalPartialStrideFixedRateRemainderAtLoopExitPhi->addIncoming(ZERO, exitBlock);
        if (CheckAssertions) {
            const auto prefix = makeKernelName(mKernelId);
            b->CreateAssert(mExecutedAtLeastOnceAtLoopEntryPhi, prefix + " failed to execute a single stride");
        }
    }

    b->CreateBr(mKernelLoopExit);

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

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateKernelExitPhisAfterInitiallyTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::updateKernelExitPhisAfterInitiallyTerminated(BuilderRef b) {
    Constant * const completed = getTerminationSignal(b, TerminationSignal::Completed);
    mTerminatedAtExitPhi->addIncoming(completed, mKernelInitiallyTerminatedExit);
    if (mTotalNumOfStridesAtExitPhi) {
        mTotalNumOfStridesAtExitPhi->addIncoming(b->getSize(0), mKernelInitiallyTerminatedExit);
    }

    phiOutConsumedItemCountsAfterInitiallyTerminated(b);

    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const auto port = mBufferGraph[e].Port;
        mFullyProducedItemCount[port]->addIncoming(mInitiallyProducedItemCount[streamSet], mKernelInitiallyTerminatedExit);
    }

    mAnyProgressedAtExitPhi->addIncoming(mPipelineProgress, mKernelInitiallyTerminatedExit);
    cast<PHINode>(mExhaustedPipelineInputAtExit)->addIncoming(mExhaustedInput, mKernelInitiallyTerminatedExit);
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
        Value * totalNumOfStrides = mUpdatedNumOfStrides;
//        if (mIsPartitionRoot && mFinalPartialStrideFixedRateRemainderPhi) {
//            Value * const nonEmptyFinalStride = b->CreateIsNotNull(mFinalPartialStrideFixedRateRemainderPhi);
//            totalNumOfStrides = b->CreateAdd(totalNumOfStrides, b->CreateZExt(nonEmptyFinalStride, b->getSizeTy()));
//        }
        mTotalNumOfStridesAtLoopExitPhi->addIncoming(totalNumOfStrides, exitBlock);
    }
//    if (mIsPartitionRoot) {
//        Value * incomingValue = mFinalPartialStrideFixedRateRemainderPhi;
//        if (incomingValue == nullptr) {
//            incomingValue = b->getSize(0);
//        }
//        mFinalPartialStrideFixedRateRemainderAtLoopExitPhi->addIncoming(incomingValue, exitBlock);
//    }
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
    if (LLVM_UNLIKELY(canSetTerminateSignal())) {
        Constant * const unterminated = b->getSize(KernelBuilder::TerminationCode::None);
        Constant * const terminated = b->getSize(KernelBuilder::TerminationCode::Terminated);
        Value * const retVal = b->CreateSelect(mPipelineProgress, unterminated, terminated);
        b->setTerminationSignal(retVal);
    }

   // b->GetInsertBlock()->getParent()->print(errs());
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
