#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief start
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::start(BuilderRef b, Value * const initialSegNo) {

    // Create the basic blocks for the loop.
    BasicBlock * const entryBlock = b->GetInsertBlock();
    mPipelineLoop = b->CreateBasicBlock("pipelineLoop");
    mPipelineEnd = b->CreateBasicBlock("pipelineEnd");

    mKernel = nullptr;
    mKernelIndex = 0;
    b->CreateBr(mPipelineLoop);

    b->SetInsertPoint(mPipelineLoop);
    IntegerType * const sizeTy = b->getSizeTy();
    ConstantInt * const ZERO = b->getSize(0);
    ConstantInt * const NOT_TERMINATED = b->getSize(NotTerminated);

    mSegNo = b->CreatePHI(sizeTy, 2, "segNo");
    mSegNo->addIncoming(initialSegNo, entryBlock);
    mProgressCounter = b->CreatePHI(sizeTy, 2, "progressCounter");
    mProgressCounter->addIncoming(ZERO, entryBlock);
    mPipelineProgress = b->getFalse();
    // any pipeline input streams are considered produced by the P_{in} vertex.
    mTerminationGraph[0] = mPipelineKernel->isFinal();

    mPipelineTerminated = NOT_TERMINATED;
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt("+++ pipeline start +++", mSegNo);
    #endif
    startOptionalCycleCounter(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief executeKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::executeKernel(BuilderRef b) {

    resetMemoizedFields();
    mPortOrdering = lexicalOrderingOfStreamIO();

    const auto prefix = makeKernelName(mKernelIndex);
    mKernelLoopEntry = b->CreateBasicBlock(prefix + "_loopEntry", mPipelineEnd);
    mKernelLoopCall = b->CreateBasicBlock(prefix + "_executeKernel", mPipelineEnd);
    mKernelTerminationCheck = b->CreateBasicBlock(prefix + "_normalTerminationCheck", mPipelineEnd);
    mKernelTerminated = b->CreateBasicBlock(prefix + "_terminated", mPipelineEnd);
    mKernelLoopExit = b->CreateBasicBlock(prefix + "_loopExit", mPipelineEnd);
    mKernelExit = b->CreateBasicBlock(prefix + "_kernelExit", mPipelineEnd);
    // The phi catch simplifies compilation logic by "forward declaring" the loop exit point.
    // Subsequent optimization phases will collapse it into the correct exit block.
    mKernelLoopExitPhiCatch = b->CreateBasicBlock(prefix + "_kernelExitPhiCatch", mPipelineEnd);

    /// -------------------------------------------------------------------------------------
    /// KERNEL ENTRY
    /// -------------------------------------------------------------------------------------

    loadBufferHandles(b);
    readInitialItemCounts(b);
    mKernelEntry = b->GetInsertBlock();
    b->CreateUnlikelyCondBr(initiallyTerminated(b), mKernelExit, mKernelLoopEntry);

    // Set up some PHI nodes early to simplify accumulating their incoming values.
    initializeKernelLoopEntryPhis(b);
    initializeKernelCallPhis(b);
    initializeKernelTerminatedPhis(b);
    initializeKernelLoopExitPhis(b);
    initializeKernelExitPhis(b);

    /// -------------------------------------------------------------------------------------
    /// KERNEL LOOP ENTRY
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopEntry);
    readConsumedItemCounts(b);
    checkForSufficientInputDataAndOutputSpace(b);
    determineNumOfLinearStrides(b);

    // TODO: it would be better to try and statically prove whether a kernel will only ever
    // need a single "run" per segment rather than allowing only source kernels to have this
    // optimization.

    Value * isFinal = nullptr;

    if (mNumOfLinearStrides) {

        BasicBlock * const enteringNonFinalSegment = b->CreateBasicBlock(prefix + "_nonFinalSegment", mKernelLoopCall);
        BasicBlock * const enteringFinalStride = b->CreateBasicBlock(prefix + "_finalStride", mKernelLoopCall);

        isFinal = b->CreateICmpEQ(mNumOfLinearStrides, b->getSize(0));

        b->CreateUnlikelyCondBr(isFinal, enteringFinalStride, enteringNonFinalSegment);

        /// -------------------------------------------------------------------------------------
        /// KERNEL ENTERING FINAL STRIDE
        /// -------------------------------------------------------------------------------------

        b->SetInsertPoint(enteringFinalStride);
        calculateFinalItemCounts(b);
        b->CreateBr(mKernelLoopCall);

        /// -------------------------------------------------------------------------------------
        /// KERNEL ENTERING NON-FINAL SEGMENT
        /// -------------------------------------------------------------------------------------

        b->SetInsertPoint(enteringNonFinalSegment);
        calculateNonFinalItemCounts(b);
        b->CreateBr(mKernelLoopCall);

    } else {

        mNumOfLinearStrides = b->getSize(1);
        calculateNonFinalItemCounts(b);
        b->CreateBr(mKernelLoopCall);

    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL CALL
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopCall);
    writeKernelCall(b);
    writeCopyBackLogic(b);

    BasicBlock * const abnormalTermination =
            b->CreateBasicBlock(prefix + "_abnormalTermination", mKernelTerminationCheck);

    // If the kernel explicitly terminates, it must set its processed/produced item counts.
    // Otherwise, the pipeline will update any countable rates, even upon termination.
    b->CreateUnlikelyCondBr(mTerminatedExplicitly, abnormalTermination, mKernelTerminationCheck);

    /// -------------------------------------------------------------------------------------
    /// KERNEL NORMAL TERMINATION CHECK
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelTerminationCheck);
    normalTerminationCheck(b, isFinal);

    /// -------------------------------------------------------------------------------------
    /// KERNEL ABNORMAL TERMINATION
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(abnormalTermination);
    loadItemCountsOfCountableRateStreams(b);
    b->CreateBr(mKernelTerminated);

    /// -------------------------------------------------------------------------------------
    /// KERNEL TERMINATED
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelTerminated);
    zeroFillPartiallyWrittenOutputStreams(b);
    Value * mode = setTerminated(b, mTerminatedExplicitly, TerminatedExplicitly, TerminatedNormally);
    updatePhisAfterTermination(b, mode);
    b->CreateBr(mKernelLoopExit);

    /// -------------------------------------------------------------------------------------
    /// KERNEL LOOP EXIT
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopExit);
    writeUpdatedItemCounts(b);
    computeFullyProcessedItemCounts(b);
    computeMinimumConsumedItemCounts(b);
    computeMinimumPopCountReferenceCounts(b);
    writeCopyForwardLogic(b);
    writePopCountComputationLogic(b);
    computeFullyProducedItemCounts(b);
    mKernelLoopExitPhiCatch->moveAfter(b->GetInsertBlock());
    b->CreateBr(mKernelLoopExitPhiCatch);
    b->SetInsertPoint(mKernelLoopExitPhiCatch);
    b->CreateBr(mKernelExit);

    /// -------------------------------------------------------------------------------------
    /// KERNEL EXIT (CONTINUED)
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelExit);
    mKernelExit->moveAfter(mKernelLoopExitPhiCatch);
    writeFinalConsumedItemCounts(b);
    updatePopCountReferenceCounts(b);
    readFinalProducedItemCounts(b);
    updateOptionalCycleCounter(b);

    assert (mKernel == mPipeline[mKernelIndex] && b->getKernel() == mKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief next
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::end(BuilderRef b, const unsigned step) {
    b->setKernel(mPipelineKernel);

    ConstantInt * const ZERO = b->getSize(0);
    ConstantInt * const ONE = b->getSize(1);
    ConstantInt * const TWO = b->getSize(2);
    Value * const plusOne = b->CreateAdd(mProgressCounter, ONE);
    Value * const newProgressCounter = b->CreateSelect(mPipelineProgress, ZERO, plusOne);
    Value * const noProgress = b->CreateICmpEQ(newProgressCounter, TWO);

    const auto pipelineOutput = mLastKernel;

    // check whether every sink has terminated
    Value * allTerminated = b->getTrue();
    for (const auto e : make_iterator_range(in_edges(pipelineOutput, mTerminationGraph))) {
        const auto kernelVertex = source(e, mTerminationGraph);
        Value * const kernelState = mTerminationGraph[kernelVertex];
        Value * const terminated = b->CreateICmpNE(kernelState, b->getSize(NotTerminated));
        allTerminated = b->CreateAnd(allTerminated, terminated);
    }

    Value * done = allTerminated;
    if (nestedPipeline()) {
        done = b->CreateOr(allTerminated, noProgress);
    } else if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        b->CreateAssertZero(noProgress,
            "Dead lock detected: pipeline could not progress after two iterations");
    }

    #ifdef PRINT_DEBUG_MESSAGES
    Constant * const ONES = Constant::getAllOnesValue(mSegNo->getType());
    b->CallPrintInt("+++ pipeline end +++", b->CreateSelect(done, ONES, mSegNo));
    #endif

    Value * const nextSegNo = b->CreateAdd(mSegNo, b->getSize(step));
    BasicBlock * const exitBlock = b->GetInsertBlock();
    mSegNo->addIncoming(nextSegNo, exitBlock);
    mProgressCounter->addIncoming(newProgressCounter, exitBlock);
    b->CreateUnlikelyCondBr(done, mPipelineEnd, mPipelineLoop);

    b->SetInsertPoint(mPipelineEnd);
    mSegNo = nullptr;
    b->setKernel(mPipelineKernel);

// TODO: not correct for threaded pipelines
//    if (mPipelineKernel->canSetTerminateSignal()) {
//        Value * const terminatedPtr = mPipelineKernel->getTerminationSignalPtr();
//        b->CreateStore(allTerminated, terminatedPtr);
//    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelLoopEntryPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelLoopEntryPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelLoopEntry);
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    Type * const sizeTy = b->getSizeTy();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = mKernel->getInputStreamSetBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, input);
        mAlreadyProcessedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProcessed");
        mAlreadyProcessedPhi[i]->addIncoming(mInitiallyProcessedItemCount[i], mKernelEntry);
        if (mInitiallyProcessedDeferredItemCount[i]) {
            mAlreadyProcessedDeferredPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProcessedDeferred");
            mAlreadyProcessedDeferredPhi[i]->addIncoming(mInitiallyProcessedDeferredItemCount[i], mKernelEntry);
        }
    }
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = mKernel->getOutputStreamSetBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, output);
        mAlreadyProducedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProduced");
        mAlreadyProducedPhi[i]->addIncoming(mInitiallyProducedItemCount[i], mKernelEntry);
    }
    // Since we may loop and call the kernel again, we want to mark that we've progressed
    // if we execute any kernel even if we could not complete a full segment.
    const auto prefix = makeKernelName(mKernelIndex);
    mAlreadyProgressedPhi = b->CreatePHI(b->getInt1Ty(), 2, prefix + "_madeProgress");
    mAlreadyProgressedPhi->addIncoming(mPipelineProgress, mKernelEntry);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelCallPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelCallPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelLoopCall);
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    Type * const sizeTy = b->getSizeTy();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = mKernel->getInputStreamSetBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, input);
        mLinearInputItemsPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_linearlyAccessible");
    }
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = mKernel->getOutputStreamSetBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, output);
        mLinearOutputItemsPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_linearlyWritable");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelTerminatedPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelTerminatedPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelTerminated);
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    Type * const sizeTy = b->getSizeTy();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = mKernel->getInputStreamSetBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, input);
        mFinalProcessedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_finalProcessed");
    }
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = mKernel->getOutputStreamSetBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, output);
        mFinalProducedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_finalProduced");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelLoopExitPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelLoopExitPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelLoopExit);
    const auto prefix = makeKernelName(mKernelIndex);
    IntegerType * const sizeTy = b->getSizeTy();
    mTerminatedPhi = b->CreatePHI(sizeTy, 2, prefix + "_terminated");
    mHasProgressedPhi = b->CreatePHI(b->getInt1Ty(), 2, prefix + "_anyProgress");

    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = mKernel->getInputStreamSetBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, input);
        mUpdatedProcessedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessed");
        if (mAlreadyProcessedDeferredPhi[i]) {
            mUpdatedProcessedDeferredPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessedDeferred");
        }
    }
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = mKernel->getOutputStreamSetBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, output);
        mUpdatedProducedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_updatedProduced");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelExitPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelExitPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelExit);
    const auto prefix = makeKernelName(mKernelIndex);
    IntegerType * const sizeTy = b->getSizeTy();

    PHINode * const terminated = b->CreatePHI(sizeTy, 2, prefix + "_terminated");
    terminated->addIncoming(mTerminatedInitially, mKernelEntry);
    terminated->addIncoming(mTerminatedPhi, mKernelLoopExitPhiCatch);
    mTerminationGraph[mKernelIndex] = terminated;

    PHINode * const pipelineProgress = b->CreatePHI(b->getInt1Ty(), 2, prefix + "_pipelineProgress");
    pipelineProgress->addIncoming(mPipelineProgress, mKernelEntry);
    pipelineProgress->addIncoming(mHasProgressedPhi, mKernelLoopExitPhiCatch);
    mPipelineProgress = pipelineProgress;

    createConsumedPhiNodes(b);
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = mKernel->getOutputStreamSetBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, output);
        PHINode * const fullyProduced = b->CreatePHI(sizeTy, 2, prefix + "_fullyProduced");
        fullyProduced->addIncoming(mInitiallyProducedItemCount[i], mKernelEntry);
        mFullyProducedItemCount[i] = fullyProduced;
    }
    createPopCountReferenceCounts(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief normalTerminationCheck
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::normalTerminationCheck(BuilderRef b, Value * const isFinal) {
    BasicBlock * const entryBlock = b->GetInsertBlock();
    if (isFinal) {
        const auto numOfInputs = mKernel->getNumOfStreamInputs();
        for (unsigned i = 0; i < numOfInputs; ++i) {
            mAlreadyProcessedPhi[i]->addIncoming(mProcessedItemCount[i], entryBlock);
            if (mAlreadyProcessedDeferredPhi[i]) {
                mAlreadyProcessedDeferredPhi[i]->addIncoming(mProcessedDeferredItemCount[i], entryBlock);
            }
            mFinalProcessedPhi[i]->addIncoming(mProcessedItemCount[i], entryBlock);
        }
        const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            mAlreadyProducedPhi[i]->addIncoming(mProducedItemCount[i], entryBlock);
            mFinalProducedPhi[i]->addIncoming(mProducedItemCount[i], entryBlock);
        }
        if (mAlreadyProgressedPhi) {
            mAlreadyProgressedPhi->addIncoming(b->getTrue(), entryBlock);
        }
        b->CreateUnlikelyCondBr(isFinal, mKernelTerminated, mKernelLoopEntry);
    } else { // just exit the loop
        const auto numOfInputs = mKernel->getNumOfStreamInputs();
        for (unsigned i = 0; i < numOfInputs; ++i) {
            mUpdatedProcessedPhi[i]->addIncoming(mProcessedItemCount[i], entryBlock);
            if (mUpdatedProcessedDeferredPhi[i]) {
                mUpdatedProcessedDeferredPhi[i]->addIncoming(mProcessedDeferredItemCount[i], entryBlock);
            }
        }
        const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            mUpdatedProducedPhi[i]->addIncoming(mProducedItemCount[i], entryBlock);
        }
        mHasProgressedPhi->addIncoming(b->getTrue(), entryBlock);
        mTerminatedPhi->addIncoming(b->getSize(NotTerminated), entryBlock);
        b->CreateBr(mKernelLoopExit);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitialTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::initiallyTerminated(BuilderRef b) {
    b->setKernel(mPipelineKernel);
    const auto prefix = makeKernelName(mKernelIndex);
    mTerminatedInitially = b->getScalarField(prefix + TERMINATION_SIGNAL_SUFFIX);
    b->setKernel(mKernel);
    return b->CreateICmpNE(mTerminatedInitially, b->getSize(NotTerminated));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::setTerminated(BuilderRef b, Value * const condition, const TerminationMode trueMode, const TerminationMode falseMode) const  {
    const auto prefix = makeKernelName(mKernelIndex);
    b->setKernel(mPipelineKernel);
    ConstantInt * const TRUE_MODE = b->getSize(trueMode);
    ConstantInt * const FALSE_MODE = b->getSize(falseMode);
    Value * const mode = b->CreateSelect(condition, TRUE_MODE, FALSE_MODE);
    b->setScalarField(prefix + TERMINATION_SIGNAL_SUFFIX, mode);
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt("*** " + prefix + "_terminated ***", mode);
    #endif
    b->setKernel(mKernel);
    return mode;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updatePhiCountAfterTermination
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::updatePhisAfterTermination(BuilderRef b, Value * const terminationMode) {
    BasicBlock * const exitBlock = b->GetInsertBlock();
    mTerminatedPhi->addIncoming(terminationMode, exitBlock);
    mHasProgressedPhi->addIncoming(b->getTrue(), exitBlock);
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        Value * const totalCount = getTotalItemCount(b, i);
        mUpdatedProcessedPhi[i]->addIncoming(totalCount, exitBlock);
        if (mUpdatedProcessedDeferredPhi[i]) {
            mUpdatedProcessedDeferredPhi[i]->addIncoming(totalCount, exitBlock);
        }
    }
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        mUpdatedProducedPhi[i]->addIncoming(mFinalProducedPhi[i], exitBlock);
    }
}

}
