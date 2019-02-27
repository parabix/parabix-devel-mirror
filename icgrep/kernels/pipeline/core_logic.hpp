#include "pipeline_compiler.hpp"

// TODO: if we have multiple copies of the same type of kernel executing sequentially, we could avoid
// generating an "execution call" for each and instead pass in different handles/item counts. This
// could improve I-Cache utilization.

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief start
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::start(BuilderRef b) {

    // Create the basic blocks for the loop.
    BasicBlock * const entryBlock = b->GetInsertBlock();
    mPipelineLoop = b->CreateBasicBlock("pipelineLoop");
    mPipelineEnd = b->CreateBasicBlock("pipelineEnd");

    mKernel = nullptr;
    mKernelIndex = 0;
    mPipelineEntryBranch = b->CreateBr(mPipelineLoop);

    b->SetInsertPoint(mPipelineLoop);
    mMadeProgressInLastSegment = b->CreatePHI(b->getInt1Ty(), 2);
    mMadeProgressInLastSegment->addIncoming(b->getTrue(), entryBlock);
    mPipelineProgress = b->getFalse();
    // By using an atomic fetch/add here, we gain the ability to dynamically add or
    // remove threads while still using the segment pipeline parallelism model.
    // This also allows us to execute nested pipelines without requiring the outer
    // pipeline to track the current segno.
    Value * const segNoPtr = b->getScalarFieldPtr(INITIAL_LOGICAL_SEGMENT_NUMBER);
    mSegNo = b->CreateAtomicFetchAndAdd(b->getSize(1), segNoPtr);
    loadTerminationSignals(b);
    mHalted = b->getFalse();
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
    determineEvaluationOrderOfKernelIO();

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

    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt("+++ " + prefix + "_segNo", mSegNo);
    #endif
    b->setKernel(mPipelineKernel);
    b->setKernel(mKernel);
    loadBufferHandles(b);
    readInitialItemCounts(b);
    readConsumedItemCounts(b);
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
    checkForSufficientInputDataAndOutputSpace(b);
    determineNumOfLinearStrides(b);

    /// -------------------------------------------------------------------------------------
    /// KERNEL CALCULATE ITEM COUNTS
    /// -------------------------------------------------------------------------------------

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
    prepareLocalZeroExtendSpace(b);
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
    clearUnwrittenOutputData(b);
    setTerminated(b);
    updatePhisAfterTermination(b);
    b->CreateBr(mKernelLoopExit);

    /// -------------------------------------------------------------------------------------
    /// KERNEL LOOP EXIT
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopExit);
    updateTerminationSignal(mTerminatedPhi);
    writeUpdatedItemCounts(b, false);
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
    updateTerminationSignal(mTerminatedAtExitPhi);
    writeFinalConsumedItemCounts(b);
    updatePopCountReferenceCounts(b);
    readFinalProducedItemCounts(b);
    updateOptionalCycleCounter(b);
    mHalted = mHaltedPhi;
    assert (mKernel == getKernel(mKernelIndex) && b->getKernel() == mKernel);
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt("* " + prefix + ".madeProgress", mPipelineProgress);
    #endif
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief normalTerminationCheck
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::normalTerminationCheck(BuilderRef b, Value * const isFinal) {
    BasicBlock * const entryBlock = b->GetInsertBlock();
    if (isFinal) {
        const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
        for (unsigned i = 0; i < numOfInputs; ++i) {
            mAlreadyProcessedPhi[i]->addIncoming(mProcessedItemCount[i], entryBlock);
            if (mAlreadyProcessedDeferredPhi[i]) {
                mAlreadyProcessedDeferredPhi[i]->addIncoming(mProcessedDeferredItemCount[i], entryBlock);
            }
            mFinalProcessedPhi[i]->addIncoming(mProcessedItemCount[i], entryBlock);
        }
        const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            mAlreadyProducedPhi[i]->addIncoming(mProducedItemCount[i], entryBlock);
            mFinalProducedPhi[i]->addIncoming(mProducedItemCount[i], entryBlock);
        }
        if (mAlreadyProgressedPhi) {
            mAlreadyProgressedPhi->addIncoming(b->getTrue(), entryBlock);
        }
        b->CreateUnlikelyCondBr(isFinal, mKernelTerminated, mKernelLoopEntry);
    } else { // just exit the loop
        const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
        for (unsigned i = 0; i < numOfInputs; ++i) {
            mUpdatedProcessedPhi[i]->addIncoming(mProcessedItemCount[i], entryBlock);
            if (mUpdatedProcessedDeferredPhi[i]) {
                mUpdatedProcessedDeferredPhi[i]->addIncoming(mProcessedDeferredItemCount[i], entryBlock);
            }
        }
        const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            mUpdatedProducedPhi[i]->addIncoming(mProducedItemCount[i], entryBlock);
            if (mUpdatedProducedDeferredPhi[i]) {
                mUpdatedProducedDeferredPhi[i]->addIncoming(mProducedDeferredItemCount[i], entryBlock);
            }
        }
        mTerminatedPhi->addIncoming(mTerminatedInitially, entryBlock);
        mHasProgressedPhi->addIncoming(b->getTrue(), entryBlock);
        mHaltingPhi->addIncoming(mHalted, entryBlock);
        b->CreateBr(mKernelLoopExit);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief end
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::end(BuilderRef b) {

    // A pipeline will end for one or two reasons:

    // 1) Process has *halted* due to insufficient pipeline I/O.

    // 2) All pipeline sinks have terminated (i.e., any kernel that writes
    // to a pipeline output, is marked as having a side-effect, or produces
    // an input for some call in which no dependent kernels is a pipeline
    // sink).

    b->setKernel(mPipelineKernel);

    storeTerminationSignals(b);
    Value * const terminated = pipelineTerminated(b);
    Value * const done = b->CreateOr(mHalted, terminated);
    Value * const progressedOrFinished = b->CreateOr(mPipelineProgress, done);
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt("+++ pipeline end +++", mSegNo);
    #endif

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        b->CreateAssert(b->CreateOr(mMadeProgressInLastSegment, progressedOrFinished),
            "Dead lock detected: pipeline could not progress after two iterations");
    }

    //Value * const nextSegNo = b->CreateAdd(mSegNo, b->getSize(1));
    BasicBlock * const exitBlock = b->GetInsertBlock();
    mMadeProgressInLastSegment->addIncoming(progressedOrFinished, exitBlock);
    //mSegNo->addIncoming(nextSegNo, exitBlock);
    b->CreateUnlikelyCondBr(done, mPipelineEnd, mPipelineLoop);

    b->SetInsertPoint(mPipelineEnd);
    b->setKernel(mPipelineKernel);
    writePipelineIOItemCounts(b);
    if (mPipelineTerminated) {
        b->CreateStore(terminated, mPipelineTerminated);
    }
    return mSegNo;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief pipelineTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::pipelineTerminated(BuilderRef b) const {
    Value * terminated = b->getTrue();
    // check whether every sink has terminated
    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mTerminationGraph))) {
        const auto kernel = source(e, mTerminationGraph);
        terminated = b->CreateAnd(terminated, hasKernelTerminated(b, kernel));
    }
    return terminated;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readPipelineIOItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::readPipelineIOItemCounts(BuilderRef b) {

    // TODO: this needs to be considered more: if we have multiple consumers of a pipeline input and
    // they process the input data at differing rates, how do we ensure that we always resume processing
    // at the correct position? We can store the actual item counts / delta of the consumed count
    // internally but this would be problematic for optimization branches as we may have processed data
    // using the alternate path and any internally stored counts/deltas are irrelevant.

    // Would a simple "reset" be enough?


    const auto numOfBuffers = num_vertices(mBufferGraph) - PipelineOutput;

    mLocallyAvailableItems.resize(numOfBuffers, nullptr);
    mPriorConsumedItemCount.resize(numOfBuffers, nullptr);

    for (const auto e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {

        const auto buffer = target(e, mBufferGraph);
        const auto inputPort = mBufferGraph[e].inputPort();
        Value * const available = mPipelineKernel->getAvailableInputItems(inputPort);
        mLocallyAvailableItems[getBufferIndex(buffer)] = available;
        mConsumerGraph[buffer].Consumed = available;

        Value * const inPtr = mPipelineKernel->getProcessedInputItemsPtr(inputPort);
        Value * const processed = b->CreateLoad(inPtr);

        for (const auto e : make_iterator_range(out_edges(buffer, mBufferGraph))) {
            const BufferRateData & rd = mBufferGraph[e];
            const auto kernelIndex = target(e, mBufferGraph);
            const auto prefix = makeBufferName(kernelIndex, rd.Binding);
            Value * const ptr = b->getScalarFieldPtr(prefix + ITEM_COUNT_SUFFIX);
            b->CreateStore(processed, ptr);
        }
    }

    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        const auto buffer = source(e, mBufferGraph);
        const auto outputPort = mBufferGraph[e].outputPort();

        Value * outPtr = mPipelineKernel->getProducedOutputItemsPtr(outputPort);
        Value * const produced = b->CreateLoad(outPtr);

        for (const auto e : make_iterator_range(in_edges(buffer, mBufferGraph))) {
            const BufferRateData & rd = mBufferGraph[e];
            const auto kernelIndex = source(e, mBufferGraph);
            const auto prefix = makeBufferName(kernelIndex, rd.Binding);
            Value * const ptr = b->getScalarFieldPtr(prefix + ITEM_COUNT_SUFFIX);
            b->CreateStore(produced, ptr);
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writePipelineIOItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writePipelineIOItemCounts(BuilderRef b) {

    for (const auto e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
        const BufferRateData & rd = mBufferGraph[e];
        Value * const ptr = mPipelineKernel->getProcessedInputItemsPtr(rd.inputPort());
        const auto prefix = makeBufferName(PipelineInput, rd.Binding);
        Value * const consumed = b->getScalarField(prefix + CONSUMED_ITEM_COUNT_SUFFIX);
        b->CreateStore(consumed, ptr);
    }

    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        const BufferRateData & external = mBufferGraph[e];
        const auto buffer = source(e, mBufferGraph);
        const auto pe = in_edge(buffer, mBufferGraph);
        const BufferRateData & internal = mBufferGraph[pe];
        const auto producer = source(pe, mBufferGraph);
        Value * const ptr = mPipelineKernel->getProducedOutputItemsPtr(external.outputPort());
        const auto prefix = makeBufferName(producer, internal.Binding);
        Value * const produced = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
        b->CreateStore(produced, ptr);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelLoopEntryPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelLoopEntryPhis(BuilderRef b) {
    Type * const sizeTy = b->getSizeTy();
    b->SetInsertPoint(mKernelLoopEntry);

    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = getInputBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, input);
        mAlreadyProcessedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProcessed");
        mAlreadyProcessedPhi[i]->addIncoming(mInitiallyProcessedItemCount[i], mKernelEntry);
        if (mInitiallyProcessedDeferredItemCount[i]) {
            mAlreadyProcessedDeferredPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProcessedDeferred");
            mAlreadyProcessedDeferredPhi[i]->addIncoming(mInitiallyProcessedDeferredItemCount[i], mKernelEntry);
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = getOutputBinding(i);
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
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    Type * const sizeTy = b->getSizeTy();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = getInputBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, input);
        mLinearInputItemsPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_linearlyAccessible");
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = getOutputBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, output);
        mLinearOutputItemsPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_linearlyWritable");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelTerminatedPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelTerminatedPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelTerminated);
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    Type * const sizeTy = b->getSizeTy();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = getInputBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, input);
        mFinalProcessedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_finalProcessed");
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = getOutputBinding(i);
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
    IntegerType * const boolTy = b->getInt1Ty();
    mTerminatedPhi = b->CreatePHI(sizeTy, 2, prefix + "_terminated");
    mHasProgressedPhi = b->CreatePHI(boolTy, 2, prefix + "_anyProgress");
    mHaltingPhi = b->CreatePHI(boolTy, 2, prefix + "_halting");
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = getInputBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, input);
        mUpdatedProcessedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessed");
        if (mAlreadyProcessedDeferredPhi[i]) {
            mUpdatedProcessedDeferredPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessedDeferred");
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = getOutputBinding(i);
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
    mTerminatedAtExitPhi = b->CreatePHI(sizeTy, 2, prefix + "_terminated");
    mTerminatedAtExitPhi->addIncoming(mTerminatedInitially, mKernelEntry);
    mTerminatedAtExitPhi->addIncoming(mTerminatedPhi, mKernelLoopExitPhiCatch);

    IntegerType * const boolTy = b->getInt1Ty();
    mHaltedPhi = b->CreatePHI(boolTy, 2, prefix + "_halted");
    mHaltedPhi->addIncoming(mHalted, mKernelEntry);
    mHaltedPhi->addIncoming(mHaltingPhi, mKernelLoopExitPhiCatch);

    PHINode * const pipelineProgress = b->CreatePHI(boolTy, 2, prefix + "_pipelineProgress");
    pipelineProgress->addIncoming(mPipelineProgress, mKernelEntry);
    pipelineProgress->addIncoming(mHasProgressedPhi, mKernelLoopExitPhiCatch);
    mPipelineProgress = pipelineProgress;

    createConsumedPhiNodes(b);
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = getOutputBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, output);
        PHINode * const fullyProduced = b->CreatePHI(sizeTy, 2, prefix + "_fullyProduced");
        fullyProduced->addIncoming(mInitiallyProducedItemCount[i], mKernelEntry);
        mFullyProducedItemCount[i] = fullyProduced;
    }
    createPopCountReferenceCounts(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updatePhiCountAfterTermination
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::updatePhisAfterTermination(BuilderRef b) {
    BasicBlock * const exitBlock = b->GetInsertBlock();
    mTerminatedPhi->addIncoming(getTerminationSignal(b, mKernelIndex), exitBlock);
    mHasProgressedPhi->addIncoming(b->getTrue(), exitBlock);
    mHaltingPhi->addIncoming(mHalted, exitBlock);
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        Value * const totalCount = getLocallyAvailableItemCount(b, i);
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

}
