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
    if (mCheckAssertions) {
        mRethrowException = b->WriteDefaultRethrowBlock();
    }
    mKernel = nullptr;
    mKernelIndex = 0;
    mPipelineEntryBranch = b->CreateBr(mPipelineLoop);

    b->SetInsertPoint(mPipelineLoop);
    mMadeProgressInLastSegment = b->CreatePHI(b->getInt1Ty(), 2);
    mMadeProgressInLastSegment->addIncoming(b->getTrue(), entryBlock);
    mPipelineProgress = b->getFalse();
    if (isOpenSystem()) {
        assert ("open system was not given an external seg no." && mPipelineKernel->getExternalSegNo());
        PHINode * const segNoPhi = b->CreatePHI(b->getSizeTy(), 2);
        segNoPhi->addIncoming(mPipelineKernel->getExternalSegNo(), entryBlock);
        mSegNo = segNoPhi;
    } else {
        // By using an atomic fetch/add here, we gain the ability to dynamically add or
        // remove threads while still using the segment pipeline parallelism model.
        // This also allows us to execute nested pipelines without requiring the outer
        // pipeline to track the current segno.
        Value * const segNoPtr = b->getScalarFieldPtr(CURRENT_LOGICAL_SEGMENT_NUMBER);
        mSegNo = b->CreateAtomicFetchAndAdd(b->getSize(1), segNoPtr);
    }
    mHalted = b->getFalse();
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(mPipelineKernel->getName() + " +++ pipeline start +++", mSegNo);
    #endif
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
    mKernelAbnormalTermination = b->CreateBasicBlock(prefix + "_abnormalTermination", mPipelineEnd);
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
    loadBufferHandles(b);
    readInitialItemCounts(b);
    readConsumedItemCounts(b);
    incrementNumberOfSegmentsCounter(b);
    recordUnconsumedItemCounts(b);
    Value * const terminated = initiallyTerminated(b);
    mKernelEntry = b->GetInsertBlock();
    b->CreateUnlikelyCondBr(terminated, mKernelExit, mKernelLoopEntry);

    /// -------------------------------------------------------------------------------------
    /// PHI NODE INITIALIZATION
    /// -------------------------------------------------------------------------------------

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
    determineNumOfLinearStrides(b);
    prepareLocalZeroExtendSpace(b);
    Value * const isFinal = calculateItemCounts(b);


    /// -------------------------------------------------------------------------------------
    /// KERNEL CALL
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopCall);
   // checkForLastPartialSegment(b, isFinal);
    writeLookBehindLogic(b);
    writeKernelCall(b);
    writeCopyBackLogic(b);

    #if 0

    auto hexDumpOutput = [&]() {

        for (unsigned i = 0; i < getNumOfStreamOutputs(mKernelIndex); ++i) {

            const Binding & output = getOutputBinding(i);

            Constant * const MAX = b->getSize(3 * 16 + 1);
            Constant * const CAPACITY = b->getSize(3 * 17);
            Constant * const MAX_MINUS_1 = b->getSize(3 * 16);
            Constant * const LINE_MASK = b->getSize(16 - 1);
            Constant * const ZERO = b->getSize(0);
            Constant * const ONE = b->getSize(1);
            Constant * const THREE = b->getSize(3);
            Constant * const TEN = b->getInt8(10);
            Constant * const FIFTEEN = b->getInt8(15);
            Constant * const NUMERIC = b->getInt8('0');
            Constant * const ALPHA = b->getInt8('a' - 10);


            Value * P = mProducedItemCount[i];
            Value * I = mAlreadyProducedPhi[i];

            if (output.hasAttribute(AttrId::Delayed)) {
                Constant * const K = b->getSize(output.findAttribute(AttrId::Delayed).amount());
                Value * term = b->CreateIsNull(mNumOfLinearStrides);
                if (mTerminatedExplicitly) {
                    term = b->CreateOr(term, mTerminatedExplicitly);
                }
                I = b->CreateSaturatingSub(I, K);
                P = b->CreateSelect(term, P, b->CreateSaturatingSub(P, K));
            }

            Value * const length = b->CreateSub(P, I);
            StreamSetBuffer * const buffer = getOutputBuffer(i);

            Value * const prior = buffer->getRawItemPointer(b,  ZERO, b->CreateSub(I, ONE));


            Value * const chars = b->CreateAllocaAtEntryPoint(b->getInt8Ty(), CAPACITY);
            b->CreateStore(b->getInt8('\n'), b->CreateGEP(chars, MAX_MINUS_1));

            BasicBlock * const writeCond = b->CreateBasicBlock();
            BasicBlock * const writeOut = b->CreateBasicBlock();
            BasicBlock * const writeNext = b->CreateBasicBlock();
            BasicBlock * const writeExit = b->CreateBasicBlock();
            BasicBlock * const entryPoint = b->GetInsertBlock();
            b->CreateBr(writeCond);

            b->SetInsertPoint(writeCond);
            PHINode * const index = b->CreatePHI(b->getSizeTy(), 2);
            index->addIncoming(ZERO, entryPoint);
            PHINode * const priorPhi = b->CreatePHI(prior->getType(), 2);
            priorPhi->addIncoming(prior, entryPoint);


            Value * const offset = b->CreateAnd(index, LINE_MASK);
            Value * const offset1 = b->CreateMul(offset, THREE);
            Value * const offset2 = b->CreateAdd(offset1, ONE);
            Value * const offset3 = b->CreateAdd(offset2, ONE);

            Value * const start = buffer->getRawItemPointer(b,  ZERO, b->CreateAdd(I, index));

            DataLayout DL(b->getModule());
            Type * const intPtrTy = DL.getIntPtrType(start->getType());
            Value * const startInt = b->CreatePtrToInt(start, intPtrTy);
            Value * const priorInt = b->CreatePtrToInt(priorPhi, intPtrTy);

            Value * const adjacent = b->CreateICmpEQ(b->CreateSub(startInt, priorInt), ONE);
            Value * const successor = b->CreateICmpUGT(startInt, priorInt);
            Value * const delimiter = b->CreateSelect(successor, b->getInt8(']'), b->getInt8('['));

            Value * const D = b->CreateSelect(adjacent, b->getInt8(' '), delimiter);
            b->CreateStore(D, b->CreateGEP(chars, offset1));

            Value * const val = b->CreateLoad(start);

            Value * const val0 = b->CreateAnd(val, FIFTEEN);
            Value * const val1 = b->CreateLShr(val, 4);

            Value * const A0 = b->CreateAdd(val0, NUMERIC);
            Value * const A1 = b->CreateAdd(val0, ALPHA);
            Value * const A = b->CreateSelect(b->CreateICmpULT(val0, TEN), A0, A1);
            b->CreateStore(A, b->CreateGEP(chars, offset3));

            Value * const B0 = b->CreateAdd(val1, NUMERIC);
            Value * const B1 = b->CreateAdd(val1, ALPHA);
            Value * const B = b->CreateSelect(b->CreateICmpULT(val1, TEN), B0, B1);
            b->CreateStore(B, b->CreateGEP(chars, offset2));

            Value * const nextIndex = b->CreateAdd(index, ONE);
            Value * const insertLineBreak = b->CreateAnd(b->CreateICmpNE(index, ZERO), b->CreateICmpEQ(offset, ZERO));
            Value * const notLastChar = b->CreateICmpNE(nextIndex, length);
            b->CreateCondBr(b->CreateAnd(insertLineBreak, notLastChar), writeOut, writeNext);

            b->SetInsertPoint(writeOut);
            b->CreateWriteCall(b->getInt32(STDERR_FILENO), chars, MAX);
            b->CreateBr(writeNext);

            b->SetInsertPoint(writeNext);
            index->addIncoming(nextIndex, writeNext);
            priorPhi->addIncoming(start, writeNext);
            b->CreateCondBr(notLastChar, writeCond, writeExit);

            b->SetInsertPoint(writeExit);
            Value * const endLine1 = b->CreateAdd(offset3, ONE);
            Value * const endLine2 = b->CreateAdd(endLine1, ONE);
            b->CreateStore(b->getInt8('\n'), b->CreateGEP(chars, endLine1));
            b->CreateWriteCall(b->getInt32(STDERR_FILENO), chars, endLine2);
        }

    };

    if (mKernelIndex >= 52 && mKernelIndex <= 54) {

        b->CreateWriteCall(b->getInt32(STDERR_FILENO), b->GetString("PRE REFLECTION\n"), b->getSize(15));

        hexDumpOutput();
    }

    #endif


    writeLookBehindReflectionLogic(b);

    #if 0

    if (mKernelIndex >= 52 && mKernelIndex <= 54) {

        b->CreateWriteCall(b->getInt32(STDERR_FILENO), b->GetString("POST REFLECTION\n"), b->getSize(16));

        hexDumpOutput();
    }

    #endif

    // If the kernel explicitly terminates, it must set its processed/produced item counts.
    // Otherwise, the pipeline will update any countable rates, even upon termination.
    Value * const aborted = b->CreateIsNotNull(mTerminatedExplicitly);
    b->CreateUnlikelyCondBr(aborted, mKernelAbnormalTermination, mKernelTerminationCheck);

    /// -------------------------------------------------------------------------------------
    /// KERNEL NORMAL TERMINATION CHECK
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelTerminationCheck);
    normalCompletionCheck(b, isFinal);

    /// -------------------------------------------------------------------------------------
    /// KERNEL ABNORMAL TERMINATION
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelAbnormalTermination);
    loadItemCountsOfCountableRateStreams(b);
    signalAbnormalTermination(b);
    b->CreateBr(mKernelTerminated);

    /// -------------------------------------------------------------------------------------
    /// KERNEL TERMINATED
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelTerminated);
    clearUnwrittenOutputData(b);
    setTerminated(b, mTerminatedSignalPhi);
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
    writeLookAheadLogic(b);
    computeFullyProducedItemCounts(b);
    if (LLVM_UNLIKELY(mCheckAssertions && mBoundedKernel)) {

        const BufferNode & bn = mBufferGraph[mKernelIndex];
        SmallVector<char, 128> tmp;
        raw_svector_ostream msg(tmp);
        const auto lb = floor(bn.Lower);
        const auto ub = ceiling(bn.Upper);
        msg << " (" << mKernelIndex << "): processed %d strides"
               " but expected [" << lb << "," << ub << "]";

        Value * const notTooFew = b->CreateICmpUGE(mTotalNumOfStrides, b->getSize(lb));
        Value * const notTooMany = b->CreateICmpULE(mTotalNumOfStrides, b->getSize(ub));
        Value * const withinRange = b->CreateAnd(notTooFew, notTooMany);
        Value * const terminated = b->CreateIsNotNull(mTerminatedPhi);
        Value * const valid = b->CreateOr(terminated, withinRange);
        b->CreateAssert(valid, msg.str(), mTotalNumOfStrides);
    }
    mKernelLoopExitPhiCatch->moveAfter(b->GetInsertBlock());
    b->CreateBr(mKernelLoopExitPhiCatch);
    b->SetInsertPoint(mKernelLoopExitPhiCatch);
    b->CreateBr(mKernelExit);

    /// -------------------------------------------------------------------------------------
    /// KERNEL EXIT (CONTINUED)
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelExit);
    PHINode * const totalNumOfStrides = b->CreatePHI(b->getSizeTy(), 2);
    totalNumOfStrides->addIncoming(b->getSize(0), mKernelEntry);
    totalNumOfStrides->addIncoming(mTotalNumOfStrides, mKernelLoopExitPhiCatch);
    mKernelExit->moveAfter(mKernelLoopExitPhiCatch);
    updateTerminationSignal(mTerminatedAtExitPhi);
    writeFinalConsumedItemCounts(b);
    readFinalProducedItemCounts(b);
    mHalted = mHaltedPhi;
    mPipelineProgress = mNextPipelineProgress;
    assert (mKernel == getKernel(mKernelIndex) && b->getKernel() == mKernel);
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt("* " + prefix + ".madeProgress", mPipelineProgress);
    #endif
    recordStridesPerSegment(b, totalNumOfStrides);
    recordProducedItemCountDeltas(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief normalCompletionCheck
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::normalCompletionCheck(BuilderRef b, Value * const isFinal) {

    BasicBlock * const entryBlock = b->GetInsertBlock();

    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);

    Constant * const unterminated = getTerminationSignal(b, TerminationSignal::None);

    if (mBoundedKernel) {

        const auto prefix = makeKernelName(mKernelIndex);
        BasicBlock * const ioBoundsCheck = b->CreateBasicBlock(prefix + "_IOBoundsCheck", mKernelAbnormalTermination);

        for (unsigned i = 0; i < numOfInputs; ++i) {
            assert (mProcessedItemCount[i]);
            mAlreadyProcessedPhi[i]->addIncoming(mProcessedItemCount[i], ioBoundsCheck);
            if (mAlreadyProcessedDeferredPhi[i]) {
                assert (mProcessedDeferredItemCount[i]);
                mAlreadyProcessedDeferredPhi[i]->addIncoming(mProcessedDeferredItemCount[i], ioBoundsCheck);
            }
            mFinalProcessedPhi[i]->addIncoming(mProcessedItemCount[i], entryBlock);

            mUpdatedProcessedPhi[i]->addIncoming(mProcessedItemCount[i], ioBoundsCheck);
            if (mUpdatedProcessedDeferredPhi[i]) {
                assert (mProcessedDeferredItemCount[i]);
                mUpdatedProcessedDeferredPhi[i]->addIncoming(mProcessedDeferredItemCount[i], ioBoundsCheck);
            }
        }
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            assert (mProducedItemCount[i]);
            mAlreadyProducedPhi[i]->addIncoming(mProducedItemCount[i], ioBoundsCheck);
            mFinalProducedPhi[i]->addIncoming(mProducedItemCount[i], entryBlock);
            mUpdatedProducedPhi[i]->addIncoming(mProducedItemCount[i], ioBoundsCheck);
        }

        Constant * const completed = getTerminationSignal(b, TerminationSignal::Completed);
        mTerminatedSignalPhi->addIncoming(completed, entryBlock);

        mAlreadyProgressedPhi->addIncoming(b->getTrue(), ioBoundsCheck);
        mCurrentNumOfStrides->addIncoming(mUpdatedNumOfStrides, ioBoundsCheck);
        mExecutedAtLeastOncePhi->addIncoming(b->getTrue(), ioBoundsCheck);
        b->CreateUnlikelyCondBr(isFinal, mKernelTerminated, ioBoundsCheck);

        b->SetInsertPoint(ioBoundsCheck);

        // bound the number of strides by the maximum expected
        const BufferNode & bn = mBufferGraph[mKernelIndex];
        Constant * const maxStrides = b->getSize(ceiling(bn.Upper));
        Value * const done = b->CreateICmpEQ(mUpdatedNumOfStrides, maxStrides);
        mTerminatedPhi->addIncoming(unterminated, ioBoundsCheck);
        mHasProgressedPhi->addIncoming(b->getTrue(), ioBoundsCheck);
        mHaltingPhi->addIncoming(mHalted, ioBoundsCheck);
        mTotalNumOfStrides->addIncoming(mUpdatedNumOfStrides, ioBoundsCheck);
        b->CreateCondBr(done, mKernelLoopExit, mKernelLoopEntry);


    } else { // just exit the loop

        for (unsigned i = 0; i < numOfInputs; ++i) {
            mUpdatedProcessedPhi[i]->addIncoming(mProcessedItemCount[i], entryBlock);
            if (mUpdatedProcessedDeferredPhi[i]) {
                mUpdatedProcessedDeferredPhi[i]->addIncoming(mProcessedDeferredItemCount[i], entryBlock);
            }
        }
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            mUpdatedProducedPhi[i]->addIncoming(mProducedItemCount[i], entryBlock);
            if (mUpdatedProducedDeferredPhi[i]) {
                mUpdatedProducedDeferredPhi[i]->addIncoming(mProducedDeferredItemCount[i], entryBlock);
            }
        }
        mTerminatedPhi->addIncoming(unterminated, entryBlock);
        mHasProgressedPhi->addIncoming(b->getTrue(), entryBlock);
        mHaltingPhi->addIncoming(mHalted, entryBlock);
        mTotalNumOfStrides->addIncoming(mUpdatedNumOfStrides, entryBlock);
        b->CreateBr(mKernelLoopExit);
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

    b->setKernel(mPipelineKernel);
    Value * const terminated = hasPipelineTerminated(b);
    Value * const done = b->CreateOr(mHalted, b->CreateIsNotNull(terminated));
    Value * const progressedOrFinished = b->CreateOr(mPipelineProgress, done);
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(mPipelineKernel->getName() + "+++ pipeline end +++", mSegNo);
    #endif

    if (LLVM_UNLIKELY(mCheckAssertions)) {
        b->CreateAssert(b->CreateOr(mMadeProgressInLastSegment, progressedOrFinished),
            "Dead lock detected: pipeline could not progress after two iterations");
    }

    BasicBlock * const exitBlock = b->GetInsertBlock();
    mMadeProgressInLastSegment->addIncoming(progressedOrFinished, exitBlock);
    if (isOpenSystem()) {
        Constant * const numOfThreads = b->getSize(mNumOfThreads);
        Value * const nextSegNo = b->CreateAdd(mSegNo, numOfThreads);
        cast<PHINode>(mSegNo)->addIncoming(nextSegNo, exitBlock);
    }
    b->CreateUnlikelyCondBr(done, mPipelineEnd, mPipelineLoop);

    b->SetInsertPoint(mPipelineEnd);
    b->setKernel(mPipelineKernel);
    writePipelineIOItemCounts(b);
    if (mPipelineTerminated) {
        b->CreateStore(terminated, mPipelineTerminated);
    }
    // free any truncated input buffers
    for (Value * const bufferPtr : mTruncatedInputBuffer) {
        b->CreateFree(b->CreateLoad(bufferPtr));
    }
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
            const auto prefix = makeBufferName(kernelIndex, rd.Port);
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
            const auto prefix = makeBufferName(kernelIndex, rd.Port);
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
        const auto prefix = makeBufferName(PipelineInput, rd.Port);
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
        const auto prefix = makeBufferName(producer, internal.Port);
        Value * const produced = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
        b->CreateStore(produced, ptr);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelLoopEntryPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelLoopEntryPhis(BuilderRef b) {
    IntegerType * const sizeTy = b->getSizeTy();
    IntegerType * const boolTy = b->getInt1Ty();
    b->SetInsertPoint(mKernelLoopEntry);
    BasicBlock * const entryBlock = mKernelEntry;
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Input, i});
        mAlreadyProcessedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProcessed");
        mAlreadyProcessedPhi[i]->addIncoming(mInitiallyProcessedItemCount[i], entryBlock);
        if (mInitiallyProcessedDeferredItemCount[i]) {
            mAlreadyProcessedDeferredPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProcessedDeferred");
            mAlreadyProcessedDeferredPhi[i]->addIncoming(mInitiallyProcessedDeferredItemCount[i], entryBlock);
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, i});
        mAlreadyProducedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_alreadyProduced");
        mAlreadyProducedPhi[i]->addIncoming(mInitiallyProducedItemCount[i], entryBlock);
    }
    // Since we may loop and call the kernel again, we want to mark that we've progressed
    // if we execute any kernel even if we could not complete a full segment.
    const auto prefix = makeKernelName(mKernelIndex);
    mAlreadyProgressedPhi = b->CreatePHI(boolTy, 2, prefix + "_madeProgress");
    mAlreadyProgressedPhi->addIncoming(mPipelineProgress, entryBlock);
    mExecutedAtLeastOncePhi = b->CreatePHI(boolTy, 2, prefix + "_executedAtLeastOnce");
    mExecutedAtLeastOncePhi->addIncoming(b->getFalse(), entryBlock);
    mCurrentNumOfStrides = b->CreatePHI(sizeTy, 2, prefix + "_currentNumOfStrides");
    mCurrentNumOfStrides->addIncoming(b->getSize(0), entryBlock);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelCallPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelCallPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelLoopCall);
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    Type * const sizeTy = b->getSizeTy();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Input, i});
        mLinearInputItemsPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_linearlyAccessible");
        Type * const bufferTy = getInputBuffer(i)->getPointerType();
        mInputEpochPhi[i] = b->CreatePHI(bufferTy, 2, prefix + "_baseAddress");
    }

    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, i});
        mLinearOutputItemsPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_linearlyWritable");
    }
    mFixedRateFactorPhi = nullptr;
    if (LLVM_LIKELY(hasFixedRateLCM())) {
        const auto prefix = makeKernelName(mKernelIndex);
        mFixedRateFactorPhi = b->CreatePHI(sizeTy, 2, prefix + "_fixedRateFactor");
    }
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
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Input, i});
        mFinalProcessedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_finalProcessed");
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, i});
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
    mTerminatedPhi = b->CreatePHI(sizeTy, 2, prefix + "_terminatedLE");
    mHasProgressedPhi = b->CreatePHI(boolTy, 2, prefix + "_anyProgressLE");
    mHaltingPhi = b->CreatePHI(boolTy, 2, prefix + "_haltingLE");
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Input, i});
        mUpdatedProcessedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessed");
        if (mAlreadyProcessedDeferredPhi[i]) {
            mUpdatedProcessedDeferredPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_updatedProcessedDeferred");
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, i});
        mUpdatedProducedPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_updatedProduced");
    }
    mTotalNumOfStrides = b->CreatePHI(sizeTy, 2, prefix + "_totalNumOfStrides");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelExitPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelExitPhis(BuilderRef b) {
    b->SetInsertPoint(mKernelExit);
    const auto prefix = makeKernelName(mKernelIndex);
    IntegerType * const sizeTy = b->getSizeTy();
    mTerminatedAtExitPhi = b->CreatePHI(sizeTy, 2, prefix + "_terminatedKE");
    mTerminatedAtExitPhi->addIncoming(mTerminatedInitially, mKernelEntry);
    mTerminatedAtExitPhi->addIncoming(mTerminatedPhi, mKernelLoopExitPhiCatch);

    IntegerType * const boolTy = b->getInt1Ty();
    mHaltedPhi = b->CreatePHI(boolTy, 2, prefix + "_haltedKE");
    mHaltedPhi->addIncoming(mHalted, mKernelEntry);
    mHaltedPhi->addIncoming(mHaltingPhi, mKernelLoopExitPhiCatch);

    PHINode * const pipelineProgress = b->CreatePHI(boolTy, 2, prefix + "_pipelineProgressKE");
    pipelineProgress->addIncoming(mPipelineProgress, mKernelEntry);
    pipelineProgress->addIncoming(mHasProgressedPhi, mKernelLoopExitPhiCatch);
    mNextPipelineProgress = pipelineProgress;

    createConsumedPhiNodes(b);
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, i});
        PHINode * const fullyProduced = b->CreatePHI(sizeTy, 2, prefix + "_fullyProduced");
        fullyProduced->addIncoming(mInitiallyProducedItemCount[i], mKernelEntry);
        mFullyProducedItemCount[i] = fullyProduced;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updatePhiCountAfterTermination
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::updatePhisAfterTermination(BuilderRef b) {
    BasicBlock * const exitBlock = b->GetInsertBlock();
    mTerminatedPhi->addIncoming(mTerminatedSignalPhi, exitBlock);
    mHasProgressedPhi->addIncoming(b->getTrue(), exitBlock);
    mHaltingPhi->addIncoming(mHalted, exitBlock);
    mTotalNumOfStrides->addIncoming(mCurrentNumOfStrides, exitBlock);
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
