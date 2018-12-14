#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addInternalKernelProperties(BuilderRef b) {
    initializePopCounts();
    const auto numOfKernels = mPipeline.size();
    b->setKernel(mPipelineKernel);
    for (unsigned i = 0; i < numOfKernels; ++i) {
        addBufferHandlesToPipelineKernel(b, i);
        addPopCountScalarsToPipelineKernel(b, i);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateInitializeMethod(BuilderRef b) {
    const auto numOfKernels = mPipeline.size();
    for (unsigned i = 0; i < numOfKernels; ++i) {
        mPipeline[i]->addKernelDeclarations(b);
    }
    for (unsigned i = 0; i < numOfKernels; ++i) {
        Kernel * const kernel = mPipeline[i];
        if (!kernel->hasFamilyName()) {
            Value * const handle = kernel->createInstance(b);
            b->setScalarField(makeKernelName(i), handle);
        }
    }
    constructBuffers(b);
    std::vector<Value *> args;
    for (unsigned i = 0; i < numOfKernels; ++i) {
        setActiveKernel(b, i);
        args.resize(in_degree(i, mScalarDependencyGraph) + 1);
        #ifndef NDEBUG
        std::fill(args.begin(), args.end(), nullptr);
        #endif
        args[0] = mKernel->getHandle();
        b->setKernel(mPipelineKernel);
        for (const auto ce : make_iterator_range(in_edges(i, mScalarDependencyGraph))) {
            const auto j = mScalarDependencyGraph[ce] + 1;
            const auto pe = in_edge(source(ce, mScalarDependencyGraph), mScalarDependencyGraph);
            const auto k = mScalarDependencyGraph[pe];
            const Binding & input = mPipelineKernel->getInputScalarBinding(k);
            assert (args[j] == nullptr);
            args[j] = b->getScalarField(input.getName());
        }
        b->setKernel(mKernel);
        b->CreateCall(getInitializationFunction(b), args);
    }
}

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
    mSegNo = b->CreatePHI(b->getSizeTy(), 2, "segNo");
    mSegNo->addIncoming(initialSegNo, entryBlock);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        mDeadLockCounter = b->CreatePHI(b->getSizeTy(), 2, "deadLockCounter");
        mDeadLockCounter->addIncoming(b->getSize(0), entryBlock);
        mPipelineProgress = b->getFalse();
    }
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
    loadBufferHandles(b);

    mKernelEntry = b->GetInsertBlock();

    const auto kernelName = makeKernelName(mKernelIndex);
    BasicBlock * const checkProducers = b->CreateBasicBlock(kernelName + "_checkProducers", mPipelineEnd);
    mKernelLoopEntry = b->CreateBasicBlock(kernelName + "_loopEntry", mPipelineEnd);
    mKernelLoopCall = b->CreateBasicBlock(kernelName + "_executeKernel", mPipelineEnd);
    mKernelLoopExit = b->CreateBasicBlock(kernelName + "_loopExit", mPipelineEnd);
    mKernelExit = b->CreateBasicBlock(kernelName + "_kernelExit", mPipelineEnd);
    // The phi catch simplifies compilation logic by "forward declaring" the loop exit point.
    // Subsequent optimization phases will collapse it into the correct exit block.
    mKernelLoopExitPhiCatch = b->CreateBasicBlock(kernelName + "_kernelExitPhiCatch", mPipelineEnd);

    /// -------------------------------------------------------------------------------------
    /// KERNEL ENTRY
    /// -------------------------------------------------------------------------------------
    Value * const term = b->getTerminationSignal();
    #ifdef PRINT_DEBUG_MESSAGES
    if (1) {
    Constant * const MAX_INT = ConstantInt::getAllOnesValue(mSegNo->getType());
    Value * const round = b->CreateSelect(term, MAX_INT, mSegNo);
    b->CallPrintInt("--- " + kernelName + "_start ---", round);
    }
    #endif
    b->CreateUnlikelyCondBr(term, mKernelExit, checkProducers);

    /// -------------------------------------------------------------------------------------
    /// KERNEL CHECK PRODUCERS
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(checkProducers);
    readInitialProducedItemCounts(b);
    b->CreateBr(mKernelLoopEntry);

    // Set up some PHI nodes "early" to simplify accumulating their incoming values.

    /// -------------------------------------------------------------------------------------
    /// KERNEL LOOP ENTRY
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopEntry);
    // Since we may loop and call the kernel again, we want to mark that we've progressed
    // if we execute any kernel even if we could not complete a full segment.
    if (mPipelineProgress) {
        mAlreadyProgressedPhi = b->CreatePHI(b->getInt1Ty(), 2, kernelName + "_madeProgress");
        mAlreadyProgressedPhi->addIncoming(mPipelineProgress, checkProducers);
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL CALL
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopCall);
    initializeKernelCallPhis(b);

    /// -------------------------------------------------------------------------------------
    /// KERNEL LOOP EXIT
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopExit);
    mTerminatedPhi = b->CreatePHI(b->getInt1Ty(), 2, kernelName + "_terminated");
    if (mPipelineProgress) {
        mHasProgressedPhi = b->CreatePHI(b->getInt1Ty(), 2, kernelName + "_anyProgress");
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL EXIT
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelExit);
    initializeKernelExitPhis(b);

    /// -------------------------------------------------------------------------------------
    /// KERNEL LOOP ENTRY (CONTINUED)
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopEntry);
    checkForSufficientInputDataAndOutputSpace(b);
    determineNumOfLinearStrides(b);

    Value * isFinal = nullptr;

    ConstantInt * const ZERO = b->getSize(0);

    if (mNumOfLinearStrides) {

        BasicBlock * const enteringNonFinalSegment = b->CreateBasicBlock(kernelName + "_nonFinalSegment", mKernelLoopCall);
        BasicBlock * const enteringFinalStride = b->CreateBasicBlock(kernelName + "_finalStride", mKernelLoopCall);

        isFinal = b->CreateICmpEQ(mNumOfLinearStrides, ZERO);

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
        mNumOfLinearStrides = ZERO;
        b->CreateBr(mKernelLoopCall);
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL CALL (CONTINUED)
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopCall);
    expandOutputBuffers(b);
    writeKernelCall(b);

    BasicBlock * const incrementItemCounts = b->CreateBasicBlock(kernelName + "_incrementItemCounts", mKernelLoopExit);
    BasicBlock * const terminationCheck = b->CreateBasicBlock(kernelName + "_normalTerminationCheck", mKernelLoopExit);
    BasicBlock * const terminated = b->CreateBasicBlock(kernelName + "_terminated", mKernelLoopExit);

    // If the kernel itself terminates, it must set the final processed/produced item counts.
    // Otherwise, the pipeline will update any countable rates, even upon termination.
    b->CreateUnlikelyCondBr(terminatedExplicitly(b), terminated, incrementItemCounts);

    /// -------------------------------------------------------------------------------------
    /// KERNEL INCREMENT ITEM COUNTS
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(incrementItemCounts);
    // TODO: phi out the item counts and set them once at the end.
    incrementItemCountsOfCountableRateStreams(b);
    writeCopyBackLogic(b);
    b->CreateBr(terminationCheck);

    /// -------------------------------------------------------------------------------------
    /// KERNEL NORMAL TERMINATION CHECK
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(terminationCheck);
    if (isFinal) {
        if (mAlreadyProgressedPhi) {
            mAlreadyProgressedPhi->addIncoming(b->getTrue(), terminationCheck);
        }
        b->CreateUnlikelyCondBr(isFinal, terminated, mKernelLoopEntry);
    } else { // just exit the loop
        if (mHasProgressedPhi) {
            mHasProgressedPhi->addIncoming(b->getTrue(), terminationCheck);
        }
        mTerminatedPhi->addIncoming(b->getFalse(), terminationCheck);
        b->CreateBr(mKernelLoopExit);
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL TERMINATED
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(terminated);
    zeroFillPartiallyWrittenOutputStreams(b);
    setTerminated(b);
    BasicBlock * const kernelTerminatedEnd = b->GetInsertBlock();
    mTerminatedPhi->addIncoming(b->getTrue(), kernelTerminatedEnd);
    if (mHasProgressedPhi) {
        mHasProgressedPhi->addIncoming(b->getTrue(), kernelTerminatedEnd);
    }
    b->CreateBr(mKernelLoopExit);

    /// -------------------------------------------------------------------------------------
    /// KERNEL LOOP EXIT (CONTINUED)
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopExit);
    computeFullyProcessedItemCounts(b);
    computeMinimumConsumedItemCounts(b);
    computeMinimumPopCountReferenceCounts(b);
    writeCopyForwardLogic(b);
    writePopCountComputationLogic(b);
    b->CreateBr(mKernelLoopExitPhiCatch);
    b->SetInsertPoint(mKernelLoopExitPhiCatch);
    b->CreateBr(mKernelExit);

    /// -------------------------------------------------------------------------------------
    /// KERNEL EXIT (CONTINUED)
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelExit);
    writeFinalConsumedItemCounts(b);
    updatePopCountReferenceCounts(b);

    // TODO: logically we should only need to read produced item counts in the loop exit; however, that
    // would mean that we'd first need to load the initial produced item counts prior to the loop entry
    // to have access to them here and then PHI them out within the kernel loop

    readFinalProducedItemCounts(b);
    releaseCurrentSegment(b);
    updateOptionalCycleCounter(b);

    assert (mKernel == mPipeline[mKernelIndex] && b->getKernel() == mKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief wait
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::synchronize(BuilderRef b) {

    const auto kernelName = makeKernelName(mKernelIndex);

    BasicBlock * const kernelWait = b->CreateBasicBlock(kernelName + "Wait", mPipelineEnd);
    b->CreateBr(kernelWait);

    b->SetInsertPoint(kernelWait);
    const Kernel * waitingOn = mKernel;
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::SerializeThreads))) {
        waitingOn = mPipeline.back();
    }
    b->setKernel(waitingOn);
    Value * const processedSegmentCount = b->acquireLogicalSegmentNo();
    assert (processedSegmentCount->getType() == mSegNo->getType());
    Value * const ready = b->CreateICmpEQ(mSegNo, processedSegmentCount);

    BasicBlock * const kernelCheck = b->CreateBasicBlock(kernelName + "Check", mPipelineEnd);
    b->CreateCondBr(ready, kernelCheck, kernelWait);

    b->SetInsertPoint(kernelCheck);
    b->setKernel(mKernel);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief next
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::end(BuilderRef b, const unsigned step) {

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        ConstantInt * const ZERO = b->getSize(0);
        ConstantInt * const ONE = b->getSize(1);
        ConstantInt * const TWO = b->getSize(2);
        Value * const plusOne = b->CreateAdd(mDeadLockCounter, ONE);
        Value * const newCount = b->CreateSelect(mPipelineProgress, ZERO, plusOne);
        b->CreateAssert(b->CreateICmpNE(newCount, TWO),
                        "Dead lock detected: pipeline could not progress after two iterations");
        mDeadLockCounter->addIncoming(newCount, b->GetInsertBlock());
    }
    // check whether every sink has terminated
    Value * allTerminated = b->getTrue();
    const auto pipelineOutputVertex = mPipeline.size();
    for (const auto e : make_iterator_range(in_edges(pipelineOutputVertex, mTerminationGraph))) {
        const auto u = source(e, mTerminationGraph);
        assert (mTerminationGraph[u]);
        allTerminated = b->CreateAnd(allTerminated, mTerminationGraph[u]);
    }
    // or if any output stream of this pipeline cannot support a full stride
    Value * notEnoughSpace = b->getFalse();
    for (const auto e : make_iterator_range(in_edges(pipelineOutputVertex, mBufferGraph))) {
        // TODO: not a very elegant way here; revise
        const auto bufferVertex = source(e, mBufferGraph);
        mKernelIndex = parent(bufferVertex, mBufferGraph);
        mKernel = mPipeline[mKernelIndex];
        resetMemoizedFields();
        const auto outputPort = mBufferGraph[e].Port;
        Value * const writable = getWritableOutputItems(b, outputPort);
        // NOTE: this method doesn't check a popcount's ref stream to determine how many
        // items we actually require. Instead it just calculates them as bounded rates.
        // To support a precise bound, we'd need to produce more ref items than the kernel
        // that writes to this output actually consumes. Since this effectively adds a
        // delay equivalent to a LookAhead of a full stride, this doesn't seem useful.
        Value * const strideLength = getMaximumStrideLength(b, Port::Output, outputPort);
        notEnoughSpace = b->CreateOr(b->CreateICmpULT(writable, strideLength), notEnoughSpace);
    }
    Value * const done = b->CreateOr(allTerminated, notEnoughSpace);
    #ifdef PRINT_DEBUG_MESSAGES
    Constant * const ONES = Constant::getAllOnesValue(mSegNo->getType());
    b->CallPrintInt("+++ pipeline end +++", b->CreateSelect(done, ONES, mSegNo));
    #endif

    Value * const nextSegNo = b->CreateAdd(mSegNo, b->getSize(step));
    mSegNo->addIncoming(nextSegNo, b->GetInsertBlock());
    b->CreateUnlikelyCondBr(done, mPipelineEnd, mPipelineLoop);

    b->SetInsertPoint(mPipelineEnd);
    mSegNo = nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateFinalizeMethod(BuilderRef b) {
    printOptionalCycleCounter(b);
    const auto numOfKernels = mPipeline.size();
    mOutputScalars.resize(numOfKernels);
    for (unsigned i = 0; i < numOfKernels; ++i) {
        setActiveKernel(b, i);
        loadBufferHandles(b);
        mOutputScalars[i] = b->CreateCall(getFinalizeFunction(b), mKernel->getHandle());
    }
    releaseBuffers(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeOutputScalars
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeOutputScalars(BuilderRef b, const unsigned u, std::vector<Value *> & args) {
    args.clear();
    const auto n = in_degree(u, mScalarDependencyGraph);
    args.resize(n, nullptr);
    const auto numOfKernels = mPipeline.size();
    for (const auto e : make_iterator_range(in_edges(u, mScalarDependencyGraph))) {
        const auto bufferVertex = source(e, mScalarDependencyGraph);
        if (LLVM_LIKELY(mScalarDependencyGraph[bufferVertex] == nullptr)) {
            const auto producer = in_edge(source(e, mScalarDependencyGraph), mScalarDependencyGraph);
            const auto i = source(producer, mScalarDependencyGraph);
            const auto j = mScalarDependencyGraph[producer];
            Value * val = nullptr;
            if (i == numOfKernels) {
                const Binding & input = mPipelineKernel->getInputScalarBinding(j);
                val = b->getScalarField(input.getName());
            } else { // output scalar of some kernel
                Value * const outputScalars = mOutputScalars[i]; assert (outputScalars);
                if (outputScalars->getType()->isAggregateType()) {
                    val = b->CreateExtractValue(outputScalars, {j});
                } else { assert (j == 0 && "scalar type is not an aggregate");
                    val = outputScalars;
                }
            }
            mScalarDependencyGraph[bufferVertex] = val;
        }
        const auto k = mScalarDependencyGraph[e];
        assert (args[k] == nullptr);
        args[k] = mScalarDependencyGraph[bufferVertex];
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalOutputScalars
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Value *> PipelineCompiler::getFinalOutputScalars(BuilderRef b) {
    const auto numOfKernels = mPipeline.size();
    const auto & calls = mPipelineKernel->getCallBindings();
    const auto numOfCalls = calls.size();
    std::vector<Value *> args;
    b->setKernel(mPipelineKernel);
    for (unsigned k = 0; k < numOfCalls; ++k) {
        writeOutputScalars(b, numOfKernels + k + 1, args);
        Function * const f = cast<Function>(calls[k].Callee);
        auto i = f->arg_begin();
        for (auto j = args.begin(); j != args.end(); ++i, ++j) {
            assert (i != f->arg_end());
            *j = b->CreateZExtOrTrunc(*j, i->getType());
        }
        assert (i == f->arg_end());
        b->CreateCall(f, args);
    }
    writeOutputScalars(b, numOfKernels, args);
    return args;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelCallPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelCallPhis(BuilderRef b) {
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    Type * const sizeTy = b->getSizeTy();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = mKernel->getInputStreamSetBinding(i);
        const auto prefix = makeBufferName(mKernelIndex, input);
        mLinearInputItemsPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_linearlyAccessible");
    }
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (LLVM_LIKELY(getOutputBufferType(i) != BufferType::Managed)) {
            const Binding & output = mKernel->getOutputStreamSetBinding(i);
            const auto prefix = makeBufferName(mKernelIndex, output);
            mLinearOutputItemsPhi[i] = b->CreatePHI(sizeTy, 2, prefix + "_linearlyWritable");
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelExitPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelExitPhis(BuilderRef b) {
    const auto kernelName = makeKernelName(mKernelIndex);
    mTerminatedFlag = b->CreatePHI(b->getInt1Ty(), 2, kernelName + "_terminated");
    mTerminatedFlag->addIncoming(b->getTrue(), mKernelEntry);
    mTerminatedFlag->addIncoming(mTerminatedPhi, mKernelLoopExitPhiCatch);
    mTerminationGraph[mKernelIndex] = mTerminatedFlag;
    if (mPipelineProgress) {
        PHINode * const pipelineProgress = b->CreatePHI(b->getInt1Ty(), 2, "pipelineProgress");
        pipelineProgress->addIncoming(mPipelineProgress, mKernelEntry);
        pipelineProgress->addIncoming(mHasProgressedPhi, mKernelLoopExitPhiCatch);
        mPipelineProgress = pipelineProgress;
    }
    createConsumedPhiNodes(b);
    createPopCountReferenceCounts(b);
}

#warning TODO: move termination variables into pipeline if the kernel cannot signal termination itself.

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief terminatedExplicitly
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::terminatedExplicitly(BuilderRef b) const {
    if (LLVM_UNLIKELY(mKernel->hasAttribute(AttrId::MustExplicitlyTerminate) || mKernel->hasAttribute(AttrId::CanTerminateEarly))) {
        return b->getTerminationSignal();
    } else {
        return b->getFalse();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::setTerminated(BuilderRef b) {
    if (LLVM_UNLIKELY(mKernel->hasAttribute(AttrId::MustExplicitlyTerminate))) {
        return;
    }
    const auto prefix = makeKernelName(mKernelIndex);
    BasicBlock * const terminationIsSet = b->CreateBasicBlock(prefix + "_terminationIsSet", mKernelLoopExit);
    if (mKernel->hasAttribute(AttrId::CanTerminateEarly)) {
        BasicBlock * const setTermination = b->CreateBasicBlock(prefix + "_setTermination", terminationIsSet);
        Value * const terminated = b->getTerminationSignal();
        b->CreateCondBr(terminated, terminationIsSet, setTermination);

        b->SetInsertPoint(setTermination);
    }
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mKernel->getHandle(), CBuilder::Protect::WRITE);
    }
    b->setTerminationSignal();
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mKernel->getHandle(), CBuilder::Protect::READ);
    }
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt("*** " + prefix + "_terminated ***", b->getTrue());
    #endif
    b->CreateBr(terminationIsSet);

    b->SetInsertPoint(terminationIsSet);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief releaseCurrentSegment
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::releaseCurrentSegment(BuilderRef b) {
    Value * const nextSegNo = b->CreateAdd(mSegNo, b->getSize(1));
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mKernel->getHandle(), CBuilder::Protect::WRITE);
    }
    b->releaseLogicalSegmentNo(nextSegNo);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mKernel->getHandle(), CBuilder::Protect::READ);
    }
}

}
