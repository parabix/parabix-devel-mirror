#include "pipeline_compiler.hpp"

const static std::string TERMINATION_SIGNAL = "terminationSignal";

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addPipelineKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addPipelineKernelProperties(BuilderRef b) {
    initializePopCounts();
    const auto numOfKernels = mPipeline.size();
    b->setKernel(mPipelineKernel);
    for (unsigned i = 0; i < numOfKernels; ++i) {
        addInternalKernelProperties(b, i);
        addBufferHandlesToPipelineKernel(b, i);
        addPopCountScalarsToPipelineKernel(b, i);
    }
    b->setKernel(mPipelineKernel);
}

//const static std::string PROCESSED_ITEM_COUNT_SUFFIX = "_processedItemCount";
//const static std::string PRODUCED_ITEM_COUNT_SUFFIX = "_producedItemCount";
// const static std::string NON_DEFERRED_ITEM_COUNT_SUFFIX = "_nonDeferredItemCount";
// const static std::string LOGICAL_SEGMENT_NO_SCALAR = "segmentNo";

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addInternalKernelProperties(BuilderRef b, const unsigned kernelIndex) {
//    Kernel * const kernel = mPipeline[kernelIndex];
    IntegerType * const sizeTy = b->getSizeTy();

    const auto name = makeKernelName(kernelIndex);
    // TODO: prove two termination signals can be fused into a single counter?
    mPipelineKernel->addInternalScalar(b->getInt1Ty(), name + TERMINATION_SIGNAL);
    // TODO: non deferred item count for fixed rates could be calculated from seg no.
    mPipelineKernel->addInternalScalar(sizeTy, name + LOGICAL_SEGMENT_NO_SCALAR);

//    const auto numOfInputs = kernel->getNumOfStreamInputs();
//    for (unsigned i = 0; i < numOfInputs; i++) {
//        const Binding & input = kernel->getInputStreamSetBinding(i);
//        const auto prefix = makeBufferName(kernelIndex, input);
//        mPipelineKernel->addInternalScalar(sizeTy, prefix + PROCESSED_ITEM_COUNT_SUFFIX);
//        if (input.isDeferred()) {
//            mPipelineKernel->addInternalScalar(sizeTy, prefix + NON_DEFERRED_ITEM_COUNT_SUFFIX);
//        }
//    }

//    const auto numOfOutputs = kernel->getNumOfStreamOutputs();
//    for (unsigned i = 0; i < numOfOutputs; i++) {
//        const Binding & output = kernel->getOutputStreamSetBinding(i);
//        const auto prefix = makeBufferName(kernelIndex, output);
//        mPipelineKernel->addInternalScalar(sizeTy, prefix + PRODUCED_ITEM_COUNT_SUFFIX);
//        if (output.isDeferred()) {
//            mPipelineKernel->addInternalScalar(sizeTy, prefix + NON_DEFERRED_ITEM_COUNT_SUFFIX);
//        }
//    }

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
        Value * const terminatedOnInit = b->CreateCall(getInitializationFunction(b), args);
        if (mKernel->canSetTerminateSignal()) {
            setTerminated(b, terminatedOnInit);
        }
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
    Value * const initiallyTerminated = getInitialTerminationSignal(b);
    #ifdef PRINT_DEBUG_MESSAGES
    if (1) {
    Constant * const MAX_INT = ConstantInt::getAllOnesValue(mSegNo->getType());
    Value * const round = b->CreateSelect(initiallyTerminated, MAX_INT, mSegNo);
    b->CallPrintInt("--- " + kernelName + "_start ---", round);
    }
    #endif
    b->CreateUnlikelyCondBr(initiallyTerminated, mKernelExit, checkProducers);

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
    b->CreateUnlikelyCondBr(mTerminationExplicitly, terminated, incrementItemCounts);

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
    setTerminated(b, b->getTrue());
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
    updateOptionalCycleCounter(b);

    assert (mKernel == mPipeline[mKernelIndex] && b->getKernel() == mKernel);
}

// Synchronization actions for executing a kernel for a particular logical segment.

// Before the segment is processed, CreateAtomicLoadAcquire must be used to load
// the segment number of the kernel state to ensure that the previous segment is
// complete (by checking that the acquired segment number is equal to the desired segment
// number).

// After all segment processing actions for the kernel are complete, and any necessary
// data has been extracted from the kernel for further pipeline processing, the
// segment number must be incremented and stored using CreateAtomicStoreRelease.

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief synchronize
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::synchronize(BuilderRef b) {

    const auto prefix = makeKernelName(mKernelIndex);
    b->setKernel(mPipelineKernel);
    BasicBlock * const kernelWait = b->CreateBasicBlock(prefix + "Wait", mPipelineEnd);
    b->CreateBr(kernelWait);

    b->SetInsertPoint(kernelWait);
    const auto serialize = codegen::DebugOptionIsSet(codegen::SerializeThreads);
    const unsigned waitingOnIdx = serialize ? mPipeline.size() - 1 : mKernelIndex;
    const auto waitingOn = makeKernelName(waitingOnIdx);
    Value * const waitingOnPtr = b->getScalarFieldPtr(waitingOn + LOGICAL_SEGMENT_NO_SCALAR);
    Value * const processedSegmentCount = b->CreateAtomicLoadAcquire(waitingOnPtr);
    assert (processedSegmentCount->getType() == mSegNo->getType());
    Value * const ready = b->CreateICmpEQ(mSegNo, processedSegmentCount);

    BasicBlock * const kernelCheck = b->CreateBasicBlock(prefix + "Check", mPipelineEnd);
    b->CreateCondBr(ready, kernelCheck, kernelWait);

    b->SetInsertPoint(kernelCheck);
    b->setKernel(mKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief releaseCurrentSegment
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::releaseCurrentSegment(BuilderRef b) {
    b->setKernel(mPipelineKernel);
    Value * const nextSegNo = b->CreateAdd(mSegNo, b->getSize(1));
    const auto prefix = makeKernelName(mKernelIndex);
    Value * const waitingOnPtr = b->getScalarFieldPtr(prefix + LOGICAL_SEGMENT_NO_SCALAR);
    b->CreateAtomicStoreRelease(nextSegNo, waitingOnPtr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief next
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::end(BuilderRef b, const unsigned step) {
    b->setKernel(mPipelineKernel);
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
        setActiveKernel(b, parent(bufferVertex, mBufferGraph));
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
    b->setKernel(mPipelineKernel);
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitialTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getInitialTerminationSignal(BuilderRef b) const {
    b->setKernel(mPipelineKernel);
    const auto prefix = makeKernelName(mKernelIndex);
    Value * const terminated = b->getScalarField(prefix + TERMINATION_SIGNAL);
    b->setKernel(mKernel);
    return terminated;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::setTerminated(BuilderRef b, Value * const value) {
    const auto prefix = makeKernelName(mKernelIndex);
    b->setKernel(mPipelineKernel);
    b->setScalarField(prefix + TERMINATION_SIGNAL, value);
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt("*** " + prefix + "_terminated ***", value);
    #endif
    b->setKernel(mKernel);
}

}
