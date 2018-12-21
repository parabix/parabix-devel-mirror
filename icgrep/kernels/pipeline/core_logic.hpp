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

    IntegerType * const sizeTy = b->getSizeTy();

    const auto name = makeKernelName(kernelIndex);
    // TODO: prove two termination signals can be fused into a single counter?
    mPipelineKernel->addInternalScalar(sizeTy, name + TERMINATION_SIGNAL);
    mPipelineKernel->addInternalScalar(sizeTy, name + LOGICAL_SEGMENT_SUFFIX);

    // TODO: non deferred item count for fixed rates could be calculated from seg no.
    // Should I seperate non-deferred from normal item counts to improve cache locality?
    const Kernel * const kernel = mPipeline[kernelIndex];
    const auto numOfInputs = kernel->getNumOfStreamInputs();
    for (unsigned i = 0; i < numOfInputs; i++) {
        const Binding & input = kernel->getInputStreamSetBinding(i);
        const auto prefix = makeBufferName(kernelIndex, input);
        if (input.isDeferred()) {
            mPipelineKernel->addInternalScalar(sizeTy, prefix + DEFERRED_ITEM_COUNT_SUFFIX);
        }
        mPipelineKernel->addInternalScalar(sizeTy, prefix + ITEM_COUNT_SUFFIX);
    }

    const auto numOfOutputs = kernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const Binding & output = kernel->getOutputStreamSetBinding(i);
        const auto prefix = makeBufferName(kernelIndex, output);
        if (output.isDeferred()) {
            mPipelineKernel->addInternalScalar(sizeTy, prefix + DEFERRED_ITEM_COUNT_SUFFIX);
        }
        mPipelineKernel->addInternalScalar(sizeTy, prefix + ITEM_COUNT_SUFFIX);
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
    expandOutputBuffers(b);
    writeKernelCall(b);

    BasicBlock * const copyBack =
            b->CreateBasicBlock(prefix + "_copyBack", mKernelTerminationCheck);
    BasicBlock * const abnormalTermination =
            b->CreateBasicBlock(prefix + "_abnormalTermination", mKernelTerminationCheck);

    // If the kernel explicitly terminates, it must set its processed/produced item counts.
    // Otherwise, the pipeline will update any countable rates, even upon termination.
    b->CreateUnlikelyCondBr(mTerminationExplicitly, abnormalTermination, copyBack);

    /// -------------------------------------------------------------------------------------
    /// KERNEL COPY BACK
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(copyBack);
    writeCopyBackLogic(b);
    b->CreateBr(mKernelTerminationCheck);

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
    setTerminated(b, b->getTrue());
    updatePhisAfterTermination(b);
    b->CreateBr(mKernelLoopExit);

    /// -------------------------------------------------------------------------------------
    /// KERNEL LOOP EXIT
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopExit);
    writeUpdatedItemCounts(b);
    computeFullyProcessedItemCounts(b);
    computeMinimumConsumedItemCounts(b);
    computeMinimumPopCountReferenceCounts(b);
    computeFullyProducedItemCounts(b);
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
    readFinalProducedItemCounts(b);
    updateOptionalCycleCounter(b);

    assert (mKernel == mPipeline[mKernelIndex] && b->getKernel() == mKernel);
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

        const auto bufferVertex = source(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[bufferVertex];
        const StreamSetBuffer * const buffer = bn.Buffer;

        Value * const produced = bn.TotalItems; assert (produced);
        Value * const consumed = b->getSize(0);  assert (consumed);
        Value * const writable = buffer->getLinearlyWritableItems(b, produced, consumed, getCopyBack(bufferVertex));

//        const auto kernelVertex = parent(bufferVertex, mBufferGraph);
        const BufferRateData & rd = mBufferGraph[e];
        const auto outputPort = rd.Port;

        // NOTE: this method doesn't check a popcount's ref stream to determine how many
        // items we actually require. Instead it just calculates them as bounded rates.
        // To support a precise bound, we'd need to produce more ref items than the kernel
        // that writes to this output actually consumes. Since this effectively adds a
        // delay equivalent to a LookAhead of a full stride, this doesn't seem useful.
        const Binding & output = mPipelineKernel->getOutputStreamSetBinding(outputPort);
        Value * const strideLength = getMaximumStrideLength(b, mPipelineKernel, output);
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
    if (mPipelineProgress) {
        const auto prefix = makeKernelName(mKernelIndex);
        mAlreadyProgressedPhi = b->CreatePHI(b->getInt1Ty(), 2, prefix + "_madeProgress");
        mAlreadyProgressedPhi->addIncoming(mPipelineProgress, mKernelEntry);
    }
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
    mTerminatedPhi = b->CreatePHI(b->getInt1Ty(), 2, prefix + "_terminated");
    if (mPipelineProgress) {
        mHasProgressedPhi = b->CreatePHI(b->getInt1Ty(), 2, prefix + "_anyProgress");
    }
    Type * const sizeTy = b->getSizeTy();
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
    PHINode * const terminated = b->CreatePHI(b->getInt1Ty(), 2, prefix + "_terminated");
    terminated->addIncoming(b->getTrue(), mKernelEntry);
    terminated->addIncoming(mTerminatedPhi, mKernelLoopExitPhiCatch);
    mTerminationGraph[mKernelIndex] = terminated;
    if (mPipelineProgress) {
        PHINode * const pipelineProgress = b->CreatePHI(b->getInt1Ty(), 2, prefix + "_pipelineProgress");
        pipelineProgress->addIncoming(mPipelineProgress, mKernelEntry);
        pipelineProgress->addIncoming(mHasProgressedPhi, mKernelLoopExitPhiCatch);
        mPipelineProgress = pipelineProgress;
    }
    createConsumedPhiNodes(b);
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    Type * const sizeTy = b->getSizeTy();
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
        if (mHasProgressedPhi) {
            mHasProgressedPhi->addIncoming(b->getTrue(), entryBlock);
        }
        mTerminatedPhi->addIncoming(b->getFalse(), entryBlock);
        b->CreateBr(mKernelLoopExit);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitialTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::initiallyTerminated(BuilderRef b) const {
    b->setKernel(mPipelineKernel);
    const auto prefix = makeKernelName(mKernelIndex);
    Value * const terminated = b->getScalarField(prefix + TERMINATION_SIGNAL);
    b->setKernel(mKernel);
    return b->CreateICmpNE(terminated, b->getSize(0));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::setTerminated(BuilderRef b, Value * const value) {
    const auto prefix = makeKernelName(mKernelIndex);
    b->setKernel(mPipelineKernel);
    b->setScalarField(prefix + TERMINATION_SIGNAL, b->CreateZExtOrTrunc(value, b->getSizeTy()));
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt("*** " + prefix + "_terminated ***", value);
    #endif
    b->setKernel(mKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updatePhiCountAfterTermination
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::updatePhisAfterTermination(BuilderRef b) {
    BasicBlock * const exitBlock = b->GetInsertBlock();
    assert (mTerminatedPhi);
    mTerminatedPhi->addIncoming(b->getTrue(), exitBlock);
    if (mHasProgressedPhi) {
        mHasProgressedPhi->addIncoming(b->getTrue(), exitBlock);
    }
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        // TODO: set these to the total produced item count for that input?
        mUpdatedProcessedPhi[i]->addIncoming(mFinalProcessedPhi[i], exitBlock);
        if (mUpdatedProcessedDeferredPhi[i]) {
            mUpdatedProcessedDeferredPhi[i]->addIncoming(mFinalProcessedPhi[i], exitBlock);
        }
    }
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        mUpdatedProducedPhi[i]->addIncoming(mFinalProducedPhi[i], exitBlock);
    }
}

}
