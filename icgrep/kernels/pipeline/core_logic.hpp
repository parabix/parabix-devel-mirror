#include "pipeline_compiler.hpp"

namespace kernel {

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

    const auto numOfKernels = mPipeline.size();
    for (mKernelIndex = 0; mKernelIndex < numOfKernels; ++mKernelIndex) {
        mKernel = mPipeline[mKernelIndex];
        const auto numOfInputs = mKernel->getNumOfStreamInputs();
        for (unsigned j = 0; j < numOfInputs; ++j) {
            allocateThreadLocalState(b, Port::Input, j);
        }
        const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
        for (unsigned j = 0; j < numOfOutputs; ++j) {
            allocateThreadLocalState(b, Port::Output, j);
        }
    }

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
    startOptionalCycleCounter(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief executeKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::executeKernel(BuilderRef b) {

    assert (mKernel == mPipeline[mKernelIndex] && b->getKernel() == mKernel);

    loadBufferHandles(b);
    mNoMore = nullptr;

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

    b->CreateUnlikelyCondBr(b->getTerminationSignal(), mKernelExit, checkProducers);

    /// -------------------------------------------------------------------------------------
    /// KERNEL CHECK PRODUCERS
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(checkProducers);
    checkIfAllProducingKernelsHaveTerminated(b);
    storeCopyForwardProducedItemCounts(b);
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

    assert (mKernel == mPipeline[mKernelIndex] && b->getKernel() == mKernel);

    /// -------------------------------------------------------------------------------------
    /// KERNEL CALL
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopCall);
    initializeKernelCallPhis(b);

    assert (mKernel == mPipeline[mKernelIndex] && b->getKernel() == mKernel);

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

    assert (mKernel == mPipeline[mKernelIndex] && b->getKernel() == mKernel);

    /// -------------------------------------------------------------------------------------
    /// KERNEL LOOP ENTRY (CONTINUED)
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopEntry);
    ConstantInt * const ZERO = b->getSize(0);
    ConstantInt * const TRUE = b->getTrue();
    ConstantInt * const FALSE = b->getFalse();
    Value * const nonFinal = checkForSufficientInputDataAndOutputSpace(b);
    if (nonFinal) {

        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            Value * const valid = b->CreateOr(nonFinal, mNoMore);
            b->CreateAssert(valid, kernelName + ": isFinal was set before all producers have terminated");
        }

        BasicBlock * const enteringNonFinalSegment = b->CreateBasicBlock(kernelName + "_nonFinalSegment", mKernelLoopCall);
        BasicBlock * const enteringFinalStride = b->CreateBasicBlock(kernelName + "_finalStride", mKernelLoopCall);
        b->CreateLikelyCondBr(nonFinal, enteringNonFinalSegment, enteringFinalStride);

        /// -------------------------------------------------------------------------------------
        /// KERNEL ENTERING NON-FINAL SEGMENT
        /// -------------------------------------------------------------------------------------

        b->SetInsertPoint(enteringNonFinalSegment);
        Value * const numOfLinearStrides = determineNumOfLinearStrides(b);
        calculateNonFinalItemCounts(b, numOfLinearStrides);
        BasicBlock * const endOfNonFinalStride = b->GetInsertBlock();
        mNumOfLinearStridesPhi->addIncoming(numOfLinearStrides, endOfNonFinalStride);
        mIsFinalPhi->addIncoming(FALSE, endOfNonFinalStride);
        b->CreateBr(mKernelLoopCall);

        /// -------------------------------------------------------------------------------------
        /// KERNEL ENTERING FINAL STRIDE
        /// -------------------------------------------------------------------------------------

        b->SetInsertPoint(enteringFinalStride);
        calculateFinalItemCounts(b);
        BasicBlock * const endOfFinalStride = b->GetInsertBlock();
        mNumOfLinearStridesPhi->addIncoming(ZERO, endOfFinalStride);
        mIsFinalPhi->addIncoming(TRUE, endOfFinalStride);
        b->CreateBr(mKernelLoopCall);

    } else {

        BasicBlock * const endOfLoopEntry = b->GetInsertBlock();
        mNumOfLinearStridesPhi->addIncoming(ZERO, endOfLoopEntry);
        mIsFinalPhi->addIncoming(mNoMore, endOfLoopEntry);
        b->CreateBr(mKernelLoopCall);
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL CALL (CONTINUED)
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopCall);
    assert (mKernel == mPipeline[mKernelIndex] && b->getKernel() == mKernel);
    storeCopyBackProducedItemCounts(b);
    writeKernelCall(b);
    assert (mKernel == mPipeline[mKernelIndex] && b->getKernel() == mKernel);
    writeCopyBackLogic(b);
    Value * const terminated = isTerminated(b);
    BasicBlock * const kernelTerminated = b->CreateBasicBlock(kernelName + "_markAsTerminated", mKernelLoopExit);
    BasicBlock * const callKernelEnd = b->GetInsertBlock();
    if (LLVM_LIKELY(mKernel->getNumOfStreamInputs() > 0)) { // hasAnyInputChecks()
        if (mAlreadyProgressedPhi) {
            mAlreadyProgressedPhi->addIncoming(TRUE, callKernelEnd);
        }
        b->CreateUnlikelyCondBr(terminated, kernelTerminated, mKernelLoopEntry);
    } else { // just exit the loop
        if (mHasProgressedPhi) {
            mHasProgressedPhi->addIncoming(TRUE, callKernelEnd);
        }
        mTerminatedPhi->addIncoming(FALSE, callKernelEnd);
        b->CreateUnlikelyCondBr(terminated, kernelTerminated, mKernelLoopExit);
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL TERMINATED
    /// -------------------------------------------------------------------------------------

    // mark this kernel as terminated
    b->SetInsertPoint(kernelTerminated);
    zeroFillPartiallyWrittenOutputStreams(b);
    setTerminated(b);
    BasicBlock * const kernelTerminatedExit = b->GetInsertBlock();
    mTerminatedPhi->addIncoming(TRUE, kernelTerminatedExit);
    if (mHasProgressedPhi) {
        mHasProgressedPhi->addIncoming(TRUE, kernelTerminatedExit);
    }
    b->CreateBr(mKernelLoopExit);

    /// -------------------------------------------------------------------------------------
    /// KERNEL LOOP EXIT (CONTINUED)
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelLoopExit);
    computeMinimumConsumedItemCounts(b);
    writeCopyForwardLogic(b);
    b->CreateBr(mKernelLoopExitPhiCatch);
    b->SetInsertPoint(mKernelLoopExitPhiCatch);
    b->CreateBr(mKernelExit);

    /// -------------------------------------------------------------------------------------
    /// KERNEL EXIT (CONTINUED)
    /// -------------------------------------------------------------------------------------

    b->SetInsertPoint(mKernelExit);
    writeFinalConsumedItemCounts(b);

    // TODO: logically we should only need to read produced item counts in the loop exit; however, that
    // would mean that we'd first need to load the initial produced item counts prior to the loop entry
    // to have access to them here and then PHI them out within the kernel loop

    readCurrentProducedItemCounts(b);
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
        b->CreateAssert(b->CreateICmpNE(newCount, TWO), "Dead lock detected: pipeline could not progress after two iterations");
        mDeadLockCounter->addIncoming(newCount, b->GetInsertBlock());
    }
    // return whether each sink has terminated
    Value * done = b->getTrue();
    for (const auto e : make_iterator_range(in_edges(mPipeline.size(), mTerminationGraph))) {
        const auto u = source(e, mTerminationGraph);
        assert (mTerminationGraph[u]);
        done = b->CreateAnd(done, mTerminationGraph[u]);
    }
    Value * const nextSegNo = b->CreateAdd(mSegNo, b->getSize(step));
    mSegNo->addIncoming(nextSegNo, b->GetInsertBlock());
    b->CreateUnlikelyCondBr(done, mPipelineEnd, mPipelineLoop);

    b->SetInsertPoint(mPipelineEnd);
    for (unsigned i = 0; i < mPipeline.size(); ++i) {
        setActiveKernel(b, i);
        const auto numOfInputs = mKernel->getNumOfStreamInputs();
        for (unsigned j = 0; j < numOfInputs; ++j) {
            deallocateThreadLocalState(b, Port::Input, j);
        }
        const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
        for (unsigned j = 0; j < numOfOutputs; ++j) {
            deallocateThreadLocalState(b, Port::Output, j);
        }
    }
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

    const auto kernelName = makeKernelName(mKernelIndex);

    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    mAccessibleInputItemsPhi.clear();
    mAccessibleInputItemsPhi.resize(numOfInputs, nullptr);
    Type * const sizeTy = b->getSizeTy();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = mKernel->getInputStreamSetBinding(i);
        mAccessibleInputItemsPhi[i] = b->CreatePHI(sizeTy, 2, kernelName + "_" + input.getName() + "_accessible");
    }
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    mWritableOutputItemsPhi.clear();
    mWritableOutputItemsPhi.resize(numOfOutputs, nullptr);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = mKernel->getOutputStreamSetBinding(i);
        if (!output.hasAttribute(AttrId::ManagedBuffer)) {
            mWritableOutputItemsPhi[i] = b->CreatePHI(sizeTy, 2, kernelName + "_" + output.getName() + "_writable");
        }
    }
    mNumOfLinearStridesPhi = b->CreatePHI(b->getSizeTy(), 2, kernelName + "_numOfLinearStrides");
    mIsFinalPhi = b->CreatePHI(b->getInt1Ty(), 2, kernelName + "_isFinal");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernelExitPhis
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeKernelExitPhis(BuilderRef b) {
    const auto kernelName = makeKernelName(mKernelIndex);

    PHINode * const terminatedPhi = b->CreatePHI(b->getInt1Ty(), 2, kernelName + "_terminated");
    terminatedPhi->addIncoming(b->getTrue(), mKernelEntry);
    terminatedPhi->addIncoming(mTerminatedPhi, mKernelLoopExitPhiCatch);
    mTerminationGraph[mKernelIndex] = terminatedPhi;
    if (mPipelineProgress) {
        PHINode * const pipelineProgress = b->CreatePHI(b->getInt1Ty(), 2, "pipelineProgress");
        pipelineProgress->addIncoming(mPipelineProgress, mKernelEntry);
        pipelineProgress->addIncoming(mHasProgressedPhi, mKernelLoopExitPhiCatch);
        mPipelineProgress = pipelineProgress;
    }
    createConsumedPhiNodes(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkIfAllInputKernelsHaveFinished
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::checkIfAllProducingKernelsHaveTerminated(BuilderRef b) {
    const auto n = in_degree(mKernelIndex, mTerminationGraph);
    Value * noMore = nullptr;
    if (LLVM_UNLIKELY(n == 0)) {
        noMore = b->getFalse();
    } else {
        for (auto e : make_iterator_range(in_edges(mKernelIndex, mTerminationGraph))) {
            const auto u = source(e, mTerminationGraph);
            Value * const finished = mTerminationGraph[u];
            assert (finished);
            if (noMore) {
                noMore = b->CreateAnd(noMore, finished);
            } else {
                noMore = finished;
            }
        }
    }
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(mKernel->getName() + "_noMore", noMore);
    #endif
    mNoMore = noMore;
}

#warning TODO: move termination variables into pipeline if the kernel cannot signal termination itself.

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::isTerminated(BuilderRef b) const {
    if (LLVM_UNLIKELY(mKernel->hasAttribute(AttrId::MustExplicitlyTerminate))) {
        return b->getTerminationSignal();
    } else {
        if (mKernel->hasAttribute(AttrId::CanTerminateEarly)) {
            Value * const terminated = b->getTerminationSignal();
            return b->CreateOr(mIsFinalPhi, terminated);
        } else {
            return mIsFinalPhi;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::setTerminated(BuilderRef b) {
    if (LLVM_UNLIKELY(mKernel->hasAttribute(AttrId::MustExplicitlyTerminate))) {
        return;
    }
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mKernel->getHandle(), CBuilder::Protect::WRITE);
    }
    b->setTerminationSignal();
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mKernel->getHandle(), CBuilder::Protect::READ);
    }
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
