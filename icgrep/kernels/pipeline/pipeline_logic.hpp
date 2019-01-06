#ifndef PIPELINE_LOGIC_HPP
#define PIPELINE_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

// NOTE: the following is a workaround for an LLVM bug for 32-bit VMs on 64-bit architectures.
// When calculating the address of a local stack allocated object, the size of a pointer is
// 32-bits but when performing the same GEP on a pointer returned by "malloc" or passed as a
// function argument, the size is 64-bits. More investigation is needed to determine which
// versions of LLVM are affected by this bug.

inline LLVM_READNONE bool allocateOnHeap(const BuilderRef b) {
    DataLayout DL(b->getModule());
    return (DL.getPointerSizeInBits() != b->getSizeTy()->getBitWidth());
}

inline Value * makeStateObject(BuilderRef b, Type * type) {
    Value * ptr = nullptr;
    if (LLVM_UNLIKELY(allocateOnHeap(b))) {
        ptr = b->CreateCacheAlignedMalloc(type);
    } else {
        ptr = b->CreateCacheAlignedAlloca(type);
    }
    b->CreateMemZero(ptr, ConstantExpr::getSizeOf(type), b->getCacheAlignment());
    return ptr;
}

inline void destroyStateObject(BuilderRef b, Value * ptr) {
    if (LLVM_UNLIKELY(allocateOnHeap(b))) {
        b->CreateFree(ptr);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addPipelineKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addPipelineKernelProperties(BuilderRef b) {
    b->setKernel(mPipelineKernel);
    addTerminationProperties(b);
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        addBufferHandlesToPipelineKernel(b, i);
        addInternalKernelProperties(b, i);
        addConsumerKernelProperties(b, i);
        addPopCountScalarsToPipelineKernel(b, i);
    }
    b->setKernel(mPipelineKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addInternalKernelProperties(BuilderRef b, const unsigned kernelIndex) {

    IntegerType * const sizeTy = b->getSizeTy();

    const auto name = makeKernelName(kernelIndex);

//    mPipelineKernel->addInternalScalar(sizeTy, name + TERMINATION_SIGNAL_SUFFIX);
    mPipelineKernel->addInternalScalar(sizeTy, name + LOGICAL_SEGMENT_SUFFIX);

    // TODO: non deferred item count for fixed rates could be calculated from total # of segments.
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
        mPipelineKernel->addInternalScalar(sizeTy, prefix + ITEM_COUNT_SUFFIX);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateInitializeMethod(BuilderRef b) {
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        mPipeline[i]->addKernelDeclarations(b);
    }
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        Kernel * const kernel = mPipeline[i];
        if (!kernel->hasFamilyName()) {
            Value * const handle = kernel->createInstance(b);
            b->setScalarField(makeKernelName(i), handle);
        }
    }
    constructBuffers(b);
    std::vector<Value *> args;
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        setActiveKernel(b, i);
        args.resize(in_degree(i, mScalarDependencyGraph) + 1);
        args[0] = mKernel->getHandle();
        b->setKernel(mPipelineKernel);
        for (const auto ce : make_iterator_range(in_edges(i, mScalarDependencyGraph))) {
            const auto j = mScalarDependencyGraph[ce] + 1;
            const auto pe = in_edge(source(ce, mScalarDependencyGraph), mScalarDependencyGraph);
            const auto k = mScalarDependencyGraph[pe];
            const Binding & input = mPipelineKernel->getInputScalarBinding(k);
            args[j] = b->getScalarField(input.getName());
        }
        b->setKernel(mKernel);
        Value * const terminatedOnInit = b->CreateCall(getInitializationFunction(b), args);

        const auto prefix = makeKernelName(mKernelIndex);
        BasicBlock * const kernelTerminated = b->CreateBasicBlock(prefix + "_terminatedOnInit");
        BasicBlock * const kernelExit = b->CreateBasicBlock(prefix + "_exit");
        b->CreateUnlikelyCondBr(terminatedOnInit, kernelTerminated, kernelExit);

        b->SetInsertPoint(kernelTerminated);
        setTerminated(b);
        b->CreateBr(kernelExit);

        b->SetInsertPoint(kernelExit);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateSingleThreadKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateSingleThreadKernelMethod(BuilderRef b) {
    StructType * const localStateType = getLocalStateType(b);
    Value * const localState = makeStateObject(b, localStateType);
    b->CreateMemZero(localState, ConstantExpr::getSizeOf(localStateType), b->getCacheAlignment());
    allocateThreadLocalState(b, localState);
    setThreadLocalState(b, localState);
    start(b, b->getSize(0));
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        setActiveKernel(b, i);
        executeKernel(b);
    }
    end(b, 1);
    deallocateThreadLocalState(b, localState);
    destroyStateObject(b, localState);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateMultiThreadKernelMethod
 *
 * Given a computation expressed as a logical pipeline of K kernels k0, k_1, ...k_(K-1)
 * operating over an input stream set S, a segment-parallel implementation divides the input
 * into segments and coordinates a set of T <= K threads to each process one segment at a time.
 * Let S_0, S_1, ... S_N be the segments of S.   Segments are assigned to threads in a round-robin
 * fashion such that processing of segment S_i by the full pipeline is carried out by thread i mod T.
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateMultiThreadKernelMethod(BuilderRef b) {

    const auto numOfThreads = mPipelineKernel->getNumOfThreads();

    assert (numOfThreads > 1);

    Module * const m = b->getModule();
    PointerType * const voidPtrTy = b->getVoidPtrTy();
    ConstantInt * const ZERO = b->getInt32(0);
    Constant * const nullVoidPtrVal = ConstantPointerNull::getNullValue(voidPtrTy);

    FunctionType * const threadFuncType = FunctionType::get(b->getVoidTy(), {voidPtrTy}, false);
    const auto threadName = mPipelineKernel->getName() + "_DoSegmentThread";
    Function * const threadFunc = Function::Create(threadFuncType, Function::InternalLinkage, threadName, m);
    threadFunc->setCallingConv(CallingConv::C);
    auto args = threadFunc->arg_begin();
    args->setName("kernelStateObject");

    // -------------------------------------------------------------------------------------------------------------------------
    // MAKE PIPELINE DRIVER CONTINUED
    // -------------------------------------------------------------------------------------------------------------------------

    // use the process thread to handle the initial segment function after spawning
    // (n - 1) threads to handle the subsequent offsets
    const unsigned threads = numOfThreads - 1;
    Type * const pthreadsTy = ArrayType::get(b->getPThreadTy(), threads);
    AllocaInst * const pthreads = b->CreateCacheAlignedAlloca(pthreadsTy);
    std::vector<Value *> threadIdPtr(threads);
    std::vector<Value *> threadState(threads);
    for (unsigned i = 0; i < threads; ++i) {
        threadState[i] = allocateThreadState(b, i + 1);
        threadIdPtr[i] = b->CreateGEP(pthreads, {ZERO, b->getInt32(i)});
        b->CreatePThreadCreateCall(threadIdPtr[i], nullVoidPtrVal, threadFunc, threadState[i]);
    }

    // execute the process thread
    Value * const processState = allocateThreadState(b, 0);
    b->CreateCall(threadFunc, b->CreatePointerCast(processState, voidPtrTy));
    deallocateThreadState(b, processState);

    // wait for all other threads to complete
    AllocaInst * const status = b->CreateAlloca(voidPtrTy);
    for (unsigned i = 0; i < threads; ++i) {
        Value * threadId = b->CreateLoad(threadIdPtr[i]);
        b->CreatePThreadJoinCall(threadId, status);
        deallocateThreadState(b, threadState[i]);
    }

    // store where we'll resume compiling the DoSegment method
    const auto resumePoint = b->saveIP();

    // -------------------------------------------------------------------------------------------------------------------------
    // MAKE PIPELINE THREAD
    // -------------------------------------------------------------------------------------------------------------------------
    b->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", threadFunc));
    Value * const threadStruct = b->CreateBitCast(&*(args), getThreadStateType(b)->getPointerTo());
    Value * const segmentOffset = setThreadState(b, threadStruct);
    // generate the pipeline logic for this thread
    start(b, segmentOffset);
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        setActiveKernel(b, i);
        acquireCurrentSegment(b);
        executeKernel(b);
        releaseCurrentSegment(b);
    }
    mKernel = nullptr;
    mKernelIndex = 0;
    end(b, numOfThreads);
    // only call pthread_exit() within spawned threads; otherwise it'll be equivalent to calling exit() within the process
    BasicBlock * const exitThread = b->CreateBasicBlock("ExitThread");
    BasicBlock * const exitFunction = b->CreateBasicBlock("ExitProcessFunction");
    b->CreateCondBr(b->CreateIsNull(segmentOffset), exitFunction, exitThread);
    b->SetInsertPoint(exitThread);
    b->CreatePThreadExitCall(nullVoidPtrVal);
    b->CreateBr(exitFunction);
    b->SetInsertPoint(exitFunction);
    b->CreateRetVoid();

    // Restore our position to allow the pipeline kernel to complete the function
    b->restoreIP(resumePoint);
    setThreadState(b, processState);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateFinalizeMethod(BuilderRef b) {
    printOptionalCycleCounter(b);
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        setActiveKernel(b, i);
        loadBufferHandles(b);
        mScalarDependencyGraph[i] = b->CreateCall(getFinalizeFunction(b), mKernel->getHandle());
    }
    releaseBuffers(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalOutputScalars
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Value *> PipelineCompiler::getFinalOutputScalars(BuilderRef b) {

    const auto & calls = mPipelineKernel->getCallBindings();
    const auto numOfCalls = calls.size();
    std::vector<Value *> args;
    b->setKernel(mPipelineKernel);
    const auto pipelineOutput = mLastKernel;
    const auto firstCall = pipelineOutput + 1;
    for (unsigned k = 0; k < numOfCalls; ++k) {
        writeOutputScalars(b, firstCall + k, args);
        Function * const f = cast<Function>(calls[k].Callee);
        auto i = f->arg_begin();
        for (auto j = args.begin(); j != args.end(); ++i, ++j) {
            assert (i != f->arg_end());
            *j = b->CreateZExtOrTrunc(*j, i->getType());
        }
        assert (i == f->arg_end());
        b->CreateCall(f, args);
    }
    writeOutputScalars(b, pipelineOutput, args);
    return args;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeOutputScalars
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeOutputScalars(BuilderRef b, const unsigned index, std::vector<Value *> & args) {
    const auto n = in_degree(index, mScalarDependencyGraph);
    args.resize(n);
    const auto pipelineInput = 0;
    for (const auto e : make_iterator_range(in_edges(index, mScalarDependencyGraph))) {
        const auto scalar = source(e, mScalarDependencyGraph);
        // If we have not already retrieved the specific scalar, construct/load/extract it.
        if (LLVM_LIKELY(mScalarDependencyGraph[scalar] == nullptr)) {
            const auto producer = in_edge(scalar, mScalarDependencyGraph);
            const auto i = source(producer, mScalarDependencyGraph);
            const auto j = mScalarDependencyGraph[producer];
            Value * value = nullptr;
            if (LLVM_UNLIKELY(i == pipelineInput)) {
                const Binding & input = mPipelineKernel->getInputScalarBinding(j);
                const Relationship * const rel = getRelationship(input);
                if (LLVM_UNLIKELY(isa<ScalarConstant>(rel))) {
                    value = cast<ScalarConstant>(rel)->value();
                } else {
                    value = b->getScalarField(input.getName());
                }
            } else { // output scalar of some kernel
                Value * const outputScalars = mScalarDependencyGraph[i]; assert (outputScalars);
                if (outputScalars->getType()->isAggregateType()) {
                    value = b->CreateExtractValue(outputScalars, {j});
                } else { assert (j == 0 && "scalar type is not an aggregate");
                    value = outputScalars;
                }
            }
            mScalarDependencyGraph[scalar] = value;
        }
        const auto k = mScalarDependencyGraph[e];
        args[k] = mScalarDependencyGraph[scalar];
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief acquireCurrentSegment
 *
 * Before the segment is processed, this loads the segment number of the kernel state and ensures the previous
 * segment is complete (by checking that the acquired segment number is equal to the desired segment number).
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::acquireCurrentSegment(BuilderRef b) {

    b->setKernel(mPipelineKernel);
    const auto prefix = makeKernelName(mKernelIndex);
    const auto serialize = codegen::DebugOptionIsSet(codegen::SerializeThreads);
    const unsigned waitingOnIdx = serialize ? (mLastKernel - 1) : mKernelIndex;
    const auto waitingOn = makeKernelName(waitingOnIdx);
    Value * const waitingOnPtr = b->getScalarFieldPtr(waitingOn + LOGICAL_SEGMENT_SUFFIX);
    BasicBlock * const kernelWait = b->CreateBasicBlock(prefix + "Wait", mPipelineEnd);
    b->CreateBr(kernelWait);

    b->SetInsertPoint(kernelWait);
    Value * const processedSegmentCount = b->CreateAtomicLoadAcquire(waitingOnPtr);
    assert (processedSegmentCount->getType() == mSegNo->getType());
    Value * const ready = b->CreateICmpEQ(mSegNo, processedSegmentCount);
    BasicBlock * const kernelStart = b->CreateBasicBlock(prefix + "Start", mPipelineEnd);
    b->CreateCondBr(ready, kernelStart, kernelWait);

    b->SetInsertPoint(kernelStart);
    b->setKernel(mKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief releaseCurrentSegment
 *
 * After executing the kernel, the segment number must be incremented to release the kernel for the next thread.
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::releaseCurrentSegment(BuilderRef b) {
    b->setKernel(mPipelineKernel);
    Value * const nextSegNo = b->CreateAdd(mSegNo, b->getSize(1));
    const auto prefix = makeKernelName(mKernelIndex);
    Value * const waitingOnPtr = b->getScalarFieldPtr(prefix + LOGICAL_SEGMENT_SUFFIX);
    b->CreateAtomicStoreRelease(nextSegNo, waitingOnPtr);
}

enum : unsigned {
    HANDLE_INDEX = 0
    , SEGMENT_OFFSET_INDEX = 1
    , LOCAL_STATE_INDEX = 2
    , FIRST_INPUT_STREAM_INDEX = 3
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getThreadStateType
 ** ------------------------------------------------------------------------------------------------------------- */
inline StructType * PipelineCompiler::getThreadStateType(BuilderRef b) {
    std::vector<Type *> threadStructFields;
    Type * const handleType = mPipelineKernel->getHandle()->getType();
    threadStructFields.push_back(handleType);
    threadStructFields.push_back(b->getSizeTy());
    threadStructFields.push_back(getLocalStateType(b));
    const auto numOfInputs = mPipelineKernel->getNumOfStreamInputs();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        auto buffer = mPipelineKernel->getInputStreamSetBuffer(i);
        Value * const handle = buffer->getHandle();
        threadStructFields.push_back(handle->getType());
    }
    const auto numOfOutputs = mPipelineKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        auto buffer = mPipelineKernel->getOutputStreamSetBuffer(i);
        Value * const handle = buffer->getHandle();
        threadStructFields.push_back(handle->getType());
    }
    return StructType::get(b->getContext(), threadStructFields);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructThreadState
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::allocateThreadState(BuilderRef b, const unsigned segOffset) {

    StructType * const threadStructType = getThreadStateType(b);
    Value * const threadState = makeStateObject(b, threadStructType);

    std::vector<Value *> indices(2);
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(HANDLE_INDEX);
    Value * const handle = mPipelineKernel->getHandle();
    b->CreateStore(handle, b->CreateGEP(threadState, indices));
    indices[1] = b->getInt32(SEGMENT_OFFSET_INDEX);
    b->CreateStore(b->getSize(segOffset), b->CreateGEP(threadState, indices));
    indices[1] = b->getInt32(LOCAL_STATE_INDEX);
    allocateThreadLocalState(b, b->CreateGEP(threadState, indices));

    const auto numOfInputs = mPipelineKernel->getNumOfStreamInputs();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto buffer = mPipelineKernel->getInputStreamSetBuffer(i);
        Value * const handle = buffer->getHandle();
        indices[1] = b->getInt32(FIRST_INPUT_STREAM_INDEX + i);
        b->CreateStore(handle, b->CreateGEP(threadState, indices));
    }

    const auto FIRST_OUTPUT_STREAM_INDEX = FIRST_INPUT_STREAM_INDEX + numOfInputs;
    const auto numOfOutputs = mPipelineKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto buffer = mPipelineKernel->getOutputStreamSetBuffer(i);
        Value * const handle = buffer->getHandle();
        indices[1] = b->getInt32(FIRST_OUTPUT_STREAM_INDEX + i);
        b->CreateStore(handle, b->CreateGEP(threadState, indices));
    }

    return threadState;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructThreadState
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::setThreadState(BuilderRef b, Value * threadState) {

    std::vector<Value *> indices(2);
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(HANDLE_INDEX);

    Value * handle = b->CreateLoad(b->CreateGEP(threadState, indices));
    mPipelineKernel->setHandle(b, handle);

    indices[1] = b->getInt32(SEGMENT_OFFSET_INDEX);
    Value * const segmentOffset = b->CreateLoad(b->CreateGEP(threadState, indices));

    indices[1] = b->getInt32(LOCAL_STATE_INDEX);
    setThreadLocalState(b, b->CreateGEP(threadState, indices));

    const auto numOfInputs = mPipelineKernel->getNumOfStreamInputs();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        indices[1] = b->getInt32(FIRST_INPUT_STREAM_INDEX + i);
        Value * streamHandle = b->CreateLoad(b->CreateGEP(threadState, indices));
        auto buffer = mPipelineKernel->getInputStreamSetBuffer(i);
        buffer->setHandle(b, streamHandle);
    }
    const auto FIRST_OUTPUT_STREAM_INDEX = FIRST_INPUT_STREAM_INDEX + numOfInputs;
    const auto numOfOutputs = mPipelineKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        indices[1] = b->getInt32(FIRST_OUTPUT_STREAM_INDEX + i);
        Value * streamHandle = b->CreateLoad(b->CreateGEP(threadState, indices));
        auto buffer = mPipelineKernel->getOutputStreamSetBuffer(i);
        buffer->setHandle(b, streamHandle);
    }

    return segmentOffset;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deallocateThreadLocalSpace
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::deallocateThreadState(BuilderRef b, Value * const threadState) {
    std::vector<Value *> indices(2);
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(LOCAL_STATE_INDEX);
    deallocateThreadLocalState(b, b->CreateGEP(threadState, indices));
    destroyStateObject(b, threadState);
}

enum : unsigned {
    POP_COUNT_STRUCT_INDEX = 0
    , TERMINATION_SIGNAL_INDEX = 1
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getLocalStateType
 ** ------------------------------------------------------------------------------------------------------------- */
inline StructType * PipelineCompiler::getLocalStateType(BuilderRef b) {
    std::vector<Type *> fields(2);
    fields[POP_COUNT_STRUCT_INDEX] = getPopCountThreadLocalStateType(b);
    if (mPipelineKernel->getNumOfThreads() != 1 && mPipelineKernel->canSetTerminateSignal()) {
        fields[TERMINATION_SIGNAL_INDEX] = b->getInt1Ty();
    } else {
        fields[TERMINATION_SIGNAL_INDEX] = StructType::get(b->getContext());
    }
    return StructType::get(b->getContext(), fields);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief allocateThreadLocalState
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::allocateThreadLocalState(BuilderRef b, Value * const localState) {
    std::vector<Value *> indices(2);
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(POP_COUNT_STRUCT_INDEX);
    allocatePopCountArrays(b, b->CreateGEP(localState, indices));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setThreadLocalState
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::setThreadLocalState(BuilderRef b, Value * const localState) {
    std::vector<Value *> indices(2);
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(POP_COUNT_STRUCT_INDEX);
    assert (localState->getType()->getPointerElementType() == getLocalStateType(b));
    mPopCountState = b->CreateGEP(localState, indices);
    assert (mPopCountState->getType()->getPointerElementType() == getPopCountThreadLocalStateType(b));
    if (mPipelineKernel->canSetTerminateSignal()) {
        if (mPipelineKernel->getNumOfThreads() != 1) {
            indices[1] = b->getInt32(TERMINATION_SIGNAL_INDEX);
            mPipelineTerminated = b->CreateGEP(localState, indices);
        } else {
            mPipelineTerminated = mPipelineKernel->getTerminationSignalPtr();
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deallocateThreadLocalState
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::deallocateThreadLocalState(BuilderRef b, Value * const localState) {
    std::vector<Value *> indices(2);
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(POP_COUNT_STRUCT_INDEX);
    assert (localState->getType()->getPointerElementType() == getLocalStateType(b));
    deallocatePopCountArrays(b, b->CreateGEP(localState, indices));
    if (mPipelineTerminated && mPipelineKernel->getNumOfThreads() != 1) {
        indices[1] = b->getInt32(TERMINATION_SIGNAL_INDEX);
        Value * terminated = b->CreateLoad(b->CreateGEP(localState, indices));
        Value * terminatedPtr = mPipelineKernel->getTerminationSignalPtr();
        terminated = b->CreateOr(b->CreateLoad(terminatedPtr), terminated);
        b->CreateStore(terminated, terminatedPtr);
    }
}

}

#endif // PIPELINE_LOGIC_HPP
