#ifndef PIPELINE_LOGIC_HPP
#define PIPELINE_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

// NOTE: the following is a workaround for an LLVM bug for 32-bit VMs on 64-bit architectures.
// When calculating the address of a local stack allocated object, the size of a pointer is
// 32-bits but when performing the same GEP on a pointer returned by "malloc" or passed as a
// function argument, the size is 64-bits. More investigation is needed to determine which
// versions of LLVM are affected by this bug.

inline LLVM_READNONE bool allocateOnHeap(BuilderRef b) {
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
    // TODO: look into improving cache locality/false sharing of this struct
    b->setKernel(mPipelineKernel);
    mPipelineKernel->addInternalScalar(b->getSizeTy(), INITIAL_LOGICAL_SEGMENT_NUMBER);
    addTerminationProperties(b);
    addConsumerKernelProperties(b, PipelineInput);
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        addBufferHandlesToPipelineKernel(b, i);
        addInternalKernelProperties(b, i);
        addConsumerKernelProperties(b, i);
        addPopCountScalarsToPipelineKernel(b, i);
        addCycleCounterProperties(b, i);
    }
    b->setKernel(mPipelineKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addInternalKernelProperties(BuilderRef b, const unsigned kernelIndex) {
    const Kernel * const kernel = getKernel(kernelIndex);
    IntegerType * const sizeTy = b->getSizeTy();

    // TODO: if we've proven we do not need synchronization then we've already proven that
    // we can calculate the item count and num of strides from the input item counts.
    // With the inclusion of SynchronizationFree attributes for PipelineKernels, this is
    // no longer true and the test requires greater precision.

    if (requiresSynchronization(kernelIndex)) {
        mPipelineKernel->addInternalScalar(sizeTy, makeKernelName(kernelIndex) + LOGICAL_SEGMENT_SUFFIX);
    }

    // TODO: if an kernel I/O stream is a pipeline I/O and the kernel processes it at the
    // rate the pipeline processes it, can use the local state instead of storing the
    // item count in the kernel.
    const auto numOfInputs = in_degree(kernelIndex, mBufferGraph);
    for (unsigned i = 0; i < numOfInputs; i++) {
        const Binding & input = getInputBinding(kernelIndex, i);
        const auto prefix = makeBufferName(kernelIndex, input);
        if (input.isDeferred()) {
            mPipelineKernel->addInternalScalar(sizeTy, prefix + DEFERRED_ITEM_COUNT_SUFFIX);
        }
        mPipelineKernel->addInternalScalar(sizeTy, prefix + ITEM_COUNT_SUFFIX);
    }

    const auto numOfOutputs = out_degree(kernelIndex, mBufferGraph);
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const Binding & output = getOutputBinding(kernelIndex, i);
        const auto prefix = makeBufferName(kernelIndex, output);
        if (output.isDeferred()) {
            mPipelineKernel->addInternalScalar(sizeTy, prefix + DEFERRED_ITEM_COUNT_SUFFIX);
        }
        mPipelineKernel->addInternalScalar(sizeTy, prefix + ITEM_COUNT_SUFFIX);
    }

    if (LLVM_LIKELY(kernel->isStateful() && !kernel->hasFamilyName())) {
        // if this is a family kernel, it's handle will be passed into the kernel
        // methods rather than stored within the pipeline state
        PointerType * const handlePtrTy = kernel->getSharedStateType()->getPointerTo(0);
        mPipelineKernel->addInternalScalar(handlePtrTy, makeKernelName(kernelIndex));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateInitializeMethod(BuilderRef b) {

    std::fill(mScalarValue.begin(), mScalarValue.end(), nullptr);

    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernel = getKernel(i);
        if (kernel->isStateful()) {
            if (kernel->hasFamilyName()) {


            } else {
                Value * const handle = kernel->createInstance(b);
                b->setScalarField(makeKernelName(i), handle);
            }
        }
    }

    constructBuffers(b);
    std::vector<Value *> args;
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        setActiveKernel(b, i);
        const auto hasHandle = mKernel->isStateful() ? 1U : 0U;
        args.resize(hasHandle + in_degree(i, mScalarGraph));
        if (LLVM_LIKELY(hasHandle)) {
            args[0] = mKernel->getHandle();
        }
        b->setKernel(mPipelineKernel);
        for (const auto ce : make_iterator_range(in_edges(i, mScalarGraph))) {
            const auto j = mScalarGraph[ce] + hasHandle;
            const auto scalar = source(ce, mScalarGraph);
            args[j] = getScalar(b, scalar);
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
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::generateKernelMethod(BuilderRef b) {
    std::fill(mScalarValue.begin(), mScalarValue.end(), nullptr);
    readPipelineIOItemCounts(b);
    if (mPipelineKernel->getNumOfThreads() == 1) {
        generateSingleThreadKernelMethod(b);
    } else {
        generateMultiThreadKernelMethod(b);
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
    readThreadLocalState(b, localState);
    start(b);
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        setActiveKernel(b, i);
        acquireCurrentSegment(b);
        executeKernel(b);
        releaseCurrentSegment(b);
    }
    end(b);
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
    const unsigned threads = mPipelineKernel->getNumOfThreads() - 1;
    Type * const pthreadsTy = ArrayType::get(b->getPThreadTy(), threads);
    AllocaInst * const pthreads = b->CreateCacheAlignedAlloca(pthreadsTy);
    std::vector<Value *> threadIdPtr(threads);
    std::vector<Value *> threadState(threads);

    Value * const processThreadId = b->CreatePThreadSelf();

    for (unsigned i = 0; i < threads; ++i) {
        threadState[i] = allocateThreadState(b, processThreadId);
        threadIdPtr[i] = b->CreateGEP(pthreads, {ZERO, b->getInt32(i)});
        b->CreatePThreadCreateCall(threadIdPtr[i], nullVoidPtrVal, threadFunc, threadState[i]);
    }

    // execute the process thread
    Value * const processState = allocateThreadState(b);
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
    Value * const threadStruct = b->CreateBitCast(&*(args), processState->getType());
    setThreadState(b, threadStruct);
    // generate the pipeline logic for this thread
    mPipelineKernel->initializeScalarMap(b);
    start(b);
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        setActiveKernel(b, i);
        acquireCurrentSegment(b);
        executeKernel(b);
        releaseCurrentSegment(b);
    }
    mKernel = nullptr;
    mKernelIndex = 0;
    end(b);
    // only call pthread_exit() within spawned threads; otherwise it'll be equivalent to calling exit() within the process
    BasicBlock * const exitThread = b->CreateBasicBlock("ExitThread");
    BasicBlock * const exitFunction = b->CreateBasicBlock("ExitProcessFunction");
    b->CreateCondBr(isProcessThread(b, threadStruct), exitFunction, exitThread);
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
    std::fill(mScalarValue.begin(), mScalarValue.end(), nullptr);
    printOptionalCycleCounter(b);
    std::vector<Value *> params;
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        setActiveKernel(b, i);
        loadBufferHandles(b);
        params.clear();
        if (LLVM_LIKELY(mKernel->isStateful())) {
            params.push_back(mKernel->getHandle());
        }
        mScalarValue[i] = b->CreateCall(getFinalizeFunction(b), params);
    }
    releaseBuffers(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief acquireCurrentSegment
 *
 * Before the segment is processed, this loads the segment number of the kernel state and ensures the previous
 * segment is complete (by checking that the acquired segment number is equal to the desired segment number).
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::acquireCurrentSegment(BuilderRef b) {
    if (LLVM_LIKELY(requiresSynchronization(mKernelIndex))) {

        b->setKernel(mPipelineKernel);
        const auto prefix = makeKernelName(mKernelIndex);
        const auto serialize = codegen::DebugOptionIsSet(codegen::SerializeThreads);
        const unsigned waitingOnIdx = serialize ? LastKernel : mKernelIndex;
        const auto waitingOn = makeKernelName(waitingOnIdx);
        Value * const waitingOnPtr = b->getScalarFieldPtr(waitingOn + LOGICAL_SEGMENT_SUFFIX);
        BasicBlock * const kernelWait = b->CreateBasicBlock(prefix + "Wait", mPipelineEnd);
        b->CreateBr(kernelWait);

        b->SetInsertPoint(kernelWait);
        Value * const currentSegNo = b->CreateAtomicLoadAcquire(waitingOnPtr);
        Value * const ready = b->CreateICmpEQ(mSegNo, currentSegNo);
        BasicBlock * const kernelStart = b->CreateBasicBlock(prefix + "Start", mPipelineEnd);
        b->CreateCondBr(ready, kernelStart, kernelWait);

        b->SetInsertPoint(kernelStart);
        b->setKernel(mKernel);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief releaseCurrentSegment
 *
 * After executing the kernel, the segment number must be incremented to release the kernel for the next thread.
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::releaseCurrentSegment(BuilderRef b) {
    if (LLVM_LIKELY(requiresSynchronization(mKernelIndex))) {
        b->setKernel(mPipelineKernel);
        Value * const nextSegNo = b->CreateAdd(mSegNo, b->getSize(1));
        const auto prefix = makeKernelName(mKernelIndex);
        Value * const waitingOnPtr = b->getScalarFieldPtr(prefix + LOGICAL_SEGMENT_SUFFIX);
        b->CreateAtomicStoreRelease(nextSegNo, waitingOnPtr);
        b->setKernel(mKernel);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresSynchronization
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::requiresSynchronization(const unsigned kernelIndex) const {
    const Kernel * const kernel = getKernel(kernelIndex);
    if (kernel->hasAttribute(AttrId::SynchronizationFree)) {
        return false;
    }
    // TODO: Not quite ready yet: we need a function to calculate how many items
    // will be processed/produced by the i-th execution of this kernel based
    // strictly on the number of items produced by the (i-1)-th and the i-th
    // segment for each input to this kernel. Moreover, if we have static buffers,
    // we must statically know how many items will be consumed by any segment
    // based only only the value of i and/or the above-mentioned information.
    return true;
#if 0
    if (LLVM_LIKELY(kernel->isStateful())) {
        return true;
    }
    const auto numOfInputs = in_degree(kernelIndex, mBufferGraph);
    for (unsigned i = 0; i < numOfInputs; i++) {
        const Binding & input = getInputBinding(kernelIndex, i);
        if (!input.getRate().isFixed()) {
            return true;
        }
    }
    const auto numOfOutputs = out_degree(kernelIndex, mBufferGraph);
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const Binding & output = getOutputBinding(kernelIndex, i);
        if (!output.getRate().isFixed()) {
            return true;
        }
    }
    return false;
#endif
}

enum : unsigned {
    LOCAL_STATE_INDEX = 0
    , SHARED_STATE_INDEX = 1
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getThreadStateType
 ** ------------------------------------------------------------------------------------------------------------- */
inline StructType * PipelineCompiler::getThreadStateType(BuilderRef b) {
    FixedArray<Type *, 2> fields;
    fields[LOCAL_STATE_INDEX] = getLocalStateType(b);
    LLVMContext & C = b->getContext();
    fields[SHARED_STATE_INDEX] = StructType::get(C, mPipelineKernel->getDoSegmentFields(b));
    return StructType::get(C, fields);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructThreadState
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::allocateThreadState(BuilderRef b, Value * const threadId) {
    StructType * const threadStructType = getThreadStateType(b);
    Value * const threadState = makeStateObject(b, threadStructType);
    FixedArray<Value *, 2> indices1;
    indices1[0] = b->getInt32(0);
    indices1[1] = b->getInt32(LOCAL_STATE_INDEX);
    allocateThreadLocalState(b, b->CreateGEP(threadState, indices1), threadId);
    FixedArray<Value *, 3> indices2;
    const auto props = mPipelineKernel->getDoSegmentProperties(b);
    indices2[0] = indices1[0];
    indices2[1] = b->getInt32(SHARED_STATE_INDEX);
    const auto n = props.size();
    assert (threadStructType->getStructElementType(SHARED_STATE_INDEX)->getStructNumElements() == n);
    for (unsigned i = 0; i < n; ++i) {
        indices2[2] = b->getInt32(i);
        b->CreateStore(props[i], b->CreateGEP(threadState, indices2));
    }
    return threadState;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructThreadState
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::setThreadState(BuilderRef b, Value * threadState) {
    FixedArray<Value *, 2> indices1;
    indices1[0] = b->getInt32(0);
    indices1[1] = b->getInt32(LOCAL_STATE_INDEX);
    readThreadLocalState(b, b->CreateGEP(threadState, indices1));
    FixedArray<Value *, 3> indices2;
    indices2[0] = indices1[0];
    indices2[1] = b->getInt32(SHARED_STATE_INDEX);
    Type * const kernelStructType = threadState->getType()->getPointerElementType();
    const auto n = kernelStructType->getStructElementType(SHARED_STATE_INDEX)->getStructNumElements();
    std::vector<Value *> args(n);
    args.reserve(n);
    for (unsigned i = 0; i < n; ++i) {
        indices2[2] = b->getInt32(i);
        args[i] = b->CreateLoad(b->CreateGEP(threadState, indices2));
    }
    mPipelineKernel->setDoSegmentProperties(b, args);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deallocateThreadLocalSpace
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::deallocateThreadState(BuilderRef b, Value * const threadState) {
    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(LOCAL_STATE_INDEX);
    deallocateThreadLocalState(b, b->CreateGEP(threadState, indices));
    destroyStateObject(b, threadState);
}

enum : unsigned {
    PROCESS_THREAD_INDEX
    , POP_COUNT_STRUCT_INDEX
    , ZERO_EXTENDED_BUFFER_INDEX
    , ZERO_EXTENDED_SPACE_INDEX
    , TERMINATION_SIGNAL_INDEX
// --------------------------------
    , THREAD_LOCAL_COUNT
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getLocalStateType
 ** ------------------------------------------------------------------------------------------------------------- */
inline StructType * PipelineCompiler::getLocalStateType(BuilderRef b) {
    FixedArray<Type *, THREAD_LOCAL_COUNT> fields;
    Type * const emptyTy = StructType::get(b->getContext());
    fields[PROCESS_THREAD_INDEX] = b->getPThreadTy();
    fields[POP_COUNT_STRUCT_INDEX] = getPopCountThreadLocalStateType(b);
    fields[ZERO_EXTENDED_BUFFER_INDEX] = mHasZeroExtendedStream ? b->getVoidPtrTy() : emptyTy;
    fields[ZERO_EXTENDED_SPACE_INDEX] = mHasZeroExtendedStream ? b->getSizeTy() : emptyTy;
    if (mPipelineKernel->getNumOfThreads() != 1 && mPipelineKernel->canSetTerminateSignal()) {
        fields[TERMINATION_SIGNAL_INDEX] = b->getInt1Ty();
    } else {
        fields[TERMINATION_SIGNAL_INDEX] = emptyTy;
    }
    return StructType::get(b->getContext(), fields);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isProcessThread
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::isProcessThread(BuilderRef b, Value * const threadState) const {
    FixedArray<Value *, 3> indices;
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(LOCAL_STATE_INDEX);
    indices[2] = b->getInt32(PROCESS_THREAD_INDEX);
    Value * const ptr = b->CreateGEP(threadState, indices);
    return b->CreateIsNull(b->CreateLoad(ptr));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief allocateThreadLocalState
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::allocateThreadLocalState(BuilderRef b, Value * const localState, Value * threadId) {
    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    if (threadId == nullptr) {
        threadId = ConstantInt::getNullValue(b->getPThreadTy());
    }
    indices[1] = b->getInt32(PROCESS_THREAD_INDEX);
    b->CreateStore(threadId, b->CreateGEP(localState, indices));
    indices[1] = b->getInt32(POP_COUNT_STRUCT_INDEX);
    allocatePopCountArrays(b, b->CreateGEP(localState, indices));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readThreadLocalState
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::readThreadLocalState(BuilderRef b, Value * const localState) {
    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(POP_COUNT_STRUCT_INDEX);
    mPopCountState = b->CreateGEP(localState, indices);
    if (mHasZeroExtendedStream) {
        indices[1] = b->getInt32(ZERO_EXTENDED_BUFFER_INDEX);
        mZeroExtendBuffer = b->CreateGEP(localState, indices);
        indices[1] = b->getInt32(ZERO_EXTENDED_SPACE_INDEX);
        mZeroExtendSpace = b->CreateGEP(localState, indices);
    }
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
    assert (localState->getType()->getPointerElementType() == getLocalStateType(b));
    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(POP_COUNT_STRUCT_INDEX);
    deallocatePopCountArrays(b, b->CreateGEP(localState, indices));
    if (mHasZeroExtendedStream) {
        indices[1] = b->getInt32(ZERO_EXTENDED_BUFFER_INDEX);
        b->CreateFree(b->CreateLoad(b->CreateGEP(localState, indices)));
    }
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
