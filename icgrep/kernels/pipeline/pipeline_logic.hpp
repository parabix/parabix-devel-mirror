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

bool isFromCurrentFunction(BuilderRef b, const Value * const value) {
    if (value == nullptr) {
        return true;
    }
    if (LLVM_UNLIKELY(&b->getContext() != &value->getContext())) {
        return false;
    }
    if (isa<Constant>(value)) {
        return true;
    }
    const Function * const builderFunction = b->GetInsertBlock()->getParent();
    const Function * function = builderFunction;
    if (isa<Argument>(value)) {
        function = cast<Argument>(value)->getParent();
    } else if (isa<Instruction>(value)) {
        function = cast<Instruction>(value)->getParent()->getParent();
    }
    return (builderFunction == function);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateImplicitKernels
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::generateImplicitKernels(BuilderRef b) {
    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        Kernel * const kernel = const_cast<Kernel *>(getKernel(i));
        if (LLVM_LIKELY(kernel->getModule())) continue;
        kernel->setModule(mPipelineKernel->getModule());
        if (kernel->getInitializeFunction(b)) {
            kernel->prepareCachedKernel(b);
        } else {
            kernel->prepareKernel(b);
            kernel->generateKernel(b);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addPipelineKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addPipelineKernelProperties(BuilderRef b) {
    // TODO: look into improving cache locality/false sharing of this struct

    // TODO: create a non-persistent / pass through input scalar type to allow the
    // pipeline to pass an input scalar to a kernel rather than recording it needlessly?
    // Non-family kernels can be contained within the shared state but family ones
    // must be allocated dynamically.

    if (!isOpenSystem()) {
        mPipelineKernel->addInternalScalar(b->getSizeTy(), CURRENT_LOGICAL_SEGMENT_NUMBER);
    }
    Type * const localStateType = getThreadLocalStateType(b);
    if (localStateType->isEmptyTy()) {
        mHasThreadLocalPipelineState = false;
    } else {
        mPipelineKernel->addThreadLocalScalar(localStateType, PIPELINE_THREAD_LOCAL_STATE);
        mHasThreadLocalPipelineState = true;
    }
    addTerminationProperties(b);
    addConsumerKernelProperties(b, PipelineInput);
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        addBufferHandlesToPipelineKernel(b, i);
        addInternalKernelProperties(b, i);
        addConsumerKernelProperties(b, i);
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
    // With the inclusion of InternallySynchronized attributes for PipelineKernels, this is
    // no longer true and the test requires greater precision.

    if (requiresSynchronization(kernelIndex)) {
        const auto prefix = makeKernelName(kernelIndex);
        mPipelineKernel->addInternalScalar(sizeTy, prefix + LOGICAL_SEGMENT_SUFFIX);
        if (kernel->hasAttribute(AttrId::InternallySynchronized)) {
            mPipelineKernel->addInternalScalar(sizeTy, prefix + ITERATION_COUNT_SUFFIX);
        }
    }

    // TODO: if an kernel I/O stream is a pipeline I/O and the kernel processes it at the
    // rate the pipeline processes it, can use the local state instead of storing the
    // item count in the kernel.
    const auto numOfInputs = in_degree(kernelIndex, mBufferGraph);
    for (unsigned i = 0; i < numOfInputs; i++) {
        const Binding & input = getInputBinding(kernelIndex, i);
        const auto prefix = makeBufferName(kernelIndex, StreamPort{PortType::Input, i});
        if (input.isDeferred()) {
            mPipelineKernel->addInternalScalar(sizeTy, prefix + DEFERRED_ITEM_COUNT_SUFFIX);
        }
        mPipelineKernel->addInternalScalar(sizeTy, prefix + ITEM_COUNT_SUFFIX);
    }

    const auto numOfOutputs = out_degree(kernelIndex, mBufferGraph);
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const Binding & output = getOutputBinding(kernelIndex, i);
        const auto prefix = makeBufferName(kernelIndex, StreamPort{PortType::Output, i});
        if (output.isDeferred()) {
            mPipelineKernel->addInternalScalar(sizeTy, prefix + DEFERRED_ITEM_COUNT_SUFFIX);
        }
        mPipelineKernel->addInternalScalar(sizeTy, prefix + ITEM_COUNT_SUFFIX);
    }

    if (kernel->hasThreadLocal()) {
        const auto prefix = makeKernelName(kernelIndex);
        mPipelineKernel->addThreadLocalScalar(kernel->getThreadLocalStateType(), prefix + KERNEL_THREAD_LOCAL_SUFFIX);
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

    ArgVec args;
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        setActiveKernel(b, i);
        args.clear();
        if (LLVM_LIKELY(mKernel->isStateful())) {
            args.push_back(mKernel->getHandle());
        }
        b->setKernel(mPipelineKernel);
        #ifndef NDEBUG
        unsigned expected = 0;
        #endif
        for (const auto & e : make_iterator_range(in_edges(i, mScalarGraph))) {
            assert (mScalarGraph[e].Type == PortType::Input);
            assert (expected++ == mScalarGraph[e].Number);
            const auto scalar = source(e, mScalarGraph);
            args.push_back(getScalar(b, scalar));
        }
        b->setKernel(mKernel);
        Value * const f = getInitializeFunction(b);
        if (LLVM_UNLIKELY(f == nullptr)) {
            report_fatal_error(mKernel->getName() + " does not have an initialize method");
        }
        Value * const terminatedOnInit = b->CreateCall(f, args);

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
    if (mPipelineKernel->hasThreadLocal()) {
        bindCompilerVariablesToThreadLocalState(b, mPipelineKernel->getThreadLocalHandle());
    }
    start(b);
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        startCycleCounter(b, CycleCounter::INITIAL);
        setActiveKernel(b, i);
        acquireCurrentSegment(b);
        executeKernel(b);
        releaseCurrentSegment(b);
        updateCycleCounter(b, CycleCounter::INITIAL, CycleCounter::FINAL);
    }
    end(b);
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
    auto threadStructArg = threadFunc->arg_begin();
    threadStructArg->setName("threadStruct");

    Value * const initialSharedState = mPipelineKernel->getHandle();
    Value * const initialThreadLocal = mPipelineKernel->getThreadLocalHandle();
    Value * const initialTerminationSignalPtr = mPipelineKernel->getTerminationSignalPtr();
    Value * const initialExternalSegNo = mPipelineKernel->getExternalSegNo();

    // -------------------------------------------------------------------------------------------------------------------------
    // MAKE PIPELINE DRIVER CONTINUED
    // -------------------------------------------------------------------------------------------------------------------------

    // use the process thread to handle the initial segment function after spawning
    // (n - 1) threads to handle the subsequent offsets
    const unsigned threads = mPipelineKernel->getNumOfThreads() - 1;
    Type * const pthreadsTy = ArrayType::get(b->getPThreadTy(), threads);
    AllocaInst * const pthreads = b->CreateCacheAlignedAlloca(pthreadsTy);
    SmallVector<Value *, 8> threadIdPtr(threads);
    SmallVector<Value *, 8> threadState(threads);
    SmallVector<Value *, 8> threadLocal(threads);

    Value * const processThreadId = b->CreatePThreadSelf();
    for (unsigned i = 0; i < threads; ++i) {

        if (mPipelineKernel->hasThreadLocal()) {
            threadLocal[i] = mPipelineKernel->createThreadLocalInstance(b);
            SmallVector<Value *, 2> args;
            if (LLVM_UNLIKELY(mPipelineKernel->isStateful())) {
                args.push_back(initialSharedState);
            }
            args.push_back(threadLocal[i]);
            mPipelineKernel->initializeThreadLocalInstance(b, args);
            assert (isFromCurrentFunction(b, threadLocal[i]));
            mPipelineKernel->setThreadLocalHandle(threadLocal[i]);
        }

        threadState[i] = allocateThreadState(b, processThreadId);
        FixedArray<Value *, 2> indices;
        indices[0] = ZERO;
        indices[1] = b->getInt32(i);
        threadIdPtr[i] = b->CreateGEP(pthreads, indices);
        b->CreatePThreadCreateCall(threadIdPtr[i], nullVoidPtrVal, threadFunc, threadState[i]);
    }

    // execute the process thread
    assert (isFromCurrentFunction(b, initialThreadLocal));
    mPipelineKernel->setThreadLocalHandle(initialThreadLocal);
    Value * const processState = allocateThreadState(b, Constant::getNullValue(b->getPThreadTy()));
    b->CreateCall(threadFunc, b->CreatePointerCast(processState, voidPtrTy));

    Value * terminated = readTerminationSignalFromLocalState(b, initialThreadLocal);

    // wait for all other threads to complete
    AllocaInst * const status = b->CreateAlloca(voidPtrTy);
    for (unsigned i = 0; i < threads; ++i) {
        Value * threadId = b->CreateLoad(threadIdPtr[i]);
        b->CreatePThreadJoinCall(threadId, status);
        SmallVector<Value *, 2> args;
        if (LLVM_LIKELY(mPipelineKernel->isStateful())) {
            args.push_back(initialSharedState);
        }
        args.push_back(threadLocal[i]);
        terminated = b->CreateOr(terminated, readTerminationSignalFromLocalState(b, threadLocal[i]));
        mPipelineKernel->finalizeThreadLocalInstance(b, args);
        destroyStateObject(b, threadState[i]);
    }

    b->CreateStore(terminated, initialTerminationSignalPtr);

    // store where we'll resume compiling the DoSegment method
    const auto resumePoint = b->saveIP();

    // -------------------------------------------------------------------------------------------------------------------------
    // MAKE PIPELINE THREAD
    // -------------------------------------------------------------------------------------------------------------------------
    b->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", threadFunc));
    Value * const threadStruct = b->CreateBitCast(&*threadStructArg, processState->getType());
    assert (isFromCurrentFunction(b, threadStruct));
    readThreadState(b, threadStruct);
    assert (isFromCurrentFunction(b, mPipelineKernel->getHandle()));
    assert (isFromCurrentFunction(b, mPipelineKernel->getThreadLocalHandle()));
    // generate the pipeline logic for this thread
    start(b);
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        startCycleCounter(b, CycleCounter::INITIAL);
        setActiveKernel(b, i);
        acquireCurrentSegment(b);
        executeKernel(b);
        releaseCurrentSegment(b);
        updateCycleCounter(b, CycleCounter::INITIAL, CycleCounter::FINAL);
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
    assert (isFromCurrentFunction(b, processState));
    assert (isFromCurrentFunction(b, initialSharedState));
    // TODO: the pipeline kernel scalar state is invalid after leaving this function. Best bet would be to copy the
    // scalarmap and replace it.
    destroyStateObject(b, processState);
    mPipelineKernel->setHandle(b, initialSharedState);
    mPipelineKernel->setThreadLocalHandle(initialThreadLocal);
    mPipelineKernel->setTerminationSignalPtr(initialTerminationSignalPtr);
    mPipelineKernel->setExternalSegNo(initialExternalSegNo);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateFinalizeMethod(BuilderRef b) {
    std::fill(mScalarValue.begin(), mScalarValue.end(), nullptr);
    printOptionalCycleCounter(b);
    printOptionalBlockingIOStatistics(b);
    printOptionalBufferExpansionHistory(b);
    SmallVector<Value *, 16> params;
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

enum : unsigned {
    PIPELINE_STATE_INDEX
    , PROCESS_THREAD_INDEX
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getThreadStateType
 ** ------------------------------------------------------------------------------------------------------------- */
inline StructType * PipelineCompiler::getThreadStateType(BuilderRef b) const {
    FixedArray<Type *, 2> fields;
    LLVMContext & C = b->getContext();
    fields[PIPELINE_STATE_INDEX] = StructType::get(C, mPipelineKernel->getDoSegmentFields(b));
    fields[PROCESS_THREAD_INDEX] = b->getPThreadTy();
    return StructType::get(C, fields);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructThreadState
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::allocateThreadState(BuilderRef b, Value * const threadId) {
    StructType * const threadStructType = getThreadStateType(b);
    Value * const threadState = makeStateObject(b, threadStructType);

    const auto props = mPipelineKernel->getDoSegmentProperties(b);
    const auto n = props.size();
    assert (threadStructType->getStructElementType(PIPELINE_STATE_INDEX)->getStructNumElements() == n);

    FixedArray<Value *, 3> indices3;
    indices3[0] = b->getInt32(0);
    indices3[1] = b->getInt32(PIPELINE_STATE_INDEX);
    for (unsigned i = 0; i < n; ++i) {
        indices3[2] = b->getInt32(i);
        b->CreateStore(props[i], b->CreateGEP(threadState, indices3));
    }
    FixedArray<Value *, 2> indices2;
    indices2[0] = b->getInt32(0);
    indices2[1] = b->getInt32(PROCESS_THREAD_INDEX);
    b->CreateStore(threadId, b->CreateGEP(threadState, indices2));

    return threadState;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructThreadState
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::readThreadState(BuilderRef b, Value * threadState) {
    FixedArray<Value *, 3> indices;
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(PIPELINE_STATE_INDEX);
    Type * const kernelStructType = threadState->getType()->getPointerElementType();
    const auto n = kernelStructType->getStructElementType(PIPELINE_STATE_INDEX)->getStructNumElements();
    SmallVector<Value *, 16> args(n);
    args.reserve(n);
    for (unsigned i = 0; i < n; ++i) {
        indices[2] = b->getInt32(i);
        args[i] = b->CreateLoad(b->CreateGEP(threadState, indices));
    }
    mPipelineKernel->setDoSegmentProperties(b, args);
    if (mPipelineKernel->hasThreadLocal()) {
        bindCompilerVariablesToThreadLocalState(b, mPipelineKernel->getThreadLocalHandle());
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isProcessThread
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::isProcessThread(BuilderRef b, Value * const threadState) const {
    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(PROCESS_THREAD_INDEX);
    Value * const ptr = b->CreateGEP(threadState, indices);
    return b->CreateIsNull(b->CreateLoad(ptr));
}

enum : unsigned {
    POP_COUNT_STRUCT_INDEX
    , ZERO_EXTENDED_BUFFER_INDEX
    , ZERO_EXTENDED_SPACE_INDEX
    , TERMINATION_SIGNAL_INDEX
// --------------------------------
    , THREAD_LOCAL_COUNT
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getThreadLocalStateType
 ** ------------------------------------------------------------------------------------------------------------- */
inline StructType * PipelineCompiler::getThreadLocalStateType(BuilderRef b) {
    FixedArray<Type *, THREAD_LOCAL_COUNT> fields;
    Type * const emptyTy = StructType::get(b->getContext());
    fields[POP_COUNT_STRUCT_INDEX] = emptyTy; // getPopCountThreadLocalStateType(b);
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
 * @brief bindCompilerVariablesToThreadLocalState
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::bindCompilerVariablesToThreadLocalState(BuilderRef b, Value * const localState) {
    FixedArray<Value *, 3> indices;
    indices[0] = b->getInt32(0);
    indices[1] = indices[0];
    if (mHasZeroExtendedStream) {
        indices[2] = b->getInt32(ZERO_EXTENDED_BUFFER_INDEX);
        mZeroExtendBuffer = b->CreateGEP(localState, indices);
        indices[2] = b->getInt32(ZERO_EXTENDED_SPACE_INDEX);
        mZeroExtendSpace = b->CreateGEP(localState, indices);
    }
    if (mPipelineKernel->canSetTerminateSignal()) {
        if (mPipelineKernel->getNumOfThreads() != 1) {
            indices[2] = b->getInt32(TERMINATION_SIGNAL_INDEX);
            mPipelineTerminated = b->CreateGEP(localState, indices);
        } else {
            mPipelineTerminated = mPipelineKernel->getTerminationSignalPtr();
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateInitializeThreadLocalMethod(BuilderRef b) {
    if (mPipelineKernel->hasThreadLocal()) {
        for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
            const Kernel * const kernel = getKernel(i);
            if (kernel->hasThreadLocal()) {
                setActiveKernel(b, i);
                SmallVector<Value *, 2> args;
                if (LLVM_LIKELY(kernel->isStateful())) {
                    args.push_back(kernel->getHandle());
                }
                args.push_back(mPipelineKernel->getScalarValuePtr(b.get(), makeKernelName(i) + KERNEL_THREAD_LOCAL_SUFFIX));
                Value * const f = getInitializeThreadLocalFunction(b);
                if (LLVM_UNLIKELY(f == nullptr)) {
                    report_fatal_error(mKernel->getName() + " does not have an initialize method for its threadlocal state");
                }
                b->CreateCall(f, args);
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateFinalizeThreadLocalMethod(BuilderRef b) {
    if (mPipelineKernel->hasThreadLocal()) {
        if (mHasThreadLocalPipelineState) {
            Value * const localState = mPipelineKernel->getScalarValuePtr(b.get(), PIPELINE_THREAD_LOCAL_STATE);
            FixedArray<Value *, 2> indices;
            indices[0] = b->getInt32(0);
            if (mHasZeroExtendedStream) {
                indices[1] = b->getInt32(ZERO_EXTENDED_BUFFER_INDEX);
                b->CreateFree(b->CreateLoad(b->CreateGEP(localState, indices)));
            }
        }
        for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
            const Kernel * const kernel = getKernel(i);
            if (kernel->hasThreadLocal()) {
                setActiveKernel(b, i);
                SmallVector<Value *, 2> args;
                if (LLVM_LIKELY(kernel->isStateful())) {
                    args.push_back(kernel->getHandle());
                }
                args.push_back(mPipelineKernel->getScalarValuePtr(b.get(), makeKernelName(i) + KERNEL_THREAD_LOCAL_SUFFIX));
                Value * const f = getFinalizeThreadLocalFunction(b);
                if (LLVM_UNLIKELY(f == nullptr)) {
                    report_fatal_error(mKernel->getName() + " does not to have an finalize method for its threadlocal state");
                }
                b->CreateCall(f, args);
            }
        }
        // Since all of the nested kernels thread local state is contained within
        // this pipeline thread's thread local state, freeing the pipeline's will
        // also free the inner kernels.
        b->CreateFree(mPipelineKernel->getThreadLocalHandle());
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readTerminationSignalFromLocalState
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::readTerminationSignalFromLocalState(BuilderRef b, Value * const localState) const {
    // TODO: generalize a OR/ADD/etc "combination" mechanism for thread-local to output scalars?
    if (mPipelineTerminated) {
        FixedArray<Value *, 2> indices;
        indices[0] = b->getInt32(0);
        indices[1] = b->getInt32(TERMINATION_SIGNAL_INDEX);
        return b->CreateLoad(b->CreateGEP(localState, indices));
    } else {
        return b->getFalse();
    }
}

}

#endif // PIPELINE_LOGIC_HPP
