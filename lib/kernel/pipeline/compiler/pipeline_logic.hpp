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
 * @brief generateImplicitKernels
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::generateImplicitKernels(BuilderRef b) {
    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        Kernel * const kernel = const_cast<Kernel *>(getKernel(i));
        if (LLVM_LIKELY(kernel->isGenerated())) continue;
        if (kernel->getInitializeFunction(b, false)) {
            kernel->loadCachedKernel(b);
        } else {
            kernel->setModule(mTarget->getModule());
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

    if (LLVM_LIKELY(!ExternallySynchronized)) {
        mTarget->addInternalScalar(b->getSizeTy(), CURRENT_LOGICAL_SEGMENT_NUMBER);
    }
    Type * const localStateType = getThreadLocalStateType(b);
    if (localStateType->isEmptyTy()) {
        mHasThreadLocalPipelineState = false;
    } else {
        mTarget->addThreadLocalScalar(localStateType, PIPELINE_THREAD_LOCAL_STATE);
        mHasThreadLocalPipelineState = true;
    }

    addBufferHandlesToPipelineKernel(b, PipelineInput);
    addConsumerKernelProperties(b, PipelineInput);
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        addBufferHandlesToPipelineKernel(b, i);
        addTerminationProperties(b, i);
        addInternalKernelProperties(b, i);
        addConsumerKernelProperties(b, i);
        addCycleCounterProperties(b, i);
        addProducedItemCountDeltaProperties(b, i);
        addUnconsumedItemCountProperties(b, i);
        addFamilyKernelProperties(b, i);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addInternalKernelProperties(BuilderRef b, const unsigned kernelIndex) {
    const Kernel * const kernel = getKernel(kernelIndex);
    IntegerType * const sizeTy = b->getSizeTy();

    const auto internallySynchronized = kernel->hasAttribute(AttrId::InternallySynchronized);

    // TODO: if we've proven we do not need synchronization then we've already proven that
    // we can calculate the item count and num of strides from the input item counts.
    // With the inclusion of InternallySynchronized attributes for PipelineKernels, this is
    // no longer true and the test requires greater precision.




    const auto prefix = makeKernelName(kernelIndex);
    if (internallySynchronized) {
        mTarget->addInternalScalar(sizeTy, prefix + ITEM_COUNT_READ_GUARD_SUFFIX);
        mTarget->addInternalScalar(sizeTy, prefix + CURRENT_LOGICAL_SEGMENT_NUMBER);
    } else {
        mTarget->addInternalScalar(sizeTy, prefix + LOGICAL_SEGMENT_SUFFIX);
    }

    if (LLVM_LIKELY(kernel->isStateful())) {
        PointerType * sharedStateTy = nullptr;
        if (LLVM_UNLIKELY(kernel->externallyInitialized())) {
            sharedStateTy = b->getVoidPtrTy();
        } else {
            sharedStateTy = kernel->getSharedStateType()->getPointerTo(0);
        }
        mTarget->addInternalScalar(sharedStateTy, prefix);
    }

    if (kernel->hasThreadLocal()) {
        // we cannot statically allocate a "family" thread local object.
        PointerType * localStateTy = nullptr;
        if (LLVM_UNLIKELY(kernel->externallyInitialized())) {
            localStateTy = b->getVoidPtrTy();
        } else {
            localStateTy = kernel->getThreadLocalStateType()->getPointerTo(0);
        }
        mTarget->addThreadLocalScalar(localStateTy, prefix + KERNEL_THREAD_LOCAL_SUFFIX);
    }

    /* mPortEvaluationOrder = */ determineEvaluationOrderOfKernelIO(kernelIndex);

    const auto numOfInputs = in_degree(kernelIndex, mBufferGraph);

    for (const auto i : mPortEvaluationOrder) {
        StreamSetPort port{};
        BufferGraph::edge_descriptor e;
        if (i < numOfInputs) {
            port.Type = PortType::Input;
            port.Number = i;
            e = getInput(kernelIndex, port.Number);
        } else {
            port.Type = PortType::Output;
            port.Number = i - numOfInputs;
            e = getOutput(kernelIndex, port.Number);
        }
        const auto prefix = makeBufferName(kernelIndex, port);
        const BufferRateData & br = mBufferGraph[e];
        const Binding & binding = br.Binding;
        if (LLVM_UNLIKELY(binding.isDeferred())) {
            mTarget->addInternalScalar(sizeTy, prefix + DEFERRED_ITEM_COUNT_SUFFIX);
        }
        unsigned buffer{};
        if (i < numOfInputs) {
            buffer = source(e, mBufferGraph);
        } else {
            buffer = target(e, mBufferGraph);
        }
        const BufferNode & bn = mBufferGraph[buffer];
        if (LLVM_LIKELY(bn.isInternal() || bn.isOwned())) {
            mTarget->addInternalScalar(sizeTy, prefix + ITEM_COUNT_SUFFIX);
        } else {
            mTarget->addNonPersistentScalar(sizeTy, prefix + ITEM_COUNT_SUFFIX);
        }
    }

    if (internallySynchronized) {
        mTarget->addInternalScalar(sizeTy, prefix + LOGICAL_SEGMENT_SUFFIX);
    }

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers))) {
        for (const auto e : make_iterator_range(out_edges(kernelIndex, mBufferGraph))) {
            const auto bufferVertex = target(e, mBufferGraph);
            const BufferNode & bn = mBufferGraph[bufferVertex];
            if (isa<DynamicBuffer>(bn.Buffer)) {
                const BufferRateData & rd = mBufferGraph[e];
                const auto prefix = makeBufferName(kernelIndex, rd.Port);
                LLVMContext & C = b->getContext();
                const auto numOfConsumers = out_degree(bufferVertex, mConsumerGraph);

                // segment num  0
                // new capacity 1
                // produced item count 2
                // consumer processed item count [3,n)
                Type * const traceStructTy = ArrayType::get(sizeTy, numOfConsumers + 3);

                FixedArray<Type *, 2> traceStruct;
                traceStruct[0] = traceStructTy->getPointerTo(); // pointer to trace log
                traceStruct[1] = sizeTy; // length of trace log
                mTarget->addInternalScalar(StructType::get(C, traceStruct),
                                                   prefix + STATISTICS_BUFFER_EXPANSION_SUFFIX);
            }
        }
    }

    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceStridesPerSegment))) {
        LLVMContext & C = b->getContext();
        FixedArray<Type *, 2> recordStruct;
        recordStruct[0] = sizeTy; // segment num
        recordStruct[1] = sizeTy; // # of strides
        Type * const recordStructTy = StructType::get(C, recordStruct);

        FixedArray<Type *, 4> traceStruct;
        traceStruct[0] = sizeTy; // last num of strides (to avoid unnecessary loads of the trace
                                 // log and simplify the logic for first stride)
        traceStruct[1] = recordStructTy->getPointerTo(); // pointer to trace log
        traceStruct[2] = sizeTy; // trace length
        traceStruct[3] = sizeTy; // trace capacity (for realloc)

        const auto prefix = makeKernelName(kernelIndex);

        mTarget->addInternalScalar(StructType::get(C, traceStruct),
                                           prefix + STATISTICS_STRIDES_PER_SEGMENT_SUFFIX);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateInitializeMethod(BuilderRef b) {

    // TODO: if we detect a fatal error at init, we should not execute
    // the pipeline loop.

    std::fill(mScalarValue.begin(), mScalarValue.end(), nullptr);

    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernel = getKernel(i);
        if (LLVM_LIKELY(kernel->isStateful() && !kernel->externallyInitialized())) {
            Value * const handle = kernel->createInstance(b);
            b->setScalarField(makeKernelName(i), handle);
        }
    }

    allocateOwnedBuffers(b);    
    initializeBufferExpansionHistory(b);

    Constant * const unterminated = getTerminationSignal(b, TerminationSignal::None);

    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {

        // Family kernels must be initialized in the "main" method.
        const Kernel * kernel = getKernel(i);
        if (LLVM_UNLIKELY(kernel->externallyInitialized())) {
            continue;
        }

        setActiveKernel(b, i);
        initializeStridesPerSegment(b);
        ArgVec args;

        if (LLVM_LIKELY(mKernel->isStateful())) {
            args.push_back(mKernelHandle);
        }
        #ifndef NDEBUG
        unsigned expected = 0;
        #endif
        for (const auto e : make_iterator_range(in_edges(i, mScalarGraph))) {
            assert (mScalarGraph[e].Type == PortType::Input);
            assert (expected++ == mScalarGraph[e].Number);
            const auto scalar = source(e, mScalarGraph);
            Value * const scalarValue = getScalar(b, scalar);
            args.push_back(scalarValue);
        }
        Value * const f = getKernelInitializeFunction(b);
        if (LLVM_UNLIKELY(f == nullptr)) {
            report_fatal_error(mKernel->getName() + " does not have an initialize method");
        }

        Value * const signal = b->CreateCall(f, args);
        Value * const terminatedOnInit = b->CreateICmpNE(signal, unterminated);
        const auto prefix = makeKernelName(mKernelIndex);
        BasicBlock * const kernelTerminated = b->CreateBasicBlock(prefix + "_terminatedOnInit");
        BasicBlock * const kernelExit = b->CreateBasicBlock(prefix + "_exit");
        b->CreateUnlikelyCondBr(terminatedOnInit, kernelTerminated, kernelExit);

        b->SetInsertPoint(kernelTerminated);
        writeTerminationSignal(b, getTerminationSignal(b, TerminationSignal::Aborted));
        b->CreateBr(kernelExit);

        b->SetInsertPoint(kernelExit);
    }
    resetInternalBufferHandles();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::generateKernelMethod(BuilderRef b) {
    std::fill(mScalarValue.begin(), mScalarValue.end(), nullptr);
    readPipelineIOItemCounts(b);
    if (mNumOfThreads == 1) {
        generateSingleThreadKernelMethod(b);
    } else {
        generateMultiThreadKernelMethod(b);
    }
    resetInternalBufferHandles();
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
        Value * const available = getAvailableInputItems(inputPort);
        mLocallyAvailableItems[getBufferIndex(buffer)] = available;
        mConsumerGraph[buffer].Consumed = available;

        Value * const inPtr = getProcessedInputItemsPtr(inputPort);
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

        Value * outPtr = getProducedOutputItemsPtr(outputPort);
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
 * @brief generateSingleThreadKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateSingleThreadKernelMethod(BuilderRef b) {
    if (mTarget->hasThreadLocal()) {
        bindCompilerVariablesToThreadLocalState(b);
    }
    start(b);
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        startCycleCounter(b, CycleCounter::INITIAL);
        setActiveKernel(b, i);
        executeKernel(b);
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
    const auto threadName = mTarget->getName() + "_DoSegmentThread";
    Function * const threadFunc = Function::Create(threadFuncType, Function::InternalLinkage, threadName, m);
    threadFunc->setCallingConv(CallingConv::C);
    auto threadStructArg = threadFunc->arg_begin();
    threadStructArg->setName("threadStruct");

    Value * const initialSharedState = getHandle();
    assert (initialSharedState);
    Value * const initialThreadLocal = getThreadLocalHandle();
    Value * const initialTerminationSignalPtr = getTerminationSignalPtr();
    assert (initialThreadLocal || initialTerminationSignalPtr == nullptr);
    assert (initialTerminationSignalPtr || mCurrentThreadTerminationSignalPtr == nullptr);
    Value * const initialExternalSegNo = getExternalSegNo();

    // -------------------------------------------------------------------------------------------------------------------------
    // MAKE PIPELINE DRIVER CONTINUED
    // -------------------------------------------------------------------------------------------------------------------------

    // use the process thread to handle the initial segment function after spawning
    // (n - 1) threads to handle the subsequent offsets
    const auto additionalThreads = mNumOfThreads - 1U;
    Type * const pthreadsTy = ArrayType::get(b->getPThreadTy(), additionalThreads);
    AllocaInst * const pthreads = b->CreateCacheAlignedAlloca(pthreadsTy);
    SmallVector<Value *, 8> threadIdPtr(additionalThreads);
    SmallVector<Value *, 8> threadState(additionalThreads);
    SmallVector<Value *, 8> threadLocal(additionalThreads);

    Value * const processThreadId = b->CreatePThreadSelf();
    for (unsigned i = 0; i < additionalThreads; ++i) {
        if (mTarget->hasThreadLocal()) {
            threadLocal[i] = mTarget->initializeThreadLocalInstance(b, initialSharedState);
            assert (isFromCurrentFunction(b, threadLocal[i]));
        }
        threadState[i] = allocateThreadState(b, processThreadId);
        FixedArray<Value *, 2> indices;
        indices[0] = ZERO;
        indices[1] = b->getInt32(i);
        threadIdPtr[i] = b->CreateInBoundsGEP(pthreads, indices);
        b->CreatePThreadCreateCall(threadIdPtr[i], nullVoidPtrVal, threadFunc, threadState[i]);
    }

    // execute the process thread
    assert (isFromCurrentFunction(b, initialThreadLocal));
    Value * const processState = allocateThreadState(b, Constant::getNullValue(b->getPThreadTy()));
    b->CreateCall(threadFunc, b->CreatePointerCast(processState, voidPtrTy));

    // wait for all other threads to complete
    AllocaInst * const status = b->CreateAlloca(voidPtrTy);
    for (unsigned i = 0; i < additionalThreads; ++i) {
        Value * threadId = b->CreateLoad(threadIdPtr[i]);
        b->CreatePThreadJoinCall(threadId, status);
    }

    if (mTarget->hasThreadLocal()) {
        Value * terminated = readTerminationSignalFromLocalState(b, initialThreadLocal);
        for (unsigned i = 0; i < additionalThreads; ++i) {
            SmallVector<Value *, 2> args;
            if (LLVM_LIKELY(mTarget->isStateful())) {
                args.push_back(initialSharedState);
            }
            args.push_back(threadLocal[i]);
            Value * const terminatedSignal = readTerminationSignalFromLocalState(b, threadLocal[i]);
            terminated = b->CreateUMax(terminated, terminatedSignal);
            mTarget->finalizeThreadLocalInstance(b, args);
            destroyStateObject(b, threadState[i]);
        }
        if (LLVM_LIKELY(terminated && initialTerminationSignalPtr)) {
            b->CreateStore(terminated, initialTerminationSignalPtr);
        }
    }

    // store where we'll resume compiling the DoSegment method
    const auto resumePoint = b->saveIP();

    // -------------------------------------------------------------------------------------------------------------------------
    // MAKE PIPELINE THREAD
    // -------------------------------------------------------------------------------------------------------------------------
    b->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", threadFunc));
    Value * const threadStruct = b->CreatePointerCast(threadStructArg, processState->getType());
    assert (isFromCurrentFunction(b, threadStruct));
    readThreadState(b, threadStruct);
    assert (isFromCurrentFunction(b, getHandle()));
    assert (isFromCurrentFunction(b, getThreadLocalHandle()));

    // generate the pipeline logic for this thread
    start(b);
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {        
        setActiveKernel(b, i);
        startCycleCounter(b, CycleCounter::INITIAL);
        executeKernel(b);
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
    setHandle(initialSharedState);
    setThreadLocalHandle(initialThreadLocal);
    setTerminationSignalPtr(initialTerminationSignalPtr);
    setExternalSegNo(initialExternalSegNo);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateFinalizeMethod(BuilderRef b) {
    std::fill(mScalarValue.begin(), mScalarValue.end(), nullptr);
    // calculate the last segment # used by any kernel in case any reports require it.
    mSegNo = nullptr;
    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        Value * const segNo = b->getScalarField(makeKernelName(i) + LOGICAL_SEGMENT_SUFFIX);
        mSegNo = b->CreateUMax(mSegNo, segNo);
    }    
    loadInternalStreamSetHandles(b);
    printOptionalCycleCounter(b);
    printOptionalBlockingIOStatistics(b);
    printOptionalBlockedIOPerSegment(b);
    printOptionalBufferExpansionHistory(b);
    printOptionalStridesPerSegment(b);
    printProducedItemCountDeltas(b);
    printUnconsumedItemCounts(b);
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        setActiveKernel(b, i);        
        SmallVector<Value *, 1> params;
        if (LLVM_LIKELY(mKernel->isStateful())) {
            params.push_back(mKernelHandle);
        }
        mScalarValue[i] = b->CreateCall(getKernelFinalizeFunction(b), params);
    }
    releaseOwnedBuffers(b);
    resetInternalBufferHandles();
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
    fields[PIPELINE_STATE_INDEX] = StructType::get(C, mTarget->getDoSegmentFields(b));
    fields[PROCESS_THREAD_INDEX] = b->getPThreadTy();
    return StructType::get(C, fields);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructThreadState
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::allocateThreadState(BuilderRef b, Value * const threadId) {
    StructType * const threadStructType = getThreadStateType(b);
    Value * const threadState = makeStateObject(b, threadStructType);

    const auto props = getDoSegmentProperties(b);
    const auto n = props.size();
    assert (threadStructType->getStructElementType(PIPELINE_STATE_INDEX)->getStructNumElements() == n);

    FixedArray<Value *, 3> indices3;
    indices3[0] = b->getInt32(0);
    indices3[1] = b->getInt32(PIPELINE_STATE_INDEX);
    for (unsigned i = 0; i < n; ++i) {
        indices3[2] = b->getInt32(i);
        b->CreateStore(props[i], b->CreateInBoundsGEP(threadState, indices3));
    }
    FixedArray<Value *, 2> indices2;
    indices2[0] = b->getInt32(0);
    indices2[1] = b->getInt32(PROCESS_THREAD_INDEX);
    b->CreateStore(threadId, b->CreateInBoundsGEP(threadState, indices2));

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
        args[i] = b->CreateLoad(b->CreateInBoundsGEP(threadState, indices));
    }
    setDoSegmentProperties(b, args);
    if (mTarget->hasThreadLocal()) {
        bindCompilerVariablesToThreadLocalState(b);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isProcessThread
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::isProcessThread(BuilderRef b, Value * const threadState) const {
    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(PROCESS_THREAD_INDEX);
    Value * const ptr = b->CreateInBoundsGEP(threadState, indices);
    return b->CreateIsNull(b->CreateLoad(ptr));
}

enum : unsigned {
    ZERO_EXTENDED_BUFFER_INDEX
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
    if (mHasZeroExtendedStream) {
        fields[ZERO_EXTENDED_BUFFER_INDEX] = b->getVoidPtrTy();
        fields[ZERO_EXTENDED_SPACE_INDEX] = b->getSizeTy();
    } else {
        fields[ZERO_EXTENDED_BUFFER_INDEX] = emptyTy;
        fields[ZERO_EXTENDED_SPACE_INDEX] = emptyTy;
    }
    if (LLVM_LIKELY(mNumOfThreads > 1 && canSetTerminateSignal())) {
        fields[TERMINATION_SIGNAL_INDEX] = b->getSizeTy();
    } else {
        fields[TERMINATION_SIGNAL_INDEX] = emptyTy;
    }
    return StructType::get(b->getContext(), fields);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief bindCompilerVariablesToThreadLocalState
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::bindCompilerVariablesToThreadLocalState(BuilderRef b) {
    FixedArray<Value *, 3> indices;
    Constant * const ZERO = b->getInt32(0);
    indices[0] = ZERO;
    indices[1] = ZERO;
    if (mHasZeroExtendedStream) {
        indices[2] = b->getInt32(ZERO_EXTENDED_BUFFER_INDEX);
        mZeroExtendBuffer = b->CreateInBoundsGEP(mThreadLocalHandle, indices);
        indices[2] = b->getInt32(ZERO_EXTENDED_SPACE_INDEX);
        mZeroExtendSpace = b->CreateInBoundsGEP(mThreadLocalHandle, indices);
    }
    mCurrentThreadTerminationSignalPtr = nullptr;
    if (canSetTerminateSignal()) {
        if (mNumOfThreads != 1) {
            indices[2] = b->getInt32(TERMINATION_SIGNAL_INDEX);
            mCurrentThreadTerminationSignalPtr = b->CreateInBoundsGEP(mThreadLocalHandle, indices);
        } else {
            mCurrentThreadTerminationSignalPtr = getTerminationSignalPtr();
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateInitializeThreadLocalMethod(BuilderRef b) {
    if (mTarget->hasThreadLocal()) {
        for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
            const Kernel * const kernel = getKernel(i);
            if (kernel->hasThreadLocal()) {
                setActiveKernel(b, i);
                assert (mKernel == kernel);
                Value * const f = getKernelInitializeThreadLocalFunction(b);
                if (LLVM_UNLIKELY(f == nullptr)) {
                    report_fatal_error(mKernel->getName() + " does not have an initialize method for its threadlocal state");
                }
                Value * const handle = b->CreateCall(f, mKernelHandle);
                b->CreateStore(handle, getThreadLocalHandlePtr(b, i));
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateFinalizeThreadLocalMethod(BuilderRef b) {
    if (mTarget->hasThreadLocal()) {
        if (mHasThreadLocalPipelineState) {
            Value * const localState = getScalarFieldPtr(b.get(), PIPELINE_THREAD_LOCAL_STATE);
            FixedArray<Value *, 2> indices;
            indices[0] = b->getInt32(0);
            if (mHasZeroExtendedStream) {
                indices[1] = b->getInt32(ZERO_EXTENDED_BUFFER_INDEX);
                b->CreateFree(b->CreateLoad(b->CreateInBoundsGEP(localState, indices)));
            }
        }
        for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
            const Kernel * const kernel = getKernel(i);
            if (kernel->hasThreadLocal()) {
                setActiveKernel(b, i);
                assert (mKernel == kernel);
                SmallVector<Value *, 2> args;
                if (LLVM_LIKELY(mKernel->isStateful())) {
                    args.push_back(mKernelHandle);
                }
                args.push_back(b->CreateLoad(getThreadLocalHandlePtr(b, i)));
                Value * const f = getKernelFinalizeThreadLocalFunction(b);
                if (LLVM_UNLIKELY(f == nullptr)) {
                    report_fatal_error(mKernel->getName() + " does not to have an finalize method for its threadlocal state");
                }
                b->CreateCall(f, args);
            }
        }
        // Since all of the nested kernels thread local state is contained within
        // this pipeline thread's thread local state, freeing the pipeline's will
        // also free the inner kernels.
        b->CreateFree(getThreadLocalHandle());
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readTerminationSignalFromLocalState
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::readTerminationSignalFromLocalState(BuilderRef b, Value * const localState) const {
    // TODO: generalize a OR/ADD/etc "combination" mechanism for thread-local to output scalars?
    assert (localState);
    if (mCurrentThreadTerminationSignalPtr) {
        FixedArray<Value *, 2> indices;
        indices[0] = b->getInt32(0);
        indices[1] = b->getInt32(TERMINATION_SIGNAL_INDEX);
        return b->CreateLoad(b->CreateInBoundsGEP(localState, indices));
    }
    return nullptr;
}

}

#endif // PIPELINE_LOGIC_HPP
