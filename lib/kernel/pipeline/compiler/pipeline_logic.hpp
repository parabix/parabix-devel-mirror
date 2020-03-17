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
        if (LLVM_LIKELY(kernel->isGenerated())) {
            kernel->ensureLoaded();
            continue;
        }
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

    Type * const localStateType = getThreadLocalStateType(b);
    if (localStateType->isEmptyTy()) {
        mHasThreadLocalPipelineState = false;
    } else {
        mTarget->addThreadLocalScalar(localStateType, PIPELINE_THREAD_LOCAL_STATE);
        mHasThreadLocalPipelineState = true;
    }

    IntegerType * const sizeTy = b->getSizeTy();
    mTarget->addInternalScalar(sizeTy, NEXT_LOGICAL_SEGMENT_SUFFIX, 0);

    auto currentPartitionId = -1U;
    addBufferHandlesToPipelineKernel(b, PipelineInput);
    addConsumerKernelProperties(b, PipelineInput);
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        addBufferHandlesToPipelineKernel(b, i);
        // Is this the start of a new partition?
        const auto partitionId = KernelPartitionId[i];
        if (partitionId != currentPartitionId) {
            addTerminationProperties(b, i);
            currentPartitionId = partitionId;
        }
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
inline void PipelineCompiler::addInternalKernelProperties(BuilderRef b, const unsigned kernelId) {
    const Kernel * const kernel = getKernel(kernelId);
    IntegerType * const sizeTy = b->getSizeTy();

    // TODO: if we've proven we do not need synchronization then we've already proven that
    // we can calculate the item count and num of strides from the input item counts.
    // With the inclusion of InternallySynchronized attributes for PipelineKernels, this is
    // no longer true and the test requires greater precision.

    const auto name = makeKernelName(kernelId);
    mTarget->addInternalScalar(sizeTy, name + LOGICAL_SEGMENT_SUFFIX, kernelId);

    if (LLVM_LIKELY(kernel->isStateful())) {
        PointerType * sharedStateTy = nullptr;
        if (LLVM_UNLIKELY(kernel->externallyInitialized())) {
            sharedStateTy = b->getVoidPtrTy();
        } else {
            sharedStateTy = kernel->getSharedStateType()->getPointerTo(0);
        }
        mTarget->addInternalScalar(sharedStateTy, name, kernelId);
    }

    if (kernel->hasThreadLocal()) {
        // we cannot statically allocate a "family" thread local object.
        PointerType * localStateTy = nullptr;
        if (LLVM_UNLIKELY(kernel->externallyInitialized())) {
            localStateTy = b->getVoidPtrTy();
        } else {
            localStateTy = kernel->getThreadLocalStateType()->getPointerTo(0);
        }
        mTarget->addThreadLocalScalar(localStateTy, name + KERNEL_THREAD_LOCAL_SUFFIX, kernelId);
    }


    for (const auto e : make_iterator_range(in_edges(kernelId, mBufferGraph))) {
        const BufferRateData & br = mBufferGraph[e];
        const auto prefix = makeBufferName(kernelId, br.Port);
        const Binding & binding = br.Binding;
        if (LLVM_UNLIKELY(binding.isDeferred())) {
            mTarget->addInternalScalar(sizeTy, prefix + DEFERRED_ITEM_COUNT_SUFFIX, kernelId);
        }
        mTarget->addInternalScalar(sizeTy, prefix + ITEM_COUNT_SUFFIX);
    }

    for (const auto e : make_iterator_range(out_edges(kernelId, mBufferGraph))) {
        const BufferRateData & br = mBufferGraph[e];
        const auto prefix = makeBufferName(kernelId, br.Port);
        const Binding & binding = br.Binding;

        bool requiresFullyProducedItemCount = false;
        bool isDeferred = false;
        for (const Attribute & attr : binding.getAttributes()) {
            switch (attr.getKind()) {
                case AttrId::Delayed:
                case AttrId::BlockSize:
                    requiresFullyProducedItemCount = true;
                    break;
                case AttrId::Deferred:
                    isDeferred = true;
                default: break;
            }
        }

        if (LLVM_UNLIKELY(isDeferred)) {
            mTarget->addInternalScalar(sizeTy, prefix + DEFERRED_ITEM_COUNT_SUFFIX, kernelId);
        }
        mTarget->addInternalScalar(sizeTy, prefix + ITEM_COUNT_SUFFIX, kernelId);

        if (requiresFullyProducedItemCount) {
            mTarget->addInternalScalar(sizeTy, prefix + FULLY_PRODUCED_ITEM_COUNT_SUFFIX, kernelId);
        } else if (isDeferred) {
            addAlias(prefix + FULLY_PRODUCED_ITEM_COUNT_SUFFIX, prefix + DEFERRED_ITEM_COUNT_SUFFIX);
        } else {
            addAlias(prefix + FULLY_PRODUCED_ITEM_COUNT_SUFFIX, prefix + ITEM_COUNT_SUFFIX);
        }
    }

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers))) {
        for (const auto e : make_iterator_range(out_edges(kernelId, mBufferGraph))) {
            const auto bufferVertex = target(e, mBufferGraph);
            const BufferNode & bn = mBufferGraph[bufferVertex];
            if (isa<DynamicBuffer>(bn.Buffer)) {
                const BufferRateData & rd = mBufferGraph[e];
                const auto prefix = makeBufferName(kernelId, rd.Port);
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
                                                   prefix + STATISTICS_BUFFER_EXPANSION_SUFFIX, kernelId);
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

        mTarget->addInternalScalar(StructType::get(C, traceStruct),
                                           name + STATISTICS_STRIDES_PER_SEGMENT_SUFFIX, kernelId);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateInitializeMethod(BuilderRef b) {

    // TODO: if we detect a fatal error at init, we should not execute
    // the pipeline loop.

    mScalarValue.reset(FirstKernel, LastScalar);

    initializeKernelAssertions(b);

    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernel = getKernel(i);
        if (LLVM_LIKELY(kernel->isStateful() && !kernel->externallyInitialized())) {
            Value * const handle = kernel->createInstance(b);
            b->setScalarField(makeKernelName(i), handle);
        }
    }

    Constant * const unterminated = getTerminationSignal(b, TerminationSignal::None);
    Constant * const aborted = getTerminationSignal(b, TerminationSignal::Aborted);

    Value * terminated = nullptr;
    auto partitionId = KernelPartitionId[FirstKernel];

    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {

        // Family kernels must be initialized in the "main" method.
        const Kernel * kernel = getKernel(i);
        if (LLVM_LIKELY(!kernel->externallyInitialized())) {
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
                args.push_back(getScalar(b, scalar));
            }
            Value * const f = getKernelInitializeFunction(b);
            if (LLVM_UNLIKELY(f == nullptr)) {
                report_fatal_error(mKernel->getName() + " does not have an initialize method");
            }

            for (auto i = 0U; i != args.size(); ++i) {
                assert (isFromCurrentFunction(b, args[i], false));
            }

            Value * const signal = b->CreateCall(f, args);
            Value * const terminatedOnInit = b->CreateICmpNE(signal, unterminated);

            if (terminated) {
                terminated = b->CreateOr(terminated, terminatedOnInit);
            } else {
                terminated = terminatedOnInit;
            }

        }

        // Is this the last kernel in a partition? If so, store the accumulated
        // termination signal.
        const auto nextPartitionId = KernelPartitionId[i + 1];
        if (terminated && partitionId != nextPartitionId) {
            Value * const signal = b->CreateSelect(terminated, aborted, unterminated);
            writeTerminationSignal(b, signal);
            partitionId = nextPartitionId;
            terminated = nullptr;
        }

    }
    resetInternalBufferHandles();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::generateKernelMethod(BuilderRef b) {
    mScalarValue.reset(FirstKernel, LastScalar);
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

    mKernelId = PipelineInput;

    // NOTE: all outputs of PipelineInput node are inputs to the PipelineKernel
    for (const auto e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {

        const auto buffer = target(e, mBufferGraph);
        const StreamSetPort inputPort = mBufferGraph[e].Port;
        assert (inputPort.Type == PortType::Output);
        Value * const available = getAvailableInputItems(inputPort.Number);
        setLocallyAvailableItemCount(b, inputPort, available);
        initializeConsumedItemCount(b, inputPort, available);

        Value * const inPtr = getProcessedInputItemsPtr(inputPort.Number);
        Value * const processed = b->CreateLoad(inPtr);

        for (const auto e : make_iterator_range(out_edges(buffer, mBufferGraph))) {
            const BufferRateData & rd = mBufferGraph[e];
            const auto kernelIndex = target(e, mBufferGraph);
            const auto prefix = makeBufferName(kernelIndex, rd.Port);
            Value * const ptr = b->getScalarFieldPtr(prefix + ITEM_COUNT_SUFFIX);
            b->CreateStore(processed, ptr);
        }
    }

    mKernelId = PipelineOutput;

    // NOTE: all inputs of PipelineOutput node are outputs of the PipelineKernel
    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        const auto buffer = source(e, mBufferGraph);
        const StreamSetPort outputPort = mBufferGraph[e].Port;
        assert (outputPort.Type == PortType::Input);
        Value * outPtr = getProducedOutputItemsPtr(outputPort.Number);
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
 * @brief concat
 ** ------------------------------------------------------------------------------------------------------------- */
inline StringRef concat(StringRef A, StringRef B, SmallVector<char, 256> & tmp) {
    Twine C = A + B;
    tmp.clear();
    C.toVector(tmp);
    return StringRef(tmp.data(), tmp.size());
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

    SmallVector<char, 256> tmp;
    const auto threadName = concat(mTarget->getName(), "_DoSegmentThread", tmp);

    FunctionType * const threadFuncType = FunctionType::get(b->getVoidTy(), {voidPtrTy}, false);
    Function * const threadFunc = Function::Create(threadFuncType, Function::InternalLinkage, threadName, m);
    threadFunc->setCallingConv(CallingConv::C);
    auto threadStructArg = threadFunc->arg_begin();
    threadStructArg->setName("threadStruct");

    Value * const initialSharedState = getHandle();
    Value * const initialThreadLocal = getThreadLocalHandle();
    Value * const initialTerminationSignalPtr = getTerminationSignalPtr();

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
            if (LLVM_LIKELY(mTarget->allocatesInternalStreamSets())) {
                Function * const allocInternal = mTarget->getAllocateThreadLocalInternalStreamSetsFunction(b, false);
                SmallVector<Value *, 3> allocArgs;
                if (LLVM_LIKELY(mTarget->isStateful())) {
                    allocArgs.push_back(initialSharedState);
                }
                allocArgs.push_back(threadLocal[i]);
                allocArgs.push_back(b->getSize(1));
                b->CreateCall(allocInternal, allocArgs);
            }
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
    const auto storedState = storeDoSegmentState();

    // -------------------------------------------------------------------------------------------------------------------------
    // MAKE PIPELINE THREAD
    // -------------------------------------------------------------------------------------------------------------------------
    b->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", threadFunc));
    PointerType * const threadStructTy = cast<PointerType>(processState->getType());
    #if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(4, 0, 0)
    Value * const threadStruct = b->CreatePointerCast(&*threadStructArg, threadStructTy);
    #else
    Value * const threadStruct = b->CreatePointerCast(threadStructArg, threadStructTy);
    #endif
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
    mKernelId = 0;
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
    restoreDoSegmentState(storedState);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateFinalizeMethod(BuilderRef b) {
    mScalarValue.reset(FirstKernel, LastScalar);
    // calculate the last segment # used by any kernel in case any reports require it.
    mSegNo = nullptr;
    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        Value * const segNo = b->getScalarField(makeKernelName(i) + LOGICAL_SEGMENT_SUFFIX);
        mSegNo = b->CreateUMax(mSegNo, segNo);
    }
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
    releaseOwnedBuffers(b, true);
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
    assert (mTarget->hasThreadLocal());
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateAllocateInternalStreamSetsMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateAllocateSharedInternalStreamSetsMethod(BuilderRef b, Value * expectedNumOfStrides) {
    allocateOwnedBuffers(b, expectedNumOfStrides, true);
    initializeBufferExpansionHistory(b);
    resetInternalBufferHandles();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateAllocateThreadLocalInternalStreamSetsMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateAllocateThreadLocalInternalStreamSetsMethod(BuilderRef b, Value * expectedNumOfStrides) {
    assert (mTarget->hasThreadLocal());
    allocateOwnedBuffers(b, expectedNumOfStrides, false);
    resetInternalBufferHandles();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateFinalizeThreadLocalMethod(BuilderRef b) {
    assert (mTarget->hasThreadLocal());
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
    releaseOwnedBuffers(b, false);
    // Since all of the nested kernels thread local state is contained within
    // this pipeline thread's thread local state, freeing the pipeline's will
    // also free the inner kernels.
    b->CreateFree(getThreadLocalHandle());
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
