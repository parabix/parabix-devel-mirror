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

inline void destroyStateObject(BuilderRef b, Value * threadState) {
    if (LLVM_UNLIKELY(allocateOnHeap(b))) {
        b->CreateFree(threadState);
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

    IntegerType * const sizeTy = b->getSizeTy();

    if (!ExternallySynchronized) {
        mTarget->addInternalScalar(sizeTy, NEXT_LOGICAL_SEGMENT_NUMBER, 0);
    }

    mTarget->addInternalScalar(sizeTy, EXPECTED_NUM_OF_STRIDES_MULTIPLIER, 0);

    #ifdef PERMIT_BUFFER_MEMORY_REUSE
    if (LLVM_LIKELY(RequiredThreadLocalStreamSetMemory > 0)) {
        PointerType * const int8PtrTy = b->getInt8PtrTy();
        mTarget->addThreadLocalScalar(int8PtrTy, BASE_THREAD_LOCAL_STREAMSET_MEMORY, 0);
    }
    #endif

    // NOTE: both the shared and thread local objects are parameters to the kernel.
    // They get automatically set by reading in the appropriate params.

    if (HasZeroExtendedStream) {
        PointerType * const voidPtrTy = b->getVoidPtrTy();
        mTarget->addThreadLocalScalar(voidPtrTy, ZERO_EXTENDED_BUFFER);
        mTarget->addThreadLocalScalar(sizeTy, ZERO_EXTENDED_SPACE);
    }

    auto currentPartitionId = -1U;
    addBufferHandlesToPipelineKernel(b, PipelineInput);
    addConsumerKernelProperties(b, PipelineInput);
    addPipelinePriorItemCountProperties(b);
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        addBufferHandlesToPipelineKernel(b, i);
        // Is this the start of a new partition?
        const auto partitionId = KernelPartitionId[i];
        const bool isRoot = (partitionId != currentPartitionId);
        if (isRoot) {
            addTerminationProperties(b, i);
            currentPartitionId = partitionId;
        }
        addInternalKernelProperties(b, i);
        addConsumerKernelProperties(b, i);
        addCycleCounterProperties(b, i, isRoot);
        addProducedItemCountDeltaProperties(b, i);
        addUnconsumedItemCountProperties(b, i);
        addFamilyKernelProperties(b, i);
    }



}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addPipelinePriorItemCountProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addPipelinePriorItemCountProperties(BuilderRef b) {
    // If we have external I/O, it's possible that the processed/produced counts passed into
    // the pipeline are greater than the expected number. E.g.,, the OptimizationBranch kernel
    // could perform a segment of work utilizing one branch A before switching to branch B. The
    // latter would be unaware of any computations performed by the former and (unless the
    // OptimizationBranch adjusted the item counts and the virtual base address of each stream)
    // and would try to redo the work, likely leading to invalid output. To handle it in a
    // general way, the pipeline automatically increments all of the internal processed/
    // produced counts of the

//    if (ExternallySynchronized) {

//        IntegerType * const sizeTy = b->getSizeTy();

//        for (const auto input : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
//            const BufferPort & br = mBufferGraph[input];
//            const auto prefix = makeBufferName(PipelineInput, br.Port);
//            mTarget->addInternalScalar(sizeTy, prefix + EXTERNAL_IO_PRIOR_ITEM_COUNT_SUFFIX, 0);
//        }

//        for (const auto output : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
//            const BufferPort & br = mBufferGraph[output];
//            const auto prefix = makeBufferName(PipelineOutput, br.Port);
//            mTarget->addInternalScalar(sizeTy, prefix + EXTERNAL_IO_PRIOR_ITEM_COUNT_SUFFIX, 0);
//        }

//    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addPipelinePriorItemCountProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::updateInternalPipelineItemCount(BuilderRef b) {

//    if (ExternallySynchronized) {

//        for (const auto input : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
//            const BufferPort & br = mBufferGraph[input];
//            const auto prefix = makeBufferName(PipelineInput, br.Port);
//            Value * const ptr = b->getScalarFieldPtr(prefix + EXTERNAL_IO_PRIOR_ITEM_COUNT_SUFFIX);
//            Value * const prior = b->CreateLoad(ptr);
//            Value * const current = getProcessedInputItemsPtr(br.Port.Number);
//            b->CreateStore(current, ptr);
//            b->CreateSub(current, prior);

//        }



//    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addInternalKernelProperties(BuilderRef b, const unsigned kernelId) {
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
//        const auto streamSet = source(e, mBufferGraph);
//        const BufferNode & bn = mBufferGraph[streamSet];
//        if (LLVM_UNLIKELY(bn.isExternal())) {
//            continue;
//        }
//        assert (parent(streamSet, mBufferGraph) != PipelineInput);
        const BufferPort & br = mBufferGraph[e];
        const auto prefix = makeBufferName(kernelId, br.Port);
        if (LLVM_UNLIKELY(br.IsDeferred)) {
            mTarget->addInternalScalar(sizeTy, prefix + DEFERRED_ITEM_COUNT_SUFFIX, kernelId);
        }
        mTarget->addInternalScalar(sizeTy, prefix + ITEM_COUNT_SUFFIX, kernelId);
    }

    for (const auto e : make_iterator_range(out_edges(kernelId, mBufferGraph))) {
//        const auto streamSet = target(e, mBufferGraph);
//        const BufferNode & bn = mBufferGraph[streamSet];
//        if (LLVM_UNLIKELY(bn.isExternal())) {
//            continue;
//        }
//        assert (parent(streamSet, mBufferGraph) != PipelineInput);
        const BufferPort & br = mBufferGraph[e];
        const auto prefix = makeBufferName(kernelId, br.Port);
        if (LLVM_UNLIKELY(br.IsDeferred)) {
            mTarget->addInternalScalar(sizeTy, prefix + DEFERRED_ITEM_COUNT_SUFFIX, kernelId);
        }
        mTarget->addInternalScalar(sizeTy, prefix + ITEM_COUNT_SUFFIX, kernelId);
    }

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers))) {
        for (const auto e : make_iterator_range(out_edges(kernelId, mBufferGraph))) {
            const auto bufferVertex = target(e, mBufferGraph);
            const BufferNode & bn = mBufferGraph[bufferVertex];
            if (isa<DynamicBuffer>(bn.Buffer)) {
                const BufferPort & rd = mBufferGraph[e];
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
            setActiveKernel(b, i, false);
            initializeStridesPerSegment(b);
            ArgVec args;

            if (LLVM_LIKELY(mKernel->isStateful())) {
                args.push_back(mKernelSharedHandle);
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
 * @brief generateAllocateInternalStreamSetsMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateAllocateSharedInternalStreamSetsMethod(BuilderRef b, Value * const expectedNumOfStrides) {
    b->setScalarField(EXPECTED_NUM_OF_STRIDES_MULTIPLIER, expectedNumOfStrides);
    allocateOwnedBuffers(b, expectedNumOfStrides, true);
    initializeBufferExpansionHistory(b);
    resetInternalBufferHandles();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateInitializeThreadLocalMethod(BuilderRef b) {
    assert (mTarget->hasThreadLocal());
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernel = getKernel(i);
        if (kernel->hasThreadLocal()) {
            setActiveKernel(b, i, true);
            assert (mKernel == kernel);
            Value * const f = getKernelInitializeThreadLocalFunction(b);
            if (LLVM_UNLIKELY(f == nullptr)) {
                report_fatal_error(mKernel->getName() + " does not have an initialize method for its threadlocal state");
            }
            Value * const handle = b->CreateCall(f, mKernelSharedHandle);
            b->CreateStore(handle, getThreadLocalHandlePtr(b, i));
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateAllocateThreadLocalInternalStreamSetsMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateAllocateThreadLocalInternalStreamSetsMethod(BuilderRef b, Value * const expectedNumOfStrides) {
    assert (mTarget->hasThreadLocal());
    #ifdef PERMIT_BUFFER_MEMORY_REUSE
    if (LLVM_LIKELY(RequiredThreadLocalStreamSetMemory > 0)) {
        ConstantInt * const reqMemory = b->getSize(RequiredThreadLocalStreamSetMemory);
        Value * const base = b->CreateCacheAlignedMalloc(b->CreateMul(reqMemory, expectedNumOfStrides));
        PointerType * const int8PtrTy = b->getInt8PtrTy();
        b->setScalarField(BASE_THREAD_LOCAL_STREAMSET_MEMORY, b->CreatePointerCast(base, int8PtrTy));
    }
    #endif
    allocateOwnedBuffers(b, expectedNumOfStrides, false);
    resetInternalBufferHandles();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::generateKernelMethod(BuilderRef b) {
    initializeKernelAssertions(b);
    verifyBufferRelationships();
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
 * @brief generateSingleThreadKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateSingleThreadKernelMethod(BuilderRef b) {    
    createThreadStateForSingleThread(b);
    start(b);
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        setActiveKernel(b, i, true);            
        executeKernel(b);
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

enum : unsigned {
    PIPELINE_PARAMS
    , PROCESS_THREAD_ID
    , TERMINATION_SIGNAL
    // -------------------
    , THREAD_STRUCT_SIZE
};

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
    // MAKE PIPELINE DRIVER
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

    for (unsigned i = 0; i != additionalThreads; ++i) {
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
        threadState[i] = constructThreadStructObject(b, processThreadId, threadLocal[i]);
        FixedArray<Value *, 2> indices;
        indices[0] = ZERO;
        indices[1] = b->getInt32(i);
        threadIdPtr[i] = b->CreateInBoundsGEP(pthreads, indices);
        b->CreatePThreadCreateCall(threadIdPtr[i], nullVoidPtrVal, threadFunc, threadState[i]);
    }


    // execute the process thread
    assert (isFromCurrentFunction(b, initialThreadLocal));
    Value * const pty_ZERO = Constant::getNullValue(b->getPThreadTy());
    Value * const processState = constructThreadStructObject(b, pty_ZERO, initialThreadLocal);
    b->CreateCall(threadFunc, b->CreatePointerCast(processState, voidPtrTy));

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
    readThreadStuctObject(b, threadStruct);
    assert (isFromCurrentFunction(b, getHandle()));
    assert (isFromCurrentFunction(b, getThreadLocalHandle()));

    // generate the pipeline logic for this thread
    start(b);
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        setActiveKernel(b, i, true);
        executeKernel(b);
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

    // -------------------------------------------------------------------------------------------------------------------------
    // MAKE PIPELINE DRIVER CONTINUED
    // -------------------------------------------------------------------------------------------------------------------------

    b->restoreIP(resumePoint);
    assert (isFromCurrentFunction(b, processState));
    assert (isFromCurrentFunction(b, initialSharedState));
    assert (isFromCurrentFunction(b, initialThreadLocal));

    // wait for all other threads to complete
    AllocaInst * const status = b->CreateAlloca(voidPtrTy);
    for (unsigned i = 0; i != additionalThreads; ++i) {
        Value * threadId = b->CreateLoad(threadIdPtr[i]);
        b->CreatePThreadJoinCall(threadId, status);
    }

    if (mTarget->hasThreadLocal()) {
        const auto n = mTarget->isStateful() ? 2 : 1;
        SmallVector<Value *, 2> args(n);
        if (LLVM_LIKELY(mTarget->isStateful())) {
            args[0] = initialSharedState;
        }
        for (unsigned i = 0; i < additionalThreads; ++i) {
            args[n - 1] = threadLocal[i];
            mTarget->finalizeThreadLocalInstance(b, args);
        }
    }
    if (PipelineHasTerminationSignal) {
        assert (initialTerminationSignalPtr);
        assert (mCurrentThreadTerminationSignalPtr);
        Value * terminated = readTerminationSignalFromLocalState(b, processState);
        assert (terminated);
        for (unsigned i = 0; i < additionalThreads; ++i) {
            Value * const terminatedSignal = readTerminationSignalFromLocalState(b, threadState[i]);
            assert (terminatedSignal);
            assert (terminated->getType() == terminatedSignal->getType());
            terminated = b->CreateUMax(terminated, terminatedSignal);
        }
        assert (terminated);
        b->CreateStore(terminated, initialTerminationSignalPtr);
    }

    // TODO: the pipeline kernel scalar state is invalid after leaving this function. Best bet would be to copy the
    // scalarmap and replace it.
    for (unsigned i = 0; i < additionalThreads; ++i) {
        destroyStateObject(b, threadState[i]);
    }
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
        const auto prefix = makeKernelName(i);
        Value * const ptr = getScalarFieldPtr(b.get(), prefix + LOGICAL_SEGMENT_SUFFIX);
        Value * const segNo = b->CreateLoad(ptr);
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
        setActiveKernel(b, i, false);
        SmallVector<Value *, 1> params;
        if (LLVM_LIKELY(mKernel->isStateful())) {
            params.push_back(mKernelSharedHandle);
        }
        mScalarValue[i] = b->CreateCall(getKernelFinalizeFunction(b), params);
    }
    releaseOwnedBuffers(b, true);
    resetInternalBufferHandles();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getThreadStateType
 ** ------------------------------------------------------------------------------------------------------------- */
inline StructType * PipelineCompiler::getThreadStuctType(BuilderRef b) const {
    FixedArray<Type *, THREAD_STRUCT_SIZE> fields;
    LLVMContext & C = b->getContext();

    assert (mNumOfThreads > 1);

    // NOTE: both the shared and thread local objects are parameters to the kernel.
    // They get automatically set by reading in the appropriate params.

    fields[PIPELINE_PARAMS] = StructType::get(C, mTarget->getDoSegmentFields(b));
    fields[PROCESS_THREAD_ID] = b->getPThreadTy();
    if (LLVM_LIKELY(PipelineHasTerminationSignal)) {
        fields[TERMINATION_SIGNAL] = b->getSizeTy();
    } else {
        fields[TERMINATION_SIGNAL] = StructType::get(C);
    }
    return StructType::get(C, fields);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructThreadStructObject
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::constructThreadStructObject(BuilderRef b, Value * const threadId, Value * const threadLocal) {
    StructType * const threadStructType = getThreadStuctType(b);
    Value * const threadState = makeStateObject(b, threadStructType);
    setThreadLocalHandle(threadLocal);
    const auto props = getDoSegmentProperties(b);
    const auto n = props.size();
    assert (threadStructType->getStructElementType(PIPELINE_PARAMS)->getStructNumElements() == n);

    FixedArray<Value *, 3> indices3;
    indices3[0] = b->getInt32(0);
    indices3[1] = b->getInt32(PIPELINE_PARAMS);
    for (unsigned i = 0; i < n; ++i) {
        indices3[2] = b->getInt32(i);
        b->CreateStore(props[i], b->CreateInBoundsGEP(threadState, indices3));
    }
    FixedArray<Value *, 2> indices2;
    indices2[0] = b->getInt32(0);
    indices2[1] = b->getInt32(PROCESS_THREAD_ID);
    b->CreateStore(threadId, b->CreateInBoundsGEP(threadState, indices2));

    return threadState;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readThreadStuctObject
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::readThreadStuctObject(BuilderRef b, Value * threadState) {
    FixedArray<Value *, 3> indices3;
    Constant * const ZERO = b->getInt32(0);
    indices3[0] = ZERO;
    indices3[1] = b->getInt32(PIPELINE_PARAMS);
    Type * const kernelStructType = threadState->getType()->getPointerElementType();
    const auto n = kernelStructType->getStructElementType(PIPELINE_PARAMS)->getStructNumElements();
    SmallVector<Value *, 16> args(n);
    for (unsigned i = 0; i != n; ++i) {
        indices3[2] = b->getInt32(i);
        args[i] = b->CreateLoad(b->CreateInBoundsGEP(threadState, indices3));
    }
    setDoSegmentProperties(b, args);

    FixedArray<Value *, 2> indices2;
    indices2[0] = ZERO;

    assert (mNumOfThreads > 1);
    mCurrentThreadTerminationSignalPtr = getTerminationSignalPtr();
    if (PipelineHasTerminationSignal) {
        indices2[1] = b->getInt32(TERMINATION_SIGNAL);
        mCurrentThreadTerminationSignalPtr = b->CreateInBoundsGEP(threadState, indices2);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createThreadStateForSingleThread
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::createThreadStateForSingleThread(BuilderRef /* b */) {
    if (PipelineHasTerminationSignal) {
        mCurrentThreadTerminationSignalPtr = getTerminationSignalPtr();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isProcessThread
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::isProcessThread(BuilderRef b, Value * const threadState) const {
    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(PROCESS_THREAD_ID);
    Value * const ptr = b->CreateInBoundsGEP(threadState, indices);
    return b->CreateIsNull(b->CreateLoad(ptr));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateFinalizeThreadLocalMethod(BuilderRef b) {

    assert (mTarget->hasThreadLocal());
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernel = getKernel(i);
        if (kernel->hasThreadLocal()) {
            setActiveKernel(b, i, true);
            assert (mKernel == kernel);
            SmallVector<Value *, 2> args;
            if (LLVM_LIKELY(mKernelSharedHandle != nullptr)) {
                args.push_back(mKernelSharedHandle);
            }
            args.push_back(mKernelThreadLocalHandle);
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
    #ifdef PERMIT_BUFFER_MEMORY_REUSE
    if (LLVM_LIKELY(RequiredThreadLocalStreamSetMemory > 0)) {
        Value * const addr = b->getScalarField(BASE_THREAD_LOCAL_STREAMSET_MEMORY);
        b->CreateFree(addr);
    }
    #endif
    b->CreateFree(getThreadLocalHandle());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readTerminationSignalFromLocalState
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::readTerminationSignalFromLocalState(BuilderRef b, Value * const threadState) const {
    // TODO: generalize a OR/ADD/etc "combination" mechanism for thread-local to output scalars?
    assert (threadState);
    assert (mCurrentThreadTerminationSignalPtr);
    assert (PipelineHasTerminationSignal);
    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(TERMINATION_SIGNAL);
    Value * const signal = b->CreateLoad(b->CreateInBoundsGEP(threadState, indices));
    assert (signal->getType()->isIntegerTy());
    return signal;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifyBufferRelationships
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::verifyBufferRelationships() const {

    // If this pipeline is internally synchronized, it must own and manage its output buffers; otherwise
    // the outer pipeline would have to be able to manage it without having correct knowledge of its
    // current state in multithreaded mode. Verify that the correct attributes have been set.
    if (LLVM_UNLIKELY(ExternallySynchronized)) {
        for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
            const auto streamSet = source(e, mBufferGraph);
            const auto producer = parent(streamSet, mBufferGraph);
            const Kernel * const kernelObj = getKernel(producer);
            assert (kernelObj);
            const auto synchronized = kernelObj->hasAttribute(AttrId::InternallySynchronized);
            const BufferPort & br = mBufferGraph[e];
            const Binding & output = br.Binding;

            const auto managed = br.IsManaged;
            const auto sharedManaged = br.IsShared;

            if (LLVM_UNLIKELY(managed && sharedManaged)) {
                SmallVector<char, 256> tmp;
                raw_svector_ostream out(tmp);
                out << mTarget->getName() << "." << output.getName();
                out << " cannot be both a Managed and SharedManaged buffer.";
                report_fatal_error(out.str());
            }

            const auto unmanaged = !(managed | sharedManaged);

            if (LLVM_UNLIKELY(synchronized ^ unmanaged)) {
                SmallVector<char, 256> tmp;
                raw_svector_ostream out(tmp);
                out << mTarget->getName() << "." << output.getName()
                    << " should ";
                if (synchronized) {
                    out << "not ";
                }
                out << "have been marked as a ManagedBuffer.";
                report_fatal_error(out.str());
            }


        }
    }



#if 0

    // verify that the buffer config is valid
    for (unsigned i = FirstStreamSet; i <= LastStreamSet; ++i) {

        const BufferNode & bn = G[i];
        const auto pe = in_edge(i, G);
        const auto producerVertex = source(pe, G);
        const Kernel * const producer = getKernel(producerVertex);
        const BufferRateData & producerRate = G[pe];
        const Binding & output = producerRate.Binding;




        // Type check stream set I/O types.
        Type * const baseType = output.getType();
        for (const auto e : make_iterator_range(out_edges(i, G))) {
            const BufferRateData & consumerRate = G[e];
            const Binding & input = consumerRate.Binding;
            if (LLVM_UNLIKELY(baseType != input.getType())) {
                SmallVector<char, 256> tmp;
                raw_svector_ostream msg(tmp);
                msg << producer->getName() << ':' << output.getName()
                    << " produces a ";
                baseType->print(msg);
                const Kernel * const consumer = getKernel(target(e, G));
                msg << " but "
                    << consumer->getName() << ':' << input.getName()
                    << " expects ";
                input.getType()->print(msg);
                report_fatal_error(msg.str());
            }
        }

        for (const auto ce : make_iterator_range(out_edges(i, G))) {
            const Binding & input = G[ce].Binding;
            if (LLVM_UNLIKELY(requiresLinearAccess(input))) {
                SmallVector<char, 256> tmp;
                raw_svector_ostream out(tmp);
                const auto consumer = target(ce, G);
                out << getKernel(consumer)->getName()
                    << '.' << input.getName()
                    << " requires that "
                    << producer->getName()
                    << '.' << output.getName()
                    << " is a Linear buffer.";
                report_fatal_error(out.str());
            }
        }


    }

#endif

}

}

#endif // PIPELINE_LOGIC_HPP
