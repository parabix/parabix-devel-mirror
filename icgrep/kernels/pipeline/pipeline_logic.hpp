#ifndef PIPELINE_LOGIC_HPP
#define PIPELINE_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateSingleThreadKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateSingleThreadKernelMethod(BuilderRef b) {
    StructType * const localStateType = getLocalStateType(b);
    Value * const localState = allocateThreadLocalState(b, localStateType);
    setThreadLocalState(b, localState);
    start(b, b->getSize(0));
    for (unsigned i = 0; i < mPipeline.size(); ++i) {
        setActiveKernel(b, i);
        executeKernel(b);
    }
    end(b, 1);
    deallocateThreadLocalState(b, localState);
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
void PipelineCompiler::generateMultiThreadKernelMethod(BuilderRef b, const unsigned numOfThreads) {

    assert (numOfThreads > 1);

    Module * const m = b->getModule();
    IntegerType * const sizeTy = b->getSizeTy();
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
    Type * const pthreadsTy = ArrayType::get(sizeTy, threads);
    AllocaInst * const pthreads = b->CreateAlloca(pthreadsTy);
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
    deallocateThreadLocalState(b, processState);

    // wait for all other threads to complete
    AllocaInst * const status = b->CreateAlloca(voidPtrTy);
    for (unsigned i = 0; i < threads; ++i) {
        Value * threadId = b->CreateLoad(threadIdPtr[i]);
        b->CreatePThreadJoinCall(threadId, status);
        deallocateThreadLocalState(b, threadState[i]);
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
    for (unsigned i = 0; i < mPipeline.size(); ++i) {
        setActiveKernel(b, i);
        synchronize(b);
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
 * @brief getThreadStateType
 ** ------------------------------------------------------------------------------------------------------------- */
inline StructType * PipelineCompiler::getThreadStateType(BuilderRef b) {

    StructType * const localStateTy = getLocalStateType(b);
    std::vector<Type *> threadStructFields;
    threadStructFields.push_back(mPipelineKernel->getHandle()->getType());
    threadStructFields.push_back(b->getSizeTy());
    threadStructFields.push_back(localStateTy->getPointerTo());
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
inline AllocaInst * PipelineCompiler::allocateThreadState(BuilderRef b, const unsigned segOffset) {

    Constant * const ZERO = b->getInt32(0);
    Constant * const HANDLE = ZERO;
    Constant * const SEG_OFFSET = b->getInt32(1);
    Constant * const LOCAL_STATE = b->getInt32(2);

    StructType * const threadStructType = getThreadStateType(b);
    AllocaInst * const threadState = b->CreateAlloca(threadStructType);
    b->CreateStore(mPipelineKernel->getHandle(), b->CreateGEP(threadState, {ZERO, HANDLE}));
    b->CreateStore(b->getSize(segOffset), b->CreateGEP(threadState, {ZERO, SEG_OFFSET}));
    StructType * const localStateTy = getLocalStateType(b);
    Value * const localState = allocateThreadLocalState(b, localStateTy);
    b->CreateStore(localState, b->CreateGEP(threadState, {ZERO, LOCAL_STATE}));
    const auto numOfInputs = mPipelineKernel->getNumOfStreamInputs();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        auto buffer = mPipelineKernel->getInputStreamSetBuffer(i);
        Value * const handle = buffer->getHandle();
        b->CreateStore(handle, b->CreateGEP(threadState, {ZERO, b->getInt32(i + 3)}));
    }
    const auto numOfOutputs = mPipelineKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        auto buffer = mPipelineKernel->getOutputStreamSetBuffer(i);
        Value * const handle = buffer->getHandle();
        b->CreateStore(handle, b->CreateGEP(threadState, {ZERO, b->getInt32(i + numOfInputs + 3)}));
    }

    return threadState;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructThreadState
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::setThreadState(BuilderRef b, Value * threadState) {

    Constant * const ZERO = b->getInt32(0);
    Constant * const HANDLE = ZERO;
    Constant * const SEG_OFFSET = b->getInt32(1);
    Constant * const LOCAL_STATE = b->getInt32(2);

    Value * handle = b->CreateLoad(b->CreateGEP(threadState, {ZERO, HANDLE}));

    mPipelineKernel->setHandle(b, handle);
    Value * const segmentOffset = b->CreateLoad(b->CreateGEP(threadState, {ZERO, SEG_OFFSET}));
    setThreadLocalState(b, b->CreateLoad(b->CreateGEP(threadState, {ZERO, LOCAL_STATE})));
    const auto numOfInputs = mPipelineKernel->getNumOfStreamInputs();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        Value * streamHandle = b->CreateLoad(b->CreateGEP(threadState, {ZERO, b->getInt32(i + 3)}));
        auto buffer = mPipelineKernel->getInputStreamSetBuffer(i);
        buffer->setHandle(b, streamHandle);
    }
    const auto numOfOutputs = mPipelineKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        Value * streamHandle = b->CreateLoad(b->CreateGEP(threadState, {ZERO, b->getInt32(i + numOfInputs + 3)}));
        auto buffer = mPipelineKernel->getOutputStreamSetBuffer(i);
        buffer->setHandle(b, streamHandle);
    }

    return segmentOffset;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deallocateThreadLocalSpace
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::deallocateThreadState(BuilderRef b, Value * const threadState) {
    Constant * const ZERO = b->getInt32(0);
    Constant * const LOCAL_STATE = b->getInt32(2);
    deallocateThreadLocalState(b, b->CreateGEP(threadState, {ZERO, LOCAL_STATE}));
}


enum : int {
    POP_COUNT_STRUCT_INDEX = 0
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getLocalStateType
 ** ------------------------------------------------------------------------------------------------------------- */
inline StructType * PipelineCompiler::getLocalStateType(BuilderRef b) {
    StructType * const popCountTy = getPopCountThreadLocalStateType(b);
    return StructType::get(popCountTy, nullptr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief allocateThreadLocalState
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::allocateThreadLocalState(BuilderRef b, StructType * localStateType) {
    Value * const localState = b->CreateCacheAlignedAlloca(localStateType);
    Constant * const ZERO = b->getInt32(0);
    Constant * const POP_COUNT_STRUCT = b->getInt32(POP_COUNT_STRUCT_INDEX);
    allocatePopCountArrays(b, b->CreateGEP(localState, {ZERO, POP_COUNT_STRUCT}));
    return localState;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setThreadLocalState
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::setThreadLocalState(BuilderRef b, Value * const localState) {
    Constant * const ZERO = b->getInt32(0);
    Constant * const POP_COUNT_STRUCT = b->getInt32(POP_COUNT_STRUCT_INDEX);
    mPopCountState = b->CreateGEP(localState, {ZERO, POP_COUNT_STRUCT});
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deallocateThreadLocalState
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::deallocateThreadLocalState(BuilderRef b, Value * const localState) {
    Constant * const ZERO = b->getInt32(0);
    Constant * const POP_COUNT_STRUCT = b->getInt32(POP_COUNT_STRUCT_INDEX);
    deallocatePopCountArrays(b, b->CreateGEP(localState, {ZERO, POP_COUNT_STRUCT}));
}

}

#endif // PIPELINE_LOGIC_HPP
