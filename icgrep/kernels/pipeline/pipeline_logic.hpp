#ifndef PIPELINE_LOGIC_HPP
#define PIPELINE_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief compileSingleThread
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::generateSingleThreadKernelMethod(BuilderRef b) {
    StructType * const localStateType = getLocalStateType(b);
    Value * const localState = allocateThreadLocalSpace(b, localStateType);
    setThreadLocalSpace(b, localState);
    start(b, b->getSize(0));
    for (unsigned i = 0; i < mPipeline.size(); ++i) {
        setActiveKernel(b, i);
        executeKernel(b);
    }
    end(b, 1);
    deallocateThreadLocalSpace(b, localState);
}



/** ------------------------------------------------------------------------------------------------------------- *
 * @brief compileMultiThread
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
    ConstantInt * const ONE = b->getInt32(1);
    ConstantInt * const TWO = b->getInt32(2);

    Value * const handle = mPipelineKernel->getHandle(); assert (handle);
    StructType * const localStateTy = getLocalStateType(b);
    PointerType * const localStatePtrTy = localStateTy->getPointerTo();
    StructType * const threadStructType = StructType::get(m->getContext(), {handle->getType(), sizeTy, localStatePtrTy});
    FunctionType * const threadFuncType = FunctionType::get(b->getVoidTy(), {voidPtrTy}, false);

    const auto threadName = mPipelineKernel->getName() + "_DoSegmentThread";
    Function * const threadFunc = Function::Create(threadFuncType, Function::InternalLinkage, threadName, b->getModule());
    threadFunc->setCallingConv(CallingConv::C);
    auto args = threadFunc->arg_begin();
    args->setName("kernelStateObject");

    // store where we'll resume compiling the DoSegment method
    const auto resumePoint = b->saveIP();

    // -------------------------------------------------------------------------------------------------------------------------
    // MAKE PIPELINE THREAD
    // -------------------------------------------------------------------------------------------------------------------------
    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", threadFunc));
    Value * const threadStruct = b->CreateBitCast(&*(args), threadStructType->getPointerTo());
    mPipelineKernel->setHandle(b, b->CreateLoad(b->CreateGEP(threadStruct, {ZERO, ZERO})));
    Value * const segmentOffset = b->CreateLoad(b->CreateGEP(threadStruct, {ZERO, ONE}));
    setThreadLocalSpace(b, b->CreateLoad(b->CreateGEP(threadStruct, {ZERO, TWO})));
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
    Constant * const nullVoidPtrVal = ConstantPointerNull::getNullValue(voidPtrTy);
    b->CreatePThreadExitCall(nullVoidPtrVal);
    b->CreateBr(exitFunction);
    b->SetInsertPoint(exitFunction);
    b->CreateRetVoid();

    // -------------------------------------------------------------------------------------------------------------------------
    // MAKE PIPELINE DRIVER CONTINUED
    // -------------------------------------------------------------------------------------------------------------------------
    b->restoreIP(resumePoint);
    mPipelineKernel->setHandle(b, handle);
    const unsigned threads = numOfThreads - 1;
    Type * const pthreadsTy = ArrayType::get(sizeTy, threads);
    AllocaInst * const pthreads = b->CreateAlloca(pthreadsTy);
    Value * threadIdPtr[threads];
    for (unsigned i = 0; i < threads; ++i) {
        threadIdPtr[i] = b->CreateGEP(pthreads, {ZERO, b->getInt32(i)});
    }

    // use the process thread to handle the initial segment function after spawning
    // (n - 1) threads to handle the subsequent offsets
    std::vector<Value *> localState(numOfThreads);
    for (unsigned i = 0; i < threads; ++i) {
        AllocaInst * const threadState = b->CreateAlloca(threadStructType);
        b->CreateStore(handle, b->CreateGEP(threadState, {ZERO, ZERO}));
        b->CreateStore(b->getSize(i + 1), b->CreateGEP(threadState, {ZERO, ONE}));
        localState[i] = allocateThreadLocalSpace(b, localStateTy);
        b->CreateStore(localState[i], b->CreateGEP(threadState, {ZERO, TWO}));
        b->CreatePThreadCreateCall(threadIdPtr[i], nullVoidPtrVal, threadFunc, threadState);
    }

    AllocaInst * const threadState = b->CreateAlloca(threadStructType);
    b->CreateStore(handle, b->CreateGEP(threadState, {ZERO, ZERO}));
    b->CreateStore(b->getSize(0), b->CreateGEP(threadState, {ZERO, ONE}));
    b->CreateCall(threadFunc, b->CreatePointerCast(threadState, voidPtrTy));

    AllocaInst * const status = b->CreateAlloca(voidPtrTy);
    for (unsigned i = 0; i < threads; ++i) {
        Value * threadId = b->CreateLoad(threadIdPtr[i]);
        b->CreatePThreadJoinCall(threadId, status);
        deallocateThreadLocalSpace(b, localState[i]);
    }

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
 * @brief allocateThreadLocalSpace
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::allocateThreadLocalSpace(BuilderRef b, StructType * localStateType) {
    Value * const localState = b->CreateCacheAlignedAlloca(localStateType);
    Constant * const ZERO = b->getInt32(0);
    Constant * const POP_COUNT_STRUCT = b->getInt32(POP_COUNT_STRUCT_INDEX);
    allocatePopCountArrays(b, b->CreateGEP(localState, {ZERO, POP_COUNT_STRUCT}));
    return localState;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setThreadLocalSpace
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::setThreadLocalSpace(BuilderRef b, Value * const localState) {
    Constant * const ZERO = b->getInt32(0);
    Constant * const POP_COUNT_STRUCT = b->getInt32(POP_COUNT_STRUCT_INDEX);
    mPopCountState = b->CreateGEP(localState, {ZERO, POP_COUNT_STRUCT});
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deallocateThreadLocalSpace
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::deallocateThreadLocalSpace(BuilderRef b, Value * const localState) {
    Constant * const ZERO = b->getInt32(0);
    Constant * const POP_COUNT_STRUCT = b->getInt32(POP_COUNT_STRUCT_INDEX);
    deallocatePopCountArrays(b, b->CreateGEP(localState, {ZERO, POP_COUNT_STRUCT}));
}

}

#endif // PIPELINE_LOGIC_HPP
