/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#include <toolchain.h>
#include "pipeline.h"

#include <IDISA/idisa_builder.h>

#include <kernels/interface.h>
#include <kernels/kernel.h>
#include <kernels/s2p_kernel.h>

#include <llvm/IR/TypeBuilder.h>
#include <iostream>

using namespace kernel;

Function * generateSegmentParallelPipelineThreadFunction(std::string name, IDISA::IDISA_Builder * iBuilder, std::vector<KernelBuilder *> kernels, Type * sharedStructType, int id) {

    Module * m = iBuilder->getModule();
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const voidTy = Type::getVoidTy(m->getContext());
    Type * const voidPtrTy = TypeBuilder<void *, false>::get(m->getContext());
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();

    Function * const threadFunc = cast<Function>(m->getOrInsertFunction(name, voidTy, int8PtrTy, nullptr));
    threadFunc->setCallingConv(CallingConv::C);
    Function::arg_iterator args = threadFunc->arg_begin();

    Value * const input = &*(args++);
    input->setName("input");

    unsigned threadNum = codegen::ThreadNum;

     // Create the basic blocks for the thread function.
    BasicBlock * entryBlock = BasicBlock::Create(iBuilder->getContext(), "entry", threadFunc, 0);
    BasicBlock * segmentLoop = BasicBlock::Create(iBuilder->getContext(), "segmentLoop", threadFunc, 0);
    BasicBlock * exitThreadBlock = BasicBlock::Create(iBuilder->getContext(), "exitThread", threadFunc, 0);
    std::vector<BasicBlock *> segmentWait;
    std::vector<BasicBlock *> segmentLoopBody;
    for (unsigned i = 0; i < kernels.size(); i++) {
        segmentWait.push_back(BasicBlock::Create(iBuilder->getContext(), "segmentWait"+std::to_string(i), threadFunc, 0));
        segmentLoopBody.push_back(BasicBlock::Create(iBuilder->getContext(), "segmentWait"+std::to_string(i), threadFunc, 0));
    }

    iBuilder->SetInsertPoint(entryBlock);
    Value * sharedStruct = iBuilder->CreateBitCast(input, PointerType::get(sharedStructType, 0));
    Constant * myThreadId = ConstantInt::get(size_ty, id);
    std::vector<Value *> instancePtrs;
    for (unsigned i = 0; i < kernels.size(); i++) {
        Value * ptr = iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(i + 1)});
        instancePtrs.push_back(iBuilder->CreateLoad(ptr));
    }
    
    // Some important constant values.
    int segmentSize = codegen::SegmentSize;
    Constant * segmentBlocks = ConstantInt::get(size_ty, segmentSize);
    iBuilder->CreateBr(segmentLoop);

    iBuilder->SetInsertPoint(segmentLoop);
    PHINode * segNo = iBuilder->CreatePHI(size_ty, 2, "segNo");
    segNo->addIncoming(myThreadId, entryBlock);
    unsigned last_kernel = kernels.size() - 1;
    Value * alreadyDone = kernels[last_kernel]->getTerminationSignal(instancePtrs[last_kernel]);
    iBuilder->CreateCondBr(alreadyDone, exitThreadBlock, segmentWait[0]);

    for (unsigned i = 0; i < kernels.size(); i++) {
        iBuilder->SetInsertPoint(segmentWait[i]);
        Value * processedSegmentCount = kernels[i]->getLogicalSegmentNo(instancePtrs[i]);
        Value * cond = iBuilder->CreateICmpEQ(segNo, processedSegmentCount);
        iBuilder->CreateCondBr(cond, segmentLoopBody[i], segmentWait[i]);

        iBuilder->SetInsertPoint(segmentLoopBody[i]);
        kernels[i]->createDoSegmentCall(instancePtrs[i], segmentBlocks);
        if (i == last_kernel) break;
        iBuilder->CreateBr(segmentWait[i+1]);
    }
   
    segNo->addIncoming(iBuilder->CreateAdd(segNo, ConstantInt::get(size_ty, threadNum)), segmentLoopBody[last_kernel]);
    Value * endSignal = kernels[last_kernel]->getTerminationSignal(instancePtrs[last_kernel]);
    iBuilder->CreateCondBr(endSignal, exitThreadBlock, segmentLoop);
    
    iBuilder->SetInsertPoint(exitThreadBlock);
    Value * nullVal = Constant::getNullValue(voidPtrTy);
    Function * pthreadExitFunc = m->getFunction("pthread_exit");
    CallInst * exitThread = iBuilder->CreateCall(pthreadExitFunc, {nullVal});
    exitThread->setDoesNotReturn();
    iBuilder->CreateRetVoid();

    return threadFunc;
}

void generateSegmentParallelPipeline(IDISA::IDISA_Builder * iBuilder, std::vector<KernelBuilder *> kernels, std::vector<Value *> instances, Value * fileSize) {
    
    unsigned threadNum = codegen::ThreadNum;

    Module * m = iBuilder->getModule();

    Type * const size_ty = iBuilder->getSizeTy();
    Type * const voidPtrTy = TypeBuilder<void *, false>::get(m->getContext());
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();
    Type * const pthreadsTy = ArrayType::get(size_ty, threadNum);
    AllocaInst * const pthreads = iBuilder->CreateAlloca(pthreadsTy);
    std::vector<Value *> pthreadsPtrs;
    for (unsigned i = 0; i < threadNum; i++) {
        pthreadsPtrs.push_back(iBuilder->CreateGEP(pthreads, {iBuilder->getInt32(0), iBuilder->getInt32(i)}));
    }
    Value * nullVal = Constant::getNullValue(voidPtrTy);
    AllocaInst * const status = iBuilder->CreateAlloca(int8PtrTy);

    std::vector<Type *> structTypes;
    structTypes.push_back(size_ty);//input size
    for (unsigned i = 0; i < instances.size(); i++) {
        structTypes.push_back(instances[i]->getType());
    }
    Type * sharedStructType = StructType::get(m->getContext(), structTypes);

    AllocaInst * sharedStruct;
    sharedStruct = iBuilder->CreateAlloca(sharedStructType);
    Value * sizePtr = iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    iBuilder->CreateStore(fileSize, sizePtr);
    for (unsigned i = 0; i < instances.size(); i++) {
        Value * ptr = iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(i+1)});
        iBuilder->CreateStore(instances[i], ptr);
    }

    std::vector<Function *> thread_functions;
    const auto ip = iBuilder->saveIP();
    for (unsigned i = 0; i < threadNum; i++) {
        thread_functions.push_back(generateSegmentParallelPipelineThreadFunction("thread"+std::to_string(i), iBuilder, kernels, sharedStructType, i));
    }
    iBuilder->restoreIP(ip);

    Function * pthreadCreateFunc = m->getFunction("pthread_create");
    Function * pthreadJoinFunc = m->getFunction("pthread_join");

    for (unsigned i = 0; i < threadNum; i++) {
        iBuilder->CreateCall(pthreadCreateFunc, std::vector<Value *>({pthreadsPtrs[i], nullVal, thread_functions[i], iBuilder->CreateBitCast(sharedStruct, int8PtrTy)}));
    }

    std::vector<Value *> threadIDs;
    for (unsigned i = 0; i < threadNum; i++) { 
        threadIDs.push_back(iBuilder->CreateLoad(pthreadsPtrs[i]));
    }
    
    for (unsigned i = 0; i < threadNum; i++) { 
        iBuilder->CreateCall(pthreadJoinFunc, std::vector<Value *>({threadIDs[i], status}));
    }

}

void generatePipelineParallel(IDISA::IDISA_Builder * iBuilder, std::vector<KernelBuilder *> kernels, std::vector<Value *> instances) {
 
    Module * m = iBuilder->getModule();

    Type * pthreadTy = iBuilder->getSizeTy();     
    Type * const voidPtrTy = TypeBuilder<void *, false>::get(m->getContext());
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();

    Type * const pthreadsTy = ArrayType::get(pthreadTy, kernels.size());
    AllocaInst * const pthreads = iBuilder->CreateAlloca(pthreadsTy);
    std::vector<Value *> pthreadsPtrs;
    for (unsigned i = 0; i < kernels.size(); i++) {
        pthreadsPtrs.push_back(iBuilder->CreateGEP(pthreads, {iBuilder->getInt32(0), iBuilder->getInt32(i)}));
    }
    Value * nullVal = Constant::getNullValue(voidPtrTy);
    AllocaInst * const status = iBuilder->CreateAlloca(int8PtrTy);

    std::vector<Function *> kernel_functions;
    const auto ip = iBuilder->saveIP();
    for (unsigned i = 0; i < kernels.size(); i++) {
        kernel_functions.push_back(kernels[i]->generateThreadFunction("k_"+std::to_string(i)));
    }
    iBuilder->restoreIP(ip);

    Function * pthreadCreateFunc = m->getFunction("pthread_create");
    Function * pthreadJoinFunc = m->getFunction("pthread_join");

    for (unsigned i = 0; i < kernels.size(); i++) {
        iBuilder->CreateCall(pthreadCreateFunc, std::vector<Value *>({pthreadsPtrs[i], nullVal, kernel_functions[i], iBuilder->CreateBitCast(instances[i], int8PtrTy)}));
    }

    std::vector<Value *> threadIDs;
    for (unsigned i = 0; i < kernels.size(); i++) { 
        threadIDs.push_back(iBuilder->CreateLoad(pthreadsPtrs[i]));
    }
    
    for (unsigned i = 0; i < kernels.size(); i++) { 
        iBuilder->CreateCall(pthreadJoinFunc, std::vector<Value *>({threadIDs[i], status}));
    }
}


void generatePipelineLoop(IDISA::IDISA_Builder * iBuilder, std::vector<KernelBuilder *> kernels, std::vector<Value *> instances, Value * fileSize) {
    
    BasicBlock * entryBlock = iBuilder->GetInsertBlock();
    Function * main = entryBlock->getParent();
        
    const unsigned segmentSize = codegen::SegmentSize;
    Type * const size_ty = iBuilder->getSizeTy();

    // Create the basic blocks for the loop.
    BasicBlock * segmentBlock = BasicBlock::Create(iBuilder->getContext(), "segmentLoop", main, 0);
    BasicBlock * exitBlock = BasicBlock::Create(iBuilder->getContext(), "exitBlock", main, 0);
    iBuilder->CreateBr(segmentBlock);
    iBuilder->SetInsertPoint(segmentBlock);
    Constant * segBlocks = ConstantInt::get(size_ty, segmentSize * iBuilder->getStride() / iBuilder->getBitBlockWidth());
    for (unsigned i = 0; i < kernels.size(); i++) {
        kernels[i]->createDoSegmentCall(instances[i], segBlocks);
    }
    Value * endSignal = kernels[kernels.size()-1]->getTerminationSignal(instances[kernels.size()-1]);
    iBuilder->CreateCondBr(endSignal, exitBlock, segmentBlock);
    iBuilder->SetInsertPoint(exitBlock);

}
