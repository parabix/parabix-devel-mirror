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
    BasicBlock * segmentLoop = BasicBlock::Create(iBuilder->getContext(), "segmentCond", threadFunc, 0);
    BasicBlock * finalSegmentLoopExit = BasicBlock::Create(iBuilder->getContext(), "partialSegmentCond", threadFunc, 0);
    BasicBlock * exitThreadBlock = BasicBlock::Create(iBuilder->getContext(), "exitThread", threadFunc, 0);
    std::vector<BasicBlock *> segmentWait;
    std::vector<BasicBlock *> segmentLoopBody;
    std::vector<BasicBlock *> partialSegmentWait;
    std::vector<BasicBlock *> partialSegmentLoopBody;
    for (unsigned i = 0; i < kernels.size(); i++) {
        segmentWait.push_back(BasicBlock::Create(iBuilder->getContext(), "segmentWait"+std::to_string(i), threadFunc, 0));
        segmentLoopBody.push_back(BasicBlock::Create(iBuilder->getContext(), "segmentWait"+std::to_string(i), threadFunc, 0));
        partialSegmentWait.push_back(BasicBlock::Create(iBuilder->getContext(), "partialSegmentWait"+std::to_string(i), threadFunc, 0));
        partialSegmentLoopBody.push_back(BasicBlock::Create(iBuilder->getContext(), "partialSegmentLoopBody"+std::to_string(i), threadFunc, 0));
    }

    iBuilder->SetInsertPoint(entryBlock);
    Value * sharedStruct = iBuilder->CreateBitCast(input, PointerType::get(sharedStructType, 0));
    Value * myThreadId = ConstantInt::get(size_ty, id);
    Value * fileSize = iBuilder->CreateLoad(iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(0)}));
    std::vector<Value *> instancePtrs;
    for (unsigned i = 0; i < kernels.size(); i++) {
        Value * ptr = iBuilder->CreateGEP(sharedStruct, {iBuilder->getInt32(0), iBuilder->getInt32(i + 1)});
        instancePtrs.push_back(iBuilder->CreateLoad(ptr));
    }
    
    // Some important constant values.
    int segmentSize = codegen::SegmentSize;
    Constant * segmentBlocks = ConstantInt::get(size_ty, segmentSize);
    Constant * segmentBytes = ConstantInt::get(size_ty, iBuilder->getStride() * segmentSize);
    Constant * hypersegmentBytes = ConstantInt::get(size_ty, iBuilder->getStride() * segmentSize * threadNum);
    Constant * const blockSize = ConstantInt::get(size_ty, iBuilder->getStride());

    Value * myFirstSegNo = myThreadId;  // 
    // The offset of my starting segment within the thread group hypersegment.
    Value * myOffset = iBuilder->CreateMul(segmentBytes, myThreadId);
    Value * fullSegLimit = iBuilder->CreateAdd(myOffset, segmentBytes);

    iBuilder->CreateBr(segmentLoop);

    iBuilder->SetInsertPoint(segmentLoop);
    PHINode * remainingBytes = iBuilder->CreatePHI(size_ty, 2, "remainingBytes");
    remainingBytes->addIncoming(fileSize, entryBlock);
    PHINode * segNo = iBuilder->CreatePHI(size_ty, 2, "segNo");
    segNo->addIncoming(myFirstSegNo, entryBlock);

    Value * LT_fullSegment = iBuilder->CreateICmpSLT(remainingBytes, fullSegLimit);
    iBuilder->CreateCondBr(LT_fullSegment, finalSegmentLoopExit, segmentWait[0]);

    for (unsigned i = 0; i < kernels.size(); i++) {
        iBuilder->SetInsertPoint(segmentWait[i]);
        Value * processedSegmentCount = kernels[i]->getLogicalSegmentNo(instancePtrs[i]);
        Value * cond = iBuilder->CreateICmpEQ(segNo, processedSegmentCount);
        iBuilder->CreateCondBr(cond, segmentLoopBody[i], segmentWait[i]);

        iBuilder->SetInsertPoint(segmentLoopBody[i]);
        kernels[i]->createDoSegmentCall(instancePtrs[i], segmentBlocks);
        if (i == kernels.size() - 1) break;
        iBuilder->CreateBr(segmentWait[i+1]);
    }
   
    remainingBytes->addIncoming(iBuilder->CreateSub(remainingBytes, hypersegmentBytes), segmentLoopBody[kernels.size()-1]);
    segNo->addIncoming(iBuilder->CreateAdd(segNo, ConstantInt::get(size_ty, threadNum)), segmentLoopBody[kernels.size()-1]);
    iBuilder->CreateBr(segmentLoop);

    // Now we may have a partial segment, or we may be completely done
    // because the last segment was handled by a previous thread in the group.
    iBuilder->SetInsertPoint(finalSegmentLoopExit);
    Value * alreadyDone = iBuilder->CreateICmpSLT(remainingBytes, myOffset);
    Value * remainingForMe = iBuilder->CreateSub(remainingBytes, myOffset);
    Value * blocksToDo = iBuilder->CreateUDiv(remainingForMe, blockSize);
    iBuilder->CreateCondBr(alreadyDone, exitThreadBlock, partialSegmentWait[0]);

    // Full Block Pipeline loop
    for (unsigned i = 0; i < kernels.size(); i++) {
        iBuilder->SetInsertPoint(partialSegmentWait[i]);
        Value * processedSegmentCount = kernels[i]->getLogicalSegmentNo(instancePtrs[i]);
        Value * cond = iBuilder->CreateICmpEQ(segNo, processedSegmentCount);
        iBuilder->CreateCondBr(cond, partialSegmentLoopBody[i], partialSegmentWait[i]);

        iBuilder->SetInsertPoint(partialSegmentLoopBody[i]);
        kernels[i]->createDoSegmentCall(instancePtrs[i], blocksToDo);
        kernels[i]->createFinalBlockCall(instancePtrs[i], iBuilder->CreateURem(remainingForMe, blockSize));
        if (i == kernels.size() - 1) break;
        iBuilder->CreateBr(partialSegmentWait[i+1]);
    }
    iBuilder->CreateBr(exitThreadBlock);

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
    BasicBlock * segmentCondBlock = nullptr;
    BasicBlock * segmentBodyBlock = nullptr;
    if (segmentSize > 1) {
        segmentCondBlock = BasicBlock::Create(iBuilder->getContext(), "segmentCond", main, 0);
        segmentBodyBlock = BasicBlock::Create(iBuilder->getContext(), "segmentBody", main, 0);
    }
    BasicBlock * fullCondBlock = BasicBlock::Create(iBuilder->getContext(), "fullCond", main, 0);
    BasicBlock * fullBodyBlock = BasicBlock::Create(iBuilder->getContext(), "fullBody", main, 0);
    BasicBlock * finalBlock = BasicBlock::Create(iBuilder->getContext(), "final", main, 0);
    BasicBlock * exitBlock = BasicBlock::Create(iBuilder->getContext(), "exit", main, 0);
    
    
    Value * initialBufferSize = nullptr;
    Value * initialBlockNo = nullptr;
    BasicBlock * initialBlock = nullptr;
    
    if (segmentSize > 1) {
        iBuilder->CreateBr(segmentCondBlock);
        iBuilder->SetInsertPoint(segmentCondBlock);
        PHINode * remainingBytes = iBuilder->CreatePHI(size_ty, 2, "remainingBytes");
        remainingBytes->addIncoming(fileSize, entryBlock);
        PHINode * blockNo = iBuilder->CreatePHI(size_ty, 2, "blockNo");
        blockNo->addIncoming(ConstantInt::get(size_ty, 0), entryBlock);
        
        Constant * const step = ConstantInt::get(size_ty, iBuilder->getStride() * segmentSize);
        Value * segmentCondTest = iBuilder->CreateICmpULT(remainingBytes, step);
        iBuilder->CreateCondBr(segmentCondTest, fullCondBlock, segmentBodyBlock);
        
        iBuilder->SetInsertPoint(segmentBodyBlock);
        Value * segBlocks = ConstantInt::get(size_ty, segmentSize);
        for (unsigned i = 0; i < kernels.size(); i++) {
            kernels[i]->createDoSegmentCall(instances[i], segBlocks);
        }
        remainingBytes->addIncoming(iBuilder->CreateSub(remainingBytes, step), segmentBodyBlock);
        blockNo->addIncoming(iBuilder->CreateAdd(blockNo, segBlocks), segmentBodyBlock);
        
        iBuilder->CreateBr(segmentCondBlock);
        initialBufferSize = remainingBytes;
        initialBlockNo = blockNo;
        initialBlock = segmentCondBlock;
    } else {
        initialBufferSize = fileSize;
        initialBlockNo = ConstantInt::get(size_ty, 0);
        initialBlock = entryBlock;
        iBuilder->CreateBr(fullCondBlock);
    }
    
    iBuilder->SetInsertPoint(fullCondBlock);
    PHINode * remainingBytes = iBuilder->CreatePHI(size_ty, 2, "remainingBytes");
    remainingBytes->addIncoming(initialBufferSize, initialBlock);
    PHINode * blockNo = iBuilder->CreatePHI(size_ty, 2, "blockNo");
    blockNo->addIncoming(initialBlockNo, initialBlock);
    
    Constant * const step = ConstantInt::get(size_ty, iBuilder->getStride());
    Value * fullCondTest = iBuilder->CreateICmpULT(remainingBytes, step);
    iBuilder->CreateCondBr(fullCondTest, finalBlock, fullBodyBlock);
    
    // Full Block Pipeline loop
    iBuilder->SetInsertPoint(fullBodyBlock);
    for (unsigned i = 0; i < kernels.size(); i++) {
        kernels[i]->createDoSegmentCall(instances[i], ConstantInt::get(size_ty, 1));
    }
    
    remainingBytes->addIncoming(iBuilder->CreateSub(remainingBytes, step), fullBodyBlock);
    blockNo->addIncoming(iBuilder->CreateAdd(blockNo, ConstantInt::get(size_ty, 1)), fullBodyBlock);
    iBuilder->CreateBr(fullCondBlock);
    
    iBuilder->SetInsertPoint(finalBlock);
    for (unsigned i = 0; i < kernels.size(); i++) {
        kernels[i]->createFinalBlockCall(instances[i], remainingBytes);
    }
    iBuilder->CreateBr(exitBlock);
    iBuilder->SetInsertPoint(exitBlock);

}
