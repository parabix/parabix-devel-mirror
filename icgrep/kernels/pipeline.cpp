/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#include <toolchain.h>
#include "pipeline.h"
#include "utf_encoding.h"

#include <IDISA/idisa_builder.h>

#include <kernels/interface.h>
#include <kernels/kernel.h>
#include <kernels/s2p_kernel.h>

#include <llvm/IR/TypeBuilder.h>

using namespace kernel;

void generatePipelineParallel(IDISA::IDISA_Builder * iBuilder, std::vector<KernelBuilder *> kernels, std::vector<Value *> instances) {
 
    Module * m = iBuilder->getModule();

    Type * pthreadTy = iBuilder->getInt64Ty(); //Pthread Type for 64-bit machine.      
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
    
    
    Value * initialBufferSize = nullptr;
    Value * initialBlockNo = nullptr;
    BasicBlock * initialBlock = nullptr;
    Value * rslt = nullptr;
    
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
}
