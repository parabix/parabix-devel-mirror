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


using namespace kernel;


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
        
        Constant * const step = ConstantInt::get(size_ty, iBuilder->getBitBlockWidth() * segmentSize);
        Value * segmentCondTest = iBuilder->CreateICmpULT(remainingBytes, step);
        iBuilder->CreateCondBr(segmentCondTest, fullCondBlock, segmentBodyBlock);
        
        iBuilder->SetInsertPoint(segmentBodyBlock);
        Value * segBlocks = ConstantInt::get(size_ty, segmentSize);
        Value * rslt = kernels[0]->createDoSegmentCall(instances[0], segBlocks);
        for (unsigned i = 1; i < kernels.size(); i++) {
            rslt = kernels[i]->createDoSegmentCall(instances[i], rslt->getType()->isVoidTy() ? segBlocks : rslt);
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
    
    Constant * const step = ConstantInt::get(size_ty, iBuilder->getBitBlockWidth());
    Value * fullCondTest = iBuilder->CreateICmpULT(remainingBytes, step);
    iBuilder->CreateCondBr(fullCondTest, finalBlock, fullBodyBlock);
    
    // Full Block Pipeline loop
    iBuilder->SetInsertPoint(fullBodyBlock);
    rslt = kernels[0]->createDoSegmentCall(instances[0], ConstantInt::get(size_ty, 1));
    for (unsigned i = 1; i < kernels.size(); i++) {
        rslt = kernels[i]->createDoSegmentCall(instances[i], rslt->getType()->isVoidTy() ? ConstantInt::get(size_ty, 1) : rslt);
    }
    
    remainingBytes->addIncoming(iBuilder->CreateSub(remainingBytes, step), fullBodyBlock);
    blockNo->addIncoming(iBuilder->CreateAdd(blockNo, ConstantInt::get(size_ty, 1)), fullBodyBlock);
    iBuilder->CreateBr(fullCondBlock);
    
    iBuilder->SetInsertPoint(finalBlock);
    rslt = kernels[0]-> createFinalBlockCall(instances[0], remainingBytes);
    for (unsigned i = 1; i < kernels.size(); i++) {
        kernels[i]->createFinalBlockCall(instances[i], rslt->getType()->isVoidTy() ? remainingBytes : rslt);
    }
}
