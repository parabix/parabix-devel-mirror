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
    Type * const int64ty = iBuilder->getInt64Ty();

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
    
    if (segmentSize > 1) {
        iBuilder->CreateBr(segmentCondBlock);
        iBuilder->SetInsertPoint(segmentCondBlock);
        PHINode * remainingBytes = iBuilder->CreatePHI(int64ty, 2, "remainingBytes");
        remainingBytes->addIncoming(fileSize, entryBlock);
        PHINode * blockNo = iBuilder->CreatePHI(int64ty, 2, "blockNo");
        blockNo->addIncoming(iBuilder->getInt64(0), entryBlock);
        
        Constant * const step = ConstantInt::get(int64ty, iBuilder->getBitBlockWidth() * segmentSize);
        Value * segmentCondTest = iBuilder->CreateICmpULT(remainingBytes, step);
        iBuilder->CreateCondBr(segmentCondTest, fullCondBlock, segmentBodyBlock);
        
        iBuilder->SetInsertPoint(segmentBodyBlock);
        Value * segBlocks = ConstantInt::get(int64ty, segmentSize);
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
        initialBlockNo = ConstantInt::get(int64ty, 0);
        initialBlock = entryBlock;
        iBuilder->CreateBr(fullCondBlock);
    }
    
    iBuilder->SetInsertPoint(fullCondBlock);
    PHINode * remainingBytes = iBuilder->CreatePHI(int64ty, 2, "remainingBytes");
    remainingBytes->addIncoming(initialBufferSize, initialBlock);
    PHINode * blockNo = iBuilder->CreatePHI(int64ty, 2, "blockNo");
    blockNo->addIncoming(initialBlockNo, initialBlock);
    
    Constant * const step = ConstantInt::get(int64ty, iBuilder->getBitBlockWidth());
    Value * fullCondTest = iBuilder->CreateICmpULT(remainingBytes, step);
    iBuilder->CreateCondBr(fullCondTest, finalBlock, fullBodyBlock);
    
    // Full Block Pipeline loop
    iBuilder->SetInsertPoint(fullBodyBlock);
    
    for (unsigned i = 0; i < kernels.size(); i++) {
        kernels[i]->createDoSegmentCall(instances[i], ConstantInt::get(int64ty, 1));
    }
    
    remainingBytes->addIncoming(iBuilder->CreateSub(remainingBytes, step), fullBodyBlock);
    blockNo->addIncoming(iBuilder->CreateAdd(blockNo, iBuilder->getInt64(1)), fullBodyBlock);
    iBuilder->CreateBr(fullCondBlock);
    
    iBuilder->SetInsertPoint(finalBlock);
    
    for (unsigned i = 0; i < kernels.size(); i++) {
        std::vector<Value *> basePtrs;
        std::vector<Value *> blockMasks;
        for (auto sSet : kernels[i]->mStreamSetInputs) {
            basePtrs.push_back(kernels[i]->getScalarField(instances[i], sSet.ssName + basePtrSuffix));
            blockMasks.push_back(kernels[i]->getScalarField(instances[i], sSet.ssName + blkMaskSuffix));
        }
        for (auto sSet : kernels[i]->mStreamSetOutputs) {
            basePtrs.push_back(kernels[i]->getScalarField(instances[i], sSet.ssName + basePtrSuffix));
            blockMasks.push_back(kernels[i]->getScalarField(instances[i], sSet.ssName + blkMaskSuffix));
        }
        std::vector<Value *> args;
        for (unsigned i = 0; i < basePtrs.size(); i++) {
            args.push_back(iBuilder->CreateGEP(basePtrs[i], iBuilder->CreateAnd(blockNo, blockMasks[i])));
        }
        kernels[i]->createFinalBlockCall(instances[i], remainingBytes, args);
    }
}
