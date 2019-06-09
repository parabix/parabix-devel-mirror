/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "scan_kernel.h"

#include <kernels/kernel_builder.h>
#include <llvm/IR/Module.h>

#define MASK_SIZE 64
#define STRIDE_WIDTH 32

using namespace llvm;

namespace kernel {

void ScanKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfStrides) {
    Module * const module = b->getModule();

    const unsigned blockWidth = b->getBitBlockWidth();
    const unsigned numStridesPerBlock = blockWidth / STRIDE_WIDTH;
    const unsigned numBlocksPerMask = MASK_SIZE / numStridesPerBlock;
    Constant * const sz_NUM_BLOCKS_PER_MASK = b->getSize(numBlocksPerMask);
    Constant * const sz_STRIDE_WIDTH = b->getSize(STRIDE_WIDTH);
    Constant * const sz_BLOCK_WIDTH = b->getSize(blockWidth);

    Type * const sizeTy = b->getSizeTy();
    Constant * const sz_ZERO = b->getSize(0);
    Constant * const sz_ONE = b->getSize(1);

    Type * const maskTy = b->getIntNTy(MASK_SIZE);
    Constant * const m_ZERO = b->getIntN(MASK_SIZE, 0);

    Type * const strideTy = b->getIntNTy(STRIDE_WIDTH);
    Constant * const s_ZERO = b->getIntN(STRIDE_WIDTH, 0);

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const buildMask = b->CreateBasicBlock("buildMask");
    BasicBlock * const moreMask = b->CreateBasicBlock("moreMask");
    BasicBlock * const partialMask = b->CreateBasicBlock("partialMask");
    BasicBlock * const maskReady = b->CreateBasicBlock("maskReady");
    BasicBlock * const processMask = b->CreateBasicBlock("processMask");
    BasicBlock * const processStride = b->CreateBasicBlock("processStride");
    BasicBlock * const strideDone = b->CreateBasicBlock("strideDone");
    BasicBlock * const maskDone = b->CreateBasicBlock("maskDone");
    BasicBlock * const exitBlock = b->CreateBasicBlock("exitBlock");
    b->CreateBr(buildMask);

    b->SetInsertPoint(buildMask);
    PHINode * const blockNo = b->CreatePHI(sizeTy, 3, "blockNo");
    blockNo->addIncoming(sz_ZERO, entryBlock);
    PHINode * const maskNo = b->CreatePHI(sizeTy, 3, "maskNo");
    maskNo->addIncoming(sz_ZERO, entryBlock);
    PHINode * const streamOffset = b->CreatePHI(sizeTy, 3, "streamOffset");
    streamOffset->addIncoming(sz_ZERO, entryBlock);
    PHINode * const mask = b->CreatePHI(maskTy, 3, "mask");
    mask->addIncoming(m_ZERO, entryBlock);
    Value * const nextBlockNo = b->CreateAdd(blockNo, sz_ONE);
    blockNo->addIncoming(nextBlockNo, moreMask);
    Value * const nextMaskNo = b->CreateAdd(maskNo, sz_ONE);
    Value * const nextStreamOffset = b->CreateAdd(streamOffset, sz_ONE);
    Value * const block = b->loadInputStreamBlock("scan", streamOffset);
    Value * const any = b->simd_any(STRIDE_WIDTH, block);
    Value * const blockMask = b->CreateZExt(b->hsimd_signmask(STRIDE_WIDTH, any), maskTy);
    Value * const combinedMask = b->CreateOr(mask, b->CreateShl(blockMask, b->CreateZExtOrTrunc(b->CreateMul(blockNo, sz_NUM_BLOCKS_PER_MASK), maskTy)));
    b->CreateCondBr(b->CreateICmpEQ(nextStreamOffset, numOfStrides), partialMask, moreMask);

    b->SetInsertPoint(moreMask);
    maskNo->addIncoming(maskNo, moreMask);
    streamOffset->addIncoming(nextStreamOffset, moreMask);
    mask->addIncoming(combinedMask, moreMask);
    b->CreateCondBr(b->CreateICmpNE(nextBlockNo, sz_NUM_BLOCKS_PER_MASK), buildMask, maskReady);

    b->SetInsertPoint(partialMask);
    b->CreateZExt(combinedMask, maskTy);
    b->CreateBr(maskReady);


    b->SetInsertPoint(maskReady);
    b->CreateLikelyCondBr(b->CreateICmpEQ(combinedMask, m_ZERO), maskDone, processMask);

    b->SetInsertPoint(processMask);
    PHINode * const maskPhi = b->CreatePHI(maskTy, 2, "maskPhi");
    maskPhi->addIncoming(combinedMask, maskReady);
    Value * const strideNo_InMask = b->CreateCountForwardZeroes(maskPhi, true);
    Value * const blockNo_InMask = b->CreateUDiv(strideNo_InMask, sz_NUM_BLOCKS_PER_MASK);
    Value * const strideNo_InBlock = b->CreateURem(strideNo_InMask, sz_NUM_BLOCKS_PER_MASK);
    Value * const blockNo_InStream = b->CreateAdd(b->CreateMul(maskNo, sz_NUM_BLOCKS_PER_MASK), blockNo_InMask);
    Value * const stridePtr = b->CreateBitCast(b->getInputStreamBlockPtr("scan", blockNo_InStream), strideTy->getPointerTo());
    Value * const stride = b->CreateLoad(b->CreateGEP(stridePtr, strideNo_InBlock));
    Value * const processedMask = b->CreateResetLowestBit(maskPhi);
    b->CreateBr(processStride);

    b->SetInsertPoint(processStride);
    PHINode * const stridePhi = b->CreatePHI(strideTy, 2, "stridePhi");
    stridePhi->addIncoming(stride, processMask);
    Value * const bitPos_InStride = b->CreateZExt(b->CreateCountForwardZeroes(stridePhi, true), sizeTy);
    Value * const bitPos_InBlock = b->CreateAdd(b->CreateMul(strideNo_InBlock, sz_STRIDE_WIDTH), bitPos_InStride);
    Value * const bitPos_InStream = b->CreateAdd(b->CreateMul(blockNo_InStream, sz_BLOCK_WIDTH), bitPos_InBlock);
    Value * const sourceStreamBlockPtr = b->CreatePointerCast(b->getInputStreamBlockPtr("source", blockNo_InStream), b->getInt8PtrTy());
    Value * const charPtr = b->CreateGEP(sourceStreamBlockPtr, bitPos_InBlock);
    Function * const callback = module->getFunction(mCallbackName); assert (callback);
    b->CreateCall(callback, {charPtr, bitPos_InStream});
    Value * const processedStride = b->CreateResetLowestBit(stridePhi);
    stridePhi->addIncoming(processedStride, processStride);
    b->CreateUnlikelyCondBr(b->CreateICmpNE(processedStride, s_ZERO), processStride, strideDone);

    b->SetInsertPoint(strideDone);
    maskPhi->addIncoming(processedMask, strideDone);
    b->CreateCondBr(b->CreateICmpNE(processedMask, m_ZERO), processMask, maskDone);

    b->SetInsertPoint(maskDone);
    maskNo->addIncoming(nextMaskNo, maskDone);
    blockNo->addIncoming(sz_ZERO, maskDone);
    streamOffset->addIncoming(nextStreamOffset, maskDone);
    mask->addIncoming(m_ZERO, maskDone);
    b->CreateLikelyCondBr(b->CreateICmpNE(nextStreamOffset, numOfStrides), buildMask, exitBlock);

    b->SetInsertPoint(exitBlock);
}

ScanKernel::ScanKernel(const std::unique_ptr<KernelBuilder> & iBuilder, StreamSet * scanStream, StreamSet * sourceStream, StringRef callbackName)
: MultiBlockKernel(iBuilder, "ScanKernel_" + std::string(callbackName), {{"scan", scanStream}, {"source", sourceStream}}, {}, {}, {}, {})
, mCallbackName(callbackName)
{
    assert (scanStream->getNumElements() == 1 && sourceStream->getNumElements() == 1);
    addAttribute(SideEffecting());
}

}
