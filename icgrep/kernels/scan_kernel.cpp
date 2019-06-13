/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "scan_kernel.h"
#include <toolchain/toolchain.h>
#include <kernels/kernel_builder.h>
#include <llvm/IR/Module.h>

using namespace llvm;

#define IS_POW_2(i) ((i > 0) && ((i & (i - 1)) == 0))

namespace kernel {

const unsigned ScanKernelBase::ScanWordContext::maxStrideWidth = 4096; // gives scan word width of 64-bits;

ScanKernelBase::ScanWordContext::ScanWordContext(BuilderRef b, unsigned strideWidth) 
: width(std::max(minScanWordWidth, strideWidth / strideMaskWidth))
, wordsPerBlock(b->getBitBlockWidth() / width)
, wordsPerStride(strideMaskWidth)
, fieldWidth(width)
, Ty(b->getIntNTy(width))
, PointerTy(Ty->getPointerTo())
, StrideMaskTy(b->getIntNTy(strideMaskWidth))
, WIDTH(b->getSize(width))
, WORDS_PER_BLOCK(b->getSize(wordsPerBlock))
, WORDS_PER_STRIDE(b->getSize(wordsPerStride))
{
    assert (IS_POW_2(strideWidth) && strideWidth >= b->getBitBlockWidth() && strideWidth <= maxStrideWidth);
}

void ScanKernelBase::initializeBase(BuilderRef b) {
    mInitialPos = b->getProcessedItemCount(mScanStreamSetName);
}

Value * ScanKernelBase::computeStridePosition(BuilderRef b, Value * strideNumber) const {
    return b->CreateAdd(mInitialPos, b->CreateMul(strideNumber, sz_STRIDE_WIDTH));
}

Value * ScanKernelBase::computeStrideBlockOffset(BuilderRef b, Value * strideNo) const {
    return b->CreateMul(strideNo, sz_NUM_BLOCKS_PER_STRIDE);
}

Value * ScanKernelBase::loadScanStreamBitBlock(BuilderRef b, Value * strideNo, Value * blockNo) {
    Value * idx = b->CreateAdd(blockNo, computeStrideBlockOffset(b, strideNo));
    return b->loadInputStreamBlock(mScanStreamSetName, b->getSize(0), idx);
}

Value * ScanKernelBase::orBlockIntoMask(BuilderRef b, ScanWordContext const & sw, Value * maskAccum, Value * block, Value * blockNo) {
    Value * const any = b->simd_any(sw.fieldWidth, block);
    Value * const signMask = b->CreateZExt(b->hsimd_signmask(sw.fieldWidth, any), sw.StrideMaskTy);
    Value * const shiftedSignMask = b->CreateShl(signMask, b->CreateZExtOrTrunc(b->CreateMul(blockNo, sw.WORDS_PER_BLOCK), sw.StrideMaskTy));
    return b->CreateOr(maskAccum, shiftedSignMask);
}

ScanKernelBase::ScanKernelBase(BuilderRef b, unsigned strideWidth, StringRef scanStreamSetName)
: mStrideWidth(strideWidth)
, mScanStreamSetName(scanStreamSetName)
, mInitialPos(nullptr)
, sz_STRIDE_WIDTH(b->getSize(strideWidth))
, sz_NUM_BLOCKS_PER_STRIDE(b->getSize(strideWidth / b->getBitBlockWidth()))
{}

void ScanKernel::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {
    ScanWordContext sw(b, mStride);
    Module * const module = b->getModule();

    Type * const sizeTy = b->getSizeTy();
    Value * const sz_ZERO = b->getSize(0);
    Value * const sz_ONE = b->getSize(1);

    Value * const ZERO_MASK = Constant::getNullValue(sw.StrideMaskTy);
    Value * const ZERO_WORD = Constant::getNullValue(sw.Ty);

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const strideInit = b->CreateBasicBlock("strideInit");
    BasicBlock * const buildMask = b->CreateBasicBlock("buildMask");
    BasicBlock * const maskReady = b->CreateBasicBlock("maskReady");
    BasicBlock * const processMask = b->CreateBasicBlock("processMask");
    BasicBlock * const processWord = b->CreateBasicBlock("processWord");
    BasicBlock * const wordDone = b->CreateBasicBlock("wordDone");
    BasicBlock * const maskDone = b->CreateBasicBlock("maskDone");
    BasicBlock * const exitBlock = b->CreateBasicBlock("exitBlock");
    
    initializeBase(b);
    b->CreateBr(strideInit);

    b->SetInsertPoint(strideInit);
    PHINode * const strideNo = b->CreatePHI(sizeTy, 2, "strideNo");
    strideNo->addIncoming(sz_ZERO, entryBlock);
    Value * const nextStrideNo = b->CreateAdd(strideNo, sz_ONE);
    b->CreateBr(buildMask);

    b->SetInsertPoint(buildMask);
    PHINode * const strideMaskAccum = b->CreatePHI(sw.StrideMaskTy, 2, "strideMaskAccum");
    strideMaskAccum->addIncoming(ZERO_MASK, strideInit);
    PHINode * const blockNo = b->CreatePHI(sizeTy, 2, "blockNo");
    blockNo->addIncoming(sz_ZERO, strideInit);
    Value * const block = loadScanStreamBitBlock(b, strideNo, blockNo);
    Value * const strideMask = orBlockIntoMask(b, sw, strideMaskAccum, block, blockNo);
    strideMaskAccum->addIncoming(strideMask, buildMask);
    Value * const nextBlockNo = b->CreateAdd(blockNo, sz_ONE);
    blockNo->addIncoming(nextBlockNo, buildMask);
    b->CreateCondBr(b->CreateICmpNE(nextBlockNo, sz_NUM_BLOCKS_PER_STRIDE), buildMask, maskReady);

    b->SetInsertPoint(maskReady);
    // Mask for this stride has been computed and is ready for processing.
    // If the mask is empty there there is nothing to be done for this stride.
    createOptimizedContinueProcessingBr(b, strideMask, processMask, maskDone);

    b->SetInsertPoint(processMask);
    // There is at least one 1 bit in the mask.
    PHINode * const processingMask = b->CreatePHI(sw.StrideMaskTy, 2, "processingMask");
    processingMask->addIncoming(strideMask, maskReady);
    Value * const wordOffset = b->CreateCountForwardZeroes(processingMask, true);
    Value * const strideOffset = b->CreateMul(strideNo, sz_NUM_BLOCKS_PER_STRIDE);
    Value * const stridePtr = b->CreateBitCast(b->getInputStreamBlockPtr(mScanStreamSetName, sz_ZERO, strideOffset), sw.PointerTy);
    Value * const wordPtr = b->CreateGEP(stridePtr, wordOffset);
    Value * const word = b->CreateLoad(wordPtr, "scanWord");
    b->CreateBr(processWord);

    b->SetInsertPoint(processWord);
    // Scan through the word and trigger a callback at the location of each bit.
    PHINode * const processingWord = b->CreatePHI(sw.Ty, 2, "processingWord");
    processingWord->addIncoming(word, processMask);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        b->CreateAssert(b->CreateICmpNE(processingWord, ZERO_WORD), "ScanKernel::processWord: processing word cannot be zero!");
    }
    Value * const bitIndex_InWord = b->CreateZExt(b->CreateCountForwardZeroes(processingWord, true), sizeTy);
    // b->CallPrintInt("bit index", bitIndex_InWord);
    Value * const wordIndex_InStride = b->CreateMul(wordOffset, sw.WIDTH);
    Value * const strideIndex = computeStridePosition(b, strideNo);
    Value * const callbackIndex = b->CreateAdd(strideIndex, b->CreateAdd(wordIndex_InStride, bitIndex_InWord), "callbackIndex");
    // b->CallPrintInt("callback index", callbackIndex);
    Value * const sourcePtr = b->getRawInputPointer("source", callbackIndex);
    Function * const callback = module->getFunction(mCallbackName); assert (callback);
    b->CreateCall(callback, {sourcePtr, callbackIndex});
    Value * const processedWord = b->CreateResetLowestBit(processingWord);
    processingWord->addIncoming(processedWord, processWord);
    // Loop back if the scan word has another 1 bit in it.
    createOptimizedContinueProcessingBr(b, processedWord, processWord, wordDone);

    b->SetInsertPoint(wordDone);
    // Finished processing the scan word. If there are more bits still in the 
    // mask loop back and process those as well.
    Value * const processedMask = b->CreateResetLowestBit(processingMask);
    processingMask->addIncoming(processedMask, wordDone);
    createOptimizedContinueProcessingBr(b, processedMask, processMask, maskDone);

    b->SetInsertPoint(maskDone);
    // Finished processing the mask and, as a result, this stride. If there are
    // still more strides avaliable, loop back and process those.
    strideNo->addIncoming(nextStrideNo, maskDone);
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), strideInit, exitBlock);

    b->SetInsertPoint(exitBlock);
}

void ScanKernel::createOptimizedContinueProcessingBr(BuilderRef b, Value * value, BasicBlock * trueBlock, BasicBlock * falseBlock) {
    switch (mOptimizeMode) {
    case OptimizeMode::Sparse:
        b->CreateUnlikelyCondBr(b->CreateICmpNE(value, Constant::getNullValue(value->getType())), trueBlock, falseBlock);
        break;
    case OptimizeMode::Dense:
        b->CreateLikelyCondBr(b->CreateICmpNE(value, Constant::getNullValue(value->getType())), trueBlock, falseBlock);
        break;
    default:
        llvm_unreachable("Invalid ScanKernel::OptimizeMode");
    }
}

static inline std::string ScanKernel_GenName(unsigned strideWidth, std::string const & callbackName) {
    return "ScanKernel_sw" + std::to_string(strideWidth) + "_" + callbackName;
}

ScanKernel::ScanKernel(BuilderRef b, StreamSet * scanStream, StreamSet * sourceStream, StringRef callbackName, OptimizeMode optimizeMode)
: ScanKernelBase(b, std::min(codegen::ScanBlocks * b->getBitBlockWidth(), ScanWordContext::maxStrideWidth), "scan")
, MultiBlockKernel(b, ScanKernel_GenName(ScanKernelBase::mStrideWidth, callbackName), 
    {{"scan", scanStream}, {"source", sourceStream}}, {}, {}, {}, {})
, mCallbackName(callbackName)
, mOptimizeMode(optimizeMode)
{
    assert (scanStream->getNumElements() == 1 && sourceStream->getNumElements() == 1 && sourceStream->getFieldWidth() == 8);
    if (!IS_POW_2(codegen::ScanBlocks)) {
        report_fatal_error("scan-blocks must be a power of 2");
    }
    if ((codegen::ScanBlocks * b->getBitBlockWidth()) > ScanWordContext::maxStrideWidth) {
        report_fatal_error("scan-blocks exceeds maximum allowed size of " + std::to_string(ScanWordContext::maxStrideWidth / b->getBitBlockWidth()));
    }
    addAttribute(SideEffecting());
    setStride(mStrideWidth);
}

}
