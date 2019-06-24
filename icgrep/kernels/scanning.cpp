/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "scanning.h"
#include <llvm/IR/Module.h>

#include <kernels/kernel_builder.h>

#define IS_POW_2(i) ((i > 0) && ((i & (i - 1)) == 0))

using namespace llvm;

namespace kernel {

generic::SingleStreamScanKernelTemplate::ScanWordContext::ScanWordContext(BuilderRef b, unsigned strideWidth) 
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
, NUM_BLOCKS_PER_STRIDE(b->getSize(strideWidth / b->getBitBlockWidth()))
{
    assert (IS_POW_2(strideWidth) && strideWidth >= b->getBitBlockWidth() && strideWidth <= MaxStrideWidth);
}

void generic::SingleStreamScanKernelTemplate::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {
    Type * const sizeTy = b->getSizeTy();
    Value * const sz_ZERO = b->getSize(0);
    Value * const sz_ONE = b->getSize(1);

    mEntryBlock = b->GetInsertBlock();
    mStrideStart = b->CreateBasicBlock("strideStart");
    mBuildStrideMask = b->CreateBasicBlock("buildStrideMask");
    mMaskReady = b->CreateBasicBlock("maskReady");
    mProcessMask = b->CreateBasicBlock("processMask");
    mProcessWord = b->CreateBasicBlock("processWord");
    mWordDone = b->CreateBasicBlock("wordDone");
    mStrideDone = b->CreateBasicBlock("strideDone");
    mExitBlock = b->CreateBasicBlock("exitBlock");
    Value * const scanStreamProcessedItemCount = b->getProcessedItemCount("scan");
    Value * const MASK_ZERO = Constant::getNullValue(mSW.StrideMaskTy);
    initialize(b);
    b->CreateBr(mStrideStart);

    b->SetInsertPoint(mStrideStart);
    PHINode * const strideNo = b->CreatePHI(sizeTy, 2, "strideNo");
    strideNo->addIncoming(sz_ZERO, mEntryBlock);
    mStrideNo = strideNo;
    Value * const nextStrideNo = b->CreateAdd(strideNo, sz_ONE, "nextStrideNo");
    willProcessStride(b, strideNo);
    b->CreateBr(mBuildStrideMask);

    b->SetInsertPoint(mBuildStrideMask);
    PHINode * maskAccumPhi = b->CreatePHI(mSW.StrideMaskTy, 2);
    maskAccumPhi->addIncoming(MASK_ZERO, mStrideStart);
    PHINode * const blockNo = b->CreatePHI(sizeTy, 2);
    blockNo->addIncoming(sz_ZERO, mStrideStart);
    mBlockNo = blockNo;
    maskBuildingIterationHead(b);
    Value * const nextBlockNo = b->CreateAdd(blockNo, sz_ONE);
    blockNo->addIncoming(nextBlockNo, mBuildStrideMask);
    Value * const blockIndex = b->CreateAdd(blockNo, b->CreateMul(strideNo, mSW.NUM_BLOCKS_PER_STRIDE));
    maskBuildingIterationBody(b, blockIndex);
    Value * const block = b->loadInputStreamBlock("scan", blockIndex);
    Value * const any = b->simd_any(mSW.fieldWidth, block);
    Value * const signMask = b->CreateZExt(b->hsimd_signmask(mSW.fieldWidth, any), mSW.StrideMaskTy);
    Value * const shiftedSignMask = b->CreateShl(signMask, b->CreateZExtOrTrunc(b->CreateMul(blockNo, mSW.WORDS_PER_BLOCK), mSW.StrideMaskTy));
    Value * const strideMask = b->CreateOr(maskAccumPhi, shiftedSignMask);
    maskAccumPhi->addIncoming(strideMask, mBuildStrideMask);
    b->CreateCondBr(b->CreateICmpNE(nextBlockNo, mSW.NUM_BLOCKS_PER_STRIDE), mBuildStrideMask, mMaskReady);

    b->SetInsertPoint(mMaskReady);
    didBuildMask(b, strideMask);
    b->CreateLikelyCondBr(b->CreateICmpNE(strideMask, MASK_ZERO), mProcessMask, mStrideDone);

    b->SetInsertPoint(mProcessMask);
    PHINode * const processingMask = b->CreatePHI(mSW.StrideMaskTy, 2, "processingMask");
    mProcessingMask = processingMask;
    processingMask->addIncoming(strideMask, mMaskReady);
    Value * const wordOffset = b->CreateCountForwardZeroes(processingMask, true);
    mWordOffset = wordOffset;
    Value * const strideOffset = b->CreateMul(strideNo, mSW.NUM_BLOCKS_PER_STRIDE);
    Value * const blockNumOfWord = b->CreateUDiv(wordOffset, mSW.WORDS_PER_BLOCK);
    Value * const blockOffset = b->CreateURem(wordOffset, mSW.WORDS_PER_BLOCK);
    Value * const processingBlockIndex = b->CreateAdd(strideOffset, blockNumOfWord);
    Value * const stridePtr = b->CreateBitCast(b->getInputStreamBlockPtr("scan", processingBlockIndex), mSW.PointerTy);
    Value * const wordPtr = b->CreateGEP(stridePtr, blockOffset);
    Value * const word = b->CreateLoad(wordPtr);
    willProcessWord(b, word);
    b->CreateBr(mProcessWord);

    b->SetInsertPoint(mProcessWord);
    PHINode * const processingWord = b->CreatePHI(mSW.Ty, 2, "processingWord");
    processingWord->addIncoming(word, mProcessMask);
    mProcessingWord = processingWord;
    Value * const bitIndex_InWord = b->CreateZExt(b->CreateCountForwardZeroes(processingWord, true), sizeTy);
    Value * const wordIndex_InStride = b->CreateMul(wordOffset, mSW.WIDTH);
    Value * const strideIndex = b->CreateAdd(scanStreamProcessedItemCount, b->CreateMul(strideNo, b->getSize(mStride)));
    Value * const absoluteWordIndex = b->CreateAdd(strideIndex, wordIndex_InStride);
    Value * const absoluteIndex = b->CreateAdd(absoluteWordIndex, bitIndex_InWord);

    generateProcessingLogic(b, absoluteIndex, processingBlockIndex, bitIndex_InWord);

    Value * const processedWord = b->CreateResetLowestBit(processingWord);
    processingWord->addIncoming(processedWord, mProcessWord);
    b->CreateCondBr(b->CreateICmpNE(processedWord, Constant::getNullValue(mSW.Ty)), mProcessWord, mWordDone);

    b->SetInsertPoint(mWordDone);
    didProcessWord(b);
    Value * const processedMask = b->CreateResetLowestBit(processingMask, "processedMask");
    processingMask->addIncoming(processedMask, mWordDone);
    b->CreateCondBr(b->CreateICmpNE(processedMask, MASK_ZERO), mProcessMask, mStrideDone);

    b->SetInsertPoint(mStrideDone);
    didProcessStride(b, strideNo);
    strideNo->addIncoming(nextStrideNo, mStrideDone);
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), mStrideStart, mExitBlock);

    b->SetInsertPoint(mExitBlock);
    finalize(b);
}

const uint32_t generic::SingleStreamScanKernelTemplate::MaxStrideWidth = 4096;

generic::SingleStreamScanKernelTemplate::SingleStreamScanKernelTemplate(BuilderRef b, std::string && name, StreamSet * scan, StreamSet * source)
: MultiBlockKernel(b, name + "_sb" + std::to_string(codegen::ScanBlocks), {{"scan", scan, FixedRate(1), ZeroExtended()}, {"source", source}}, {}, {}, {}, {})
, mSW(b, std::min(codegen::ScanBlocks * b->getBitBlockWidth(), MaxStrideWidth))
{
    assert (source->getNumElements() == 1 && source->getFieldWidth() == 8);
    uint32_t strideWidth = std::min(codegen::ScanBlocks * b->getBitBlockWidth(), MaxStrideWidth);
    if (!IS_POW_2(codegen::ScanBlocks)) {
        report_fatal_error("scan-blocks must be a power of 2");
    }
    if ((codegen::ScanBlocks * b->getBitBlockWidth()) > MaxStrideWidth) {
        report_fatal_error("scan-blocks exceeds maximum allowed size of " + std::to_string(MaxStrideWidth / b->getBitBlockWidth()));
    }
    setStride(strideWidth);
}



/* ----------------------------------------------------------------------------------------------------------------- *
 * ScanPositionGenerator
 * ----------------------------------------------------------------------------------------------------------------- */

void ScanPositionGenerator::initialize(BuilderRef b) {
    const uint32_t BITS_PER_BYTE = 8;
    mLineCountArrayBlockPtr = b->CreateAlignedAlloca(b->getBitBlockType(), b->getBitBlockWidth() / BITS_PER_BYTE, mSW.NUM_BLOCKS_PER_STRIDE);
}

void ScanPositionGenerator::willProcessStride(BuilderRef b, Value * const /*strideNo*/) {
    mInitialLineNum = b->getScalarField("finalStrideLineNum");
}

void ScanPositionGenerator::maskBuildingIterationHead(BuilderRef b) {
    mBaseCounts = b->CreatePHI(b->getBitBlockType(), 2);
    mBaseCounts->addIncoming(b->allZeroes(), mStrideStart);
}

void ScanPositionGenerator::maskBuildingIterationBody(BuilderRef b, Value * const blockIndex) {
    Value * const breakBlock = b->loadInputStreamBlock("lines", blockIndex);
    Value * breakCounts = b->hsimd_partial_sum(mSW.width, b->simd_popcount(mSW.width, breakBlock));
    breakCounts = b->simd_add(mSW.width, breakCounts, mBaseCounts);
    b->CreateBlockAlignedStore(b->bitCast(breakCounts), b->CreateGEP(mLineCountArrayBlockPtr, mBlockNo));
    mHighestLineCount = b->mvmd_extract(mSW.width, breakCounts, b->getBitBlockWidth()/mSW.width - 1);
    Value * nextBaseCounts = b->bitCast(b->simd_fill(mSW.width, mHighestLineCount));
    mBaseCounts->addIncoming(nextBaseCounts, mBuildStrideMask);
}

void ScanPositionGenerator::generateProcessingLogic(BuilderRef b, Value * const absoluteIndex,  Value * const blockIndex, Value * const bitOffset) {
    Value * const producedItemCount = b->getProducedItemCount("output");
    b->setProducedItemCount("output", b->CreateAdd(producedItemCount, b->getSize(1)));
    Value * const val = b->CreateZExtOrBitCast(absoluteIndex, b->getInt64Ty());
    b->CreateStore(val, b->getRawOutputPointer("output", b->getInt32(0), producedItemCount));
    Value * const blockNo = b->CreateUDiv(mWordOffset, mSW.WORDS_PER_BLOCK);
    Value * const wordIndex = b->CreateURem(mWordOffset, mSW.WORDS_PER_BLOCK);
    Value * lineCount = b->CreateLoad(b->CreateGEP(mLineCountArrayBlockPtr, blockNo));
    lineCount = b->CreateExtractElement(b->fwCast(mSW.width, lineCount), wordIndex);
    // It is possible that there are break positions in this current scan word
    // which come after the scan-bit possition but are included in the value
    // of lineCount. We need to subtract the number of such break positions to
    // get the correct line number.
    Value * const breaksBlockPtr = b->CreateBitCast(b->getInputStreamBlockPtr("lines", blockIndex), mSW.PointerTy);
    Value * const breaksWord = b->CreateLoad(b->CreateGEP(breaksBlockPtr, wordIndex));
    Value * const highMask = b->CreateNot(b->CreateMaskToLowestBitInclusive(mProcessingWord));
    Value * const maskedBreaksWord = b->CreateAnd(breaksWord, highMask);
    lineCount = b->CreateSub(lineCount, b->CreatePopcount(maskedBreaksWord));
    lineCount = b->CreateZExt(lineCount, b->getInt64Ty());
    lineCount = b->CreateAdd(mInitialLineNum, lineCount);
    b->CreateStore(lineCount, b->getRawOutputPointer("output", b->getInt32(1), producedItemCount));
}

void ScanPositionGenerator::didProcessStride(BuilderRef b, Value * const /*strideNo*/) {
    Value * nextFinalStrideLineNum = b->CreateZExtOrTrunc(mHighestLineCount, b->getInt64Ty());
    nextFinalStrideLineNum = b->CreateAdd(mInitialLineNum, nextFinalStrideLineNum);
    b->setScalarField("finalStrideLineNum", nextFinalStrideLineNum);
}

ScanPositionGenerator::ScanPositionGenerator(BuilderRef b, StreamSet * scan, StreamSet * linebreakStream, StreamSet * source, StreamSet * output)
: generic::SingleStreamScanKernelTemplate(b, "ScanPositionGenerator", scan, source)
{
    assert (linebreakStream->getNumElements() == 1 && linebreakStream->getFieldWidth() == 1);
    addInternalScalar(b->getInt64Ty(), "finalStrideLineNum");
    mInputStreamSets.push_back({"lines", linebreakStream});
    mOutputStreamSets.push_back({"output", output, PopcountOf("scan")});
}



/* ----------------------------------------------------------------------------------------------------------------- *
 * LineSpanGenerator
 * ----------------------------------------------------------------------------------------------------------------- */

void LineSpanGenerator::generateProcessingLogic(BuilderRef b, Value * const absoluteIndex) {
    Value * const producedCount = b->getProducedItemCount("output");
    Value * const beginIdx = b->getScalarField("lineBegin");
    Value * const endIdx = b->CreateZExtOrBitCast(absoluteIndex, b->getInt64Ty());
    Value * const beginStorePtr = b->getRawOutputPointer("output", b->getInt32(0), producedCount);
    Value * const endStorePtr = b->getRawOutputPointer("output", b->getInt32(1), producedCount);
    b->CreateStore(beginIdx, beginStorePtr);
    b->CreateStore(endIdx, endStorePtr);
    b->setScalarField("lineBegin", b->CreateAdd(endIdx, b->getInt64(1)));
    b->setProducedItemCount("output", b->CreateAdd(producedCount, b->getSize(1)));
}

LineSpanGenerator::LineSpanGenerator(BuilderRef b, StreamSet * linebreakStream, StreamSet * source, StreamSet * output)
: generic::SingleStreamScanKernelTemplate(b, "LineSpanGenerator", linebreakStream, source)
{
    assert (linebreakStream->getNumElements() == 1 && linebreakStream->getFieldWidth() == 1);
    addInternalScalar(b->getInt64Ty(), "lineBegin");
    mOutputStreamSets.push_back({"output", output, PopcountOf("scan")});
}



/* ----------------------------------------------------------------------------------------------------------------- *
 * LineBasedScanPositionReader
 * ----------------------------------------------------------------------------------------------------------------- */

void LineBasedScanPositionReader::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {
    Module * const module = b->getModule();

    Type * const i64Ty = b->getInt64Ty();
    Value * const i64_ZERO = b->getInt64(0);
    Value * const i64_ONE = b->getInt64(1);

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const processScanPosition = b->CreateBasicBlock("processScanPosition");
    BasicBlock * const skipLineSpan = b->CreateBasicBlock("skipLineSpan");
    BasicBlock * const triggerCallback = b->CreateBasicBlock("triggerCallback");
    BasicBlock * const finalizeBlock = b->CreateBasicBlock("finalizeBlock");
    BasicBlock * const exitBlock = b->CreateBasicBlock("exitBlock");
    Value * const initialProcessedScanValue = b->getProcessedItemCount("scan");
    Value * const initialProcessedLineValue = b->getProcessedItemCount("lines");
    Value * const isInvalidFinalItem = b->CreateAnd(mIsFinal, b->CreateICmpEQ(i64_ZERO, b->CreateLoad(b->getRawInputPointer("scan", b->getInt32(1), initialProcessedScanValue))));
    b->CreateCondBr(isInvalidFinalItem, exitBlock, processScanPosition);

    b->SetInsertPoint(processScanPosition);
    PHINode * const strideNo = b->CreatePHI(i64Ty, 3);
    strideNo->addIncoming(i64_ZERO, entryBlock);
    PHINode * const scanIndex = b->CreatePHI(i64Ty, 3);
    scanIndex->addIncoming(initialProcessedScanValue, entryBlock);
    PHINode * const lineIndex = b->CreatePHI(i64Ty, 3);
    lineIndex->addIncoming(initialProcessedLineValue, entryBlock);
    Value * const lineNumber = b->CreateLoad(b->getRawInputPointer("scan", b->getInt32(1), scanIndex));
    // We need to match lineNumber with lineIndex to retrieve the start and end
    // pointers for the line. If their values don't match now, we keep skipping
    // over line spans until they do.
    b->CreateCondBr(b->CreateICmpEQ(lineNumber, lineIndex), triggerCallback, skipLineSpan);

    b->SetInsertPoint(triggerCallback);
    Value * const scanVal = b->CreateLoad(b->getRawInputPointer("scan", b->getInt32(0), scanIndex));
    Value * const scanPtr = b->getRawInputPointer("source", scanVal);
    Value * const beginVal = b->CreateLoad(b->getRawInputPointer("lines", b->getInt32(0), lineIndex));
    Value * const beginPtr = b->getRawInputPointer("source", beginVal);
    Value * const endVal = b->CreateLoad(b->getRawInputPointer("lines", b->getInt32(1), lineIndex));
    Value * const endPtr = b->getRawInputPointer("source", endVal);
    Function * const callback = module->getFunction(mCallbackName); assert (callback);
    Value * const oneIndexedLineNumber = b->CreateAdd(lineNumber, i64_ONE);
    b->CreateCall(callback, {scanPtr, beginPtr, endPtr, oneIndexedLineNumber});
    // Increment scanIndex but not lineIndex as there may be multiple scan
    // positions on this line.
    Value * const nextScanIndex = b->CreateAdd(scanIndex, i64_ONE);
    Value * const tc_NextStrideNo = b->CreateUMax(b->CreateSub(nextScanIndex, initialProcessedScanValue), b->CreateSub(lineIndex, initialProcessedLineValue));
    strideNo->addIncoming(tc_NextStrideNo, triggerCallback);
    scanIndex->addIncoming(nextScanIndex, triggerCallback);
    lineIndex->addIncoming(lineIndex, triggerCallback);
    // If we have more strides to work with, loop back with the incremented
    // index and try again, if not, then we set processed item counts and stop
    // processing.
    b->CreateCondBr(b->CreateICmpNE(tc_NextStrideNo, b->CreateZExtOrBitCast(numOfStrides, i64Ty)), processScanPosition, finalizeBlock);


    b->SetInsertPoint(skipLineSpan);
    // Skip over this line span as its index doesn't match the needed line #.
    Value * const nextLineIndex = b->CreateAdd(lineIndex, i64_ONE);
    Value * const sl_NextStrideNo = b->CreateUMax(b->CreateSub(scanIndex, initialProcessedScanValue), b->CreateSub(nextLineIndex, initialProcessedLineValue));
    strideNo->addIncoming(sl_NextStrideNo, skipLineSpan);
    scanIndex->addIncoming(scanIndex, skipLineSpan);
    lineIndex->addIncoming(nextLineIndex, skipLineSpan);
    // If we have more strides to work with, loop back with the incremented
    // index and try again, if not, then we set processed item counts and stop
    // processing.
    b->CreateCondBr(b->CreateICmpNE(sl_NextStrideNo, b->CreateZExtOrBitCast(numOfStrides, i64Ty)), processScanPosition, finalizeBlock);

    b->SetInsertPoint(finalizeBlock);
    PHINode * finalScanIndex = b->CreatePHI(i64Ty, 2);
    finalScanIndex->addIncoming(scanIndex, skipLineSpan);
    finalScanIndex->addIncoming(nextScanIndex, triggerCallback);
    PHINode * finalLineIndex = b->CreatePHI(i64Ty, 2);
    finalLineIndex->addIncoming(lineIndex, triggerCallback);
    finalLineIndex->addIncoming(nextLineIndex, skipLineSpan);
    b->setProcessedItemCount("scan", finalScanIndex);
    b->setProcessedItemCount("lines", finalLineIndex);
    b->CreateBr(exitBlock);

    b->SetInsertPoint(exitBlock);
}

LineBasedScanPositionReader::LineBasedScanPositionReader(BuilderRef b, StreamSet * scanPositions, StreamSet * lineSpans, StreamSet * source, StringRef callbackName)
: MultiBlockKernel(b, "LineBasedScanPositionReader_" + std::string(callbackName), 
    {{"scan", scanPositions, BoundedRate(0, 1), Principal()}, {"lines", lineSpans, BoundedRate(0, 1)}, {"source", source}}, {}, {}, {}, {})
, mCallbackName(callbackName)
{
    assert (scanPositions->getNumElements() == 2 && scanPositions->getFieldWidth() == 64);
    assert (lineSpans->getNumElements() == 2 && lineSpans->getFieldWidth() == 64);
    addAttribute(SideEffecting());
    setStride(1);
}

} // namespace kernel
