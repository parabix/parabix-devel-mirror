/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/scan/line_number_generator.h>

#include <kernel/core/kernel_builder.h>

using namespace llvm;

namespace kernel {

void LineNumberGenerator::initialize(BuilderRef b) {
    const uint32_t BITS_PER_BYTE = 8;
    mLineCountArrayBlockPtr = b->CreateAlignedAlloca(b->getBitBlockType(), b->getBitBlockWidth() / BITS_PER_BYTE, mSW.NUM_BLOCKS_PER_STRIDE);
}

void LineNumberGenerator::willProcessStride(BuilderRef b, Value * const /*strideNo*/) {
    mInitialLineNum = b->getScalarField("finalStrideLineNum");
}

void LineNumberGenerator::maskBuildingIterationHead(BuilderRef b) {
    mBaseCounts = b->CreatePHI(b->getBitBlockType(), 2);
    mBaseCounts->addIncoming(b->allZeroes(), mStrideStart);
}

void LineNumberGenerator::maskBuildingIterationBody(BuilderRef b, Value * const blockIndex) {
    Value * const breakBlock = b->loadInputStreamBlock("lines", b->getInt32(0), blockIndex);
    Value * breakCounts = b->hsimd_partial_sum(mSW.width, b->simd_popcount(mSW.width, breakBlock));
    breakCounts = b->simd_add(mSW.width, breakCounts, mBaseCounts);
    b->CreateBlockAlignedStore(b->bitCast(breakCounts), b->CreateGEP(mLineCountArrayBlockPtr, mBlockNo));
    mHighestLineCount = b->mvmd_extract(mSW.width, breakCounts, b->getBitBlockWidth()/mSW.width - 1);
    Value * nextBaseCounts = b->bitCast(b->simd_fill(mSW.width, mHighestLineCount));
    mBaseCounts->addIncoming(nextBaseCounts, mBuildStrideMask);
}

void LineNumberGenerator::generateProcessingLogic(BuilderRef b, Value * const absoluteIndex,  Value * const blockIndex, Value * const bitOffset) {
    Value * const producedItemCount = b->getProducedItemCount("output");
    b->setProducedItemCount("output", b->CreateAdd(producedItemCount, b->getSize(1)));
    Value * const blockNo = b->CreateUDiv(mWordOffset, mSW.WORDS_PER_BLOCK);
    Value * const wordIndex = b->CreateURem(mWordOffset, mSW.WORDS_PER_BLOCK);
    Value * lineCount = b->CreateLoad(b->CreateGEP(mLineCountArrayBlockPtr, blockNo));
    lineCount = b->CreateExtractElement(b->fwCast(mSW.width, lineCount), wordIndex);
    // It is possible that there are break positions in this current scan word
    // which come after the scan-bit possition but are included in the value
    // of lineCount. We need to subtract the number of such break positions to
    // get the correct line number.
    Value * const breaksBlockPtr = b->CreateBitCast(b->getInputStreamBlockPtr("lines", b->getInt32(0), blockIndex), mSW.PointerTy);
    Value * const breaksWord = b->CreateLoad(b->CreateGEP(breaksBlockPtr, wordIndex));
    Value * const highMask = b->CreateNot(b->CreateMaskToLowestBitExclusive(mProcessingWord));
    Value * const maskedBreaksWord = b->CreateAnd(breaksWord, highMask);
    lineCount = b->CreateSub(lineCount, b->CreatePopcount(maskedBreaksWord));
    lineCount = b->CreateZExt(lineCount, b->getInt64Ty());
    lineCount = b->CreateAdd(mInitialLineNum, lineCount);
    b->CreateStore(lineCount, b->getRawOutputPointer("output", b->getInt32(0), producedItemCount));
}

void LineNumberGenerator::didProcessStride(BuilderRef b, Value * const /*strideNo*/) {
    Value * nextFinalStrideLineNum = b->CreateZExtOrTrunc(mHighestLineCount, b->getInt64Ty());
    nextFinalStrideLineNum = b->CreateAdd(mInitialLineNum, nextFinalStrideLineNum);
    b->setScalarField("finalStrideLineNum", nextFinalStrideLineNum);
}

LineNumberGenerator::LineNumberGenerator(BuilderRef b, StreamSet * scan, StreamSet * linebreaks, StreamSet * output)
: SingleStreamScanKernelTemplate(b, "LineNumberGenerator", scan)
{
    assert (scan->getNumElements() == 1 && scan->getFieldWidth() == 1);
    assert (linebreaks->getNumElements() == 1 && linebreaks->getFieldWidth() == 1);
    assert (output->getNumElements() == 1 && output->getFieldWidth() == 64);
    addInternalScalar(b->getInt64Ty(), "finalStrideLineNum");
    mInputStreamSets.push_back({"lines", linebreaks});
    mOutputStreamSets.push_back({"output", output, BoundedRate(0, 1)});
}

}
