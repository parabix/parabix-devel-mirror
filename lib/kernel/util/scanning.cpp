/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/util/scanning.h>
#include <llvm/IR/Module.h>
#include <kernel/core/kernel_builder.h>

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
    initialize(b);
    Value * const scanStreamProcessedItemCount = b->getProcessedItemCount("scan");
    Value * const MASK_ZERO = Constant::getNullValue(mSW.StrideMaskTy);
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
    Value * const block = b->loadInputStreamBlock("scan", b->getInt32(0), blockIndex);
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
    Value * const stridePtr = b->CreateBitCast(b->getInputStreamBlockPtr("scan", b->getInt32(0), processingBlockIndex), mSW.PointerTy);
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

generic::SingleStreamScanKernelTemplate::SingleStreamScanKernelTemplate(BuilderRef b, std::string && name, StreamSet * scan)
: MultiBlockKernel(b, name + "_sb" + std::to_string(codegen::ScanBlocks), {{"scan", scan}}, {}, {}, {}, {})
, mSW(b, std::min(codegen::ScanBlocks * b->getBitBlockWidth(), MaxStrideWidth))
{
    assert (scan->getNumElements() == 1 && scan->getFieldWidth() == 1);
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
 * ScanIndexGenerator
 * ----------------------------------------------------------------------------------------------------------------- */

ScanIndexGenerator::ScanIndexGenerator(BuilderRef b, StreamSet * scan, StreamSet * output)
: generic::SingleStreamScanKernelTemplate(b, "ScanIndexGenerator", scan)
{
    assert (scan->getNumElements() == 1 && scan->getFieldWidth() == 1);
    assert (output->getNumElements() == 1 && output->getFieldWidth() == 64);
    mOutputStreamSets.push_back({"output", output, PopcountOf("scan")});
}

void ScanIndexGenerator::generateProcessingLogic(BuilderRef b, Value * const absoluteIndex, Value * const blockIndex, Value * const bitOffset) {
    Value * const producedItemCount = b->getProducedItemCount("output");
    b->setProducedItemCount("output", b->CreateAdd(producedItemCount, b->getSize(1)));
    Value * const val = b->CreateZExtOrBitCast(absoluteIndex, b->getInt64Ty());
    b->CreateStore(val, b->getRawOutputPointer("output", b->getInt32(0), producedItemCount));
}



/* ----------------------------------------------------------------------------------------------------------------- *
 * LineNumberGenerator
 * ----------------------------------------------------------------------------------------------------------------- */

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
: generic::SingleStreamScanKernelTemplate(b, "LineNumberGenerator", scan)
{
    assert (scan->getNumElements() == 1 && scan->getFieldWidth() == 1);
    assert (linebreaks->getNumElements() == 1 && linebreaks->getFieldWidth() == 1);
    assert (output->getNumElements() == 1 && output->getFieldWidth() == 64);
    addInternalScalar(b->getInt64Ty(), "finalStrideLineNum");
    mInputStreamSets.push_back({"lines", linebreaks});
    mOutputStreamSets.push_back({"output", output, PopcountOf("scan")});
}



/* ----------------------------------------------------------------------------------------------------------------- *
 * LineSpanGenerator
 * ----------------------------------------------------------------------------------------------------------------- */

void LineSpanGenerator::initialize(BuilderRef b) {
    mFinalBlock = b->CreateBasicBlock("finalBlock");
    mLineSpanExit = b->CreateBasicBlock("lineSpanExit");
}

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

void LineSpanGenerator::finalize(BuilderRef b) {
    // Value * atNonLineBreakTerminatedEOF = b->CreateICmpEQ(b->getScalarField("lineBegin"), b->getProcessedItemCount("scan"));
    b->CreateCondBr(mIsFinal, mFinalBlock, mLineSpanExit);

    b->SetInsertPoint(mFinalBlock);
    Value * const producedCount = b->getProducedItemCount("output");
    Value * const beginIdx = b->getScalarField("lineBegin");
    Value * const endIdx = beginIdx;
    Value * const beginStorePtr = b->getRawOutputPointer("output", b->getInt32(0), producedCount);
    Value * const endStorePtr = b->getRawOutputPointer("output", b->getInt32(1), producedCount);
    b->CreateStore(beginIdx, beginStorePtr);
    b->CreateStore(endIdx, endStorePtr);
    b->setProducedItemCount("output", b->CreateAdd(producedCount, b->getSize(1)));
    b->CreateBr(mLineSpanExit);

    b->SetInsertPoint(mLineSpanExit);
}

LineSpanGenerator::LineSpanGenerator(BuilderRef b, StreamSet * linebreakStream, StreamSet * output)
: generic::SingleStreamScanKernelTemplate(b, "LineSpanGenerator", linebreakStream)
{
    assert (linebreakStream->getNumElements() == 1 && linebreakStream->getFieldWidth() == 1);
    addInternalScalar(b->getInt64Ty(), "lineBegin");
    mOutputStreamSets.push_back({"output", output, PopcountOf("scan"), Add1()});
}



/* ----------------------------------------------------------------------------------------------------------------- *
 * LineBasedReader
 * ----------------------------------------------------------------------------------------------------------------- */

void ScanReader::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {
    Module * const module = b->getModule();
    Type * const sizeTy = b->getSizeTy();
    Value * const sz_ZERO = b->getSize(0);
    Value * const sz_ONE = b->getSize(1);

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const readItem = b->CreateBasicBlock("readItem");
    BasicBlock * const exitBlock = b->CreateBasicBlock("exitBlock");
    Value * const initialStride = b->getProcessedItemCount("scan");
    Value * const isInvalidFinalItem = b->CreateAnd(mIsFinal, b->CreateICmpEQ(b->getSize(0), b->getAccessibleItemCount("scan")));
    b->CreateCondBr(isInvalidFinalItem, exitBlock, readItem);

    b->SetInsertPoint(readItem);
    PHINode * const strideNo = b->CreatePHI(sizeTy, 2);
    strideNo->addIncoming(sz_ZERO, entryBlock);
    Value * const nextStrideNo = b->CreateAdd(strideNo, sz_ONE);
    strideNo->addIncoming(nextStrideNo, readItem);
    std::vector<Value *> callbackParams{};
    Value * maxScanIndex = nullptr;
    Value * const index = b->CreateAdd(strideNo, initialStride);
    for (uint32_t i = 0; i < mNumScanStreams; ++i) {
        Value * const scanItem = b->CreateLoad(b->getRawInputPointer("scan", b->getInt32(i), index));
        if (maxScanIndex != nullptr) {
            maxScanIndex = b->CreateUMax(maxScanIndex, scanItem);
        } else {
            maxScanIndex = scanItem;
        }
        // FIXME: We are assuming that we have access to the entire source stream, this may not always be the case.
        Value * const scanPtr = b->getRawInputPointer("source", scanItem);
        callbackParams.push_back(scanPtr);
    }
    b->setProcessedItemCount("source", maxScanIndex);
    Value * const nextIndex = b->CreateAdd(nextStrideNo, initialStride);
    b->setProcessedItemCount("scan", nextIndex);
    for (auto const & name : mAdditionalStreamNames) {
        Value * const item = b->CreateLoad(b->getRawInputPointer(name, b->getInt32(0), index));
        callbackParams.push_back(item);
        b->setProcessedItemCount(name, nextIndex);
    }
    Function * const callback = module->getFunction(mCallbackName); assert (callback);
    b->CreateCall(callback, ArrayRef<Value *>(callbackParams));
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), readItem, exitBlock);

    b->SetInsertPoint(exitBlock);
}

static std::string ScanReader_GenerateName(StreamSet * scan, StringRef callbackName) {
    return "ScanReader_" + std::to_string(scan->getNumElements()) + "xscan" + "_" + std::string(callbackName);
}

static std::string ScanReader_GenerateName(StreamSet * scan, StringRef callbackName, AdditionalStreams const & additionalStreams) {
    std::string name = ScanReader_GenerateName(scan, callbackName);
    for (auto const & stream : additionalStreams) {
        name += "_" + std::to_string(stream->getNumElements()) + "xi" + std::to_string(stream->getFieldWidth());
    }
    return name;
}

ScanReader::ScanReader(BuilderRef b, StreamSet * source, StreamSet * scanIndices, StringRef callbackName)
: MultiBlockKernel(b, ScanReader_GenerateName(scanIndices, callbackName), {
    {"scan", scanIndices, BoundedRate(0, 1), Principal()},
    {"source", source, BoundedRate(0, 1)}
  }, {}, {}, {}, {})
, mCallbackName(callbackName)
, mAdditionalStreamNames()
, mNumScanStreams(scanIndices->getNumElements())
{
    assert (scanIndices->getFieldWidth() == 64);
    assert (source->getNumElements() == 1);
    addAttribute(SideEffecting());
    setStride(1);
}

ScanReader::ScanReader(BuilderRef b, StreamSet * source, StreamSet * scanIndices, StringRef callbackName, AdditionalStreams additionalStreams)
: MultiBlockKernel(b, ScanReader_GenerateName(scanIndices, callbackName, additionalStreams), {
    {"scan", scanIndices, BoundedRate(0, 1), Principal()},
    {"source", source, BoundedRate(0, 1)}
  }, {}, {}, {}, {})
, mCallbackName(callbackName)
, mAdditionalStreamNames()
, mNumScanStreams(scanIndices->getNumElements())
{
    assert (scanIndices->getFieldWidth() == 64);
    assert (source->getNumElements() == 1);
    addAttribute(SideEffecting());
    setStride(1);
    size_t i = 0;
    for (auto const & stream : additionalStreams) {
        std::string name = "__additional_" + std::to_string(i++);
        mInputStreamSets.push_back({name, stream, BoundedRate(0, 1)});
        mAdditionalStreamNames.push_back(name);
    }
}



/* ----------------------------------------------------------------------------------------------------------------- *
 * LineBasedReader
 * ----------------------------------------------------------------------------------------------------------------- */

void LineBasedReader::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {
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
    Value * const isInvalidFinalItem = b->CreateAnd(mIsFinal, b->CreateICmpEQ(b->getSize(0), b->getAccessibleItemCount("scan")));
    b->CreateCondBr(isInvalidFinalItem, exitBlock, processScanPosition);

    b->SetInsertPoint(processScanPosition);
    PHINode * const strideNo = b->CreatePHI(i64Ty, 3);
    strideNo->addIncoming(i64_ZERO, entryBlock);
    PHINode * const scanIndex = b->CreatePHI(i64Ty, 3);
    scanIndex->addIncoming(initialProcessedScanValue, entryBlock);
    PHINode * const lineIndex = b->CreatePHI(i64Ty, 3, "lbr line #");
    lineIndex->addIncoming(initialProcessedLineValue, entryBlock);
    Value * const lineNumber = b->CreateLoad(b->getRawInputPointer("lineNums", b->getInt32(0), scanIndex), "lbr looking for line #");
    // We need to match lineNumber with lineIndex to retrieve the start and end
    // pointers for the line. If their values don't match now, we keep skipping
    // over line spans until they do.
    b->CreateCondBr(b->CreateICmpEQ(lineNumber, lineIndex), triggerCallback, skipLineSpan);

    b->SetInsertPoint(triggerCallback);
    std::vector<Value *> callbackParams{};
    callbackParams.reserve(4 + mAdditionalStreamNames.size());
    Value * const scanVal = b->CreateLoad(b->getRawInputPointer("scan", b->getInt32(0), scanIndex));
    // FIXME: We are assuming that we have access to the entire source stream, this may not always be the case.
    Value * const scanPtr = b->getRawInputPointer("source", scanVal);
    b->setProcessedItemCount("scan", b->CreateAdd(scanVal, b->getInt64(1)));
    callbackParams.push_back(scanPtr);
    Value * const beginVal = b->CreateLoad(b->getRawInputPointer("lines", b->getInt32(0), lineIndex));
    Value * const beginPtr = b->getRawInputPointer("source", beginVal);
    callbackParams.push_back(beginPtr);
    Value * const endVal = b->CreateLoad(b->getRawInputPointer("lines", b->getInt32(1), lineIndex));
    Value * const endPtr = b->getRawInputPointer("source", endVal);
    callbackParams.push_back(endPtr);
    Function * const callback = module->getFunction(mCallbackName); assert (callback);
    Value * const oneIndexedLineNumber = b->CreateAdd(lineNumber, i64_ONE);
    callbackParams.push_back(oneIndexedLineNumber);
    for (auto const & name : mAdditionalStreamNames) {
        Value * const item = b->CreateLoad(b->getRawInputPointer(name, b->getInt32(0), scanIndex));
        callbackParams.push_back(item);
    }
    b->CreateCall(callback, ArrayRef<Value *>(callbackParams));
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
    b->setProcessedItemCount("lineNums", finalScanIndex);
    for (auto const & name : mAdditionalStreamNames) {
        b->setProcessedItemCount(name, finalScanIndex);
    }
    b->setProcessedItemCount("lines", finalLineIndex);
    b->CreateBr(exitBlock);

    b->SetInsertPoint(exitBlock);
}

static std::string LineBasedReader_GenerateName(StreamSet * scan, StringRef callbackName) {
    return "LineBasedReader_" + std::to_string(scan->getNumElements()) + "xscan" + "_" + std::string(callbackName);
}

static std::string LineBasedReader_GenerateName(StreamSet * scan, StringRef callbackName, AdditionalStreams const & additionalStreams) {
    std::string name = LineBasedReader_GenerateName(scan, callbackName);
    for (auto const & stream : additionalStreams) {
        name += "_" + std::to_string(stream->getNumElements()) + "xi" + std::to_string(stream->getFieldWidth());
    }
    return name;
}

LineBasedReader::LineBasedReader(BuilderRef b, StreamSet * source, StreamSet * scanPositions, StreamSet * lineNumbers, StreamSet * lineSpans, StringRef callbackName)
: MultiBlockKernel(b, LineBasedReader_GenerateName(scanPositions, callbackName), 
    { // input streamsets
        {"scan", scanPositions, BoundedRate(0, 1), Principal()}, 
        {"lineNums", lineNumbers, BoundedRate(0, 1)}, 
        {"lines", lineSpans, BoundedRate(0, 1)}, 
        {"source", source, BoundedRate(0, 1)}
    }, {}, {}, {}, {})
, mCallbackName(callbackName)
, mAdditionalStreamNames()
{
    assert (scanPositions->getNumElements() == 1 && scanPositions->getFieldWidth() == 64);
    assert (lineNumbers->getNumElements() == 1 && lineNumbers->getFieldWidth() == 64);
    assert (lineSpans->getNumElements() == 2 && lineSpans->getFieldWidth() == 64);
    addAttribute(SideEffecting());
    setStride(1);
}

LineBasedReader::LineBasedReader(BuilderRef b, 
                                 StreamSet * source, 
                                 StreamSet * scanPositions, 
                                 StreamSet * lineNumbers, 
                                 StreamSet * lineSpans, 
                                 StringRef callbackName, 
                                 AdditionalStreams additionalStreams)
: MultiBlockKernel(b, LineBasedReader_GenerateName(scanPositions, callbackName, additionalStreams), 
    { // input streamsets
        {"scan", scanPositions, BoundedRate(0, 1), Principal()}, 
        {"lineNums", lineNumbers, BoundedRate(0, 1)}, 
        {"lines", lineSpans, BoundedRate(0, 1)}, 
        {"source", source}
    }, {}, {}, {}, {})
, mCallbackName(callbackName)
, mAdditionalStreamNames()
{
    assert (scanPositions->getNumElements() == 1 && scanPositions->getFieldWidth() == 64);
    assert (lineNumbers->getNumElements() == 1 && lineNumbers->getFieldWidth() == 64);
    assert (lineSpans->getNumElements() == 2 && lineSpans->getFieldWidth() == 64);
    addAttribute(SideEffecting());
    setStride(1);
    size_t i = 0;
    for (auto const & stream : additionalStreams) {
        std::string name = "__additional_" + std::to_string(i++);
        mInputStreamSets.push_back({name, stream, BoundedRate(0, 1)});
        mAdditionalStreamNames.push_back(name);
    }
}

} // namespace kernel
