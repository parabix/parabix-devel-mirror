/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/util/scan_kernel.h>

#include <vector>
#include <toolchain/toolchain.h>
#include <kernel/core/kernel_builder.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/ErrorHandling.h>

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

Value * ScanKernelBase::loadScanStreamBitBlock(BuilderRef b, Value * strideNo, Value * blockNo, llvm::Value * streamIndex) {
    Value * const sidx = streamIndex == nullptr ? b->getSize(0) : streamIndex;
    Value * idx = b->CreateAdd(blockNo, computeStrideBlockOffset(b, strideNo));
    return b->loadInputStreamBlock(mScanStreamSetName, sidx, idx);
}

Value * ScanKernelBase::orBlockIntoMask(BuilderRef b, ScanWordContext const & sw, Value * maskAccum, Value * block, Value * blockNo) {
    Value * const any = b->simd_any(sw.fieldWidth, block);
    Value * const signMask = b->CreateZExt(b->hsimd_signmask(sw.fieldWidth, any), sw.StrideMaskTy);
    Value * const shiftedSignMask = b->CreateShl(signMask, b->CreateZExtOrTrunc(b->CreateMul(blockNo, sw.WORDS_PER_BLOCK), sw.StrideMaskTy));
    return b->CreateOr(maskAccum, shiftedSignMask);
}

Value * ScanKernelBase::loadScanWord(BuilderRef b, ScanWordContext const & sw, Value * wordOffset, Value * strideNo, llvm::Value * streamIndex) {
    Value * const index = streamIndex == nullptr ? b->getSize(0) : streamIndex;
    Value * const strideOffset = b->CreateMul(strideNo, sz_NUM_BLOCKS_PER_STRIDE);
    Value * const blockNo = b->CreateUDiv(wordOffset, sw.WORDS_PER_BLOCK);
    Value * const blockOffset = b->CreateURem(wordOffset, sw.WORDS_PER_BLOCK);
    Value * const stridePtr = b->CreateBitCast(b->getInputStreamBlockPtr(mScanStreamSetName, index, b->CreateAdd(strideOffset, blockNo)), sw.PointerTy);
    Value * const wordPtr = b->CreateGEP(stridePtr, blockOffset);
    return b->CreateLoad(wordPtr);
}

void ScanKernelBase::createOptimizedContinueProcessingBr(BuilderRef b, Value * value, BasicBlock * trueBlock, BasicBlock * falseBlock) {
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

ScanKernelBase::ScanKernelBase(BuilderRef b, unsigned strideWidth, StringRef scanStreamSetName, OptimizeMode optimizeMode)
: mStrideWidth(strideWidth)
, mScanStreamSetName(scanStreamSetName)
, mInitialPos(nullptr)
, sz_STRIDE_WIDTH(b->getSize(strideWidth))
, sz_NUM_BLOCKS_PER_STRIDE(b->getSize(strideWidth / b->getBitBlockWidth()))
, mOptimizeMode(optimizeMode)
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
    Value * const word = loadScanWord(b, sw, wordOffset, strideNo);
    b->CreateBr(processWord);

    b->SetInsertPoint(processWord);
    // Scan through the word and trigger a callback at the location of each bit.
    PHINode * const processingWord = b->CreatePHI(sw.Ty, 2, "processingWord");
    processingWord->addIncoming(word, processMask);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        b->CreateAssert(b->CreateICmpNE(processingWord, ZERO_WORD), "ScanKernel::processWord: processing word cannot be zero!");
    }
    Value * const bitIndex_InWord = b->CreateZExt(b->CreateCountForwardZeroes(processingWord, true), sizeTy);
    Value * const wordIndex_InStride = b->CreateMul(wordOffset, sw.WIDTH);
    Value * const strideIndex = computeStridePosition(b, strideNo);
    Value * const callbackIndex = b->CreateAdd(strideIndex, b->CreateAdd(wordIndex_InStride, bitIndex_InWord), "callbackIndex");
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

static inline std::string ScanKernel_GenName(unsigned strideWidth, std::string const & callbackName) {
    return "ScanKernel_sw" + std::to_string(strideWidth) + "_" + callbackName;
}

ScanKernel::ScanKernel(BuilderRef b, StreamSet * scanStream, StreamSet * sourceStream, StringRef callbackName, OptimizeMode optimizeMode)
: ScanKernelBase(b, std::min(codegen::ScanBlocks * b->getBitBlockWidth(), ScanWordContext::maxStrideWidth), "scan", optimizeMode)
, MultiBlockKernel(b, ScanKernel_GenName(ScanKernelBase::mStrideWidth, callbackName), 
    {{"scan", scanStream}, {"source", sourceStream}}, {}, {}, {}, {})
, mCallbackName(callbackName)
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


static Value * collapseVector(const std::unique_ptr<KernelBuilder> & b, Value * const vec) {
    assert (vec->getType()->isVectorTy());
    uint32_t count = vec->getType()->getVectorNumElements();
    Value * accum = b->CreateExtractElement(vec, (uint64_t) 0);
    for (uint32_t i = 1; i < count; ++i) {
        accum = b->CreateOr(accum, b->CreateExtractElement(vec, i));
    }
    return accum;
}

static Value * vectorFromRepeating(const std::unique_ptr<KernelBuilder> & b, Value * const value, size_t count) {
    Value * vec = Constant::getNullValue(VectorType::get(value->getType(), count));
    for (size_t i = 0; i < count; ++i) {
        vec = b->CreateInsertElement(vec, value, i);
    }
    return vec;
}

void MultiStreamScanKernel::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {
    ScanWordContext sw(b, mStrideWidth);
    Module * const module = b->getModule();

    uint32_t numInputStreams = getInputStreamSet("scan")->getNumElements();
    
    Type * const sizeTy = b->getSizeTy();
    Value * const sz_ZERO = b->getSize(0);
    Value * const sz_ONE = b->getSize(1);

    Value * const ZERO_MASK = Constant::getNullValue(sw.StrideMaskTy);

    Type * const i1VecTy = VectorType::get(b->getInt1Ty(), numInputStreams);

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const strideInit = b->CreateBasicBlock("strideInit");
    BasicBlock * const buildMasks = b->CreateBasicBlock("buildMasks");
    BasicBlock * const phiMasksReady = b->CreateBasicBlock("phiMasksReady");
    BasicBlock * const masksReady = b->CreateBasicBlock("masksReady");
    BasicBlock * const parallelProcessMasks = b->CreateBasicBlock("parallelProcessMasks");
    BasicBlock * const parallelProcessWords = b->CreateBasicBlock("parallelProcessWords");
    BasicBlock * const performCallbacks = b->CreateBasicBlock("performCallbacks");
    BasicBlock * const triggerCallback = b->CreateBasicBlock("triggerCallback");
    BasicBlock * const performCallbackLoopTail = b->CreateBasicBlock("performCallbackLoopTail");
    BasicBlock * const finishedCallbacks = b->CreateBasicBlock("finsiedCallbacks");
    BasicBlock * const finishedProcessingWords = b->CreateBasicBlock("finishedProcessingWords");
    BasicBlock * const strideDone = b->CreateBasicBlock("strideDone");
    BasicBlock * const exitBlock = b->CreateBasicBlock("exitBlock");
    initializeBase(b);
    b->CreateBr(strideInit);

    b->SetInsertPoint(strideInit);
    PHINode * const strideNo = b->CreatePHI(sizeTy, 2, "strideNo");
    strideNo->addIncoming(sz_ZERO, entryBlock);
    Value * const nextStrideNo = b->CreateAdd(strideNo, sz_ONE);
    b->CreateBr(buildMasks);

    b->SetInsertPoint(buildMasks);
    PHINode * const blockNo = b->CreatePHI(sizeTy, 2, "blockNo");
    blockNo->addIncoming(sz_ZERO, strideInit);
    // Use PHI nodes to build up scan masks in an unrolled loop.
    // Values will be written to the mask array once, after they have been built.
    std::vector<PHINode *> maskAccumPhis(numInputStreams, nullptr);
    std::vector<Value *> maskValues(numInputStreams, nullptr);
    for (uint32_t i = 0; i < numInputStreams; ++i) {
        maskAccumPhis[i] = b->CreatePHI(sw.StrideMaskTy, 2);
        maskAccumPhis[i]->addIncoming(ZERO_MASK, strideInit);
    }
    for (uint32_t i = 0; i < numInputStreams; ++i) {
        Value * const block = loadScanStreamBitBlock(b, strideNo, blockNo, b->getInt32(i));
        Value * const mergedMask = orBlockIntoMask(b, sw, maskAccumPhis[i], block, blockNo);
        maskAccumPhis[i]->addIncoming(mergedMask, buildMasks);
        maskValues[i] = mergedMask;
    }
    Value * const nextBlockNo = b->CreateAdd(blockNo, sz_ONE);
    blockNo->addIncoming(nextBlockNo, buildMasks);
    b->CreateCondBr(b->CreateICmpNE(nextBlockNo, sz_NUM_BLOCKS_PER_STRIDE), buildMasks, phiMasksReady);

    b->SetInsertPoint(phiMasksReady);
    // First check to see if there are any bits in any of the masks. If there are 
    // none, then we don't need to bother with writting values to the array.
    assert (maskValues.size() > 0);
    Value * phiMaskAccum = maskValues[0];
    for (uint32_t i = 1; i < numInputStreams; ++i) {
        phiMaskAccum = b->CreateOr(phiMaskAccum, maskValues[i]);
    }
    createOptimizedContinueProcessingBr(b, phiMaskAccum, masksReady, strideDone);

    b->SetInsertPoint(masksReady);
    // There is some data to be processed in the masks, we'll store the mask
    // values in a vector to be processed in parallel.
    Value * maskVec = Constant::getNullValue(VectorType::get(sw.StrideMaskTy, numInputStreams));
    for (uint32_t i = 0; i < numInputStreams; ++i) {
        maskVec = b->CreateInsertElement(maskVec, maskValues[i], i);
    }
    b->CreateBr(parallelProcessMasks);

    // To ensure that callback bits are processed in the order that they appear 
    // in the scan streamset, regardless of which stream they are in, all of 
    // the masks for stride are processed in parallel.
    //
    // We can't simply process each mask in order of its index because that
    // would mean that all of the callbacks for index 0 are triggered before
    // any other indicies, even if they have bits which appear earlier in the
    // stream.
    //
    // The parallel processing algorithm is as such:
    //  (1) isolate the lowest bit of each non-zero field
    //  (2) set all zero fields to be all ones
    //  (3) the minimum field via unsigned icmp
    //  (4) compute CountForwardZeros for minimum field
    //  (5) logically shift the original vector right by (4)
    //  (6) do some operation one each field with a 1-bit at bit index 0, then
    //      reset the lowest bit for each of the fields that have been processed.
    //  (7) repeat until all fields are 0
    //
    // This algorithm is used for processing both masks and scan words. When
    // processing scan words, step (6) triggers the callback for the stream.
    // When processing masks however, the result of step (6) is used to select
    // which scan words to parallel process.

    b->SetInsertPoint(parallelProcessMasks);
    Type * const maskVectorTy = VectorType::get(sw.StrideMaskTy, numInputStreams);
    Value * const nullMaskVector = Constant::getNullValue(maskVectorTy);
    Value * const allOneMaskVector = Constant::getAllOnesValue(maskVectorTy);
    PHINode * const maskVectorPhi = b->CreatePHI(maskVectorTy, 2);
    maskVectorPhi->addIncoming(maskVec, masksReady);
    Value * const isolatedOrFull = b->CreateSelect(b->CreateICmpNE(maskVectorPhi, nullMaskVector), 
                                                   b->CreateIsolateLowestBit(maskVectorPhi),
                                                   allOneMaskVector);
    Value * minMask = b->CreateExtractElement(isolatedOrFull, (uint64_t) 0);
    for (uint32_t i = 1; i < numInputStreams; ++i) {
        Value * const val = b->CreateExtractElement(isolatedOrFull, i);
        minMask = b->CreateSelect(b->CreateICmpULT(val, minMask), val, minMask);
    }
    Value * maskShiftAmount = b->CreateCountForwardZeroes(minMask);
    Value * const wordOffset = maskShiftAmount;
    Value * const shiftedMaskVector = b->CreateLShr(maskVectorPhi, vectorFromRepeating(b, maskShiftAmount, numInputStreams));
    // Load scan words for this offset for masks with a 1-bit in the lowest position.
    Value * const lowBitVec = b->CreateTrunc(shiftedMaskVector, i1VecTy);
    
    // FIXME: Here we are loading words for all streams instead of only the ones with
    //        a bit in the lowest position. This is to reduce the number of branches
    //        but it may be better to loop through each mask and only load the words
    //        we need.

    Type * const wordVectorTy = VectorType::get(sw.Ty, numInputStreams);
    Value * const nullWordVector = Constant::getNullValue(wordVectorTy);
    Value * loadedWordVec = nullWordVector;
    for (uint32_t i = 0; i < numInputStreams; ++i) {
        Value * const val = loadScanWord(b, sw, wordOffset, strideNo, b->getInt32(i));
        loadedWordVec = b->CreateInsertElement(loadedWordVec, val, i);
    }
    Value * const wordVec = b->CreateSelect(lowBitVec, loadedWordVec, nullWordVector);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * wordVecAccum = b->CreateExtractElement(wordVec, (uint64_t) 0);
        for (uint32_t i = 1; i < numInputStreams; ++i) {
            wordVecAccum = b->CreateOr(wordVecAccum, b->CreateExtractElement(wordVec, i));
        }
        b->CreateAssert(b->CreateICmpNE(wordVecAccum, Constant::getNullValue(wordVecAccum->getType())), "Failed to load scan word!");
    }
    b->CreateBr(parallelProcessWords);

    // Use the same algorithm to process scan words in parallel.
    b->SetInsertPoint(parallelProcessWords);
    PHINode * const wordVectorPhi = b->CreatePHI(wordVectorTy, 2);
    wordVectorPhi->addIncoming(wordVec, parallelProcessMasks);
    Value * const allOneWordVector = Constant::getAllOnesValue(wordVectorTy);
    Value * const isolatedOrFull_Word = b->CreateSelect(b->CreateICmpNE(wordVectorPhi, nullWordVector),
                                                        b->CreateIsolateLowestBit(wordVectorPhi),
                                                        allOneWordVector);
    Value * minWord = b->CreateExtractElement(isolatedOrFull_Word, (uint64_t) 0);
    for (uint32_t i = 0; i < numInputStreams; ++i) {
        Value * const val = b->CreateExtractElement(isolatedOrFull_Word, i);
        minWord = b->CreateSelect(b->CreateICmpULT(val, minWord), val, minWord);
    }
    Value * const wordShiftAmount = b->CreateCountForwardZeroes(minWord);
    Value * const bitOffset = b->CreateZExt(wordShiftAmount, sizeTy);
    Value * const shiftedWordVector = b->CreateLShr(wordVectorPhi, vectorFromRepeating(b, wordShiftAmount, numInputStreams));
    Value * const triggerVector = b->CreateTrunc(shiftedWordVector, i1VecTy);
    b->CreateBr(performCallbacks);

    // Loop through the scan word vector and trigger a callback for each scan word with
    // a one in the lowest bit position.
    b->SetInsertPoint(performCallbacks);
    PHINode * const streamIndex = b->CreatePHI(sizeTy, 2);
    streamIndex->addIncoming(sz_ZERO, parallelProcessWords);
    Value * const trigger = b->CreateExtractElement(triggerVector, streamIndex);
    b->CreateCondBr(trigger, triggerCallback, performCallbackLoopTail);

    b->SetInsertPoint(triggerCallback);
    Value * const wordIndex = b->CreateZExt(b->CreateMul(wordOffset, sw.WIDTH), sizeTy);
    Value * const strideIndex = computeStridePosition(b, strideNo);
    Value * const callbackIndex = b->CreateAdd(strideIndex, b->CreateAdd(wordIndex, bitOffset), "callbackIndex");
    Value * const sourcePtr = b->getRawInputPointer("source", callbackIndex);
    Function * const callback = module->getFunction(mCallbackName); assert (callback);
    b->CreateCall(callback, {sourcePtr, callbackIndex, streamIndex});
    b->CreateBr(performCallbackLoopTail);

    b->SetInsertPoint(performCallbackLoopTail);
    Value * const nextStreamIndex = b->CreateAdd(streamIndex, sz_ONE);
    streamIndex->addIncoming(nextStreamIndex, performCallbackLoopTail);
    b->CreateCondBr(b->CreateICmpNE(nextStreamIndex, b->getSize(numInputStreams)), performCallbacks, finishedCallbacks);

    b->SetInsertPoint(finishedCallbacks);
    Value * const resetLowestBitVector_Words = b->CreateResetLowestBit(wordVectorPhi);
    Value * const newWordVector = b->CreateSelect(triggerVector, resetLowestBitVector_Words, wordVectorPhi);
    wordVectorPhi->addIncoming(newWordVector, finishedCallbacks);
    b->CreateCondBr(collapseVector(b, b->CreateICmpNE(newWordVector, nullWordVector)), parallelProcessWords, finishedProcessingWords);

    b->SetInsertPoint(finishedProcessingWords);
    Value * const resetLowestBitVector_Masks = b->CreateResetLowestBit(maskVectorPhi);
    Value * const newMaskVector = b->CreateSelect(lowBitVec, resetLowestBitVector_Masks, maskVectorPhi);
    maskVectorPhi->addIncoming(newMaskVector, finishedProcessingWords);
    b->CreateCondBr(collapseVector(b, b->CreateICmpNE(newMaskVector, nullMaskVector)), parallelProcessMasks, strideDone);

    b->SetInsertPoint(strideDone);
    strideNo->addIncoming(nextStrideNo, strideDone);
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), strideInit, exitBlock);

    b->SetInsertPoint(exitBlock);
}

static inline std::string MultiStreamScanKernel_GenName(unsigned inputStreamCount, unsigned strideWidth, std::string const & callbackName) {
    return "MultiStreamScanKerenl_x" + std::to_string(inputStreamCount) + "_sw" + std::to_string(strideWidth) + "_" + callbackName;
}

MultiStreamScanKernel::MultiStreamScanKernel(BuilderRef b, StreamSet * scanStream, StreamSet * sourceStream, StringRef callbackName, OptimizeMode optimizeMode)
: ScanKernelBase(b, std::min(codegen::ScanBlocks * b->getBitBlockWidth(), ScanWordContext::maxStrideWidth), "scan", optimizeMode)
, MultiBlockKernel(b, MultiStreamScanKernel_GenName(ScanKernelBase::mStrideWidth, scanStream->getNumElements(), callbackName),
    {{"scan", scanStream}, {"source", sourceStream}}, {}, {}, {}, {})
, mCallbackName(callbackName)
{
    assert (sourceStream->getNumElements() == 1 && sourceStream->getFieldWidth() == 8);
    if (!IS_POW_2(codegen::ScanBlocks)) {
        report_fatal_error("scan-blocks must be a power of 2");
    }
    if ((codegen::ScanBlocks * b->getBitBlockWidth()) > ScanWordContext::maxStrideWidth) {
        report_fatal_error("scan-blocks exceeds maximum allowed size of " + std::to_string(ScanWordContext::maxStrideWidth / b->getBitBlockWidth()));
    }
    addAttribute(SideEffecting());
    setStride(mStrideWidth);
}

} // namespace kernel
