/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/util/scanmatchgen.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#include <kernel/core/kernel_builder.h>

using namespace llvm;

namespace kernel {

const unsigned BITS_PER_BYTE = 8;
const unsigned SIZE_T_BITS = sizeof(size_t) * BITS_PER_BYTE;

struct ScanWordParameters {
    unsigned width;
    unsigned indexWidth;
    Type * const Ty;
    Type * const pointerTy;
    Constant * const WIDTH;
    Constant * const ix_MAXBIT;
    Constant * WORDS_PER_BLOCK;
    Constant * WORDS_PER_STRIDE;

    ScanWordParameters(const std::unique_ptr<KernelBuilder> & b, unsigned stride) :
#ifdef PREFER_NARROW_SCANWIDTH
    width(std::max(BITS_PER_BYTE, stride/SIZE_T_BITS)),
#else
    width(std::min(SIZE_T_BITS, stride/BITS_PER_BYTE)),
#endif
    indexWidth(stride/width),
    Ty(b->getIntNTy(width)),
    pointerTy(Ty->getPointerTo()),
    WIDTH(b->getSize(width)),
    ix_MAXBIT(b->getSize(indexWidth - 1)),
    WORDS_PER_BLOCK(b->getSize(b->getBitBlockWidth()/width)),
    WORDS_PER_STRIDE(b->getSize(indexWidth))
    {   //  The stride must be a power of 2 and a multiple of the BitBlock width.
        assert((((stride & (stride - 1)) == 0) && (stride >= b->getBitBlockWidth()) && (stride <= SIZE_T_BITS * SIZE_T_BITS)));
    }
};


void ScanMatchKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfStrides) {
    const bool mLineNumbering = true;
    // Determine the parameters for two-level scanning.
    ScanWordParameters sw(b, mStride);

    Module * const m = b->getModule();
    Constant * sz_STRIDE = b->getSize(mStride);
    Constant * sz_BLOCKS_PER_STRIDE = b->getSize(mStride/b->getBitBlockWidth());
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_ONE = b->getSize(1);
    Constant * sz_BITS = b->getSize(SIZE_T_BITS);
    Constant * sz_MAXBIT = b->getSize(SIZE_T_BITS - 1);
    Type * sizeTy = b->getSizeTy();

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const stridePrologue = b->CreateBasicBlock("stridePrologue");
    BasicBlock * const stridePrecomputation = b->CreateBasicBlock("stridePrecomputation");
    BasicBlock * const strideMasksReady = b->CreateBasicBlock("strideMasksReady");
    BasicBlock * const updateLineInfo = b->CreateBasicBlock("updateLineInfo");
    BasicBlock * const strideMatchLoop = b->CreateBasicBlock("strideMatchLoop");
    BasicBlock * const dispatch = b->CreateBasicBlock("dispatch");
    BasicBlock * const matchesDone = b->CreateBasicBlock("matchesDone");
    BasicBlock * const stridesDone = b->CreateBasicBlock("stridesDone");
    BasicBlock * const callFinalizeScan = b->CreateBasicBlock("callFinalizeScan");
    BasicBlock * const scanReturn = b->CreateBasicBlock("scanReturn");

    Value * const initialPos = b->getProcessedItemCount("matchResult");
    Value * accumulator = b->getScalarField("accumulator_address");
    Value * const avail = b->getAvailableItemCount("InputStream");
    Value * const initialLineStart = b->getProcessedItemCount("InputStream");
    Value * initialLineNum = nullptr;
    Value * lineCountArrayBlockPtr = nullptr;
    Value * lineCountArrayWordPtr = nullptr;
    if (mLineNumbering) {
        initialLineNum = b->getScalarField("LineNum");
        lineCountArrayBlockPtr = b->CreateAlignedAlloca(b->getBitBlockType(),
                                                        b->getBitBlockWidth()/BITS_PER_BYTE,
                                                        sz_BLOCKS_PER_STRIDE);
        // Bitcast the lineNumberArrayptr to access by scanWord number
        lineCountArrayWordPtr = b->CreateBitCast(lineCountArrayBlockPtr, sw.pointerTy);
    }
    b->CreateBr(stridePrologue);

    b->SetInsertPoint(stridePrologue);
    // Set up the loop variables as PHI nodes at the beginning of each stride.
    PHINode * const strideNo = b->CreatePHI(sizeTy, 2);
    strideNo->addIncoming(sz_ZERO, entryBlock);
    PHINode * const pendingLineStart = b->CreatePHI(sizeTy, 2);
    pendingLineStart->addIncoming(initialLineStart, entryBlock);
    PHINode * pendingLineNum = nullptr;
    if (mLineNumbering) {
        pendingLineNum = b->CreatePHI(sizeTy, 2);
        pendingLineNum->addIncoming(initialLineNum, entryBlock);
    }
    Value * stridePos = b->CreateAdd(initialPos, b->CreateMul(strideNo, sz_STRIDE));
    Value * strideBlockOffset = b->CreateMul(strideNo, sz_BLOCKS_PER_STRIDE);
    Value * nextStrideNo = b->CreateAdd(strideNo, sz_ONE);
    b->CreateBr(stridePrecomputation);
    // Precompute index masks for one stride of the match result and line break streams,
    // as well as a partial sum popcount of line numbers if line numbering is on.
    b->SetInsertPoint(stridePrecomputation);
    PHINode * const matchMaskAccum = b->CreatePHI(sizeTy, 2);
    matchMaskAccum->addIncoming(sz_ZERO, stridePrologue);
    PHINode * const breakMaskAccum = b->CreatePHI(sizeTy, 2);
    breakMaskAccum->addIncoming(sz_ZERO, stridePrologue);
    PHINode * const blockNo = b->CreatePHI(sizeTy, 2);
    blockNo->addIncoming(sz_ZERO, stridePrologue);
    PHINode * baseCounts = nullptr;
    if (mLineNumbering) {
        baseCounts = b->CreatePHI(b->getBitBlockType(), 2);
        baseCounts->addIncoming(b->allZeroes(), stridePrologue);
    }
    Value * strideBlockIndex = b->CreateAdd(strideBlockOffset, blockNo);
    Value * matchBitBlock = b->loadInputStreamBlock("matchResult", sz_ZERO, strideBlockIndex);
    Value * breakBitBlock = b->loadInputStreamBlock("lineBreak", sz_ZERO, strideBlockIndex);
    Value * const anyMatch = b->simd_any(sw.width, matchBitBlock);
    Value * const anyBreak = b->simd_any(sw.width, breakBitBlock);
    if (mLineNumbering) {
        Value * breakCounts = b->hsimd_partial_sum(sw.width, b->simd_popcount(sw.width, breakBitBlock));
        breakCounts = b->simd_add(sw.width, breakCounts, baseCounts);
        b->CreateBlockAlignedStore(b->bitCast(breakCounts), b->CreateGEP(lineCountArrayBlockPtr, blockNo));
        Value * baseCountsNext = b->bitCast(b->simd_fill(sw.width, b->mvmd_extract(sw.width, breakCounts, b->getBitBlockWidth()/sw.width - 1)));
        baseCounts->addIncoming(baseCountsNext, stridePrecomputation);
    }
    Value * matchWordMask = b->CreateZExtOrTrunc(b->hsimd_signmask(sw.width, anyMatch), sizeTy);
    Value * breakWordMask = b->CreateZExtOrTrunc(b->hsimd_signmask(sw.width, anyBreak), sizeTy);
    Value * matchMask = b->CreateOr(matchMaskAccum, b->CreateShl(matchWordMask, b->CreateMul(blockNo, sw.WORDS_PER_BLOCK)), "matchMask");
    Value * breakMask = b->CreateOr(breakMaskAccum, b->CreateShl(breakWordMask, b->CreateMul(blockNo, sw.WORDS_PER_BLOCK)), "breakMask");
    Value * const nextBlockNo = b->CreateAdd(blockNo, sz_ONE);
    matchMaskAccum->addIncoming(matchMask, stridePrecomputation);
    breakMaskAccum->addIncoming(breakMask, stridePrecomputation);
    blockNo->addIncoming(nextBlockNo, stridePrecomputation);
    b->CreateCondBr(b->CreateICmpNE(nextBlockNo, sz_BLOCKS_PER_STRIDE), stridePrecomputation, strideMasksReady);

    b->SetInsertPoint(strideMasksReady);
    // If there are no breaks in the stride, there are no matches.   We can move on to
    // the next stride immediately.
    b->CreateUnlikelyCondBr(b->CreateICmpEQ(breakMask, sz_ZERO), matchesDone, updateLineInfo);

    b->SetInsertPoint(updateLineInfo);
    // We have at least one line break.   Determine the end-of-stride line start position
    // and line number, if needed.
    Value * matchWordBasePtr = b->getInputStreamBlockPtr("matchResult", sz_ZERO, strideBlockOffset);
    matchWordBasePtr = b->CreateBitCast(matchWordBasePtr, sw.pointerTy);
    Value * breakWordBasePtr = b->getInputStreamBlockPtr("lineBreak", sz_ZERO, strideBlockOffset);
    breakWordBasePtr = b->CreateBitCast(breakWordBasePtr, sw.pointerTy);

    Value * finalBreakIdx = b->CreateSub(sz_MAXBIT, b->CreateCountReverseZeroes(breakMask), "finalBreakIdx");
    Value * finalBreakWord = b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(breakWordBasePtr, finalBreakIdx)), sizeTy);
    Value * finalLineStartInWord = b->CreateSub(sz_BITS, b->CreateCountReverseZeroes(finalBreakWord));
    Value * finalLineStartPos = b->CreateAdd(stridePos, b->CreateMul(finalBreakIdx, sw.WIDTH));
    finalLineStartPos = b->CreateAdd(finalLineStartInWord, finalLineStartPos);
    Value * strideFinalLineNum = nullptr;
    if (mLineNumbering) {
        // compute the final line number.
        Value * strideLineCount = b->CreateLoad(b->CreateGEP(lineCountArrayWordPtr, sw.ix_MAXBIT));
        strideFinalLineNum = b->CreateAdd(pendingLineNum, b->CreateZExtOrTrunc(strideLineCount, sizeTy));
    }
    // Now check whether there are any matches at all in the stride.   If not, we
    // can immediately move on to the next stride.
    // We optimize for the case of no matches; the cost of the branch penalty
    // is expected to be small relative to the processing of each match.
    b->CreateLikelyCondBr(b->CreateICmpEQ(matchMask, sz_ZERO), matchesDone, strideMatchLoop);

    // Precondition: we have at least one more match to process.
    b->SetInsertPoint(strideMatchLoop);
    PHINode * const matchMaskPhi = b->CreatePHI(sizeTy, 2);
    matchMaskPhi->addIncoming(matchMask, updateLineInfo);
    PHINode * const matchWordPhi = b->CreatePHI(sizeTy, 2);
    matchWordPhi->addIncoming(sz_ZERO, updateLineInfo);

    // If we have any bits in the current matchWordPhi, continue with those, otherwise load
    // the next match word.
    Value * matchWordIdx = b->CreateCountForwardZeroes(matchMaskPhi, "matchWordIdx");
    Value * nextMatchWord = b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(matchWordBasePtr, matchWordIdx)), sizeTy);
    Value * matchBreakWord = b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(breakWordBasePtr, matchWordIdx)), sizeTy);
    Value * theMatchWord = b->CreateSelect(b->CreateICmpEQ(matchWordPhi, sz_ZERO), nextMatchWord, matchWordPhi);
    Value * matchWordPos = b->CreateAdd(stridePos, b->CreateMul(matchWordIdx, sw.WIDTH));
    Value * matchEndPosInWord = b->CreateCountForwardZeroes(theMatchWord);
    Value * matchEndPos = b->CreateAdd(matchWordPos, matchEndPosInWord, "matchEndPos");
    // Find the prior line break.  There are three possibilities.
    // (a) a prior break in the break word corresponding to the current match word.
    // (b) the last break in a prior word within the current stride.
    // (c) the pending line start from previous iterations.
    // Case (b) is most likely and requires a load of the prior break word.
    // We avoid branching by safely loading a prior word in any case and then
    // using selects to handle cases (a) and (c).
    Value * priorBreaksThisWord = b->CreateZeroHiBitsFrom(matchBreakWord, matchEndPosInWord);
    Value * priorBreaksInStride = b->CreateZeroHiBitsFrom(breakMask, matchWordIdx);
    Value * inWordCond = b->CreateICmpNE(priorBreaksThisWord, sz_ZERO);
    Value * inStrideCond = b->CreateICmpNE(priorBreaksInStride, sz_ZERO);
    Value * breakWordIdx = b->CreateSub(sz_MAXBIT, b->CreateCountReverseZeroes(priorBreaksInStride), "breakWordIdx_");
    // Create a safe index to load; the loaded value will be ignored for cases (a), (c).
    breakWordIdx = b->CreateSelect(inWordCond, matchWordIdx, b->CreateSelect(inStrideCond, breakWordIdx, sz_ZERO), "breakWordIdx");
    Value * breakWord = b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(breakWordBasePtr, breakWordIdx)), sizeTy);
    // For case (a), we use the previously masked value of the break word.
    breakWord = b->CreateSelect(inWordCond, priorBreaksThisWord, breakWord);   // cases (a) and (b)
    Value * lineStartInWord = b->CreateSub(sz_BITS, b->CreateCountReverseZeroes(breakWord));
    Value * lineStartBase = b->CreateAdd(stridePos, b->CreateMul(breakWordIdx, sw.WIDTH));
    Value * lineStartPos = b->CreateAdd(lineStartBase, lineStartInWord);
    // The break position is the line start for cases (a), (b); otherwise use the pending value.
    Value * const matchStart = b->CreateSelect(b->CreateOr(inWordCond, inStrideCond), lineStartPos, pendingLineStart, "matchStart");
    Value * matchRecordNum = nullptr;
    if (mLineNumbering) {
        // We must handle cases (a), (b), (c)
        // Get the line number for all positions up to and including the break word.
        Value * lineCountInStride = b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(lineCountArrayWordPtr, breakWordIdx)), sizeTy);
        // For case (a), we may need to subtract some breaks, if they are at the match end or later.
        Value * extraBreaks = b->CreateXor(breakWord, b->CreateSelect(inWordCond, matchBreakWord, breakWord));
        lineCountInStride = b->CreateSub(lineCountInStride, b->CreatePopcount(extraBreaks));
        // For case (c), there are no line breaks.
        lineCountInStride = b->CreateSelect(b->CreateOr(inWordCond, inStrideCond), lineCountInStride, sz_ZERO);
        matchRecordNum = b->CreateAdd(pendingLineNum, lineCountInStride);
    }
    // It is possible that the matchRecordEnd position is one past EOF.  Make sure not
    // to access past EOF.
    Value * const bufLimit = b->CreateSub(avail, sz_ONE);
    matchEndPos = b->CreateUMin(matchEndPos, bufLimit);
    // matchStart should never be past EOF, but in case it is....
    //b->CreateAssert(b->CreateICmpULT(matchStart, avail), "match position past EOF");
    b->CreateCondBr(b->CreateICmpULT(matchStart, avail), dispatch, callFinalizeScan);

    b->SetInsertPoint(dispatch);
    Function * const dispatcher = m->getFunction("accumulate_match_wrapper"); assert (dispatcher);
    Value * const startPtr = b->getRawInputPointer("InputStream", matchStart);
    Value * const endPtr = b->getRawInputPointer("InputStream", matchEndPos);
    auto argi = dispatcher->arg_begin();
    const auto matchRecNumArg = &*(argi++);
    Value * const matchRecNum = b->CreateZExtOrTrunc(matchRecordNum, matchRecNumArg->getType());
    b->CreateCall(dispatcher, {accumulator, matchRecNum, startPtr, endPtr});

    //  We've dealt with the match, now prepare for the next one, if any.
    // There may be more matches in the current word.
    Value * dropMatch = b->CreateResetLowestBit(theMatchWord, "dropMatch");
    Value * thisWordDone = b->CreateICmpEQ(dropMatch, sz_ZERO);
    // There may be more matches in the match mask.
    Value * resetMatchMask = b->CreateResetLowestBit(matchMaskPhi, "nextMatchMask");
    Value * nextMatchMask = b->CreateSelect(thisWordDone, resetMatchMask, matchMaskPhi);
    BasicBlock * currentBB = b->GetInsertBlock();
    matchMaskPhi->addIncoming(nextMatchMask, currentBB);
    matchWordPhi->addIncoming(dropMatch, currentBB);
    b->CreateCondBr(b->CreateICmpNE(nextMatchMask, sz_ZERO), strideMatchLoop, matchesDone);

    b->SetInsertPoint(matchesDone);
    PHINode * strideFinalLineStart = b->CreatePHI(sizeTy, 3);
    strideFinalLineStart->addIncoming(pendingLineStart, strideMasksReady);
    strideFinalLineStart->addIncoming(finalLineStartPos, updateLineInfo);
    strideFinalLineStart->addIncoming(finalLineStartPos, currentBB);
    PHINode * strideFinalLineNumPhi = nullptr;
    if (mLineNumbering) {
        strideFinalLineNumPhi = b->CreatePHI(sizeTy, 3);
        strideFinalLineNumPhi->addIncoming(pendingLineNum, strideMasksReady);
        strideFinalLineNumPhi->addIncoming(strideFinalLineNum, updateLineInfo);
        strideFinalLineNumPhi->addIncoming(strideFinalLineNum, currentBB);
    }
    strideNo->addIncoming(nextStrideNo, matchesDone);
    pendingLineStart->addIncoming(strideFinalLineStart, matchesDone);
    if (mLineNumbering) {
        pendingLineNum->addIncoming(strideFinalLineNumPhi, matchesDone);
    }
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), stridePrologue, stridesDone);

    b->SetInsertPoint(stridesDone);
    if (mLineNumbering) {
        b->setScalarField("LineNum", strideFinalLineNumPhi);
    }
    b->setProcessedItemCount("InputStream", strideFinalLineStart);
    b->CreateCondBr(mIsFinal, callFinalizeScan, scanReturn);

    b->SetInsertPoint(callFinalizeScan);
    b->setProcessedItemCount("InputStream", avail);
    Function * finalizer = m->getFunction("finalize_match_wrapper"); assert (finalizer);
    Value * const bufferEnd = b->getRawInputPointer("InputStream", avail);
    b->CreateCall(finalizer, {accumulator, bufferEnd});
    b->CreateBr(scanReturn);

    b->SetInsertPoint(scanReturn);
}

ScanMatchKernel::ScanMatchKernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * const Matches, StreamSet * const LineBreakStream, StreamSet * const ByteStream, Scalar * const callbackObject, unsigned strideBlocks)
    : MultiBlockKernel(b, "scanMatch" + std::to_string(strideBlocks),
// inputs
{Binding{"matchResult", Matches}
,Binding{"lineBreak", LineBreakStream}
,Binding{"InputStream", ByteStream, FixedRate(), { Deferred() }}},
// outputs
{},
// input scalars
{Binding{"accumulator_address", callbackObject}},
// output scalars
{},
// kernel state
{InternalScalar{b->getSizeTy(), "LineNum"}}) {
    addAttribute(SideEffecting());
    setStride(std::min(b->getBitBlockWidth() * strideBlocks, SIZE_T_BITS * SIZE_T_BITS));
}

enum MatchCoordinatesEnum {LINE_STARTS = 0, LINE_ENDS = 1, LINE_NUMBERS = 2};

MatchCoordinatesKernel::MatchCoordinatesKernel(const std::unique_ptr<kernel::KernelBuilder> & b,
                                               StreamSet * const Matches, StreamSet * const LineBreakStream,
                                               StreamSet * const Coordinates, unsigned strideBlocks)
: MultiBlockKernel(b, "matchCoordinates" + std::to_string(strideBlocks),
// inputs
{Binding{"matchResult", Matches}, Binding{"lineBreak", LineBreakStream, FixedRate(1), ZeroExtended()}},
// outputs
{Binding{"Coordinates", Coordinates, PopcountOf("matchResult")}},
// input scalars
{},
// output scalars
{},
// kernel state
{InternalScalar{b->getSizeTy(), "LineNum"},
 InternalScalar{b->getSizeTy(), "LineStart"}}) {
     // The stride size must be limited so that the scanword mask is a single size_t value.
     setStride(std::min(b->getBitBlockWidth() * strideBlocks, SIZE_T_BITS * SIZE_T_BITS));
     assert (Matches->getNumElements() == 1);
     assert (LineBreakStream->getNumElements() == 1);
     assert (Coordinates->getNumElements() == 3);
}

void MatchCoordinatesKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfStrides) {
    const bool mLineNumbering = true;

    // Determine the parameters for two-level scanning.
    ScanWordParameters sw(b, mStride);

    Constant * sz_STRIDE = b->getSize(mStride);
    Constant * sz_BLOCKS_PER_STRIDE = b->getSize(mStride/b->getBitBlockWidth());
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_ONE = b->getSize(1);
    Constant * sz_BITS = b->getSize(SIZE_T_BITS);
    Constant * sz_MAXBIT = b->getSize(SIZE_T_BITS - 1);
    Type * sizeTy = b->getSizeTy();

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const stridePrologue = b->CreateBasicBlock("stridePrologue");
    BasicBlock * const stridePrecomputation = b->CreateBasicBlock("stridePrecomputation");
    BasicBlock * const strideMasksReady = b->CreateBasicBlock("strideMasksReady");
    BasicBlock * const updateLineInfo = b->CreateBasicBlock("updateLineInfo");
    BasicBlock * const strideCoordinateLoop = b->CreateBasicBlock("strideCoordinateLoop");
    BasicBlock * const strideCoordinatesDone = b->CreateBasicBlock("strideCoordinatesDone");
    BasicBlock * const stridesDone = b->CreateBasicBlock("stridesDone");

    Value * const initialPos = b->getProcessedItemCount("matchResult");
    Value * const initialLineStart = b->getScalarField("LineStart");
    Value * initialLineNum = nullptr;
    Value * lineCountArrayBlockPtr = nullptr;
    Value * lineCountArrayWordPtr = nullptr;
    if (mLineNumbering) {
        initialLineNum = b->getScalarField("LineNum");
        lineCountArrayBlockPtr = b->CreateAlignedAlloca(b->getBitBlockType(),
                                                        b->getBitBlockWidth()/BITS_PER_BYTE,
                                                        sz_BLOCKS_PER_STRIDE);
        // Bitcast the lineNumberArrayptr to access by scanWord number
        lineCountArrayWordPtr = b->CreateBitCast(lineCountArrayBlockPtr, sw.pointerTy);
    }
    Value * const initialMatchCount = b->getProducedItemCount("Coordinates");
    b->CreateBr(stridePrologue);

    b->SetInsertPoint(stridePrologue);
    // Set up the loop variables as PHI nodes at the beginning of each stride.
    PHINode * const strideNo = b->CreatePHI(sizeTy, 2);
    strideNo->addIncoming(sz_ZERO, entryBlock);
    PHINode * const currenMatchCount = b->CreatePHI(sizeTy, 2);
    currenMatchCount->addIncoming(initialMatchCount, entryBlock);
    PHINode * const pendingLineStart = b->CreatePHI(sizeTy, 2);
    pendingLineStart->addIncoming(initialLineStart, entryBlock);
    PHINode * pendingLineNum = nullptr;
    if (mLineNumbering) {
        pendingLineNum = b->CreatePHI(sizeTy, 2);
        pendingLineNum->addIncoming(initialLineNum, entryBlock);
    }
    Value * stridePos = b->CreateAdd(initialPos, b->CreateMul(strideNo, sz_STRIDE));
    Value * strideBlockOffset = b->CreateMul(strideNo, sz_BLOCKS_PER_STRIDE);
    Value * nextStrideNo = b->CreateAdd(strideNo, sz_ONE);
    b->CreateBr(stridePrecomputation);
    // Precompute index masks for one stride of the match result and line break streams,
    // as well as a partial sum popcount of line numbers if line numbering is on.
    b->SetInsertPoint(stridePrecomputation);
    PHINode * const matchMaskAccum = b->CreatePHI(sizeTy, 2);
    matchMaskAccum->addIncoming(sz_ZERO, stridePrologue);
    PHINode * const breakMaskAccum = b->CreatePHI(sizeTy, 2);
    breakMaskAccum->addIncoming(sz_ZERO, stridePrologue);
    PHINode * const blockNo = b->CreatePHI(sizeTy, 2);
    blockNo->addIncoming(sz_ZERO, stridePrologue);
    PHINode * baseCounts = nullptr;
    if (mLineNumbering) {
        baseCounts = b->CreatePHI(b->getBitBlockType(), 2);
        baseCounts->addIncoming(b->allZeroes(), stridePrologue);
    }
    Value * strideBlockIndex = b->CreateAdd(strideBlockOffset, blockNo);
    Value * matchBitBlock = b->loadInputStreamBlock("matchResult", sz_ZERO, strideBlockIndex);
    Value * breakBitBlock = b->loadInputStreamBlock("lineBreak", sz_ZERO, strideBlockIndex);
    Value * const anyMatch = b->simd_any(sw.width, matchBitBlock);
    Value * const anyBreak = b->simd_any(sw.width, breakBitBlock);
    if (mLineNumbering) {
        Value * breakCounts = b->hsimd_partial_sum(sw.width, b->simd_popcount(sw.width, breakBitBlock));
        breakCounts = b->simd_add(sw.width, breakCounts, baseCounts);
        b->CreateBlockAlignedStore(b->bitCast(breakCounts), b->CreateGEP(lineCountArrayBlockPtr, blockNo));
        Value * baseCountsNext = b->bitCast(b->simd_fill(sw.width, b->mvmd_extract(sw.width, breakCounts, b->getBitBlockWidth()/sw.width - 1)));
        baseCounts->addIncoming(baseCountsNext, stridePrecomputation);
    }
    Value * matchWordMask = b->CreateZExtOrTrunc(b->hsimd_signmask(sw.width, anyMatch), sizeTy);
    Value * breakWordMask = b->CreateZExtOrTrunc(b->hsimd_signmask(sw.width, anyBreak), sizeTy);
    Value * matchMask = b->CreateOr(matchMaskAccum, b->CreateShl(matchWordMask, b->CreateMul(blockNo, sw.WORDS_PER_BLOCK)), "matchMask");
    Value * breakMask = b->CreateOr(breakMaskAccum, b->CreateShl(breakWordMask, b->CreateMul(blockNo, sw.WORDS_PER_BLOCK)), "breakMask");
    Value * const nextBlockNo = b->CreateAdd(blockNo, sz_ONE);
    matchMaskAccum->addIncoming(matchMask, stridePrecomputation);
    breakMaskAccum->addIncoming(breakMask, stridePrecomputation);
    blockNo->addIncoming(nextBlockNo, stridePrecomputation);
    b->CreateCondBr(b->CreateICmpNE(nextBlockNo, sz_BLOCKS_PER_STRIDE), stridePrecomputation, strideMasksReady);

    b->SetInsertPoint(strideMasksReady);
    // If there are no breaks in the stride, there are no matches.   We can move on to
    // the next stride immediately.
    b->CreateUnlikelyCondBr(b->CreateICmpEQ(breakMask, sz_ZERO), strideCoordinatesDone, updateLineInfo);

    b->SetInsertPoint(updateLineInfo);
    // We have at least one line break.   Determine the end-of-stride line start position
    // and line number, if needed.
    Value * matchWordBasePtr = b->getInputStreamBlockPtr("matchResult", sz_ZERO, strideBlockOffset);
    matchWordBasePtr = b->CreateBitCast(matchWordBasePtr, sw.pointerTy);
    Value * breakWordBasePtr = b->getInputStreamBlockPtr("lineBreak", sz_ZERO, strideBlockOffset);
    breakWordBasePtr = b->CreateBitCast(breakWordBasePtr, sw.pointerTy);

    Value * finalBreakIdx = b->CreateSub(sz_MAXBIT, b->CreateCountReverseZeroes(breakMask), "finalBreakIdx");
    Value * finalBreakWord = b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(breakWordBasePtr, finalBreakIdx)), sizeTy);
    Value * finalLineStartInWord = b->CreateSub(sz_BITS, b->CreateCountReverseZeroes(finalBreakWord));
    Value * finalLineStartPos = b->CreateAdd(stridePos, b->CreateMul(finalBreakIdx, sw.WIDTH));
    finalLineStartPos = b->CreateAdd(finalLineStartInWord, finalLineStartPos);
    Value * strideFinalLineNum = nullptr;
    if (mLineNumbering) {
        // compute the final line number.
        Value * strideLineCount = b->CreateLoad(b->CreateGEP(lineCountArrayWordPtr, sw.ix_MAXBIT));
        strideFinalLineNum = b->CreateAdd(pendingLineNum, b->CreateZExtOrTrunc(strideLineCount, sizeTy));
   }
    // Now check whether there are any matches at all in the stride.   If not, we
    // can immediately move on to the next stride.
    // We optimize for the case of no matches; the cost of the branch penalty
    // is expected to be small relative to the processing of each match.
    b->CreateLikelyCondBr(b->CreateICmpEQ(matchMask, sz_ZERO), strideCoordinatesDone, strideCoordinateLoop);

    // Precondition: we have at least one more match to process.
    b->SetInsertPoint(strideCoordinateLoop);
    PHINode * const matchMaskPhi = b->CreatePHI(sizeTy, 2);
    matchMaskPhi->addIncoming(matchMask, updateLineInfo);
    PHINode * const matchWordPhi = b->CreatePHI(sizeTy, 2);
    matchWordPhi->addIncoming(sz_ZERO, updateLineInfo);
    PHINode * const matchNumPhi = b->CreatePHI(sizeTy, 2, "matchNumPhi");
    matchNumPhi->addIncoming(currenMatchCount, updateLineInfo);

    // If we have any bits in the current matchWordPhi, continue with those, otherwise load
    // the next match word.
    Value * matchWordIdx = b->CreateCountForwardZeroes(matchMaskPhi, "matchWordIdx");
    Value * nextMatchWord = b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(matchWordBasePtr, matchWordIdx)), sizeTy);
    Value * matchBreakWord = b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(breakWordBasePtr, matchWordIdx)), sizeTy);
    Value * theMatchWord = b->CreateSelect(b->CreateICmpEQ(matchWordPhi, sz_ZERO), nextMatchWord, matchWordPhi);
    Value * matchWordPos = b->CreateAdd(stridePos, b->CreateMul(matchWordIdx, sw.WIDTH));
    Value * matchEndPosInWord = b->CreateCountForwardZeroes(theMatchWord);
    Value * matchEndPos = b->CreateAdd(matchWordPos, matchEndPosInWord, "matchEndPos");
    // Find the prior line break.  There are three possibilities.
    // (a) a prior break in the break word corresponding to the current match word.
    // (b) the last break in a prior word within the current stride.
    // (c) the pending line start from previous iterations.
    // Case (b) is most likely and requires a load of the prior break word.
    // We avoid branching by safely loading a prior word in any case and then
    // using selects to handle cases (a) and (c).
    Value * priorBreaksThisWord = b->CreateZeroHiBitsFrom(matchBreakWord, matchEndPosInWord);
    Value * priorBreaksInStride = b->CreateZeroHiBitsFrom(breakMask, matchWordIdx);
    Value * inWordCond = b->CreateICmpNE(priorBreaksThisWord, sz_ZERO);
    Value * inStrideCond = b->CreateICmpNE(priorBreaksInStride, sz_ZERO);
    Value * breakWordIdx = b->CreateSub(sz_MAXBIT, b->CreateCountReverseZeroes(priorBreaksInStride), "breakWordIdx_");
    // Create a safe index to load; the loaded value will be ignored for cases (a), (c).
    breakWordIdx = b->CreateSelect(inWordCond, matchWordIdx, b->CreateSelect(inStrideCond, breakWordIdx, sz_ZERO), "breakWordIdx");
    Value * breakWord = b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(breakWordBasePtr, breakWordIdx)), sizeTy);
    // For case (a), we use the previously masked value of the break word.
    breakWord = b->CreateSelect(inWordCond, priorBreaksThisWord, breakWord);   // cases (a) and (b)
    Value * lineStartInWord = b->CreateSub(sz_BITS, b->CreateCountReverseZeroes(breakWord));
    Value * lineStartBase = b->CreateAdd(stridePos, b->CreateMul(breakWordIdx, sw.WIDTH));
    Value * lineStartPos = b->CreateAdd(lineStartBase, lineStartInWord);
    // The break position is the line start for cases (a), (b); otherwise use the pending value.
    Value * const matchStart = b->CreateSelect(b->CreateOr(inWordCond, inStrideCond), lineStartPos, pendingLineStart, "matchStart");

    Value * const matchStartPtr = b->getRawOutputPointer("Coordinates", b->getInt32(LINE_STARTS), matchNumPhi);
    b->CreateStore(matchStart, matchStartPtr);
    Value * const lineEndsPtr = b->getRawOutputPointer("Coordinates", b->getInt32(LINE_ENDS), matchNumPhi);
    b->CreateStore(matchEndPos, lineEndsPtr);

    if (mLineNumbering) {
        // We must handle cases (a), (b), (c)
        // Get the line number for all positions up to and including the break word.
        Value * lineCountInStride = b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(lineCountArrayWordPtr, breakWordIdx)), sizeTy);
        // For case (a), we may need to subtract some breaks, if they are at the match end or later.
        Value * extraBreaks = b->CreateXor(breakWord, b->CreateSelect(inWordCond, matchBreakWord, breakWord));
        lineCountInStride = b->CreateSub(lineCountInStride, b->CreatePopcount(extraBreaks));
        // For case (c), there are no line breaks.
        lineCountInStride = b->CreateSelect(b->CreateOr(inWordCond, inStrideCond), lineCountInStride, sz_ZERO);
        Value * lineNum = b->CreateAdd(pendingLineNum, lineCountInStride);
        b->CreateStore(lineNum, b->getRawOutputPointer("Coordinates", b->getInt32(LINE_NUMBERS), matchNumPhi));
    }
    //  We've dealt with the match, now prepare for the next one, if any.
    // There may be more matches in the current word.
    Value * dropMatch = b->CreateResetLowestBit(theMatchWord);
    Value * thisWordDone = b->CreateICmpEQ(dropMatch, sz_ZERO);
    // There may be more matches in the match mask.
    Value * nextMatchMask = b->CreateSelect(thisWordDone, b->CreateResetLowestBit(matchMaskPhi), matchMaskPhi);
    BasicBlock * currentBB = b->GetInsertBlock();
    matchMaskPhi->addIncoming(nextMatchMask, currentBB);
    matchWordPhi->addIncoming(dropMatch, currentBB);
    Value * nextMatchNum = b->CreateAdd(matchNumPhi, sz_ONE);
    matchNumPhi->addIncoming(nextMatchNum, currentBB);
    b->CreateCondBr(b->CreateICmpNE(nextMatchMask, sz_ZERO), strideCoordinateLoop, strideCoordinatesDone);

    b->SetInsertPoint(strideCoordinatesDone);
    PHINode * finalStrideMatchCount = b->CreatePHI(sizeTy, 3);
    finalStrideMatchCount->addIncoming(currenMatchCount, strideMasksReady);
    finalStrideMatchCount->addIncoming(currenMatchCount, updateLineInfo);
    finalStrideMatchCount->addIncoming(nextMatchNum, currentBB);
    PHINode * strideFinalLineStart = b->CreatePHI(sizeTy, 3);
    strideFinalLineStart->addIncoming(pendingLineStart, strideMasksReady);
    strideFinalLineStart->addIncoming(finalLineStartPos, updateLineInfo);
    strideFinalLineStart->addIncoming(finalLineStartPos, currentBB);
    PHINode * strideFinalLineNumPhi = nullptr;
    if (mLineNumbering) {
        strideFinalLineNumPhi = b->CreatePHI(sizeTy, 3);
        strideFinalLineNumPhi->addIncoming(pendingLineNum, strideMasksReady);
        strideFinalLineNumPhi->addIncoming(strideFinalLineNum, updateLineInfo);
        strideFinalLineNumPhi->addIncoming(strideFinalLineNum, currentBB);
    }
    strideNo->addIncoming(nextStrideNo, strideCoordinatesDone);
    currenMatchCount->addIncoming(finalStrideMatchCount, strideCoordinatesDone);
    pendingLineStart->addIncoming(strideFinalLineStart, strideCoordinatesDone);
    if (mLineNumbering) {
        pendingLineNum->addIncoming(strideFinalLineNumPhi, strideCoordinatesDone);
    }
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), stridePrologue, stridesDone);

    b->SetInsertPoint(stridesDone);
    b->setScalarField("LineStart", strideFinalLineStart);
    if (mLineNumbering) {
        b->setScalarField("LineNum", strideFinalLineNumPhi);
    }
    // b->setProducedItemCount("Coordinates", finalStrideMatchCount);
}

MatchReporter::MatchReporter(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * ByteStream, StreamSet * const Coordinates, Scalar * const callbackObject)
: SegmentOrientedKernel(b, "matchReporter" + std::to_string(Coordinates->getNumElements()),
// inputs
{Binding{"InputStream", ByteStream, GreedyRate(), Deferred()},
 Binding{"Coordinates", Coordinates, GreedyRate(1)}},
// outputs
{},
// input scalars
{Binding{"accumulator_address", callbackObject}},
// output scalars
{},
// kernel state
{}) {
    setStride(1);
    addAttribute(SideEffecting());
}

    // TO DO:  investigate add linebreaks as input:  set consumed by the last linebreak?

void MatchReporter::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) {
    Module * const m = b->getModule();
    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const processMatchCoordinates = b->CreateBasicBlock("processMatchCoordinates");
    BasicBlock * const dispatch = b->CreateBasicBlock("dispatch");
    BasicBlock * const coordinatesDone = b->CreateBasicBlock("coordinatesDone");
    BasicBlock * const callFinalizeScan = b->CreateBasicBlock("callFinalizeScan");
    BasicBlock * const scanReturn = b->CreateBasicBlock("scanReturn");

    Value * accumulator = b->getScalarField("accumulator_address");
    Value * const avail = b->getAvailableItemCount("InputStream");
    Value * matchesProcessed = b->getProcessedItemCount("Coordinates");
    Value * matchesAvail = b->getAvailableItemCount("Coordinates");

    Constant * const sz_ONE = b->getSize(1);

    b->CreateCondBr(b->CreateICmpNE(matchesProcessed, matchesAvail), processMatchCoordinates, coordinatesDone);

    b->SetInsertPoint(processMatchCoordinates);
    PHINode * phiMatchNum = b->CreatePHI(b->getSizeTy(), 2, "matchNum");
    phiMatchNum->addIncoming(matchesProcessed, entryBlock);

    Value * nextMatchNum = b->CreateAdd(phiMatchNum, sz_ONE);

    Value * matchRecordStart = b->CreateLoad(b->getRawInputPointer("Coordinates", b->getInt32(LINE_STARTS), phiMatchNum), "matchStartLoad");
    Value * matchRecordEnd = b->CreateLoad(b->getRawInputPointer("Coordinates", b->getInt32(LINE_ENDS), phiMatchNum), "matchEndLoad");
    Value * matchRecordNum = b->CreateLoad(b->getRawInputPointer("Coordinates", b->getInt32(LINE_NUMBERS), phiMatchNum), "matchNumLoad");

    // It is possible that the matchRecordEnd position is one past EOF.  Make sure not
    // to access past EOF.
    Value * const bufLimit = b->CreateSub(avail, sz_ONE);
    matchRecordEnd = b->CreateUMin(matchRecordEnd, bufLimit);
    // matchStart should never be past EOF, but in case it is....
    //b->CreateAssert(b->CreateICmpULT(matchRecordStart, avail), "match position past EOF");
    b->CreateCondBr(b->CreateICmpULT(matchRecordStart, avail), dispatch, callFinalizeScan);

    b->SetInsertPoint(dispatch);
    Function * const dispatcher = m->getFunction("accumulate_match_wrapper"); assert (dispatcher);
    Value * const startPtr = b->getRawInputPointer("InputStream", matchRecordStart);
    Value * const endPtr = b->getRawInputPointer("InputStream", matchRecordEnd);
    auto argi = dispatcher->arg_begin();
    const auto matchRecNumArg = &*(argi++);
    Value * const matchRecNum = b->CreateZExtOrTrunc(matchRecordNum, matchRecNumArg->getType());
    b->CreateCall(dispatcher, {accumulator, matchRecNum, startPtr, endPtr});
    Value * haveMoreMatches = b->CreateICmpNE(nextMatchNum, matchesAvail);
    phiMatchNum->addIncoming(nextMatchNum, b->GetInsertBlock());
    b->CreateCondBr(haveMoreMatches, processMatchCoordinates, coordinatesDone);

    b->SetInsertPoint(coordinatesDone);
    //b->setProcessedItemCount("InputStream", matchRecordEnd);
    b->CreateCondBr(mIsFinal, callFinalizeScan, scanReturn);

    b->SetInsertPoint(callFinalizeScan);
    b->setProcessedItemCount("InputStream", avail);
    Function * finalizer = m->getFunction("finalize_match_wrapper"); assert (finalizer);
    Value * const bufferEnd = b->getRawInputPointer("InputStream", avail);
    b->CreateCall(finalizer, {accumulator, bufferEnd});
    b->CreateBr(scanReturn);

    b->SetInsertPoint(scanReturn);

}

}
