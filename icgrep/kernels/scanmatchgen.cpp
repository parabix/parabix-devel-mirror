/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "scanmatchgen.h"
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Module.h>
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>
#include <grep/grep_engine.h>

using namespace llvm;

namespace kernel {

void ScanMatchKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfStrides) {

    Module * const m = b->getModule();

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const scanMatchStride = b->CreateBasicBlock("scanMatchStride");
    BasicBlock * const scanWordIteration = b->CreateBasicBlock("ScanWordIteration");
    BasicBlock * const matches_test_block = b->CreateBasicBlock("matches_test_block");
    BasicBlock * const processMatchesEntry = b->CreateBasicBlock("process_matches_loop");
    BasicBlock * const prior_breaks_block = b->CreateBasicBlock("prior_breaks_block");
    BasicBlock * const loop_final_block = b->CreateBasicBlock("loop_final_block");
    BasicBlock * const processMatchesExit = b->CreateBasicBlock("matches_done_block");
    BasicBlock * const remaining_breaks_block = b->CreateBasicBlock("remaining_breaks_block");
    BasicBlock * const return_block = b->CreateBasicBlock("return_block");
    BasicBlock * const finalizeStride = b->CreateBasicBlock("finalizeStride");
    BasicBlock * const stridesDone = b->CreateBasicBlock("stridesDone");
    BasicBlock * const callFinalizeScan = b->CreateBasicBlock("callFinalizeScan");
    BasicBlock * const scanReturn = b->CreateBasicBlock("scanReturn");
    IntegerType * const sizeTy = b->getSizeTy();
    const unsigned scansPerStride = mStride / sizeTy->getBitWidth();
    PointerType * const scanwordPointerType =  sizeTy->getPointerTo();
    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);
    Constant * const BITBLOCK_WIDTH = b->getSize(b->getBitBlockWidth());
    Value * const initialPos = b->getProcessedItemCount("matchResult");
    //b->CallPrintInt("initialPos", initialPos);
    Value * accumulator = b->getScalarField("accumulator_address");
    Value * const avail = b->getAvailableItemCount("InputStream");
//    b->CallPrintInt("numOfStrides", numOfStrides);
//    b->CallPrintInt("avail", avail);
//    for (unsigned i = 0; i < mStride/b->getBitBlockWidth(); i++) {
//        b->CallPrintRegister("matches[" + std::to_string(i) + "]", b->loadInputStreamBlock("matchResult", ZERO, b->getSize(i)));
//    }
    b->CreateBr(scanMatchStride);

    b->SetInsertPoint(scanMatchStride);
    PHINode * const positionOffset = b->CreatePHI(sizeTy, 2);
    positionOffset->addIncoming(ZERO, entryBlock);
    Value * blockOffset = b->CreateUDiv(positionOffset, BITBLOCK_WIDTH);
    Value * matches = b->getInputStreamBlockPtr("matchResult", ZERO, blockOffset);
    matches = b->CreateBitCast(matches, scanwordPointerType);
    Value * linebreaks = b->getInputStreamBlockPtr("lineBreak", ZERO, blockOffset);
    linebreaks = b->CreateBitCast(linebreaks, scanwordPointerType);
    Value * const scanwordPos = b->CreateAdd(initialPos, positionOffset);
    Value * const consumed = b->getProcessedItemCount("InputStream");
    Value * const consumedLines = b->getScalarField("LineNum");
    //b->CallPrintInt("consumed", consumed);
    //b->CallPrintInt("consumedPtr", b->getRawInputPointer("InputStream", consumed));

    b->CreateBr(scanWordIteration);

    b->SetInsertPoint(scanWordIteration);

        // while (phiIndex < words per stride)
        PHINode * const phiIndex = b->CreatePHI(sizeTy, 2, "index");
        phiIndex->addIncoming(ZERO, scanMatchStride);
        PHINode * const phiScanwordPos = b->CreatePHI(scanwordPos->getType(), 2, "pos");
        phiScanwordPos->addIncoming(scanwordPos, scanMatchStride);
        PHINode * const phiLineStart = b->CreatePHI(consumed->getType(), 2, "recordstart");
        phiLineStart->addIncoming(consumed, scanMatchStride);
        PHINode * const phiLineNum = b->CreatePHI(consumedLines->getType(), 2, "recordnum");
        phiLineNum->addIncoming(consumedLines, scanMatchStride);

        Value * const matchWord = b->CreateLoad(b->CreateGEP(matches, phiIndex));
        Value * const recordBreaks = b->CreateLoad(b->CreateGEP(linebreaks, phiIndex));
        // The match scanner works with a loop involving four variables:
        // (a) the bit stream scanword of matches marking the ends of selected records,
        // (b) the bit stream scanword of record_breaks marking the ends of all records,
        // (c) the integer lastRecordNum indicating the number of records processed so far,
        // (d) the index lastRecordStart indicating the file position of the last record.
        // We set up a loop structure, in which a set of 4 phi nodes initialize these
        // variables from either the input to the scanner or the computed values within
        // the loop body.

        b->CreateBr(matches_test_block);

        // LOOP Test Block
        b->SetInsertPoint(matches_test_block);
        PHINode * const phiMatchWord = b->CreatePHI(sizeTy, 2, "matches");
        PHINode * const phiRecordBreaks = b->CreatePHI(sizeTy, 2, "recordbreaks");
        PHINode * const phiRecordStart = b->CreatePHI(sizeTy, 2, "recordstart");
        PHINode * const phiRecordNum = b->CreatePHI(sizeTy, 2, "recordnum");
        phiMatchWord->addIncoming(matchWord, scanWordIteration);
        phiRecordBreaks->addIncoming(recordBreaks, scanWordIteration);
        phiRecordStart->addIncoming(phiLineStart, scanWordIteration);
        phiRecordNum->addIncoming(phiLineNum, scanWordIteration);
        Value * const anyMatches = b->CreateICmpNE(phiMatchWord, ZERO);
        b->CreateCondBr(anyMatches, processMatchesEntry, processMatchesExit);

            // LOOP BODY
            // The loop body is entered if we have more matches to process.
            b->SetInsertPoint(processMatchesEntry);
            Value * prior_breaks = b->CreateAnd(b->CreateMaskToLowestBitExclusive(phiMatchWord), phiRecordBreaks);
            // Within the loop we have a conditional block that is executed if there are any prior record breaks.
            Value * prior_breaks_cond = b->CreateICmpNE(prior_breaks, ZERO);
            b->CreateCondBr(prior_breaks_cond, prior_breaks_block, loop_final_block);

                // PRIOR_BREAKS_BLOCK
                // If there are prior breaks, we count them and compute the record start position.
                b->SetInsertPoint(prior_breaks_block);
                Value * matchedRecordNum = b->CreateAdd(b->CreatePopcount(prior_breaks), phiRecordNum);
                Value * reverseDistance = b->CreateCountReverseZeroes(prior_breaks, true);
                Value * width = ConstantInt::get(sizeTy, sizeTy->getBitWidth());
                Value * priorRecordStart = b->CreateAdd(phiScanwordPos, b->CreateSub(width, reverseDistance));
                b->CreateBr(loop_final_block);

            // LOOP FINAL BLOCK
            // The prior breaks, if any have been counted.  Set up phi nodes for the recordNum
            // and recortStart depending on whether the conditional execution of prior_breaks_block.
            b->SetInsertPoint(loop_final_block);
            PHINode * matchRecordNum = b->CreatePHI(sizeTy, 2, "matchRecordNum");
            matchRecordNum->addIncoming(phiRecordNum, processMatchesEntry);
            matchRecordNum->addIncoming(matchedRecordNum, prior_breaks_block);
            phiRecordNum->addIncoming(matchRecordNum, loop_final_block);

            PHINode * const matchRecordStart = b->CreatePHI(sizeTy, 2, "matchRecordStart");
            matchRecordStart->addIncoming(phiRecordStart, processMatchesEntry);
            matchRecordStart->addIncoming(priorRecordStart, prior_breaks_block);
            phiRecordStart->addIncoming(matchRecordStart, loop_final_block);
            Value * matchRecordEnd = b->CreateAdd(phiScanwordPos, b->CreateCountForwardZeroes(phiMatchWord, true));
            // It is possible that the matchRecordEnd position is one past EOF.  Make sure not
            // to access past EOF.
            Value * const bufLimit = b->CreateSub(avail, ONE);
            //b->CallPrintInt("bufLimit", bufLimit);
            //b->CallPrintInt("matchRecordEnd", matchRecordEnd);
            matchRecordEnd = b->CreateUMin(matchRecordEnd, bufLimit);
            Function * const dispatcher = m->getFunction("accumulate_match_wrapper"); assert (dispatcher);
            Value * const startPtr = b->getRawInputPointer("InputStream", matchRecordStart);
            Value * const endPtr = b->getRawInputPointer("InputStream", matchRecordEnd);
            auto argi = dispatcher->arg_begin();
            const auto matchRecNumArg = &*(argi++);
            Value * const matchRecNum = b->CreateZExtOrTrunc(matchRecordNum, matchRecNumArg->getType());
            b->CreateCall(dispatcher, {accumulator, matchRecNum, startPtr, endPtr});
            Value * remaining_matches = b->CreateResetLowestBit(phiMatchWord);
            phiMatchWord->addIncoming(remaining_matches, loop_final_block);

            Value * remaining_breaks = b->CreateXor(phiRecordBreaks, prior_breaks);
            phiRecordBreaks->addIncoming(remaining_breaks, loop_final_block);

            b->CreateBr(matches_test_block);

        // LOOP EXIT/MATCHES_DONE
        b->SetInsertPoint(processMatchesExit);
        // When the matches are done, there may be additional record breaks remaining
        Value * more_breaks_cond = b->CreateICmpNE(phiRecordBreaks, ZERO);
        b->CreateCondBr(more_breaks_cond, remaining_breaks_block, return_block);

            // REMAINING_BREAKS_BLOCK: process remaining record breaks after all matches are processed
            b->SetInsertPoint(remaining_breaks_block);
            Value * break_count = b->CreatePopcount(phiRecordBreaks);
            Value * final_record_num = b->CreateAdd(phiRecordNum, break_count);
            Value * reverseZeroes = b->CreateCountReverseZeroes(phiRecordBreaks);
            Value * pendingLineStart = b->CreateAdd(phiScanwordPos, b->CreateSub(width, reverseZeroes));
            b->CreateBr(return_block);

        // RETURN block
        b->SetInsertPoint(return_block);
        PHINode * phiFinalRecordNum = b->CreatePHI(sizeTy, 2, "finalRecordCount");
        PHINode * phiFinalRecordStart = b->CreatePHI(sizeTy, 2, "finalRecordStart");

        phiFinalRecordNum->addIncoming(phiRecordNum, processMatchesExit);
        phiFinalRecordNum->addIncoming(final_record_num, remaining_breaks_block);
        phiLineNum->addIncoming(phiFinalRecordNum, return_block);

        phiFinalRecordStart->addIncoming(phiRecordStart, processMatchesExit);
        phiFinalRecordStart->addIncoming(pendingLineStart, remaining_breaks_block);
        phiLineStart->addIncoming(phiFinalRecordStart, return_block);

        Value * nextScanwordPos = b->CreateAdd(phiScanwordPos, ConstantInt::get(sizeTy, sizeTy->getBitWidth()));
        phiScanwordPos->addIncoming(nextScanwordPos, return_block);
        Value * nextIndex = b->CreateAdd(phiIndex, ONE);
        phiIndex->addIncoming(nextIndex, return_block);
        b->CreateLikelyCondBr(b->CreateICmpNE(nextIndex, b->getSize(scansPerStride)), scanWordIteration, finalizeStride);

    b->SetInsertPoint(finalizeStride);
    b->setScalarField("LineNum", phiFinalRecordNum);
    b->setProcessedItemCount("InputStream", phiFinalRecordStart);
    Value * const nextPositionOffset = b->CreateAdd(positionOffset, b->getSize(mStride));
    Value * const nextStride = b->CreateUDiv(nextPositionOffset, b->getSize(mStride));
    positionOffset->addIncoming(nextPositionOffset, finalizeStride);
    b->CreateLikelyCondBr(b->CreateICmpNE(nextStride, numOfStrides), scanMatchStride, stridesDone);

    b->SetInsertPoint(stridesDone);
    b->CreateCondBr(mIsFinal, callFinalizeScan, scanReturn);

    b->SetInsertPoint(callFinalizeScan);
    b->setProcessedItemCount("InputStream", avail);
    Function * finalizer = m->getFunction("finalize_match_wrapper"); assert (finalizer);
    Value * const bufferEnd = b->getRawInputPointer("InputStream", avail);
    b->CreateCall(finalizer, {accumulator, bufferEnd});
    b->CreateBr(scanReturn);

    b->SetInsertPoint(scanReturn);
}

ScanMatchKernel::ScanMatchKernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * const Matches, StreamSet * const LineBreakStream, StreamSet * const ByteStream, Scalar * const callbackObject)
: MultiBlockKernel(b, "scanMatch",
// inputs
{Binding{"matchResult", Matches }
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
    setStride(b->getBitBlockWidth() * 2);
}

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


enum MatchCoordinatesEnum {LINE_STARTS = 0, LINE_ENDS = 1, LINE_NUMBERS = 2};

MatchCoordinatesKernel::MatchCoordinatesKernel(const std::unique_ptr<kernel::KernelBuilder> & b,
                                               StreamSet * const Matches, StreamSet * const LineBreakStream,
                                               StreamSet * const Coordinates, unsigned strideBlocks)
: MultiBlockKernel(b, "matchCoordinates" + std::to_string(Coordinates->getNumElements()),
// inputs
{Binding{"matchResult", Matches}, Binding{"lineBreak", LineBreakStream}},
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
// TODO: Find something better than the following hack deals with the case that matchResult
// has reported a match at one past EOF, while there is no linebreak bit at that position.
    breakBitBlock = b->simd_or(breakBitBlock, matchBitBlock);
    //b->CallPrintRegister("matchBitBlock", matchBitBlock);
    //b->CallPrintRegister("breakBitBlock", breakBitBlock);
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
    Value * matchEndPos = b->CreateAdd(matchWordPos, b->CreateCountForwardZeroes(theMatchWord), "matchEndPos");
    // Find the prior line break.  There are three possibilities.
    // (a) a prior break in the break word corresponding to the current match word.
    // (b) the last break in a prior word within the current stride.
    // (c) the pending line start from previous iterations.
    // Case (b) is most likely and requires a load of the prior break word.
    // We avoid branching by safely loading a prior word in any case and then
    // using selects to handle cases (a) and (c).
    Value * priorBreaksThisWord = b->CreateAnd(b->CreateMaskToLowestBitExclusive(theMatchWord), matchBreakWord, "priorBreaksThisWord");
    Value * priorBreaksInStride = b->CreateAnd(b->CreateMaskToLowestBitExclusive(matchMaskPhi), breakMask, "priorBreaksInStride");
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
//    b->CallPrintInt("matchStart", matchStart);
//    b->CallPrintInt("matchStartPtr.LINE_STARTS", matchStartPtr);
    b->CreateStore(matchStart, matchStartPtr);

    Value * const lineEndsPtr = b->getRawOutputPointer("Coordinates", b->getInt32(LINE_ENDS), matchNumPhi);
//    b->CallPrintInt("matchEndPos", matchEndPos);
//    b->CallPrintInt("matchStartPtr.LINE_ENDS", lineEndsPtr);
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
        //b->CallPrintInt("  lineNum", lineNum);
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
    BasicBlock * const coordinatesDone = b->CreateBasicBlock("coordinatesDone");
    BasicBlock * const exit = b->CreateBasicBlock("exit");

    Value * accumulator = b->getScalarField("accumulator_address");
    Value * const avail = b->getAvailableItemCount("InputStream");
    Value * matchesProcessed = b->getProcessedItemCount("Coordinates");
    Value * matchesAvail = b->getAvailableItemCount("Coordinates");

    Constant * const sz_ONE = b->getSize(1);

    b->CreateCondBr(b->CreateICmpNE(matchesProcessed, matchesAvail), processMatchCoordinates, exit);

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
    b->setProcessedItemCount("InputStream", matchRecordEnd);
    b->CreateBr(exit);

    b->SetInsertPoint(exit);
}

}
