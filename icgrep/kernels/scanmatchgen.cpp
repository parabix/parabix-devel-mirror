/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "scanmatchgen.h"
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Module.h>
#include <kernels/kernel_builder.h>
#include <IR_Gen/FunctionTypeBuilder.h>
#include <llvm/Support/raw_ostream.h>
#include <grep_engine.h>

using namespace llvm;

inline static unsigned floor_log2(const unsigned v) {
    assert ("log2(0) is undefined!" && v != 0);
    return 31 - __builtin_clz(v);
}

namespace kernel {

Value * ScanMatchKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &iBuilder, Value * const numOfStrides) {

    Module * const m = iBuilder->getModule();
    
    BasicBlock * const entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * const initialBlock = iBuilder->CreateBasicBlock("initialBlock");
    BasicBlock * const scanWordIteration = iBuilder->CreateBasicBlock("ScanWordIteration");
    BasicBlock * const matches_test_block = iBuilder->CreateBasicBlock("matches_test_block");
    BasicBlock * const processMatchesEntry = iBuilder->CreateBasicBlock("process_matches_loop");
    BasicBlock * const prior_breaks_block = iBuilder->CreateBasicBlock("prior_breaks_block");
    BasicBlock * const loop_final_block = iBuilder->CreateBasicBlock("loop_final_block");
    BasicBlock * const processMatchesExit = iBuilder->CreateBasicBlock("matches_done_block");
    BasicBlock * const remaining_breaks_block = iBuilder->CreateBasicBlock("remaining_breaks_block");
    BasicBlock * const return_block = iBuilder->CreateBasicBlock("return_block");
    BasicBlock * const scanWordExit = iBuilder->CreateBasicBlock("ScanWordExit");
    BasicBlock * const blocksExit = iBuilder->CreateBasicBlock("blocksExit");
    BasicBlock * const callFinalizeScan = iBuilder->CreateBasicBlock("callFinalizeScan");
    BasicBlock * const scanReturn = iBuilder->CreateBasicBlock("scanReturn");
    IntegerType * const sizeTy = iBuilder->getSizeTy();
    const unsigned fieldCount = iBuilder->getBitBlockWidth() / sizeTy->getBitWidth();
    VectorType * const scanwordVectorType =  VectorType::get(sizeTy, fieldCount);
    Constant * const ZERO = ConstantInt::getNullValue(sizeTy);

    Value * match_result = iBuilder->getInputStreamBlockPtr("matchResult", iBuilder->getInt32(0));
    Value * line_break = iBuilder->getInputStreamBlockPtr("lineBreak", iBuilder->getInt32(0));

    Value * blocksToDo = iBuilder->CreateAdd(numOfStrides, iBuilder->CreateZExt(mIsFinal, numOfStrides->getType()));
    blocksToDo = iBuilder->CreateMul(blocksToDo, iBuilder->getSize(mStride / iBuilder->getBitBlockWidth()));
    
    Value * match_result_ptr = iBuilder->CreateBitCast(match_result, scanwordVectorType->getPointerTo());
    Value * line_break_ptr = iBuilder->CreateBitCast(line_break, scanwordVectorType->getPointerTo());
    Value * accumulator = iBuilder->getScalarField("accumulator_address");

    iBuilder->CreateCondBr(iBuilder->CreateICmpUGT(blocksToDo, iBuilder->getSize(0)), initialBlock, blocksExit);

    iBuilder->SetInsertPoint(initialBlock);
    PHINode * const blockBase = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
    blockBase->addIncoming(iBuilder->getSize(0), entryBlock);
    Value * const blockNo = iBuilder->getScalarField("BlockNo");
    Value * const scanwordPos = iBuilder->CreateShl(blockNo, floor_log2(iBuilder->getBitBlockWidth()));
    Value * const lastRecordStart = iBuilder->getProcessedItemCount("InputStream");
    Value * const lastRecordNum = iBuilder->getScalarField("LineNum");
    iBuilder->CreateBr(scanWordIteration);

    iBuilder->SetInsertPoint(scanWordIteration);

        // while (phiIndex < words per stride)
        PHINode * const phiIndex = iBuilder->CreatePHI(iBuilder->getInt32Ty(), 2, "index");
        phiIndex->addIncoming(iBuilder->getInt32(0), initialBlock);
        PHINode * const phiScanwordPos = iBuilder->CreatePHI(scanwordPos->getType(), 2, "pos");
        phiScanwordPos->addIncoming(scanwordPos, initialBlock);
        PHINode * const phiLineStart = iBuilder->CreatePHI(lastRecordStart->getType(), 2, "recordstart");
        phiLineStart->addIncoming(lastRecordStart, initialBlock);
        PHINode * const phiLineNum = iBuilder->CreatePHI(lastRecordNum->getType(), 2, "recordnum");
        phiLineNum->addIncoming(lastRecordNum, initialBlock);
        Value * const matches = iBuilder->CreateLoad(iBuilder->CreateGEP(match_result_ptr, blockBase));
        Value * const linebreaks = iBuilder->CreateLoad(iBuilder->CreateGEP(line_break_ptr, blockBase));
        Value * const matchWord = iBuilder->CreateExtractElement(matches, phiIndex);
        Value * const recordBreaks = iBuilder->CreateExtractElement(linebreaks, phiIndex);

        // The match scanner works with a loop involving four variables:
        // (a) the bit stream scanword of matches marking the ends of selected records,
        // (b) the bit stream scanword of record_breaks marking the ends of all records,
        // (c) the integer lastRecordNum indicating the number of records processed so far,
        // (d) the index lastRecordStart indicating the file position of the last record.
        // We set up a loop structure, in which a set of 4 phi nodes initialize these
        // variables from either the input to the scanner or the computed values within
        // the loop body.

        iBuilder->CreateBr(matches_test_block);

        // LOOP Test Block
        iBuilder->SetInsertPoint(matches_test_block);
        PHINode * phiMatchWord = iBuilder->CreatePHI(sizeTy, 2, "matches");
        PHINode * phiRecordBreaks = iBuilder->CreatePHI(sizeTy, 2, "recordbreaks");
        PHINode * phiRecordStart = iBuilder->CreatePHI(sizeTy, 2, "recordstart");
        PHINode * phiRecordNum = iBuilder->CreatePHI(sizeTy, 2, "recordnum");
        phiMatchWord->addIncoming(matchWord, scanWordIteration);
        phiRecordBreaks->addIncoming(recordBreaks, scanWordIteration);
        phiRecordNum->addIncoming(phiLineNum, scanWordIteration);
        phiRecordStart->addIncoming(phiLineStart, scanWordIteration);
        Value * anyMatches = iBuilder->CreateICmpNE(phiMatchWord, ZERO);
        iBuilder->CreateCondBr(anyMatches, processMatchesEntry, processMatchesExit);

            // LOOP BODY
            // The loop body is entered if we have more matches to process.
            iBuilder->SetInsertPoint(processMatchesEntry);
            Value * prior_breaks = iBuilder->CreateAnd(iBuilder->CreateMaskToLowestBitExclusive(phiMatchWord), phiRecordBreaks);
            // Within the loop we have a conditional block that is executed if there are any prior record breaks.
            Value * prior_breaks_cond = iBuilder->CreateICmpNE(prior_breaks, ZERO);
            iBuilder->CreateCondBr(prior_breaks_cond, prior_breaks_block, loop_final_block);

                // PRIOR_BREAKS_BLOCK
                // If there are prior breaks, we count them and compute the record start position.
                iBuilder->SetInsertPoint(prior_breaks_block);
                Value * matchedRecordNum = iBuilder->CreateAdd(iBuilder->CreatePopcount(prior_breaks), phiRecordNum);
                Value * reverseDistance = iBuilder->CreateCountReverseZeroes(prior_breaks, true);
                Value * width = ConstantInt::get(sizeTy, sizeTy->getBitWidth());
                Value * priorRecordStart = iBuilder->CreateAdd(phiScanwordPos, iBuilder->CreateSub(width, reverseDistance));                
                iBuilder->CreateBr(loop_final_block);

            // LOOP FINAL BLOCK
            // The prior breaks, if any have been counted.  Set up phi nodes for the recordNum
            // and recortStart depending on whether the conditional execution of prior_breaks_block.
            iBuilder->SetInsertPoint(loop_final_block);
            PHINode * matchRecordNum = iBuilder->CreatePHI(sizeTy, 2, "matchRecordNum");
            matchRecordNum->addIncoming(phiRecordNum, processMatchesEntry);
            matchRecordNum->addIncoming(matchedRecordNum, prior_breaks_block);
            phiRecordNum->addIncoming(matchRecordNum, loop_final_block);

            PHINode * const matchRecordStart = iBuilder->CreatePHI(sizeTy, 2, "matchRecordStart");
            matchRecordStart->addIncoming(phiRecordStart, processMatchesEntry);
            matchRecordStart->addIncoming(priorRecordStart, prior_breaks_block);
            phiRecordStart->addIncoming(matchRecordStart, loop_final_block);
            Value * matchRecordEnd = iBuilder->CreateAdd(phiScanwordPos, iBuilder->CreateCountForwardZeroes(phiMatchWord, true));
            // It is possible that the matchRecordEnd position is one past EOF.  Make sure not
            // to access past EOF.
            Value * bufLimit = iBuilder->CreateSub(iBuilder->getBufferedSize("InputStream"), ConstantInt::get(sizeTy, 1));
            matchRecordEnd = iBuilder->CreateSelect(iBuilder->CreateICmpULT(matchRecordEnd, bufLimit), matchRecordEnd, bufLimit);
            Function * const dispatcher = m->getFunction("accumulate_match_wrapper"); assert (dispatcher);
            Value * const startPtr = iBuilder->getRawInputPointer("InputStream", matchRecordStart);
            Value * const endPtr = iBuilder->getRawInputPointer("InputStream", matchRecordEnd);
            iBuilder->CreateCall(dispatcher, {accumulator, matchRecordNum, startPtr, endPtr});
            Value * remaining_matches = iBuilder->CreateResetLowestBit(phiMatchWord);
            phiMatchWord->addIncoming(remaining_matches, loop_final_block);

            Value * remaining_breaks = iBuilder->CreateXor(phiRecordBreaks, prior_breaks);
            phiRecordBreaks->addIncoming(remaining_breaks, loop_final_block);

            iBuilder->CreateBr(matches_test_block);

        // LOOP EXIT/MATCHES_DONE
        iBuilder->SetInsertPoint(processMatchesExit);
        // When the matches are done, there may be additional record breaks remaining
        Value * more_breaks_cond = iBuilder->CreateICmpNE(phiRecordBreaks, ZERO);
        iBuilder->CreateCondBr(more_breaks_cond, remaining_breaks_block, return_block);

            // REMAINING_BREAKS_BLOCK: process remaining record breaks after all matches are processed
            iBuilder->SetInsertPoint(remaining_breaks_block);
            Value * break_count = iBuilder->CreatePopcount(phiRecordBreaks);
            Value * final_record_num = iBuilder->CreateAdd(phiRecordNum, break_count);
            Value * reverseZeroes = iBuilder->CreateCountReverseZeroes(phiRecordBreaks);
            Value * pendingLineStart = iBuilder->CreateAdd(phiScanwordPos, iBuilder->CreateSub(width, reverseZeroes));
            iBuilder->CreateBr(return_block);

        // RETURN block
        iBuilder->SetInsertPoint(return_block);
        PHINode * phiFinalRecordNum = iBuilder->CreatePHI(sizeTy, 2, "finalRecordCount");
        PHINode * phiFinalRecordStart = iBuilder->CreatePHI(sizeTy, 2, "finalRecordStart");

        phiFinalRecordNum->addIncoming(phiRecordNum, processMatchesExit);
        phiFinalRecordNum->addIncoming(final_record_num, remaining_breaks_block);
        phiLineNum->addIncoming(phiFinalRecordNum, return_block);

        phiFinalRecordStart->addIncoming(phiRecordStart, processMatchesExit);
        phiFinalRecordStart->addIncoming(pendingLineStart, remaining_breaks_block);
        phiLineStart->addIncoming(phiFinalRecordStart, return_block);

        Value * nextScanwordPos = iBuilder->CreateAdd(phiScanwordPos, ConstantInt::get(sizeTy, sizeTy->getBitWidth()));
        phiScanwordPos->addIncoming(nextScanwordPos, return_block);
        Value * nextIndex = iBuilder->CreateAdd(phiIndex, iBuilder->getInt32(1));
        phiIndex->addIncoming(nextIndex, return_block);
        iBuilder->CreateLikelyCondBr(iBuilder->CreateICmpNE(nextIndex, iBuilder->getInt32(fieldCount)), scanWordIteration, scanWordExit);

    iBuilder->SetInsertPoint(scanWordExit);
    iBuilder->setScalarField("BlockNo", iBuilder->CreateAdd(blockNo, ConstantInt::get(blockNo->getType(), 1)));
    iBuilder->setScalarField("LineNum", phiFinalRecordNum);
    iBuilder->setProcessedItemCount("InputStream", phiFinalRecordStart);
    Value * blockBaseNext = iBuilder->CreateAdd(blockBase, ConstantInt::get(iBuilder->getSizeTy(), 1));
    blockBase->addIncoming(blockBaseNext, scanWordExit);
    iBuilder->CreateLikelyCondBr(iBuilder->CreateICmpNE(blockBaseNext, blocksToDo), initialBlock, blocksExit);

    iBuilder->SetInsertPoint(blocksExit);
    iBuilder->CreateCondBr(mIsFinal, callFinalizeScan, scanReturn);

    iBuilder->SetInsertPoint(callFinalizeScan);
    Value * bufSize = iBuilder->getBufferedSize("InputStream");
    Function * finalizer = m->getFunction("finalize_match_wrapper"); assert (finalizer);
    Value * const buffer_base = iBuilder->getRawInputPointer("InputStream", iBuilder->getInt32(0));
    Value * buffer_end_address = iBuilder->CreateGEP(buffer_base, bufSize);
    iBuilder->CreateCall(finalizer, {accumulator, buffer_end_address});
    iBuilder->CreateBr(scanReturn);

    iBuilder->SetInsertPoint(scanReturn);
    return numOfStrides;
}

ScanMatchKernel::ScanMatchKernel(const std::unique_ptr<kernel::KernelBuilder> & b)
: MultiBlockKernel("scanMatch",
// inputs
{Binding{b->getStreamSetTy(1, 1), "matchResult", FixedRate(), Principal()}
,Binding{b->getStreamSetTy(1, 1), "lineBreak"}
,Binding{b->getStreamSetTy(1, 8), "InputStream", FixedRate(), Deferred()}},
// outputs
{},
// input scalars
{Binding{b->getIntAddrTy(), "accumulator_address"}},
// output scalars
{},
// kernel state
{Binding{b->getSizeTy(), "BlockNo"}
,Binding{b->getSizeTy(), "LineNum"}}) {

}

}
