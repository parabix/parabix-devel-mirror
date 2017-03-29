/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "scanmatchgen.h"
#include <llvm/IR/Intrinsics.h>
#include <IR_Gen/idisa_builder.h>
#include <llvm/IR/Module.h>

using namespace llvm;

namespace kernel {

Value * generateForwardZeroesMask(IDISA::IDISA_Builder * iBuilder, Value * bits) {
    Value * bits_minus1 = iBuilder->CreateSub(bits, ConstantInt::get(bits->getType(), 1));
    return iBuilder->CreateAnd(bits_minus1, iBuilder->CreateNot(bits));
}

Value * generatePopcount(IDISA::IDISA_Builder * iBuilder, Value * bits) {
    Value * ctpopFunc = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::ctpop, bits->getType());
    return iBuilder->CreateCall(ctpopFunc, std::vector<Value *>({bits}));
}

Value * generateCountForwardZeroes(IDISA::IDISA_Builder * iBuilder, Value * bits) {
    Value * cttzFunc = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::cttz, bits->getType());
    return iBuilder->CreateCall(cttzFunc, std::vector<Value *>({bits, ConstantInt::get(iBuilder->getInt1Ty(), 0)}));
}

Value * generateCountReverseZeroes(IDISA::IDISA_Builder * iBuilder, Value * bits) {
    Value * ctlzFunc = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::ctlz, bits->getType());
    return iBuilder->CreateCall(ctlzFunc, std::vector<Value *>({bits, ConstantInt::get(iBuilder->getInt1Ty(), 0)}));
}

Value * generateResetLowestBit(IDISA::IDISA_Builder * iBuilder, Value * bits) {
    Value * bits_minus1 = iBuilder->CreateSub(bits, ConstantInt::get(bits->getType(), 1));
    return iBuilder->CreateAnd(bits_minus1, bits);
}
        


void ScanMatchKernel::generateDoBlockMethod() {

    Module * const m = iBuilder->getModule();
    BasicBlock * const entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * const scanWordIteration = CreateBasicBlock("ScanWordIteration");
    BasicBlock * const matches_test_block = CreateBasicBlock("matches_test_block");
    BasicBlock * const processMatchesEntry = CreateBasicBlock("process_matches_loop");
    BasicBlock * const prior_breaks_block = CreateBasicBlock("prior_breaks_block");
    BasicBlock * const loop_final_block = CreateBasicBlock("loop_final_block");
    BasicBlock * const processMatchesExit = CreateBasicBlock("matches_done_block");
    BasicBlock * const remaining_breaks_block = CreateBasicBlock("remaining_breaks_block");
    BasicBlock * const return_block = CreateBasicBlock("return_block");
    BasicBlock * const scanWordExit = CreateBasicBlock("ScanWordExit");
    IntegerType * const sizeTy = iBuilder->getSizeTy();
    PointerType * const codeUnitTy = iBuilder->getIntNTy(mCodeUnitWidth)->getPointerTo();
    const unsigned fieldCount = iBuilder->getBitBlockWidth() / sizeTy->getBitWidth();
    VectorType * const scanwordVectorType =  VectorType::get(sizeTy, fieldCount);
    Value * const blockNo = getScalarField("BlockNo");
    Value * const scanwordPos = iBuilder->CreateMul(blockNo, ConstantInt::get(blockNo->getType(), iBuilder->getBitBlockWidth()));
    Value * const lastRecordStart = getScalarField("LineStart");
    Value * const lastRecordNum = getScalarField("LineNum");
    Value * const inputStream = iBuilder->CreatePointerCast(getRawInputPointer("InputStream", iBuilder->getInt32(0), iBuilder->getInt32(0)), codeUnitTy);

    Value * fileSize = iBuilder->CreateAdd(getProcessedItemCount("InputStream"), getScalarField("PendingBytes"));

    Constant * matchProcessor = nullptr;
    Value * fileIdx = nullptr;
    switch (mGrepType) {
        case GrepType::Normal:
            fileIdx = getScalarField("FileIdx");
            matchProcessor = m->getOrInsertFunction("wrapped_report_match" + std::to_string(mCodeUnitWidth), iBuilder->getVoidTy(), sizeTy, sizeTy, sizeTy, codeUnitTy, sizeTy, sizeTy, nullptr);
            break;
        case GrepType::NameExpression:
            matchProcessor = m->getOrInsertFunction("insert_codepoints", iBuilder->getVoidTy(), sizeTy, sizeTy, sizeTy, codeUnitTy, nullptr);
            break;
        case GrepType::PropertyValue:
            matchProcessor = m->getOrInsertFunction("insert_property_values", iBuilder->getVoidTy(), sizeTy, sizeTy, sizeTy, codeUnitTy, nullptr);
            break;
        default: llvm_unreachable("unknown grep type");
    }
    Value * const matchesPtr = getInputStreamBlockPtr("matchResult", iBuilder->getInt32(0));
    Value * const matches = iBuilder->CreateBitCast(iBuilder->CreateBlockAlignedLoad(matchesPtr), scanwordVectorType);

    Value * const linebreaksPtr = getInputStreamBlockPtr("lineBreak", iBuilder->getInt32(0));
    Value * const linebreaks = iBuilder->CreateBitCast(iBuilder->CreateBlockAlignedLoad(linebreaksPtr), scanwordVectorType);

    iBuilder->CreateBr(scanWordIteration);

    iBuilder->SetInsertPoint(scanWordIteration);

        // while (phiIndex < words per stride)
        PHINode * const phiIndex = iBuilder->CreatePHI(iBuilder->getInt32Ty(), 2, "index");
        phiIndex->addIncoming(iBuilder->getInt32(0), entryBlock);
        PHINode * const phiScanwordPos = iBuilder->CreatePHI(scanwordPos->getType(), 2, "pos");
        phiScanwordPos->addIncoming(scanwordPos, entryBlock);
        PHINode * const phiLineStart = iBuilder->CreatePHI(lastRecordStart->getType(), 2, "recordstart");
        phiLineStart->addIncoming(lastRecordStart, entryBlock);
        PHINode * const phiLineNum = iBuilder->CreatePHI(lastRecordNum->getType(), 2, "recordnum");
        phiLineNum->addIncoming(lastRecordNum, entryBlock);
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
        Value * anyMatches = iBuilder->CreateICmpNE(phiMatchWord, ConstantInt::getNullValue(sizeTy));
        iBuilder->CreateCondBr(anyMatches, processMatchesEntry, processMatchesExit);

            // LOOP BODY
            // The loop body is entered if we have more matches to process.
            iBuilder->SetInsertPoint(processMatchesEntry);
            Value * prior_breaks = iBuilder->CreateAnd(generateForwardZeroesMask(iBuilder, phiMatchWord), phiRecordBreaks);
            // Within the loop we have a conditional block that is executed if there are any prior record breaks.
            Value * prior_breaks_cond = iBuilder->CreateICmpNE(prior_breaks, ConstantInt::getNullValue(sizeTy));
            iBuilder->CreateCondBr(prior_breaks_cond, prior_breaks_block, loop_final_block);

                // PRIOR_BREAKS_BLOCK
                // If there are prior breaks, we count them and compute the record start position.
                iBuilder->SetInsertPoint(prior_breaks_block);
                Value * matchedRecordNum = iBuilder->CreateAdd(generatePopcount(iBuilder, prior_breaks), phiRecordNum);
                Value * reverseDistance = generateCountReverseZeroes(iBuilder, prior_breaks);
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

            PHINode * matchRecordStart = iBuilder->CreatePHI(sizeTy, 2, "matchRecordStart");
            matchRecordStart->addIncoming(phiRecordStart, processMatchesEntry);
            matchRecordStart->addIncoming(priorRecordStart, prior_breaks_block);
            phiRecordStart->addIncoming(matchRecordStart, loop_final_block);

            Value * matchRecordEnd = iBuilder->CreateAdd(phiScanwordPos, generateCountForwardZeroes(iBuilder, phiMatchWord));
            switch (mGrepType) {
                case GrepType::Normal:
                    iBuilder->CreateCall(matchProcessor, {matchRecordNum, matchRecordStart, matchRecordEnd, inputStream, fileSize, fileIdx});
                    break;
                case GrepType::NameExpression:
                case GrepType::PropertyValue:
                    iBuilder->CreateCall(matchProcessor, {matchRecordNum, matchRecordStart, matchRecordEnd, inputStream});
                    break;
                default: break;
            }

            Value * remaining_matches = generateResetLowestBit(iBuilder, phiMatchWord);
            phiMatchWord->addIncoming(remaining_matches, loop_final_block);

            Value * remaining_breaks = iBuilder->CreateXor(phiRecordBreaks, prior_breaks);
            phiRecordBreaks->addIncoming(remaining_breaks, loop_final_block);

            iBuilder->CreateBr(matches_test_block);

        // LOOP EXIT/MATCHES_DONE
        iBuilder->SetInsertPoint(processMatchesExit);
        // When the matches are done, there may be additional record breaks remaining
        Value * more_breaks_cond = iBuilder->CreateICmpNE(phiRecordBreaks, ConstantInt::getNullValue(sizeTy));
        iBuilder->CreateCondBr(more_breaks_cond, remaining_breaks_block, return_block);

            // REMAINING_BREAKS_BLOCK: process remaining record breaks after all matches are processed
            iBuilder->SetInsertPoint(remaining_breaks_block);
            Value * break_count = generatePopcount(iBuilder, phiRecordBreaks);
            Value * final_record_num = iBuilder->CreateAdd(phiRecordNum, break_count);
            Value * reverseZeroes = generateCountReverseZeroes(iBuilder, phiRecordBreaks);
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
    setScalarField("BlockNo", iBuilder->CreateAdd(blockNo, ConstantInt::get(blockNo->getType(), 1)));
    setScalarField("LineStart", phiFinalRecordStart);
    setScalarField("LineNum", phiFinalRecordNum);
}

void ScanMatchKernel::generateInitMethod() {
    setScalarField("PendingBytes", iBuilder->getSize(iBuilder->getBitBlockWidth() + 2));
}

void ScanMatchKernel::generateFinalBlockMethod(llvm::Value * remainingItems) {
    setScalarField("PendingBytes", remainingItems);
    CreateDoBlockMethodCall();
}

ScanMatchKernel::ScanMatchKernel(IDISA::IDISA_Builder * iBuilder, GrepType grepType, const unsigned codeUnitWidth)
: BlockOrientedKernel(iBuilder, "scanMatch" + std::to_string(codeUnitWidth),
    {Binding{iBuilder->getStreamSetTy(1, 8), "InputStream"}, Binding{iBuilder->getStreamSetTy(1, 1), "matchResult"}, Binding{iBuilder->getStreamSetTy(1, 1), "lineBreak"}},
    {},
    {Binding{iBuilder->getSizeTy(), "FileIdx"}},
    {},
    {Binding{iBuilder->getSizeTy(), "BlockNo"}, Binding{iBuilder->getSizeTy(), "LineStart"}, Binding{iBuilder->getSizeTy(), "LineNum"}, Binding{iBuilder->getSizeTy(), "PendingBytes"}})
, mGrepType(grepType)
, mCodeUnitWidth(codeUnitWidth) {
}

}
