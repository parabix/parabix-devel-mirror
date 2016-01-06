/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#include "scanmatchgen.h"
#include <llvm/IR/Intrinsics.h>

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

/*
        
void write_matches_ending_in_segment(scanword_t matches, scanword_t lbreaks, ssize_t segmentPos, ssize_t & pendingLineStart, int & lineNum) {
    scanword_t remaining_matches = matches;
    scanword_t remaining_LBs = lbreaks;
    while (remaining_matches != 0) {
        // Find all the line marks prior to the match position 
        scanword_t priorLineMarks = ForwardZeroesMask(remaining_matches) & remaining_LBs;
        if (priorLineMarks != 0) {
            int matchLineNum = lineNum + popcount(priorLineMarks);
            // The last of these line marks prior to the match position is the
            // starting positions of the matched line.
            matchLineStart = segmentPos + segmentBitWidth - CountReverseZeroes(priorLineMarks);
        }
        else {
            matchLineNum = lineNum;
            matchLineStart = pendingLineStart;
        }
        // The line end is marked by the match posiiton.
        matchLineEnd = segmentPos + CountForwardZeroes(remaining_matches);
        call write_match(matchLineNum, matchLineStart, matchLineEnd);
        
        remaining_matches = ResetLowestBit(remaining_matches);
        remaining_LBs = remaining_LBs ^ priorLineMarks;
        lineNum = matchLineNum;
    }
    if (remaining_LBs != 0) {
        lineNum += popcount(remaining_LBs);
        pendingLineStart = segmentPos + segmentBitWidth - CountReverseZeroes(remaining_LBs);
    }
}    
        */

        
        
void generateScanSegmentRoutine(Module * m, IDISA::IDISA_Builder * iBuilder, int segBitWidth) {
    LLVMContext & ctxt = m->getContext();
    Type * T = iBuilder->getIntNTy(segBitWidth);
    Type * returnType = StructType::get(ctxt, std::vector<Type *>({T, T}));
    FunctionType * functionType = FunctionType::get(returnType, std::vector<Type *>({T, T, T, T, T}), false);
    Function * sFunction;
        
    SmallVector<AttributeSet, 6> Attrs;
    Attrs.push_back(AttributeSet::get(ctxt, ~0U, std::vector<Attribute::AttrKind>({ Attribute::NoUnwind, Attribute::UWTable })));
    Attrs.push_back(AttributeSet::get(ctxt, 1, std::vector<Attribute::AttrKind>({})));
    Attrs.push_back(AttributeSet::get(ctxt, 2, std::vector<Attribute::AttrKind>({})));
    Attrs.push_back(AttributeSet::get(ctxt, 3, std::vector<Attribute::AttrKind>({})));
    Attrs.push_back(AttributeSet::get(ctxt, 4, std::vector<Attribute::AttrKind>({})));
    Attrs.push_back(AttributeSet::get(ctxt, 5, std::vector<Attribute::AttrKind>({})));
    AttributeSet AttrSet = AttributeSet::get(ctxt, Attrs);
    
    sFunction = Function::Create(functionType, GlobalValue::ExternalLinkage, "scan_matches_in_segment", m);
    sFunction->setCallingConv(CallingConv::C);
    sFunction->setAttributes(AttrSet);
        
    Function::arg_iterator args = sFunction->arg_begin();
    Value * matches_input_parm = args++;
    matches_input_parm->setName("matches");
    Value * record_breaks_input_parm = args++;
    record_breaks_input_parm->setName("breaks");
    Value * segmentPos = args++;
    segmentPos->setName("segmentPos");
    Value * recordStart_input_parm = args++;
    recordStart_input_parm->setName("pendingLineStart");
    Value * recordNum_input_parm = args++;
    recordNum_input_parm->setName("lineNum");
    
    Constant * matchProcessor = m->getOrInsertFunction("wrapped_report_match", Type::getVoidTy(ctxt), T, T, T, NULL);

    
    iBuilder->SetInsertPoint(BasicBlock::Create(ctxt, "entry", sFunction,0));

    BasicBlock * entry_block = iBuilder->GetInsertBlock();
    BasicBlock * matches_test_block = BasicBlock::Create(ctxt, "matches_test_block", sFunction, 0);
    BasicBlock * process_matches_loop_entry = BasicBlock::Create(ctxt, "process_matches_loop", sFunction, 0);
    BasicBlock * prior_breaks_block = BasicBlock::Create(ctxt, "prior_breaks_block", sFunction, 0);
    BasicBlock * loop_final_block = BasicBlock::Create(ctxt, "loop_final_block", sFunction, 0);
    BasicBlock * matches_done_block = BasicBlock::Create(ctxt, "matches_done_block", sFunction, 0);
    BasicBlock * remaining_breaks_block = BasicBlock::Create(ctxt, "remaining_breaks_block", sFunction, 0);
    BasicBlock * return_block = BasicBlock::Create(ctxt, "return_block", sFunction, 0);
        
        
    // The match scanner works with a loop involving four variables:
    // (a) the bit stream segment of matches marking the ends of selected records,
    // (b) the bit stream segment of record_breaks marking the ends of all records,
    // (c) the integer lastRecordNum indicating the number of records processed so far, 
    // (d) the index lastRecordStart indicating the file position of the last record.
    // We set up a loop structure, in which a set of 4 phi nodes initialize these
    // variables from either the input to the scanner or the computed values within
    // the loop body.

    
    iBuilder->CreateBr(matches_test_block);

    // LOOP Test Block 
    iBuilder->SetInsertPoint(matches_test_block);
    PHINode * matches_phi = iBuilder->CreatePHI(T, 2, "matches");
    PHINode * record_breaks_phi = iBuilder->CreatePHI(T, 2, "record_breaks");
    PHINode * recordNum_phi = iBuilder->CreatePHI(T, 2, "recordNum");
    PHINode * recordStart_phi = iBuilder->CreatePHI(T, 2, "recordStart");
    matches_phi->addIncoming(matches_input_parm, entry_block);
    record_breaks_phi->addIncoming(record_breaks_input_parm, entry_block);
    recordNum_phi->addIncoming(recordNum_input_parm, entry_block);
    recordStart_phi->addIncoming(recordStart_input_parm, entry_block);
    Value * have_matches_cond = iBuilder->CreateICmpNE(matches_phi, ConstantInt::get(T, 0));
    iBuilder->CreateCondBr(have_matches_cond, process_matches_loop_entry, matches_done_block);
    
    // LOOP BODY
    // The loop body is entered if we have more matches to process.
    iBuilder->SetInsertPoint(process_matches_loop_entry);
    Value * prior_breaks = iBuilder->CreateAnd(generateForwardZeroesMask(iBuilder, matches_phi), record_breaks_phi);
    // Within the loop we have a conditional block that is executed if there are any prior 
    // record breaks.
    Value * prior_breaks_cond = iBuilder->CreateICmpNE(prior_breaks, ConstantInt::get(T, 0));
    iBuilder->CreateCondBr(prior_breaks_cond, prior_breaks_block, loop_final_block);

    // PRIOR_BREAKS_BLOCK
    // If there are prior breaks, we count them and compute the record start position.
    iBuilder->SetInsertPoint(prior_breaks_block);
    Value * matchRecordNum = iBuilder->CreateAdd(generatePopcount(iBuilder, prior_breaks), recordNum_phi);
    Value * reverseDistance = generateCountReverseZeroes(iBuilder, prior_breaks);
    Value * width = ConstantInt::get(T, segBitWidth);
    Value * matchRecordStart = iBuilder->CreateAdd(segmentPos, iBuilder->CreateSub(width, reverseDistance));
    iBuilder->CreateBr(loop_final_block);
    
    // LOOP FINAL BLOCK
    // The prior breaks, if any have been counted.  Set up phi nodes for the recordNum
    // and recortStart depending on whether the conditional execution of prior_breaks_block.
    iBuilder->SetInsertPoint(loop_final_block);
    PHINode * matchRecordNum_phi = iBuilder->CreatePHI(T, 2, "matchRecordNum");
    PHINode * matchRecordStart_phi = iBuilder->CreatePHI(T, 2, "matchRecordStart");
    matchRecordNum_phi->addIncoming(recordNum_phi, process_matches_loop_entry);
    matchRecordNum_phi->addIncoming(matchRecordNum, prior_breaks_block);
    matchRecordStart_phi->addIncoming(recordStart_phi, process_matches_loop_entry);
    matchRecordStart_phi->addIncoming(matchRecordStart, prior_breaks_block);
    Value * matchRecordEnd = iBuilder->CreateAdd(segmentPos, generateCountForwardZeroes(iBuilder, matches_phi));
    iBuilder->CreateCall(matchProcessor, std::vector<Value *>({matchRecordNum_phi, matchRecordStart_phi, matchRecordEnd}));
    Value * remaining_matches = generateResetLowestBit(iBuilder, matches_phi);
    Value * remaining_breaks = iBuilder->CreateXor(record_breaks_phi, prior_breaks);
    matches_phi->addIncoming(remaining_matches, loop_final_block);
    record_breaks_phi->addIncoming(remaining_breaks, loop_final_block);
    recordNum_phi->addIncoming(matchRecordNum_phi, loop_final_block);
    recordStart_phi->addIncoming(matchRecordStart_phi, loop_final_block);
    iBuilder->CreateBr(matches_test_block);
    
    
    // LOOP EXIT/MATCHES_DONE
    iBuilder->SetInsertPoint(matches_done_block);
    // When the matches are done, there may be additional record breaks remaining
    Value * more_breaks_cond = iBuilder->CreateICmpNE(record_breaks_phi, ConstantInt::get(T, 0));
    iBuilder->CreateCondBr(more_breaks_cond, remaining_breaks_block, return_block);
    
    // REMAINING_BREAKS_BLOCK: process remaining record breaks after all matches are processed
    iBuilder->SetInsertPoint(remaining_breaks_block);
    Value * break_count = generatePopcount(iBuilder, record_breaks_phi);
    Value * final_record_num = iBuilder->CreateAdd(recordNum_phi, break_count);
    Value * reverseZeroes = generateCountReverseZeroes(iBuilder, record_breaks_phi);
    Value * pendingLineStart = iBuilder->CreateAdd(segmentPos, iBuilder->CreateSub(width, reverseZeroes));
    iBuilder->CreateBr(return_block);
    
    // RETURN block
    iBuilder->SetInsertPoint(return_block);
    PHINode * finalRecordCount_phi = iBuilder->CreatePHI(T, 2, "finalRecordCount");
    PHINode * finalRecordStart_phi = iBuilder->CreatePHI(T, 2, "finalRecordStart");
    finalRecordCount_phi->addIncoming(recordNum_phi, matches_done_block);
    finalRecordCount_phi->addIncoming(final_record_num, remaining_breaks_block);
    finalRecordStart_phi->addIncoming(recordStart_phi, matches_done_block);
    finalRecordStart_phi->addIncoming(pendingLineStart, remaining_breaks_block);
    Value * retVal = UndefValue::get(returnType);
    retVal = iBuilder->CreateInsertValue(retVal, finalRecordCount_phi, 0);
    retVal = iBuilder->CreateInsertValue(retVal, finalRecordStart_phi, 1);
    iBuilder->CreateRet(retVal);
    
}


void generateScanBitBlockRoutine(Module * m, IDISA::IDISA_Builder * iBuilder, int segBitWidth) {
    LLVMContext & ctxt = m->getContext();
    Type * B = iBuilder->getBitBlockType();
    Type * T = iBuilder->getIntNTy(segBitWidth);
    generateScanSegmentRoutine(m, iBuilder, segBitWidth);
    int fieldCount = iBuilder->getBitBlockWidth()/segBitWidth;
    Type * segmentVectorType =  VectorType::get(T, fieldCount);
    
    
    Type * returnType = StructType::get(ctxt, std::vector<Type *>({T, T}));
    FunctionType * functionType = FunctionType::get(returnType, std::vector<Type *>({B, B, T, T, T}), false);
    Function * sFunction;
    
    SmallVector<AttributeSet, 6> Attrs;
    Attrs.push_back(AttributeSet::get(ctxt, ~0U, std::vector<Attribute::AttrKind>({ Attribute::NoUnwind, Attribute::UWTable })));
    Attrs.push_back(AttributeSet::get(ctxt, 1, std::vector<Attribute::AttrKind>({})));
    Attrs.push_back(AttributeSet::get(ctxt, 2, std::vector<Attribute::AttrKind>({})));
    Attrs.push_back(AttributeSet::get(ctxt, 3, std::vector<Attribute::AttrKind>({})));
    Attrs.push_back(AttributeSet::get(ctxt, 4, std::vector<Attribute::AttrKind>({})));
    Attrs.push_back(AttributeSet::get(ctxt, 5, std::vector<Attribute::AttrKind>({})));
    AttributeSet AttrSet = AttributeSet::get(ctxt, Attrs);
    sFunction = Function::Create(functionType, GlobalValue::ExternalLinkage, "scan_matches_in_bitblock", m);
    sFunction->setCallingConv(CallingConv::C);
    sFunction->setAttributes(AttrSet);
    
    
    
    Function::arg_iterator args = sFunction->arg_begin();
    Value * matches_input_parm = args++;
    matches_input_parm->setName("matches");
    Value * record_breaks_input_parm = args++;
    record_breaks_input_parm->setName("breaks");
    Value * blockPos = args++;
    blockPos->setName("blockPos");
    Value * recordStart_input_parm = args++;
    recordStart_input_parm->setName("pendingLineStart");
    Value * recordNum_input_parm = args++;
    recordNum_input_parm->setName("lineNum");
    
    iBuilder->SetInsertPoint(BasicBlock::Create(ctxt, "entry", sFunction,0));
    
    Value * matchSegVector = iBuilder->CreateBitCast(matches_input_parm, segmentVectorType);
    Value * breakSegVector = iBuilder->CreateBitCast(record_breaks_input_parm, segmentVectorType);
    Value * segmentPos = blockPos;
    Value * recordStart = recordStart_input_parm;
    Value * recordNum = recordNum_input_parm;
    Value * segResult = nullptr;
    Function * segScanFcn = m->getFunction("scan_matches_in_segment");
    for (uint64_t i = 0; i < iBuilder->getBitBlockWidth()/segBitWidth; i++) {
        Value * matchSeg = iBuilder->CreateExtractElement(matchSegVector, ConstantInt::get(T, i));
        Value * recordBreaksSeg = iBuilder->CreateExtractElement(breakSegVector, ConstantInt::get(T, i));
        segResult = iBuilder->CreateCall(segScanFcn, std::vector<Value *>({matchSeg, recordBreaksSeg, segmentPos, recordStart, recordNum}));
        segmentPos = iBuilder->CreateAdd(segmentPos, ConstantInt::get(T, segBitWidth));
        recordStart = iBuilder->CreateExtractValue(segResult, std::vector<unsigned>({0}));
        recordNum = iBuilder->CreateExtractValue(segResult, std::vector<unsigned>({1}));
    }
    iBuilder->CreateRet(segResult);
}




