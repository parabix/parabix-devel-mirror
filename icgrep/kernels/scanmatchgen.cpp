/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include "scanmatchgen.h"
#include <llvm/IR/Intrinsics.h>
#include <IDISA/idisa_builder.h>
#include <llvm/Support/raw_os_ostream.h>

using namespace llvm;

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

Function * generateScanWordRoutine(Module * m, IDISA::IDISA_Builder * iBuilder, unsigned scanwordBitWidth, KernelBuilder * const kBuilder, bool isNameExpression) {

    Function * function = m->getFunction("scan_matches_in_scanword");
    if (LLVM_UNLIKELY(function != nullptr)) {
        return function;
    }

    LLVMContext & ctxt = m->getContext();
    Type * T = iBuilder->getIntNTy(scanwordBitWidth);
    Type * S = PointerType::get(iBuilder->getIntNTy(8), 0);
    Type * returnType = StructType::get(ctxt, std::vector<Type *>({T, T}));
    FunctionType * functionType = FunctionType::get(returnType, std::vector<Type *>({PointerType::get(kBuilder->getKernelStructType(), 0), T, T, T, T, T}), false);

    SmallVector<AttributeSet, 6> Attrs;
    Attrs.push_back(AttributeSet::get(ctxt, ~0U, std::vector<Attribute::AttrKind>({ Attribute::NoUnwind, Attribute::UWTable })));
    Attrs.push_back(AttributeSet::get(ctxt, 1, std::vector<Attribute::AttrKind>({})));
    Attrs.push_back(AttributeSet::get(ctxt, 2, std::vector<Attribute::AttrKind>({})));
    Attrs.push_back(AttributeSet::get(ctxt, 3, std::vector<Attribute::AttrKind>({})));
    Attrs.push_back(AttributeSet::get(ctxt, 4, std::vector<Attribute::AttrKind>({})));
    Attrs.push_back(AttributeSet::get(ctxt, 5, std::vector<Attribute::AttrKind>({})));
    AttributeSet AttrSet = AttributeSet::get(ctxt, Attrs);

    function = Function::Create(functionType, GlobalValue::ExternalLinkage, "scan_matches_in_scanword", m);
    function->setCallingConv(CallingConv::C);
    function->setAttributes(AttrSet);
    function->addFnAttr(llvm::Attribute::AlwaysInline);

    Function::arg_iterator args = function->arg_begin();
    Value * this_input_parm = args++;
    this_input_parm->setName("this");
    Value * matches_input_parm = args++;
    matches_input_parm->setName("matches");
    Value * record_breaks_input_parm = args++;
    record_breaks_input_parm->setName("breaks");
    Value * scanwordPos = args++;
    scanwordPos->setName("scanwordPos");
    Value * recordStart_input_parm = args++;
    recordStart_input_parm->setName("pendingLineStart");
    Value * recordNum_input_parm = args++;
    recordNum_input_parm->setName("lineNum");

    Constant * matchProcessor;
    if (isNameExpression) {
        matchProcessor = m->getOrInsertFunction("insert_codepoints", Type::getVoidTy(ctxt), T, T, T, S, nullptr);
    } else {
        matchProcessor = m->getOrInsertFunction("wrapped_report_match", Type::getVoidTy(ctxt), T, T, T, S, T, S, nullptr);
    }
    iBuilder->SetInsertPoint(BasicBlock::Create(ctxt, "entry", function,0));

    BasicBlock * entry_block = iBuilder->GetInsertBlock();
    BasicBlock * matches_test_block = BasicBlock::Create(ctxt, "matches_test_block", function, 0);
    BasicBlock * process_matches_loop_entry = BasicBlock::Create(ctxt, "process_matches_loop", function, 0);
    BasicBlock * prior_breaks_block = BasicBlock::Create(ctxt, "prior_breaks_block", function, 0);
    BasicBlock * loop_final_block = BasicBlock::Create(ctxt, "loop_final_block", function, 0);
    BasicBlock * matches_done_block = BasicBlock::Create(ctxt, "matches_done_block", function, 0);
    BasicBlock * remaining_breaks_block = BasicBlock::Create(ctxt, "remaining_breaks_block", function, 0);
    BasicBlock * return_block = BasicBlock::Create(ctxt, "return_block", function, 0);


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
    Value * width = ConstantInt::get(T, scanwordBitWidth);
    Value * matchRecordStart = iBuilder->CreateAdd(scanwordPos, iBuilder->CreateSub(width, reverseDistance));
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
    Value * matchRecordEnd = iBuilder->CreateAdd(scanwordPos, generateCountForwardZeroes(iBuilder, matches_phi));

    Value* filebuf_gep = kBuilder->getInternalState("FileBuf", this_input_parm);
    Value* filebufptr = iBuilder->CreateLoad(filebuf_gep, "filebuf");

    if (isNameExpression) {
        iBuilder->CreateCall(matchProcessor, std::vector<Value *>({matchRecordNum_phi, matchRecordStart_phi, matchRecordEnd, filebufptr}));
    } else {
        Value * filesize_gep = kBuilder->getInternalState("FileSize", this_input_parm);
        Value * filesize = iBuilder->CreateLoad(filesize_gep, "filesize");

        Value * filename_gep = kBuilder->getInternalState("FileName", this_input_parm);
        Value * filenameptr = iBuilder->CreateLoad(filename_gep, "filename");

        iBuilder->CreateCall(matchProcessor, std::vector<Value *>({matchRecordNum_phi, matchRecordStart_phi, matchRecordEnd, filebufptr, filesize, filenameptr}));
    }

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
    Value * pendingLineStart = iBuilder->CreateAdd(scanwordPos, iBuilder->CreateSub(width, reverseZeroes));
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
    retVal = iBuilder->CreateInsertValue(retVal, finalRecordStart_phi, 0);
    retVal = iBuilder->CreateInsertValue(retVal, finalRecordCount_phi, 1);
    iBuilder->CreateRet(retVal);

    return function;
}


void generateScanMatch(Module * m, IDISA::IDISA_Builder * iBuilder, unsigned scanWordBitWidth, KernelBuilder * kBuilder, bool isNameExpression) {


    Type * T = iBuilder->getIntNTy(scanWordBitWidth);
    Type * S = PointerType::get(iBuilder->getIntNTy(8), 0);
    const unsigned fieldCount = iBuilder->getBitBlockWidth() / scanWordBitWidth;
    Type * scanwordVectorType =  VectorType::get(T, fieldCount);

    kBuilder->addInputStream(1, "matches");
    kBuilder->addInputStream(1, "breaks");
    //use index
    const unsigned lineStart = kBuilder->addInternalState(T, "LineStart");
    const unsigned lineNum = kBuilder->addInternalState(T, "LineNum");
    kBuilder->addInternalState(S, "FileBuf");
    kBuilder->addInternalState(T, "FileSize");
    kBuilder->addInternalState(S, "FileName");

    Function * function = kBuilder->prepareFunction();

    // Type * kernelStuctType = PointerType::get(kBuilder->getKernelStructType(), 0);

    Function * scanWordFunction = generateScanWordRoutine(m, iBuilder, scanWordBitWidth, kBuilder, isNameExpression);

    iBuilder->SetInsertPoint(&function->getEntryBlock());

    Value * kernelStuctParam = kBuilder->getKernelStructParam();

    Value * scanwordPos = iBuilder->CreateBlockAlignedLoad(kBuilder->getInternalState("BlockNo"));
    scanwordPos = iBuilder->CreateMul(scanwordPos, ConstantInt::get(scanwordPos->getType(), iBuilder->getBitBlockWidth()));

    Value * recordStart = iBuilder->CreateBlockAlignedLoad(kBuilder->getInternalState(lineStart));
    Value * recordNum = iBuilder->CreateBlockAlignedLoad(kBuilder->getInternalState(lineNum));

    Value * wordResult = nullptr;

    const unsigned segmentBlocks = kBuilder->getSegmentBlocks();
    const unsigned scanWordBlocks =  segmentBlocks * fieldCount;
    for(unsigned j = 0; j < segmentBlocks; ++j) {
        Value * matchWordVector = iBuilder->CreateBitCast(iBuilder->CreateBlockAlignedLoad(kBuilder->getInputStream(0)), scanwordVectorType);
        Value * breakWordVector = iBuilder->CreateBitCast(iBuilder->CreateBlockAlignedLoad(kBuilder->getInputStream(1)), scanwordVectorType);
        for(unsigned i = 0; i < scanWordBlocks; ++i){
            Value * matchWord = iBuilder->CreateExtractElement(matchWordVector, ConstantInt::get(T, i));
            Value * recordBreaksWord = iBuilder->CreateExtractElement(breakWordVector, ConstantInt::get(T, i));
            wordResult = iBuilder->CreateCall(scanWordFunction, std::vector<Value *>({kernelStuctParam, matchWord, recordBreaksWord, scanwordPos, recordStart, recordNum}));
            scanwordPos = iBuilder->CreateAdd(scanwordPos, ConstantInt::get(T, scanWordBitWidth));
            recordStart = iBuilder->CreateExtractValue(wordResult, std::vector<unsigned>({0}));
            recordNum = iBuilder->CreateExtractValue(wordResult, std::vector<unsigned>({1}));
        }
        kBuilder->increment();
    }
    kBuilder->setInternalState(lineStart, recordStart);
    kBuilder->setInternalState(lineNum, recordNum);
    kBuilder->finalize();
}
