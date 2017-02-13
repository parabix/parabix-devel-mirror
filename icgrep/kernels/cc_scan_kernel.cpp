/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#include "cc_scan_kernel.h"
#include <IR_Gen/idisa_builder.h>
#include <llvm/IR/Module.h>

using namespace llvm;

namespace kernel {

void CCScanKernel::generateDoBlockMethod() {
    auto savePoint = iBuilder->saveIP();
    Function * scanWordFunction = generateScanWordRoutine(iBuilder->getModule());
    iBuilder->restoreIP(savePoint);

    const unsigned fieldCount = iBuilder->getBitBlockWidth() / mScanwordBitWidth;
    Type * T = iBuilder->getIntNTy(mScanwordBitWidth);
    VectorType * scanwordVectorType =  VectorType::get(T, fieldCount);    
    Value * blockNo = getScalarField("BlockNo");
    Value * scanwordPos = iBuilder->CreateMul(blockNo, ConstantInt::get(blockNo->getType(), iBuilder->getBitBlockWidth()));
    
    std::vector<Value * > matchWordVectors;
    for(unsigned d = 0; d < mStreamNum; d++) {
        Value * ptr = getInputStream("matchResults", iBuilder->getInt32(d));
        Value * matches = iBuilder->CreateBlockAlignedLoad(ptr);
        matchWordVectors.push_back(iBuilder->CreateBitCast(matches, scanwordVectorType));
    }
    
    for(unsigned i = 0; i < fieldCount; ++i) {
        for(unsigned d = 0; d < mStreamNum; d++) {
            Value * matchWord = iBuilder->CreateExtractElement(matchWordVectors[d], ConstantInt::get(T, i));
            iBuilder->CreateCall(scanWordFunction, {matchWord, iBuilder->getInt32(d), scanwordPos});
        }
        scanwordPos = iBuilder->CreateAdd(scanwordPos, ConstantInt::get(T, mScanwordBitWidth));
    }   
    setScalarField("BlockNo", iBuilder->CreateAdd(blockNo, iBuilder->getSize(1)));
}

Function * CCScanKernel::generateScanWordRoutine(Module * m) const {

    IntegerType * T = iBuilder->getIntNTy(mScanwordBitWidth);

    Function * scanFunc = cast<Function>(m->getOrInsertFunction("scan_word", iBuilder->getVoidTy(), T, iBuilder->getInt32Ty(), T, nullptr));
    scanFunc->setCallingConv(CallingConv::C);
    Function::arg_iterator args = scanFunc->arg_begin();

    Value * matchWord = &*(args++);
    matchWord->setName("matchWord");
    Value * dist = &*(args++);
    dist->setName("dist");
    Value * basePos = &*(args++);
    basePos->setName("basePos");

    Constant * matchProcessor = m->getOrInsertFunction("wrapped_report_pos", iBuilder->getVoidTy(), T, iBuilder->getInt32Ty(), nullptr);

    BasicBlock * entryBlock = BasicBlock::Create(m->getContext(), "entry", scanFunc, 0);

    BasicBlock * matchesCondBlock = BasicBlock::Create(m->getContext(), "matchesCond", scanFunc, 0);
    BasicBlock * matchesLoopBlock = BasicBlock::Create(m->getContext(), "matchesLoop", scanFunc, 0);
    BasicBlock * matchesDoneBlock = BasicBlock::Create(m->getContext(), "matchesDone", scanFunc, 0);

    iBuilder->SetInsertPoint(entryBlock);
    iBuilder->CreateBr(matchesCondBlock);

    iBuilder->SetInsertPoint(matchesCondBlock);
    PHINode * matches_phi = iBuilder->CreatePHI(T, 2, "matches");
    matches_phi->addIncoming(matchWord, entryBlock);
    Value * have_matches_cond = iBuilder->CreateICmpUGT(matches_phi, ConstantInt::get(T, 0));
    iBuilder->CreateCondBr(have_matches_cond, matchesLoopBlock, matchesDoneBlock);

    iBuilder->SetInsertPoint(matchesLoopBlock);

    Value * cttzFunc = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::cttz, matches_phi->getType());
    Value * tz = iBuilder->CreateCall(cttzFunc, std::vector<Value *>({matches_phi, ConstantInt::get(iBuilder->getInt1Ty(), 0)}));

    Value * match_pos = iBuilder->CreateAdd(tz, basePos);
    Value * matches_new = iBuilder->CreateAnd(matches_phi, iBuilder->CreateSub(matches_phi, ConstantInt::get(T, 1)));
    matches_phi->addIncoming(matches_new, matchesLoopBlock);
    iBuilder->CreateCall(matchProcessor, std::vector<Value *>({match_pos, dist}));
    iBuilder->CreateBr(matchesCondBlock);

    iBuilder->SetInsertPoint(matchesDoneBlock);
    iBuilder -> CreateRetVoid();

    return scanFunc;

}

CCScanKernel::CCScanKernel(IDISA::IDISA_Builder * iBuilder, unsigned streamNum) :
BlockOrientedKernel(iBuilder, "CCScan",
              {Binding{iBuilder->getStreamSetTy(streamNum), "matchResults"}},
              {}, {}, {}, {Binding{iBuilder->getSizeTy(), "BlockNo"}}),
mStreamNum(streamNum),
mScanwordBitWidth(iBuilder->getSizeTy()->getBitWidth()) {

}

}
