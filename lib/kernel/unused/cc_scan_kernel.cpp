/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#include <kernel/util/cc_scan_kernel.h>
#include <llvm/IR/Module.h>
#include <kernel/core/kernel_builder.h>

#include "LLVMVersion.h"

using namespace llvm;
using namespace llvm_version;

namespace kernel {

void CCScanKernel::generateDoBlockMethod(BuilderRef iBuilder) {
    auto savePoint = iBuilder->saveIP();
    Function * scanWordFunction = generateScanWordRoutine(iBuilder);
    iBuilder->restoreIP(savePoint);

    const unsigned fieldCount = iBuilder->getBitBlockWidth() / mScanwordBitWidth;
    Type * T = iBuilder->getIntNTy(mScanwordBitWidth);
    VectorType * scanwordVectorType =  llvm_version::getVectorType(T, fieldCount);
    Value * blockNo = iBuilder->getScalarField("BlockNo");
    Value * scanwordPos = iBuilder->CreateMul(blockNo, ConstantInt::get(blockNo->getType(), iBuilder->getBitBlockWidth()));

    std::vector<Value * > matchWordVectors;
    for(unsigned d = 0; d < mStreamNum; d++) {
        Value * matches = iBuilder->loadInputStreamBlock("matchResults", iBuilder->getInt32(d));
        matchWordVectors.push_back(iBuilder->CreateBitCast(matches, scanwordVectorType));
    }

    for(unsigned i = 0; i < fieldCount; ++i) {
        for(unsigned d = 0; d < mStreamNum; d++) {
            Value * matchWord = iBuilder->CreateExtractElement(matchWordVectors[d], ConstantInt::get(T, i));
            iBuilder->CreateCall(scanWordFunction, {matchWord, iBuilder->getInt32(d), scanwordPos});
        }
        scanwordPos = iBuilder->CreateAdd(scanwordPos, ConstantInt::get(T, mScanwordBitWidth));
    }
    iBuilder->setScalarField("BlockNo", iBuilder->CreateAdd(blockNo, iBuilder->getSize(1)));
}

Function * CCScanKernel::generateScanWordRoutine(BuilderRef iBuilder) const {

    IntegerType * T = iBuilder->getIntNTy(mScanwordBitWidth);

    Module * const m = iBuilder->getModule();

    FunctionType * scanTy = FunctionType::get(iBuilder->getVoidTy(), {T, iBuilder->getInt32Ty(), T}, false);
    Function * const scanFunc = Function::Create(scanTy, Function::InternalLinkage, "scan_word", m);

    scanFunc->setCallingConv(CallingConv::C);
    Function::arg_iterator args = scanFunc->arg_begin();

    Value * matchWord = &*(args++);
    matchWord->setName("matchWord");
    Value * dist = &*(args++);
    dist->setName("dist");
    Value * basePos = &*(args++);
    basePos->setName("basePos");

    FunctionType * fTy = FunctionType::get(iBuilder->getVoidTy(), {T, iBuilder->getInt32Ty()}, false);
    Function * const matchProcessor = Function::Create(fTy, Function::ExternalLinkage, "wrapped_report_pos", m);

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

CCScanKernel::CCScanKernel(BuilderRef b, unsigned streamNum)
: BlockOrientedKernel(b, "CCScan",
              {Binding{b->getStreamSetTy(streamNum), "matchResults"}},
              {}, {}, {}, {InternalScalar{b->getSizeTy(), "BlockNo"}}),
mStreamNum(streamNum),
mScanwordBitWidth(b->getSizeTy()->getBitWidth()) {

}

}
