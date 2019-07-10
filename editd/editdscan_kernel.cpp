/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#include "editdscan_kernel.h"
#include <kernels/kernel_builder.h>
#include <llvm/IR/Module.h>

using namespace llvm;

namespace kernel {

void editdScanKernel::generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & b) {
    auto savePoint = b->saveIP();
    Function * scanWordFunction = generateScanWordRoutine(b);
    b->restoreIP(savePoint);

    const unsigned fieldCount = b->getBitBlockWidth() / mScanwordBitWidth;
    Type * T = b->getIntNTy(mScanwordBitWidth);
    VectorType * scanwordVectorType =  VectorType::get(T, fieldCount);
    Value * blockNo = b->getScalarField("BlockNo");
    Value * scanwordPos = b->CreateMul(blockNo, ConstantInt::get(blockNo->getType(), b->getBitBlockWidth()));

    std::vector<Value * > matchWordVectors;
    for(unsigned d = 0; d < mNumElements; d++) {
        Value * matches = b->loadInputStreamBlock("matchResults", b->getInt32(d));
        matchWordVectors.push_back(b->CreateBitCast(matches, scanwordVectorType));
    }

    for(unsigned i = 0; i < fieldCount; ++i) {
        for(unsigned d = 0; d < mNumElements; d++) {
            Value * matchWord = b->CreateExtractElement(matchWordVectors[d], ConstantInt::get(T, i));
            b->CreateCall(scanWordFunction, {matchWord, b->getInt32(d), scanwordPos});
        }
        scanwordPos = b->CreateAdd(scanwordPos, ConstantInt::get(T, mScanwordBitWidth));
    }

    b->setScalarField("BlockNo", b->CreateAdd(blockNo, b->getSize(1)));
}

Function * editdScanKernel::generateScanWordRoutine(const std::unique_ptr<KernelBuilder> &b) const {

    IntegerType * T = b->getIntNTy(mScanwordBitWidth);
    Module * const m = b->getModule();

    FunctionType * scanTy = FunctionType::get(b->getVoidTy(), {T, b->getInt32Ty(), T}, false);
    Function * const scanFunc = Function::Create(scanTy, Function::InternalLinkage, "scan_word", m);
    scanFunc->setCallingConv(CallingConv::C);
    Function::arg_iterator args = scanFunc->arg_begin();

    Value * matchWord = &*(args++);
    matchWord->setName("matchWord");
    Value * dist = &*(args++);
    dist->setName("dist");
    Value * basePos = &*(args++);
    basePos->setName("basePos");

    FunctionType * fTy = FunctionType::get(b->getVoidTy(), {T, b->getInt32Ty()}, false);
    Function * const matchProcessor = Function::Create(fTy, Function::InternalLinkage, "wrapped_report_pos", m);

    BasicBlock * entryBlock = BasicBlock::Create(b->getContext(), "entry", scanFunc, 0);
    BasicBlock * matchesCondBlock = BasicBlock::Create(b->getContext(), "matchesCond", scanFunc, 0);
    BasicBlock * matchesLoopBlock = BasicBlock::Create(b->getContext(), "matchesLoop", scanFunc, 0);
    BasicBlock * matchesDoneBlock = BasicBlock::Create(b->getContext(), "matchesDone", scanFunc, 0);

    b->SetInsertPoint(entryBlock);
    b->CreateBr(matchesCondBlock);

    b->SetInsertPoint(matchesCondBlock);
    PHINode * matches_phi = b->CreatePHI(T, 2, "matches");
    matches_phi->addIncoming(matchWord, entryBlock);
    Value * have_matches_cond = b->CreateICmpUGT(matches_phi, ConstantInt::get(T, 0));
    b->CreateCondBr(have_matches_cond, matchesLoopBlock, matchesDoneBlock);

    b->SetInsertPoint(matchesLoopBlock);
    Value * match_pos = b->CreateAdd(b->CreateCountForwardZeroes(matches_phi), basePos);
    Value * matches_new = b->CreateAnd(matches_phi, b->CreateSub(matches_phi, ConstantInt::get(T, 1)));
    matches_phi->addIncoming(matches_new, matchesLoopBlock);
    b->CreateCall(matchProcessor, std::vector<Value *>({match_pos, dist}));
    b->CreateBr(matchesCondBlock);

    b->SetInsertPoint(matchesDoneBlock);
    b -> CreateRetVoid();

    return scanFunc;

}

editdScanKernel::editdScanKernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * matchResults) :
BlockOrientedKernel(b, "editdScanMatch" + std::to_string(matchResults->getNumElements()),
              {Binding{"matchResults", matchResults}},
              {}, {}, {}, {InternalScalar{b->getSizeTy(), "BlockNo"}}),
mNumElements(matchResults->getNumElements()),
mScanwordBitWidth(b->getSizeTy()->getBitWidth()) {
    addAttribute(SideEffecting());
}

}
