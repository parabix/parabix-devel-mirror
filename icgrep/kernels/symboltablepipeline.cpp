#include "symboltablepipeline.h"

/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "pipeline.h"
#include "toolchain.h"
#include "utf_encoding.h"

#include <kernels/s2p_kernel.h>
#include <kernels/instance.h>

#include <pablo/function.h>
#include <pablo/pablo_compiler.h>

#include <re/re_cc.h>
#include <re/re_rep.h>
#include <re/re_name.h>
#include <re/re_compiler.h>
#include <re/printer_re.h>

#include <cc/cc_compiler.h>

#include <pablo/printer_pablos.h>
#include <iostream>

using namespace re;
using namespace pablo;

namespace kernel {

SymbolTableBuilder::SymbolTableBuilder(Module * m, IDISA::IDISA_Builder * b)
: mMod(m)
, iBuilder(b)
, mLongestLookahead(0)
, mBitBlockType(b->getBitBlockType())
, mBlockSize(b->getBitBlockWidth()) {

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateLeadingFunction
 ** ------------------------------------------------------------------------------------------------------------- */
PabloFunction * SymbolTableBuilder::generateLeadingFunction(const std::vector<unsigned> & endpoints) {
    PabloFunction * const function = PabloFunction::Create("leading", 8, endpoints.size() + 2);
    Encoding enc(Encoding::Type::ASCII, 8);
    cc::CC_Compiler ccCompiler(*function, enc);
    re::RE_Compiler reCompiler(*function, ccCompiler);
    RE * cc = makeName(makeCC(makeCC(65, 90), makeCC(97, 122)));
    reCompiler.compileUnicodeNames(cc);
    PabloAST * const matches = reCompiler.compile(cc).stream;
    PabloBlock * const entry = function->getEntryBlock();
    PabloAST * const adv = entry->createAdvance(matches, 1);
    PabloAST * const starts = entry->createAnd(matches, entry->createNot(adv));
    PabloAST * const ends = entry->createAnd(adv, entry->createNot(matches));

    function->setResult(0, entry->createAssign("S", starts));
    function->setResult(1, entry->createAssign("E", ends));

    PabloAST * M = ends;
    unsigned step = 1;
    unsigned i = 0;
    for (unsigned endpoint : endpoints) {
        assert (endpoint >= step);
        unsigned span = endpoint - step;
        while (span > step) {
            M = entry->createOr(entry->createAdvance(M, step), M);
            span = span - step;
            step *= 2;
        }
        M = entry->createOr(entry->createAdvance(M, span), M);
        function->setResult(i + 2, entry->createAssign("M" + std::to_string(i), M));
        ++i;
        step += span;
    }

    return function;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateSortingFunction
 ** ------------------------------------------------------------------------------------------------------------- */
PabloFunction * SymbolTableBuilder::generateSortingFunction(const PabloFunction * const leading, const std::vector<unsigned> & endpoints) {
    PabloFunction * const function = PabloFunction::Create("sorting", leading->getNumOfResults(), leading->getNumOfResults() * 2);
    PabloBlock * const entry = function->getEntryBlock();
    function->setParameter(0, entry->createVar("S"));
    function->setParameter(1, entry->createVar("E"));
    for (unsigned i = 2; i < leading->getNumOfResults(); ++i) {
        function->setParameter(i, entry->createVar("M" + std::to_string(i - 2)));
    }
    PabloAST * R = function->getParameter(0);
    PabloAST * const E = entry->createNot(function->getParameter(1));
    unsigned i = 1;
    unsigned lowerbound = 0;
    for (unsigned endpoint : endpoints) {
        PabloAST * const M = function->getParameter(i + 1);
        PabloAST * const L = entry->createLookahead(M, endpoint, "lookahead" + std::to_string(endpoint));
        PabloAST * S = entry->createAnd(L, R);
        Assign * Si = entry->createAssign("S_" + std::to_string(i), S);
        R = entry->createXor(R, S);
        PabloAST * F = entry->createScanThru(R, E);
        Assign * Ei = entry->createAssign("E_" + std::to_string(i), F);
        function->setResult(i * 2, Si);
        function->setResult(i * 2 + 1, Ei);
        ++i;
        lowerbound = endpoint;
    }
    Assign * Si = entry->createAssign("S_n", R);
    PabloAST * F = entry->createScanThru(R, E);
    Assign * Ei = entry->createAssign("E_n", F);
    function->setResult(i * 2, Si);
    function->setResult(i * 2 + 1, Ei);
    mLongestLookahead = lowerbound;
    return function;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createKernels
 ** ------------------------------------------------------------------------------------------------------------- */
void SymbolTableBuilder::createKernels() {

    std::vector<unsigned> endpoints;
    endpoints.push_back(1);
    endpoints.push_back(2);
    endpoints.push_back(4);
    endpoints.push_back(8);
    endpoints.push_back(16);

    PabloCompiler pablo_compiler(mMod, iBuilder);
    PabloFunction * const leading = generateLeadingFunction(endpoints);
    PabloFunction * const sorting = generateSortingFunction(leading, endpoints);

    mS2PKernel = new KernelBuilder("s2p", mMod, iBuilder);
    mLeadingKernel = new KernelBuilder("leading", mMod, iBuilder);
    mSortingKernel = new KernelBuilder("sorting", mMod, iBuilder);

    mLeadingKernel->setLongestLookaheadAmount(mLongestLookahead);
    mSortingKernel->setLongestLookaheadAmount(mLongestLookahead);

    generateS2PKernel(mMod, iBuilder, mS2PKernel);

    pablo_compiler.setKernel(mLeadingKernel);
    pablo_compiler.compile(leading);
    pablo_compiler.setKernel(mSortingKernel);
    pablo_compiler.compile(sorting);

    delete leading;
    delete sorting;

    releaseSlabAllocatorMemory();
}

Function * SymbolTableBuilder::ExecuteKernels(){

    Type * intType = iBuilder->getInt64Ty();

    Type * inputType = PointerType::get(ArrayType::get(StructType::get(mMod->getContext(), std::vector<Type *>({ArrayType::get(mBitBlockType, 8)})), 1), 0);
    Function * const main = cast<Function>(mMod->getOrInsertFunction("Main", Type::getVoidTy(mMod->getContext()), inputType, intType, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();

    Value * const inputStream = args++;
    inputStream->setName("input");

    Value * const bufferSize = args++;
    bufferSize->setName("buffersize");

    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", main,0));

    BasicBlock * entryBlock = iBuilder->GetInsertBlock();

    BasicBlock * leadingTestBlock = BasicBlock::Create(mMod->getContext(), "leadingCond", main, 0);
    BasicBlock * leadingBodyBlock = BasicBlock::Create(mMod->getContext(), "leadingBody", main, 0);

    BasicBlock * regularTestBlock = BasicBlock::Create(mMod->getContext(), "fullCond", main, 0);
    BasicBlock * regularBodyBlock = BasicBlock::Create(mMod->getContext(), "fullBody", main, 0);
    BasicBlock * regularExitBlock = BasicBlock::Create(mMod->getContext(), "fullExit", main, 0);

    BasicBlock * partialBlock = BasicBlock::Create(mMod->getContext(),  "partialBlock", main, 0);

    BasicBlock * finalTestBlock = BasicBlock::Create(mMod->getContext(),  "finalCond", main, 0);
    BasicBlock * finalBodyBlock = BasicBlock::Create(mMod->getContext(),  "finalBody", main, 0);

    BasicBlock * exitBlock = BasicBlock::Create(mMod->getContext(), "exit", main, 0);

    Instance * s2pInstance = mS2PKernel->instantiate();
    Instance * leadingInstance = mLeadingKernel->instantiate();
    Instance * sortingInstance = mSortingKernel->instantiate();

    Value * basisBits = s2pInstance->getOutputStreamSet();
    Value * leadingData = leadingInstance->getOutputStreamSet();

    const unsigned leadingBlocks = (mLongestLookahead + iBuilder->getBitBlockWidth() - 1) / iBuilder->getBitBlockWidth();

    Value * const requiredBytes = iBuilder->getInt64(mBlockSize * leadingBlocks);
    Value * const blockSize = iBuilder->getInt64(mBlockSize);

    // If the buffer size is smaller than our largest length group, only check up to the buffer size.
    Value * safetyCheck = iBuilder->CreateICmpUGE(bufferSize, blockSize);
    if (blockSize == requiredBytes) {
        iBuilder->CreateCondBr(safetyCheck, leadingTestBlock, exitBlock); // fix this to be a special case
    } else {
        throw std::runtime_error("Not supported yet!");
    }

    // First compute any necessary leading blocks to allow the sorting kernel access to the "future" data produced by
    // the leading kernel ...
    iBuilder->SetInsertPoint(leadingTestBlock);
    PHINode * blockNo = iBuilder->CreatePHI(intType, 2);
    blockNo->addIncoming(iBuilder->getInt64(0), entryBlock);
    PHINode * remainingBytes = iBuilder->CreatePHI(intType, 2);
    remainingBytes->addIncoming(bufferSize, entryBlock);
    Value * leadingBlocksCond = iBuilder->CreateICmpULT(blockNo, iBuilder->getInt64(leadingBlocks));
    iBuilder->CreateCondBr(leadingBlocksCond, leadingBodyBlock, regularTestBlock);
    iBuilder->SetInsertPoint(leadingBodyBlock);
    s2pInstance->call(iBuilder->CreateGEP(inputStream, blockNo));
    leadingInstance->call(basisBits);
    blockNo->addIncoming(iBuilder->CreateAdd(blockNo, iBuilder->getInt64(1)), leadingBodyBlock);
    remainingBytes->addIncoming(iBuilder->CreateSub(remainingBytes, blockSize), leadingBodyBlock);
    iBuilder->CreateBr(leadingTestBlock);

    // Now all the data for which we can produce and consume a full leading block...
    iBuilder->SetInsertPoint(regularTestBlock);
    PHINode * blockNo2 = iBuilder->CreatePHI(intType, 2);
    blockNo2->addIncoming(blockNo, leadingTestBlock);
    PHINode * remainingBytes2 = iBuilder->CreatePHI(intType, 2);
    remainingBytes2->addIncoming(remainingBytes, leadingTestBlock);
    Value * remainingBytesCond = iBuilder->CreateICmpUGE(remainingBytes2, requiredBytes);
    iBuilder->CreateCondBr(remainingBytesCond, regularBodyBlock, regularExitBlock);
    iBuilder->SetInsertPoint(regularBodyBlock);
    s2pInstance->call(iBuilder->CreateGEP(inputStream, blockNo2));
    leadingInstance->call(basisBits);
    sortingInstance->call(leadingData);
    blockNo2->addIncoming(iBuilder->CreateAdd(blockNo2, iBuilder->getInt64(1)), regularBodyBlock);
    remainingBytes2->addIncoming(iBuilder->CreateSub(remainingBytes2, blockSize), regularBodyBlock);
    iBuilder->CreateBr(regularTestBlock);


    // Check if we have a partial blocks worth of leading data remaining
    iBuilder->SetInsertPoint(regularExitBlock);
    Value * partialBlockCond = iBuilder->CreateICmpUGT(remainingBytes2, ConstantInt::getNullValue(intType));
    iBuilder->CreateCondBr(partialBlockCond, partialBlock, finalTestBlock);

    // If we do, process it and mask out the data
    iBuilder->SetInsertPoint(partialBlock);
    s2pInstance->call(iBuilder->CreateGEP(inputStream, blockNo2));
    Value * partialLeadingData[2];
    for (unsigned i = 0; i < 2; ++i) {
        partialLeadingData[i] = leadingInstance->getOutputStream(i);
    }
    leadingInstance->call(basisBits);
    Type * fullBitBlockType = iBuilder->getIntNTy(mBlockSize);
    Value * remaining = iBuilder->CreateZExt(iBuilder->CreateSub(blockSize, remainingBytes2), fullBitBlockType);
    Value * eofMask = iBuilder->CreateLShr(ConstantInt::getAllOnesValue(fullBitBlockType), remaining);
    eofMask = iBuilder->CreateBitCast(eofMask, mBitBlockType);
    for (unsigned i = 0; i < 2; ++i) {
        Value * value = iBuilder->CreateAnd(iBuilder->CreateBlockAlignedLoad(partialLeadingData[i]), eofMask);
        iBuilder->CreateBlockAlignedStore(value, partialLeadingData[i]);
    }
    for (unsigned i = 0; i < 2; ++i) {
        iBuilder->CreateBlockAlignedStore(ConstantInt::getNullValue(mBitBlockType), leadingInstance->getOutputStream(i));
    }
    sortingInstance->call(leadingData);
    iBuilder->CreateBr(finalTestBlock);

    // Now clear the leading data and test the final blocks
    iBuilder->SetInsertPoint(finalTestBlock);
    PHINode * remainingFullBlocks = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 3);
    remainingFullBlocks->addIncoming(iBuilder->getInt64(leadingBlocks), regularExitBlock);
    remainingFullBlocks->addIncoming(iBuilder->getInt64(leadingBlocks), partialBlock);
    Value * remainingFullBlocksCond = iBuilder->CreateICmpUGT(remainingFullBlocks, ConstantInt::getNullValue(intType));
    iBuilder->CreateCondBr(remainingFullBlocksCond, finalBodyBlock, exitBlock);

    iBuilder->SetInsertPoint(finalBodyBlock);
    for (unsigned i = 0; i < 2; ++i) {
        iBuilder->CreateBlockAlignedStore(ConstantInt::getNullValue(mBitBlockType), leadingInstance->getOutputStream(i));
    }
    Value * blockNoPtr = leadingInstance->getBlockNo();
    Value * blockNoValue = iBuilder->CreateLoad(blockNoPtr);
    blockNoValue = iBuilder->CreateAdd(blockNoValue, ConstantInt::get(blockNoValue->getType(), 1));
    iBuilder->CreateStore(blockNoValue, blockNoPtr);

    sortingInstance->call(leadingData);

    remainingFullBlocks->addIncoming(iBuilder->CreateSub(remainingFullBlocks, iBuilder->getInt64(1)), finalBodyBlock);

    iBuilder->CreateBr(finalTestBlock);

    iBuilder->SetInsertPoint(exitBlock);
    iBuilder->CreateRetVoid();

    main->dump();

    return main;
}

SymbolTableBuilder::~SymbolTableBuilder() {
    delete mS2PKernel;
    delete mLeadingKernel;
    delete mSortingKernel;
}

}
