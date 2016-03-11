#include "symboltablepipeline.h"

/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "pipeline.h"
#include "toolchain.h"
#include "utf_encoding.h"

#include <kernels/scanmatchgen.h>
#include <kernels/s2p_kernel.h>

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

SymbolTableBuilder::SymbolTableBuilder(Module * m, IDISA::IDISA_Builder * b)
: mMod(m)
, iBuilder(b)
, mFileBufIdx(7)
, mFileSizeIdx(8)
, mFileNameIdx(9)
, mLongestLookahead(0)
, mBitBlockType(b->getBitBlockType())
, mBlockSize(b->getBitBlockWidth()){

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
        PabloAST * const L = entry->createLookahead(M, endpoint);
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

    raw_os_ostream out(std::cerr);

    out << "LEADING:\n";
    PabloFunction * const leading = generateLeadingFunction(endpoints);
    PabloPrinter::print(*leading, out);

    out << "\n\nSORTING:\n";
    PabloFunction * const sorting = generateSortingFunction(leading, endpoints);
    PabloPrinter::print(*sorting, out);

    out.flush();

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

void SymbolTableBuilder::ExecuteKernels(){

    Type * T = iBuilder->getIntNTy(64);
    Type * S = PointerType::get(iBuilder->getIntNTy(8), 0);
    Type * inputType = PointerType::get(ArrayType::get(StructType::get(mMod->getContext(), std::vector<Type *>({ArrayType::get(mBitBlockType, 8)})), 1), 0);
    Function * const main = cast<Function>(mMod->getOrInsertFunction("Main", Type::getVoidTy(mMod->getContext()), inputType, T, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();

    Value * const input_param = args++;
    input_param->setName("input");

    Value * const bufferSize = args++;
    bufferSize->setName("buffersize");

    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", main,0));

    BasicBlock * entryBlock = iBuilder->GetInsertBlock();

    BasicBlock * leadingTestBlock = BasicBlock::Create(mMod->getContext(), "ltb", main, 0);
    BasicBlock * leadingBodyBlock = BasicBlock::Create(mMod->getContext(), "lbb", main, 0);
    BasicBlock * leadingExitBlock = BasicBlock::Create(mMod->getContext(), "leb", main, 0);

    BasicBlock * regularTestBlock = BasicBlock::Create(mMod->getContext(), "rtb", main, 0);
    BasicBlock * regularBodyBlock = BasicBlock::Create(mMod->getContext(), "rbb", main, 0);
    BasicBlock * regularExitBlock = BasicBlock::Create(mMod->getContext(), "reb", main, 0);

//    BasicBlock * pipeline_test_block = BasicBlock::Create(mMod->getContext(), "pipeline_test_block", main, 0);
//    BasicBlock * pipeline_do_block = BasicBlock::Create(mMod->getContext(), "pipeline_do_block", main, 0);
//    BasicBlock * pipeline_final_block = BasicBlock::Create(mMod->getContext(), "pipeline_final_block", main, 0);
//    BasicBlock * pipeline_partial_block = BasicBlock::Create(mMod->getContext(), "pipeline_partial_block", main, 0);
//    BasicBlock * pipeline_empty_block = BasicBlock::Create(mMod->getContext(), "pipeline_empty_block", main, 0);
//    BasicBlock * pipeline_end_block = BasicBlock::Create(mMod->getContext(), "pipeline_end_block", main, 0);
//    BasicBlock * pipeline_Unterminated_block = BasicBlock::Create(mMod->getContext(), "pipeline_Unterminated_block", main, 0);
//    BasicBlock * pipeline_return_block = BasicBlock::Create(mMod->getContext(), "pipeline_return_block", main, 0);

    Value * s2pKernelStruct = mS2PKernel->generateKernelInstance();
    Value * leadingKernelStruct = mLeadingKernel->generateKernelInstance();
    Value * sortingKernelStruct = mSortingKernel->generateKernelInstance();

    Value * basis_bits = iBuilder->CreateGEP(s2pKernelStruct, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
    Value * leadingData = iBuilder->CreateGEP(leadingKernelStruct, {iBuilder->getInt32(0), iBuilder->getInt32(1)});


    const unsigned leadingBlocks = (mLongestLookahead + iBuilder->getBitBlockWidth() - 1) / iBuilder->getBitBlockWidth();

    // If the buffer size is smaller than our largest length group, only check up to the buffer size.
    Value * safetyCheck = iBuilder->CreateICmpSLT(bufferSize, iBuilder->getInt64(leadingBlocks * iBuilder->getBitBlockWidth()));
    iBuilder->CreateCondBr(safetyCheck, regularExitBlock, leadingTestBlock);

    // Now process the leading blocks ...
    iBuilder->SetInsertPoint(leadingTestBlock);
    PHINode * remainingBytes = iBuilder->CreatePHI(T, 2, "remainingBytes");
    PHINode * leadingOffset = iBuilder->CreatePHI(T, 2, "blockIndex");
    remainingBytes->addIncoming(bufferSize, entryBlock);
    leadingOffset->addIncoming(iBuilder->getInt64(0), entryBlock);
    Value * remainingLeadingBlocksCond = iBuilder->CreateICmpULT(leadingOffset, iBuilder->getInt64(leadingBlocks));
    iBuilder->CreateCondBr(remainingLeadingBlocksCond, leadingBodyBlock, leadingExitBlock);
    iBuilder->SetInsertPoint(leadingBodyBlock);
    Value * gep = iBuilder->CreateGEP(input_param, leadingOffset);
    mS2PKernel->generateDoBlockCall(gep);
    mLeadingKernel->generateDoBlockCall(basis_bits);
    leadingOffset->addIncoming(iBuilder->CreateAdd(leadingOffset, iBuilder->getInt64(1)), leadingBodyBlock);
    remainingBytes->addIncoming(iBuilder->CreateSub(remainingBytes, iBuilder->getInt64(mBlockSize)), leadingBodyBlock);
    iBuilder->CreateBr(leadingTestBlock);
    iBuilder->SetInsertPoint(leadingExitBlock);

    // Now process the leading blocks ...
    iBuilder->CreateBr(regularTestBlock);
    iBuilder->SetInsertPoint(regularTestBlock);
    PHINode * remainingBytes2 = iBuilder->CreatePHI(T, 2, "remainingBytes");
    PHINode * leadingOffset2 = iBuilder->CreatePHI(T, 2, "blockIndex");
    remainingBytes2->addIncoming(remainingBytes, leadingExitBlock);
    leadingOffset2->addIncoming(leadingOffset, leadingExitBlock);
    Value * remainingBytesCond = iBuilder->CreateICmpUGE(remainingBytes2, iBuilder->getInt64(mBlockSize));
    iBuilder->CreateCondBr(remainingBytesCond, regularBodyBlock, regularExitBlock);
    iBuilder->SetInsertPoint(regularBodyBlock);
    Value * gep2 = iBuilder->CreateGEP(input_param, leadingOffset2);
    mS2PKernel->generateDoBlockCall(gep2);
    mLeadingKernel->generateDoBlockCall(basis_bits);
    leadingOffset2->addIncoming(iBuilder->CreateAdd(leadingOffset2, iBuilder->getInt64(1)), regularBodyBlock);
    remainingBytes2->addIncoming(iBuilder->CreateSub(remainingBytes2, iBuilder->getInt64(mBlockSize)), regularBodyBlock);
    mSortingKernel->generateDoBlockCall(leadingData);
    iBuilder->CreateBr(regularTestBlock);
    iBuilder->SetInsertPoint(regularExitBlock);



//    Value * gep = iBuilder->CreateGEP(sortingKernelStruct, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(mFileBufIdx)});
//    Value* filebuf = iBuilder->CreateBitCast(input_param, S);
//    iBuilder->CreateStore(filebuf, gep);

//    gep = iBuilder->CreateGEP(sortingKernelStruct, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(mFileSizeIdx)});
//    iBuilder->CreateStore(buffersize_param, gep);

//    gep = iBuilder->CreateGEP(sortingKernelStruct, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(mFileNameIdx)});
//    iBuilder->CreateStore(filename_param, gep);

//    Value * basis_bits = iBuilder->CreateGEP(s2pKernelStruct, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
//    Value * results = iBuilder->CreateGEP(leadingKernelStruct, {iBuilder->getInt32(0), iBuilder->getInt32(1)});

//    iBuilder->CreateBr(pipeline_test_block);

//    iBuilder->SetInsertPoint(pipeline_test_block);
//    PHINode * remaining_phi = iBuilder->CreatePHI(T, 2, "remaining");
//    PHINode * blkNo_phi = iBuilder->CreatePHI(T, 2, "blkNo");
//    remaining_phi->addIncoming(buffersize_param, entry_block);
//    blkNo_phi->addIncoming(iBuilder->getInt64(0), entry_block);

//    Value * final_block_cond = iBuilder->CreateICmpSLT(remaining_phi, ConstantInt::get(T, mBlockSize));
//    iBuilder->CreateCondBr(final_block_cond, pipeline_final_block, pipeline_do_block);

//    iBuilder->SetInsertPoint(pipeline_do_block);

//    gep = iBuilder->CreateGEP(input_param, {blkNo_phi});
//    Value * update_blkNo = iBuilder->CreateAdd(blkNo_phi, iBuilder->getInt64(1));
//    blkNo_phi->addIncoming(update_blkNo, pipeline_do_block);

//    mS2PKernel->generateDoBlockCall(gep);
//    mICgrepKernel->generateDoBlockCall(basis_bits);
//    mScanMatchKernel->generateDoBlockCall(results);

//    Value * update_remaining = iBuilder->CreateSub(remaining_phi, iBuilder->getInt64(mBlockSize));
//    remaining_phi->addIncoming(update_remaining, pipeline_do_block);
//    iBuilder->CreateBr(pipeline_test_block);

//    iBuilder->SetInsertPoint(pipeline_final_block);

//    Value * empty_block_cond = iBuilder->CreateICmpEQ(remaining_phi, ConstantInt::get(T, 0));
//    iBuilder->CreateCondBr(empty_block_cond, pipeline_empty_block, pipeline_partial_block);

//    iBuilder->SetInsertPoint(pipeline_partial_block);

//    gep = iBuilder->CreateGEP(input_param, {blkNo_phi});
//    mS2PKernel->generateDoBlockCall(gep);
//    iBuilder->CreateBr(pipeline_end_block);

//    iBuilder->SetInsertPoint(pipeline_empty_block);

//    iBuilder->CreateMemSet(basis_bits, iBuilder->getInt8(0), mBlockSize, 4);
//    iBuilder->CreateBr(pipeline_end_block);

//    iBuilder->SetInsertPoint(pipeline_end_block);

//    Value * return_block_cond = iBuilder->CreateICmpEQ(finalLineUnterminated_param, ConstantInt::get(T, 0));
//    iBuilder->CreateCondBr(return_block_cond, pipeline_return_block, pipeline_Unterminated_block);

//    iBuilder->SetInsertPoint(pipeline_Unterminated_block);

//    Value * remaining = iBuilder->CreateZExt(remaining_phi, iBuilder->getIntNTy(mBlockSize));
//    Value * EOF_pos = iBuilder->CreateShl(ConstantInt::get(iBuilder->getIntNTy(mBlockSize), 1), remaining);
//    EOF_pos = iBuilder->CreateBitCast(EOF_pos, mBitBlockType);

//    Value * gep_bits4 = iBuilder->CreateGEP(basis_bits, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(4)});
//    Value * bits4 = iBuilder->CreateAlignedLoad(gep_bits4, mBlockSize/8, false, "bits4");
//    bits4 = iBuilder->CreateOr(bits4, EOF_pos);
//    iBuilder->CreateAlignedStore(bits4, gep_bits4, mBlockSize/8, false);

//    Value * gep_bits6 = iBuilder->CreateGEP(basis_bits, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(6)});
//    Value * bits6 = iBuilder->CreateAlignedLoad(gep_bits6, mBlockSize/8, false, "bits6");
//    bits6 = iBuilder->CreateOr(bits6, EOF_pos);
//    iBuilder->CreateAlignedStore(bits6, gep_bits6, mBlockSize/8, false);
//    iBuilder->CreateBr(pipeline_return_block);

//    iBuilder->SetInsertPoint(pipeline_return_block);

//    mICgrepKernel->generateDoBlockCall(basis_bits);
//    mScanMatchKernel->generateDoBlockCall(results);
    iBuilder->CreateRetVoid();


    mMod->dump();
}

SymbolTableBuilder::~SymbolTableBuilder() {
    delete mS2PKernel;
    delete mLeadingKernel;
    delete mSortingKernel;
}
