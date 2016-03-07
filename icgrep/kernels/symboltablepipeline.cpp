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
 * @brief generateLookaheadFunction
 ** ------------------------------------------------------------------------------------------------------------- */
PabloFunction * SymbolTableBuilder::generateLookaheadFunction(const PabloFunction * const leading, const std::vector<unsigned> & endpoints) {
    PabloFunction * const function = PabloFunction::Create("lookahead", leading->getNumOfResults(), leading->getNumOfResults());
    PabloBlock * const entry = function->getEntryBlock();
    function->setParameter(0, entry->createVar("S"));
    function->setParameter(1, entry->createVar("E"));
    for (unsigned i = 2; i < leading->getNumOfResults(); ++i) {
        function->setParameter(i, entry->createVar("M" + std::to_string(i - 2)));
    }
    function->setResult(0, entry->createAssign("S", function->getParameter(0)));
    function->setResult(1, entry->createAssign("E", function->getParameter(1)));
    unsigned i = 1;
    unsigned lowerbound = 0;
    for (unsigned endpoint : endpoints) {
        PabloAST * const M = function->getParameter(i + 1);
        PabloAST * const L = entry->createLookahead(M, endpoint);
        function->setResult(i + 1, entry->createAssign("L" + std::to_string(i), L));
        ++i;
        lowerbound = endpoint;
    }
    return function;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateSortingFunction
 ** ------------------------------------------------------------------------------------------------------------- */
PabloFunction * SymbolTableBuilder::generateSortingFunction(const PabloFunction * const lookahead) {
    PabloFunction * const function = PabloFunction::Create("sorting", lookahead->getNumOfResults(), 0);
    PabloBlock * const entry = function->getEntryBlock();
    function->setParameter(0, entry->createVar("S"));
    function->setParameter(1, entry->createVar("E"));
    for (unsigned i = 2; i < lookahead->getNumOfResults(); ++i) {
        function->setParameter(i, entry->createVar("L" + std::to_string(i - 1)));
    }

    PabloAST * R = function->getParameter(0);
    PabloAST * const E = entry->createNot(function->getParameter(1));
    for (unsigned i = 2; i < lookahead->getNumOfResults(); ++i) {
        PabloAST * const L = function->getParameter(i);
        PabloAST * S = entry->createAnd(L, R);
        R = entry->createXor(R, S);
        PabloBlock * const block = PabloBlock::Create(entry);
        PabloAST * F = block->createScanThru(R, E);
        block->createCall(Prototype::Create("length_group_" + std::to_string(i - 1), 2, 0, nullptr), std::vector<PabloAST *>{S, F});
        entry->createIf(S, {}, block);
    }
    PabloBlock * const block = PabloBlock::Create(entry);
    PabloAST * F = block->createScanThru(R, E);
    block->createCall(Prototype::Create("unknown_length_group", 2, 0, nullptr), std::vector<PabloAST *>{R, F});
    entry->createIf(R, {}, block);

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

    mS2PKernel = new KernelBuilder("s2p", mMod, iBuilder);
    mLeadingKernel = new KernelBuilder("leading", mMod, iBuilder);
    mLookaheadKernel = new KernelBuilder("lookahead", mMod, iBuilder);
    mSortingKernel = new KernelBuilder("sorting", mMod, iBuilder);

    generateS2PKernel(mMod, iBuilder, mS2PKernel);

    PabloCompiler pablo_compiler(mMod, iBuilder);

    raw_os_ostream out(std::cerr);

    out << "LEADING:\n";
    PabloFunction * const leading = generateLeadingFunction(endpoints);
    PabloPrinter::print(*leading, out);

//    out << "\n\nLOOKAHEAD:\n";
//    PabloFunction * const lookahead = generateLookaheadFunction(leading, endpoints);
//    PabloPrinter::print(*lookahead, out);

//    out << "\n\nSORTING:\n";
//    PabloFunction * const sorting = generateSortingFunction(lookahead);
//    PabloPrinter::print(*sorting, out);

    out.flush();

    pablo_compiler.setKernel(mLeadingKernel);
    pablo_compiler.compile(leading);
//    pablo_compiler.setKernel(mLookaheadKernel);
//    pablo_compiler.compile(lookahead);
//    pablo_compiler.setKernel(mSortingKernel);
//    pablo_compiler.compile(sorting);

    delete leading;
//    delete lookahead;
//    delete sorting;
    releaseSlabAllocatorMemory();

}

void SymbolTableBuilder::ExecuteKernels(){

//    Type * T = iBuilder->getIntNTy(64);
//    Type * S = PointerType::get(iBuilder->getIntNTy(8), 0);
//    Type * inputType = PointerType::get(ArrayType::get(StructType::get(mMod->getContext(), std::vector<Type *>({ArrayType::get(mBitBlockType, 8)})), 1), 0);
//    Function * const main = cast<Function>(mMod->getOrInsertFunction("Main", Type::getVoidTy(mMod->getContext()), inputType, T, S, T, nullptr));
//    main->setCallingConv(CallingConv::C);
//    Function::arg_iterator args = main->arg_begin();

//    Value* input_param = args++;
//    input_param->setName("input");
//    Value* buffersize_param = args++;
//    buffersize_param->setName("buffersize");
//    Value* filename_param = args++;
//    filename_param->setName("filename");
//    Value* finalLineUnterminated_param = args++;
//    finalLineUnterminated_param->setName("finalLineUnterminated");

//    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", main,0));

//    BasicBlock * entry_block = iBuilder->GetInsertBlock();
//    BasicBlock * pipeline_test_block = BasicBlock::Create(mMod->getContext(), "pipeline_test_block", main, 0);
//    BasicBlock * pipeline_do_block = BasicBlock::Create(mMod->getContext(), "pipeline_do_block", main, 0);
//    BasicBlock * pipeline_final_block = BasicBlock::Create(mMod->getContext(), "pipeline_final_block", main, 0);
//    BasicBlock * pipeline_partial_block = BasicBlock::Create(mMod->getContext(), "pipeline_partial_block", main, 0);
//    BasicBlock * pipeline_empty_block = BasicBlock::Create(mMod->getContext(), "pipeline_empty_block", main, 0);
//    BasicBlock * pipeline_end_block = BasicBlock::Create(mMod->getContext(), "pipeline_end_block", main, 0);
//    BasicBlock * pipeline_Unterminated_block = BasicBlock::Create(mMod->getContext(), "pipeline_Unterminated_block", main, 0);
//    BasicBlock * pipeline_return_block = BasicBlock::Create(mMod->getContext(), "pipeline_return_block", main, 0);

//    Value * s2pKernelStruct = mS2PKernel->generateKernelInstance();
//    Value * icGrepKernelStruct = mICgrepKernel->generateKernelInstance();
//    Value * scanMatchKernelStruct = mScanMatchKernel->generateKernelInstance();

//    Value * gep = iBuilder->CreateGEP(scanMatchKernelStruct, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(mFileBufIdx)});
//    Value* filebuf = iBuilder->CreateBitCast(input_param, S);
//    iBuilder->CreateStore(filebuf, gep);

//    gep = iBuilder->CreateGEP(scanMatchKernelStruct, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(mFileSizeIdx)});
//    iBuilder->CreateStore(buffersize_param, gep);

//    gep = iBuilder->CreateGEP(scanMatchKernelStruct, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(mFileNameIdx)});
//    iBuilder->CreateStore(filename_param, gep);

//    Value * basis_bits = iBuilder->CreateGEP(s2pKernelStruct, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
//    Value * results = iBuilder->CreateGEP(icGrepKernelStruct, {iBuilder->getInt32(0), iBuilder->getInt32(1)});

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
//    iBuilder->CreateRetVoid();

}
