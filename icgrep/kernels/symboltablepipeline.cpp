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
#include <pablo/analysis/pabloverifier.hpp>

#include <re/re_cc.h>
#include <re/re_rep.h>
#include <re/re_name.h>
#include <re/re_compiler.h>
#include <re/printer_re.h>

#include <cc/cc_compiler.h>

#include <pablo/printer_pablos.h>
#include <iostream>

#include <llvm/IR/Intrinsics.h>

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
    PabloBlock * entry = function->getEntryBlock();
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
        PabloAST * F = entry->createScanThru(R, E);
        Assign * Ei = entry->createAssign("E_" + std::to_string(i), F);
        function->setResult(i * 2, Si);
        function->setResult(i * 2 + 1, Ei);
        R = entry->createXor(R, S);
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
 * @brief generateCountForwardZeroes
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * generateCountForwardZeroes(IDISA::IDISA_Builder * iBuilder, Value * bits) {
    Value * cttzFunc = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::cttz, bits->getType());
    return iBuilder->CreateCall(cttzFunc, std::vector<Value *>({bits, ConstantInt::get(iBuilder->getInt1Ty(), 0)}));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateGather
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * SymbolTableBuilder::generateGather(Value * const base, Value * const vindex) {

    /*
        From Intel:

        extern __m256i _mm256_i32gather_epi32(int const * base, __m256i vindex, const int scale)

        From Clang avx2intrin.h:

        #define _mm256_i32gather_epi32(m, i, s) __extension__ ({ \
          (__m256i)__builtin_ia32_gatherd_d256((__v8si)_mm256_undefined_si256(), \
                                               (int const *)(m),  \
                                               (__v8si)(__m256i)(i), \
                                               (__v8si)_mm256_set1_epi32(-1), (s)); })

        From llvm IntrinsicsX86.td:

        def llvm_ptr_ty        : LLVMPointerType<llvm_i8_ty>;             // i8*

        def int_x86_avx2_gather_d_d_256 : GCCBuiltin<"__builtin_ia32_gatherd_d256">,
           Intrinsic<[llvm_v8i32_ty],
           [llvm_v8i32_ty, llvm_ptr_ty, llvm_v8i32_ty, llvm_v8i32_ty, llvm_i8_ty],
           [IntrReadArgMem]>;

     */

    VectorType * const vecType = VectorType::get(iBuilder->getInt32Ty(), 8);
    Function * vgather = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::x86_avx2_gather_d_d_256);
    return iBuilder->CreateCall(vgather, {Constant::getAllOnesValue(vecType), base, iBuilder->CreateBitCast(vindex, vecType), Constant::getAllOnesValue(vecType), iBuilder->getInt8(1)});
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateMaskedGather
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * SymbolTableBuilder::generateMaskedGather(Value * const base, Value * const vindex, Value * const mask) {

    /*
        From Intel:

        extern __m256i _mm256_mask_i32gather_epi32(__m256i def_vals, int const * base, __m256i vindex, __m256i vmask, const int scale);

        From Clang avx2intrin.h:

        #define _mm256_mask_i32gather_epi32(a, m, i, mask, s) __extension__ ({ \
           (__m256i)__builtin_ia32_gatherd_d256((__v8si)(__m256i)(a), \
                                                (int const *)(m), \
                                                (__v8si)(__m256i)(i), \
                                                (__v8si)(__m256i)(mask), (s)); })
        From llvm IntrinsicsX86.td:

        def llvm_ptr_ty        : LLVMPointerType<llvm_i8_ty>;             // i8*

        def int_x86_avx2_gather_d_d_256 : GCCBuiltin<"__builtin_ia32_gatherd_d256">,
           Intrinsic<[llvm_v8i32_ty],
           [llvm_v8i32_ty, llvm_ptr_ty, llvm_v8i32_ty, llvm_v8i32_ty, llvm_i8_ty],
           [IntrReadArgMem]>;

     */

    VectorType * const vecType = VectorType::get(iBuilder->getInt32Ty(), 8);
    Function * vgather = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::x86_avx2_gather_d_d_256);
    return iBuilder->CreateCall(vgather, {Constant::getNullValue(vecType), base, iBuilder->CreateBitCast(vindex, vecType), iBuilder->CreateBitCast(mask, vecType), iBuilder->getInt8(1)});
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateResetLowestBit
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * generateResetLowestBit(IDISA::IDISA_Builder * iBuilder, Value * bits) {
    Value * bits_minus1 = iBuilder->CreateSub(bits, ConstantInt::get(bits->getType(), 1));
    return iBuilder->CreateAnd(bits_minus1, bits);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateScanMatch
 ** ------------------------------------------------------------------------------------------------------------- */
void SymbolTableBuilder::generateScannerKernel(KernelBuilder * kBuilder, const unsigned minKeyLength, const unsigned maxKeyLength, const unsigned scanWordBitWidth) {

    Type * intScanWordTy = iBuilder->getIntNTy(scanWordBitWidth);
    const unsigned fieldCount = iBuilder->getBitBlockWidth() / scanWordBitWidth;
    Type * scanWordVectorType =  VectorType::get(intScanWordTy, fieldCount);
    const unsigned vectorWidth = iBuilder->getBitBlockWidth() / 32;
    Type * gatherVectorType =  VectorType::get(iBuilder->getInt32Ty(), vectorWidth);

    const unsigned baseIdx = kBuilder->addInternalState(iBuilder->getInt8PtrTy(), "Base");
    const unsigned startIndexIdx = kBuilder->addInternalState(iBuilder->getInt32Ty(), "StartIndex");
    const unsigned startArrayIdx = kBuilder->addInternalState(ArrayType::get(iBuilder->getInt32Ty(), iBuilder->getBitBlockWidth() + vectorWidth), "StartArray");
    const unsigned endIndexIdx = kBuilder->addInternalState(iBuilder->getInt32Ty(), "EndIndex");
    const unsigned endArrayIdx = kBuilder->addInternalState(gatherVectorType, "EndArray");

    kBuilder->addInputStream(1, "startStream");
    kBuilder->addInputStream(1, "endStream");

    Function * function = kBuilder->prepareFunction();

    BasicBlock * const entry = iBuilder->GetInsertBlock();

    BasicBlock * startOuterCond = BasicBlock::Create(mMod->getContext(), "startOuterCond", function, 0);
    BasicBlock * startOuterBody = BasicBlock::Create(mMod->getContext(), "startOuterBody", function, 0);
    BasicBlock * startInnerCond = BasicBlock::Create(mMod->getContext(), "startInnerCond", function, 0);
    BasicBlock * startInnerBody = BasicBlock::Create(mMod->getContext(), "startInnerBody", function, 0);

    BasicBlock * endOuterCond = BasicBlock::Create(mMod->getContext(), "endOuterCond", function, 0);
    BasicBlock * endOuterBody = BasicBlock::Create(mMod->getContext(), "endOuterBody", function, 0);
    BasicBlock * endInnerCond = BasicBlock::Create(mMod->getContext(), "endInnerCond", function, 0);
    BasicBlock * endInnerBody = BasicBlock::Create(mMod->getContext(), "endInnerBody", function, 0);

    BasicBlock * gatherInit = BasicBlock::Create(mMod->getContext(), "gatherInit", function, 0);

    BasicBlock * gatherFullCond = BasicBlock::Create(mMod->getContext(), "gatherFullCond", function, 0);
    BasicBlock * gatherFullBody = BasicBlock::Create(mMod->getContext(), "gatherFullBody", function, 0);

//    BasicBlock * gatherPartialCond = BasicBlock::Create(mMod->getContext(), "gatherPartialCond", function, 0);
//    BasicBlock * gatherPartialBody = BasicBlock::Create(mMod->getContext(), "gatherPartialBody", function, 0);

    BasicBlock * exit = BasicBlock::Create(mMod->getContext(), "exit", function, 0);

    //TODO: this won't work on files > 2^32 bytes yet; needs an intermediate flush then a recalculation of the base pointer.
    Value * const base = iBuilder->CreateLoad(kBuilder->getInternalState(baseIdx), "base");
    Value * blockPos = iBuilder->CreateLoad(kBuilder->getBlockNo());
    blockPos = iBuilder->CreateMul(blockPos, iBuilder->getInt64(iBuilder->getBitBlockWidth()));

    // if two positions cannot be in the same vector element, we could possibly do some work in parallel here.
    Value * startIndex = iBuilder->CreateLoad(kBuilder->getInternalState(startIndexIdx), "startIndex");
    Value * startArray = kBuilder->getInternalState(startArrayIdx);
    Value * startStream = iBuilder->CreateBitCast(iBuilder->CreateBlockAlignedLoad(kBuilder->getInputStream(0)), scanWordVectorType, "startStream");

    Value * endIndex = iBuilder->CreateLoad(kBuilder->getInternalState(endIndexIdx), "endIndex");
    Value * endArray = kBuilder->getInternalState(endArrayIdx);
    Value * endStream = iBuilder->CreateBitCast(iBuilder->CreateBlockAlignedLoad(kBuilder->getInputStream(1)), scanWordVectorType, "endStream");

    iBuilder->CreateBr(startOuterCond);
    iBuilder->SetInsertPoint(startOuterCond);

    PHINode * startIV = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 2);
    startIV->addIncoming(iBuilder->getInt64(0), entry);
    Value * startOuterTest = iBuilder->CreateICmpNE(startIV, iBuilder->getInt64(fieldCount));
    iBuilder->CreateCondBr(startOuterTest, startOuterBody, endOuterCond);

    iBuilder->SetInsertPoint(startOuterBody);
    Value * startField = iBuilder->CreateExtractElement(startStream, startIV);
    startIV->addIncoming(iBuilder->CreateAdd(startIV, iBuilder->getInt64(1)), startInnerCond);
    iBuilder->CreateBr(startInnerCond);

    iBuilder->SetInsertPoint(startInnerCond);
    PHINode * startIndexPhi = iBuilder->CreatePHI(startIndex->getType(), 2);
    startIndexPhi->addIncoming(startIndex, startOuterBody);
    PHINode * startFieldPhi = iBuilder->CreatePHI(intScanWordTy, 2);
    startFieldPhi->addIncoming(startField, startOuterBody);
    Value * test = iBuilder->CreateICmpNE(startFieldPhi, ConstantInt::getNullValue(intScanWordTy));
    iBuilder->CreateCondBr(test, startInnerBody, startOuterCond);

    iBuilder->SetInsertPoint(startInnerBody);
    Value * startPos = generateCountForwardZeroes(iBuilder, startFieldPhi);
    startFieldPhi->addIncoming(generateResetLowestBit(iBuilder, startFieldPhi), startInnerBody);
    startPos = iBuilder->CreateTruncOrBitCast(iBuilder->CreateOr(startPos, blockPos), iBuilder->getInt32Ty());
    iBuilder->CreateStore(startPos, iBuilder->CreateGEP(startArray, {iBuilder->getInt32(0), startIndexPhi}));
    startIndexPhi->addIncoming(iBuilder->CreateAdd(startIndexPhi, ConstantInt::get(startIndexPhi->getType(), 1)), startInnerBody);
    iBuilder->CreateBr(startInnerCond);
    // END POINT OUTER COND
    iBuilder->SetInsertPoint(endOuterCond);
    PHINode * endIV = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 2);
    endIV->addIncoming(iBuilder->getInt64(0), startOuterCond);
    Value * endOuterTest = iBuilder->CreateICmpNE(endIV, iBuilder->getInt64(fieldCount));
    iBuilder->CreateCondBr(endOuterTest, endOuterBody, exit);
    // END POINT OUTER BODY
    iBuilder->SetInsertPoint(endOuterBody);
    Value * endField = iBuilder->CreateExtractElement(endStream, endIV);
    endIV->addIncoming(iBuilder->CreateAdd(endIV, iBuilder->getInt64(1)), endInnerCond);
    iBuilder->CreateBr(endInnerCond);
    // END POINT INNER COND
    iBuilder->SetInsertPoint(endInnerCond);
    PHINode * endIndexPhi = iBuilder->CreatePHI(endIndex->getType(), 3);
    endIndexPhi->addIncoming(endIndex, endOuterBody);
    PHINode * endFieldPhi = iBuilder->CreatePHI(intScanWordTy, 3);
    endFieldPhi->addIncoming(endField, endOuterBody);
    Value * endInnerTest = iBuilder->CreateICmpNE(endFieldPhi, ConstantInt::getNullValue(intScanWordTy));
    iBuilder->CreateCondBr(endInnerTest, endInnerBody, endOuterCond);
    // END POINT INNER BODY
    iBuilder->SetInsertPoint(endInnerBody);
    Value * endPos = generateCountForwardZeroes(iBuilder, endFieldPhi);
    Value * updatedEndFieldPhi = generateResetLowestBit(iBuilder, endFieldPhi);
    endFieldPhi->addIncoming(updatedEndFieldPhi, endInnerBody);
    endPos = iBuilder->CreateTruncOrBitCast(iBuilder->CreateOr(endPos, blockPos), iBuilder->getInt32Ty());
    iBuilder->CreateStore(endPos, iBuilder->CreateGEP(endArray, {iBuilder->getInt32(0), endIndexPhi}));
    Value * updatedEndIndexPhi = iBuilder->CreateAdd(endIndexPhi, ConstantInt::get(endIndexPhi->getType(), 1));
    endIndexPhi->addIncoming(updatedEndIndexPhi, endInnerBody);
    Value * filledEndPosBufferTest = iBuilder->CreateICmpEQ(updatedEndIndexPhi, ConstantInt::get(updatedEndIndexPhi->getType(), vectorWidth));
    iBuilder->CreateCondBr(filledEndPosBufferTest, gatherInit, endInnerCond);
    // GATHER INIT
    iBuilder->SetInsertPoint(gatherInit);
    Value * rawTokenBuffer = iBuilder->CreateAlloca(ArrayType::get(gatherVectorType, (maxKeyLength / 4) + (maxKeyLength % 4) != 0 ? 1 : 0));
    rawTokenBuffer = iBuilder->CreatePointerCast(rawTokenBuffer, PointerType::get(gatherVectorType, 0));
    Value * const startPositions = iBuilder->CreateAlignedLoad(iBuilder->CreatePointerCast(startArray, PointerType::get(gatherVectorType, 0)), 4);
    iBuilder->CreateBr(gatherFullCond);
    // GATHER FULL COND
    iBuilder->SetInsertPoint(gatherFullCond);

    endIndexPhi->addIncoming(iBuilder->getInt32(0), gatherFullCond);
    endFieldPhi->addIncoming(updatedEndFieldPhi, gatherFullCond);

    PHINode * fullGatherIV = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 2);
    fullGatherIV->addIncoming(iBuilder->getInt64(0), gatherInit);
    PHINode * startPositionsPhi = iBuilder->CreatePHI(startPositions->getType(), 2);
    startPositionsPhi->addIncoming(startPositions, gatherInit);

    Value * fullGatherTest = iBuilder->CreateICmpNE(fullGatherIV, iBuilder->getInt64(minKeyLength / vectorWidth));
    iBuilder->CreateCondBr(fullGatherTest, gatherFullBody, endInnerCond);
    // GATHER FULL BODY
    iBuilder->SetInsertPoint(gatherFullBody);
    Value * gathered = generateGather(base, startPositionsPhi);
    startPositionsPhi->addIncoming(iBuilder->CreateAdd(startPositionsPhi, iBuilder->CreateVectorSplat(vectorWidth, iBuilder->getInt32(4))), gatherFullBody);
    iBuilder->CreateAlignedStore(gathered, iBuilder->CreateGEP(rawTokenBuffer, fullGatherIV), 4);
    fullGatherIV->addIncoming(iBuilder->CreateAdd(fullGatherIV, iBuilder->getInt64(1)), gatherFullBody);
    iBuilder->CreateBr(gatherFullCond);

    iBuilder->SetInsertPoint(exit);
    // need to save the start/end index still
    kBuilder->finalize();
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

    const auto bufferSize = ((mLongestLookahead + iBuilder->getBitBlockWidth() - 1) / iBuilder->getBitBlockWidth()) + 1;

    mS2PKernel = new KernelBuilder("s2p", mMod, iBuilder, 1);
    mLeadingKernel = new KernelBuilder("leading", mMod, iBuilder, bufferSize);
    mSortingKernel = new KernelBuilder("sorting", mMod, iBuilder, bufferSize);
    mScannerKernel = new KernelBuilder("scanner", mMod, iBuilder, 1);

    generateS2PKernel(mMod, iBuilder, mS2PKernel);

    pablo_compiler.setKernel(mLeadingKernel);
    pablo_compiler.compile(leading);
    pablo_compiler.setKernel(mSortingKernel);
    pablo_compiler.compile(sorting);

    delete leading;
    delete sorting;

    releaseSlabAllocatorMemory();

    generateScannerKernel(mScannerKernel, 1, 1, 64);

}

Function * SymbolTableBuilder::ExecuteKernels(){

    Type * intType = iBuilder->getInt64Ty();

    Type * inputType = PointerType::get(ArrayType::get(StructType::get(mMod->getContext(), std::vector<Type *>({ArrayType::get(mBitBlockType, 8)})), 1), 0);
    Function * const main = cast<Function>(mMod->getOrInsertFunction("Main", Type::getVoidTy(mMod->getContext()), inputType, intType, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();

    Value * const inputStream = args++;
    inputStream->setName("inputStream");

    Value * const bufferSize = args++;
    bufferSize->setName("bufferSize");

    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", main,0));

    BasicBlock * entryBlock = iBuilder->GetInsertBlock();

    BasicBlock * leadingTestBlock = BasicBlock::Create(mMod->getContext(), "leadingCond", main, 0);
    BasicBlock * safetyCheckBlock = BasicBlock::Create(mMod->getContext(), "safetyCheck", main, 0);
    BasicBlock * leadingBodyBlock = BasicBlock::Create(mMod->getContext(), "leadingBody", main, 0);

    BasicBlock * regularTestBlock = BasicBlock::Create(mMod->getContext(), "fullCond", main, 0);
    BasicBlock * regularBodyBlock = BasicBlock::Create(mMod->getContext(), "fullBody", main, 0);
    BasicBlock * regularExitBlock = BasicBlock::Create(mMod->getContext(), "fullExit", main, 0);

    BasicBlock * partialBlock = BasicBlock::Create(mMod->getContext(),  "partialBlock", main, 0);

    BasicBlock * finalTestBlock = BasicBlock::Create(mMod->getContext(),  "finalCond", main, 0);
    BasicBlock * finalBodyBlock = BasicBlock::Create(mMod->getContext(),  "finalBody", main, 0);

    BasicBlock * exitBlock = BasicBlock::Create(mMod->getContext(), "exit", main, 0);

    Instance * s2pInstance = mS2PKernel->instantiate(inputStream);
    Instance * leadingInstance = mLeadingKernel->instantiate(s2pInstance->getOutputStreamSet());
    Instance * sortingInstance = mSortingKernel->instantiate(leadingInstance->getOutputStreamSet());

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
    iBuilder->CreateCondBr(leadingBlocksCond, safetyCheckBlock, regularTestBlock);

    iBuilder->SetInsertPoint(safetyCheckBlock);
    Value * safetyCheckCond = iBuilder->CreateICmpULT(remainingBytes, blockSize);
    iBuilder->CreateCondBr(safetyCheckCond, regularExitBlock, leadingBodyBlock);

    iBuilder->SetInsertPoint(leadingBodyBlock);
    s2pInstance->CreateDoBlockCall();
    leadingInstance->CreateDoBlockCall();
    blockNo->addIncoming(iBuilder->CreateAdd(blockNo, iBuilder->getInt64(1)), leadingBodyBlock);
    remainingBytes->addIncoming(iBuilder->CreateSub(remainingBytes, blockSize), leadingBodyBlock);
    iBuilder->CreateBr(leadingTestBlock);

    // Now all the data for which we can produce and consume a full leading block...
    iBuilder->SetInsertPoint(regularTestBlock);
    PHINode * remainingBytes2 = iBuilder->CreatePHI(intType, 2);
    remainingBytes2->addIncoming(remainingBytes, leadingTestBlock);
    Value * remainingBytesCond = iBuilder->CreateICmpULT(remainingBytes2, requiredBytes);
    iBuilder->CreateCondBr(remainingBytesCond, regularExitBlock, regularBodyBlock);
    iBuilder->SetInsertPoint(regularBodyBlock);
    s2pInstance->CreateDoBlockCall();
    leadingInstance->CreateDoBlockCall();
    sortingInstance->CreateDoBlockCall();
    remainingBytes2->addIncoming(iBuilder->CreateSub(remainingBytes2, blockSize), regularBodyBlock);
    iBuilder->CreateBr(regularTestBlock);

    // Check if we have a partial blocks worth of leading data remaining
    iBuilder->SetInsertPoint(regularExitBlock);
    PHINode * remainingBytes3 = iBuilder->CreatePHI(intType, 2);
    remainingBytes3->addIncoming(remainingBytes, safetyCheckBlock);
    remainingBytes3->addIncoming(remainingBytes2, regularTestBlock);
    Value * partialBlockCond = iBuilder->CreateICmpNE(remainingBytes3, ConstantInt::getNullValue(intType));
    iBuilder->CreateCondBr(partialBlockCond, finalTestBlock, partialBlock);

    // If we do, process it and mask out the data
    iBuilder->SetInsertPoint(partialBlock);
    s2pInstance->CreateDoBlockCall();
    leadingInstance->CreateDoBlockCall();
    leadingInstance->clearOutputStreamSet();
    sortingInstance->CreateDoBlockCall();
    iBuilder->CreateBr(finalTestBlock);

    // Now clear the leading data and test the final blocks
    iBuilder->SetInsertPoint(finalTestBlock);
    PHINode * remainingFullBlocks = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 3);
    remainingFullBlocks->addIncoming(iBuilder->getInt64(leadingBlocks), regularExitBlock);
    remainingFullBlocks->addIncoming(iBuilder->getInt64(leadingBlocks), partialBlock);
    Value * remainingFullBlocksCond = iBuilder->CreateICmpUGT(remainingFullBlocks, ConstantInt::getNullValue(intType));
    iBuilder->CreateCondBr(remainingFullBlocksCond, finalBodyBlock, exitBlock);

    iBuilder->SetInsertPoint(finalBodyBlock);
    leadingInstance->clearOutputStreamSet();
    sortingInstance->CreateDoBlockCall();
    remainingFullBlocks->addIncoming(iBuilder->CreateSub(remainingFullBlocks, iBuilder->getInt64(1)), finalBodyBlock);

    iBuilder->CreateBr(finalTestBlock);
    iBuilder->SetInsertPoint(exitBlock);
    iBuilder->CreateRetVoid();

    return main;
}

SymbolTableBuilder::~SymbolTableBuilder() {
    delete mS2PKernel;
    delete mLeadingKernel;
    delete mSortingKernel;
    delete mScannerKernel;
}


}
