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
    RE * cc = makeName(makeCC(makeCC(makeCC('a', 'z'), makeCC('A', 'Z')), makeCC('0', '9')));
    reCompiler.compileUnicodeNames(cc);
    PabloAST * const matches = reCompiler.compile(cc).stream;
    PabloBlock * const entry = function->getEntryBlock();
    PabloAST * const adv = entry->createAdvance(matches, 1);
    PabloAST * const starts = entry->createAnd(matches, entry->createNot(adv));
    PabloAST * const ends = entry->createAnd(adv, entry->createNot(matches));

    function->setResult(0, entry->createAssign("l.S", starts));
    function->setResult(1, entry->createAssign("l.E", ends));

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
        function->setResult(i + 2, entry->createAssign("l.M" + std::to_string(i), M));
        ++i;
        step += span;
    }

    return function;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateSortingFunction
 ** ------------------------------------------------------------------------------------------------------------- */
PabloFunction * SymbolTableBuilder::generateSortingFunction(const PabloFunction * const leading, const std::vector<unsigned> & endpoints) {
    PabloFunction * const function = PabloFunction::Create("sorting", leading->getNumOfResults(), (leading->getNumOfResults() - 1) * 2);
    PabloBlock * entry = function->getEntryBlock();
    function->setParameter(0, entry->createVar("l.S"));
    function->setParameter(1, entry->createVar("l.E"));
    for (unsigned i = 2; i < leading->getNumOfResults(); ++i) {
        function->setParameter(i, entry->createVar("l.M" + std::to_string(i - 2)));
    }
    PabloAST * R = function->getParameter(0);
    PabloAST * const E = entry->createNot(function->getParameter(1));
    unsigned i = 0;
    unsigned lowerbound = 0;
    for (unsigned endpoint : endpoints) {
        PabloAST * const M = function->getParameter(i + 2);
        PabloAST * const L = entry->createLookahead(M, endpoint, "lookahead" + std::to_string(endpoint));
        PabloAST * S = entry->createAnd(L, R);
        Assign * Si = entry->createAssign("s.S_" + std::to_string(i + 1), S);
        PabloAST * F = entry->createScanThru(S, E);
        Assign * Ei = entry->createAssign("s.E_" + std::to_string(i + 1), F);
        function->setResult(i * 2, Si);
        function->setResult(i * 2 + 1, Ei);
        R = entry->createXor(R, S);
        ++i;
        lowerbound = endpoint;
    }
    Assign * Si = entry->createAssign("s.S_n", R);
    PabloAST * F = entry->createScanThru(R, E);
    Assign * Ei = entry->createAssign("s.E_n", F);
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
    Function * const vgather = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::x86_avx2_gather_d_d_256);
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
 * @brief generateGatherKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void SymbolTableBuilder::generateGatherKernel(KernelBuilder * kBuilder, const std::vector<unsigned> & endpoints, const unsigned scanWordBitWidth) {

    Type * const intScanWordTy = iBuilder->getIntNTy(scanWordBitWidth);
    const unsigned fieldCount = iBuilder->getBitBlockWidth() / scanWordBitWidth;
    Type * const scanWordVectorType = VectorType::get(intScanWordTy, fieldCount);
    const unsigned vectorWidth = iBuilder->getBitBlockWidth() / 32;
    const unsigned gatherCount = vectorWidth * 4;

    Type * startArrayType = ArrayType::get(iBuilder->getInt32Ty(), iBuilder->getBitBlockWidth() + gatherCount);
    Type * endArrayType = ArrayType::get(iBuilder->getInt32Ty(), gatherCount);
    Type * groupType = StructType::get(iBuilder->getInt32Ty(), startArrayType, iBuilder->getInt32Ty(), endArrayType, nullptr);
    const unsigned baseIdx = kBuilder->addInternalState(iBuilder->getInt8PtrTy(), "Base");
    const unsigned gatherPositionArrayIdx = kBuilder->addInternalState(ArrayType::get(groupType, endpoints.size()), "Positions");

    for (unsigned maxKeyLength : endpoints) {
        kBuilder->addInputStream(1, "startStream" + std::to_string(maxKeyLength));
        kBuilder->addInputStream(1, "endStream" + std::to_string(maxKeyLength));
        kBuilder->addOutputStream(4); // ((maxKeyLength + 3) / 4) * 4
    }
    kBuilder->addInputStream(1, "startStreamN");
    kBuilder->addInputStream(1, "endStreamN");

    Function * const function = kBuilder->prepareFunction();

    BasicBlock * const entry = iBuilder->GetInsertBlock();

    BasicBlock * groupCond = BasicBlock::Create(mMod->getContext(), "groupCond", function, 0);
    BasicBlock * groupBody = BasicBlock::Create(mMod->getContext(), "groupBody", function, 0);

    BasicBlock * startOuterCond = BasicBlock::Create(mMod->getContext(), "startOuterCond", function, 0);
    BasicBlock * startOuterBody = BasicBlock::Create(mMod->getContext(), "startOuterBody", function, 0);
    BasicBlock * startInnerCond = BasicBlock::Create(mMod->getContext(), "startInnerCond", function, 0);
    BasicBlock * startInnerBody = BasicBlock::Create(mMod->getContext(), "startInnerBody", function, 0);

    BasicBlock * endOuterCond = BasicBlock::Create(mMod->getContext(), "endOuterCond", function, 0);
    BasicBlock * endOuterBody = BasicBlock::Create(mMod->getContext(), "endOuterBody", function, 0);
    BasicBlock * endInnerCond = BasicBlock::Create(mMod->getContext(), "endInnerCond", function, 0);
    BasicBlock * endInnerBody = BasicBlock::Create(mMod->getContext(), "endInnerBody", function, 0);

    BasicBlock * gather = BasicBlock::Create(mMod->getContext(), "gather", function, 0);

    BasicBlock * nextGroup = BasicBlock::Create(mMod->getContext(), "nextGroup", function, 0);

    BasicBlock * exit = BasicBlock::Create(mMod->getContext(), "exit", function, 0);


    // ENTRY BLOCK
    iBuilder->SetInsertPoint(entry);
    Type * const int32PtrTy = PointerType::get(iBuilder->getInt32Ty(), 0);
    FunctionType * const gatherFunctionType = FunctionType::get(iBuilder->getVoidTy(), {iBuilder->getInt8PtrTy(), int32PtrTy, int32PtrTy, iBuilder->getInt32Ty(), iBuilder->getInt8PtrTy()}, false);
    Value * const gatherFunctionPtrArray = iBuilder->CreateAlloca(PointerType::get(gatherFunctionType, 0), iBuilder->getInt32(endpoints.size()), "gatherFunctionPtrArray");

    unsigned i = 0;
    unsigned minKeyLength = 0;
    for (unsigned maxKeyLength : endpoints) {
        Function * f = generateGatherFunction(minKeyLength, maxKeyLength);
        mGatherFunction.push_back(f);
        iBuilder->CreateStore(f, iBuilder->CreateGEP(gatherFunctionPtrArray, iBuilder->getInt32(i++)));
        minKeyLength = maxKeyLength;
    }

    //TODO: this won't work on files > 2^32 bytes yet; needs an intermediate flush then a recalculation of the base pointer.
    Value * const base = iBuilder->CreateLoad(kBuilder->getInternalState(baseIdx), "base");
    Value * const positionArray = kBuilder->getInternalState(gatherPositionArrayIdx);

    Value * blockPos = iBuilder->CreateLoad(kBuilder->getBlockNo());
    blockPos = iBuilder->CreateMul(blockPos, iBuilder->getInt64(iBuilder->getBitBlockWidth()));

    iBuilder->CreateBr(groupCond);

    // GROUP COND
    iBuilder->SetInsertPoint(groupCond);
    PHINode * groupIV = iBuilder->CreatePHI(iBuilder->getInt32Ty(), 2);
    groupIV->addIncoming(iBuilder->getInt32(0), entry);
    Value * groupTest = iBuilder->CreateICmpNE(groupIV, iBuilder->getInt32(endpoints.size()));
    iBuilder->CreateCondBr(groupTest, groupBody, exit);

    // GROUP BODY
    iBuilder->SetInsertPoint(groupBody);
    // if two positions cannot be in the same vector element, we could possibly do some work in parallel here.

    Value * index = iBuilder->CreateMul(groupIV, iBuilder->getInt32(2));
    Value * startStreamPtr = kBuilder->getInputStream(index);
    Value * startStream = iBuilder->CreateBlockAlignedLoad(startStreamPtr);
    startStream = iBuilder->CreateBitCast(startStream, scanWordVectorType, "startStream");

    index = iBuilder->CreateAdd(index, iBuilder->getInt32(1));
    Value * endStreamPtr = kBuilder->getInputStream(index);
    Value * endStream = iBuilder->CreateBlockAlignedLoad(endStreamPtr);
    endStream = iBuilder->CreateBitCast(endStream, scanWordVectorType, "endStream");

    Value * startIndexPtr = iBuilder->CreateGEP(positionArray, {iBuilder->getInt32(0), groupIV, iBuilder->getInt32(0)});
    Value * startIndex = iBuilder->CreateLoad(startIndexPtr, "startIndex");
    Value * startPosArray = iBuilder->CreateGEP(positionArray, {iBuilder->getInt32(0), groupIV, iBuilder->getInt32(1)}, "startPosArray");
    Value * endIndexPtr = iBuilder->CreateGEP(positionArray, {iBuilder->getInt32(0), groupIV, iBuilder->getInt32(2)}, "endIndexPtr");
    Value * endIndex = iBuilder->CreateLoad(endIndexPtr, "endIndex");
    Value * endPosArray = iBuilder->CreateGEP(positionArray, {iBuilder->getInt32(0), groupIV, iBuilder->getInt32(3)}, "endPosArray");

    iBuilder->CreateBr(startOuterCond);

    // START OUTER COND
    iBuilder->SetInsertPoint(startOuterCond);
    PHINode * startBlockOffset = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 2);
    startBlockOffset->addIncoming(blockPos, groupBody);
    PHINode * startIndexPhi1 = iBuilder->CreatePHI(startIndex->getType(), 2, "startIndexPhi1");
    startIndexPhi1->addIncoming(startIndex, groupBody);
    PHINode * startIV = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 2);
    startIV->addIncoming(iBuilder->getInt64(0), groupBody);
    Value * startOuterTest = iBuilder->CreateICmpNE(startIV, iBuilder->getInt64(fieldCount));
    iBuilder->CreateCondBr(startOuterTest, startOuterBody, endOuterCond);

    // START OUTER BODY
    iBuilder->SetInsertPoint(startOuterBody);
    Value * startField = iBuilder->CreateExtractElement(startStream, startIV);
    startIV->addIncoming(iBuilder->CreateAdd(startIV, iBuilder->getInt64(1)), startInnerCond);
    startBlockOffset->addIncoming(iBuilder->CreateAdd(startBlockOffset, iBuilder->getInt64(scanWordBitWidth)), startInnerCond);
    iBuilder->CreateBr(startInnerCond);

    // START INNER COND
    iBuilder->SetInsertPoint(startInnerCond);
    PHINode * startIndexPhi2 = iBuilder->CreatePHI(startIndex->getType(), 2, "startIndexPhi2");
    startIndexPhi2->addIncoming(startIndexPhi1, startOuterBody);
    startIndexPhi1->addIncoming(startIndexPhi2, startInnerCond);
    PHINode * startFieldPhi = iBuilder->CreatePHI(intScanWordTy, 2);
    startFieldPhi->addIncoming(startField, startOuterBody);
    Value * test = iBuilder->CreateICmpNE(startFieldPhi, ConstantInt::getNullValue(intScanWordTy));
    iBuilder->CreateCondBr(test, startInnerBody, startOuterCond);

    // START INNER BODY
    iBuilder->SetInsertPoint(startInnerBody);
    Value * startPos = generateCountForwardZeroes(iBuilder, startFieldPhi);
    startFieldPhi->addIncoming(generateResetLowestBit(iBuilder, startFieldPhi), startInnerBody);
    startPos = iBuilder->CreateTruncOrBitCast(iBuilder->CreateOr(startPos, startBlockOffset), iBuilder->getInt32Ty());
    iBuilder->CreateStore(startPos, iBuilder->CreateGEP(startPosArray, {iBuilder->getInt32(0), startIndexPhi2}));
    startIndexPhi2->addIncoming(iBuilder->CreateAdd(startIndexPhi2, ConstantInt::get(startIndexPhi2->getType(), 1)), startInnerBody);
    iBuilder->CreateBr(startInnerCond);

    // END POINT OUTER COND
    iBuilder->SetInsertPoint(endOuterCond);
    PHINode * endBlockOffset = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 2);
    endBlockOffset->addIncoming(blockPos, startOuterCond);
    PHINode * endIndexPhi1 = iBuilder->CreatePHI(endIndex->getType(), 2);
    endIndexPhi1->addIncoming(endIndex, startOuterCond);
    PHINode * startIndexPhi3 = iBuilder->CreatePHI(startIndex->getType(), 2, "startIndexPhi3");
    startIndexPhi3->addIncoming(startIndexPhi1, startOuterCond);
    PHINode * endIV = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 2);
    endIV->addIncoming(iBuilder->getInt64(0), startOuterCond);
    Value * endOuterTest = iBuilder->CreateICmpNE(endIV, iBuilder->getInt64(fieldCount));
    iBuilder->CreateCondBr(endOuterTest, endOuterBody, nextGroup);

    // END POINT OUTER BODY
    iBuilder->SetInsertPoint(endOuterBody);
    Value * endField = iBuilder->CreateExtractElement(endStream, endIV);
    endIV->addIncoming(iBuilder->CreateAdd(endIV, iBuilder->getInt64(1)), endInnerCond);
    endBlockOffset->addIncoming(iBuilder->CreateAdd(endBlockOffset, iBuilder->getInt64(scanWordBitWidth)), endInnerCond);
    iBuilder->CreateBr(endInnerCond);

    // END POINT INNER COND
    iBuilder->SetInsertPoint(endInnerCond);
    PHINode * startIndexPhi4 = iBuilder->CreatePHI(startIndexPhi3->getType(), 3, "startIndexPhi4");
    startIndexPhi4->addIncoming(startIndexPhi3, endOuterBody);
    startIndexPhi4->addIncoming(startIndexPhi4, endInnerBody);
    startIndexPhi3->addIncoming(startIndexPhi4, endInnerCond);
    PHINode * endIndexPhi2 = iBuilder->CreatePHI(endIndex->getType(), 3);
    endIndexPhi2->addIncoming(endIndexPhi1, endOuterBody);
    endIndexPhi1->addIncoming(endIndexPhi2, endInnerCond);
    endIndexPhi2->addIncoming(ConstantInt::getNullValue(endIndex->getType()), gather);
    PHINode * endFieldPhi = iBuilder->CreatePHI(intScanWordTy, 3);
    endFieldPhi->addIncoming(endField, endOuterBody);
    Value * endInnerTest = iBuilder->CreateICmpNE(endFieldPhi, ConstantInt::getNullValue(intScanWordTy));
    iBuilder->CreateCondBr(endInnerTest, endInnerBody, endOuterCond);

    // END POINT INNER BODY
    iBuilder->SetInsertPoint(endInnerBody);
    Value * endPos = generateCountForwardZeroes(iBuilder, endFieldPhi);
    Value * updatedEndFieldPhi = generateResetLowestBit(iBuilder, endFieldPhi);
    endFieldPhi->addIncoming(updatedEndFieldPhi, endInnerBody);
    endFieldPhi->addIncoming(updatedEndFieldPhi, gather);
    endPos = iBuilder->CreateTruncOrBitCast(iBuilder->CreateOr(endPos, endBlockOffset), iBuilder->getInt32Ty());
    iBuilder->CreateStore(endPos, iBuilder->CreateGEP(endPosArray, {iBuilder->getInt32(0), endIndexPhi2}));
    Value * updatedEndIndexPhi = iBuilder->CreateAdd(endIndexPhi2, ConstantInt::get(endIndexPhi2->getType(), 1));
    endIndexPhi2->addIncoming(updatedEndIndexPhi, endInnerBody);
    Value * filledEndPosBufferTest = iBuilder->CreateICmpEQ(updatedEndIndexPhi, ConstantInt::get(updatedEndIndexPhi->getType(), gatherCount));
    iBuilder->CreateCondBr(filledEndPosBufferTest, gather, endInnerCond);

    // GATHER
    iBuilder->SetInsertPoint(gather);

    Value * startArrayPtr = iBuilder->CreatePointerCast(startPosArray, PointerType::get(iBuilder->getInt32Ty(), 0));
    Value * endArrayPtr = iBuilder->CreatePointerCast(endPosArray, PointerType::get(iBuilder->getInt32Ty(), 0));
    Value * gatherFunctionPtr = iBuilder->CreateLoad(iBuilder->CreateGEP(gatherFunctionPtrArray, groupIV));
    Value * outputBuffer = iBuilder->CreatePointerCast(kBuilder->getOutputStream(groupIV), iBuilder->getInt8PtrTy());
    iBuilder->CreateCall5(gatherFunctionPtr, base, startArrayPtr, endArrayPtr, iBuilder->getInt32(32), outputBuffer);
    // Copy the unused start positions to the front of the start position array and adjust the start index
    Value * remainingArrayPtr = iBuilder->CreateGEP(startArrayPtr, iBuilder->getInt32(gatherCount));
    Value * remainingCount = iBuilder->CreateSub(startIndexPhi4, iBuilder->getInt32(gatherCount));
    Value * remainingBytes = iBuilder->CreateMul(remainingCount, iBuilder->getInt32(4));
    iBuilder->CreateMemMove(startArrayPtr, remainingArrayPtr, remainingBytes, 4);
    startIndexPhi4->addIncoming(remainingCount, gather);
    iBuilder->CreateBr(endInnerCond);

    // NEXT GROUP
    iBuilder->SetInsertPoint(nextGroup);
    iBuilder->CreateStore(startIndexPhi3, startIndexPtr);
    iBuilder->CreateStore(endIndexPhi1, endIndexPtr);
    groupIV->addIncoming(iBuilder->CreateAdd(groupIV, ConstantInt::get(groupIV->getType(), 1)), nextGroup);
    iBuilder->CreateBr(groupCond);

    iBuilder->SetInsertPoint(exit);
    kBuilder->finalize();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateGatherFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * SymbolTableBuilder::generateGatherFunction(const unsigned minKeyLength, const unsigned maxKeyLength) {

    assert (minKeyLength < maxKeyLength);

    const std::string functionName = "gather_" + std::to_string(minKeyLength) + "_to_" + std::to_string(maxKeyLength);
    Function * function = mMod->getFunction(functionName);
    if (function == nullptr) {

        const auto ip = iBuilder->saveIP();

        const unsigned minCount = (minKeyLength / 4);
        const unsigned maxCount = ((maxKeyLength + 3) / 4);

        const unsigned vectorWidth = iBuilder->getBitBlockWidth() / 32;
        Type * const gatherVectorType =  VectorType::get(iBuilder->getInt32Ty(), vectorWidth);
        const unsigned gatherByteWidth = gatherVectorType->getPrimitiveSizeInBits() / 8;
        Type * const transposedVectorType = VectorType::get(iBuilder->getInt8Ty(), iBuilder->getBitBlockWidth() / 8);
        const unsigned transposedByteWidth = transposedVectorType->getPrimitiveSizeInBits() / 8;


        Type * const int32PtrTy = PointerType::get(iBuilder->getInt32Ty(), 0);
        FunctionType * const functionType = FunctionType::get(iBuilder->getVoidTy(), {iBuilder->getInt8PtrTy(), int32PtrTy, int32PtrTy, iBuilder->getInt32Ty(), iBuilder->getInt8PtrTy()}, false);
        function = Function::Create(functionType, GlobalValue::ExternalLinkage, functionName, mMod);
        function->setCallingConv(CallingConv::C);
        function->setDoesNotCapture(1);
        function->setDoesNotCapture(2);
        function->setDoesNotCapture(3);
        function->setDoesNotThrow();

        Function::arg_iterator args = function->arg_begin();
        Value * const base = args++;
        base->setName("base");
        Value * startArray = args++;
        startArray->setName("startArray");
        Value * endArray = args++;
        endArray->setName("endArray");
        Value * const numOfKeys = args++;
        numOfKeys->setName("numOfKeys");
        Value * result = args++;
        result->setName("result");

        BasicBlock * entry = BasicBlock::Create(mMod->getContext(), "entry", function, 0);
        BasicBlock * gatherCond = BasicBlock::Create(mMod->getContext(), "gatherCond", function, 0);
        BasicBlock * partialGatherCond = BasicBlock::Create(mMod->getContext(), "partialGatherCond", function, 0);
        BasicBlock * partialGatherBody = BasicBlock::Create(mMod->getContext(), "partialGatherBody", function, 0);
        BasicBlock * gatherBody = BasicBlock::Create(mMod->getContext(), "gatherBody", function, 0);
        BasicBlock * transposeCond = BasicBlock::Create(mMod->getContext(), "transposeCond", function, 0);
        BasicBlock * transposeBody = BasicBlock::Create(mMod->getContext(), "transposeBody", function, 0);
        BasicBlock * exit = BasicBlock::Create(mMod->getContext(), "exit", function, 0);

        Value * const four = iBuilder->CreateVectorSplat(vectorWidth, iBuilder->getInt32(4));

        // ENTRY
        iBuilder->SetInsertPoint(entry);

        AllocaInst * const buffer = iBuilder->CreateAlloca(gatherVectorType, iBuilder->getInt32(maxCount * 4), "buffer");
        Value * end = iBuilder->CreateGEP(buffer, {iBuilder->getInt32(maxCount * 4)});
        Value * size = iBuilder->CreateSub(iBuilder->CreatePtrToInt(end, iBuilder->getInt64Ty()), iBuilder->CreatePtrToInt(buffer, iBuilder->getInt64Ty()));
        iBuilder->CreateMemSet(buffer, iBuilder->getInt8(0), size, 4);
        Value * const transposed = iBuilder->CreateBitCast(buffer, transposedVectorType->getPointerTo(), "transposed");

        startArray = iBuilder->CreateBitCast(startArray, gatherVectorType->getPointerTo());
        endArray = iBuilder->CreateBitCast(endArray, gatherVectorType->getPointerTo());

        iBuilder->CallPrintInt(functionName + ".numOfKeys", numOfKeys);

        iBuilder->CreateBr(gatherCond);

        // FULL GATHER COND
        iBuilder->SetInsertPoint(gatherCond);
        PHINode * remainingLanes = iBuilder->CreatePHI(iBuilder->getInt32Ty(), 2);
        remainingLanes->addIncoming(numOfKeys, entry);

        PHINode * gatherIV = iBuilder->CreatePHI(iBuilder->getInt32Ty(), 2);
        gatherIV->addIncoming(iBuilder->getInt32(0), entry);

        Value * gatherLoopTest = iBuilder->CreateICmpSGE(remainingLanes, iBuilder->getInt32(vectorWidth));
        iBuilder->CreateCondBr(gatherLoopTest, gatherBody, partialGatherCond);

        // PARTIAL GATHER COND
        iBuilder->SetInsertPoint(partialGatherCond);
        Value * partialGatherLoopTest = iBuilder->CreateICmpSLE(remainingLanes, iBuilder->getInt32(0));
        iBuilder->CreateCondBr(partialGatherLoopTest, transposeCond, partialGatherBody);

        // PARTIAL GATHER BODY
        iBuilder->SetInsertPoint(partialGatherBody);
        Type * registerType = iBuilder->getIntNTy(iBuilder->getBitBlockWidth());
        Value * maskedLanes = iBuilder->CreateSub(iBuilder->getInt32(vectorWidth), remainingLanes);       
        maskedLanes = iBuilder->CreateMul(maskedLanes, iBuilder->getInt32(32));
        maskedLanes = iBuilder->CreateZExt(maskedLanes, registerType);
        maskedLanes = iBuilder->CreateLShr(Constant::getAllOnesValue(registerType), maskedLanes);
        maskedLanes = iBuilder->CreateBitCast(maskedLanes, gatherVectorType);
        iBuilder->CreateBr(gatherBody);

        // FULL GATHER BODY
        iBuilder->SetInsertPoint(gatherBody);
        PHINode * activeLanes = iBuilder->CreatePHI(gatherVectorType, 2, "activeLanes");
        activeLanes->addIncoming(Constant::getAllOnesValue(gatherVectorType), gatherCond);
        activeLanes->addIncoming(maskedLanes, partialGatherBody);


        Value * startPos = iBuilder->CreateAlignedLoad(iBuilder->CreateGEP(startArray, gatherIV), 4);
        Value * const endPos = iBuilder->CreateAlignedLoad(iBuilder->CreateGEP(endArray, gatherIV), 4);

        for (unsigned blockCount = 0; blockCount < minCount; ++blockCount) {
            Value * tokenData = generateMaskedGather(base, startPos, activeLanes);
            Value * ptr = iBuilder->CreateGEP(buffer, iBuilder->CreateOr(gatherIV, iBuilder->getInt32(blockCount * 4)));
            iBuilder->CreateAlignedStore(tokenData, ptr, transposedByteWidth);
            startPos = iBuilder->CreateAdd(startPos, four);
        }

        for (unsigned blockCount = minCount; blockCount < maxCount; ++blockCount) {

            // if we have not fully gathered the data for this key
            Value * atLeastOneByte = iBuilder->CreateSExt(iBuilder->CreateICmpSLT(startPos, endPos), startPos->getType());
            atLeastOneByte = iBuilder->CreateAnd(atLeastOneByte, activeLanes, "atLeastOneByte");

            // gather it ...
            Value * tokenData = generateMaskedGather(base, startPos, atLeastOneByte);

            // and compute how much data is remaining.
            Value * remaining = iBuilder->CreateSub(endPos, startPos);

            // if this token has at least 4 bytes remaining ...
            Value * atLeastFourBytes = iBuilder->CreateSExt(iBuilder->CreateICmpUGE(remaining, four), remaining->getType(), "atLeastFourBytes");

            // determine how many bits do *not* belong to the token
            remaining = iBuilder->CreateSub(four, remaining);
            remaining = iBuilder->CreateShl(remaining, ConstantInt::get(remaining->getType(), 3));

            // then mask them out prior to storing the value
            Value * partialTokenMask = iBuilder->CreateLShr(ConstantInt::getAllOnesValue(remaining->getType()), remaining);
            partialTokenMask = iBuilder->CreateOr(partialTokenMask, atLeastFourBytes);
            tokenData = iBuilder->CreateAnd(partialTokenMask, tokenData);
            Value * ptr = iBuilder->CreateGEP(buffer, iBuilder->CreateOr(gatherIV, iBuilder->getInt32(blockCount * 4)));
            iBuilder->CreateAlignedStore(tokenData, ptr, transposedByteWidth);

            startPos = iBuilder->CreateAdd(startPos, four);
        }

        gatherIV->addIncoming(iBuilder->CreateAdd(gatherIV, iBuilder->getInt32(1)), gatherBody);
        remainingLanes->addIncoming(iBuilder->CreateSub(remainingLanes, iBuilder->getInt32(vectorWidth)), gatherBody);
        iBuilder->CreateBr(gatherCond);

        // TRANSPOSE COND
        iBuilder->SetInsertPoint(transposeCond);
        PHINode * transposeIV = iBuilder->CreatePHI(iBuilder->getInt32Ty(), 2);
        transposeIV->addIncoming(iBuilder->getInt32(0), partialGatherCond);
        Value * transposeLoopTest = iBuilder->CreateICmpNE(transposeIV, iBuilder->getInt32(maxCount));
        iBuilder->CreateCondBr(transposeLoopTest, transposeBody, exit);

        // TRANSPOSE BODY
        iBuilder->SetInsertPoint(transposeBody);

        Value * offset = iBuilder->CreateMul(transposeIV, iBuilder->getInt32(4));

        Value * value[4];
        for (unsigned i = 0; i < 4; ++i) {
            Value * const ptr = iBuilder->CreateGEP(buffer, iBuilder->CreateAdd(offset, iBuilder->getInt32(i)));
            value[i] = iBuilder->CreateLoad(ptr);
        }

        for (unsigned byteWidth = 2; byteWidth; --byteWidth) {
            const unsigned fieldWidth = (byteWidth * 8);
            const unsigned fieldCount = iBuilder->getBitBlockWidth() / fieldWidth;
            VectorType * const type = VectorType::get(Type::getIntNTy(iBuilder->getContext(), fieldWidth), fieldCount);
            std::vector<Constant *> even(fieldCount);
            std::vector<Constant *> odd(fieldCount);
            for (unsigned j = 0; j < fieldCount; ++j) {
                even[j] = iBuilder->getInt32(j * 2);
                odd[j] = iBuilder->getInt32(j * 2 + 1);
            }
            Constant * const evenVector = ConstantVector::get(even);
            Constant * const oddVector = ConstantVector::get(odd);
            Value * result[4];
            for (unsigned i = 0; i < 4; i += 2) {
                value[i] = iBuilder->CreateBitCast(value[i], type);
                value[i + 1] = iBuilder->CreateBitCast(value[i + 1], type);
                result[(i / byteWidth)] = iBuilder->CreateShuffleVector(value[i], value[i + 1], evenVector);
                result[(i / byteWidth) + byteWidth] = iBuilder->CreateShuffleVector(value[i], value[i + 1], oddVector);
            }
            for (unsigned i = 0; i < 4; ++i) {
                value[i] = result[i];
            }
        }

        for (unsigned i = 0; i < 4; ++i) {
            Value * ptr = iBuilder->CreateGEP(transposed, iBuilder->CreateAdd(offset, iBuilder->getInt32(i)));
            iBuilder->CreateAlignedStore(value[i], ptr, gatherByteWidth);
        }

        transposeIV->addIncoming(iBuilder->CreateAdd(transposeIV, iBuilder->getInt32(1)), transposeBody);
        iBuilder->CreateBr(transposeCond);

        // EXIT
        iBuilder->SetInsertPoint(exit);

        // ... call hashing function ...

        for (unsigned i = 0; i < maxKeyLength; ++i) {
            Value * ptr = iBuilder->CreateGEP(transposed, iBuilder->getInt32(i));
            Value * value = iBuilder->CreateAlignedLoad(ptr, gatherByteWidth);
            iBuilder->CallPrintRegister(functionName + ".output" + std::to_string(i), value);
        }

        iBuilder->CreateRetVoid();

        iBuilder->restoreIP(ip);
    }

    return function;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createKernels
 ** ------------------------------------------------------------------------------------------------------------- */
void SymbolTableBuilder::createKernels() {

    std::vector<unsigned> endpoints;
    endpoints.push_back(8);
    endpoints.push_back(17);
    endpoints.push_back(27);
    endpoints.push_back(39);
    endpoints.push_back(77);
    endpoints.push_back(124);
    endpoints.push_back(178);
    endpoints.push_back(278);

    PabloCompiler pablo_compiler(mMod, iBuilder);
    PabloFunction * const leading = generateLeadingFunction(endpoints);
    PabloFunction * const sorting = generateSortingFunction(leading, endpoints);

    const auto bufferSize = ((mLongestLookahead + iBuilder->getBitBlockWidth() - 1) / iBuilder->getBitBlockWidth()) + 1;

    mS2PKernel = new KernelBuilder(iBuilder, "s2p", 1);
    mLeadingKernel = new KernelBuilder(iBuilder, "leading", bufferSize);
    mSortingKernel = new KernelBuilder(iBuilder, "sorting", bufferSize);
    mGatherKernel = new KernelBuilder(iBuilder, "gathering", 1);

    generateS2PKernel(mMod, iBuilder, mS2PKernel);

    pablo_compiler.setKernel(mLeadingKernel);
    pablo_compiler.compile(leading);
    pablo_compiler.setKernel(mSortingKernel);
    pablo_compiler.compile(sorting);

    delete leading;
    delete sorting;

    releaseSlabAllocatorMemory();

    generateGatherKernel(mGatherKernel, endpoints, 64);
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

    BasicBlock * leadingBlock = BasicBlock::Create(mMod->getContext(), "leadingBody", main, 0);

    BasicBlock * partialLeadingCond = BasicBlock::Create(mMod->getContext(), "partialLeadingCond", main, 0);
    BasicBlock * partialLeadingBody = BasicBlock::Create(mMod->getContext(), "partialLeadingBody", main, 0);

    BasicBlock * regularCondBlock = BasicBlock::Create(mMod->getContext(), "regularCond", main, 0);
    BasicBlock * regularBodyBlock = BasicBlock::Create(mMod->getContext(), "regularBody", main, 0);

    BasicBlock * partialCondBlock = BasicBlock::Create(mMod->getContext(), "partialCond", main, 0);
    BasicBlock * partialBodyBlock = BasicBlock::Create(mMod->getContext(),  "partialBody", main, 0);

    BasicBlock * flushLengthGroupsBlock = BasicBlock::Create(mMod->getContext(), "flushLengthGroups", main, 0);

    Instance * s2pInstance = mS2PKernel->instantiate(inputStream);
    Instance * leadingInstance = mLeadingKernel->instantiate(s2pInstance->getOutputStreamBuffer());
    Instance * sortingInstance = mSortingKernel->instantiate(leadingInstance->getOutputStreamBuffer());
    Instance * gatheringInstance = mGatherKernel->instantiate(sortingInstance->getOutputStreamBuffer());

    gatheringInstance->setInternalState("Base", iBuilder->CreateBitCast(inputStream, iBuilder->getInt8PtrTy()));

    const unsigned leadingBlocks = (mLongestLookahead + iBuilder->getBitBlockWidth() - 1) / iBuilder->getBitBlockWidth();

    Value * const requiredBytes = iBuilder->getInt64(mBlockSize * leadingBlocks);
    Value * const blockSize = iBuilder->getInt64(mBlockSize);

    // First compute any necessary leading blocks to allow the sorting kernel access to the "future" data produced by
    // the leading kernel ...

    Value * enoughDataForLookaheadCond = iBuilder->CreateICmpUGE(bufferSize, requiredBytes);
    iBuilder->CreateCondBr(enoughDataForLookaheadCond, leadingBlock, partialLeadingCond);

    iBuilder->SetInsertPoint(leadingBlock);
    for (unsigned i = 0; i < leadingBlocks; ++i) {
        s2pInstance->CreateDoBlockCall();
        leadingInstance->CreateDoBlockCall();
    }
    iBuilder->CreateBr(regularCondBlock);

    iBuilder->SetInsertPoint(partialLeadingCond);
    PHINode * remainingBytes1 = iBuilder->CreatePHI(intType, 2);
    remainingBytes1->addIncoming(bufferSize, entryBlock);
    Value * remainingCond = iBuilder->CreateICmpUGT(remainingBytes1, blockSize);
    iBuilder->CreateCondBr(remainingCond, partialLeadingBody, partialCondBlock);

    iBuilder->SetInsertPoint(partialLeadingBody);
    s2pInstance->CreateDoBlockCall();
    leadingInstance->CreateDoBlockCall();
    remainingBytes1->addIncoming(iBuilder->CreateSub(remainingBytes1, blockSize), partialLeadingBody);
    iBuilder->CreateBr(partialLeadingCond);

    // Now all the data for which we can produce and consume a full leading block...
    iBuilder->SetInsertPoint(regularCondBlock);
    PHINode * remainingBytes2 = iBuilder->CreatePHI(intType, 2);
    remainingBytes2->addIncoming(bufferSize, leadingBlock);
    Value * remainingBytesCond = iBuilder->CreateICmpUGT(remainingBytes2, requiredBytes);
    iBuilder->CreateCondBr(remainingBytesCond, regularBodyBlock, partialCondBlock);

    iBuilder->SetInsertPoint(regularBodyBlock);
    s2pInstance->CreateDoBlockCall();
    leadingInstance->CreateDoBlockCall();
    sortingInstance->CreateDoBlockCall();
    gatheringInstance->CreateDoBlockCall();
    remainingBytes2->addIncoming(iBuilder->CreateSub(remainingBytes2, blockSize), regularBodyBlock);
    iBuilder->CreateBr(regularCondBlock);

    // Check if we have a partial blocks worth of leading data remaining
    iBuilder->SetInsertPoint(partialCondBlock);
    PHINode * remainingBytes3 = iBuilder->CreatePHI(intType, 3);
    remainingBytes3->addIncoming(bufferSize, partialLeadingCond);
    remainingBytes3->addIncoming(remainingBytes2, regularCondBlock);
    Value * partialBlockCond = iBuilder->CreateICmpSGT(remainingBytes3, iBuilder->getInt64(0));
    iBuilder->CreateCondBr(partialBlockCond, partialBodyBlock, flushLengthGroupsBlock);

    // If we do, process it and mask out the data
    iBuilder->SetInsertPoint(partialBodyBlock);
    s2pInstance->clearOutputStreamSet();
    leadingInstance->CreateDoBlockCall();    
    sortingInstance->CreateDoBlockCall();
    gatheringInstance->CreateDoBlockCall();
    remainingBytes3->addIncoming(iBuilder->CreateSub(remainingBytes3, blockSize), partialBodyBlock);
    iBuilder->CreateBr(partialCondBlock);

    // perform a final partial gather on all length groups ...
    iBuilder->SetInsertPoint(flushLengthGroupsBlock);

    Value * const base = iBuilder->CreateLoad(gatheringInstance->getInternalState("Base"));
    Value * positionArray = gatheringInstance->getInternalState("Positions");

    for (unsigned i = 0; i < mGatherFunction.size(); ++i) {
        BasicBlock * nonEmptyGroup = BasicBlock::Create(mMod->getContext(), "flushLengthGroup" + std::to_string(i), main, 0);

        BasicBlock * nextNonEmptyGroup = BasicBlock::Create(mMod->getContext(), "", main, 0);

        ConstantInt * groupIV = iBuilder->getInt32(i);
        Value * startIndexPtr = iBuilder->CreateGEP(positionArray, {iBuilder->getInt32(0), groupIV, iBuilder->getInt32(0)}, "startIndexPtr");
        Value * startIndex = iBuilder->CreateLoad(startIndexPtr, "remaining");
        Value * cond = iBuilder->CreateICmpNE(startIndex, ConstantInt::getNullValue(startIndex->getType()));
        iBuilder->CreateCondBr(cond, nonEmptyGroup, nextNonEmptyGroup);

        iBuilder->SetInsertPoint(nonEmptyGroup);
        Value * startArray = iBuilder->CreateGEP(positionArray, {iBuilder->getInt32(0), groupIV, iBuilder->getInt32(1)}, "startArray");
        Value * startArrayPtr = iBuilder->CreatePointerCast(startArray, PointerType::get(iBuilder->getInt32Ty(), 0));
        Value * endArray = iBuilder->CreateGEP(positionArray, {iBuilder->getInt32(0), groupIV, iBuilder->getInt32(3)}, "endArray");
        Value * endArrayPtr = iBuilder->CreatePointerCast(endArray, PointerType::get(iBuilder->getInt32Ty(), 0));
        Value * outputBuffer = iBuilder->CreatePointerCast(gatheringInstance->getOutputStream(groupIV), iBuilder->getInt8PtrTy());
        iBuilder->CreateCall5(mGatherFunction.at(i), base, startArrayPtr, endArrayPtr, startIndex, outputBuffer);
        iBuilder->CreateBr(nextNonEmptyGroup);

        iBuilder->SetInsertPoint(nextNonEmptyGroup);
    }
    iBuilder->CreateRetVoid();

    return main;
}

SymbolTableBuilder::~SymbolTableBuilder() {
    delete mS2PKernel;
    delete mLeadingKernel;
    delete mSortingKernel;
    delete mGatherKernel;
}


}
