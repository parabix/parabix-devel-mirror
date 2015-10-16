/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "idisa_builder.h"
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Function.h>

namespace IDISA {

Value * IDISA_Builder::bitBlockCast(Value * a) {
    return a->getType() == mBitBlockType ? a : mLLVMBuilder->CreateBitCast(a, mBitBlockType);
}

VectorType * IDISA_Builder::fwVectorType(unsigned fw) {
    int fieldCount = mBitBlockWidth/fw;
    return VectorType::get(mLLVMBuilder->getIntNTy(fw), fieldCount);
}

Value * IDISA_Builder::fwCast(unsigned fw, Value * a) {
    return a->getType() == fwVectorType(fw) ? a : mLLVMBuilder->CreateBitCast(a, fwVectorType(fw));
}

Value * IDISA_Builder::simd_add(unsigned fw, Value * a, Value * b) {
    return mLLVMBuilder->CreateAdd(fwCast(fw, a), fwCast(fw, b));
}

Value * IDISA_Builder::simd_sub(unsigned fw, Value * a, Value * b) {
    return mLLVMBuilder->CreateSub(fwCast(fw, a), fwCast(fw, b));
}

Value * IDISA_Builder::simd_mult(unsigned fw, Value * a, Value * b) {
    return mLLVMBuilder->CreateMul(fwCast(fw, a), fwCast(fw, b));
}

Value * IDISA_Builder::simd_eq(unsigned fw, Value * a, Value * b) {
    return mLLVMBuilder->CreateSExt(mLLVMBuilder->CreateICmpEQ(fwCast(fw, a), fwCast(fw, b)), fwVectorType(fw));
}

Value * IDISA_Builder::simd_gt(unsigned fw, Value * a, Value * b) {
    return mLLVMBuilder->CreateSExt(mLLVMBuilder->CreateICmpSGT(fwCast(fw, a), fwCast(fw, b)), fwVectorType(fw));
}

Value * IDISA_Builder::simd_ugt(unsigned fw, Value * a, Value * b) {
    return mLLVMBuilder->CreateSExt(mLLVMBuilder->CreateICmpUGT(fwCast(fw, a), fwCast(fw, b)), fwVectorType(fw));
}

Value * IDISA_Builder::simd_lt(unsigned fw, Value * a, Value * b) {
    return mLLVMBuilder->CreateSExt(mLLVMBuilder->CreateICmpSLT(fwCast(fw, a), fwCast(fw, b)), fwVectorType(fw));
}

Value * IDISA_Builder::simd_ult(unsigned fw, Value * a, Value * b) {
    return mLLVMBuilder->CreateSExt(mLLVMBuilder->CreateICmpULT(fwCast(fw, a), fwCast(fw, b)), fwVectorType(fw));
}

Value * IDISA_Builder::simd_max(unsigned fw, Value * a, Value * b) {
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    return mLLVMBuilder->CreateSelect(mLLVMBuilder->CreateICmpSGT(aVec, bVec), aVec, bVec);
}

Value * IDISA_Builder::simd_umax(unsigned fw, Value * a, Value * b) {
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    return mLLVMBuilder->CreateSelect(mLLVMBuilder->CreateICmpUGT(aVec, bVec), aVec, bVec);
}

Value * IDISA_Builder::simd_min(unsigned fw, Value * a, Value * b) {
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    return mLLVMBuilder->CreateSelect(mLLVMBuilder->CreateICmpSLT(aVec, bVec), aVec, bVec);
}

Value * IDISA_Builder::simd_umin(unsigned fw, Value * a, Value * b) {
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    return mLLVMBuilder->CreateSelect(mLLVMBuilder->CreateICmpULT(aVec, bVec), aVec, bVec);
}

Value * IDISA_Builder::simd_slli(unsigned fw, Value * a, unsigned shift) {
    return mLLVMBuilder->CreateShl(fwCast(fw, a), shift);
}

Value * IDISA_Builder::simd_srli(unsigned fw, Value * a, unsigned shift) {
    return mLLVMBuilder->CreateLShr(fwCast(fw, a), shift);
}

Value * IDISA_Builder::simd_srai(unsigned fw, Value * a, unsigned shift) {
    return mLLVMBuilder->CreateAShr(fwCast(fw, a), shift);
}

Value * IDISA_Builder::simd_cttz(unsigned fw, Value * a) {
    Value * cttzFunc = Intrinsic::getDeclaration(mMod, Intrinsic::cttz, fwVectorType(fw));
    Value * rslt = mLLVMBuilder->CreateCall(cttzFunc, std::vector<Value *>({fwCast(fw, a), ConstantInt::get(mLLVMBuilder->getInt1Ty(), 0)}));
    return rslt;
}

Value * IDISA_Builder::simd_popcount(unsigned fw, Value * a) {
    Value * ctpopFunc = Intrinsic::getDeclaration(mMod, Intrinsic::ctpop, fwVectorType(fw));
    Value * rslt = mLLVMBuilder->CreateCall(ctpopFunc, std::vector<Value *>({fwCast(fw, a)}));
    return rslt;
}

Value * IDISA_Builder::esimd_mergeh(unsigned fw, Value * a, Value * b) {
    unsigned field_count = mBitBlockWidth/fw;
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    std::vector<Constant*> Idxs;
    for (unsigned i = field_count/2; i < field_count; i++) {
        Idxs.push_back(mLLVMBuilder->getInt32(i));    // selects elements from first reg.
        Idxs.push_back(mLLVMBuilder->getInt32(i + field_count)); // selects elements from second reg.
    }
    return mLLVMBuilder->CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs));
}

Value * IDISA_Builder::esimd_mergel(unsigned fw, Value * a, Value * b) {
    unsigned field_count = mBitBlockWidth/fw;
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    std::vector<Constant*> Idxs;
    for (unsigned i = 0; i < field_count/2; i++) {
        Idxs.push_back(mLLVMBuilder->getInt32(i));    // selects elements from first reg.
        Idxs.push_back(mLLVMBuilder->getInt32(i + field_count)); // selects elements from second reg.
    }
    return mLLVMBuilder->CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs));
}

Value * IDISA_Builder::hsimd_packh(unsigned fw, Value * a, Value * b) {
    unsigned field_count = 2 * mBitBlockWidth/fw;
    Value * aVec = fwCast(fw/2, a);
    Value * bVec = fwCast(fw/2, b);
    std::vector<Constant*> Idxs;
    for (unsigned i = 0; i < field_count; i++) {
        Idxs.push_back(mLLVMBuilder->getInt32(2*i));
    }
    return mLLVMBuilder->CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs));
}

Value * IDISA_Builder::hsimd_packl(unsigned fw, Value * a, Value * b) {
    unsigned field_count = 2 * mBitBlockWidth/fw;
    Value * aVec = fwCast(fw/2, a);
    Value * bVec = fwCast(fw/2, b);
    std::vector<Constant*> Idxs;
    for (unsigned i = 0; i < field_count; i++) {
        Idxs.push_back(mLLVMBuilder->getInt32(2*i+1));
    }
    return mLLVMBuilder->CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs));
}

Value * IDISA_Builder::hsimd_signmask(unsigned fw, Value * a) {
    Value * mask = mLLVMBuilder->CreateICmpSLT(fwCast(fw, a), ConstantAggregateZero::get(fwVectorType(fw)));
    return mLLVMBuilder->CreateBitCast(mask, mLLVMBuilder->getIntNTy(mBitBlockWidth/fw));
}

Value * IDISA_Builder::mvmd_extract(unsigned fw, Value * a, unsigned fieldIndex) {
    Value * aVec = fwCast(fw, a);
    return mLLVMBuilder->CreateExtractElement(aVec, mLLVMBuilder->getInt32(fieldIndex));
}

Value * IDISA_Builder::mvmd_dslli(unsigned fw, Value * a, Value * b, unsigned shift) {
    unsigned field_count = mBitBlockWidth/fw;
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    std::vector<Constant*> Idxs;
    for (unsigned i = shift; i < field_count + shift; i++) {
        Idxs.push_back(mLLVMBuilder->getInt32(i));
    }
    return mLLVMBuilder->CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs));
}

Value * IDISA_Builder::bitblock_any(Value * a) {
    Type * iBitBlock = mLLVMBuilder->getIntNTy(mBitBlockWidth);
    return mLLVMBuilder->CreateICmpNE(mLLVMBuilder->CreateBitCast(a, iBitBlock),  ConstantInt::get(iBitBlock, 0));
}

Value * IDISA_Builder::simd_and(Value * a, Value * b) {
    return a->getType() == b->getType() ? mLLVMBuilder->CreateAnd(a, b) : mLLVMBuilder->CreateAnd(bitBlockCast(a), bitBlockCast(b));
}

Value * IDISA_Builder::simd_or(Value * a, Value * b) {
    return a->getType() == b->getType() ? mLLVMBuilder->CreateOr(a, b) : mLLVMBuilder->CreateOr(bitBlockCast(a), bitBlockCast(b));
}
    
Value * IDISA_Builder::simd_xor(Value * a, Value * b) {
    return a->getType() == b->getType() ? mLLVMBuilder->CreateXor(a, b) : mLLVMBuilder->CreateXor(bitBlockCast(a), bitBlockCast(b));
}

Value * IDISA_Builder::simd_not(Value * a) {
    return simd_xor(a, Constant::getAllOnesValue(a->getType()));
}

}
