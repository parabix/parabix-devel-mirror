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
#include <iostream>

using namespace IDISA;

Value * IDISA_Builder::bitBlockCast(Value * a) {
    return llvm_builder->CreateBitCast(a, mBitBlockType);
}

VectorType * IDISA_Builder::fwVectorType(unsigned fw) {
    int fieldCount = mBitBlockSize/fw;
    return VectorType::get(llvm_builder->getIntNTy(fw), fieldCount);
}

Value * IDISA_Builder::fwCast(unsigned fw, Value * a) {
    return llvm_builder->CreateBitCast(a, fwVectorType(fw));
}

Value * IDISA_Builder::simd_add(unsigned fw, Value * a, Value * b) {
    return bitBlockCast(llvm_builder->CreateAdd(fwCast(fw, a), fwCast(fw, b)));
}

Value * IDISA_Builder::simd_sub(unsigned fw, Value * a, Value * b) {
    return bitBlockCast(llvm_builder->CreateSub(fwCast(fw, a), fwCast(fw, b)));
}

Value * IDISA_Builder::simd_mult(unsigned fw, Value * a, Value * b) {
    return bitBlockCast(llvm_builder->CreateMul(fwCast(fw, a), fwCast(fw, b)));
}

Value * IDISA_Builder::simd_eq(unsigned fw, Value * a, Value * b) {
    return bitBlockCast(llvm_builder->CreateSExt(llvm_builder->CreateICmpEQ(fwCast(fw, a), fwCast(fw, b)), fwVectorType(fw)));
}

Value * IDISA_Builder::simd_gt(unsigned fw, Value * a, Value * b) {
    return bitBlockCast(llvm_builder->CreateSExt(llvm_builder->CreateICmpSGT(fwCast(fw, a), fwCast(fw, b)), fwVectorType(fw)));
}

Value * IDISA_Builder::simd_ugt(unsigned fw, Value * a, Value * b) {
    return bitBlockCast(llvm_builder->CreateSExt(llvm_builder->CreateICmpUGT(fwCast(fw, a), fwCast(fw, b)), fwVectorType(fw)));
}

Value * IDISA_Builder::simd_lt(unsigned fw, Value * a, Value * b) {
    return bitBlockCast(llvm_builder->CreateSExt(llvm_builder->CreateICmpSLT(fwCast(fw, a), fwCast(fw, b)), fwVectorType(fw)));
}

Value * IDISA_Builder::simd_ult(unsigned fw, Value * a, Value * b) {
    return bitBlockCast(llvm_builder->CreateSExt(llvm_builder->CreateICmpULT(fwCast(fw, a), fwCast(fw, b)), fwVectorType(fw)));
}

Value * IDISA_Builder::simd_max(unsigned fw, Value * a, Value * b) {
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    return bitBlockCast(llvm_builder->CreateSelect(llvm_builder->CreateICmpSGT(aVec, bVec), aVec, bVec));
}

Value * IDISA_Builder::simd_umax(unsigned fw, Value * a, Value * b) {
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    return bitBlockCast(llvm_builder->CreateSelect(llvm_builder->CreateICmpUGT(aVec, bVec), aVec, bVec));
}

Value * IDISA_Builder::simd_min(unsigned fw, Value * a, Value * b) {
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    return bitBlockCast(llvm_builder->CreateSelect(llvm_builder->CreateICmpSLT(aVec, bVec), aVec, bVec));
}

Value * IDISA_Builder::simd_umin(unsigned fw, Value * a, Value * b) {
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    return bitBlockCast(llvm_builder->CreateSelect(llvm_builder->CreateICmpULT(aVec, bVec), aVec, bVec));
}

Value * IDISA_Builder::simd_slli(unsigned fw, Value * a, unsigned shift) {
    return bitBlockCast(llvm_builder->CreateShl(fwCast(fw, a), shift));
}

Value * IDISA_Builder::simd_srli(unsigned fw, Value * a, unsigned shift) {
    return bitBlockCast(llvm_builder->CreateLShr(fwCast(fw, a), shift));
}

Value * IDISA_Builder::simd_srai(unsigned fw, Value * a, unsigned shift) {
    return bitBlockCast(llvm_builder->CreateAShr(fwCast(fw, a), shift));
}

Value * IDISA_Builder::simd_cttz(unsigned fw, Value * a) {
    Value * cttzFunc = Intrinsic::getDeclaration(mMod, Intrinsic::cttz, fwVectorType(fw));
    Value * rslt = llvm_builder->CreateCall(cttzFunc, {fwCast(fw, a), ConstantInt::get(llvm_builder->getInt1Ty(), 0)});
    return bitBlockCast(rslt);
}

Value * IDISA_Builder::esimd_mergeh(unsigned fw, Value * a, Value * b) {
    unsigned field_count = mBitBlockSize/fw;
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    std::vector<Constant*> Idxs;
    for (unsigned i = field_count/2; i < field_count; i++) {
        Idxs.push_back(llvm_builder->getInt32(i));    // selects elements from first reg.
        Idxs.push_back(llvm_builder->getInt32(i + field_count)); // selects elements from second reg.
    }
    return bitBlockCast(llvm_builder->CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs)));
}

Value * IDISA_Builder::esimd_mergel(unsigned fw, Value * a, Value * b) {
    unsigned field_count = mBitBlockSize/fw;
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    std::vector<Constant*> Idxs;
    for (unsigned i = 0; i < field_count/2; i++) {
        Idxs.push_back(llvm_builder->getInt32(i));    // selects elements from first reg.
        Idxs.push_back(llvm_builder->getInt32(i + field_count)); // selects elements from second reg.
    }
    return bitBlockCast(llvm_builder->CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs)));
}

Value * IDISA_Builder::hsimd_packh(unsigned fw, Value * a, Value * b) {
    unsigned field_count = 2 * mBitBlockSize/fw;
    Value * aVec = fwCast(fw/2, a);
    Value * bVec = fwCast(fw/2, b);
    std::vector<Constant*> Idxs;
    for (unsigned i = 0; i < field_count; i++) {
        Idxs.push_back(llvm_builder->getInt32(2*i)); 
    }
    return bitBlockCast(llvm_builder->CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs)));
}

Value * IDISA_Builder::hsimd_packl(unsigned fw, Value * a, Value * b) {
    unsigned field_count = 2 * mBitBlockSize/fw;
    Value * aVec = fwCast(fw/2, a);
    Value * bVec = fwCast(fw/2, b);
    std::vector<Constant*> Idxs;
    for (unsigned i = 0; i < field_count; i++) {
        Idxs.push_back(llvm_builder->getInt32(2*i+1)); 
    }
    return bitBlockCast(llvm_builder->CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs)));
}

Value * IDISA_Builder::hsimd_signmask(unsigned fw, Value * a) {
    Value * mask = llvm_builder->CreateICmpSLT(fwCast(fw, a), ConstantAggregateZero::get(fwVectorType(fw)));
    return mask;
}

Value * IDISA_Builder::mvmd_dslli(unsigned fw, Value * a, Value * b, unsigned shift) {
    unsigned field_count = mBitBlockSize/fw;
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    std::vector<Constant*> Idxs;
    for (unsigned i = shift; i < field_count + shift; i++) {
        Idxs.push_back(llvm_builder->getInt32(i));
    }
    return bitBlockCast(llvm_builder->CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs)));
}


