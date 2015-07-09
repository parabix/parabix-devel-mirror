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

