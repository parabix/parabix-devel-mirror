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

VectorType * IDISA_Builder::fwVectorType(unsigned fw) {
    int fieldCount = mBitBlockWidth/fw;
    return VectorType::get(mLLVMBuilder->getIntNTy(fw), fieldCount);
}

Value * IDISA_Builder::fwCast(unsigned fw, Value * a) {
    return a->getType() == fwVectorType(fw) ? a : mLLVMBuilder->CreateBitCast(a, fwVectorType(fw));
}

void IDISA_Builder::genPrintRegister(std::string regName, Value * bitblockValue) {
    if (mPrintRegisterFunction == nullptr) {
        mPrintRegisterFunction = mMod->getOrInsertFunction("wrapped_print_register", Type::getVoidTy(mMod->getContext()), Type::getInt8PtrTy(mMod->getContext()), mBitBlockType, NULL);
    }
    Constant * regNameData = ConstantDataArray::getString(mMod->getContext(), regName);
    GlobalVariable *regStrVar = new GlobalVariable(*mMod,
                                                   ArrayType::get(IntegerType::get(mMod->getContext(), 8), regName.length()+1),
                                                   /*isConstant=*/ true,
                                                   /*Linkage=*/ GlobalValue::PrivateLinkage,
                                                   /*Initializer=*/ regNameData);
    Value * regStrPtr = mLLVMBuilder->CreateGEP(regStrVar, std::vector<Value *>({mLLVMBuilder->getInt64(0), mLLVMBuilder->getInt32(0)}));
    mLLVMBuilder->CreateCall(mPrintRegisterFunction, std::vector<Value *>({regStrPtr, bitCast(bitblockValue)}));
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

Value * IDISA_Builder:: esimd_bitspread(unsigned fw, Value * bitmask) {
    unsigned field_count = mBitBlockWidth/fw;
    Type * field_type = mLLVMBuilder->getIntNTy(fw);
    if (bitmask->getType()->getIntegerBitWidth() < fw) {
        bitmask = mLLVMBuilder->CreateZExt(bitmask, field_type);
    }
    else if (bitmask->getType()->getIntegerBitWidth() > fw) {
        bitmask = mLLVMBuilder->CreateTrunc(bitmask, field_type);
    }
    Value * spread_field = mLLVMBuilder->CreateBitCast(bitmask, VectorType::get(mLLVMBuilder->getIntNTy(fw), 1));
    Value * undefVec = UndefValue::get(VectorType::get(mLLVMBuilder->getIntNTy(fw), 1));
    Value * broadcast = mLLVMBuilder->CreateShuffleVector(spread_field, undefVec, Constant::getNullValue(VectorType::get(mLLVMBuilder->getInt32Ty(), field_count)));
    std::vector<Constant*> bitSel;
    std::vector<Constant*> bitShift;
    for (unsigned i = 0; i < field_count; i++) {
        bitSel.push_back(ConstantInt::get(field_type, 1 << i));
        bitShift.push_back(ConstantInt::get(field_type, i));
    }
    Value * bitSelVec = ConstantVector::get(bitSel);
    Value * bitShiftVec = ConstantVector::get(bitShift);
    return mLLVMBuilder->CreateLShr(mLLVMBuilder->CreateAnd(bitSelVec, broadcast), bitShiftVec);
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
    for (unsigned i = 0; i < field_count; i++) {
        Idxs.push_back(mLLVMBuilder->getInt32(i + shift));
    }
    return mLLVMBuilder->CreateShuffleVector(bVec, aVec, ConstantVector::get(Idxs));
}

Value * IDISA_Builder::bitblock_any(Value * a) {
    Type * iBitBlock = mLLVMBuilder->getIntNTy(mBitBlockWidth);
    return mLLVMBuilder->CreateICmpNE(mLLVMBuilder->CreateBitCast(a, iBitBlock),  ConstantInt::get(iBitBlock, 0));
}

Value * IDISA_Builder::simd_and(Value * a, Value * b) {
    return a->getType() == b->getType() ? mLLVMBuilder->CreateAnd(a, b) : mLLVMBuilder->CreateAnd(bitCast(a), bitCast(b));
}

Value * IDISA_Builder::simd_or(Value * a, Value * b) {
    return a->getType() == b->getType() ? mLLVMBuilder->CreateOr(a, b) : mLLVMBuilder->CreateOr(bitCast(a), bitCast(b));
}
    
Value * IDISA_Builder::simd_xor(Value * a, Value * b) {
    return a->getType() == b->getType() ? mLLVMBuilder->CreateXor(a, b) : mLLVMBuilder->CreateXor(bitCast(a), bitCast(b));
}

Value * IDISA_Builder::simd_not(Value * a) {
    return simd_xor(a, Constant::getAllOnesValue(a->getType()));
}

}
