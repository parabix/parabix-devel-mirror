/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "idisa_avx_builder.h"
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Function.h>

namespace IDISA {

Value * IDISA_AVX_Builder::hsimd_signmask(unsigned fw, Value * a) {
    if (mBitBlockWidth == 256) {
        if (fw == 64) {
            Value * signmask_f64func = Intrinsic::getDeclaration(mMod, Intrinsic::x86_avx_movmsk_pd_256);
            Type * bitBlock_f64type = VectorType::get(getDoubleTy(), mBitBlockWidth/64);
            Value * a_as_pd = CreateBitCast(a, bitBlock_f64type);
            Value * mask = CreateCall(signmask_f64func, std::vector<Value *>({a_as_pd}));
            return mask;
        }
        else if (fw == 32) {
            Value * signmask_f32func = Intrinsic::getDeclaration(mMod, Intrinsic::x86_avx_movmsk_ps_256);
            Type * bitBlock_f32type = VectorType::get(getFloatTy(), mBitBlockWidth/32);
            Value * a_as_ps = CreateBitCast(a, bitBlock_f32type);
            Value * mask = CreateCall(signmask_f32func, std::vector<Value *>({a_as_ps}));
            return mask;
        }
    }
    else if (mBitBlockWidth == 512) {
        if (fw == 64) {
            Type * bitBlock_f32type = VectorType::get(getFloatTy(), mBitBlockWidth/32);
            Value * a_as_ps = CreateBitCast(a, bitBlock_f32type);
            std::vector<Constant*> Idxs;
            for (unsigned i = 0; i < 8; i++) {
                Idxs.push_back(getInt32(2*i+1));
            }
            Value * packh = CreateShuffleVector(a_as_ps, UndefValue::get(bitBlock_f32type), ConstantVector::get(Idxs));
            Type * halfBlock_f32type = VectorType::get(getFloatTy(), mBitBlockWidth/64);
            Value * pack_as_ps = CreateBitCast(packh, halfBlock_f32type);
            Value * signmask_f32func = Intrinsic::getDeclaration(mMod, Intrinsic::x86_avx_movmsk_ps_256);
            Value * mask = CreateCall(signmask_f32func, std::vector<Value *>({pack_as_ps}));
            return mask;
        }
    }
    Value * mask = CreateICmpSLT(fwCast(fw, a), ConstantAggregateZero::get(fwVectorType(fw)));
    return CreateBitCast(mask, getIntNTy(mBitBlockWidth/fw));
}
    
Value * IDISA_AVX2_Builder::hsimd_packh(unsigned fw, Value * a, Value * b) {
    unsigned field_count = 2 * mBitBlockWidth/fw;
    Value * aVec = fwCast(fw/2, a);
    Value * bVec = fwCast(fw/2, b);
    if (fw <= 64) {
        std::vector<Constant*> Idxs;
        for (unsigned i = 0; i < field_count/4; i++) {
            Idxs.push_back(getInt32(2*i + 1));
        }
        for (unsigned i = 0; i < field_count/4; i++) {
            Idxs.push_back(getInt32(2*i));
        }
        for (unsigned i = 0; i < field_count/4; i++) {
            Idxs.push_back(getInt32(field_count/2 + 2*i + 1));
        }
        for (unsigned i = 0; i < field_count/4; i++) {
            Idxs.push_back(getInt32(field_count/2 + 2*i));
        }
        Value * shufa = CreateShuffleVector(aVec, aVec, ConstantVector::get(Idxs));
        Value * shufb = CreateShuffleVector(bVec, bVec, ConstantVector::get(Idxs));
        Value * pk = hsimd_packh(128, shufa, shufb);
        return pk;
    }
    else {
        std::vector<Constant*> Idxs;
        for (unsigned i = 0; i < field_count; i++) {
            Idxs.push_back(getInt32(2*i));
        }
        return CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs));
    }
}

Value * IDISA_AVX2_Builder::hsimd_packl(unsigned fw, Value * a, Value * b) {
    unsigned field_count = 2 * mBitBlockWidth/fw;
    Value * aVec = fwCast(fw/2, a);
    Value * bVec = fwCast(fw/2, b);
    if (fw <= 64) {
        std::vector<Constant*> Idxs;
        for (unsigned i = 0; i < field_count/4; i++) {
            Idxs.push_back(getInt32(2*i + 1));
        }
        for (unsigned i = 0; i < field_count/4; i++) {
            Idxs.push_back(getInt32(2*i));
        }
        for (unsigned i = 0; i < field_count/4; i++) {
            Idxs.push_back(getInt32(field_count/2 + 2*i + 1));
        }
        for (unsigned i = 0; i < field_count/4; i++) {
            Idxs.push_back(getInt32(field_count/2 + 2*i));
        }
        Value * shufa = CreateShuffleVector(aVec, aVec, ConstantVector::get(Idxs));
        Value * shufb = CreateShuffleVector(bVec, bVec, ConstantVector::get(Idxs));
        return hsimd_packl(128, shufa, shufb);
    }
    else {
        std::vector<Constant*> Idxs;
        for (unsigned i = 0; i < field_count; i++) {
            Idxs.push_back(getInt32(2*i+1));
        }
        return CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs));
    }
}
}
