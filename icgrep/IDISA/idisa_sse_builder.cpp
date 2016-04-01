/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "idisa_sse_builder.h"
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Function.h>

namespace IDISA {


Value * IDISA_SSE2_Builder::hsimd_packh(unsigned fw, Value * a, Value * b) {
    if ((fw == 16) && (mBitBlockWidth == 128)) {
        Value * packuswb_func = Intrinsic::getDeclaration(mMod, Intrinsic::x86_sse2_packuswb_128);
        return CreateCall2(packuswb_func, simd_srli(16, a, 8), simd_srli(16, b, 8));
    }
    // Otherwise use default logic.
    return IDISA_Builder::hsimd_packh(fw, a, b);
}

Value * IDISA_SSE2_Builder::hsimd_packl(unsigned fw, Value * a, Value * b) {
    if ((fw == 16) && (mBitBlockWidth == 128)) {
        Value * packuswb_func = Intrinsic::getDeclaration(mMod, Intrinsic::x86_sse2_packuswb_128);
        Value * mask = simd_lomask(16);
        return CreateCall2(packuswb_func, fwCast(16, simd_and(a, mask)), fwCast(16, simd_and(b, mask)));
    }
    // Otherwise use default logic.
    return IDISA_Builder::hsimd_packl(fw, a, b);
}

Value * IDISA_SSE2_Builder::hsimd_signmask(unsigned fw, Value * a) {
    // SSE2 special case using Intrinsic::x86_sse2_movmsk_pd (fw=32 only)
    if (mBitBlockWidth == 128) {
        if (fw == 64) {
            Value * signmask_f64func = Intrinsic::getDeclaration(mMod, Intrinsic::x86_sse2_movmsk_pd);
            Type * bitBlock_f64type = VectorType::get(getDoubleTy(), mBitBlockWidth/64);
            Value * a_as_pd = CreateBitCast(a, bitBlock_f64type);
            Value * mask = CreateCall(signmask_f64func, a_as_pd);
            return mask;
        }
        if (fw == 8) {
            Value * pmovmskb_func = Intrinsic::getDeclaration(mMod, Intrinsic::x86_sse2_pmovmskb_128);
            Value * mask = CreateCall(pmovmskb_func, fwCast(8, a));
            return mask;
        }
    }
    int fieldCount = mBitBlockWidth/fw;
    if ((fieldCount > 4) && (fieldCount <= 16)) {
        Value * pmovmskb_func = Intrinsic::getDeclaration(mMod, Intrinsic::x86_sse2_pmovmskb_128);
        int fieldBytes = fw/8;
        int hiByte = fieldBytes - 1;
        std::vector<Constant*> Idxs;
        for (unsigned i = 0; i < fieldCount; i++) {
            Idxs.push_back(getInt32(fieldBytes*i+hiByte));
        }
        for (unsigned i = fieldCount; i < 16; i++) {
            Idxs.push_back(getInt32(mBitBlockWidth/8));
        }
        Value * packh = CreateShuffleVector(fwCast(8, a), fwCast(8, allZeroes()), ConstantVector::get(Idxs));
        Value * mask = CreateCall(pmovmskb_func, packh);
        return mask;
    }
    // Otherwise use default SSE logic.
    return IDISA_SSE_Builder::hsimd_signmask(fw, a);
}

Value * IDISA_SSE_Builder::hsimd_signmask(unsigned fw, Value * a) {
    // SSE special cases using Intrinsic::x86_sse_movmsk_ps (fw=32 only)
    if (fw == 32) {
        Value * signmask_f32func = Intrinsic::getDeclaration(mMod, Intrinsic::x86_sse_movmsk_ps);
        Type * bitBlock_f32type = VectorType::get(getFloatTy(), mBitBlockWidth/32);
        Value * a_as_ps = CreateBitCast(a, bitBlock_f32type);
        if (mBitBlockWidth == 128) {
            return CreateCall(signmask_f32func, a_as_ps);
        }
    }
    else if ((fw == 64) && (mBitBlockWidth == 256)) {
        Type * bitBlock_f32type = VectorType::get(getFloatTy(), mBitBlockWidth/32);
        Value * a_as_ps = CreateBitCast(a, bitBlock_f32type);
        std::vector<Constant*> Idxs;
        for (unsigned i = 0; i < mBitBlockWidth/fw; i++) {
            Idxs.push_back(getInt32(2*i+1));
        }
        Value * packh = CreateShuffleVector(a_as_ps, UndefValue::get(bitBlock_f32type), ConstantVector::get(Idxs));
        Type * halfBlock_f32type = VectorType::get(getFloatTy(), mBitBlockWidth/64);
        Value * pack_as_ps = CreateBitCast(packh, halfBlock_f32type);
        Value * signmask_f32func = Intrinsic::getDeclaration(mMod, Intrinsic::x86_sse_movmsk_ps);
        Value * mask = CreateCall(signmask_f32func, pack_as_ps);
        return mask;
    }
    // Otherwise use default logic.
    return IDISA_Builder::hsimd_signmask(fw, a);
}
    
}
