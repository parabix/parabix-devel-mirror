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
        return hsimd_packh(mBitBlockWidth/2, shufa, shufb);
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
        return hsimd_packl(mBitBlockWidth/2, shufa, shufb);
    }
    else {
        std::vector<Constant*> Idxs;
        for (unsigned i = 0; i < field_count; i++) {
            Idxs.push_back(getInt32(2*i+1));
        }
        return CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs));
    }
}
    
Value * IDISA_AVX2_Builder::esimd_mergeh(unsigned fw, Value * a, Value * b) {
    if ((fw == 128) && (mBitBlockWidth == 256)) {
        Value * vperm2i128func = Intrinsic::getDeclaration(mMod, Intrinsic::x86_avx2_vperm2i128);
        return CreateCall3(vperm2i128func, fwCast(64, a), fwCast(64, b), getInt8(0x31));
    }
    unsigned field_count = mBitBlockWidth/fw;
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    std::vector<Constant*> Idxs;
    for (unsigned i = field_count/2; i < field_count; i++) {
        Idxs.push_back(getInt32(i));    // selects elements from first reg.
        Idxs.push_back(getInt32(i + field_count)); // selects elements from second reg.
    }
    return CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs));
}

Value * IDISA_AVX2_Builder::esimd_mergel(unsigned fw, Value * a, Value * b) {
    if ((fw == 128) && (mBitBlockWidth == 256)) {
        Value * vperm2i128func = Intrinsic::getDeclaration(mMod, Intrinsic::x86_avx2_vperm2i128);
        return CreateCall3(vperm2i128func, fwCast(64, a), fwCast(64, b), getInt8(0x20));
    }
    unsigned field_count = mBitBlockWidth/fw;
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    std::vector<Constant*> Idxs;
    for (unsigned i = 0; i < field_count/2; i++) {
        Idxs.push_back(getInt32(i));    // selects elements from first reg.
        Idxs.push_back(getInt32(i + field_count)); // selects elements from second reg.
    }
    return CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs));
}

Value * IDISA_AVX2_Builder::hsimd_packl_in_lanes(unsigned lanes, unsigned fw, Value * a, Value * b) {
    if ((fw == 16)  && (lanes == 2)) {
        Value * vpackuswbfunc = Intrinsic::getDeclaration(mMod, Intrinsic::x86_avx2_packuswb);
        Value * a_low = fwCast(16, simd_and(a, simd_lomask(fw)));
        Value * b_low = fwCast(16, simd_and(b, simd_lomask(fw)));
        Value * pack = CreateCall2(vpackuswbfunc, a_low, b_low);
        return pack;
    }
    unsigned fw_out = fw/2;
    unsigned fields_per_lane = mBitBlockWidth/(fw_out * lanes);
    unsigned field_offset_for_b = mBitBlockWidth/fw_out;
    Value * aVec = fwCast(fw_out, a);
    Value * bVec = fwCast(fw_out, b);
    std::vector<Constant*> Idxs;
    for (unsigned lane = 0; lane < lanes; lane++) {
        unsigned first_field_in_lane = lane * fields_per_lane; // every second field
        for (unsigned i = 0; i < fields_per_lane/2; i++) {
            Idxs.push_back(getInt32(first_field_in_lane + 2*i));
        }
        for (unsigned i = 0; i < fields_per_lane/2; i++) {
            Idxs.push_back(getInt32(field_offset_for_b + first_field_in_lane + 2*i));
        }
    }
    Value * pack = CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs));
    return pack;
}

Value * IDISA_AVX2_Builder::hsimd_packh_in_lanes(unsigned lanes, unsigned fw, Value * a, Value * b) {
    if ((fw == 16)  && (lanes == 2)) {
        Value * vpackuswbfunc = Intrinsic::getDeclaration(mMod, Intrinsic::x86_avx2_packuswb);
        Value * a_low = simd_srli(fw, a, fw/2);
        Value * b_low = simd_srli(fw, b, fw/2);
        Value * pack = CreateCall2(vpackuswbfunc, a_low, b_low);
        return pack;
    }
    unsigned fw_out = fw/2;
    unsigned fields_per_lane = mBitBlockWidth/(fw_out * lanes);
    unsigned field_offset_for_b = mBitBlockWidth/fw_out;
    Value * aVec = fwCast(fw_out, a);
    Value * bVec = fwCast(fw_out, b);
    std::vector<Constant*> Idxs;
    for (unsigned lane = 0; lane < lanes; lane++) {
        unsigned first_field_in_lane = lane * fields_per_lane; // every second field
        for (unsigned i = 0; i < fields_per_lane/2; i++) {
            Idxs.push_back(getInt32(first_field_in_lane + 2*i));
        }
        for (unsigned i = 0; i < fields_per_lane/2; i++) {
            Idxs.push_back(getInt32(field_offset_for_b + first_field_in_lane + 2*i));
        }
    }
    Value * pack = CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs));
    return pack;
}
    
}
