/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "idisa_sse_builder.h"
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Module.h>

using namespace llvm;

namespace IDISA {

std::string IDISA_SSE_Builder::getBuilderUniqueName() { return mBitBlockWidth != 128 ? "SSE_" + std::to_string(mBitBlockWidth) : "SSE";}
std::string IDISA_SSE2_Builder::getBuilderUniqueName() { return mBitBlockWidth != 128 ? "SSE2_" + std::to_string(mBitBlockWidth) : "SSE2";}
std::string IDISA_SSSE3_Builder::getBuilderUniqueName() { return mBitBlockWidth != 128 ? "SSSE3_" + std::to_string(mBitBlockWidth) : "SSSE3";}

Value * IDISA_SSE2_Builder::hsimd_packh(unsigned fw, Value * a, Value * b) {    
    if ((fw == 16) && (mBitBlockWidth == 128)) {
        Value * packuswb_func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_sse2_packuswb_128);
        return CreateCall(packuswb_func, {simd_srli(16, a, 8), simd_srli(16, b, 8)});
    }
    // Otherwise use default logic.
    return IDISA_SSE_Builder::hsimd_packh(fw, a, b);
}

Value * IDISA_SSE2_Builder::hsimd_packl(unsigned fw, Value * a, Value * b) {
    if ((fw == 16) && (mBitBlockWidth == 128)) {
        Value * packuswb_func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_sse2_packuswb_128);
        Value * mask = simd_lomask(16);
        return CreateCall(packuswb_func, {fwCast(16, simd_and(a, mask)), fwCast(16, simd_and(b, mask))});
    }
    // Otherwise use default logic.
    return IDISA_Builder::hsimd_packl(fw, a, b);
}

Value * IDISA_SSE2_Builder::hsimd_signmask(unsigned fw, Value * a) {
    // SSE2 special case using Intrinsic::x86_sse2_movmsk_pd (fw=32 only)
    if (mBitBlockWidth == 128) {
        if (fw == 64) {
            Value * signmask_f64func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_sse2_movmsk_pd);
            Type * bitBlock_f64type = VectorType::get(getDoubleTy(), mBitBlockWidth/64);
            Value * a_as_pd = CreateBitCast(a, bitBlock_f64type);
            return CreateCall(signmask_f64func, a_as_pd);
        }
        if (fw == 8) {
            Value * pmovmskb_func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_sse2_pmovmskb_128);
            return CreateCall(pmovmskb_func, fwCast(8, a));
        }
    }
    const auto fieldCount = mBitBlockWidth / fw;
    if ((fieldCount > 4) && (fieldCount <= 16)) {
        Value * pmovmskb_func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_sse2_pmovmskb_128);
        int fieldBytes = fw / 8;
        int hiByte = fieldBytes - 1;
        Constant * Idxs[16];
        for (unsigned i = 0; i < fieldCount; i++) {
            Idxs[i] = getInt32(fieldBytes * i + hiByte);
        }
        for (unsigned i = fieldCount; i < 16; i++) {
            Idxs[i] = getInt32(mBitBlockWidth / 8);
        }
        Value * packh = CreateShuffleVector(fwCast(8, a), fwCast(8, allZeroes()), ConstantVector::get({Idxs, 16}));
        return CreateCall(pmovmskb_func, packh);
    }
    // Otherwise use default SSE logic.
    return IDISA_SSE_Builder::hsimd_signmask(fw, a);
}

Value * IDISA_SSE_Builder::hsimd_signmask(const unsigned fw, Value * a) {
    // SSE special cases using Intrinsic::x86_sse_movmsk_ps (fw=32 only)
    if (fw == 32) {
        Value * signmask_f32func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_sse_movmsk_ps);
        Type * bitBlock_f32type = VectorType::get(getFloatTy(), mBitBlockWidth/32);
        Value * a_as_ps = CreateBitCast(a, bitBlock_f32type);
        if (mBitBlockWidth == 128) {
            return CreateCall(signmask_f32func, a_as_ps);
        }
    } else if ((fw == 64) && (mBitBlockWidth == 256)) {
        Type * bitBlock_f32type = VectorType::get(getFloatTy(), mBitBlockWidth/32);
        Value * a_as_ps = CreateBitCast(a, bitBlock_f32type);
        Constant * Idxs[4];
        for (unsigned i = 0; i < 4; i++) {
            Idxs[i] = getInt32(2 * i + 1);
        }
        Value * packh = CreateShuffleVector(a_as_ps, UndefValue::get(bitBlock_f32type), ConstantVector::get({Idxs, 4}));
        Type * halfBlock_f32type = VectorType::get(getFloatTy(), mBitBlockWidth/64);
        Value * pack_as_ps = CreateBitCast(packh, halfBlock_f32type);
        Value * signmask_f32func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_sse_movmsk_ps);
        Value * mask = CreateCall(signmask_f32func, pack_as_ps);
        return mask;
    }
    // Otherwise use default logic.
    return IDISA_Builder::hsimd_signmask(fw, a);
}

#define SHIFT_FIELDWIDTH 64
//#define LEAVE_CARRY_UNNORMALIZED

// full shift producing {shiftout, shifted}
std::pair<Value *, Value *> IDISA_SSE2_Builder::bitblock_advance(Value * a, Value * shiftin, unsigned shift) {
    Value * shifted = nullptr;
    Value * shiftout = nullptr;
    Type * shiftTy = shiftin->getType();
    if (LLVM_UNLIKELY(shift == 0)) {
        return std::pair<Value *, Value *>(Constant::getNullValue(shiftTy), a);
    }
    Value * si = shiftin;
    if (shiftTy != mBitBlockType) {
        si = bitCast(CreateZExt(shiftin, getIntNTy(mBitBlockWidth)));
    }
    if (LLVM_UNLIKELY(shift == mBitBlockWidth)) {
        return std::pair<Value *, Value *>(CreateBitCast(a, shiftTy), si);
    }
#ifndef LEAVE_CARRY_UNNORMALIZED
    if (LLVM_UNLIKELY((shift % 8) == 0)) { // Use a single whole-byte shift, if possible.
        shifted = bitCast(simd_or(mvmd_slli(8, a, shift / 8), si));
        shiftout = bitCast(mvmd_srli(8, a, (mBitBlockWidth - shift) / 8));
        return std::pair<Value *, Value *>(shiftout, shifted);
    }
    Value * shiftback = simd_srli(SHIFT_FIELDWIDTH, a, SHIFT_FIELDWIDTH - (shift % SHIFT_FIELDWIDTH));
    Value * shiftfwd = simd_slli(SHIFT_FIELDWIDTH, a, shift % SHIFT_FIELDWIDTH);
    if (LLVM_LIKELY(shift < SHIFT_FIELDWIDTH)) {
        shiftout = mvmd_srli(SHIFT_FIELDWIDTH, shiftback, mBitBlockWidth/SHIFT_FIELDWIDTH - 1);
        shifted = simd_or(simd_or(shiftfwd, si), mvmd_slli(SHIFT_FIELDWIDTH, shiftback, 1));
    }
    else {
        shiftout = simd_or(shiftback, mvmd_srli(SHIFT_FIELDWIDTH, shiftfwd, 1));
        shifted = simd_or(si, mvmd_slli(SHIFT_FIELDWIDTH, shiftfwd, (mBitBlockWidth - shift) / SHIFT_FIELDWIDTH));
        if (shift < mBitBlockWidth - SHIFT_FIELDWIDTH) {
            shiftout = mvmd_srli(SHIFT_FIELDWIDTH, shiftout, (mBitBlockWidth - shift) / SHIFT_FIELDWIDTH);
            shifted = simd_or(shifted, mvmd_slli(SHIFT_FIELDWIDTH, shiftback, shift/SHIFT_FIELDWIDTH + 1));
        }
    }
#endif
#ifdef LEAVE_CARRY_UNNORMALIZED
    shiftout = a;
    if (LLVM_UNLIKELY((shift % 8) == 0)) { // Use a single whole-byte shift, if possible.
        shifted = mvmd_dslli(8, a, shiftin, (mBitBlockWidth - shift) / 8);
    }
    else if (LLVM_LIKELY(shift < SHIFT_FIELDWIDTH)) {
        Value * ahead = mvmd_dslli(SHIFT_FIELDWIDTH, a, shiftin, mBitBlockWidth / SHIFT_FIELDWIDTH - 1);
        shifted = simd_or(simd_srli(SHIFT_FIELDWIDTH, ahead, SHIFT_FIELDWIDTH - shift), simd_slli(SHIFT_FIELDWIDTH, a, shift));
    }
    else {
        throw std::runtime_error("Unsupported shift.");
    }
#endif
    if (shiftTy != mBitBlockType) {
        shiftout = CreateBitCast(shiftout, shiftTy);
    }
    //CallPrintRegister("shifted", shifted);
    //CallPrintRegister("shiftout", shiftout);
    return std::pair<Value *, Value *>(shiftout, shifted);
}
    
Value * IDISA_SSE2_Builder::mvmd_shuffle(unsigned fw, Value * a, Value * index_vector) {
    if ((mBitBlockWidth == 128) && (fw == 64)) {
        // First create a vector with exchanged values of the 2 fields.
        Constant * idx[2] = {ConstantInt::get(getInt32Ty(), 1), ConstantInt::get(getInt32Ty(), 0)};
        Value * exchanged = CreateShuffleVector(a, UndefValue::get(fwVectorType(fw)), ConstantVector::get({idx, 2}));
        // bits that change if the value in a needs to be exchanged.
        Value * changed = simd_xor(a, exchanged);
        // Now create a mask to select between original and exchanged values.
        Constant * xchg[2] = {ConstantInt::get(getInt64Ty(), 1), ConstantInt::get(getInt64Ty(), 0)};
        Value * xchg_vec = ConstantVector::get({xchg, 2});
        Constant * oneSplat = ConstantVector::getSplat(2, ConstantInt::get(getInt64Ty(), 1));
        Value * exchange_mask = simd_eq(fw, simd_and(index_vector, oneSplat), xchg_vec);
        Value * rslt = simd_xor(simd_and(changed, exchange_mask), a);
        return rslt;
    }
    return IDISA_SSE_Builder::mvmd_shuffle(fw, a, index_vector);
}

Value * IDISA_SSE_Builder::mvmd_compress(unsigned fw, Value * a, Value * selector) {
    if ((mBitBlockWidth == 128) && (fw == 64)) {
        Constant * keep[2] = {ConstantInt::get(getInt64Ty(), 1), ConstantInt::get(getInt64Ty(), 3)};
        Constant * keep_mask = ConstantVector::get({keep, 2});
        Constant * shift[2] = {ConstantInt::get(getInt64Ty(), 2), ConstantInt::get(getInt64Ty(), 0)};
        Constant * shifted_mask = ConstantVector::get({shift, 2});
        Value * a_srli1 = mvmd_srli(64, a, 1);
        Value * bdcst = simd_fill(64, CreateZExt(selector, getInt64Ty()));
        Value * kept = simd_and(simd_eq(64, simd_and(keep_mask, bdcst), keep_mask), a);
        Value * shifted = simd_and(a_srli1, simd_eq(64, shifted_mask, bdcst));
        return simd_or(kept, shifted);
    }
    if ((mBitBlockWidth == 128) && (fw == 32)) {
        Value * bdcst = simd_fill(32, CreateZExtOrTrunc(selector, getInt32Ty()));
        Constant * fieldBit[4] =
        {ConstantInt::get(getInt32Ty(), 1), ConstantInt::get(getInt32Ty(), 2),
            ConstantInt::get(getInt32Ty(), 4), ConstantInt::get(getInt32Ty(), 8)};
        Constant * fieldMask = ConstantVector::get({fieldBit, 4});
        Value * a_selected = simd_and(simd_eq(32, fieldMask, simd_and(fieldMask, bdcst)), a);
        Constant * rotateInwards[4] =
        {ConstantInt::get(getInt32Ty(), 1), ConstantInt::get(getInt32Ty(), 0),
            ConstantInt::get(getInt32Ty(), 3), ConstantInt::get(getInt32Ty(), 2)};
        Constant * rotateVector = ConstantVector::get({rotateInwards, 4});
        Value * rotated = CreateShuffleVector(fwCast(32, a_selected), UndefValue::get(fwVectorType(fw)), rotateVector);
        Constant * rotate_bit[2] = {ConstantInt::get(getInt64Ty(), 2), ConstantInt::get(getInt64Ty(), 4)};
        Constant * rotate_mask = ConstantVector::get({rotate_bit, 2});
        Value * rotateControl = simd_eq(64, fwCast(64, simd_and(bdcst, rotate_mask)), allZeroes());
        Value * centralResult = simd_if(1, rotateControl, rotated, a_selected);
        Value * delete_marks_lo = CreateAnd(CreateNot(selector), ConstantInt::get(selector->getType(), 3));
        Value * delCount_lo = CreateSub(delete_marks_lo, CreateLShr(delete_marks_lo, 1));
        return mvmd_srl(32, centralResult, delCount_lo);
    }
    return IDISA_Builder::mvmd_compress(fw, a, selector);
}

Value * IDISA_SSSE3_Builder::esimd_mergeh(unsigned fw, Value * a, Value * b) {
    if ((fw == 1) || (fw == 2)) {
        Constant * interleave_table = bit_interleave_byteshuffle_table(fw);
        // Merge the bytes.
        Value * byte_merge = esimd_mergeh(8, a, b);
        Value * low_bits = mvmd_shuffle(8, interleave_table, fwCast(8, simd_and(byte_merge, simd_lomask(8))));
        Value * high_bits = simd_slli(16, mvmd_shuffle(8, interleave_table, fwCast(8, simd_srli(8, byte_merge, 4))), fw);
        // For each 16-bit field, interleave the low bits of the two bytes.
        low_bits = simd_or(simd_select_lo(16, low_bits), simd_srli(16, low_bits, 8-fw));
        // For each 16-bit field, interleave the high bits of the two bytes.
        high_bits = simd_or(simd_select_hi(16, high_bits), simd_slli(16, high_bits, 8-fw));
        return simd_or(low_bits, high_bits);
    }
    // Otherwise use default SSE logic.
    return IDISA_SSE2_Builder::esimd_mergeh(fw, a, b);
}

Value * IDISA_SSSE3_Builder::esimd_mergel(unsigned fw, Value * a, Value * b) {
    if ((fw == 1) || (fw == 2)) {
        Constant * interleave_table = bit_interleave_byteshuffle_table(fw);
        // Merge the bytes.
        Value * byte_merge = esimd_mergel(8, a, b);
        Value * low_bits = mvmd_shuffle(8, interleave_table, fwCast(8, simd_and(byte_merge, simd_lomask(8))));
        Value * high_bits = simd_slli(16, mvmd_shuffle(8, interleave_table, fwCast(8, simd_srli(8, byte_merge, 4))), fw);
        // For each 16-bit field, interleave the low bits of the two bytes.
        low_bits = simd_or(simd_select_lo(16, low_bits), simd_srli(16, low_bits, 8-fw));
        // For each 16-bit field, interleave the high bits of the two bytes.
        high_bits = simd_or(simd_select_hi(16, high_bits), simd_slli(16, high_bits, 8-fw));
        return simd_or(low_bits, high_bits);
    }
    // Otherwise use default SSE2 logic.
    return IDISA_SSE2_Builder::esimd_mergel(fw, a, b);
}

llvm::Value * IDISA_SSSE3_Builder::mvmd_shuffle(unsigned fw, llvm::Value * data_table, llvm::Value * index_vector) {
    if (mBitBlockWidth == 128 && fw > 8) {
        // Create a table for shuffling with smaller field widths.
        const unsigned fieldCount = mBitBlockWidth/fw;
        Constant * idxMask = ConstantVector::getSplat(fieldCount, ConstantInt::get(getIntNTy(fw), fieldCount-1));
        Value * idx = simd_and(index_vector, idxMask);
        unsigned half_fw = fw/2;
        unsigned field_count = mBitBlockWidth/half_fw;
        // Build a ConstantVector of alternating 0 and 1 values.
        Constant * Idxs[field_count];
        for (unsigned int i = 0; i < field_count; i++) {
            Idxs[i] = ConstantInt::get(getIntNTy(fw/2), i & 1);
        }
        Constant * splat01 = ConstantVector::get({Idxs, field_count});
        
        Value * half_fw_indexes = simd_or(idx, mvmd_slli(half_fw, idx, 1));
        half_fw_indexes = simd_add(fw, simd_add(fw, half_fw_indexes, half_fw_indexes), splat01);
        Value * rslt = mvmd_shuffle(half_fw, data_table, half_fw_indexes);
        return rslt;
    }
    if (mBitBlockWidth == 128 && fw == 8) {
        Value * shuf8Func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_ssse3_pshuf_b_128);
        return CreateCall(shuf8Func, {fwCast(8, data_table), fwCast(8, simd_and(index_vector, simd_lomask(8)))});
    }
    return IDISA_SSE2_Builder::mvmd_shuffle(fw, data_table, index_vector);
}


}
