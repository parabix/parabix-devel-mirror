/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "idisa_avx_builder.h"
#include <toolchain/toolchain.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/Intrinsics.h>

using namespace llvm;

namespace IDISA {

std::string IDISA_AVX_Builder::getBuilderUniqueName() {
    return mBitBlockWidth != 256 ? "AVX_" + std::to_string(mBitBlockWidth) : "AVX";
}

Value * IDISA_AVX_Builder::hsimd_signmask(unsigned fw, Value * a) {
    // AVX2 special cases
    if (mBitBlockWidth == 256) {
        if (fw == 64) {
            Value * signmask_f64func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx_movmsk_pd_256);
            Type * bitBlock_f64type = VectorType::get(getDoubleTy(), mBitBlockWidth/64);
            Value * a_as_pd = CreateBitCast(a, bitBlock_f64type);
            return CreateCall(signmask_f64func, a_as_pd);
        } else if (fw == 32) {
            Value * signmask_f32func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx_movmsk_ps_256);
            Type * bitBlock_f32type = VectorType::get(getFloatTy(), mBitBlockWidth/32);
            Value * a_as_ps = CreateBitCast(a, bitBlock_f32type);
            return CreateCall(signmask_f32func, a_as_ps);
        }
    } else if (mBitBlockWidth == 512) {
        if (fw == 64) {
            Type * bitBlock_f32type = VectorType::get(getFloatTy(), mBitBlockWidth / 32);
            Value * a_as_ps = CreateBitCast(a, bitBlock_f32type);
            Constant * indicies[8];
            for (unsigned i = 0; i < 8; i++) {
                indicies[i] = getInt32(2 * i + 1);
            }
            Value * packh = CreateShuffleVector(a_as_ps, UndefValue::get(bitBlock_f32type), ConstantVector::get({indicies, 8}));
            Type * halfBlock_f32type = VectorType::get(getFloatTy(), mBitBlockWidth/64);
            Value * pack_as_ps = CreateBitCast(packh, halfBlock_f32type);
            Value * signmask_f32func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx_movmsk_ps_256);
            return CreateCall(signmask_f32func, pack_as_ps);
        }
    }
    // Otherwise use default SSE2 logic.
    return IDISA_SSE2_Builder::hsimd_signmask(fw, a);
}

Value * IDISA_AVX_Builder::CreateZeroHiBitsFrom(Value * bits, Value * pos) {
    Type * Ty = bits->getType();
    if (hasBMI1 && (Ty == getInt64Ty())) {
        Value * bzhi_64 = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_bmi_bzhi_64);
        return CreateCall(bzhi_64, {bits, pos});
    }
    if (hasBMI1 && (Ty == getInt32Ty())) {
        Value * bzhi_32 = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_bmi_bzhi_32);
        return CreateCall(bzhi_32, {bits, pos});
    }
    return CBuilder::CreateZeroHiBitsFrom(bits, pos);
}

std::string IDISA_AVX2_Builder::getBuilderUniqueName() {
    return mBitBlockWidth != 256 ? "AVX2_" + std::to_string(mBitBlockWidth) : "AVX2";
}

Value * IDISA_AVX2_Builder::hsimd_packh(unsigned fw, Value * a, Value * b) {
    if ((fw > 8) && (fw <= 64)) {
        Value * aVec = fwCast(fw / 2, a);
        Value * bVec = fwCast(fw / 2, b);
        const auto field_count = 2 * mBitBlockWidth / fw;
        SmallVector<Constant *, 32> Idxs(field_count);
        const auto H = (field_count / 2);
        const auto Q = (field_count / 4);
        for (unsigned i = 0; i < Q; i++) {
            Idxs[i] = getInt32(2 * i);
            Idxs[i + Q] = getInt32((2 * i) + 1);
            Idxs[i + H] = getInt32((2 * i) + H);
            Idxs[i + H + Q] = getInt32((2 * i) + 1 + H);
        }
        Constant * const IdxVec = ConstantVector::get(Idxs);
        Value * shufa = CreateShuffleVector(aVec, aVec, IdxVec);
        Value * shufb = CreateShuffleVector(bVec, bVec, IdxVec);
        return hsimd_packh(mBitBlockWidth / 2, shufa, shufb);
    }
    // Otherwise use default SSE logic.
    return IDISA_SSE2_Builder::hsimd_packh(fw, a, b);
}

Value * IDISA_AVX2_Builder::hsimd_packl(unsigned fw, Value * a, Value * b) {
    if ((fw > 8) && (fw <= 64)) {
        Value * aVec = fwCast(fw / 2, a);
        Value * bVec = fwCast(fw / 2, b);
        const auto field_count = 2 * mBitBlockWidth / fw;
        SmallVector<Constant *, 16> Idxs(field_count);
        const auto H = (field_count / 2);
        const auto Q = (field_count / 4);
        for (unsigned i = 0; i < Q; i++) {
            Idxs[i] = getInt32(2 * i);
            Idxs[i + Q] = getInt32((2 * i) + 1);
            Idxs[i + H] = getInt32((2 * i) + H);
            Idxs[i + H + Q] = getInt32((2 * i) + H + 1);
        }
        Constant * const IdxVec = ConstantVector::get(Idxs);
        Value * shufa = CreateShuffleVector(aVec, aVec, IdxVec);
        Value * shufb = CreateShuffleVector(bVec, bVec, IdxVec);
        return hsimd_packl(mBitBlockWidth / 2, shufa, shufb);
    }
    // Otherwise use default SSE logic.
    return IDISA_SSE2_Builder::hsimd_packl(fw, a, b);
}

Value * IDISA_AVX2_Builder::esimd_mergeh(unsigned fw, Value * a, Value * b) {
    if (getVectorBitWidth(a) == mNativeBitBlockWidth) {
        if ((fw == 1) || (fw == 2)) {
            // Bit interleave using shuffle.
            Value * shufFn = Intrinsic::getDeclaration(getModule(),  Intrinsic::x86_avx2_pshuf_b);
            // Make a shuffle table that translates the lower 4 bits of each byte in
            // order to spread out the bits: xxxxdcba => .d.c.b.a
            // We use two copies of the table for the AVX2 _mm256_shuffle_epi8
            Constant * interleave_table = bit_interleave_byteshuffle_table(fw);
            // Merge the bytes.
            Value * byte_merge = esimd_mergeh(8, a, b);
            Value * low_bits = CreateCall(shufFn, {interleave_table,  fwCast(8, simd_select_lo(8, byte_merge))});
            Value * high_bits = simd_slli(16, CreateCall(shufFn, {interleave_table, fwCast(8, simd_srli(8, byte_merge, 4))}), fw);
            // For each 16-bit field, interleave the low bits of the two bytes.
            low_bits = simd_or(simd_select_lo(16, low_bits), simd_srli(16, low_bits, 8-fw));
            // For each 16-bit field, interleave the high bits of the two bytes.
            high_bits = simd_or(simd_select_hi(16, high_bits), simd_slli(16, high_bits, 8-fw));
            return simd_or(low_bits, high_bits);
        }
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(6, 0, 0)
        if (fw == 128) {
            Value * vperm2i128func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx2_vperm2i128);
            return CreateCall(vperm2i128func, {fwCast(64, a), fwCast(64, b), getInt8(0x31)});
        }
#endif
    }
    // Otherwise use default SSE logic.
    return IDISA_SSE2_Builder::esimd_mergeh(fw, a, b);
}

Value * IDISA_AVX2_Builder::esimd_mergel(unsigned fw, Value * a, Value * b) {
    if (getVectorBitWidth(a) == mNativeBitBlockWidth) {
        if ((fw == 1) || (fw == 2)) {
            // Bit interleave using shuffle.
            Value * shufFn = Intrinsic::getDeclaration(getModule(),  Intrinsic::x86_avx2_pshuf_b);
            // Make a shuffle table that translates the lower 4 bits of each byte in
            // order to spread out the bits: xxxxdcba => .d.c.b.a
            // We use two copies of the table for the AVX2 _mm256_shuffle_epi8
            Constant * interleave_table = bit_interleave_byteshuffle_table(fw);
            // Merge the bytes.
            Value * byte_merge = esimd_mergel(8, a, b);
            Value * low_bits = CreateCall(shufFn, {interleave_table,  fwCast(8, simd_select_lo(8, byte_merge))});
            Value * high_bits = simd_slli(16, CreateCall(shufFn, {interleave_table, fwCast(8, simd_srli(8, byte_merge, 4))}), fw);
            // For each 16-bit field, interleave the low bits of the two bytes.
            low_bits = simd_or(simd_select_lo(16, low_bits), simd_srli(16, low_bits, 8-fw));
            // For each 16-bit field, interleave the high bits of the two bytes.
            high_bits = simd_or(simd_select_hi(16, high_bits), simd_slli(16, high_bits, 8-fw));
            return simd_or(low_bits, high_bits);
        }
    #if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(6, 0, 0)
        if ((fw == 128) && (mBitBlockWidth == 256)) {
            Value * vperm2i128func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx2_vperm2i128);
            return CreateCall(vperm2i128func, {fwCast(64, a), fwCast(64, b), getInt8(0x20)});
        }
    #endif
    }
    // Otherwise use default SSE logic.
    return IDISA_SSE_Builder::esimd_mergel(fw, a, b);
}

Value * IDISA_AVX2_Builder::hsimd_packl_in_lanes(unsigned lanes, unsigned fw, Value * a, Value * b) {
    if ((fw == 16)  && (lanes == 2)) {
        Value * vpackuswbfunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx2_packuswb);
        Value * a_low = fwCast(16, simd_and(a, simd_lomask(fw)));
        Value * b_low = fwCast(16, simd_and(b, simd_lomask(fw)));
        return CreateCall(vpackuswbfunc, {a_low, b_low});
    }
    // Otherwise use default SSE logic.
    return IDISA_SSE_Builder::hsimd_packl_in_lanes(lanes, fw, a, b);
}

Value * IDISA_AVX2_Builder::hsimd_packh_in_lanes(unsigned lanes, unsigned fw, Value * a, Value * b) {
    if ((fw == 16)  && (lanes == 2)) {
        Value * vpackuswbfunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx2_packuswb);
        Value * a_low = simd_srli(fw, a, fw/2);
        Value * b_low = simd_srli(fw, b, fw/2);
        return CreateCall(vpackuswbfunc, {a_low, b_low});
    }
    // Otherwise use default SSE logic.
    return IDISA_SSE_Builder::hsimd_packh_in_lanes(lanes, fw, a, b);
}


Value * IDISA_AVX2_Builder::hsimd_packus(unsigned fw, Value * a, Value * b) {
    if (((fw == 32) || (fw == 16)) && (getVectorBitWidth(a) == AVX_width)) {
        Value * pack_func = Intrinsic::getDeclaration(getModule(), fw == 16 ? Intrinsic::x86_avx2_packuswb : Intrinsic::x86_avx2_packusdw);
        Value * packed = fwCast(64, CreateCall(pack_func, {fwCast(fw, a), fwCast(fw, b)}));
        auto field_count = AVX_width/64;
        SmallVector<Constant *, 4> Idxs(field_count);
        for (unsigned int i = 0; i < field_count/2; i++) {
            Idxs[i] = getInt32(2*i);
            Idxs[i + field_count/2] = getInt32(2*i + 1);
        }
        llvm::Constant * shuffleMask = ConstantVector::get(Idxs);
        return bitCast(CreateShuffleVector(packed, UndefValue::get(fwVectorType(64)), shuffleMask));
    }
    // Otherwise use default logic.
    return IDISA_Builder::hsimd_packus(fw, a, b);
}

Value * IDISA_AVX2_Builder::hsimd_packss(unsigned fw, Value * a, Value * b) {
    if (((fw == 32) || (fw == 16)) && (getVectorBitWidth(a) == AVX_width)) {
        Value * pack_func = Intrinsic::getDeclaration(getModule(), fw == 16 ? Intrinsic::x86_avx2_packsswb : Intrinsic::x86_avx2_packssdw);
        Value * packed = fwCast(64, CreateCall(pack_func, {fwCast(fw, a), fwCast(fw, b)}));
        auto field_count = AVX_width/64;
        SmallVector<Constant *, 4> Idxs(field_count);
        for (unsigned int i = 0; i < field_count/2; i++) {
            Idxs[i] = getInt32(2*i);
            Idxs[i + field_count/2] = getInt32(2*i + 1);
        }
        llvm::Constant * shuffleMask = ConstantVector::get(Idxs);
        return bitCast(CreateShuffleVector(packed, UndefValue::get(fwVectorType(64)), shuffleMask));
    }
    // Otherwise use default logic.
    return IDISA_Builder::hsimd_packus(fw, a, b);
}


std::pair<Value *, Value *> IDISA_AVX2_Builder::bitblock_add_with_carry(Value * e1, Value * e2, Value * carryin) {
    // using LONG_ADD
    Type * carryTy = carryin->getType();
    if (carryTy == mBitBlockType) {
        carryin = mvmd_extract(32, carryin, 0);
    }
    Value * carrygen = simd_and(e1, e2);
    Value * carryprop = simd_or(e1, e2);
    Value * digitsum = simd_add(64, e1, e2);
    Value * digitcarry = simd_or(carrygen, simd_and(carryprop, CreateNot(digitsum)));
    Value * carryMask = hsimd_signmask(64, digitcarry);
    Value * carryMask2 = CreateOr(CreateAdd(carryMask, carryMask), carryin);
    Value * bubble = simd_eq(64, digitsum, allOnes());
    Value * bubbleMask = hsimd_signmask(64, bubble);
    Value * incrementMask = CreateXor(CreateAdd(bubbleMask, carryMask2), bubbleMask);
    Value * increments = esimd_bitspread(64,incrementMask);
    Value * sum = simd_add(64, digitsum, increments);
    Value * carry_out = CreateLShr(incrementMask, mBitBlockWidth / 64);
    if (carryTy == mBitBlockType) {
        carry_out = bitCast(CreateZExt(carry_out, getIntNTy(mBitBlockWidth)));
    }
    return std::pair<Value *, Value *>{carry_out, bitCast(sum)};
}

Value * IDISA_AVX2_Builder::simd_pext(unsigned fieldwidth, Value * v, Value * extract_mask) {
    if (hasBMI2 && ((fieldwidth == 64) || (fieldwidth == 32))) {
        Value * PEXT_f = (fieldwidth == 64) ? Intrinsic::getDeclaration(getModule(), Intrinsic::x86_bmi_pext_64)
                                            : Intrinsic::getDeclaration(getModule(), Intrinsic::x86_bmi_pext_32);
        const auto n = getBitBlockWidth() / fieldwidth;
        Value * result = UndefValue::get(fwVectorType(fieldwidth));
        for (unsigned i = 0; i < n; i++) {
            Value * v_i = mvmd_extract(fieldwidth, v, i);
            Value * mask_i = mvmd_extract(fieldwidth, extract_mask, i);
            Value * bits = CreateCall(PEXT_f, {v_i, mask_i});
            result = mvmd_insert(fieldwidth, result, bits, i);
        }
        return bitCast(result);
    }
    return IDISA_Builder::simd_pext(fieldwidth, v, extract_mask);
}

Value * IDISA_AVX2_Builder::simd_pdep(unsigned fieldwidth, Value * v, Value * deposit_mask) {
    if (hasBMI2 && ((fieldwidth == 64) || (fieldwidth == 32))) {
        Value * PDEP_f = (fieldwidth == 64) ? Intrinsic::getDeclaration(getModule(), Intrinsic::x86_bmi_pdep_64)
                                            : Intrinsic::getDeclaration(getModule(), Intrinsic::x86_bmi_pdep_32);
        const auto n = getBitBlockWidth() / fieldwidth;
        Value * result = UndefValue::get(fwVectorType(fieldwidth));
        for (unsigned i = 0; i < n; i++) {
            Value * v_i = mvmd_extract(fieldwidth, v, i);
            Value * mask_i = mvmd_extract(fieldwidth, deposit_mask, i);
            Value * bits = CreateCall(PDEP_f, {v_i, mask_i});
            result = mvmd_insert(fieldwidth, result, bits, i);
        }
        return bitCast(result);
    }
    return IDISA_Builder::simd_pdep(fieldwidth, v, deposit_mask);
}

std::pair<Value *, Value *> IDISA_AVX2_Builder::bitblock_indexed_advance(Value * strm, Value * index_strm, Value * shiftIn, unsigned shiftAmount) {
    const unsigned bitWidth = getSizeTy()->getBitWidth();
    if (hasBMI2 && ((bitWidth == 64) || (bitWidth == 32))) {
        Value * PEXT_f = (bitWidth == 64) ? Intrinsic::getDeclaration(getModule(), Intrinsic::x86_bmi_pext_64)
                                          : Intrinsic::getDeclaration(getModule(), Intrinsic::x86_bmi_pext_32);
        Value * PDEP_f = (bitWidth == 64) ? Intrinsic::getDeclaration(getModule(), Intrinsic::x86_bmi_pdep_64)
                                          : Intrinsic::getDeclaration(getModule(), Intrinsic::x86_bmi_pdep_32);
        Value * const popcount = Intrinsic::getDeclaration(getModule(), Intrinsic::ctpop, getSizeTy());
        Type * iBitBlock = getIntNTy(getBitBlockWidth());
        Value * shiftVal = getSize(shiftAmount);
        const auto n = getBitBlockWidth() / bitWidth;
        VectorType * const vecTy = VectorType::get(getSizeTy(), n);
        if (LLVM_LIKELY(shiftAmount < bitWidth)) {
            Value * carry = mvmd_extract(bitWidth, shiftIn, 0);
            Value * result = UndefValue::get(vecTy);
            for (unsigned i = 0; i < n; i++) {
                Value * s = mvmd_extract(bitWidth, strm, i);
                Value * ix = mvmd_extract(bitWidth, index_strm, i);
                Value * ix_popcnt = CreateCall(popcount, {ix});
                Value * bits = CreateCall(PEXT_f, {s, ix});
                Value * adv = CreateOr(CreateShl(bits, shiftAmount), carry);
                // We have two cases depending on whether the popcount of the index pack is < shiftAmount or not.
                Value * popcount_small = CreateICmpULT(ix_popcnt, shiftVal);
                Value * carry_if_popcount_small =
                    CreateOr(CreateShl(bits, CreateSub(shiftVal, ix_popcnt)),
                                CreateLShr(carry, ix_popcnt));
                Value * carry_if_popcount_large = CreateLShr(bits, CreateSub(ix_popcnt, shiftVal));
                carry = CreateSelect(popcount_small, carry_if_popcount_small, carry_if_popcount_large);
                result = mvmd_insert(bitWidth, result, CreateCall(PDEP_f, {adv, ix}), i);
            }
            Value * carryOut = mvmd_insert(bitWidth, allZeroes(), carry, 0);
            return std::pair<Value *, Value *>{bitCast(carryOut), bitCast(result)};
        }
        else if (shiftAmount <= mBitBlockWidth) {
            // The shift amount is always greater than the popcount of the individual
            // elements that we deal with.   This simplifies some of the logic.
            Value * carry = CreateBitCast(shiftIn, iBitBlock);
            Value * result = UndefValue::get(vecTy);
            for (unsigned i = 0; i < n; i++) {
                Value * s = mvmd_extract(bitWidth, strm, i);
                Value * ix = mvmd_extract(bitWidth, index_strm, i);
                Value * ix_popcnt = CreateCall(popcount, {ix});
                Value * bits = CreateCall(PEXT_f, {s, ix});  // All these bits are shifted out (appended to carry).
                result = mvmd_insert(bitWidth, result, CreateCall(PDEP_f, {mvmd_extract(bitWidth, carry, 0), ix}), i);
                carry = CreateLShr(carry, CreateZExt(ix_popcnt, iBitBlock)); // Remove the carry bits consumed, make room for new bits.
                carry = CreateOr(carry, CreateShl(CreateZExt(bits, iBitBlock), CreateZExt(CreateSub(shiftVal, ix_popcnt), iBitBlock)));
            }
            return std::pair<Value *, Value *>{bitCast(carry), bitCast(result)};
        }
        else {
            // The shift amount is greater than the total popcount.   We will consume popcount
            // bits from the shiftIn value only, and produce a carry out value of the selected bits.
            // elements that we deal with.   This simplifies some of the logic.
            Value * carry = CreateBitCast(shiftIn, iBitBlock);
            Value * result = UndefValue::get(vecTy);
            Value * carryOut = ConstantInt::getNullValue(iBitBlock);
            Value * generated = getSize(0);
            for (unsigned i = 0; i < n; i++) {
                Value * s = mvmd_extract(bitWidth, strm, i);
                Value * ix = mvmd_extract(bitWidth, index_strm, i);
                Value * ix_popcnt = CreateCall(popcount, {ix});
                Value * bits = CreateCall(PEXT_f, {s, ix});  // All these bits are shifted out (appended to carry).
                result = mvmd_insert(bitWidth, result, CreateCall(PDEP_f, {mvmd_extract(bitWidth, carry, 0), ix}), i);
                carry = CreateLShr(carry, CreateZExt(ix_popcnt, iBitBlock)); // Remove the carry bits consumed.
                carryOut = CreateOr(carryOut, CreateShl(CreateZExt(bits, iBitBlock), CreateZExt(generated, iBitBlock)));
                generated = CreateAdd(generated, ix_popcnt);
            }
            return std::pair<Value *, Value *>{bitCast(carryOut), bitCast(result)};
        }
    }
    return IDISA_Builder::bitblock_indexed_advance(strm, index_strm, shiftIn, shiftAmount);
}

Value * IDISA_AVX2_Builder::hsimd_signmask(unsigned fw, Value * a) {
    // AVX2 special cases
    if (mBitBlockWidth == 256) {
        if (fw == 8) {
            Value * signmask_f8func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx2_pmovmskb);
            Type * bitBlock_i8type = VectorType::get(getInt8Ty(), mBitBlockWidth/8);
            Value * a_as_ps = CreateBitCast(a, bitBlock_i8type);
            return CreateCall(signmask_f8func, a_as_ps);
        }
    }
    // Otherwise use default SSE logic.
    return IDISA_AVX_Builder::hsimd_signmask(fw, a);
}

llvm::Value * IDISA_AVX2_Builder::mvmd_srl(unsigned fw, llvm::Value * a, llvm::Value * shift, const bool safe) {
    // Intrinsic::x86_avx2_permd) allows an efficient implementation for field width 32.
    // Translate larger field widths to 32 bits.
    if (fw > 32) {
        return fwCast(fw, mvmd_srl(32, a, CreateMul(shift, ConstantInt::get(shift->getType(), fw/32)), safe));
    }
    if ((mBitBlockWidth == 256) && (fw == 32)) {
        Value * permuteFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx2_permd);
        const unsigned fieldCount = mBitBlockWidth/fw;
        Type * fieldTy = getIntNTy(fw);
        SmallVector<Constant *, 16> indexes(fieldCount);
        for (unsigned int i = 0; i < fieldCount; i++) {
            indexes[i] = ConstantInt::get(fieldTy, i);
        }
        Constant * indexVec = ConstantVector::get(indexes);
        Constant * fieldCountSplat = ConstantVector::getSplat(fieldCount, ConstantInt::get(fieldTy, fieldCount));
        Value * shiftSplat = simd_fill(fw, CreateZExtOrTrunc(shift, fieldTy));
        Value * permuteVec = CreateAdd(indexVec, shiftSplat);
        // Zero out fields that are above the max.
        permuteVec = simd_and(permuteVec, simd_ult(fw, permuteVec, fieldCountSplat));
        // Insert a zero value at position 0 (OK for shifts > 0)
        Value * a0 = mvmd_insert(fw, a, Constant::getNullValue(fieldTy), 0);
        Value * shifted = CreateCall(permuteFunc, {a0, permuteVec});
        return simd_if(1, simd_eq(fw, shiftSplat, allZeroes()), a, shifted);
    }
    return IDISA_Builder::mvmd_srl(fw, a, shift, safe);
}

llvm::Value * IDISA_AVX2_Builder::mvmd_sll(unsigned fw, llvm::Value * a, llvm::Value * shift, const bool safe) {
    // Intrinsic::x86_avx2_permd) allows an efficient implementation for field width 32.
    // Translate larger field widths to 32 bits.
    if (fw > 32) {
        return fwCast(fw, mvmd_sll(32, a, CreateMul(shift, ConstantInt::get(shift->getType(), fw/32)), safe));
    }
    if ((mBitBlockWidth == 256) && (fw == 32)) {
        Value * permuteFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx2_permd);
        const unsigned fieldCount = mBitBlockWidth/fw;
        Type * fieldTy = getIntNTy(fw);
        SmallVector<Constant *, 16> indexes(fieldCount);
        for (unsigned int i = 0; i < fieldCount; i++) {
            indexes[i] = ConstantInt::get(fieldTy, i);
        }
        Constant * indexVec = ConstantVector::get(indexes);
        Value * shiftSplat = simd_fill(fw, CreateZExtOrTrunc(shift, fieldTy));
        Value * permuteVec = CreateSub(indexVec, shiftSplat);
        // Negative indexes are for fields that must be zeroed.  Convert the
        // permute constant to an all ones value, that will select item 7.
        permuteVec = simd_or(permuteVec, simd_lt(fw, permuteVec, fwCast(fw, allZeroes())));
        // Insert a zero value at position 7 (OK for shifts > 0)
        Value * a0 = mvmd_insert(fw, a, Constant::getNullValue(fieldTy), 7);
        Value * shifted = CreateCall(permuteFunc, {a0, permuteVec});
        return simd_if(1, simd_eq(fw, shiftSplat, allZeroes()), a, shifted);
    }
    return IDISA_Builder::mvmd_sll(fw, a, shift, safe);
}


llvm::Value * IDISA_AVX2_Builder::mvmd_shuffle(unsigned fw, llvm::Value * a, llvm::Value * index_vector) {
    if (mBitBlockWidth == 256 && fw > 32) {
        const unsigned fieldCount = mBitBlockWidth/fw;
        // Create a table for shuffling with smaller field widths.
        Constant * idxMask = ConstantVector::getSplat(fieldCount, ConstantInt::get(getIntNTy(fw), fieldCount-1));
        Value * idx = simd_and(index_vector, idxMask);
        unsigned half_fw = fw/2;
        unsigned field_count = mBitBlockWidth/half_fw;
        // Build a ConstantVector of alternating 0 and 1 values.
        SmallVector<Constant *, 16> Idxs(field_count);
        for (unsigned int i = 0; i < field_count; i++) {
            Idxs[i] = ConstantInt::get(getIntNTy(fw/2), i & 1);
        }
        Constant * splat01 = ConstantVector::get(Idxs);
        Value * half_fw_indexes = simd_or(idx, mvmd_slli(half_fw, idx, 1));
        half_fw_indexes = simd_add(fw, simd_add(fw, half_fw_indexes, half_fw_indexes), splat01);
        Value * rslt = mvmd_shuffle(half_fw, a, half_fw_indexes);
        return rslt;
    }
    if (mBitBlockWidth == 256 && fw == 32) {
        Value * shuf32Func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx2_permd);
        return CreateCall(shuf32Func, {fwCast(32, a), fwCast(32, index_vector)});
    }
    return IDISA_Builder::mvmd_shuffle(fw, a, index_vector);
}

llvm::Value * IDISA_AVX2_Builder::mvmd_compress(unsigned fw, llvm::Value * a, llvm::Value * select_mask) {
    if (hasBMI2 && (mBitBlockWidth == 256) && (fw == 64)) {
        Value * PDEP_func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_bmi_pdep_32);
        Value * mask = CreateZExt(select_mask, getInt32Ty());
        Value * mask32 = CreateMul(CreateCall(PDEP_func, {mask, getInt32(0x55)}), getInt32(3));
        Value * result = fwCast(fw, mvmd_compress(32, fwCast(32, a), CreateTrunc(mask32, getInt8Ty())));
        return result;
    }
    if (hasBMI2 && (mBitBlockWidth == 256) && (fw == 32)) {
        Type * v1xi32Ty = VectorType::get(getInt32Ty(), 1);
        Type * v8xi32Ty = VectorType::get(getInt32Ty(), 8);
        Type * v8xi1Ty = VectorType::get(getInt1Ty(), 8);
        Constant * mask0000000Fsplaat = ConstantVector::getSplat(8, ConstantInt::get(getInt32Ty(), 0xF));
        Value * PEXT_func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_bmi_pext_32);
        Value * PDEP_func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_bmi_pdep_32);
        Value * const popcount_func = Intrinsic::getDeclaration(getModule(), Intrinsic::ctpop, getInt32Ty());
        // First duplicate each mask bit to select 4-bit fields
        Value * mask = CreateZExt(select_mask, getInt32Ty());
        Value * field_count = CreateCall(popcount_func, mask);
        Value * spread = CreateCall(PDEP_func, {mask, getInt32(0x11111111)});
        Value * ext_mask = CreateMul(spread, getInt32(0xF));
        // Now extract the 4-bit index values for the required fields.
        Value * indexes = CreateCall(PEXT_func, {getInt32(0x76543210), ext_mask});
        // Broadcast to all fields
        Value * bdcst = CreateShuffleVector(CreateBitCast(indexes, v1xi32Ty),
                                            UndefValue::get(v1xi32Ty),
                                            ConstantVector::getNullValue(v8xi32Ty));
        Constant * Shifts[8];
        for (unsigned int i = 0; i < 8; i++) {
            Shifts[i] = getInt32(i*4);
        }
        Value * shuf = CreateAnd(CreateLShr(bdcst, ConstantVector::get({Shifts, 8})), mask0000000Fsplaat);
        Value * compress = mvmd_shuffle(32, a, shuf);
        Value * field_mask = CreateTrunc(CreateSub(CreateShl(getInt32(1), field_count), getInt32(1)), getInt8Ty());
        Value * result = CreateAnd(compress, CreateSExt(CreateBitCast(field_mask, v8xi1Ty), v8xi32Ty));
        return result;
    }
    return IDISA_Builder::mvmd_compress(fw, a, select_mask);
}

#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 8, 0)

std::string IDISA_AVX512F_Builder::getBuilderUniqueName() {
    return mBitBlockWidth != 512 ? "AVX512F_" + std::to_string(mBitBlockWidth) : "AVX512BW";
}

llvm::Value * IDISA_AVX512F_Builder::hsimd_packh(unsigned fw, llvm::Value * a, llvm::Value * b) {
    if ((mBitBlockWidth == 512) && (fw == 16)) {
        a = fwCast(fw, a);
        a = IDISA_Builder::simd_srli(fw, a, fw/2);
        b = fwCast(fw, b);
        b = IDISA_Builder::simd_srli(fw, b, fw/2);
        return hsimd_packl(fw, a, b);
    }
    return IDISA_Builder::hsimd_packh(fw, a, b);
}

llvm::Value * IDISA_AVX512F_Builder::hsimd_packl(unsigned fw, llvm::Value * a, llvm::Value * b) {
    if ((mBitBlockWidth == 512) && (fw == 16)) {

        const unsigned int field_count = 64;
        Constant * Idxs[field_count];
        for (unsigned int i = 0; i < field_count; i++) {
            Idxs[i] = getInt32(i);
        }
        llvm::Constant * shuffleMask = ConstantVector::get({Idxs, 64});
        Value * a1 = CreateTrunc(fwCast(fw, a), VectorType::get(getInt8Ty(), 32));
        Value * b1 = CreateTrunc(fwCast(fw, b), VectorType::get(getInt8Ty(), 32));
        llvm::Value * c = CreateShuffleVector(a1, b1, shuffleMask);
        c = bitCast(c);
        return c;
    }
    return IDISA_Builder::hsimd_packl(fw, a, b);
}

Value * IDISA_AVX512F_Builder::hsimd_packus(unsigned fw, Value * a, Value * b) {
    if (hostCPUFeatures.hasAVX512BW && ((fw == 16) || (fw == 32)) && (getVectorBitWidth(a) == AVX512_width)) {
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(5, 0, 0)
        Value * pack_func = Intrinsic::getDeclaration(getModule(), fw == 16 ? Intrinsic::x86_avx512_mask_packuswb_512 : Intrinsic::x86_avx512_mask_packusdw_512);
        Constant * mask = Constant::getAllOnesValue(getIntNTy(AVX512_width/(fw/2)));
        Value * packed = CreateCall(pack_func, {fwCast(fw, a), fwCast(fw, b), fwCast(32, allZeroes()), mask});
#else
        Value * pack_func = Intrinsic::getDeclaration(getModule(), fw == 16 ? Intrinsic::x86_avx512_packuswb_512 : Intrinsic::x86_avx512_packusdw_512);
        Value * packed = CreateCall(pack_func, {fwCast(fw, a), fwCast(fw, b)});
#endif
        auto field_count = AVX512_width/64;
        SmallVector<Constant *, 16> Idxs(field_count);
        for (unsigned int i = 0; i < field_count/2; i++) {
            Idxs[i] = getInt32(2*i);
            Idxs[i + field_count/2] = getInt32(2*i + 1);
        }
        llvm::Constant * shuffleMask = ConstantVector::get(Idxs);
        return bitCast(CreateShuffleVector(fwCast(64, packed), UndefValue::get(fwVectorType(64)), shuffleMask));
    }
    // Otherwise use default logic.
    return IDISA_Builder::hsimd_packus(fw, a, b);
}

Value * IDISA_AVX512F_Builder::hsimd_packss(unsigned fw, Value * a, Value * b) {
    if (hostCPUFeatures.hasAVX512BW && ((fw == 16) || (fw == 32)) && (getVectorBitWidth(a) == AVX512_width)) {
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(5, 0, 0)
        Value * pack_func = Intrinsic::getDeclaration(getModule(), fw == 16 ? Intrinsic::x86_avx512_mask_packsswb_512 : Intrinsic::x86_avx512_mask_packssdw_512);
        Constant * mask = Constant::getAllOnesValue(getIntNTy(AVX512_width/(fw/2)));
        Value * packed = CreateCall(pack_func, {fwCast(fw, a), fwCast(fw, b), fwCast(32, allZeroes()), mask});
#else
        Value * pack_func = Intrinsic::getDeclaration(getModule(), fw == 16 ? Intrinsic::x86_avx512_packsswb_512 : Intrinsic::x86_avx512_packssdw_512);
        Value * packed = CreateCall(pack_func, {fwCast(fw, a), fwCast(fw, b)});
#endif
        auto field_count = AVX512_width/64;
        SmallVector<Constant *, 16> Idxs(field_count);
        for (unsigned int i = 0; i < field_count/2; i++) {
            Idxs[i] = getInt32(2*i);
            Idxs[i + field_count/2] = getInt32(2*i + 1);
        }
        llvm::Constant * shuffleMask = ConstantVector::get(Idxs);
        return bitCast(CreateShuffleVector(fwCast(64, packed), UndefValue::get(fwVectorType(64)), shuffleMask));
    }
    // Otherwise use default logic.
    return IDISA_Builder::hsimd_packus(fw, a, b);
}


llvm::Value * IDISA_AVX512F_Builder::esimd_bitspread(unsigned fw, llvm::Value * bitmask) {
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(6, 0, 0)
    const auto field_count = mBitBlockWidth / fw;
    Type * maskTy = VectorType::get(getInt1Ty(), field_count);
    Type * resultTy = fwVectorType(fw);
    return CreateZExt(CreateBitCast(CreateZExtOrTrunc(bitmask, getIntNTy(field_count)), maskTy), resultTy);
#else
    if (mBitBlockWidth == 512 && fw == 64) {
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(7, 0, 0)
        Value * broadcastFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_broadcasti64x4_512);
#else
        Value * broadcastFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_broadcasti64x4_512);
#endif
        Value * broadcastMask = CreateZExtOrTrunc(bitmask, getInt8Ty());

        const unsigned int srcFieldCount = 8;
        Constant * srcArr[srcFieldCount];
        for (unsigned int i = 0; i < srcFieldCount; i++) {
            srcArr[i] = getInt64(0);
        }
        Constant * src = ConstantVector::get({srcArr, srcFieldCount});

        const unsigned int aFieldCount = 4;
        Constant * aArr[aFieldCount];
        for (unsigned int i = 0; i < aFieldCount; i++) {
            aArr[i] = getInt64(1);
        }
        Constant * a = ConstantVector::get({aArr, aFieldCount});

        return CreateCall(broadcastFunc, {a, src, broadcastMask});
    }

    return IDISA_Builder::esimd_bitspread(fw, bitmask);
#endif
}

llvm::Value * IDISA_AVX512F_Builder::mvmd_srl(unsigned fw, llvm::Value * a, llvm::Value * shift, const bool safe) {
    const unsigned fieldCount = mBitBlockWidth/fw;
    Type * fieldTy = getIntNTy(fw);
    SmallVector<Constant *, 16> indexes(fieldCount);
    for (unsigned int i = 0; i < fieldCount; i++) {
        indexes[i] = ConstantInt::get(fieldTy, i);
    }
    Constant * indexVec = ConstantVector::get(indexes);
    Value * permuteVec = CreateAdd(indexVec, simd_fill(fw, CreateZExtOrTrunc(shift, fieldTy)));
    if (mBitBlockWidth == 512) {
        return bitCast(mvmd_shuffle2(fw, fwCast(fw, a), fwCast(fw, allZeroes()), permuteVec));
    }
    return IDISA_Builder::mvmd_srl(fw, a, shift, safe);
}

llvm::Value * IDISA_AVX512F_Builder::mvmd_sll(unsigned fw, llvm::Value * a, llvm::Value * shift, const bool safe) {
    const unsigned fieldCount = mBitBlockWidth/fw;
    Type * fieldTy = getIntNTy(fw);
    SmallVector<Constant *, 16> indexes(fieldCount);
    for (unsigned int i = 0; i < fieldCount; i++) {
        indexes[i] = ConstantInt::get(fieldTy, fieldCount + i);
    }
    Constant * indexVec = ConstantVector::get(indexes);
    Value * permuteVec = CreateSub(indexVec, simd_fill(fw, CreateZExtOrTrunc(shift, fieldTy)));
    if (mBitBlockWidth == 512) {
        return bitCast(mvmd_shuffle2(fw, fwCast(fw, allZeroes()), fwCast(fw, a), permuteVec));
    }
    return IDISA_Builder::mvmd_sll(fw, a, shift);
}

llvm::Value * IDISA_AVX512F_Builder::mvmd_shuffle(unsigned fw, llvm::Value * data_table, llvm::Value * index_vector) {
    if (mBitBlockWidth == 512 && ((fw == 64) || (fw == 32) | (fw == 16))) {
        return mvmd_shuffle2(fw, data_table, data_table, index_vector);
    }
    return IDISA_Builder::mvmd_shuffle(fw, data_table, index_vector);
}


#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(7, 0, 0)
#define AVX512_MASK_PERMUTE_INTRINSIC(i) Intrinsic::x86_avx512_mask_vpermt2##i
#else
#define AVX512_MASK_PERMUTE_INTRINSIC(i) Intrinsic::x86_avx512_vpermi2##i
#endif

llvm::Value * IDISA_AVX512F_Builder::mvmd_shuffle2(unsigned fw, Value * table0, llvm::Value * table1, llvm::Value * index_vector) {
    if (mBitBlockWidth == 512) {
        Value * permuteFunc = nullptr;
        if (mBitBlockWidth == 512 && fw == 32) {
            permuteFunc = Intrinsic::getDeclaration(getModule(), AVX512_MASK_PERMUTE_INTRINSIC(var_d_512));
        }
        if (mBitBlockWidth == 512 && fw == 64) {
            permuteFunc = Intrinsic::getDeclaration(getModule(), AVX512_MASK_PERMUTE_INTRINSIC(var_q_512));
        }
        if (mBitBlockWidth == 512 && fw == 16 && hostCPUFeatures.hasAVX512BW) {
            permuteFunc = Intrinsic::getDeclaration(getModule(), AVX512_MASK_PERMUTE_INTRINSIC(var_hi_512));
        }
        if (permuteFunc) {
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(7, 0, 0)
            const unsigned fieldCount = mBitBlockWidth/fw;
            Constant * mask = ConstantInt::getAllOnesValue(getIntNTy(fieldCount));
            return CreateCall(permuteFunc, {fwCast(fw, index_vector), fwCast(fw, table0), fwCast(fw, table1), mask});
#else
            return CreateCall(permuteFunc, {fwCast(fw, table0), fwCast(fw, index_vector), fwCast(fw, table1)});
#endif
        }
    }
    return IDISA_Builder::mvmd_shuffle2(fw, table0, table1, index_vector);
}
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(9, 0, 0)
#define AVX512_MASK_COMPRESS_INTRINSIC_64 Intrinsic::x86_avx512_mask_compress_q_512
#define AVX512_MASK_COMPRESS_INTRINSIC_32 Intrinsic::x86_avx512_mask_compress_d_512
#else
#define AVX512_MASK_COMPRESS_INTRINSIC_64 Intrinsic::x86_avx512_mask_compress
#define AVX512_MASK_COMPRESS_INTRINSIC_32 Intrinsic::x86_avx512_mask_compress
#endif

llvm::Value * IDISA_AVX512F_Builder::mvmd_compress(unsigned fw, llvm::Value * a, llvm::Value * select_mask) {
    unsigned fieldCount = mBitBlockWidth/fw;
    Value * mask = CreateZExtOrTrunc(select_mask, getIntNTy(fieldCount));
    if (mBitBlockWidth == 512 && fw == 32) {
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(9, 0, 0)
        Value * compressFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_compress_d_512);
        return CreateCall(compressFunc, {fwCast(32, a), fwCast(32, allZeroes()), mask});
#else
        Type * maskTy = VectorType::get(getInt1Ty(), fieldCount);
        Value * compressFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_compress, fwVectorType(fw));
        return CreateCall(compressFunc, {fwCast(32, a), fwCast(32, allZeroes()), CreateBitCast(mask, maskTy)});
#endif
    }
    if (mBitBlockWidth == 512 && fw == 64) {
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(9, 0, 0)
        Value * compressFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_compress_q_512);
        return CreateCall(compressFunc, {fwCast(64, a), fwCast(64, allZeroes()), mask});
#else
        Type * maskTy = VectorType::get(getInt1Ty(), fieldCount);
        Value * compressFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_compress, fwVectorType(fw));
        return CreateCall(compressFunc, {fwCast(64, a), fwCast(64, allZeroes()), CreateBitCast(mask, maskTy)});
#endif
    }
    return IDISA_Builder::mvmd_compress(fw, a, select_mask);
}

Value * IDISA_AVX512F_Builder:: mvmd_slli(unsigned fw, llvm::Value * a, unsigned shift) {
    if (shift == 0) return a;
    if (fw > 32) {
        return mvmd_slli(32, a, shift * (fw/32));
    } else if (((shift % 2) == 0) && (fw < 32)) {
        return mvmd_slli(2 * fw, a, shift / 2);
    }
    if ((fw == 32) || (hostCPUFeatures.hasAVX512BW && (fw == 16)))   {
        return mvmd_dslli(fw, a, allZeroes(), shift);
    } else {
        unsigned field32_shift = (shift * fw) / 32;
        unsigned bit_shift = (shift * fw) % 32;
        return simd_or(simd_slli(32, mvmd_slli(32, a, field32_shift), bit_shift),
                       simd_srli(32, mvmd_slli(32, a, field32_shift + 1), 32-bit_shift));
    }
}

Value * IDISA_AVX512F_Builder:: mvmd_dslli(unsigned fw, llvm::Value * a, llvm::Value * b, unsigned shift) {
    if (shift == 0) return a;
    if (fw > 32) {
        return mvmd_dslli(32, a, b, shift * (fw/32));
    } else if (((shift % 2) == 0) && (fw < 32)) {
        return mvmd_dslli(2 * fw, a, b, shift / 2);
    }
    const unsigned fieldCount = mBitBlockWidth/fw;
    if ((fw == 32) || (hostCPUFeatures.hasAVX512BW && (fw == 16)))   {
        Type * fwTy = getIntNTy(fw);
        SmallVector<Constant *, 16> indices(fieldCount);
        for (unsigned i = 0; i < fieldCount; i++) {
            indices[i] = ConstantInt::get(fwTy, i + fieldCount - shift);
        }
        return bitCast(mvmd_shuffle2(fw, fwCast(fw, b), fwCast(fw, a), ConstantVector::get(indices)));

    } else {
        unsigned field32_shift = (shift * fw) / 32;
        unsigned bit_shift = (shift * fw) % 32;
        return simd_or(simd_slli(32, mvmd_dslli(32, a, b, field32_shift), bit_shift),
                       simd_srli(32, mvmd_dslli(32, a, b, field32_shift + 1), 32-bit_shift));
    }
}

llvm::Value * IDISA_AVX512F_Builder::simd_popcount(unsigned fw, llvm::Value * a) {
     if (fw == 512) {
         Constant * zero16xi8 = Constant::getNullValue(VectorType::get(getInt8Ty(), 16));
         Constant * zeroInt32 = Constant::getNullValue(getInt32Ty());
         Value * c = simd_popcount(64, a);
         //  Should probably use _mm512_reduce_add_epi64, but not found in LLVM 3.8
         Value * pack64_8_func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_pmov_qb_512);
         // popcounts of 64 bit fields will always fit in 8 bit fields.
         // We don't need the masked version of this, but the unmasked intrinsic was not found.
         c = CreateCall(pack64_8_func, {c, zero16xi8, Constant::getAllOnesValue(getInt8Ty())});
         Value * horizSADfunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_sse2_psad_bw);
         c = CreateCall(horizSADfunc, {c, zero16xi8});
         return CreateInsertElement(allZeroes(), CreateExtractElement(c, zeroInt32), zeroInt32);
    }
    if (hostCPUFeatures.hasAVX512VPOPCNTDQ && (fw == 32 || fw == 64)){
        //llvm should use vpopcntd or vpopcntq instructions
        return CreatePopcount(fwCast(fw, a));
    }
    if (hostCPUFeatures.hasAVX512BW && (fw == 64)) {
        Value * horizSADfunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_psad_bw_512);
        return bitCast(CreateCall(horizSADfunc, {fwCast(8, simd_popcount(8, a)), fwCast(8, allZeroes())}));
    }
    //https://en.wikipedia.org/wiki/Hamming_weight#Efficient_implementation
    if((fw == 64) && (mBitBlockWidth == 512)){
        Constant * m1Arr[8];
        llvm::Constant * m1;
        for (unsigned int i = 0; i < 8; i++) {
            m1Arr[i] = getInt64(0x5555555555555555);
        }
        m1 = ConstantVector::get({m1Arr, 8});

        Constant * m2Arr[8];
        llvm::Constant * m2;
        for (unsigned int i = 0; i < 8; i++) {
            m2Arr[i] = getInt64(0x3333333333333333);
        }
        m2 = ConstantVector::get({m2Arr, 8});

        Constant * m4Arr[8];
        llvm::Constant * m4;
        for (unsigned int i = 0; i < 8; i++) {
            m4Arr[i] = getInt64(0x0f0f0f0f0f0f0f0f);
        }
        m4 = ConstantVector::get({m4Arr, 8});

        Constant * h01Arr[8];
        llvm::Constant * h01;
        for (unsigned int i = 0; i < 8; i++) {
            h01Arr[i] = getInt64(0x0101010101010101);
        }
        h01 = ConstantVector::get({h01Arr, 8});

        a = simd_sub(fw, a, simd_and(simd_srli(fw, a, 1), m1));
        a = simd_add(fw, simd_and(a, m2), simd_and(simd_srli(fw, a, 2), m2));
        a = simd_and(simd_add(fw, a, simd_srli(fw, a, 4)), m4);
        return simd_srli(fw, simd_mult(fw, a, h01), 56);

    }
    return IDISA_Builder::simd_popcount(fw, a);
}

llvm::Value * IDISA_AVX512F_Builder::hsimd_signmask(unsigned fw, llvm::Value * a) {
    //IDISA_Builder::hsimd_signmask outperforms IDISA_AVX2_Builder::hsimd_signmask
    //when run with BlockSize=512
    return IDISA_Builder::hsimd_signmask(fw, a);
}

Value * IDISA_AVX512F_Builder::esimd_mergeh(unsigned fw, Value * a, Value * b) {
    if (hostCPUFeatures.hasAVX512BW && ((fw == 1) || (fw == 2))) {
        // Bit interleave using shuffle.
        // Make a shuffle table that translates the lower 4 bits of each byte in
        // order to spread out the bits: xxxxdcba => .d.c.b.a
        // We use two copies of the table for the AVX2 _mm256_shuffle_epi8
        Constant * interleave_table = bit_interleave_byteshuffle_table(fw);
        // Merge the bytes.
        Value * byte_merge = esimd_mergeh(8, a, b);
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(4, 0, 0)
        Value * shufFn = Intrinsic::getDeclaration(getModule(),  Intrinsic::x86_avx512_mask_pshuf_b_512);
        // Make a shuffle table that translates the lower 4 bits of each byte in
        // order to spread out the bits: xxxxdcba => .d.c.b.a
        // We use two copies of the table for the AVX2 _mm256_shuffle_epi8
        Value * zeroByteSplat = fwCast(8, allZeroes());
        Constant * mask = ConstantInt::getAllOnesValue(getInt64Ty());
        Value * low_bits = CreateCall(shufFn, {interleave_table, fwCast(8, simd_and(byte_merge, simd_lomask(8))), zeroByteSplat, mask});
        Value * high_bits = simd_slli(16, CreateCall(shufFn, {interleave_table, fwCast(8, simd_srli(8, byte_merge, 4)), zeroByteSplat, mask}), fw);
#else
        Value * shufFn = Intrinsic::getDeclaration(getModule(),  Intrinsic::x86_avx512_pshuf_b_512);
        Value * low_bits = CreateCall(shufFn, {interleave_table, fwCast(8, simd_and(byte_merge, simd_lomask(8)))});
        Value * high_bits = simd_slli(16, CreateCall(shufFn, {interleave_table, fwCast(8, simd_srli(8, byte_merge, 4))}), fw);
#endif
        Value * lo_move_back = simd_srli(16, low_bits, 8-fw);
        Value * hi_move_fwd = simd_slli(16, high_bits, 8-fw);
        return simd_or(simd_if(1, simd_himask(16), high_bits, low_bits), simd_or(lo_move_back, hi_move_fwd));
    }
    if (fw == 8)   {
        const unsigned fieldCount = mBitBlockWidth/fw;
        SmallVector<Constant *, 8> Idxs(fieldCount/2);
        for (unsigned i = 0; i < fieldCount / 2; i++) {
            Idxs[i] = getInt32(i+fieldCount/2); // selects elements from first reg.
        }
        Constant * high_indexes = ConstantVector::get(Idxs);
        Value * a_high = CreateShuffleVector(fwCast(8, a), UndefValue::get(fwVectorType(8)), high_indexes);
        Value * b_high = CreateShuffleVector(fwCast(8, b), UndefValue::get(fwVectorType(8)), high_indexes);
        Value * a_ext = CreateZExt(a_high, fwVectorType(16));
        Value * b_ext = CreateZExt(b_high, fwVectorType(16));
        Value * rslt = simd_or(a_ext, simd_slli(16, b_ext, 8));
        return rslt;
    }
    // Otherwise use default AVX2 logic.
    return IDISA_AVX2_Builder::esimd_mergeh(fw, a, b);
}

Value * IDISA_AVX512F_Builder::esimd_mergel(unsigned fw, Value * a, Value * b) {
    if (hostCPUFeatures.hasAVX512BW && ((fw == 1) || (fw == 2))) {
        // Bit interleave using shuffle.
        // Make a shuffle table that translates the lower 4 bits of each byte in
        // order to spread out the bits: xxxxdcba => .d.c.b.a
        // We use two copies of the table for the AVX2 _mm256_shuffle_epi8
        Constant * interleave_table = bit_interleave_byteshuffle_table(fw);
        // Merge the bytes.
        Value * byte_merge = esimd_mergel(8, a, b);

#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(4, 0, 0)
        Value * shufFn = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_pshuf_b_512);
        // Make a shuffle table that translates the lower 4 bits of each byte in
        // order to spread out the bits: xxxxdcba => .d.c.b.a
        // We use two copies of the table for the AVX2 _mm256_shuffle_epi8
        Value * zeroByteSplat = fwCast(8, allZeroes());
        Constant * mask = ConstantInt::getAllOnesValue(getInt64Ty());
        Value * low_bits = CreateCall(shufFn, {interleave_table, fwCast(8, simd_and(byte_merge, simd_lomask(8))), zeroByteSplat, mask});
        Value * high_bits = simd_slli(16, CreateCall(shufFn, {interleave_table, fwCast(8, simd_srli(8, byte_merge, 4)), zeroByteSplat, mask}), fw);
#else
        Value * shufFn = Intrinsic::getDeclaration(getModule(),  Intrinsic::x86_avx512_pshuf_b_512);
        Value * low_bits = CreateCall(shufFn, {interleave_table, fwCast(8, simd_and(byte_merge, simd_lomask(8)))});
        Value * high_bits = simd_slli(16, CreateCall(shufFn, {interleave_table, fwCast(8, simd_srli(8, byte_merge, 4))}), fw);
#endif
        Value * lo_move_back = simd_srli(16, low_bits, 8-fw);
        Value * hi_move_fwd = simd_slli(16, high_bits, 8-fw);
        return simd_or(simd_if(1, simd_himask(16), high_bits, low_bits), simd_or(lo_move_back, hi_move_fwd));
    }
    if (fw == 8) {
        const unsigned fieldCount = mBitBlockWidth/fw;
        SmallVector<Constant *, 8> Idxs(fieldCount/2);
        for (unsigned i = 0; i < fieldCount / 2; i++) {
            Idxs[i] = getInt32(i); // selects elements from first reg.
        }
        Constant * low_indexes = ConstantVector::get(Idxs);
        Value * a_low = CreateShuffleVector(fwCast(8, a), UndefValue::get(fwVectorType(8)), low_indexes);
        Value * b_low = CreateShuffleVector(fwCast(8, b), UndefValue::get(fwVectorType(8)), low_indexes);
        Value * a_ext = CreateZExt(a_low, fwVectorType(16));
        Value * b_ext = CreateZExt(b_low, fwVectorType(16));
        Value * rslt = simd_or(a_ext, simd_slli(16, b_ext, 8));
        return rslt;
    }
    // Otherwise use default AVX2 logic.
    return IDISA_AVX2_Builder::esimd_mergel(fw, a, b);
}

Value * IDISA_AVX512F_Builder::simd_if(unsigned fw, Value * cond, Value * a, Value * b) {
    if (fw == 1) {
        // Form the 8-bit table for simd-if based on the bitwise values from cond, a and b.
        //   (cond, a, b) =  (111), (110), (101), (100), (011), (010), (001), (000)
        // if(cond, a, b) =    1      1      0      0      1      0      1      0    = 0xCA
        return simd_ternary(0xCA, cond, a, b);
    }
    return IDISA_AVX2_Builder::simd_if(fw, cond, a, b);
}

Value * IDISA_AVX512F_Builder::simd_ternary(unsigned char mask, Value * a, Value * b, Value * c) {
    if (mask == 0) return allZeroes();
    else if (mask == 0xFF) return allOnes();

    unsigned char not_a_mask = mask & 0x0F;
    unsigned char a_mask = (mask >> 4) & 0x0F;
    if (a_mask == not_a_mask) return IDISA_AVX2_Builder::simd_binary(a_mask, b, c);

    unsigned char b_mask = ((mask & 0xC0) >> 4) | ((mask & 0x0C) >> 2);
    unsigned char not_b_mask = ((mask & 0x30) >> 2) | (mask & 0x03);
    if (b_mask == not_b_mask) return IDISA_AVX2_Builder::simd_binary(b_mask, a, c);

    unsigned char c_mask = ((mask & 0x80) >> 4) | ((mask & 0x20) >> 3) | ((mask & 0x08) >> 2) | ((mask & 02) >> 1);
    unsigned char not_c_mask = ((mask & 0x40) >> 3) | ((mask & 0x10) >> 2) | ((mask & 0x04) >> 1) | (mask & 01);
    if (c_mask == not_c_mask) return IDISA_AVX2_Builder::simd_binary(c_mask, a, b);

    Constant * simd_mask = ConstantInt::get(getInt32Ty(), mask);
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(7, 0, 0)
    Value * ternLogicFn = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_pternlog_d_512);
    Constant * writemask = ConstantInt::getAllOnesValue(getInt16Ty());
    Value * args[5] = {fwCast(32, a), fwCast(32, b), fwCast(32, c), simd_mask, writemask};
#else
    Value * ternLogicFn = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_pternlog_d_512);
    Value * args[4] = {fwCast(32, a), fwCast(32, b), fwCast(32, c), simd_mask};
#endif
    Value * rslt = CreateCall(ternLogicFn, args);
    return bitCast(rslt);
}

std::pair<Value *, Value *> IDISA_AVX512F_Builder::bitblock_advance(Value * a, Value * shiftin, unsigned shift) {
    if (shift == 1 && shiftin->getType() == getInt8Ty()) {
        const uint32_t fw = 64;
        Value * const ci_mask = CreateBitCast(shiftin, VectorType::get(getInt1Ty(), 8));
        Value * const v8xi64_1 = simd_fill(fw, ConstantInt::get(getInt64Ty(), 0x8000000000000000));
        Value * const ecarry_in = CreateSelect(ci_mask, v8xi64_1, Constant::getNullValue(VectorType::get(getInt64Ty(), 8)));
        Value * const a1 = mvmd_dslli(fw, a, ecarry_in, shift);
        Value * const result = simd_or(CreateLShr(a1, fw - shift), CreateShl(fwCast(fw, a), shift));

        std::vector<Constant *> v(8, ConstantInt::get(getInt64Ty(), (uint64_t) -1));
        v[0] = ConstantInt::get(getInt64Ty(), 0x7fffffffffffffff);
        Value * const v8xi64_cout_mask = ConstantVector::get(ArrayRef<Constant *>(v));
        Value * shiftout = CreateICmpUGT(a, v8xi64_cout_mask);
        shiftout = CreateBitCast(shiftout, getInt8Ty());
        CallPrintRegister("result", result);
        CallPrintInt("shiftout", shiftout);
        return std::make_pair(shiftout, result);
    } else {
        return IDISA_AVX2_Builder::bitblock_advance(a, shiftin, shift);
    }
}

void IDISA_AVX512F_Builder::getAVX512Features() {
    llvm::StringMap<bool> features;
    if (llvm::sys::getHostCPUFeatures(features)) {
        hostCPUFeatures.hasAVX512CD = features.lookup("avx512cd");
        hostCPUFeatures.hasAVX512BW = features.lookup("avx512bw");
        hostCPUFeatures.hasAVX512DQ = features.lookup("avx512dq");
        hostCPUFeatures.hasAVX512VL = features.lookup("avx512vl");

        //hostCPUFeatures.hasAVX512VBMI, hostCPUFeatures.hasAVX512VBMI2,
        //hostCPUFeatures.hasAVX512VPOPCNTDQ have not been tested as we
        //did not have hardware support. It should work in theory (tm)

        hostCPUFeatures.hasAVX512VBMI = features.lookup("avx512_vbmi");
        hostCPUFeatures.hasAVX512VBMI2 = features.lookup("avx512_vbmi2");
        hostCPUFeatures.hasAVX512VPOPCNTDQ = features.lookup("avx512_vpopcntdq");
    }
}
#endif


}
