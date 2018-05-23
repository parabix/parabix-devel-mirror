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
    // Otherwise use default SSE logic.
    return IDISA_SSE_Builder::hsimd_signmask(fw, a);
}

std::string IDISA_AVX2_Builder::getBuilderUniqueName() {
    return mBitBlockWidth != 256 ? "AVX2_" + std::to_string(mBitBlockWidth) : "AVX2";
}

Value * IDISA_AVX2_Builder::hsimd_packh(unsigned fw, Value * a, Value * b) {
    if ((fw > 8) && (fw <= 64)) {
        Value * aVec = fwCast(fw / 2, a);
        Value * bVec = fwCast(fw / 2, b);
        const auto field_count = 2 * mBitBlockWidth / fw;
        Constant * Idxs[field_count];
        const auto H = (field_count / 2);
        const auto Q = (field_count / 4);
        for (unsigned i = 0; i < Q; i++) {
            Idxs[i] = getInt32(2 * i);
            Idxs[i + Q] = getInt32((2 * i) + 1);
            Idxs[i + H] = getInt32((2 * i) + H);
            Idxs[i + H + Q] = getInt32((2 * i) + 1 + H);
        }
        Value * shufa = CreateShuffleVector(aVec, aVec, ConstantVector::get({Idxs, field_count}));
        Value * shufb = CreateShuffleVector(bVec, bVec, ConstantVector::get({Idxs, field_count}));
        return hsimd_packh(mBitBlockWidth / 2, shufa, shufb);
    }
    // Otherwise use default SSE logic.
    return IDISA_SSE_Builder::hsimd_packh(fw, a, b);
}

Value * IDISA_AVX2_Builder::hsimd_packl(unsigned fw, Value * a, Value * b) {
    if ((fw > 8) && (fw <= 64)) {
        Value * aVec = fwCast(fw / 2, a);
        Value * bVec = fwCast(fw / 2, b);
        const auto field_count = 2 * mBitBlockWidth / fw;
        Constant * Idxs[field_count];
        const auto H = (field_count / 2);
        const auto Q = (field_count / 4);
        for (unsigned i = 0; i < Q; i++) {
            Idxs[i] = getInt32(2 * i);
            Idxs[i + Q] = getInt32((2 * i) + 1);
            Idxs[i + H] = getInt32((2 * i) + H);
            Idxs[i + H + Q] = getInt32((2 * i) + H + 1);
        }
        Value * shufa = CreateShuffleVector(aVec, aVec, ConstantVector::get({Idxs, field_count}));
        Value * shufb = CreateShuffleVector(bVec, bVec, ConstantVector::get({Idxs, field_count}));
        return hsimd_packl(mBitBlockWidth / 2, shufa, shufb);
    }
    // Otherwise use default SSE logic.
    return IDISA_SSE_Builder::hsimd_packl(fw, a, b);
}

Value * IDISA_AVX2_Builder::esimd_mergeh(unsigned fw, Value * a, Value * b) {
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(6, 0, 0)
    if ((fw == 128) && (mBitBlockWidth == 256)) {
        Value * vperm2i128func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx2_vperm2i128);
        return CreateCall(vperm2i128func, {fwCast(64, a), fwCast(64, b), getInt8(0x31)});
    }
#endif
    // Otherwise use default SSE logic.
    return IDISA_SSE_Builder::esimd_mergeh(fw, a, b);
}

Value * IDISA_AVX2_Builder::esimd_mergel(unsigned fw, Value * a, Value * b) {
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(6, 0, 0)
    if ((fw == 128) && (mBitBlockWidth == 256)) {
        Value * vperm2i128func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx2_vperm2i128);
        return CreateCall(vperm2i128func, {fwCast(64, a), fwCast(64, b), getInt8(0x20)});
    }
#endif
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
    if ((fieldwidth == 64) || (fieldwidth == 32)) {
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
    if ((fieldwidth == 64) || (fieldwidth == 32)) {
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
    if ((bitWidth == 64) || (bitWidth == 32)) {
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

llvm::Value * IDISA_AVX2_Builder::mvmd_srl(unsigned fw, llvm::Value * a, llvm::Value * shift) {
    // Intrinsic::x86_avx2_permd) allows an efficient implementation for field width 32.
    // Translate larger field widths to 32 bits.
    if (fw > 32) {
        return fwCast(fw, mvmd_srl(32, a, CreateMul(shift, ConstantInt::get(shift->getType(), fw/32))));
    }
    if ((mBitBlockWidth == 256) && (fw == 32)) {
        Value * permuteFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx2_permd);
        const unsigned fieldCount = mBitBlockWidth/fw;
        Type * fieldTy = getIntNTy(fw);
        Constant * indexes[fieldCount];
        for (unsigned int i = 0; i < fieldCount; i++) {
            indexes[i] = ConstantInt::get(fieldTy, i);
        }
        Constant * indexVec = ConstantVector::get({indexes, fieldCount});
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
    return IDISA_Builder::mvmd_srl(fw, a, shift);
}

llvm::Value * IDISA_AVX2_Builder::mvmd_sll(unsigned fw, llvm::Value * a, llvm::Value * shift) {
    // Intrinsic::x86_avx2_permd) allows an efficient implementation for field width 32.
    // Translate larger field widths to 32 bits.
    if (fw > 32) {
        return fwCast(fw, mvmd_sll(32, a, CreateMul(shift, ConstantInt::get(shift->getType(), fw/32))));
    }
    if ((mBitBlockWidth == 256) && (fw == 32)) {
        Value * permuteFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx2_permd);
        const unsigned fieldCount = mBitBlockWidth/fw;
        Type * fieldTy = getIntNTy(fw);
        Constant * indexes[fieldCount];
        for (unsigned int i = 0; i < fieldCount; i++) {
            indexes[i] = ConstantInt::get(fieldTy, i);
        }
        Constant * indexVec = ConstantVector::get({indexes, fieldCount});
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
    return IDISA_Builder::mvmd_sll(fw, a, shift);
}

    
llvm::Value * IDISA_AVX2_Builder::mvmd_shuffle(unsigned fw, llvm::Value * a, llvm::Value * shuffle_table) {
    if (mBitBlockWidth == 256 && fw > 32) {
        // Create a table for shuffling with smaller field widths.
        unsigned half_fw = fw/2;
        unsigned field_count = mBitBlockWidth/half_fw;
        // Build a ConstantVector of alternating 0 and 1 values.
        Constant * Idxs[field_count];
        for (unsigned int i = 0; i < field_count; i++) {
            Idxs[i] = getInt32(i & 1);
        }
        Constant * splat01 = ConstantVector::get({Idxs, field_count});
        Value * half_shuffle_table = simd_or(shuffle_table, mvmd_slli(half_fw, shuffle_table, 1));
        half_shuffle_table = simd_add(fw, simd_add(fw, half_shuffle_table, half_shuffle_table), splat01);
        Value * rslt = mvmd_shuffle(half_fw, a, half_shuffle_table);
        return rslt;
    }
    if (mBitBlockWidth == 256 && fw == 32) {
        Value * shuf32Func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx2_permd);
        return CreateCall(shuf32Func, {fwCast(32, a), fwCast(32, shuffle_table)});
    }
    return IDISA_Builder::mvmd_shuffle(fw, a, shuffle_table);
}

llvm::Value * IDISA_AVX2_Builder::mvmd_compress(unsigned fw, llvm::Value * a, llvm::Value * select_mask) {
    if (mBitBlockWidth == 256 && fw == 64) {
        Value * PDEP_func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_bmi_pdep_32);
        Value * mask = CreateZExt(select_mask, getInt32Ty());
        Value * mask32 = CreateMul(CreateCall(PDEP_func, {mask, getInt32(0x55)}), getInt32(3));
        Value * result = fwCast(fw, mvmd_compress(32, fwCast(32, a), CreateTrunc(mask32, getInt8Ty())));
        return result;
    }
    if (mBitBlockWidth == 256 && fw == 32) {
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

        const unsigned int field_count = 64;
        Constant * Idxs[field_count];

        for (unsigned int i = 0; i < field_count; i++) {
            Idxs[i] = getInt32(i);
        }

        llvm::Value * pmovfunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_pmov_wb_512);
        llvm::Value * mask = getInt32(-1);
        llvm::Constant * shuffleMask = ConstantVector::get({Idxs, 64});
        llvm::Constant * src = UndefValue::get(VectorType::get(getInt8Ty(), 32));

        a = fwCast(fw, a);
        a = IDISA_Builder::simd_srli(fw, a, fw/2);
        a = CreateCall(pmovfunc, {a, src, mask});
        b = fwCast(fw, b);
        b = IDISA_Builder::simd_srli(fw, b, fw/2);
        b = CreateCall(pmovfunc, {b, src, mask});

        llvm::Value * c = CreateShuffleVector(a, b, shuffleMask);
        c = bitCast(c);
        return c;
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

        llvm::Value * pmovfunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_pmov_wb_512);
        llvm::Value * mask = getInt32(-1);
        llvm::Constant * shuffleMask = ConstantVector::get({Idxs, 64});
        llvm::Constant * src = UndefValue::get(VectorType::get(getInt8Ty(), 32));
        a = fwCast(fw, a);
        a = CreateCall(pmovfunc, {a, src, mask});
        b = fwCast(fw, b);
        b = CreateCall(pmovfunc, {b, src, mask});

        llvm::Value * c = CreateShuffleVector(a, b, shuffleMask);
        c = bitCast(c);
        return c;
    }
    return IDISA_Builder::hsimd_packl(fw, a, b);
}

llvm::Value * IDISA_AVX512F_Builder::esimd_bitspread(unsigned fw, llvm::Value * bitmask) {
    
    if (mBitBlockWidth == 512 && fw == 64) {
        Value * broadcastFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_broadcasti64x4_512);
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
}

llvm::Value * IDISA_AVX512F_Builder::mvmd_srl(unsigned fw, llvm::Value * a, llvm::Value * shift) {
    const unsigned fieldCount = mBitBlockWidth/fw;
    Type * fieldTy = getIntNTy(fw);
    Constant * indexes[fieldCount];
    for (unsigned int i = 0; i < fieldCount; i++) {
        indexes[i] = ConstantInt::get(fieldTy, i);
    }
    Constant * indexVec = ConstantVector::get({indexes, fieldCount});
    Value * permuteVec = CreateAdd(indexVec, simd_fill(fw, CreateZExtOrTrunc(shift, fieldTy)));
    Constant * mask = ConstantInt::getAllOnesValue(getIntNTy(fieldCount));
    if (mBitBlockWidth == 512) {
        Value * permuteFunc = nullptr;
        if (fw == 64) permuteFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_vpermt2var_q_512);
        else if (fw == 32) permuteFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_vpermt2var_d_512);
        if (permuteFunc) {
            Value * shifted = CreateCall(permuteFunc, {permuteVec, fwCast(fw, a), fwCast(fw, allZeroes()), mask});
            return shifted;
        }
    }
    return IDISA_Builder::mvmd_srl(fw, a, shift);
}
 
llvm::Value * IDISA_AVX512F_Builder::mvmd_sll(unsigned fw, llvm::Value * a, llvm::Value * shift) {
    const unsigned fieldCount = mBitBlockWidth/fw;
    Type * fieldTy = getIntNTy(fw);
    Constant * indexes[fieldCount];
    for (unsigned int i = 0; i < fieldCount; i++) {
        indexes[i] = ConstantInt::get(fieldTy, fieldCount + i);
    }
    Constant * indexVec = ConstantVector::get({indexes, fieldCount});
    Value * permuteVec = CreateSub(indexVec, simd_fill(fw, CreateZExtOrTrunc(shift, fieldTy)));
    Constant * mask = ConstantInt::getAllOnesValue(getIntNTy(fieldCount));
    if (mBitBlockWidth == 512) {
        Value * permuteFunc = nullptr;
        if (fw == 64) permuteFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_vpermt2var_q_512);
        else if (fw == 32) permuteFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_vpermt2var_d_512);
        if (permuteFunc) {
            Value * shifted = CreateCall(permuteFunc, {permuteVec, fwCast(fw, allZeroes()), fwCast(fw, a), mask});
            return shifted;
        }
    }
    return IDISA_Builder::mvmd_sll(fw, a, shift);
}

llvm::Value * IDISA_AVX512F_Builder::mvmd_shuffle(unsigned fw, llvm::Value * a, llvm::Value * shuffle_table) {
    const unsigned fieldCount = mBitBlockWidth/fw;
    if (mBitBlockWidth == 512 && fw == 32) {
        Value * permuteFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_vpermt2var_d_512);
        Constant * mask = ConstantInt::getAllOnesValue(getIntNTy(fieldCount));
        return CreateCall(permuteFunc, {fwCast(fw, shuffle_table), fwCast(fw, a), fwCast(fw, a), mask});
    }
    if (mBitBlockWidth == 512 && fw == 64) {
        Value * permuteFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_vpermt2var_q_512);
        Constant * mask = ConstantInt::getAllOnesValue(getIntNTy(fieldCount));
        return CreateCall(permuteFunc, {fwCast(fw, shuffle_table), fwCast(fw, a), fwCast(fw, a), mask});
    }
    if (mBitBlockWidth == 512 && fw == 16 && hostCPUFeatures.hasAVX512BW) {
        Value * permuteFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_maskz_vpermt2var_hi_512);
        Constant * mask = ConstantInt::getAllOnesValue(getIntNTy(fieldCount));
        return CreateCall(permuteFunc, {fwCast(fw, shuffle_table), fwCast(fw, a), fwCast(fw, a), mask});
    }
    return IDISA_Builder::mvmd_shuffle(fw, a, shuffle_table);
}

llvm::Value * IDISA_AVX512F_Builder::mvmd_shuffle2(unsigned fw, Value * a, Value * b, llvm::Value * shuffle_table) {
    const unsigned fieldCount = mBitBlockWidth/fw;
    if (mBitBlockWidth == 512 && fw == 32) {
        Value * permuteFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_vpermt2var_d_512);
        Constant * mask = ConstantInt::getAllOnesValue(getIntNTy(fieldCount));
        return CreateCall(permuteFunc, {fwCast(fw, shuffle_table), fwCast(fw, a), fwCast(fw, b), mask});
    }
    if (mBitBlockWidth == 512 && fw == 64) {
        Value * permuteFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_vpermt2var_q_512);
        Constant * mask = ConstantInt::getAllOnesValue(getIntNTy(fieldCount));
        return CreateCall(permuteFunc, {fwCast(fw, shuffle_table), fwCast(fw, a), fwCast(fw, b), mask});
    }
    if (mBitBlockWidth == 512 && fw == 16 && hostCPUFeatures.hasAVX512BW) {
        Value * permuteFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_maskz_vpermt2var_hi_512);
        Constant * mask = ConstantInt::getAllOnesValue(getIntNTy(fieldCount));
        return CreateCall(permuteFunc, {fwCast(fw, shuffle_table), fwCast(fw, a), fwCast(fw, b), mask});
    }
    return IDISA_Builder::mvmd_shuffle2(fw, a, b, shuffle_table);
}

llvm::Value * IDISA_AVX512F_Builder::mvmd_compress(unsigned fw, llvm::Value * a, llvm::Value * select_mask) {
    if (mBitBlockWidth == 512 && fw == 32) {
        Value * compressFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_compress_d_512);
        return CreateCall(compressFunc, {fwCast(32, a), fwCast(32, allZeroes()), CreateZExtOrTrunc(select_mask, getInt16Ty())});
    }
    if (mBitBlockWidth == 512 && fw == 64) {
        Value * compressFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_mask_compress_q_512);
        return CreateCall(compressFunc, {fwCast(64, a), fwCast(64, allZeroes()), CreateZExtOrTrunc(select_mask, getInt8Ty())});
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
    const unsigned fieldCount = mBitBlockWidth/fw;
    if ((fw == 32) || (hostCPUFeatures.hasAVX512BW && (fw == 16)))   {
        // Mask with 1 bit per field indicating which fields are not zeroed out.
        Type * fwTy = getIntNTy(fw);
        Constant * fieldMask = ConstantInt::get(getIntNTy(fieldCount), (1 << fieldCount) - (1 << shift));
        Value * permute_func = nullptr;
        if (fw == 32) permute_func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_maskz_vpermt2var_d_512);
        else permute_func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_maskz_vpermt2var_hi_512);
        Constant * indices[fieldCount];
        for (unsigned i = 0; i < fieldCount; i++) {
            indices[i] = i < shift ? UndefValue::get(fwTy) : ConstantInt::get(fwTy, i - shift);
        }
        Value * args[4] = {ConstantVector::get({indices, fieldCount}), fwCast(fw, a), UndefValue::get(fwVectorType(fw)), fieldMask};
        return bitCast(CreateCall(permute_func, args));
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
        Value * permute_func = nullptr;
        if (fw == 32) permute_func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_maskz_vpermt2var_d_512);
        else permute_func = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_avx512_maskz_vpermt2var_hi_512);
        Constant * indices[fieldCount];
        for (unsigned i = 0; i < fieldCount; i++) {
            indices[i] = ConstantInt::get(fwTy, i + fieldCount - shift);
        }
        Constant * mask = ConstantInt::getAllOnesValue(getIntNTy(fieldCount));
        Value * args[4] = {ConstantVector::get({indices, fieldCount}), fwCast(fw, b), fwCast(fw, a), mask};
        return bitCast(CreateCall(permute_func, args));
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
