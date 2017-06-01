/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "idisa_builder.h"
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/TypeBuilder.h>
#include <toolchain/toolchain.h>

using namespace llvm;

namespace IDISA {

VectorType * IDISA_Builder::fwVectorType(const unsigned fw) {
    return VectorType::get(getIntNTy(fw), mBitBlockWidth / fw);
}

Value * IDISA_Builder::fwCast(const unsigned fw, Value * const a) {
    VectorType * const ty = fwVectorType(fw);
    assert (a->getType()->getPrimitiveSizeInBits() == ty->getPrimitiveSizeInBits());
    return CreateBitCast(a, ty);
}

void IDISA_Builder::CallPrintRegister(const std::string & name, Value * const value) {
    Module * const m = getModule();
    Constant * printRegister = m->getFunction("PrintRegister");
    if (LLVM_UNLIKELY(printRegister == nullptr)) {
        FunctionType *FT = FunctionType::get(getVoidTy(), { PointerType::get(getInt8Ty(), 0), getBitBlockType() }, false);
        Function * function = Function::Create(FT, Function::InternalLinkage, "PrintRegister", m);
        auto arg = function->arg_begin();
        std::string tmp;
        raw_string_ostream out(tmp);
        out << "%-40s =";
        for(unsigned i = 0; i < (mBitBlockWidth / 8); ++i) {
            out << " %02x";
        }
        out << '\n';
        BasicBlock * entry = BasicBlock::Create(m->getContext(), "entry", function);
        IRBuilder<> builder(entry);
        std::vector<Value *> args;
        args.push_back(GetString(out.str().c_str()));
        Value * const name = &*(arg++);
        name->setName("name");
        args.push_back(name);
        Value * value = &*arg;
        value->setName("value");
        Type * const byteVectorType = VectorType::get(getInt8Ty(), (mBitBlockWidth / 8));
        value = builder.CreateBitCast(value, byteVectorType);
        for(unsigned i = (mBitBlockWidth / 8); i != 0; --i) {
            args.push_back(builder.CreateExtractElement(value, builder.getInt32(i - 1)));
        }
        builder.CreateCall(GetPrintf(), args);
        builder.CreateRetVoid();

        printRegister = function;
    }
    CreateCall(printRegister, {GetString(name.c_str()), CreateBitCast(value, mBitBlockType)});
}

Constant * IDISA_Builder::simd_himask(unsigned fw) {
    return Constant::getIntegerValue(getIntNTy(mBitBlockWidth), APInt::getSplat(mBitBlockWidth, APInt::getHighBitsSet(fw, fw/2)));
}

Constant * IDISA_Builder::simd_lomask(unsigned fw) {
    return Constant::getIntegerValue(getIntNTy(mBitBlockWidth), APInt::getSplat(mBitBlockWidth, APInt::getLowBitsSet(fw, fw/2)));
}

Value * IDISA_Builder::simd_fill(unsigned fw, Value * a) {
    unsigned field_count = mBitBlockWidth/fw;
    Type * singleFieldVecTy = VectorType::get(getIntNTy(fw), 1);
    Value * aVec = CreateBitCast(a, singleFieldVecTy);
    return CreateShuffleVector(aVec, UndefValue::get(singleFieldVecTy), Constant::getNullValue(VectorType::get(getInt32Ty(), field_count)));
}

Value * IDISA_Builder::simd_add(unsigned fw, Value * a, Value * b) {
    return CreateAdd(fwCast(fw, a), fwCast(fw, b));
}

Value * IDISA_Builder::simd_sub(unsigned fw, Value * a, Value * b) {
    return CreateSub(fwCast(fw, a), fwCast(fw, b));
}

Value * IDISA_Builder::simd_mult(unsigned fw, Value * a, Value * b) {
    return CreateMul(fwCast(fw, a), fwCast(fw, b));
}

Value * IDISA_Builder::simd_eq(unsigned fw, Value * a, Value * b) {
    return CreateSExt(CreateICmpEQ(fwCast(fw, a), fwCast(fw, b)), fwVectorType(fw));
}

Value * IDISA_Builder::simd_gt(unsigned fw, Value * a, Value * b) {
    return CreateSExt(CreateICmpSGT(fwCast(fw, a), fwCast(fw, b)), fwVectorType(fw));
}

Value * IDISA_Builder::simd_ugt(unsigned fw, Value * a, Value * b) {
    return CreateSExt(CreateICmpUGT(fwCast(fw, a), fwCast(fw, b)), fwVectorType(fw));
}

Value * IDISA_Builder::simd_lt(unsigned fw, Value * a, Value * b) {
    return CreateSExt(CreateICmpSLT(fwCast(fw, a), fwCast(fw, b)), fwVectorType(fw));
}

Value * IDISA_Builder::simd_ult(unsigned fw, Value * a, Value * b) {
    return CreateSExt(CreateICmpULT(fwCast(fw, a), fwCast(fw, b)), fwVectorType(fw));
}

Value * IDISA_Builder::simd_max(unsigned fw, Value * a, Value * b) {
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    return CreateSelect(CreateICmpSGT(aVec, bVec), aVec, bVec);
}

Value * IDISA_Builder::simd_umax(unsigned fw, Value * a, Value * b) {
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    return CreateSelect(CreateICmpUGT(aVec, bVec), aVec, bVec);
}

Value * IDISA_Builder::simd_min(unsigned fw, Value * a, Value * b) {
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    return CreateSelect(CreateICmpSLT(aVec, bVec), aVec, bVec);
}

Value * IDISA_Builder::simd_umin(unsigned fw, Value * a, Value * b) {
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    return CreateSelect(CreateICmpULT(aVec, bVec), aVec, bVec);
}

Value * IDISA_Builder::simd_slli(unsigned fw, Value * a, unsigned shift) {
    return CreateShl(fwCast(fw, a), shift);
}

Value * IDISA_Builder::simd_srli(unsigned fw, Value * a, unsigned shift) {
    return CreateLShr(fwCast(fw, a), shift);
}

Value * IDISA_Builder::simd_srai(unsigned fw, Value * a, unsigned shift) {
    return CreateAShr(fwCast(fw, a), shift);
}

Value * IDISA_Builder::simd_cttz(unsigned fw, Value * a) {
    Value * cttzFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::cttz, fwVectorType(fw));
    return CreateCall(cttzFunc, {fwCast(fw, a), ConstantInt::get(getInt1Ty(), 0)});
}

Value * IDISA_Builder::simd_popcount(unsigned fw, Value * a) {
    Value * ctpopFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::ctpop, fwVectorType(fw));
    return CreateCall(ctpopFunc, fwCast(fw, a));
}

Value * IDISA_Builder::simd_bitreverse(unsigned fw, Value * a) {
    /*  Pure sequential solution too slow!
     Value * func = Intrinsic::getDeclaration(getModule(), Intrinsic::bitreverse, fwVectorType(fw));
     return CreateCall(func, fwCast(fw, a));
     */
    if (fw > 8) {
        // Reverse the bits of each byte and then use a byte shuffle to complete the job.
        Value * bitrev8 = fwCast(8, simd_bitreverse(8, a));
        const auto bytes_per_field = fw/8;
        const auto byte_count = mBitBlockWidth / 8;
        Constant * Idxs[byte_count];
        for (unsigned i = 0; i < byte_count; i += bytes_per_field) {
            for (unsigned j = 0; j < bytes_per_field; j++) {
                Idxs[i + j] = getInt32(i + bytes_per_field - j - 1);
            }
        }
        return CreateShuffleVector(bitrev8, UndefValue::get(fwVectorType(8)), ConstantVector::get({Idxs, byte_count}));
    }
    else {
        if (fw > 2) {
            a = simd_bitreverse(fw/2, a);
        }
        return simd_or(simd_srli(16, simd_and(a, simd_himask(fw)), fw/2), simd_slli(16, simd_and(a, simd_lomask(fw)), fw/2));
    }
}

Value * IDISA_Builder::simd_if(unsigned fw, Value * cond, Value * a, Value * b) {
    if (fw == 1) {
        Value * a1 = bitCast(a);
        Value * b1 = bitCast(b);
        Value * c = bitCast(cond);
        return CreateOr(CreateAnd(a1, c), CreateAnd(CreateXor(c, b1), b1));
    } else {
        Value * aVec = fwCast(fw, a);
        Value * bVec = fwCast(fw, b);
        return CreateSelect(CreateICmpSLT(cond, mZeroInitializer), aVec, bVec);
    }
}
    
Value * IDISA_Builder::esimd_mergeh(unsigned fw, Value * a, Value * b) {    
    const auto field_count = mBitBlockWidth / fw;
    Constant * Idxs[field_count];
    for (unsigned i = 0; i < field_count / 2; i++) {
        Idxs[2 * i] = getInt32(i + field_count / 2); // selects elements from first reg.
        Idxs[2 * i + 1] = getInt32(i + field_count / 2 + field_count); // selects elements from second reg.
    }
    return CreateShuffleVector(fwCast(fw, a), fwCast(fw, b), ConstantVector::get({Idxs, field_count}));
}

Value * IDISA_Builder::esimd_mergel(unsigned fw, Value * a, Value * b) {    
    const auto field_count = mBitBlockWidth / fw;
    Constant * Idxs[field_count];
    for (unsigned i = 0; i < field_count / 2; i++) {
        Idxs[2 * i] = getInt32(i); // selects elements from first reg.
        Idxs[2 * i + 1] = getInt32(i + field_count); // selects elements from second reg.
    }
    return CreateShuffleVector(fwCast(fw, a), fwCast(fw, b), ConstantVector::get({Idxs, field_count}));
}

Value * IDISA_Builder::esimd_bitspread(unsigned fw, Value * bitmask) {
    const auto field_count = mBitBlockWidth / fw;
    Type * field_type = getIntNTy(fw);
    Value * spread_field = CreateBitCast(CreateZExtOrTrunc(bitmask, field_type), VectorType::get(getIntNTy(fw), 1));
    Value * undefVec = UndefValue::get(VectorType::get(getIntNTy(fw), 1));
    Value * broadcast = CreateShuffleVector(spread_field, undefVec, Constant::getNullValue(VectorType::get(getInt32Ty(), field_count)));
    Constant * bitSel[field_count];
    Constant * bitShift[field_count];
    for (unsigned i = 0; i < field_count; i++) {
        bitSel[i] = ConstantInt::get(field_type, 1 << i);
        bitShift[i] = ConstantInt::get(field_type, i);
    }
    Value * bitSelVec = ConstantVector::get({bitSel, field_count});
    Value * bitShiftVec = ConstantVector::get({bitShift, field_count});
    return CreateLShr(CreateAnd(bitSelVec, broadcast), bitShiftVec);
}

Value * IDISA_Builder::hsimd_packh(unsigned fw, Value * a, Value * b) {
    Value * aVec = fwCast(fw/2, a);
    Value * bVec = fwCast(fw/2, b);
    const auto field_count = 2 * mBitBlockWidth / fw;
    Constant * Idxs[field_count];
    for (unsigned i = 0; i < field_count; i++) {
        Idxs[i] = getInt32(2 * i + 1);
    }
    return CreateShuffleVector(aVec, bVec, ConstantVector::get({Idxs, field_count}));
}

Value * IDISA_Builder::hsimd_packl(unsigned fw, Value * a, Value * b) {
    Value * aVec = fwCast(fw/2, a);
    Value * bVec = fwCast(fw/2, b);
    const auto field_count = 2 * mBitBlockWidth / fw;
    Constant * Idxs[field_count];
    for (unsigned i = 0; i < field_count; i++) {
        Idxs[i] = getInt32(2 * i);
    }
    return CreateShuffleVector(aVec, bVec, ConstantVector::get({Idxs, field_count}));
}

Value * IDISA_Builder::hsimd_packh_in_lanes(unsigned lanes, unsigned fw, Value * a, Value * b) {
    const unsigned fw_out = fw / 2;
    const unsigned fields_per_lane = mBitBlockWidth / (fw_out * lanes);
    const unsigned field_offset_for_b = mBitBlockWidth / fw_out;
    const unsigned field_count = mBitBlockWidth / fw_out;
    Constant * Idxs[field_count];
    for (unsigned lane = 0, j = 0; lane < lanes; lane++) {
        const unsigned first_field_in_lane = lane * fields_per_lane; // every second field
        for (unsigned i = 0; i < fields_per_lane / 2; i++) {
            Idxs[j++] = getInt32(first_field_in_lane + (2 * i) + 1);
        }
        for (unsigned i = 0; i < fields_per_lane / 2; i++) {
            Idxs[j++] = getInt32(field_offset_for_b + first_field_in_lane + (2 * i) + 1);
        }
    }
    return CreateShuffleVector(fwCast(fw_out, a), fwCast(fw_out, b), ConstantVector::get({Idxs, field_count}));
}

Value * IDISA_Builder::hsimd_packl_in_lanes(unsigned lanes, unsigned fw, Value * a, Value * b) {
    const unsigned fw_out = fw / 2;
    const unsigned fields_per_lane = mBitBlockWidth / (fw_out * lanes);
    const unsigned field_offset_for_b = mBitBlockWidth / fw_out;
    const unsigned field_count = mBitBlockWidth / fw_out;
    Constant * Idxs[field_count];
    for (unsigned lane = 0, j = 0; lane < lanes; lane++) {
        const unsigned first_field_in_lane = lane * fields_per_lane; // every second field
        for (unsigned i = 0; i < fields_per_lane / 2; i++) {
            Idxs[j++] = getInt32(first_field_in_lane + (2 * i));
        }
        for (unsigned i = 0; i < fields_per_lane / 2; i++) {
            Idxs[j++] = getInt32(field_offset_for_b + first_field_in_lane + (2 * i));
        }
    }
    return CreateShuffleVector(fwCast(fw_out, a), fwCast(fw_out, b), ConstantVector::get({Idxs, field_count}));
}

Value * IDISA_Builder::hsimd_signmask(unsigned fw, Value * a) {
    Value * mask = CreateICmpSLT(fwCast(fw, a), ConstantAggregateZero::get(fwVectorType(fw)));
    return CreateZExt(CreateBitCast(mask, getIntNTy(mBitBlockWidth/fw)), getInt32Ty());
}

Value * IDISA_Builder::mvmd_extract(unsigned fw, Value * a, unsigned fieldIndex) {
    return CreateExtractElement(fwCast(fw, a), getInt32(fieldIndex));
}

Value * IDISA_Builder::mvmd_insert(unsigned fw, Value * blk, Value * elt, unsigned fieldIndex) {
    return CreateInsertElement(fwCast(fw, blk), elt, getInt32(fieldIndex));
}

Value * IDISA_Builder::mvmd_slli(unsigned fw, Value * a, unsigned shift) {
    const auto field_count = mBitBlockWidth / fw;
    return mvmd_dslli(fw, a, Constant::getNullValue(fwVectorType(fw)), field_count - shift);
}

Value * IDISA_Builder::mvmd_srli(unsigned fw, Value * a, unsigned shift) {
    return mvmd_dslli(fw, Constant::getNullValue(fwVectorType(fw)), a, shift);
}

Value * IDISA_Builder::mvmd_dslli(unsigned fw, Value * a, Value * b, unsigned shift) {
    const auto field_count = mBitBlockWidth/fw;
    Constant * Idxs[field_count];
    for (unsigned i = 0; i < field_count; i++) {
        Idxs[i] = getInt32(i + shift);
    }
    return CreateShuffleVector(fwCast(fw, b), fwCast(fw, a), ConstantVector::get({Idxs, field_count}));
}

Value * IDISA_Builder::bitblock_any(Value * a) {
    Type * iBitBlock = getIntNTy(mBitBlockWidth);
    return CreateICmpNE(CreateBitCast(a, iBitBlock),  ConstantInt::getNullValue(iBitBlock));
}

// full add producing {carryout, sum}
std::pair<Value *, Value *> IDISA_Builder::bitblock_add_with_carry(Value * a, Value * b, Value * carryin) {
    Value * carrygen = simd_and(a, b);
    Value * carryprop = simd_or(a, b);
    Value * sum = simd_add(mBitBlockWidth, simd_add(mBitBlockWidth, a, b), carryin);
    Value * carryout = CreateBitCast(simd_or(carrygen, simd_and(carryprop, CreateNot(sum))), getIntNTy(mBitBlockWidth));
    return std::pair<Value *, Value *>(bitCast(simd_srli(mBitBlockWidth, carryout, mBitBlockWidth - 1)), bitCast(sum));
}

// full shift producing {shiftout, shifted}
std::pair<Value *, Value *> IDISA_Builder::bitblock_advance(Value * a, Value * shiftin, unsigned shift) {
    Value * shiftin_bitblock = CreateBitCast(shiftin, getIntNTy(mBitBlockWidth));
    Value * a_bitblock = CreateBitCast(a, getIntNTy(mBitBlockWidth));
    Value * shifted = bitCast(CreateOr(CreateShl(a_bitblock, shift), shiftin_bitblock));
    Value * shiftout = bitCast(CreateLShr(a_bitblock, mBitBlockWidth - shift));
    return std::pair<Value *, Value *>(shiftout, shifted);
}

Value * IDISA_Builder::bitblock_mask_from(Value * pos) {
    Type * bitBlockInt = getIntNTy(getBitBlockWidth());
    return bitCast(CreateShl(ConstantInt::getAllOnesValue(bitBlockInt), CreateZExt(pos, bitBlockInt)));
    
}
Value * IDISA_Builder::bitblock_set_bit(Value * pos) {
    Type * bitBlockInt = getIntNTy(getBitBlockWidth());
    return bitCast(CreateShl(ConstantInt::get(bitBlockInt, 1), CreateZExt(pos, bitBlockInt)));
}

Value * IDISA_Builder::simd_and(Value * a, Value * b) {
    return a->getType() == b->getType() ? CreateAnd(a, b) : CreateAnd(bitCast(a), bitCast(b));
}

Value * IDISA_Builder::simd_or(Value * a, Value * b) {
    return a->getType() == b->getType() ? CreateOr(a, b) : CreateOr(bitCast(a), bitCast(b));
}
    
Value * IDISA_Builder::simd_xor(Value * a, Value * b) {
    return a->getType() == b->getType() ? CreateXor(a, b) : CreateXor(bitCast(a), bitCast(b));
}

Value * IDISA_Builder::simd_not(Value * a) {
    return simd_xor(a, Constant::getAllOnesValue(a->getType()));
}

LoadInst * IDISA_Builder::CreateBlockAlignedLoad(Value * const ptr) {
    const auto alignment = mBitBlockWidth / 8;
    if (codegen::EnableAsserts) {
        DataLayout DL(getModule());
        IntegerType * const intPtrTy = getIntPtrTy(DL);
        Value * alignmentOffset = CreateURem(CreatePtrToInt(ptr, intPtrTy), ConstantInt::get(intPtrTy, alignment));
        Value * alignmentCheck = CreateICmpEQ(alignmentOffset, ConstantInt::getNullValue(intPtrTy));
        CreateAssert(alignmentCheck, "CreateBlockAlignedLoad: pointer is unaligned");
    }
    return CreateAlignedLoad(ptr, alignment);
}

void IDISA_Builder::CreateBlockAlignedStore(Value * const value, Value * const ptr) {
    const auto alignment = mBitBlockWidth / 8;
    if (codegen::EnableAsserts) {
        DataLayout DL(getModule());
        IntegerType * const intPtrTy = getIntPtrTy(DL);
        Value * alignmentOffset = CreateURem(CreatePtrToInt(ptr, intPtrTy), ConstantInt::get(intPtrTy, alignment));
        Value * alignmentCheck = CreateICmpEQ(alignmentOffset, ConstantInt::getNullValue(intPtrTy));
        CreateAssert(alignmentCheck, "CreateBlockAlignedStore: pointer is not aligned");
    }
    CreateAlignedStore(value, ptr, alignment);
}

IDISA_Builder::IDISA_Builder(llvm::LLVMContext & C, unsigned vectorWidth, unsigned stride)
: CBuilder(C)
, mBitBlockWidth(vectorWidth)
, mStride(stride)
, mBitBlockType(VectorType::get(IntegerType::get(C, 64), vectorWidth / 64))
, mZeroInitializer(Constant::getNullValue(mBitBlockType))
, mOneInitializer(Constant::getAllOnesValue(mBitBlockType))
, mPrintRegisterFunction(nullptr) {

}

IDISA_Builder::~IDISA_Builder() {

}

}
