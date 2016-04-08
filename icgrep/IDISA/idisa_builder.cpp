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
#include <llvm/IR/TypeBuilder.h>
#include <llvm/Support/raw_ostream.h>

namespace IDISA {

VectorType * IDISA_Builder::fwVectorType(unsigned fw) {
    int fieldCount = mBitBlockWidth/fw;
    return VectorType::get(getIntNTy(fw), fieldCount);
}

Value * IDISA_Builder::fwCast(unsigned fw, Value * a) {
    return a->getType() == fwVectorType(fw) ? a : CreateBitCast(a, fwVectorType(fw));
}


Constant* geti8StrVal(Module& M, char const* str, Twine const& name) {
    LLVMContext& ctx = getGlobalContext();
    Constant* strConstant = ConstantDataArray::getString(ctx, str);
    GlobalVariable* GVStr = new GlobalVariable(M, strConstant->getType(), true, GlobalValue::InternalLinkage, strConstant, name);
    Constant* zero = Constant::getNullValue(IntegerType::getInt32Ty(ctx));
    Constant* indices[] = {zero, zero};
    Constant* strVal = ConstantExpr::getGetElementPtr(GVStr, indices, true);
    return strVal;
}

static Function * create_printf(Module * const mod) {
    Function * printf = mod->getFunction("printf");
    if (printf == nullptr) {
        FunctionType *printf_type =
            TypeBuilder<int(char *, ...), false>::get(mod->getContext());
        printf = cast<Function>(mod->getOrInsertFunction("printf", printf_type,
            AttributeSet().addAttribute(mod->getContext(), 1U, Attribute::NoAlias)));
    }
    return printf;
}

void IDISA_Builder::CallPrintRegister(const std::string & name, Value * const value) {
    Constant * printRegister = mMod->getFunction("PrintRegister");
    if (LLVM_UNLIKELY(printRegister == nullptr)) {
        FunctionType *FT = FunctionType::get(getVoidTy(), { PointerType::get(getInt8Ty(), 0), mBitBlockType }, false);
        Function * function = Function::Create(FT, Function::InternalLinkage, "PrintRegister", mMod);
        auto arg = function->arg_begin();
        std::string tmp;
        raw_string_ostream out(tmp);
        out << "%-40s =";
        for(unsigned i = 0; i < (mBitBlockWidth / 8); ++i) {
            out << " %02x";
        }
        out << '\n';
        BasicBlock * entry = BasicBlock::Create(mMod->getContext(), "entry", function);
        IRBuilder<> builder(entry);
        std::vector<Value *> args;
        args.push_back(geti8StrVal(*mMod, out.str().c_str(), ""));
        Value * const name = arg++;
        name->setName("name");
        args.push_back(name);
        Value * value = arg;
        value->setName("value");
        Type * const byteVectorType = VectorType::get(getInt8Ty(), (mBitBlockWidth / 8));
        value = builder.CreateBitCast(value, byteVectorType);
        for(unsigned i = (mBitBlockWidth / 8); i != 0; --i) {
            args.push_back(builder.CreateExtractElement(value, builder.getInt32(i - 1)));
        }
        builder.CreateCall(create_printf(mMod), args);
        builder.CreateRetVoid();

        printRegister = function;
    }
    assert (value->getType()->canLosslesslyBitCastTo(mBitBlockType));
    CreateCall2(printRegister, geti8StrVal(*mMod, name.c_str(), name), CreateBitCast(value, mBitBlockType));
}

void IDISA_Builder::CallPrintInt(const std::string & name, Value * const value) {
    Constant * printRegister = mMod->getFunction("PrintInt");
    if (LLVM_UNLIKELY(printRegister == nullptr)) {
        FunctionType *FT = FunctionType::get(getVoidTy(), { PointerType::get(getInt8Ty(), 0), getInt64Ty() }, false);
        Function * function = Function::Create(FT, Function::InternalLinkage, "PrintInt", mMod);
        auto arg = function->arg_begin();
        std::string out = "%-40s = %" PRIi64 "\n";
        BasicBlock * entry = BasicBlock::Create(mMod->getContext(), "entry", function);
        IRBuilder<> builder(entry);
        std::vector<Value *> args;
        args.push_back(geti8StrVal(*mMod, out.c_str(), ""));
        Value * const name = arg++;
        name->setName("name");
        args.push_back(name);
        Value * value = arg;
        value->setName("value");
        args.push_back(value);
        builder.CreateCall(create_printf(mMod), args);
        builder.CreateRetVoid();

        printRegister = function;
    }
    Value * num = nullptr;
    if (value->getType()->isPointerTy()) {
        num = CreatePtrToInt(value, getInt64Ty());
    } else {
        num = CreateZExtOrBitCast(value, getInt64Ty());
    }
    assert (num->getType()->isIntegerTy());
    CreateCall2(printRegister, geti8StrVal(*mMod, name.c_str(), name), num);
}

Constant * IDISA_Builder::simd_himask(unsigned fw) {
    return Constant::getIntegerValue(getIntNTy(mBitBlockWidth), APInt::getSplat(mBitBlockWidth, APInt::getHighBitsSet(fw, fw/2)));
}

Constant * IDISA_Builder::simd_lomask(unsigned fw) {
    return Constant::getIntegerValue(getIntNTy(mBitBlockWidth), APInt::getSplat(mBitBlockWidth, APInt::getLowBitsSet(fw, fw/2)));
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
    Value * cttzFunc = Intrinsic::getDeclaration(mMod, Intrinsic::cttz, fwVectorType(fw));
    Value * rslt = CreateCall(cttzFunc, std::vector<Value *>({fwCast(fw, a), ConstantInt::get(getInt1Ty(), 0)}));
    return rslt;
}

Value * IDISA_Builder::simd_popcount(unsigned fw, Value * a) {
    Value * ctpopFunc = Intrinsic::getDeclaration(mMod, Intrinsic::ctpop, fwVectorType(fw));
    Value * rslt = CreateCall(ctpopFunc, std::vector<Value *>({fwCast(fw, a)}));
    return rslt;
}

Value * IDISA_Builder::simd_if(unsigned fw, Value * cond, Value * a, Value * b) {
    if (fw == 1) {
        Value * a1 = bitCast(a);
        Value * b1 = bitCast(b);
        Value * c = bitCast(cond);
        return CreateOr(CreateAnd(a1, c), CreateAnd(CreateXor(c, b1), b1));
    }
    else {
        Value * aVec = fwCast(fw, a);
        Value * bVec = fwCast(fw, b);
        return CreateSelect(CreateICmpSLT(cond, mZeroInitializer), aVec, bVec);
    }
}

    
Value * IDISA_Builder::esimd_mergeh(unsigned fw, Value * a, Value * b) {
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

Value * IDISA_Builder::esimd_mergel(unsigned fw, Value * a, Value * b) {
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

Value * IDISA_Builder::esimd_bitspread(unsigned fw, Value * bitmask) {
    unsigned field_count = mBitBlockWidth/fw;
    Type * field_type = getIntNTy(fw);
    if (bitmask->getType()->getIntegerBitWidth() < fw) {
        bitmask = CreateZExt(bitmask, field_type);
    }
    else if (bitmask->getType()->getIntegerBitWidth() > fw) {
        bitmask = CreateTrunc(bitmask, field_type);
    }
    Value * spread_field = CreateBitCast(bitmask, VectorType::get(getIntNTy(fw), 1));
    Value * undefVec = UndefValue::get(VectorType::get(getIntNTy(fw), 1));
    Value * broadcast = CreateShuffleVector(spread_field, undefVec, Constant::getNullValue(VectorType::get(getInt32Ty(), field_count)));
    std::vector<Constant*> bitSel;
    std::vector<Constant*> bitShift;
    for (unsigned i = 0; i < field_count; i++) {
        bitSel.push_back(ConstantInt::get(field_type, 1 << i));
        bitShift.push_back(ConstantInt::get(field_type, i));
    }
    Value * bitSelVec = ConstantVector::get(bitSel);
    Value * bitShiftVec = ConstantVector::get(bitShift);
    return CreateLShr(CreateAnd(bitSelVec, broadcast), bitShiftVec);
}

Value * IDISA_Builder::hsimd_packh(unsigned fw, Value * a, Value * b) {
    unsigned field_count = 2 * mBitBlockWidth/fw;
    Value * aVec = fwCast(fw/2, a);
    Value * bVec = fwCast(fw/2, b);
    std::vector<Constant*> Idxs;
    for (unsigned i = 0; i < field_count; i++) {
        Idxs.push_back(getInt32(2*i+1));
    }
    return CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs));
}

Value * IDISA_Builder::hsimd_packl(unsigned fw, Value * a, Value * b) {
    unsigned field_count = 2 * mBitBlockWidth/fw;
    Value * aVec = fwCast(fw/2, a);
    Value * bVec = fwCast(fw/2, b);
    std::vector<Constant*> Idxs;
    for (unsigned i = 0; i < field_count; i++) {
        Idxs.push_back(getInt32(2*i));
    }
    return CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs));
}

    
Value * IDISA_Builder::hsimd_packh_in_lanes(unsigned lanes, unsigned fw, Value * a, Value * b) {
    unsigned fw_out = fw/2;
    unsigned fields_per_lane = mBitBlockWidth/(fw_out * lanes);
    unsigned field_offset_for_b = mBitBlockWidth/fw_out;
    Value * aVec = fwCast(fw_out, a);
    Value * bVec = fwCast(fw_out, b);
    std::vector<Constant*> Idxs;
    for (unsigned lane = 0; lane < lanes; lane++) {
        unsigned first_field_in_lane = lane * fields_per_lane; // every second field
        for (unsigned i = 0; i < fields_per_lane/2; i++) {
            Idxs.push_back(getInt32(first_field_in_lane + 2*i + 1));
        }
        for (unsigned i = 0; i < fields_per_lane/2; i++) {
            Idxs.push_back(getInt32(field_offset_for_b + first_field_in_lane + 2*i + 1));
        }
    }
    Value * pack = CreateShuffleVector(aVec, bVec, ConstantVector::get(Idxs));
    return pack;
}

Value * IDISA_Builder::hsimd_packl_in_lanes(unsigned lanes, unsigned fw, Value * a, Value * b) {
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

    
Value * IDISA_Builder::hsimd_signmask(unsigned fw, Value * a) {
    Value * mask = CreateICmpSLT(fwCast(fw, a), ConstantAggregateZero::get(fwVectorType(fw)));
    return CreateZExt(CreateBitCast(mask, getIntNTy(mBitBlockWidth/fw)), getInt32Ty());
}

Value * IDISA_Builder::mvmd_extract(unsigned fw, Value * a, unsigned fieldIndex) {
    Value * aVec = fwCast(fw, a);
    return CreateExtractElement(aVec, getInt32(fieldIndex));
}

Value * IDISA_Builder::mvmd_insert(unsigned fw, Value * blk, Value * elt, unsigned fieldIndex) {
    Value * vec = fwCast(fw, blk);
    return CreateInsertElement(vec, elt, getInt32(fieldIndex));
}

Value * IDISA_Builder::mvmd_slli(unsigned fw, Value * a, unsigned shift) {
    unsigned field_count = mBitBlockWidth/fw;
    return mvmd_dslli(fw, a, Constant::getNullValue(fwVectorType(fw)), field_count - shift);
}

Value * IDISA_Builder::mvmd_srli(unsigned fw, Value * a, unsigned shift) {
    unsigned field_count = mBitBlockWidth/fw;
    return mvmd_dslli(fw, Constant::getNullValue(fwVectorType(fw)), a, shift);
}

Value * IDISA_Builder::mvmd_dslli(unsigned fw, Value * a, Value * b, unsigned shift) {
    unsigned field_count = mBitBlockWidth/fw;
    Value * aVec = fwCast(fw, a);
    Value * bVec = fwCast(fw, b);
    std::vector<Constant*> Idxs;
    for (unsigned i = 0; i < field_count; i++) {
        Idxs.push_back(getInt32(i + shift));
    }
    return CreateShuffleVector(bVec, aVec, ConstantVector::get(Idxs));
}

Value * IDISA_Builder::bitblock_any(Value * a) {
    Type * iBitBlock = getIntNTy(mBitBlockWidth);
    return CreateICmpNE(CreateBitCast(a, iBitBlock),  ConstantInt::get(iBitBlock, 0));
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

}
