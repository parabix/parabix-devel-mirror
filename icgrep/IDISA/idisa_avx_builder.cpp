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

Value * IDISA_AVX2_Builder::hsimd_signmask(unsigned fw, Value * a) {
    if (fw == 64) {
        Value * signmask_f64func = Intrinsic::getDeclaration(mMod, Intrinsic::x86_avx_movmsk_pd_256);
        Type * bitBlock_f64type = VectorType::get(mLLVMBuilder->getDoubleTy(), mBitBlockWidth/64);
        Value * a_as_pd = mLLVMBuilder->CreateBitCast(a, bitBlock_f64type);
        Value * mask = mLLVMBuilder->CreateCall(signmask_f64func, std::vector<Value *>({a_as_pd}));
        return mask;
    }
    else if (fw == 32) {
        Value * signmask_f32func = Intrinsic::getDeclaration(mMod, Intrinsic::x86_avx_movmsk_ps_256);
        Type * bitBlock_f32type = VectorType::get(mLLVMBuilder->getFloatTy(), mBitBlockWidth/32);
        Value * a_as_ps = mLLVMBuilder->CreateBitCast(a, bitBlock_f32type);
        Value * mask = mLLVMBuilder->CreateCall(signmask_f32func, std::vector<Value *>({a_as_ps}));
        return mask;
    }
    Value * mask = mLLVMBuilder->CreateICmpSLT(fwCast(fw, a), ConstantAggregateZero::get(fwVectorType(fw)));
    return mLLVMBuilder->CreateBitCast(mask, mLLVMBuilder->getIntNTy(mBitBlockWidth/fw));
}
    
}
