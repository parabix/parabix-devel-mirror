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
        return CreateCall(packuswb_func, std::vector<Value *>({simd_srli(16, a, 8), simd_srli(16, b, 8)}));
    }
    // Otherwise use default logic.
    return IDISA_Builder::hsimd_packh(fw, a, b);
}

Value * IDISA_SSE2_Builder::hsimd_packl(unsigned fw, Value * a, Value * b) {
    if ((fw == 16) && (mBitBlockWidth == 128)) {
        Value * packuswb_func = Intrinsic::getDeclaration(mMod, Intrinsic::x86_sse2_packuswb_128);
        Value * mask = simd_lomask(16);
        return CreateCall(packuswb_func, std::vector<Value *>({fwCast(16, simd_and(a, mask)), fwCast(16, simd_and(b, mask))}));
    }
    // Otherwise use default logic.
    return IDISA_Builder::hsimd_packl(fw, a, b);
}

}
