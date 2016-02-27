/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "idisa_i64_builder.h"
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Function.h>

namespace IDISA {


Value * IDISA_I64_Builder::hsimd_packh(unsigned fw, Value * a, Value * b) {
    Value * mask0 = getInt64(0xFF00000000000000);
    Value * mask1 = getInt64(0x0000FF0000000000);
    Value * mask2 = getInt64(0x00000000FF000000);
    Value * mask3 = getInt64(0x000000000000FF00);

    return simd_or(simd_or(simd_or(simd_and(b, mask0), simd_slli(64, simd_and(b, mask1), 8)) 
    , simd_or(simd_slli(64, simd_and(b, mask2), 16),simd_slli(64, simd_and(b, mask3), 24))), 
    simd_or(simd_or(simd_srli(64, simd_and(a, mask0), 32), simd_srli(64, simd_and(a, mask1), 24))
    ,simd_or(simd_srli(64, simd_and(a, mask2), 16), simd_srli(64, simd_and(a, mask3), 8))));
}

Value * IDISA_I64_Builder::hsimd_packl(unsigned fw, Value * a, Value * b) {  
    Value * mask0 = getInt64(0x00FF000000000000);
    Value * mask1 = getInt64(0x000000FF00000000);
    Value * mask2 = getInt64(0x0000000000FF0000);
    Value * mask3 = getInt64(0x00000000000000FF);

    return simd_or(simd_or(simd_or(simd_slli(64, simd_and(b, mask0), 8), simd_slli(64, simd_and(b, mask1), 16)) 
    , simd_or(simd_slli(64, simd_and(b, mask2), 24),simd_slli(64, simd_and(b, mask3), 32))), 
    simd_or(simd_or(simd_srli(64, simd_and(a, mask0), 24), simd_srli(64, simd_and(a, mask1), 16))
    ,simd_or(simd_srli(64, simd_and(a, mask2), 8), simd_and(a, mask3))));

}

}
