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

/*
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
*/

/*Value * IDISA_I64_Builder::hsimd_packh(unsigned fw, Value * a, Value * b) {
    Value * mask02 = getInt64(0xFF000000FF000000);
    Value * mask13 = getInt64(0x0000FF000000FF00);

    Value * b1 = simd_or(simd_and(b, mask02), simd_slli(64, simd_and(b, mask13), 8));
    Value * a1 = simd_or(simd_and(a, mask02), simd_slli(64, simd_and(a, mask13), 8));

    Value * mask01 = getInt64(0xFFFF000000000000);
    Value * mask23 = getInt64(0x00000000FFFF0000);

    Value * b2 = simd_or(simd_and(b1, mask01), simd_slli(64, simd_and(b1, mask23), 16));
    Value * a2 = simd_or(simd_srli(64, simd_and(a1, mask01), 32), simd_srli(64, simd_and(a1, mask23), 16));

    return simd_or(b2, a2);
}
*/
    Value * IDISA_I64_Builder::hsimd_packh(unsigned fw, Value * a, Value * b) {
        Value * a_ = a;
        Value * b_ = b;
        for (unsigned w = fw; w < mBitBlockWidth; w *= 2) {
            Value * himask_odd = simd_and(simd_himask(w), simd_himask(2*w));  // high half of odd fields
            Value * himask_even = simd_and(simd_himask(w), simd_lomask(2*w));  // high half of even fields
            b_ = simd_or(simd_and(b_, himask_odd), simd_slli(mBitBlockWidth, simd_and(b_, himask_even), w/2));
            a_ = simd_or(simd_and(a_, himask_odd), simd_slli(mBitBlockWidth, simd_and(a_, himask_even), w/2));
        }
        Value * pk = simd_or(b_, simd_srli(mBitBlockWidth, a_, mBitBlockWidth/2));
        return pk;
    }
    
    
    Value * IDISA_I64_Builder::hsimd_packl(unsigned fw, Value * a, Value * b) {
        Value * a_ = a;
        Value * b_ = b;
        for (unsigned w = fw; w < mBitBlockWidth; w *= 2) {
            Value * lomask_odd = simd_and(simd_lomask(w), simd_himask(2*w));  // high half of odd fields
            Value * lomask_even = simd_and(simd_lomask(w), simd_lomask(2*w));  // high half of even fields
            b_ = simd_or(simd_and(b_, lomask_even), simd_srli(mBitBlockWidth, simd_and(b_, lomask_odd), w/2));
            a_ = simd_or(simd_and(a_, lomask_even), simd_srli(mBitBlockWidth, simd_and(a_, lomask_odd), w/2));
        }
        Value * pk = simd_or(simd_slli(mBitBlockWidth, b_, mBitBlockWidth/2), a_);
        return pk;
    }

}
