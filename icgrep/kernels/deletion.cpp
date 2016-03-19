/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


std::vector<Value *> parallel_prefix_deletion_masks(IDISA::IDISA_Builder * iBuilder, unsigned fw, Value * del_mask) {
    Value * m = iBuilder->simd_not(del_mask);
    Value * mk = iBuilder->simd_slli(fw, del_mask, 1);
    std::vector<Value *> move_masks;
    for (unsigned shift = 1; shift < fw; shift *= 2) {
        Value * mp = mk;
        for (unsigned lookright = 1; lookright < fw; lookright *= 2) {
            mp = iBuilder->simd_xor(mp, iBuilder->simd_slli(fw, mp, lookright));
        }
        Value * mv = iBuilder->simd_and(mp, m);
        m = iBuilder->simd_or(iBuilder->simd_xor(m, mv), iBuilder->simd_srli(fw, mv, shift));
        mk = iBuilder->simd_and(mk, iBuilder->simd_not(mp));
        move_masks.push_back(mv);
    }
    return move_masks;
}

Value * apply_parallel_prefix_deletion(IDISA::IDISA_Builder * iBuilder, unsigned fw, Value * del_mask, std::vector<Value *> mv, Value * strm) {
    Value * s = iBuilder->simd_and(strm, iBuilder->simd_not(del_mask));
    for (unsigned i = 0; i < mv.size(); i++) {
        unsigned shift = 1 << i;
        Value * t = iBuilder->simd_and(s, mv[i]);
        s = iBuilder->simd_or(iBuilder->simd_xor(s, t), iBuilder->simd_srli(fw, t, shift));
    }
    return s;
}


