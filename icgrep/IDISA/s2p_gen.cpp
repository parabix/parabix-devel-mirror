/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#include "s2p_gen.h"

void * s2p_step(IDISA_Builder * iBuilder, Value * s0, Value * s1, Value * hi_mask, unsigned shift, Value * p0, Value * p1) {
    Value * t0 = iBuilder->hsimd_packh(16, s0, s1);
    Value * t1 = iBuilder->hsimd_packl(16, s0, s1);
    Value * p0 = iBuilder->simd_if(1, hi_mask, t0, iBuilder->simd_srli(16, t1, shift));
    Value * p1 = iBuilder->simd_if(1, hi_mask, iBuilder->simd_slli(16, t0, shift), t1);
}

void * s2p(IDISA_Builder * iBuilder,
           Value * s0, Value * s1, Value * s2, Value * s3, Value * s4, Value * s5, Value * s6, Value * s7,
           Value * p0, Value * p1, Value * p2, Value * p3, Value * p4, Value * p5, Value * p6, Value * p7) {
    Value * bit00224466_0,bit00224466_1,bit00224466_2,bit00224466_3;
    Value * bit11335577_0,bit11335577_1,bit11335577_2,bit11335577_3;
    Value * bit00004444_0,bit22226666_0,bit00004444_1,bit22226666_1;
    Value * bit11115555_0,bit33337777_0,bit11115555_1,bit33337777_1;
    Value * hi_mask2 = iBuilder->simd_himask(2);
    Value * hi_mask4 = iBuilder->simd_himask(4);
    Value * hi_mask8 = iBuilder->simd_himask(8);
    s2p_step(iBuilder, s0,s1,hi_mask2,1,bit00224466_0,bit11335577_0);
    s2p_step(iBuilder, s2,s3,hi_mask2,1,bit00224466_1,bit11335577_1);
    s2p_step(iBuilder, s4,s5,hi_mask2,1,bit00224466_2,bit11335577_2);
    s2p_step(iBuilder, s6,s7,hi_mask2,1,bit00224466_3,bit11335577_3);
    s2p_step(iBuilder, bit00224466_0,bit00224466_1,hi_mask4,2,bit00004444_0,bit22226666_0);
    s2p_step(iBuilder, bit00224466_2,bit00224466_3,hi_mask4,2,bit00004444_1,bit22226666_1);
    s2p_step(iBuilder, bit11335577_0,bit11335577_1,hi_mask4,2,bit11115555_0,bit33337777_0);
    s2p_step(iBuilder, bit11335577_2,bit11335577_3,hi_mask4,2,bit11115555_1,bit33337777_1);
    s2p_step(iBuilder, bit00004444_0,bit00004444_1,hi_mask8,4,p0,p4);
    s2p_step(iBuilder, bit11115555_0,bit11115555_1,hi_mask8,4,p1,p5);
    s2p_step(iBuilder, bit22226666_0,bit22226666_1,hi_mask8,4,p2,p6);
    s2p_step(iBuilder, bit33337777_0,bit33337777_1,hi_mask8,4,p3,p7);
}
