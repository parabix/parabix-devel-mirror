/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "kernel.h"
#include "s2p_kernel.h"
#include <iostream>


void s2p_step(IDISA::IDISA_Builder * iBuilder, Value * s0, Value * s1, Value * hi_mask, unsigned shift, Value * &p0, Value * &p1) {
    Value * t0 = iBuilder->hsimd_packh(16, s0, s1);
    Value * t1 = iBuilder->hsimd_packl(16, s0, s1);
    p0 = iBuilder->simd_if(1, hi_mask, t0, iBuilder->simd_srli(16, t1, shift));
    p1 = iBuilder->simd_if(1, hi_mask, iBuilder->simd_slli(16, t0, shift), t1);
}

void s2p(IDISA::IDISA_Builder * iBuilder, std::vector<Value*> s, Value* p[]) {
    Value * bit00224466[4];
    Value * bit11335577[4];
    for (unsigned i = 0; i<4; i++) {
        s2p_step(iBuilder, s[2*i], s[2*i+1], iBuilder->simd_himask(2), 1, bit00224466[i], bit11335577[i]);
    }
    Value * bit00004444[2];
    Value * bit22226666[2];
    Value * bit11115555[2];
    Value * bit33337777[2];
    for (unsigned j = 0; j<2; j++) {
        s2p_step(iBuilder, bit00224466[2*j], bit00224466[2*j+1],
                 iBuilder->simd_himask(4), 2, bit00004444[j], bit22226666[j]);
        s2p_step(iBuilder, bit11335577[2*j], bit11335577[2*j+1],
                 iBuilder->simd_himask(4), 2, bit11115555[j], bit33337777[j]);
    }
    s2p_step(iBuilder, bit00004444[0], bit00004444[1], iBuilder->simd_himask(8), 4, p[0], p[4]);
    s2p_step(iBuilder, bit11115555[0], bit11115555[1], iBuilder->simd_himask(8), 4, p[1], p[5]);
    s2p_step(iBuilder, bit22226666[0], bit22226666[1], iBuilder->simd_himask(8), 4, p[2], p[6]);
    s2p_step(iBuilder, bit33337777[0], bit33337777[1], iBuilder->simd_himask(8), 4, p[3], p[7]);
}


void generateS2PKernel(Module * m, IDISA::IDISA_Builder * iBuilder, KernelBuilder * kBuilder){

    kBuilder->addKernelInputStream(8, "byte_pack");
    for(int i=0; i<8; i++)
        kBuilder->addKernelOutputStream(1);

    int segBlocks = kBuilder->getSegmentBlocks();

    kBuilder->PrepareDoBlockFunction();    
    struct Inputs inputs = kBuilder->openDoBlock();
    struct Outputs outputs;

    valptr basis_bit[segBlocks][8];
    for(int i=0; i<segBlocks; i++){
        s2p(iBuilder, inputs.streams[i], basis_bit[i]);
        outputs.streams.push_back(basis_bit[i]);
    }

    kBuilder->closeDoBlock(outputs);

    kBuilder->finalizeMethods();

}











