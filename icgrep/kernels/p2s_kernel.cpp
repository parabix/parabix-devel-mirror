#include "p2s_kernel.h"
#include "kernels/kernel.h"
#include "IDISA/idisa_builder.h"

namespace kernel{
	
void p2s_step(IDISA::IDISA_Builder * iBuilder, Value * p0, Value * p1, Value * hi_mask, unsigned shift, Value * &s1, Value * &s0) {
    Value * t0 = iBuilder->simd_if(1, hi_mask, p0, iBuilder->simd_srli(16, p1, shift));
    Value * t1 = iBuilder->simd_if(1, hi_mask, iBuilder->simd_slli(16, p0, shift), p1);
    s1 = iBuilder->esimd_mergeh(8, t1, t0);
    s0 = iBuilder->esimd_mergel(8, t1, t0);
}

inline void p2s(IDISA::IDISA_Builder * iBuilder, Value * p[], Value * s[]) {
    Value * bit00004444[2];
    Value * bit22226666[2];
    Value * bit11115555[2];
    Value * bit33337777[2];
    p2s_step(iBuilder, p[0], p[4], iBuilder->simd_himask(8), 4, bit00004444[1], bit00004444[0]);
    p2s_step(iBuilder, p[1], p[5], iBuilder->simd_himask(8), 4, bit11115555[1], bit11115555[0]);
    p2s_step(iBuilder, p[2], p[6], iBuilder->simd_himask(8), 4, bit22226666[1], bit22226666[0]);
    p2s_step(iBuilder, p[3], p[7], iBuilder->simd_himask(8), 4, bit33337777[1], bit33337777[0]);

    Value * bit00224466[4];
    Value * bit11335577[4];
    for (unsigned j = 0; j<2; j++) {
        p2s_step(iBuilder, bit00004444[j], bit22226666[j],iBuilder->simd_himask(4), 2, bit00224466[2*j+1], bit00224466[2*j]);
        p2s_step(iBuilder, bit11115555[j], bit33337777[j],iBuilder->simd_himask(4), 2, bit11335577[2*j+1], bit11335577[2*j]);
    }
    for (unsigned j = 0; j<4; j++) {
        p2s_step(iBuilder, bit00224466[j], bit11335577[j], iBuilder->simd_himask(2), 1, s[2*j+1], s[2*j]);
    }
}
    		
void generateP2SKernel(Module * m, IDISA::IDISA_Builder * iBuilder, KernelBuilder * kBuilder) {
    for (unsigned i = 0; i < 8; ++i) {
        kBuilder->addInputStream(1);
    }
    kBuilder->addOutputStream(8);
    kBuilder->prepareFunction();
    Value * input[8];
    for (unsigned j = 0; j < 8; ++j) {
        input[j] = iBuilder->CreateBlockAlignedLoad(kBuilder->getInputStream(j));
    }
    Value * output[8];
    p2s(iBuilder, input, output);
    Value * output_ptr = kBuilder->getOutputStream(0);
    for (unsigned j = 0; j < 8; ++j) {

        iBuilder->CreateBlockAlignedStore(output[j], iBuilder->CreateGEP(output_ptr, std::vector<Value *>({ iBuilder->getInt32(0), iBuilder->getInt32(j) })));
    }
    kBuilder->finalize();
}

}
