#ifndef IDISA_AVX_BUILDER_H
#define IDISA_AVX_BUILDER_H

/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
*/

#include <IDISA/idisa_builder.h>
#include <IDISA/idisa_sse_builder.h>

using namespace llvm;

namespace IDISA {

class IDISA_AVX_Builder : public IDISA_SSE_Builder {
public:
    
    IDISA_AVX_Builder(Module * m, Type * bitBlockType) : IDISA_SSE_Builder(m, bitBlockType) {
    }
    Value * hsimd_signmask(unsigned fw, Value * a) override;
    ~IDISA_AVX_Builder() {};

};

class IDISA_AVX2_Builder : public IDISA_AVX_Builder {
public:
    
    IDISA_AVX2_Builder(Module * m, Type * bitBlockType) : IDISA_AVX_Builder(m, bitBlockType) {
    }
    Value * hsimd_packh(unsigned fw, Value * a, Value * b) override;
    Value * hsimd_packl(unsigned fw, Value * a, Value * b) override;
    Value * esimd_mergeh(unsigned fw, Value * a, Value * b) override;
    Value * esimd_mergel(unsigned fw, Value * a, Value * b) override;
    Value * hsimd_packh_in_lanes(unsigned lanes, unsigned fw, Value * a, Value * b) override;
    Value * hsimd_packl_in_lanes(unsigned lanes, unsigned fw, Value * a, Value * b) override;

    ~IDISA_AVX2_Builder() {};
};
    
}
#endif // IDISA_AVX_BUILDER_H
