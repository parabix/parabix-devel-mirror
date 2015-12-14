#ifndef IDISA_AVX_BUILDER_H
#define IDISA_AVX_BUILDER_H

/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
*/

#include <IDISA/idisa_builder.h>

using namespace llvm;

namespace IDISA {

class IDISA_AVX_Builder : public IDISA_Builder {
public:
    
    IDISA_AVX_Builder(Type * bitBlockType) : IDISA_Builder(bitBlockType) {
    }
    ~IDISA_AVX_Builder() {};

};

class IDISA_AVX2_Builder : public IDISA_AVX_Builder {
public:
    
    IDISA_AVX2_Builder(Type * bitBlockType) : IDISA_AVX_Builder(bitBlockType) {
    }
    Value * hsimd_signmask(unsigned fw, Value * a) override;

    ~IDISA_AVX2_Builder() {};
};
    
}
#endif // IDISA_AVX_BUILDER_H
