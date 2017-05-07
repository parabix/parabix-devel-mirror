#ifndef IDISA_AVX_BUILDER_H
#define IDISA_AVX_BUILDER_H

/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
*/

#include <IR_Gen/idisa_sse_builder.h>

using namespace llvm;

namespace IDISA {

class IDISA_AVX_Builder : public IDISA_SSE2_Builder {
public:
    
    IDISA_AVX_Builder(llvm::LLVMContext & C, unsigned archBitWidth, unsigned bitBlockWidth, unsigned stride)
    : IDISA_Builder(C, archBitWidth, bitBlockWidth, stride)
    , IDISA_SSE2_Builder(C, archBitWidth, bitBlockWidth, stride)
    {

    }

    virtual std::string getBuilderUniqueName() override;

    Value * hsimd_signmask(unsigned fw, Value * a) override;

    ~IDISA_AVX_Builder() {}

};

class IDISA_AVX2_Builder : public IDISA_AVX_Builder {
public:
    
    IDISA_AVX2_Builder(llvm::LLVMContext & C, unsigned archBitWidth, unsigned bitBlockWidth, unsigned stride)
    : IDISA_Builder(C, archBitWidth, bitBlockWidth, stride)
    , IDISA_AVX_Builder(C, archBitWidth, bitBlockWidth, stride) {

    }

    virtual std::string getBuilderUniqueName() override;
    Value * hsimd_packh(unsigned fw, Value * a, Value * b) override;
    Value * hsimd_packl(unsigned fw, Value * a, Value * b) override;
    Value * esimd_mergeh(unsigned fw, Value * a, Value * b) override;
    Value * esimd_mergel(unsigned fw, Value * a, Value * b) override;
    Value * hsimd_packh_in_lanes(unsigned lanes, unsigned fw, Value * a, Value * b) override;
    Value * hsimd_packl_in_lanes(unsigned lanes, unsigned fw, Value * a, Value * b) override;
    std::pair<Value *, Value *> bitblock_add_with_carry(Value * a, Value * b, Value * carryin) override;

    ~IDISA_AVX2_Builder() {}
};
    
}
#endif // IDISA_AVX_BUILDER_H
