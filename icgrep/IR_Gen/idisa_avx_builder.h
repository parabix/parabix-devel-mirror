#ifndef IDISA_AVX_BUILDER_H
#define IDISA_AVX_BUILDER_H

/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
*/

#include <IR_Gen/idisa_sse_builder.h>

namespace IDISA {

class IDISA_AVX_Builder : public IDISA_SSE2_Builder {
public:

    IDISA_AVX_Builder(llvm::LLVMContext & C, unsigned vectorWidth, unsigned stride)
    : IDISA_Builder(C, vectorWidth, stride)
    , IDISA_SSE2_Builder(C, vectorWidth, stride)
    {

    }

    virtual std::string getBuilderUniqueName() override;

    llvm::Value * hsimd_signmask(unsigned fw, llvm::Value * a) override;

    ~IDISA_AVX_Builder() {}

};

class IDISA_AVX2_Builder : public IDISA_AVX_Builder {
public:

    IDISA_AVX2_Builder(llvm::LLVMContext & C, unsigned vectorWidth, unsigned stride)
    : IDISA_Builder(C, vectorWidth, stride)
    , IDISA_AVX_Builder(C, vectorWidth, stride) {

    }

    virtual std::string getBuilderUniqueName() override;
    llvm::Value * hsimd_packh(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * hsimd_packl(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * esimd_mergeh(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * esimd_mergel(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * hsimd_packh_in_lanes(unsigned lanes, unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * hsimd_packl_in_lanes(unsigned lanes, unsigned fw, llvm::Value * a, llvm::Value * b) override;
    std::pair<llvm::Value *, llvm::Value *> bitblock_add_with_carry(llvm::Value * a, llvm::Value * b, llvm::Value * carryin) override;
    std::pair<llvm::Value *, llvm::Value *> bitblock_indexed_advance(llvm::Value * a, llvm::Value * index_strm, llvm::Value * shiftin, unsigned shift) override;
    llvm::Value * hsimd_signmask(unsigned fw, llvm::Value * a) override;

    ~IDISA_AVX2_Builder() {}
};

class IDISA_AVX512F_Builder : public IDISA_AVX2_Builder {
public:

    IDISA_AVX512F_Builder(llvm::LLVMContext & C, unsigned vectorWidth, unsigned stride)
    : IDISA_Builder(C, vectorWidth, stride)
    , IDISA_AVX2_Builder(C, vectorWidth, stride) {
    }

    virtual std::string getBuilderUniqueName() override;
    llvm::Value * hsimd_packh(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * hsimd_packl(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    
    ~IDISA_AVX512F_Builder() {}
};


}
#endif // IDISA_AVX_BUILDER_H
