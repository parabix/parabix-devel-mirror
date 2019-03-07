#ifndef IDISA_AVX_BUILDER_H
#define IDISA_AVX_BUILDER_H

/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
*/

#include <IR_Gen/idisa_sse_builder.h>
#include <toolchain/toolchain.h>

namespace IDISA {
    
    const unsigned AVX_width = 256;
    const unsigned AVX512_width = 512;

class IDISA_AVX_Builder : public IDISA_SSE2_Builder {
public:
    static const unsigned NativeBitBlockWidth = AVX_width;
    IDISA_AVX_Builder(llvm::LLVMContext & C, unsigned vectorWidth, unsigned laneWidth)
    : IDISA_Builder(C, AVX_width, vectorWidth, laneWidth)
    , IDISA_SSE2_Builder(C, vectorWidth, laneWidth)
    {
    }

    virtual std::string getBuilderUniqueName() override;

    llvm::Value * hsimd_signmask(unsigned fw, llvm::Value * a) override;

    ~IDISA_AVX_Builder() {}
};

class IDISA_AVX2_Builder : public IDISA_AVX_Builder {
public:
    static const unsigned NativeBitBlockWidth = AVX_width;
    IDISA_AVX2_Builder(llvm::LLVMContext & C, unsigned vectorWidth, unsigned laneWidth)
    : IDISA_Builder(C, AVX_width, vectorWidth, laneWidth)
    , IDISA_AVX_Builder(C, vectorWidth, laneWidth) {
        llvm::StringMap<bool> features;
        hasBMI2 = llvm::sys::getHostCPUFeatures(features) && features.lookup("bmi2");
    }

    virtual std::string getBuilderUniqueName() override;
    llvm::Value * hsimd_packh(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * hsimd_packl(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * hsimd_packus(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * hsimd_packss(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * esimd_mergeh(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * esimd_mergel(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * hsimd_packh_in_lanes(unsigned lanes, unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * hsimd_packl_in_lanes(unsigned lanes, unsigned fw, llvm::Value * a, llvm::Value * b) override;
    std::pair<llvm::Value *, llvm::Value *> bitblock_add_with_carry(llvm::Value * a, llvm::Value * b, llvm::Value * carryin) override;
    std::pair<llvm::Value *, llvm::Value *> bitblock_indexed_advance(llvm::Value * a, llvm::Value * index_strm, llvm::Value * shiftin, unsigned shift) override;
    llvm::Value * hsimd_signmask(unsigned fw, llvm::Value * a) override;
    llvm::Value * mvmd_srl(unsigned fw, llvm::Value * a, llvm::Value * shift, const bool safe = false) override;
    llvm::Value * mvmd_sll(unsigned fw, llvm::Value * a, llvm::Value * shift, const bool safe = false) override;
    llvm::Value * mvmd_shuffle(unsigned fw, llvm::Value * data_table, llvm::Value * index_vector) override;
    llvm::Value * mvmd_compress(unsigned fw, llvm::Value * a, llvm::Value * select_mask) override;
    llvm::Value * simd_pext(unsigned fw, llvm::Value * v, llvm::Value * extract_mask) override;
    llvm::Value * simd_pdep(unsigned fw, llvm::Value * v, llvm::Value * deposit_mask) override;
    

    ~IDISA_AVX2_Builder() {}
protected:
    bool hasBMI2;
};

#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 8, 0)
class IDISA_AVX512F_Builder : public IDISA_AVX2_Builder {
public:
    static const unsigned NativeBitBlockWidth = AVX512_width;
    IDISA_AVX512F_Builder(llvm::LLVMContext & C, unsigned vectorWidth, unsigned laneWidth)
    : IDISA_Builder(C, AVX512_width, vectorWidth, laneWidth)
    , IDISA_AVX2_Builder(C, vectorWidth, laneWidth) {
        getAVX512Features();
    }

    virtual std::string getBuilderUniqueName() override;
    void getAVX512Features();
    llvm::Value * hsimd_packh(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * hsimd_packl(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * hsimd_packus(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * hsimd_packss(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * esimd_mergeh(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * esimd_mergel(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * esimd_bitspread(unsigned fw, llvm::Value * bitmask) override;
    llvm::Value * simd_popcount(unsigned fw, llvm::Value * a) override;
    llvm::Value * mvmd_slli(unsigned fw, llvm::Value * a, unsigned shift) override;
    llvm::Value * mvmd_dslli(unsigned fw, llvm::Value * a, llvm::Value * b, unsigned shift) override;
    llvm::Value * hsimd_signmask(unsigned fw, llvm::Value * a) override;
    llvm::Value * mvmd_shuffle(unsigned fw, llvm::Value * data_table, llvm::Value * index_vector) override;
    llvm::Value * mvmd_shuffle2(unsigned fw, llvm::Value * table0, llvm::Value * table1, llvm::Value * index_vector) override;
    llvm::Value * mvmd_compress(unsigned fw, llvm::Value * a, llvm::Value * select_mask) override;
    llvm::Value * mvmd_srl(unsigned fw, llvm::Value * a, llvm::Value * shift, const bool safe) override;
    llvm::Value * mvmd_sll(unsigned fw, llvm::Value * a, llvm::Value * shift, const bool safe) override;
    llvm::Value * simd_if(unsigned fw, llvm::Value * cond, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * simd_ternary(unsigned char mask, llvm::Value * a, llvm::Value * b, llvm::Value * c) override;

    ~IDISA_AVX512F_Builder() {
    }
private:
    struct Features {
        //not an exhaustive list, can be extended if needed
        bool hasAVX512CD = false;
        bool hasAVX512BW = false;
        bool hasAVX512DQ = false;
        bool hasAVX512VL = false;
        bool hasAVX512VBMI = false;
        bool hasAVX512VBMI2 = false;
        bool hasAVX512VPOPCNTDQ = false;
    };
    Features hostCPUFeatures;
};
#endif

}
#endif // IDISA_AVX_BUILDER_H
