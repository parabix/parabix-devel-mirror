#ifndef IDISA_SSE_BUILDER_H
#define IDISA_SSE_BUILDER_H

/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <IR_Gen/idisa_builder.h>

namespace IDISA {

class IDISA_SSE_Builder : public virtual IDISA_Builder {
public:
  
    IDISA_SSE_Builder(llvm::LLVMContext & C, unsigned bitBlockWidth, unsigned stride)
    : IDISA_Builder(C, bitBlockWidth, stride) {

    }

    virtual std::string getBuilderUniqueName() override;
    llvm::Value * hsimd_signmask(unsigned fw, llvm::Value * a) override;
    llvm::Value * mvmd_compress(unsigned fw, llvm::Value * a, llvm::Value * select_mask) override;
    ~IDISA_SSE_Builder() {}
};

class IDISA_SSE2_Builder : public IDISA_SSE_Builder {
public:
  
    IDISA_SSE2_Builder(llvm::LLVMContext & C, unsigned bitBlockWidth, unsigned stride)
    : IDISA_Builder(C, bitBlockWidth, stride)
    , IDISA_SSE_Builder(C, bitBlockWidth, stride) {

    }

    virtual std::string getBuilderUniqueName() override;
    llvm::Value * hsimd_signmask(unsigned fw, llvm::Value * a) override;
    llvm::Value * hsimd_packh(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * hsimd_packl(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    std::pair<llvm::Value *, llvm::Value *> bitblock_advance(llvm::Value * a, llvm::Value * shiftin, unsigned shift) override;
    llvm::Value * mvmd_shuffle(unsigned fw, llvm::Value * a, llvm::Value * shuffle_table) override;
    ~IDISA_SSE2_Builder() {}
};

}

#endif // IDISA_SSE_BUILDER_H
