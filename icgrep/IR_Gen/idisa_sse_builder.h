#ifndef IDISA_SSE_BUILDER_H
#define IDISA_SSE_BUILDER_H

/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <IR_Gen/idisa_builder.h>

using namespace llvm;

namespace IDISA {

class IDISA_SSE_Builder : public IDISA_Builder {
public:
  
    IDISA_SSE_Builder(llvm::Module * const module, unsigned archBitWidth, unsigned bitBlockWidth)
    : IDISA_Builder(module, archBitWidth, bitBlockWidth, bitBlockWidth) {
    }

    virtual std::string getBuilderUniqueName() override;
    Value * hsimd_signmask(unsigned fw, Value * a) override;
    ~IDISA_SSE_Builder() {}

};

class IDISA_SSE2_Builder : public IDISA_SSE_Builder {
public:
  
    IDISA_SSE2_Builder(llvm::Module * const module, unsigned archBitWidth, unsigned bitBlockWidth)
    : IDISA_SSE_Builder(module, archBitWidth, bitBlockWidth) {
    }

    virtual std::string getBuilderUniqueName() override;
    Value * hsimd_signmask(unsigned fw, Value * a) override;
    Value * hsimd_packh(unsigned fw, Value * a, Value * b) override;
    Value * hsimd_packl(unsigned fw, Value * a, Value * b) override;
    std::pair<Value *, Value *> bitblock_advance(Value * a, Value * shiftin, unsigned shift) override;

    ~IDISA_SSE2_Builder() {}

};

}

#endif // IDISA_SSE_BUILDER_H
