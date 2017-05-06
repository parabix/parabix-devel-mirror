#ifndef IDISA_I64_BUILDER_H
#define IDISA_I64_BUILDER_H

/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#include <IR_Gen/idisa_builder.h>

using namespace llvm;

namespace IDISA {

class IDISA_I64_Builder : public virtual IDISA_Builder {
public:
  
    IDISA_I64_Builder(llvm::Module * const module, unsigned archBitWidth, unsigned bitBlockWidth, unsigned stride)
    : IDISA_Builder(module, archBitWidth, bitBlockWidth, stride) {

    } 

    virtual std::string getBuilderUniqueName() override;

    Value * hsimd_packh(unsigned fw, Value * a, Value * b) override;
    Value * hsimd_packl(unsigned fw, Value * a, Value * b) override;
    ~IDISA_I64_Builder() {}

};

}

#endif // IDISA_I64_BUILDER_H
