#ifndef IDISA_I64_BUILDER_H
#define IDISA_I64_BUILDER_H

/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#include <llvm/IR/Module.h>
#include <llvm/IR/Constant.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Value.h>
#include <IDISA/idisa_builder.h>

using namespace llvm;

namespace IDISA {

class IDISA_I64_Builder : public IDISA_Builder {
public:
  
    IDISA_I64_Builder(Module * m, unsigned archBitWidth, unsigned bitBlockWidth = 64, unsigned stride = 64)
    : IDISA_Builder(m, archBitWidth, bitBlockWidth, stride) {
    } 

    Value * hsimd_packh(unsigned fw, Value * a, Value * b) override;
    Value * hsimd_packl(unsigned fw, Value * a, Value * b) override;
    ~IDISA_I64_Builder() {}

};

}

#endif // IDISA_I64_BUILDER_H
