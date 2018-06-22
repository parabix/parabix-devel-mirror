#ifndef IDISA_I64_BUILDER_H
#define IDISA_I64_BUILDER_H

/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#include <IR_Gen/idisa_builder.h>

namespace IDISA {
    const unsigned I64_width = 64;

class IDISA_I64_Builder : public virtual IDISA_Builder {
public:
    const unsigned NativeBitBlockWidth = I64_width;
  
    IDISA_I64_Builder(llvm::LLVMContext & C, unsigned bitBlockWidth, unsigned laneWidth)
    : IDISA_Builder(C, I64_width, bitBlockWidth, laneWidth) {

    } 

    virtual std::string getBuilderUniqueName() override;

    llvm::Value * hsimd_packh(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    llvm::Value * hsimd_packl(unsigned fw, llvm::Value * a, llvm::Value * b) override;
    ~IDISA_I64_Builder() {}

};

}

#endif // IDISA_I64_BUILDER_H
