/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef CC_COMPILER_TARGET_H
#define CC_COMPILER_TARGET_H

#include "cc_compiler.h"

namespace cc {

class Parabix_CC_Compiler_Builder : public CC_Compiler {
public:
    using CC_Compiler::compileCC;
    
    Parabix_CC_Compiler_Builder(pablo::PabloBlock * scope, std::vector<pablo::PabloAST *> basisBitSet);
    pablo::PabloAST * compileCC(const std::string & name, const re::CC *cc, pablo::PabloBlock & block) override;
    pablo::PabloAST * compileCC(const std::string & name, const re::CC *cc, pablo::PabloBuilder & builder) override;
    ~Parabix_CC_Compiler_Builder() {}
    
protected:
    std::unique_ptr<CC_Compiler> ccc;
};

}

#endif