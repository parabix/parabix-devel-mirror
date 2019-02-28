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
    pablo::PabloAST * createCCOp3(op3_pair_t op3, pablo::PabloAST * expr1, pablo::PabloAST * expr2, pablo::PabloAST * expr3, pablo::PabloBuilder & builder) override;
    pablo::PabloAST * createUCDSequence(const unsigned byte_no, pablo::PabloAST * target, pablo::PabloAST * var, pablo::PabloAST * prefix, pablo::PabloBuilder & builder) override;
    pablo::PabloAST * createUCDSequence(const unsigned byte_no, const unsigned len, pablo::PabloAST * target, pablo::PabloAST * var, pablo::PabloAST * prefix, pablo::PabloAST * suffix, pablo::PabloBuilder & builder) override;
    ~Parabix_CC_Compiler_Builder() {}
    
protected:
    std::unique_ptr<CC_Compiler> ccc;
};

}

#endif