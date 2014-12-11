/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CC_COMPILER_H
#define CC_COMPILER_H

#include "utf_encoding.h"
#include <pablo/codegenstate.h>
#include <pablo/pabloAST.h>
#include <re/re_cc.h>
#include <unordered_map>
#include <string>
#include <iostream>

namespace cc {

class CC_NameMap;

class CC_Compiler{
    typedef std::vector<std::pair<const re::CC*, pablo::Assign*>> ConstraintVector;
public:

    CC_Compiler(pablo::PabloBlock & cg, const Encoding encoding, const bool annotateVariableConstraints = false, const std::string basis_pattern = "basis");

    std::vector<pablo::Var *> compile(const CC_NameMap & nameMap);

    pablo::Var * compileCC(const re::CC *cc);
private:


    pablo::PabloAST * compile_re(re::RE * re);
    pablo::PabloAST * compile_re(re::Name * name);
    pablo::PabloAST * compile_re(const re::Alt * alt);
    pablo::PabloAST * compile_re(const re::Seq *seq);

    pablo::Var * getBasisVar(const int n) const;
    pablo::PabloAST * bit_pattern_expr(const unsigned pattern, unsigned selected_bits);
    pablo::PabloAST * char_test_expr(const re::CodePointType ch);
    pablo::PabloAST * make_range(const re::CodePointType n1, const re::CodePointType n2);
    pablo::PabloAST * GE_Range(const unsigned N, const unsigned n);
    pablo::PabloAST * LE_Range(const unsigned N, const unsigned n);
    pablo::PabloAST * char_or_range_expr(const re::CodePointType lo, const re::CodePointType hi);
    pablo::PabloAST * charset_expr(const re::CC *cc);

    void computeVariableConstraints();

private:
    pablo::PabloBlock &         mCG;
    const bool                  mAnnotateVariableConstraints;
    std::vector<pablo::Var *>   mBasisBit;
    const Encoding              mEncoding;
    ConstraintVector            mVariableVector;
};

}

#endif // CC_COMPILER_H


