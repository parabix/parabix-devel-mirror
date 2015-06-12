/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CC_COMPILER_H
#define CC_COMPILER_H

#include <re/re_cc.h>
#include "utf_encoding.h"
#include <string>

namespace pablo {
    class PabloAST;
    class PabloBuilder;
    class PabloBlock;
    class Var;
    class Assign;
}

namespace cc {

class CC_NameMap;

class CC_Compiler{
public:

    CC_Compiler(pablo::PabloBlock & cg, const Encoding encoding, const std::string basis_pattern = "basis");

    std::vector<pablo::Var *> getBasisBits(const CC_NameMap & nameMap);

    pablo::Assign * compileCC(const re::CC *cc, pablo::PabloBlock & block);

    pablo::Assign * compileCC(const re::CC *cc, pablo::PabloBuilder & pb);

    pablo::Assign * compileCC(const re::CC *cc);

    void compileByteClasses(re::RE * re);

private:
    pablo::Var * getBasisVar(const int n) const;
    pablo::PabloAST * bit_pattern_expr(const unsigned pattern, unsigned selected_bits, pablo::PabloBuilder & pb);
    pablo::PabloAST * char_test_expr(const re::CodePointType ch, pablo::PabloBuilder & pb);
    pablo::PabloAST * make_range(const re::CodePointType n1, const re::CodePointType n2, pablo::PabloBuilder & pb);
    pablo::PabloAST * GE_Range(const unsigned N, const unsigned n, pablo::PabloBuilder & pb);
    pablo::PabloAST * LE_Range(const unsigned N, const unsigned n, pablo::PabloBuilder & pb);
    pablo::PabloAST * char_or_range_expr(const re::CodePointType lo, const re::CodePointType hi, pablo::PabloBuilder & pb);
    pablo::PabloAST * charset_expr(const re::CC *cc, pablo::PabloBuilder & pb);
private:    
    pablo::PabloBlock &         mCG;
    std::vector<pablo::Var *>   mBasisBit;
    const Encoding              mEncoding;
};

}

#endif // CC_COMPILER_H


