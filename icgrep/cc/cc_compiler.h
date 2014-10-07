/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CC_COMPILER_H
#define CC_COMPILER_H

#include "utf_encoding.h"
#include <pablo/codegenstate.h>
#include <pablo/pe_pabloe.h>
#include <re/re_cc.h>
#include <string>
#include <set>

namespace cc {

class CC_Compiler{
    typedef std::map<std::string, re::RE *> REMap;
    typedef std::set<std::string>           ComputedSet;
public:

    CC_Compiler(pablo::PabloBlock & cg, const Encoding encoding, const std::string basis_pattern = "basis", const std::string gensym_pattern = "temp");

    void compile(const REMap & re_map);

    const std::string getBasisPattern() const {
        return mBasisPattern;
    }

private:
    void process_re(const re::RE *re);
    pablo::Var * getBasisVar(const int n) const;
    pablo::PabloE * bit_pattern_expr(const unsigned pattern, unsigned selected_bits);
    pablo::PabloE * char_test_expr(const re::CodePointType ch);
    pablo::PabloE * make_range(const re::CodePointType n1, const re::CodePointType n2);
    pablo::PabloE * GE_Range(const unsigned N, const unsigned n);
    pablo::PabloE * LE_Range(const unsigned N, const unsigned n);
    pablo::PabloE * char_or_range_expr(const re::CodePointType lo, const re::CodePointType hi);
    pablo::PabloE * charset_expr(const re::CC *cc);
    void process(const re::CC *cc);

    pablo::PabloBlock &         mCG;
    std::vector<pablo::Var *>   mBasisBit;
    const Encoding              mEncoding;
    const std::string           mGenSymPattern;
    const std::string           mBasisPattern;
    ComputedSet                 mComputedSet;
};

}

#endif // CC_COMPILER_H


