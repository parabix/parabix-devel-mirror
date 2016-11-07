/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CC_COMPILER_H
#define CC_COMPILER_H


#include <re/re_cc.h>
#include <pablo/builder.hpp>
#include <string>

namespace pablo {
    class PabloFunction;
    class Var;
}

namespace cc {

class CC_NameMap;

class CC_Compiler{

    friend pablo::PabloFunction * ParabixCharacterClassFunction(const std::string &, const std::vector<re::CC *> &, const unsigned);

public:

    using Vars = std::vector<pablo::Var *>;

    CC_Compiler(pablo::PabloFunction & function, const unsigned encodingBits = 8, const std::string prefix = "basis");

    pablo::PabloAST * compileCC(const re::CC *cc);

    pablo::PabloAST *compileCC(const re::CC *cc, pablo::PabloBlock & block);

    pablo::PabloAST * compileCC(const re::CC *cc, pablo::PabloBuilder & builder);

    pablo::PabloAST * compileCC(const std::string & canonicalName, const re::CC *cc, pablo::PabloBlock & block);

    pablo::PabloAST * compileCC(const std::string & canonicalName, const re::CC *cc, pablo::PabloBuilder & builder);

    pablo::PabloBuilder & getBuilder();

    const std::vector<pablo::PabloAST *> & getBasisBits() {
        return mBasisBit;
    }

    bool isUTF_16() const {
        return mEncodingBits == 16;
    }

private:
    pablo::PabloAST * getBasisVar(const unsigned n) const;
    template<typename PabloBlockOrBuilder>
    pablo::PabloAST * bit_pattern_expr(const unsigned pattern, unsigned selected_bits, PabloBlockOrBuilder & pb);
    template<typename PabloBlockOrBuilder>
    pablo::PabloAST * char_test_expr(const re::codepoint_t ch, PabloBlockOrBuilder & pb);
    template<typename PabloBlockOrBuilder>
    pablo::PabloAST * make_range(const re::codepoint_t n1, const re::codepoint_t n2, PabloBlockOrBuilder & pb);
    template<typename PabloBlockOrBuilder>
    pablo::PabloAST * GE_Range(const unsigned N, const unsigned n, PabloBlockOrBuilder & pb);
    template<typename PabloBlockOrBuilder>
    pablo::PabloAST * LE_Range(const unsigned N, const unsigned n, PabloBlockOrBuilder & pb);
    template<typename PabloBlockOrBuilder>
    pablo::PabloAST * char_or_range_expr(const re::codepoint_t lo, const re::codepoint_t hi, PabloBlockOrBuilder & pb);
    template<typename PabloBlockOrBuilder>
    pablo::PabloAST * charset_expr(const re::CC *cc, PabloBlockOrBuilder & pb);
private:    
    pablo::PabloBuilder             mBuilder;
    std::vector<pablo::PabloAST *>  mBasisBit;
    const unsigned                  mEncodingBits;
    unsigned                        mEncodingMask;
};

inline pablo::PabloAST *CC_Compiler::compileCC(const re::CC *cc) {
    return compileCC(cc, mBuilder);
}

inline pablo::PabloAST * CC_Compiler::compileCC(const re::CC *cc, pablo::PabloBlock & block) {
    return compileCC(cc->canonicalName(re::ByteClass), cc, block);
}

inline pablo::PabloAST *CC_Compiler::compileCC(const re::CC *cc, pablo::PabloBuilder & builder) {
    return compileCC(cc->canonicalName(re::ByteClass), cc, builder);
}

inline pablo::PabloBuilder & CC_Compiler::getBuilder() {
    return mBuilder;
}

pablo::PabloFunction * ParabixCharacterClassFunction(const std::string & name, const std::vector<re::CC *> & charClasses, const unsigned basisBitsCount);

}

#endif // CC_COMPILER_H


