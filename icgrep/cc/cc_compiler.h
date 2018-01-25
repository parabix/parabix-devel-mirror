/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CC_COMPILER_H
#define CC_COMPILER_H


#include <re/re_cc.h>
#include <pablo/builder.hpp>
#include <kernels/interface.h>
#include <string>

namespace cc {

class CC_Compiler{
    friend class ParabixCharacterClassKernelBuilder;
public:
    
    CC_Compiler(pablo::PabloKernel * kernel, std::vector<pablo::PabloAST *> basisBitSet);
    
    pablo::PabloAST * compileCC(const re::CC *cc);

    pablo::PabloAST * compileCC(const re::CC *cc, pablo::PabloBlock & block);

    pablo::PabloAST * compileCC(const re::CC *cc, pablo::PabloBuilder & builder);

    pablo::PabloAST * compileCC(const std::string & canonicalName, const re::CC *cc, pablo::PabloBlock & block);

    pablo::PabloAST * compileCC(const std::string & canonicalName, const re::CC *cc, pablo::PabloBuilder & builder);

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
    const unsigned                  mEncodingBits;
    std::vector<pablo::PabloAST *>  mBasisBit;
    unsigned                        mEncodingMask;
};

inline pablo::PabloAST *CC_Compiler::compileCC(const re::CC *cc) {
    return compileCC(cc, mBuilder);
}

inline pablo::PabloAST * CC_Compiler::compileCC(const re::CC *cc, pablo::PabloBlock & block) {
    return compileCC(cc->canonicalName(re::CC_type::ByteClass), cc, block);
}

inline pablo::PabloAST *CC_Compiler::compileCC(const re::CC *cc, pablo::PabloBuilder & builder) {
    return compileCC(cc->canonicalName(re::CC_type::ByteClass), cc, builder);
}

pablo::PabloAST * compileCCfromCodeUnitStream(const re::CC *cc, pablo::PabloAST * codeUnitStream, pablo::PabloBuilder & pb);
    
}

#endif // CC_COMPILER_H


