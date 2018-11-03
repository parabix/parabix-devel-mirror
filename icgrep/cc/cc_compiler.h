/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CC_COMPILER_H
#define CC_COMPILER_H


#include <re/re_cc.h>
#include <pablo/builder.hpp>
#include <cc/alphabet.h>

namespace cc {

class CC_Compiler {
public:
    
    virtual pablo::PabloAST * compileCC(const re::CC *cc) = 0;
    
    virtual pablo::PabloAST * compileCC(const re::CC *cc, pablo::PabloBlock & block) = 0;
    
    virtual pablo::PabloAST * compileCC(const re::CC *cc, pablo::PabloBuilder & builder) = 0;
    
    virtual pablo::PabloAST * compileCC(const std::string & canonicalName, const re::CC *cc, pablo::PabloBlock & block) = 0;
    
    virtual pablo::PabloAST * compileCC(const std::string & canonicalName, const re::CC *cc, pablo::PabloBuilder & builder) = 0;
    
    virtual ~CC_Compiler(){}

protected:
    CC_Compiler(pablo::PabloBlock * scope);
    pablo::PabloBuilder             mBuilder;
};
    
    
class Parabix_CC_Compiler : public CC_Compiler {
public:
    
    Parabix_CC_Compiler(pablo::PabloBlock * scope, std::vector<pablo::PabloAST *> basisBitSet, cc::BitNumbering b = BitNumbering::LittleEndian);
    
    pablo::PabloAST * compileCC(const re::CC *cc) override;
    
    pablo::PabloAST * compileCC(const re::CC *cc, pablo::PabloBlock & block) override;
    
    pablo::PabloAST * compileCC(const re::CC *cc, pablo::PabloBuilder & builder) override;
    
    pablo::PabloAST * compileCC(const std::string & name, const re::CC *cc, pablo::PabloBlock & block) override;

    pablo::PabloAST * compileCC(const std::string & name, const re::CC *cc, pablo::PabloBuilder & builder) override;
    
    ~Parabix_CC_Compiler() {}

private:
    template<typename PabloBlockOrBuilder>
    pablo::PabloAST * getBasisVar(const unsigned n, PabloBlockOrBuilder & pb) const;
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
    const unsigned                  mEncodingBits;
    BitNumbering               mBasisSetNumbering;
    std::vector<pablo::PabloAST *>  mBasisBit;
    unsigned                        mEncodingMask;
};

    inline pablo::PabloAST * Parabix_CC_Compiler::compileCC(const re::CC *cc) {
        return compileCC(cc, mBuilder);
    }
    
    inline pablo::PabloAST * Parabix_CC_Compiler::compileCC(const re::CC *cc, pablo::PabloBlock & block) {
        return compileCC(cc->canonicalName(), cc, block);
    }
    
    inline pablo::PabloAST * Parabix_CC_Compiler::compileCC(const re::CC *cc, pablo::PabloBuilder & builder) {
        return compileCC(cc->canonicalName(), cc, builder);
    }
    
    

class Direct_CC_Compiler : public CC_Compiler {
public:
    
    Direct_CC_Compiler(pablo::PabloBlock * scope, pablo::PabloAST * codeUnitStream);
    
    pablo::PabloAST * compileCC(const re::CC *cc) override;
    
    pablo::PabloAST * compileCC(const re::CC *cc, pablo::PabloBlock & block) override;
    
    pablo::PabloAST * compileCC(const re::CC *cc, pablo::PabloBuilder & builder) override;
    
    pablo::PabloAST * compileCC(const std::string & name, const re::CC *cc, pablo::PabloBlock & block) override;
    
    pablo::PabloAST * compileCC(const std::string & name, const re::CC *cc, pablo::PabloBuilder & builder) override;
    
    ~Direct_CC_Compiler() {}
    
private:
    pablo::PabloAST *               mCodeUnitStream;
};

    
inline pablo::PabloAST * Direct_CC_Compiler::compileCC(const re::CC *cc) {
    return compileCC(cc, mBuilder);
}

inline pablo::PabloAST * Direct_CC_Compiler::compileCC(const re::CC *cc, pablo::PabloBlock & block) {
    return compileCC(cc->canonicalName(), cc, block);
}

inline pablo::PabloAST * Direct_CC_Compiler::compileCC(const re::CC *cc, pablo::PabloBuilder & builder) {
    return compileCC(cc->canonicalName(), cc, builder);
}



pablo::PabloAST * compileCCfromCodeUnitStream(const re::CC *cc, pablo::PabloAST * codeUnitStream, pablo::PabloBuilder & pb);
    
}

#endif // CC_COMPILER_H


