/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CC_COMPILER_H
#define CC_COMPILER_H

#include <string>
#include <list>
#include <map>
#include "utf_encoding.h"
#include <pablo/codegenstate.h>
#include <pablo/pe_pabloe.h>
#include <re/re_cc.h>


namespace cc {

struct Expression{
    std::string expr_string;
    pablo::PabloE * pablo_expr;
};

class CC_Compiler{
    typedef std::map<std::string, re::RE*>      REMap;
    typedef std::map<std::string, Expression*>  ExpressionMap;
    typedef ExpressionMap::iterator             MapIterator;
    typedef std::list<pablo::PabloE *>          List;
public:
    CC_Compiler(const Encoding encoding, const std::string basis_pattern = "basis", const std::string gensym_pattern = "temp");
    void compile_from_map(const REMap & re_map);
    List get_compiled();

    const std::string getBasisPattern() const {
        return mBasisPattern;
    }

private:
    void process_re_map(const REMap &re_map);
    void process_re(const re::RE *re);
    std::string bit_var(int n);
    pablo::PabloE * make_bitv(int n);
    pablo::PabloE * bit_pattern_expr(int pattern, int selected_bits);
    pablo::PabloE * char_test_expr(const re::CodePointType ch);
    pablo::PabloE * make_range(const re::CodePointType n1, const re::CodePointType n2);
    pablo::PabloE * GE_Range(int N, int n);
    pablo::PabloE * LE_Range(int N, int n);
    pablo::PabloE * char_or_range_expr(const re::CodePointType lo, const re::CodePointType hi);
    pablo::PabloE * charset_expr(const re::CC *cc);
    Expression* expr2pabloe(pablo::PabloE * expr);
    void cc2pablos(const re::CC *cc);

    void add_predefined(std::string key_value, Expression *mapped_value);
    Expression* add_assignment(std::string value, Expression* expr);
    Expression* expr_to_variable(Expression* cgo);


    Encoding                    mEncoding;
    const std::string           mBasisPattern;
    const std::string           mGenSymPattern;
    int                         mGenSymCounter;
    List                        mStmtsl;
    ExpressionMap               mCommon_Expression_Map;
};

}

#endif // CC_COMPILER_H


