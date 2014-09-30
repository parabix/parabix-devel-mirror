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
#include <pablo/ps_pablos.h>
#include <pablo/pe_pabloe.h>
#include <re/re_cc.h>

namespace cc {

struct Expression{
    std::string expr_string;
    PabloE* pablo_expr;
};

class CC_Compiler{
    typedef std::map<std::string, re::RE*>      REMap;
    typedef std::map<std::string, Expression*>  ExpressionMap;
    typedef ExpressionMap::iterator             MapIterator;

public:
    CC_Compiler(const UTF_Encoding encoding, const std::string basis_pattern, const std::string gensym_pattern);
    std::string compile1(re::CC* cc);
    void compile_from_map(const REMap & re_map);
    std::list<PabloS*> get_compiled();
private:
    void process_re_map(const REMap &re_map);
    void process_re(const re::RE *re);
    std::string bit_var(int n);
    PabloE* make_bitv(int n);
    PabloE* bit_pattern_expr(int pattern, int selected_bits);
    PabloE* char_test_expr(const re::CodePointType ch);
    PabloE* make_range(const re::CodePointType n1, const re::CodePointType n2);
    PabloE* GE_Range(int N, int n);
    PabloE* LE_Range(int N, int n);
    PabloE* char_or_range_expr(const re::CodePointType lo, const re::CodePointType hi);
    PabloE* charset_expr(const re::CC *cc);
    Expression* expr2pabloe(PabloE* expr);
    void cc2pablos(const re::CC *cc);

    UTF_Encoding mEncoding;

    void add_predefined(std::string key_value, Expression *mapped_value);
    Expression* add_assignment(std::string value, Expression* expr);
    Expression* expr_to_variable(Expression* cgo);

    std::string mGenSym_Template;
    int mGenSymCounter;
    std::list<PabloS*> mStmtsl;
    ExpressionMap mCommon_Expression_Map;
};

}

#endif // CC_COMPILER_H


