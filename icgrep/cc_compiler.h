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
#include "ps_pablos.h"
#include "pe_pabloe.h"
#include "pe_sel.h"
#include "pe_advance.h"
#include "pe_all.h"
#include "pe_and.h"
#include "pe_charclass.h"
#include "pe_matchstar.h"
#include "pe_not.h"
#include "pe_or.h"
#include "pe_var.h"
#include "pe_xor.h"
#include "re_cc.h"

struct Expression{
    std::string expr_string;
    PabloE* pablo_expr;
};


class CC_Compiler
{
public:
    CC_Compiler(const UTF_Encoding encoding, const std::string basis_pattern, const std::string gensym_pattern);
    std::string compile1(CC* cc);    
    void compile_from_map(const std::map<std::string, RE*>& re_map);    
    std::list<PabloS*> get_compiled();
private:
    void process_re_map(const std::map<std::string, RE*>& re_map);
    void process_re(RE* re);
    std::string bit_var(int n);
    PabloE* make_bitv(int n);
    PabloE* bit_pattern_expr(int pattern, int selected_bits);
    PabloE* char_test_expr(int ch);
    PabloE* make_range(int n1, int n2);
    PabloE* GE_Range(int N, int n);
    PabloE* LE_Range(int N, int n);
    PabloE* char_or_range_expr(CharSetItem charset_item);
    PabloE* charset_expr(CC* cc);
    Expression* expr2pabloe(PabloE* expr);
    void cc2pablos(CC* cc);

    UTF_Encoding mEncoding;


    void add_predefined(std::string key_value, Expression *mapped_value);
    Expression* add_assignment(std::string value, Expression* expr);
    Expression* expr_to_variable(Expression* cgo);

    std::string mGenSym_Template;
    int mGenSymCounter;
    std::list<PabloS*> mStmtsl;
    std::map<std::string, Expression*> mCommon_Expression_Map;
};

#endif // CC_COMPILER_H


