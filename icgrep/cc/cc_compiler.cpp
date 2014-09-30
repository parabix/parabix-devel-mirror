/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "cc_compiler.h"
#include "ps_pablos.h"
#include "utf_encoding.h"
#include <pablo/pablo_routines.h>

//Pablo Expressions
#include "pe_advance.h"
#include "pe_all.h"
#include "pe_and.h"
#include "pe_call.h"
#include "pe_charclass.h"
#include "pe_matchstar.h"
#include "pe_not.h"
#include "pe_or.h"
#include "pe_pabloe.h"
#include "pe_scanthru.h"
#include "pe_sel.h"
#include "pe_var.h"
#include "pe_xor.h"

//Pablo Statements
#include "ps_pablos.h"
#include "ps_assign.h"
#include "ps_if.h"
#include "ps_while.h"

#include "re/re_alt.h"
#include "re/re_cc.h"
#include "re/re_seq.h"
#include "re/re_rep.h"

#include <utility>
#include <string>
#include <list>
#include <map>
#include <algorithm>

#include <cassert>
#include <stdlib.h>
#include <stdexcept>

using namespace re;
using namespace pablo;

namespace cc {

CC_Compiler::CC_Compiler(const UTF_Encoding encoding, const std::string basis_pattern, const std::string gensym_pattern)
{
    mEncoding = encoding;
    mEncoding.setBasisPattern(basis_pattern);
    mGenSym_Template = gensym_pattern;
    mGenSymCounter = 0;

 
    for (int i = 0; i < mEncoding.getBits(); i++)
    {
        std::string b_pattern = bit_var((mEncoding.getBits() - 1) - i);
        Expression* expr = new Expression();
        expr->expr_string  =  b_pattern;
        expr->pablo_expr = make_bitv(i);
        add_predefined(b_pattern, expr);
    }
}


void CC_Compiler::add_predefined(std::string key_value, Expression* mapped_value)
{
    mCommon_Expression_Map.insert(make_pair(key_value, mapped_value));
}

Expression* CC_Compiler::add_assignment(std::string varname, Expression* expr)
{    
    //Add the new mapping to the list of pablo statements:
    mStmtsl.push_back(new Assign(varname, expr->pablo_expr));

    //Add the new mapping to the common expression map:
    std::string key_value = expr->expr_string;
    Expression* mapped_value = new Expression();
    mapped_value->expr_string = varname;
    mapped_value->pablo_expr = new Var(varname);

    std::pair<MapIterator, bool> ret = mCommon_Expression_Map.insert(make_pair(key_value, mapped_value));

    return ret.first->second;
}

Expression* CC_Compiler::expr_to_variable(Expression * expr) {
    MapIterator itr = mCommon_Expression_Map.find(expr->expr_string);
    if (itr != mCommon_Expression_Map.end()) {
        return itr->second;
    }
    else {
        return add_assignment(mGenSym_Template + std::to_string(++mGenSymCounter), expr);
    }
}

std::list<PabloS*> CC_Compiler::get_compiled()
{
    return mStmtsl;
}



std::string CC_Compiler::compile1(CC* cc)
{
  cc2pablos(cc);
  return cc->getName();
}

void CC_Compiler::compile_from_map(const REMap &re_map)
{
    process_re_map(re_map);
}

void CC_Compiler::process_re_map(const REMap & re_map) {
    for (auto it =  re_map.crbegin(); it != re_map.crend(); ++it) {
        process_re(it->second);
    }
}

void CC_Compiler::process_re(const RE* re) {
    if (const Alt* re_alt = dyn_cast<const Alt>(re)) {
        for (const RE * re : *re_alt) {
            process_re(re);
        }
    }
    else if (const CC* re_cc = dyn_cast<const CC>(re)) {
        cc2pablos(re_cc);
    }
    else if (const Rep* re_rep = dyn_cast<const Rep>(re)) {
        process_re(re_rep->getRE());
    }
    else if (const Seq* re_seq = dyn_cast<const Seq>(re)) {
        for (const RE * re : *re_seq) {
            process_re(re);
        }
    }
}

PabloE* CC_Compiler::bit_pattern_expr(int pattern, int selected_bits)
{
    if (selected_bits == 0) return new All(1);

    std::vector<PabloE*> bit_terms;
    int bit_no = 0;

    while (selected_bits)
    {
        char test_bit = 1 << bit_no;
        if (selected_bits & test_bit)
        {
            if ((pattern & test_bit) == 0)
            {
                bit_terms.push_back(make_not(make_bitv(bit_no)));
            }
            else
            {
                bit_terms.push_back(make_bitv(bit_no));
            }
        }
        else
        {
            bit_terms.push_back(new All(1));
        }
        selected_bits &= ~test_bit;
        bit_no++;
    }

    //Reduce the list so that all of the expressions are contained within a single expression.
    while (bit_terms.size() > 1)
    {
        std::vector<PabloE*> new_terms;
        for (unsigned long i = 0; i < (bit_terms.size()/2); i++)
        {
            new_terms.push_back(make_and(bit_terms[(2 * i) + 1], bit_terms[2 * i]));
        }
        if (bit_terms.size() % 2 == 1)
        {
            new_terms.push_back(bit_terms[bit_terms.size() -1]);
        }
        std::vector<PabloE*>::iterator it;
        bit_terms.assign(new_terms.begin(), new_terms.end());
    }
    return bit_terms[0];
}

PabloE* CC_Compiler::char_test_expr(const CodePointType ch)
{
    return bit_pattern_expr(ch, mEncoding.getMask());
}

PabloE* CC_Compiler::make_range(const CodePointType n1, const CodePointType n2)
{
    CodePointType diff_count = 0;

    for (CodePointType diff_bits = n1 ^ n2; diff_bits; diff_count++, diff_bits >>= 1);

    if ((n2 < n1) || (diff_count > mEncoding.getBits()))
    {
        throw std::runtime_error(std::string("Bad Range: [") + std::to_string(n1) + "," + std::to_string(n2) + "]");
    }

    const CodePointType mask0 = (static_cast<CodePointType>(1) << diff_count) - 1;

    PabloE * common = bit_pattern_expr(n1 & ~mask0, mEncoding.getMask() ^ mask0);

    if (diff_count == 0) return common;

    const CodePointType mask1 = (static_cast<CodePointType>(1) << (diff_count - 1)) - 1;

    PabloE* lo_test = GE_Range(diff_count - 1, n1 & mask1);
    PabloE* hi_test = LE_Range(diff_count - 1, n2 & mask1);

    return make_and(common, make_sel(make_bitv(diff_count - 1), hi_test, lo_test));
}

PabloE* CC_Compiler::GE_Range(int N, int n)
{
    if (N == 0)
    {
        return new All(1); //Return a true literal.
    }
    else if (((N % 2) == 0) && ((n >> (N - 2)) == 0))
    {
        return make_or(make_or(make_bitv(N-1), make_bitv(N-2)), GE_Range(N-2, n));
    }
    else if (((N % 2) == 0) && ((n >> (N - 2)) == 3))
    {
        return make_and(make_and(make_bitv(N-1), make_bitv(N-2)), GE_Range(N-2, n-(3<<(N-2))));
    }
    else if(N >= 1)
    {
        int hi_bit = n & (1 << (N-1));
        int lo_bits = n - hi_bit;
        PabloE* lo_range = GE_Range(N-1, lo_bits);
        if (hi_bit == 0)
        {
            /*
              If the hi_bit of n is not set, then whenever the corresponding bit
              is set in the target, the target will certaily be >=.  Oterwise,
              the value of GE_range(N-1), lo_range) is required.
            */
            return make_or(make_bitv(N-1), lo_range);
        }
        else
        {
            /*
              If the hi_bit of n is set, then the corresponding bit must be set
              in the target for >= and GE_range(N-1, lo_bits) must also be true.
            */
            return make_and(make_bitv(N-1), lo_range);
        }
    }
    else
    {
        return 0;
    }
}

PabloE* CC_Compiler::LE_Range(int N, int n)
{
    /*
      If an N-bit pattern is all ones, then it is always true that any n-bit value is LE this pattern.
      Handling this as a special case avoids an overflow issue with n+1 requiring more than N bits.
    */
    if ((n + 1) == (1 << N)) {
        return new All(1); //True.
    }
    else {
        return make_not(GE_Range(N, n + 1));
    }
}

PabloE* CC_Compiler::charset_expr(const CC * cc) {
    if (cc->empty()) {
        return new All(0);
    }
    if (cc->size() > 2) {
        bool combine = true;
        for (const CharSetItem & item : *cc) {
            if (item.lo_codepoint != item.hi_codepoint) {
                combine = false;
                break;
            }
        }
        if (combine) {
            auto i = cc->cbegin();
            for (auto j = i; ++j != cc->cend(); i = j) {
                const CharSetItem & curr_item = *i;
                const CharSetItem & next_item = *j;
                if ((curr_item.lo_codepoint + 2) != next_item.lo_codepoint) {
                    combine  = false;
                    break;
                }
            }
            if (combine) {
                CodePointType lo = cc->front().lo_codepoint;
                CodePointType hi = cc->back().lo_codepoint;
                const CodePointType mask = mEncoding.getMask();
                lo &= (mask - 1);
                hi |= (mask ^ (mask - 1));
                PabloE * expr = make_range(lo, hi);
                PabloE * bit0 = make_bitv(0);
                if ((lo & 1) == 0) {
                    bit0 = make_not(bit0);
                }
                return make_and(expr, bit0);
            }
        }
    }
    PabloE * expr = nullptr;
    for (const CharSetItem & item : *cc) {
        PabloE * temp = char_or_range_expr(item.lo_codepoint, item.hi_codepoint);
        expr = (expr == nullptr) ? temp : make_or(expr, temp);
    }
    return expr;
}

inline PabloE * CC_Compiler::char_or_range_expr(const CodePointType lo, const CodePointType hi) {
    if (lo == hi) {
        return char_test_expr(lo);
    }
    else if (lo < hi) {
        return make_range(lo, hi);
    }
    throw std::runtime_error(std::string("Invalid Character Set Range: [") + std::to_string(lo) + "," + std::to_string(hi) + "]");
}


Expression* CC_Compiler::expr2pabloe(PabloE* expr) {
    /*
      Translate a Pablo Expression into three-address code using
      the code generator object CC_CodeGenObject.
    */

    Expression* retExpr = new Expression();

    if (All* all = dynamic_cast<All*>(expr))
    {
        if (all->getNum() == 1)
        {
            retExpr->expr_string = "All(1)";
            retExpr->pablo_expr = new All(1);
        }
        else if (all->getNum() == 0)
        {
            retExpr->expr_string = "All(0)";
            retExpr->pablo_expr = new All(0);
        }
    }
    else if (Var * var = dynamic_cast<Var*>(expr))
    {
            retExpr->expr_string = var->getVar();
            retExpr->pablo_expr = new Var(var->getVar());
    }
    else if (Not* pe_not = dynamic_cast<Not*>(expr))
    {
        Expression* ret = expr_to_variable(expr2pabloe(pe_not->getExpr()));
        retExpr->expr_string =  "~" + ret->expr_string;
        retExpr->pablo_expr = new Not(ret->pablo_expr);
    }
    else if(Or* pe_or = dynamic_cast<Or*>(expr))
    {
        Expression* ret1 = expr_to_variable(expr2pabloe(pe_or->getExpr1()));
        Expression* ret2 = expr_to_variable(expr2pabloe(pe_or->getExpr2()));
        retExpr->expr_string = "(" + ret1->expr_string + "|" + ret2->expr_string + ")";
        retExpr->pablo_expr = new Or(ret1->pablo_expr, ret2->pablo_expr);
    }
    else if (Xor* pe_xor = dynamic_cast<Xor*>(expr))
    {
        Expression* ret1 = expr_to_variable(expr2pabloe(pe_xor->getExpr1()));
        Expression* ret2 = expr_to_variable(expr2pabloe(pe_xor->getExpr2()));
        retExpr->expr_string = "(" + ret1->expr_string + "^" + ret2->expr_string + ")";
        retExpr->pablo_expr = new Xor(ret1->pablo_expr, ret2->pablo_expr);
    }
    else if (And* pe_and = dynamic_cast<And*>(expr))
    {
        if (Not* pe_not = dynamic_cast<Not*>(pe_and->getExpr1()))
        {
            Expression* ret1 = expr_to_variable(expr2pabloe(pe_not->getExpr()));
            Expression* ret2 = expr_to_variable(expr2pabloe(pe_and->getExpr2()));
            retExpr->expr_string = "(" + ret2->expr_string + "&~" + ret1->expr_string + ")";
            retExpr->pablo_expr = new And(ret2->pablo_expr, new Not(ret1->pablo_expr));
        }
        else if (Not* pe_not = dynamic_cast<Not*>(pe_and->getExpr2()))
        {
            Expression* ret1 = expr_to_variable(expr2pabloe(pe_and->getExpr1()));
            Expression* ret2 = expr_to_variable(expr2pabloe(pe_not->getExpr()));
            retExpr->expr_string = "(" + ret1->expr_string  + "&~" + ret2->expr_string + ")";
            retExpr->pablo_expr = new And(ret1->pablo_expr, new Not(ret2->pablo_expr));
        }
        else
        {
            Expression* ret1 = expr_to_variable(expr2pabloe(pe_and->getExpr1()));
            Expression* ret2 = expr_to_variable(expr2pabloe(pe_and->getExpr2()));
            retExpr->expr_string = "(" + ret1->expr_string + "&" + ret2->expr_string + ")";
            retExpr->pablo_expr = new And(ret1->pablo_expr, ret2->pablo_expr);
        }
    }
    else if (Sel * pe_sel = dynamic_cast<Sel*>(expr))
    {
        Expression* ret_sel = expr_to_variable(expr2pabloe(pe_sel->getIf_expr()));
        Expression* ret_true = expr_to_variable(expr2pabloe(pe_sel->getT_expr()));
        Expression* ret_false = expr_to_variable(expr2pabloe(pe_sel->getF_expr()));
        retExpr->expr_string = "((" + ret_sel->expr_string + "&" + ret_true->expr_string + ")|(~("
            + ret_sel->expr_string + ")&" + ret_false->expr_string + ")";
        retExpr->pablo_expr = new Sel(ret_sel->pablo_expr, ret_true->pablo_expr, ret_false->pablo_expr);
    }

    return retExpr;
}

void CC_Compiler::cc2pablos(const CC * cc)
{
    add_assignment(cc->getName(), expr2pabloe(charset_expr(cc)));
}

std::string CC_Compiler::bit_var(int n)
{
    return  mEncoding.getBasisPattern(0) + std::to_string(n);
}

PabloE* CC_Compiler::make_bitv(int n)
{
    return new Var(bit_var((mEncoding.getBits() - 1) - n));
}

} // end of namespace cc
