/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "cc_compiler.h"
#include "utf_encoding.h"

//Pablo Expressions
#include <pablo/codegenstate.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_name.h>

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

CC_Compiler::CC_Compiler(CodeGenState & cg, const Encoding encoding, const std::string basis_pattern, const std::string gensym_pattern)
: mCG(cg)
, mEncoding(encoding)
, mGenSymPattern(gensym_pattern)
, mBasisPattern(basis_pattern)
{
    for (int i = 0; i < mEncoding.getBits(); i++)
    {
        Var * basisVar = getBasisVar(i);
        Expression* expr = new Expression();
        expr->expr_string  =  basisVar->getName();
        expr->pablo_expr = basisVar;
        mCommon_Expression_Map.insert(make_pair(basisVar->getName(), expr));
    }
}

void CC_Compiler::compile(const REMap & re_map) {
    for (auto i =  re_map.cbegin(); i != re_map.cend(); ++i) {
        process_re(i->second);
    }
    for (auto i =  re_map.cbegin(); i != re_map.cend(); ++i) {
        //This is specifically for the utf8 multibyte character classes.
        if (Seq * seq = dyn_cast<Seq>(i->second)) {
            if (seq->getType() == Seq::Type::Byte) {
                PabloE * marker = nullptr;
                auto j = seq->begin();
                while (true) {
                    Name * name = dyn_cast<Name>(*j);
                    assert (name);
                    CharClass * cc = makeCharClass(name->getName());
                    PabloE * sym = marker ? makeAnd(marker, cc) : cc;
                    if (++j != seq->end()) {
                        marker = makeAdvance(sym);
                        mCG.push_back(marker);
                        continue;
                    }
                    mCG.push_back(makeAssign(seq->getName(), sym));
                    break;
                }
            }
        }
    }
}

Expression * CC_Compiler::add_assignment(std::string varname, Expression* expr)
{    
    //Add the new mapping to the list of pablo statements:
    mCG.push_back(makeAssign(varname, expr->pablo_expr));

    //Add the new mapping to the common expression map:
    std::string key_value = expr->expr_string;
    Expression* mapped_value = new Expression();
    mapped_value->expr_string = varname;
    mapped_value->pablo_expr = makeVar(varname);
    return mCommon_Expression_Map.insert(std::make_pair(key_value, mapped_value)).first->second;
}

Expression * CC_Compiler::expr_to_variable(Expression * expr) {
    MapIterator itr = mCommon_Expression_Map.find(expr->expr_string);
    if (itr != mCommon_Expression_Map.end()) {
        return itr->second;
    }
    else {
        return add_assignment(mCG.symgen(mGenSymPattern), expr);
    }
}

void CC_Compiler::process_re(const RE * re) {
    if (const Alt * alt = dyn_cast<const Alt>(re)) {
        for (const RE * re : *alt) {
            process_re(re);
        }
    }
    else if (const CC * cc = dyn_cast<const CC>(re)) {
        process(cc);
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

void CC_Compiler::process(const CC * cc)
{
    add_assignment(cc->getName(), expr2pabloe(charset_expr(cc)));
}


PabloE* CC_Compiler::bit_pattern_expr(int pattern, int selected_bits)
{
    if (selected_bits == 0) {
        return mCG.createAll(1);
    }

    std::vector<PabloE*> bit_terms;
    int bit_no = 0;

    while (selected_bits)
    {
        char test_bit = 1 << bit_no;
        if (selected_bits & test_bit)
        {
            if ((pattern & test_bit) == 0)
            {
                bit_terms.push_back(makeNot(getBasisVar(bit_no)));
            }
            else
            {
                bit_terms.push_back(getBasisVar(bit_no));
            }
        }
        else
        {
            bit_terms.push_back(mCG.createAll(1));
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
            new_terms.push_back(makeAnd(bit_terms[(2 * i) + 1], bit_terms[2 * i]));
        }
        if (bit_terms.size() % 2 == 1)
        {
            new_terms.push_back(bit_terms[bit_terms.size() -1]);
        }
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

    return makeAnd(common, makeSel(getBasisVar(diff_count - 1), hi_test, lo_test));
}

PabloE * CC_Compiler::GE_Range(int N, int n) {
    if (N == 0)
    {
        return mCG.createAll(1); //Return a true literal.
    }
    else if (((N % 2) == 0) && ((n >> (N - 2)) == 0))
    {
        return makeOr(makeOr(getBasisVar(N - 1), getBasisVar(N - 2)), GE_Range(N - 2, n));
    }
    else if (((N % 2) == 0) && ((n >> (N - 2)) == 3))
    {
        return makeAnd(makeAnd(getBasisVar(N - 1), getBasisVar(N - 2)), GE_Range(N - 2, n - (3 << (N - 2))));
    }
    else if (N >= 1)
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
            return makeOr(getBasisVar(N - 1), lo_range);
        }
        else
        {
            /*
              If the hi_bit of n is set, then the corresponding bit must be set
              in the target for >= and GE_range(N-1, lo_bits) must also be true.
            */
            return makeAnd(getBasisVar(N - 1), lo_range);
        }
    }
    throw std::runtime_error("Unexpected input given to ge_range: " + std::to_string(N) + ", " + std::to_string(n));
}

PabloE * CC_Compiler::LE_Range(int N, int n)
{
    /*
      If an N-bit pattern is all ones, then it is always true that any n-bit value is LE this pattern.
      Handling this as a special case avoids an overflow issue with n+1 requiring more than N bits.
    */
    if ((n + 1) == (1 << N)) {
        return mCG.createAll(1); //True.
    }
    else {
        return makeNot(GE_Range(N, n + 1));
    }
}

PabloE* CC_Compiler::charset_expr(const CC * cc) {
    if (cc->empty()) {
        return mCG.createAll(0);
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
                PabloE * bit0 = getBasisVar(0);
                if ((lo & 1) == 0) {
                    bit0 = makeNot(bit0);
                }
                return makeAnd(expr, bit0);
            }
        }
    }
    PabloE * expr = nullptr;
    for (const CharSetItem & item : *cc) {
        PabloE * temp = char_or_range_expr(item.lo_codepoint, item.hi_codepoint);
        expr = (expr == nullptr) ? temp : makeOr(expr, temp);
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

    if (All * all = dyn_cast<All>(expr))
    {
        if (all->getValue() == 1)
        {
            retExpr->expr_string = "All(1)";
        }
        else if (all->getValue() == 0)
        {
            retExpr->expr_string = "All(0)";
        }
    }
    else if (Var * var = dyn_cast<Var>(expr))
    {
            retExpr->expr_string = var->getName();
    }
    else if (Not * pe_not = dyn_cast<Not>(expr))
    {
        Expression* ret = expr_to_variable(expr2pabloe(pe_not->getExpr()));
        retExpr->expr_string =  "~" + ret->expr_string;
    }
    else if(Or * pe_or = dyn_cast<Or>(expr))
    {
        Expression* ret1 = expr_to_variable(expr2pabloe(pe_or->getExpr1()));
        Expression* ret2 = expr_to_variable(expr2pabloe(pe_or->getExpr2()));
        retExpr->expr_string = "(" + ret1->expr_string + "|" + ret2->expr_string + ")";
    }
    else if (Xor * pe_xor = dyn_cast<Xor>(expr))
    {
        Expression* ret1 = expr_to_variable(expr2pabloe(pe_xor->getExpr1()));
        Expression* ret2 = expr_to_variable(expr2pabloe(pe_xor->getExpr2()));
        retExpr->expr_string = "(" + ret1->expr_string + "^" + ret2->expr_string + ")";
    }
    else if (And * pe_and = dyn_cast<And>(expr))
    {
        if (Not * pe_not = dyn_cast<Not>(pe_and->getExpr1()))
        {
            Expression* ret1 = expr_to_variable(expr2pabloe(pe_not->getExpr()));
            Expression* ret2 = expr_to_variable(expr2pabloe(pe_and->getExpr2()));
            retExpr->expr_string = "(" + ret2->expr_string + "&~" + ret1->expr_string + ")";
        }
        else if (Not * pe_not = dyn_cast<Not>(pe_and->getExpr2()))
        {
            Expression* ret1 = expr_to_variable(expr2pabloe(pe_and->getExpr1()));
            Expression* ret2 = expr_to_variable(expr2pabloe(pe_not->getExpr()));
            retExpr->expr_string = "(" + ret1->expr_string  + "&~" + ret2->expr_string + ")";
        }
        else
        {
            Expression* ret1 = expr_to_variable(expr2pabloe(pe_and->getExpr1()));
            Expression* ret2 = expr_to_variable(expr2pabloe(pe_and->getExpr2()));
            retExpr->expr_string = "(" + ret1->expr_string + "&" + ret2->expr_string + ")";
        }
    }
    else if (Sel * pe_sel = dyn_cast<Sel>(expr))
    {
        Expression* ret_sel = expr_to_variable(expr2pabloe(pe_sel->getIf_expr()));
        Expression* ret_true = expr_to_variable(expr2pabloe(pe_sel->getT_expr()));
        Expression* ret_false = expr_to_variable(expr2pabloe(pe_sel->getF_expr()));
        retExpr->expr_string = "((" + ret_sel->expr_string + "&" + ret_true->expr_string + ")|(~("
            + ret_sel->expr_string + ")&" + ret_false->expr_string + ")";        
    }
    retExpr->pablo_expr = expr;
    return retExpr;
}

inline Var * CC_Compiler::getBasisVar(const int i) const {
    return makeVar(mBasisPattern + std::to_string((mEncoding.getBits() - 1) - i));
}

} // end of namespace cc
