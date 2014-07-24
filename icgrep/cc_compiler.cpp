/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "cc_compiler.h"

CC_Compiler::CC_Compiler(UTF_Encoding encoding)
{
    mEncoding = encoding;
}

std::list<PabloS*> CC_Compiler::compile(std::string basis_pattern,
                                        std::string gensym_pattern,
                                        const std::map<std::string, RE*>& re_map,
                                        std::list<CC*> predefined)
{
    mEncoding.setBasisPattern(basis_pattern);

    CC_CodeGenObject cgo(gensym_pattern);

    for (int i = 0; i < mEncoding.getBits(); i++)
    {
        std::string b_pattern = bit_var((mEncoding.getBits() -1) - i);
        Expression* expr = new Expression();
        expr->expr_string  =  b_pattern;
        expr->pablo_expr = make_bitv(i);
        cgo.add_predefined(b_pattern, expr);
    }

    process_re_map(cgo, re_map);
    process_predefined(cgo, predefined);

    return cgo.get_stmtsl();
}

void CC_Compiler::process_re_map(CC_CodeGenObject &cgo,const std::map<std::string, RE*>& re_map)
{
    for (auto it =  re_map.rbegin(); it != re_map.rend(); ++it)
    {
        process_re(cgo, it->second);
    }
}

void CC_Compiler::process_re(CC_CodeGenObject &cgo, RE* re)
{

    if (Alt* re_alt = dynamic_cast<Alt*>(re))
    {
        std::list<RE*>::iterator it;
        for (it = re_alt->GetREList()->begin(); it != re_alt->GetREList()->end(); ++it)
        {
            process_re(cgo, *it);
        }
    }
    else if (CC* re_cc = dynamic_cast<CC*>(re))
    {
        cgo = cc2pablos(cgo, re_cc);
    }
    else if (Rep* re_rep = dynamic_cast<Rep*>(re))
    {
        process_re(cgo, re_rep->getRE());
    }
    else if (Seq* re_seq = dynamic_cast<Seq*>(re))
    {
        std::list<RE*>::iterator it;
        for (it = re_seq->GetREList()->begin(); it != re_seq->GetREList()->end(); ++it)
        {
            process_re(cgo, *it);
        }
    }
}

void CC_Compiler::process_predefined(CC_CodeGenObject &cgo, std::list<CC*> predefined)
{
    std::list<CC*>::iterator it;
    for (it = predefined.begin(); it != predefined.end(); ++it)
    {
        cgo = cc2pablos(cgo, *it);
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
                bit_terms.push_back(CC_Compiler_Helper::make_not(make_bitv(bit_no)));
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
/*
    std::cout << "FIRST LOOP:" << std::endl;
    for (int i = bit_terms.size() - 1; i >= 0; i--)
    {
        std::cout << StatementPrinter::ShowPabloE(bit_terms.at(i)) << std::endl;
    }
*/
    //Reduce the list so that all of the expressions are contained within a single expression.
    while (bit_terms.size() > 1)
    {
        std::vector<PabloE*> new_terms;
        for (unsigned long i = 0; i < (bit_terms.size()/2); i++)
        {
            new_terms.push_back(CC_Compiler_Helper::make_and(bit_terms[(2 * i) + 1], bit_terms[2 * i]));
        }
        if (bit_terms.size() % 2 == 1)
        {
            new_terms.push_back(bit_terms[bit_terms.size() -1]);
        }
/*
        std::cout << "\nNEW TERMS ITERATION:\n" << std::endl;
        for (int i = new_terms.size() - 1; i >=0; i--)
        {
            std::cout <<  StatementPrinter::ShowPabloE(new_terms[i]) << std::endl;
        }
        std::cout << "\n" << std::endl;
*/
        std::vector<PabloE*>::iterator it;
        bit_terms.assign(new_terms.begin(), new_terms.end());
    }
/*
    std::cout << "bit_terms.size(): " << bit_terms.size() << std::endl;
    std::cout << StatementPrinter::ShowPabloE(bit_terms[0]) << std::endl;
*/
    return bit_terms[0];
}

PabloE* CC_Compiler::char_test_expr(int ch)
{
    return bit_pattern_expr(ch, mEncoding.getMask());
}

PabloE* CC_Compiler::make_range(int n1, int n2)
{
    unsigned char diff_bits = n1 ^ n2;
    int diff_count = 0;

    while (diff_bits > 0)
    {
        diff_count++;
        diff_bits >>= 1;
    }

    if ((n2 < n1) || (diff_count > mEncoding.getBits()))
    {
        int n1i = n1;
        int n2i = n2;

        std::cout << "n1: " << n1i << std::endl;
        std::cout << "n2: " << n2i << std::endl;

        std::cout << "Exception: Bad Range!" << std::endl;
        return 0;
    }

    int mask = pow(2, diff_count) - 1;

    PabloE* common = bit_pattern_expr(n1 & ~mask, mEncoding.getMask() ^ mask);
    if (diff_count == 0) return common;

    mask = pow(2, (diff_count - 1)) - 1;

    PabloE* lo_test = GE_Range(diff_count - 1, n1 & mask);
    PabloE* hi_test = LE_Range(diff_count - 1, n2 & mask);

    return CC_Compiler_Helper::make_and(common, CC_Compiler_Helper::make_sel(make_bitv(diff_count - 1), hi_test, lo_test));
}

PabloE* CC_Compiler::GE_Range(int N, int n)
{
    if (N == 0)
    {
        return new All(1); //Return a true literal.
    }
    else if (((N % 2) == 0) && ((n >> (N - 2)) == 0))
    {
        return CC_Compiler_Helper::make_or(CC_Compiler_Helper::make_or(make_bitv(N-1), make_bitv(N-2)), GE_Range(N-2, n));
    }
    else if (((N % 2) == 0) && ((n >> (N - 2)) == 3))
    {
        return CC_Compiler_Helper::make_and(CC_Compiler_Helper::make_and(make_bitv(N-1), make_bitv(N-2)), GE_Range(N-2, n-(3<<(N-2))));
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
            return CC_Compiler_Helper::make_or(make_bitv(N-1), lo_range);
        }
        else
        {
            /*
              If the hi_bit of n is set, then the corresponding bit must be set
              in the target for >= and GE_range(N-1, lo_bits) must also be true.
            */
            return CC_Compiler_Helper::make_and(make_bitv(N-1), lo_range);
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
    if ((n+1) == pow(2, N))
    {
        return new All(1); //True.
    }
    else
    {
        return CC_Compiler_Helper::make_not(GE_Range(N, n+1));
    }
}

PabloE* CC_Compiler::char_or_range_expr(CharSetItem charset_item)
{
    if (charset_item.lo_codepoint == charset_item.hi_codepoint)
    {
        return char_test_expr(charset_item.lo_codepoint);
    }
    else
    {
        if (charset_item.lo_codepoint < charset_item.hi_codepoint)
        {
            return make_range(charset_item.lo_codepoint, charset_item.hi_codepoint);
        }
    }

    std::cout << "Exception: Bad Character Set Item!" << std::endl;
    return 0;
}

PabloE* CC_Compiler::charset_expr(CC* cc)
{
    if (cc->getItems().size() == 0)
    {
        return new All(0);
    }

    if (cc->getItems().size() > 1)
    {
        bool combine = true;

        for (unsigned long i = 0; i < cc->getItems().size(); i++)
        {
            CharSetItem item = cc->getItems().at(i);
            if (item.lo_codepoint != item.hi_codepoint)
            {
                combine = false;
                break;
            }
        }

        if (combine)
        {
            for (unsigned long i = 0; i < cc->getItems().size() - 1; i ++)
            {
                CharSetItem curr_item = cc->getItems().at(i);
                CharSetItem next_item = cc->getItems().at(i + 1);
                if (curr_item.lo_codepoint != next_item.lo_codepoint + 2)
                {
                    combine  = false;
                    break;
                }
            }
        }

        if (combine)
        {
            CharSetItem first_item = cc->getItems().at(0);
            CharSetItem last_item = cc->getItems().at(cc->getItems().size() - 1);
            CharSetItem combined_item;
            combined_item.lo_codepoint = (last_item.lo_codepoint & 0xFE);
            combined_item.hi_codepoint = (first_item.hi_codepoint | 0x01);
            std::cout << "Combined!" << std::endl;
            return char_or_range_expr(combined_item);
        }
    }

    PabloE* e1 = char_or_range_expr(cc->getItems().at(0));
    if (cc->getItems().size() > 1)
    {
        for (unsigned long i = 1; i < cc->getItems().size(); i++)
        {
            e1 = CC_Compiler_Helper::make_or(e1, char_or_range_expr(cc->getItems().at(i)));
        }
    }

    return e1;
}

Expression* CC_Compiler::expr2pabloe(CC_CodeGenObject &cgo, PabloE* expr)
{
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
        else if (all->getNum() ==0)
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
        Expression* ret = cgo.expr_to_variable(expr2pabloe(cgo, pe_not->getExpr()));
        retExpr->expr_string =  "~" + ret->expr_string;
        retExpr->pablo_expr = new Not(ret->pablo_expr);
    }
    else if(Or* pe_or = dynamic_cast<Or*>(expr))
    {
        Expression* ret1 = cgo.expr_to_variable(expr2pabloe(cgo, pe_or->getExpr1()));
        Expression* ret2 = cgo.expr_to_variable(expr2pabloe(cgo, pe_or->getExpr2()));
        retExpr->expr_string = "(" + ret1->expr_string + "|" + ret2->expr_string + ")";
        retExpr->pablo_expr = new Or(ret1->pablo_expr, ret2->pablo_expr);
    }
    else if (Xor* pe_xor = dynamic_cast<Xor*>(expr))
    {
        Expression* ret1 = cgo.expr_to_variable(expr2pabloe(cgo, pe_xor->getExpr1()));
        Expression* ret2 = cgo.expr_to_variable(expr2pabloe(cgo, pe_xor->getExpr2()));
        retExpr->expr_string = "(" + ret1->expr_string + "^" + ret2->expr_string + ")";
        retExpr->pablo_expr = new Xor(ret1->pablo_expr, ret2->pablo_expr);
    }
    else if (And* pe_and = dynamic_cast<And*>(expr))
    {
        if (Not* pe_not = dynamic_cast<Not*>(pe_and->getExpr1()))
        {
            Expression* ret1 = cgo.expr_to_variable(expr2pabloe(cgo, pe_not->getExpr()));
            Expression* ret2 = cgo.expr_to_variable(expr2pabloe(cgo, pe_and->getExpr2()));
            retExpr->expr_string = "(" + ret2->expr_string + "&~" + ret1->expr_string + ")";
            retExpr->pablo_expr = new And(ret2->pablo_expr, new Not(ret1->pablo_expr));
        }
        else if (Not* pe_not = dynamic_cast<Not*>(pe_and->getExpr2()))
        {
            Expression* ret1 = cgo.expr_to_variable(expr2pabloe(cgo, pe_and->getExpr1()));
            Expression* ret2 = cgo.expr_to_variable(expr2pabloe(cgo, pe_not->getExpr()));
            retExpr->expr_string = "(" + ret1->expr_string  + "&~" + ret2->expr_string + ")";
            retExpr->pablo_expr = new And(ret1->pablo_expr, new Not(ret2->pablo_expr));
        }
        else
        {
            Expression* ret1 = cgo.expr_to_variable(expr2pabloe(cgo, pe_and->getExpr1()));
            Expression* ret2 = cgo.expr_to_variable(expr2pabloe(cgo, pe_and->getExpr2()));
            retExpr->expr_string = "(" + ret1->expr_string + "&" + ret2->expr_string + ")";
            retExpr->pablo_expr = new And(ret1->pablo_expr, ret2->pablo_expr);
        }
    }
    else if (Sel * pe_sel = dynamic_cast<Sel*>(expr))
    {
        Expression* ret_sel = cgo.expr_to_variable(expr2pabloe(cgo, pe_sel->getIf_expr()));
        Expression* ret_true = cgo.expr_to_variable(expr2pabloe(cgo, pe_sel->getT_expr()));
        Expression* ret_false = cgo.expr_to_variable(expr2pabloe(cgo, pe_sel->getF_expr()));
        retExpr->expr_string = "((" + ret_sel->expr_string + "&" + ret_true->expr_string + ")|(~("
            + ret_sel->expr_string + ")&" + ret_false->expr_string + ")";
        retExpr->pablo_expr = new Sel(ret_sel->pablo_expr, ret_true->pablo_expr, ret_false->pablo_expr);
    }

    return retExpr;
}

CC_CodeGenObject CC_Compiler::cc2pablos(CC_CodeGenObject cgo, CC* cc)
{
    cgo.add_assignment(cc->getName(), expr2pabloe(cgo, charset_expr(cc)));

    return cgo;
}

std::string CC_Compiler::bit_var(int n)
{
    return  mEncoding.getBasisPattern(0) + INT2STRING(n);
}

PabloE* CC_Compiler::make_bitv(int n)
{
    return new Var(bit_var((mEncoding.getBits() - 1) - n));
}
