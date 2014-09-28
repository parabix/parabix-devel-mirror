/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "compiler_helper.h"

Compiler_Helper::Compiler_Helper(){}

/*
    Optimizing Constructors for Boolean Expressions

     -Maintaining Assembler Instruction Form:
       -All boolean algebraic rules involving true/flase applied.

       -Negations restricted:
         -no negations within or (DeMorgan's to nand)
         -at most one negation within and.
*/

PabloE* Compiler_Helper::make_not(PabloE* expr)
{
    if (All* all = dynamic_cast<All*>(expr))
    {
        if (all->getNum() == 1) //If true literal.
        {
            all->setNum(0);
            return all; //Set to false literal.
        }
        else if (all->getNum() == 0) //If false literal.
        {
            all->setNum(1);
            return all; //Set to true literal.
        }
    }
    else if (Not* pe_not = dynamic_cast<Not*>(expr)) {
        PabloE * expr = pe_not->getExpr();
        pe_not->setExpr(nullptr);
        delete pe_not;
        return expr;
    }
    return new Not(expr);
}

PabloE* Compiler_Helper::make_and(PabloE *expr1, PabloE *expr2)
{
    if (All* all = dynamic_cast<All*>(expr1))
    {
        if (all->getNum() == 1)
        {
            return expr2;
        }
        else if (all->getNum() == 0)
        {
            return expr1;
        }
    }
    else if (All* all = dynamic_cast<All*>(expr2))
    {
        if (all->getNum() == 1)
        {
            return expr1;
        }
        else if (all->getNum() == 0)
        {
            return expr2;
        }
    }
    else if (equal_exprs(expr1, expr2 ))
    {
        return expr1;
    }
    else if (Not* pe_not_e1 = dynamic_cast<Not*>(expr1))
    {
        if (Not* pe_not_e2 = dynamic_cast<Not*>(expr2))
        {
            return make_not(make_or(pe_not_e1->getExpr(), pe_not_e2->getExpr()));
        }
        else if (equal_exprs(pe_not_e1->getExpr(), expr2))
        {
            return new All(0); //Return false literal.
        }
        else
            return new And(expr1, expr2);
    }
    else if (Not* pe_not_e2 = dynamic_cast<Not*>(expr2))
    {
        if (equal_exprs(expr1, pe_not_e2->getExpr()))
        {
            return new All(0);
        }
        else
            return new And(expr1, expr2);
    }
    else
        return new And(expr1, expr2);
}

PabloE* Compiler_Helper::make_or(PabloE *expr1, PabloE *expr2)
{
    if (All* all = dynamic_cast<All*>(expr1))
    {
        if (all->getNum() == 1)
        {
            return expr1; //Return a true literal.
        }
        else if (all->getNum() == 0)
        {
            return expr2;
        }
    }
    else if (All* all = dynamic_cast<All*>(expr2))
    {
        if (all->getNum() == 1)
        {
            return expr2; //Return a true literal.
        }
        else if (all->getNum() == 0)
        {
            return expr1;
        }
    }
    else if (Not* pe_not_e1 = dynamic_cast<Not*>(expr1))
    {
        return make_not(make_and(pe_not_e1->getExpr(), make_not(expr2)));
    }
    else if (Not* pe_not_e2 = dynamic_cast<Not*>(expr2))
    {
        return make_not(make_and(make_not(expr1), pe_not_e2->getExpr()));
    }
    else if (equal_exprs(expr1, expr2))
    {
        return expr1;
    }

    if (And* and_expr1 = dynamic_cast<And*>(expr1))
    {
        if (And* and_expr2 = dynamic_cast<And*>(expr2))
        {
            //These optimizations factor out common components that can occur when sets are formed by union
            //(e.g., union of [a-z] and [A-Z].
            if (equal_exprs(and_expr1->getExpr1(), and_expr2->getExpr1()))
            {
                return make_and(and_expr1->getExpr1(), make_or(and_expr1->getExpr2(), and_expr2->getExpr2()));
            }
            else if (equal_exprs(and_expr1->getExpr2(), and_expr2->getExpr2()))
            {
                return make_and(and_expr1->getExpr2(), make_or(and_expr1->getExpr1(), and_expr2->getExpr1()));
            }
            else if (equal_exprs(and_expr1->getExpr1(), and_expr2->getExpr2()))
            {
                return make_and(and_expr1->getExpr1(), make_or(and_expr1->getExpr2(), and_expr2->getExpr1()));
            }
            else if (equal_exprs(and_expr1->getExpr2(), and_expr2->getExpr1()))
            {
                return make_and(and_expr1->getExpr2(), make_or(and_expr1->getExpr1(), and_expr2->getExpr2()));
            }
        }
    }

    return new Or(expr1, expr2);
}

PabloE* Compiler_Helper::make_sel(PabloE *if_expr, PabloE *t_expr, PabloE *f_expr)
{
    if (All* all_if_expr = dynamic_cast<All*>(if_expr))
    {
        if (all_if_expr->getNum() == 1)
        {
            return t_expr;
        }
        else if (all_if_expr->getNum() == 0)
        {
            return f_expr;
        }
    }
    else if (All* all_t_expr = dynamic_cast<All*>(t_expr))
    {
        if (all_t_expr->getNum() == 1)
        {
            return make_or(if_expr, f_expr);
        }
        else if (all_t_expr->getNum() == 0)
        {
            return make_and(make_not(if_expr), f_expr);
        }
    }
    else if (All* all_f_expr = dynamic_cast<All*>(f_expr))
    {
        if (all_f_expr->getNum() == 1)
        {
            return make_or(make_not(if_expr), t_expr);
        }
        else if (all_f_expr->getNum() == 0)
        {
            return make_and(if_expr, t_expr);
        }
    }
    else if (equal_exprs(t_expr, f_expr))
    {
        return t_expr;
    }
    else
        return new Sel(if_expr, t_expr, f_expr);
}

PabloE* Compiler_Helper::make_xor(PabloE *expr1, PabloE *expr2)
{
    if (All* all_expr1 = dynamic_cast<All*>(expr1))
    {
        if (all_expr1->getNum() == 1)
        {
            return make_not(expr2);
        }
        else if (all_expr1->getNum() == 0)
        {
            return expr2;
        }
    }
    else if (All* all_expr2 = dynamic_cast<All*>(expr2))
    {
        if (all_expr2->getNum() == 1)
        {
            return make_not(expr1);
        }
        else if (all_expr2->getNum() == 0)
        {
            return expr1;
        }
    }

    if (Not* not_expr1 = dynamic_cast<Not*>(expr1))
    {
        if (Not* not_expr2 = dynamic_cast<Not*>(expr2))
        {
            return make_xor(not_expr1->getExpr(), not_expr2->getExpr());
        }
    }

    return new Xor(expr1, expr2);
}

/*

    Return true if expr1 and expr2 can be proven equivalent according to some rules,
    false otherwise.  Note that false may be returned i some cases when the exprs are
    equivalent.

*/

bool Compiler_Helper::equal_exprs(PabloE *expr1, PabloE *expr2)
{
    if (All* all_expr1 = dynamic_cast<All*>(expr1))
    {
        if (all_expr1->getNum() == 1)
        {
            if (All* all_expr2 = dynamic_cast<All*>(expr2))
            {
                if (all_expr2->getNum() == 1)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }
        else if (all_expr1->getNum() == 0)
        {
            if (All* all_expr2 = dynamic_cast<All*>(expr2))
            {
                if (all_expr2->getNum() == 1)
                {
                    return false;
                }
                else
                {
                    return true;
                }
            }
            else
            {
                return false;
            }
        }
    }

    if (Var* var_expr1 = dynamic_cast<Var*>(expr1))
    {
        if (Var* var_expr2 = dynamic_cast<Var*>(expr2))
        {
            return (var_expr1->getVar() == var_expr2->getVar());
        }
    }

    if (Not* not_expr1 = dynamic_cast<Not*>(expr1))
    {
        if (Not* not_expr2 = dynamic_cast<Not*>(expr2))
        {
            return equal_exprs(not_expr1->getExpr(), not_expr2->getExpr());
        }
    }

    if (And* and_expr1 = dynamic_cast<And*>(expr1))
    {
        if (And* and_expr2 = dynamic_cast<And*>(expr2))
        {
            if (equal_exprs(and_expr1->getExpr1(), and_expr2->getExpr1()))
            {
                return equal_exprs(and_expr1->getExpr2(), and_expr2->getExpr2());
            }
            else if (equal_exprs(and_expr1->getExpr1(), and_expr2->getExpr2()))
            {
                return equal_exprs(and_expr1->getExpr2(), and_expr2->getExpr1());
            }
            else
                return false;
        }
    }

    if (Or* or_expr1 = dynamic_cast<Or*>(expr1))
    {
        if (Or* or_expr2 = dynamic_cast<Or*>(expr2))
        {
            if (equal_exprs(or_expr1->getExpr1(), or_expr2->getExpr1()))
            {
                return equal_exprs(or_expr1->getExpr2(), or_expr2->getExpr2());
            }
            else if (equal_exprs(or_expr1->getExpr1(), or_expr2->getExpr2()))
            {
                return equal_exprs(or_expr1->getExpr2(), or_expr2->getExpr1());
            }
            else
                return false;
        }
    }

    if (Xor* xor_expr1 = dynamic_cast<Xor*>(expr1))
    {
        if (Xor* xor_expr2 = dynamic_cast<Xor*>(expr2))
        {
            if (equal_exprs(xor_expr1->getExpr1(), xor_expr2->getExpr1()))
            {
                return equal_exprs(xor_expr1->getExpr2(), xor_expr2->getExpr2());
            }
            else if (equal_exprs(xor_expr1->getExpr1(), xor_expr2->getExpr2()))
            {
                return equal_exprs(xor_expr1->getExpr2(), xor_expr2->getExpr1());
            }
            else
                return false;
        }
    }

    if (Sel* sel_expr1 = dynamic_cast<Sel*>(expr1))
    {
        if (Sel* sel_expr2 = dynamic_cast<Sel*>(expr2))
        {
            if (equal_exprs(sel_expr1->getIf_expr(), sel_expr2->getIf_expr()))
            {
                if (equal_exprs(sel_expr1->getT_expr(), sel_expr2->getT_expr()))
                {
                    return equal_exprs(sel_expr1->getF_expr(), sel_expr2->getF_expr());
                }
                else
                    return false;
            }
            else
                return false;
        }
    }

    return false;
}
