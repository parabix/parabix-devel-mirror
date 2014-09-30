/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pablo_routines.h"
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

namespace pablo {

/*
    Optimizing Constructors for Boolean Expressions

     -Maintaining Assembler Instruction Form:
       -All boolean algebraic rules involving true/false applied.

       -Negations restricted:
         -no negations within or (DeMorgan's to nand)
         -at most one negation within and.
*/

PabloE * make_not(PabloE * expr)
{
    if (All * all = dynamic_cast<All*>(expr)) {
        if (all->getValue() == 1) { //If true literal.
            return new All(0); //Set to false literal.
        }
        else { //if (all->getNum() == 0) //If false literal.
            return new All(1); //Set to true literal.
        }
    }
    else if (Not* pe_not = dynamic_cast<Not*>(expr)) {
        PabloE * expr = pe_not->getExpr();
        pe_not->setExpr(nullptr);
        delete pe_not;
        return expr;
    }
    else {
        return new Not(expr);
    }
}

PabloE * make_and(PabloE * expr1, PabloE *expr2)
{
    if (All* all = dynamic_cast<All*>(expr1)) {
        if (all->getValue() == 1) {
            delete all;
            return expr2;
        }
        else { //if (all->getNum() == 0)
            delete expr1;
            delete expr2;
            return new All(0);
        }
    }
    else if (All* all = dynamic_cast<All*>(expr2)) {
        if (all->getValue() == 1) {
            delete expr2;
            return expr1;
        }
        else { //if (all->getNum() == 0)
            delete expr1;
            delete expr2;
            return new All(0);
        }
    }
    else if (equals(expr1, expr2)) {
        delete expr2;
        return expr1;
    }
    else if (Not* pe_not_e1 = dynamic_cast<Not*>(expr1)) {
        if (Not* pe_not_e2 = dynamic_cast<Not*>(expr2)) {

            PabloE * e1 = pe_not_e1->getExpr();
            pe_not_e1->setExpr(nullptr);
            delete pe_not_e1;

            PabloE * e2 = pe_not_e2->getExpr();
            pe_not_e2->setExpr(nullptr);
            delete pe_not_e2;

            return make_not(make_or(e1, e2));
        }
        else if (equals(pe_not_e1->getExpr(), expr2)) {
            delete expr1;
            delete expr2;
            return new All(0); //Return false literal.
        }
    }
    else if (Not * pe_not_e2 = dynamic_cast<Not*>(expr2)) {
        if (equals(expr1, pe_not_e2->getExpr())) {
            delete expr1;
            delete expr2;
            return new All(0);
        }
    }
    return new And(expr1, expr2);
}

PabloE * make_or(PabloE * expr1, PabloE * expr2)
{
    if (All * all = dynamic_cast<All*>(expr1)) {
        if (all->getValue() == 1) {
            delete expr2;
            return all; //Return a true literal.
        }
        else { //if (all->getNum() == 0)
            delete all;
            return expr2;
        }
    }
    else if (All * all = dynamic_cast<All*>(expr2)) {
        if (all->getValue() == 1) {
            delete expr1;
            return all; //Return a true literal.
        }
        else { //if (all->getNum() == 0)
            delete all;
            return expr1;
        }
    }
    else if (Not* pe_not_e1 = dynamic_cast<Not*>(expr1)) {
        // ¬a||b = ¬¬(¬a||b) = ¬(a ∧ ¬b)
        PabloE * expr1 = pe_not_e1->getExpr();
        pe_not_e1->setExpr(nullptr);
        delete pe_not_e1;
        return make_not(make_and(expr1, make_not(expr2)));
    }
    else if (Not* pe_not_e2 = dynamic_cast<Not*>(expr2)) {
        PabloE * expr2 = pe_not_e1->getExpr();
        pe_not_e2->setExpr(nullptr);
        delete pe_not_e2;
        return make_not(make_and(expr2, make_not(expr1)));
    }
    else if (equals(expr1, expr2)) {
        delete expr2;
        return expr1;
    }

    if (And * and_expr1 = dynamic_cast<And*>(expr1))
    {
        if (And * and_expr2 = dynamic_cast<And*>(expr2))
        {
            PabloE * expr1a = and_expr1->getExpr1();
            PabloE * expr1b = and_expr1->getExpr2();
            PabloE * expr2a = and_expr2->getExpr1();
            PabloE * expr2b = and_expr2->getExpr1();

            //These optimizations factor out common components that can occur when sets are formed by union
            //(e.g., union of [a-z] and [A-Z].
            if (equals(expr1a, expr2a))
            {
                return make_and(expr1a, make_or(expr1b, expr2b));
            }
            else if (equals(expr1b, expr2b))
            {
                return make_and(expr1b, make_or(expr1a, expr2a));
            }
            else if (equals(expr1a, expr2b))
            {
                return make_and(expr1a, make_or(expr1b, expr2a));
            }
            else if (equals(expr1b, expr2a))
            {
                return make_and(expr1b, make_or(expr1a, expr2b));
            }
        }
    }

    return new Or(expr1, expr2);
}

PabloE* make_sel(PabloE *if_expr, PabloE *t_expr, PabloE *f_expr)
{
    if (All* all_if_expr = dynamic_cast<All*>(if_expr))
    {
        if (all_if_expr->getValue() == 1)
        {
            return t_expr;
        }
        else //if (all_if_expr->getNum() == 0)
        {
            return f_expr;
        }
    }
    else if (All* all_t_expr = dynamic_cast<All*>(t_expr))
    {
        if (all_t_expr->getValue() == 1)
        {
            return make_or(if_expr, f_expr);
        }
        else //if (all_t_expr->getNum() == 0)
        {
            return make_and(make_not(if_expr), f_expr);
        }
    }
    else if (All* all_f_expr = dynamic_cast<All*>(f_expr))
    {
        if (all_f_expr->getValue() == 1)
        {
            return make_or(make_not(if_expr), t_expr);
        }
        else //if (all_f_expr->getNum() == 0)
        {
            return make_and(if_expr, t_expr);
        }
    }
    else if (equals(t_expr, f_expr))
    {
        return t_expr;
    }
    else
    {
        return new Sel(if_expr, t_expr, f_expr);
    }
}

PabloE* make_xor(PabloE *expr1, PabloE *expr2)
{
    if (All* all_expr1 = dynamic_cast<All*>(expr1))
    {
        if (all_expr1->getValue() == 1)
        {
            return make_not(expr2);
        }
        else //if (all_expr1->getNum() == 0)
        {
            return expr2;
        }
    }
    else if (All* all_expr2 = dynamic_cast<All*>(expr2))
    {
        if (all_expr2->getValue() == 1)
        {
            return make_not(expr1);
        }
        else //if (all_expr2->getNum() == 0)
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

bool equals(const PabloE * expr1, const PabloE * expr2)
{
    if (const All * all1 = dynamic_cast<const All*>(expr1)) {
        if (const All * all2 = dynamic_cast<const All*>(expr2)) {
            return all1->getValue() == all2->getValue();
        }
    }
    else if (const Var * var1 = dynamic_cast<const Var*>(expr1)) {
        if (const Var * var2 = dynamic_cast<const Var*>(expr2)) {
            return (var1->getVar() == var2->getVar());
        }
    }
    else if (const Not* not1 = dynamic_cast<const Not*>(expr1)) {
        if (const Not* not2 = dynamic_cast<const Not*>(expr2)) {
            return equals(not1->getExpr(), not2->getExpr());
        }
    }
    else if (const And* and1 = dynamic_cast<const And*>(expr1)) {
        if (const And* and2 = dynamic_cast<const And*>(expr2)) {
            if (equals(and1->getExpr1(), and2->getExpr1())) {
                return equals(and1->getExpr2(), and2->getExpr2());
            }
            else if (equals(and1->getExpr1(), and2->getExpr2())) {
                return equals(and1->getExpr2(), and2->getExpr1());
            }
        }
    }
    else if (const Or * or1 = dynamic_cast<const Or*>(expr1)) {
        if (const Or* or2 = dynamic_cast<const Or*>(expr2)) {
            if (equals(or1->getExpr1(), or2->getExpr1())) {
                return equals(or1->getExpr2(), or2->getExpr2());
            }
            else if (equals(or1->getExpr1(), or2->getExpr2())) {
                return equals(or1->getExpr2(), or2->getExpr1());
            }
        }
    }
    else if (const Xor * xor1 = dynamic_cast<const Xor *>(expr1)) {
        if (const Xor * xor2 = dynamic_cast<const Xor *>(expr2)) {
            if (equals(xor1->getExpr1(), xor2->getExpr1())) {
                return equals(xor1->getExpr2(), xor2->getExpr2());
            }
            else if (equals(xor1->getExpr1(), xor2->getExpr2())) {
                return equals(xor1->getExpr2(), xor2->getExpr1());
            }
        }
    }
    else if (const Sel* sel1 = dynamic_cast<const Sel*>(expr1)) {
        if (const Sel* sel2 = dynamic_cast<const Sel*>(expr2)) {
            if (equals(sel1->getIf_expr(), sel2->getIf_expr())) {
                if (equals(sel1->getT_expr(), sel2->getT_expr())) {
                    return equals(sel1->getF_expr(), sel2->getF_expr());
                }
            }
        }
    }

    return false;
}

} // end of namespace cc
