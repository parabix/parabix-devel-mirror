/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pe_pabloe.h"
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

    Return true if expr1 and expr2 can be proven equivalent according to some rules,
    false otherwise.  Note that false may be returned i some cases when the exprs are
    equivalent.

*/

bool equals(const PabloE * expr1, const PabloE * expr2) {
    if (expr1->getClassTypeId() == expr2->getClassTypeId()) {
        if (const All * all1 = dyn_cast<const All>(expr1)) {
            if (const All * all2 = cast<const All>(expr2)) {
                return all1->getValue() == all2->getValue();
            }
        }
        else if (const Var * var1 = dyn_cast<const Var>(expr1)) {
            if (const Var * var2 = cast<const Var>(expr2)) {
                return (var1->getName() == var2->getName());
            }
        }
        else if (const Not* not1 = dyn_cast<const Not>(expr1)) {
            if (const Not* not2 = cast<const Not>(expr2)) {
                return equals(not1->getExpr(), not2->getExpr());
            }
        }
        else if (const And* and1 = dyn_cast<const And>(expr1)) {
            if (const And* and2 = cast<const And>(expr2)) {
                if (equals(and1->getExpr1(), and2->getExpr1())) {
                    return equals(and1->getExpr2(), and2->getExpr2());
                }
                else if (equals(and1->getExpr1(), and2->getExpr2())) {
                    return equals(and1->getExpr2(), and2->getExpr1());
                }
            }
        }
        else if (const Or * or1 = dyn_cast<const Or>(expr1)) {
            if (const Or* or2 = cast<const Or>(expr2)) {
                if (equals(or1->getExpr1(), or2->getExpr1())) {
                    return equals(or1->getExpr2(), or2->getExpr2());
                }
                else if (equals(or1->getExpr1(), or2->getExpr2())) {
                    return equals(or1->getExpr2(), or2->getExpr1());
                }
            }
        }
        else if (const Xor * xor1 = dyn_cast<const Xor>(expr1)) {
            if (const Xor * xor2 = cast<const Xor>(expr2)) {
                if (equals(xor1->getExpr1(), xor2->getExpr1())) {
                    return equals(xor1->getExpr2(), xor2->getExpr2());
                }
                else if (equals(xor1->getExpr1(), xor2->getExpr2())) {
                    return equals(xor1->getExpr2(), xor2->getExpr1());
                }
            }
        }
        else if (const Sel* sel1 = dyn_cast<const Sel>(expr1)) {
            if (const Sel* sel2 = cast<const Sel>(expr2)) {
                if (equals(sel1->getIf_expr(), sel2->getIf_expr())) {
                    if (equals(sel1->getT_expr(), sel2->getT_expr())) {
                        return equals(sel1->getF_expr(), sel2->getF_expr());
                    }
                }
            }
        }
    }
    return false;
}



PabloE::~PabloE(){ }



}
