/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pe_or.h"
#include "pe_all.h"
#include "pe_not.h"
#include "pe_and.h"

namespace pablo {

PabloE * make_or(PabloE * expr1, PabloE * expr2) {
    if (All * all = dyn_cast<All>(expr1)) {
        if (all->getValue() == 1) {
            return all; //Return a true literal.
        }
        else {
            return expr2;
        }
    }
    else if (All * all = dyn_cast<All>(expr2)) {
        if (all->getValue() == 1) {
            return all; //Return a true literal.
        }
        else {
            return expr1;
        }
    }
    else if (Not * not1 = dyn_cast<Not>(expr1)) {
        // ¬a∨b = ¬¬(¬a∨b) = ¬(a ∧ ¬b)
        return make_not(make_and(not1->getExpr(), make_not(expr2)));
    }
    else if (Not * not2 = dyn_cast<Not>(expr2)) {
        // a∨¬b = ¬¬(¬b∨a) = ¬(b ∧ ¬a)
        return make_not(make_and(not2->getExpr(), make_not(expr1)));
    }
    else if (equals(expr1, expr2)) {
        return expr1;
    }
    else if (And * and_expr1 = dyn_cast<And>(expr1)) {
        if (And * and_expr2 = dyn_cast<And>(expr2)) {
            PabloE * const expr1a = and_expr1->getExpr1();
            PabloE * const expr1b = and_expr1->getExpr2();
            PabloE * const expr2a = and_expr2->getExpr1();
            PabloE * const expr2b = and_expr2->getExpr1();
            //These optimizations factor out common components that can occur when sets are formed by union
            //(e.g., union of [a-z] and [A-Z].
            if (equals(expr1a, expr2a)) {
                return make_and(expr1a, make_or(expr1b, expr2b));
            }
            else if (equals(expr1b, expr2b)) {
                return make_and(expr1b, make_or(expr1a, expr2a));
            }
            else if (equals(expr1a, expr2b)) {
                return make_and(expr1a, make_or(expr1b, expr2a));
            }
            else if (equals(expr1b, expr2a)) {
                return make_and(expr1b, make_or(expr1a, expr2b));
            }
        }
    }

    return new Or(expr1, expr2);
}

}
