/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pe_and.h"
#include "pe_all.h"
#include "pe_not.h"
#include "pe_or.h"

namespace pablo {

PabloE * make_and(PabloE * expr1, PabloE *expr2) {
    if (All * all = dyn_cast<All>(expr1)) {
        if (all->getValue()) {
            return expr2;
        }
        else {
            return make_all(0);
        }
    }
    else if (All* all = dyn_cast<All>(expr2)) {
        if (all->getValue()) {
            return expr1;
        }
        else {
            return make_all(0);
        }
    }
    else if (equals(expr1, expr2)) {
        return expr1;
    }
    else if (Not * pe_not_e1 = dyn_cast<Not>(expr1)) {
        if (Not * pe_not_e2 = dyn_cast<Not>(expr2)) {
            return make_not(make_or(pe_not_e1->getExpr(), pe_not_e2->getExpr()));
        }
        else if (equals(pe_not_e1->getExpr(), expr2)) {
            return make_all(0);
        }
    }
    else if (Not * pe_not_e2 = dyn_cast<Not>(expr2)) {
        if (equals(expr1, pe_not_e2->getExpr())) {
            return make_all(0);
        }
    }
    return new And(expr1, expr2);
}

}
