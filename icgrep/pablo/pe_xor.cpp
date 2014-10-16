/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pe_xor.h>
#include <pablo/codegenstate.h>

namespace pablo {

PabloAST * OptimizeXor::operator()(PabloAST * expr1, PabloAST * expr2) {
    if (All * all1 = dyn_cast<All>(expr1)) {
        if (all1->getValue()) {
            return cg.createNot(expr2);
        }
        else {
            return expr2;
        }
    }
    else if (All* all2 = dyn_cast<All>(expr2)) {
        if (all2->getValue() == 1) {
            return cg.createNot(expr1);
        }
        else { //if (all_expr2->getNum() == 0)
            return expr1;
        }
    }
    else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return cg.createXor(not1->getExpr(), not2->getExpr());
        }
    }
    return new Xor(expr1, expr2);
}

}
