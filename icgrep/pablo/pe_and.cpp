/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pe_and.h>
#include <pablo/codegenstate.h>

namespace pablo {

PabloAST * OptimizeAnd::operator ()(PabloAST * expr1, PabloAST * expr2) {
    if (isa<Ones>(expr1)) {
	return expr2;
    }
    else if (isa<Zeroes>(expr1)){
        return expr1;        
    }
    else if (isa<Ones>(expr2)) {
	return expr1;
    }
    else if (isa<Zeroes>(expr2)){
        return expr2;
    }
    else if (equals(expr1, expr2)) {
        return expr1;
    }
    else if (Not * pe_not_e1 = dyn_cast<Not>(expr1)) {
        if (Not * pe_not_e2 = dyn_cast<Not>(expr2)) {
            return cg.createNot(cg.createOr(pe_not_e1->getExpr(), pe_not_e2->getExpr()));
        }
        else if (equals(pe_not_e1->getExpr(), expr2)) {
            return cg.createZeroes();
        }
    }
    else if (Not * pe_not_e2 = dyn_cast<Not>(expr2)) {
        if (equals(expr1, pe_not_e2->getExpr())) {
            return cg.createZeroes();
        }
    }
    return new And(expr1, expr2);
}

}
