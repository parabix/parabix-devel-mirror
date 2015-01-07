/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pe_and.h>
#include <pablo/codegenstate.h>

namespace pablo {

And::And(PabloAST * expr1, PabloAST * expr2, PabloBlock * parent)
: Statement(ClassTypeId::And, parent->makeName("and"), parent)
, mExprs({{expr1, expr2}})
{
    expr1->addUser(this);
    expr2->addUser(this);
}

PabloAST * OptimizeAnd::operator ()(PabloAST * expr1, PabloAST * expr2, PabloBlock * pb) {
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
            return pb->createNot(pb->createOr(pe_not_e1->getExpr(), pe_not_e2->getExpr()));
        }
        else if (equals(pe_not_e1->getExpr(), expr2)) {
            return pb->createZeroes();
        }
    }
    else if (Not * pe_not_e2 = dyn_cast<Not>(expr2)) {
        if (equals(expr1, pe_not_e2->getExpr())) {
            return pb->createZeroes();
        }
    }
    if (isa<Not>(expr1)) {
        std::swap(expr1, expr2);
    }
    return pb->createAndImm(expr1, expr2);
}

}
