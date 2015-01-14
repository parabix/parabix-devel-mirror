/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pe_and.h>
#include <pablo/codegenstate.h>
#include <pablo/pe_not.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>

namespace pablo {

And::And(PabloAST * expr1, PabloAST * expr2, PabloBlock * parent)
: Statement(ClassTypeId::And, {{expr1, expr2}}, parent->makeName("and"), parent)
{

}

PabloAST * OptimizeAnd::operator ()(PabloAST * expr1, PabloAST * expr2, PabloBlock * pb) {

    assert (expr1 && expr2 && pb);

    if (isa<Zeroes>(expr2) || isa<Ones>(expr1)) {
        return expr2;
    }
    else if (isa<Zeroes>(expr1) || isa<Ones>(expr2)){
        return expr1;
    }
    else if (equals(expr1, expr2)) {
        return expr1;
    }
    else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return pb->createNot(pb->createOr(not1->getExpr(), not2->getExpr()));
        }
        else if (equals(not1->getExpr(), expr2)) {
            return pb->createZeroes();
        }
    }
    else if (Not * not2 = dyn_cast<Not>(expr2)) {
        if (equals(expr1, not2->getExpr())) {
            return pb->createZeroes();
        }
    }
    if (isa<Not>(expr1)) {
        std::swap(expr1, expr2);
    }
    return pb->createAndImm(expr1, expr2);
}

}
