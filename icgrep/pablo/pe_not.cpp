/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pe_not.h>
#include <pablo/codegenstate.h>


namespace pablo {

Not::Not(PabloAST * expr, PabloBlock * parent)
: Statement(ClassTypeId::Not, parent->makeName("not"), parent)
, mExpr(expr) {
    expr->addUser(this);
}

PabloAST * OptimizeNot::operator ()(PabloAST * expr, PabloBlock * pb) {
    if (isa<Ones>(expr)) {
    return pb->createZeroes();
    }
    else if (isa<Zeroes>(expr)){
        return pb->createOnes();
    }
    else if (Not * pe_not = dyn_cast<Not>(expr)) {
        return pe_not->getExpr();
    }
    return pb->createNotImm(expr);
}

}
