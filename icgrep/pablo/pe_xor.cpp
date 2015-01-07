/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pe_xor.h>
#include <pablo/codegenstate.h>

namespace pablo {

Xor::Xor(PabloAST * expr1, PabloAST * expr2, PabloBlock * parent)
: Statement(ClassTypeId::Xor, parent->makeName("xor"), parent)
, mExprs({{expr1, expr2}})
{
    expr1->addUser(this);
    expr2->addUser(this);
}

PabloAST * OptimizeXor::operator()(PabloAST * expr1, PabloAST * expr2, PabloBlock * pb) {
    if (isa<Ones>(expr1)) {
        return pb->createNot(expr2);
    }
    else if (isa<Zeroes>(expr1)){
        return expr2;        
    }
    else if (isa<Ones>(expr2)) {
        return pb->createNot(expr1);
    }
    else if (isa<Zeroes>(expr2)){
        return expr1;
    }
    else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return pb->createXor(not1->getExpr(), not2->getExpr());
        }
    }
    return pb->createXorImm(expr1, expr2);
}

}
