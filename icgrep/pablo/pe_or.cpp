/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pe_or.h>
#include <pablo/codegenstate.h>
#include <pablo/pe_ones.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_not.h>
#include <pablo/pe_and.h>

namespace pablo {

Or::Or(PabloAST * expr1, PabloAST * expr2, PabloBlock * parent)
: Statement(ClassTypeId::Or, {{expr1, expr2}}, parent->makeName("or"), parent)
{

}

PabloAST * OptimizeOr::operator ()(PabloAST * expr1, PabloAST * expr2, PabloBlock * pb) {

    assert (expr1 && expr2 && pb);

    if (isa<Zeroes>(expr2) || isa<Ones>(expr1)) {
        return expr1;
    }
    else if (isa<Zeroes>(expr1) || isa<Ones>(expr2)){
        return expr2;
    }
    else if (equals(expr1, expr2)) {
        return expr1;
    }
    else if (Not * not1 = dyn_cast<Not>(expr1)) {
        // ¬a∨b = ¬¬(¬a∨b) = ¬(a ∧ ¬b)
        return pb->createNot(pb->createAnd(not1->getExpr(), pb->createNot(expr2)));
    }
    else if (Not * not2 = dyn_cast<Not>(expr2)) {
        // a∨¬b = ¬¬(¬b∨a) = ¬(b ∧ ¬a)
        return pb->createNot(pb->createAnd(not2->getExpr(), pb->createNot(expr1)));
    }
    else if (equals(expr1, expr2)) {
        return expr1;
    }
    else if (And * and_expr1 = dyn_cast<And>(expr1)) {
        if (And * and_expr2 = dyn_cast<And>(expr2)) {
            PabloAST * const expr1a = and_expr1->getExpr1();
            PabloAST * const expr1b = and_expr1->getExpr2();
            PabloAST * const expr2a = and_expr2->getExpr1();
            PabloAST * const expr2b = and_expr2->getExpr2();
            //These optimizations factor out common components that can occur when sets are formed by union
            //(e.g., union of [a-z] and [A-Z].
            if (equals(expr1a, expr2a)) {
                return pb->createAnd(expr1a, pb->createOr(expr1b, expr2b));
            }
            else if (equals(expr1b, expr2b)) {
                return pb->createAnd(expr1b, pb->createOr(expr1a, expr2a));
            }
            else if (equals(expr1a, expr2b)) {
                return pb->createAnd(expr1a, pb->createOr(expr1b, expr2a));
            }
            else if (equals(expr1b, expr2a)) {
                return pb->createAnd(expr1b, pb->createOr(expr1a, expr2b));
            }
        }
    }
    return pb->createOrImm(expr1, expr2);
}

}
