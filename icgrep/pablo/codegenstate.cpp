/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/codegenstate.h>

namespace pablo {

/// UNARY CREATE FUNCTIONS

Assign * PabloBlock::createAssign(const std::string prefix, PabloAST * expr, const int outputIndex)  {
    return insertAtInsertionPoint(new Assign(expr, outputIndex, mSymbolGenerator->make(prefix), this));
}

PabloAST * PabloBlock::createAdvance(PabloAST * expr, const int shiftAmount) {
    if (isa<Zeroes>(expr) || shiftAmount == 0) {
        return expr;
    }
    return insertAtInsertionPoint(new Advance(expr, shiftAmount, mSymbolGenerator, this));
}

Call * PabloBlock::createCall(const std::string name) {
    return createCall(mSymbolGenerator->get(name));
}

Call * PabloBlock::createCall(String * name) {
    return insertAtInsertionPoint(new Call(name, this));
}

PabloAST * PabloBlock::createNot(PabloAST * expr) {
    if (isa<Ones>(expr)) {
        return createZeroes();
    }
    else if (isa<Zeroes>(expr)){
        return createOnes();
    }
    else if (Not * not1 = dyn_cast<Not>(expr)) {
        return not1->getExpr();
    }
    return insertAtInsertionPoint(new Not(expr, this));
}

Var * PabloBlock::createVar(const std::string name) {
    return createVar(mSymbolGenerator->get(name));
}

Var * PabloBlock::createVar(String * name) {
    return insertAtInsertionPoint(new Var(name, this));
}

/// BINARY CREATE FUNCTIONS

Next * PabloBlock::createNext(Assign * assign, PabloAST * expr) {
    return insertAtInsertionPoint(new Next(assign, expr, this));
}

PabloAST * PabloBlock::createMatchStar(PabloAST * marker, PabloAST * charclass) {
    if (isa<Zeroes>(marker) || isa<Zeroes>(charclass)) {
        return marker;
    }
    return insertAtInsertionPoint(new MatchStar(marker, charclass, mSymbolGenerator, this));
}

PabloAST * PabloBlock::createScanThru(PabloAST * from, PabloAST * thru) {
    if (isa<Zeroes>(from) || isa<Zeroes>(thru)) {
        return from;
    }
    return insertAtInsertionPoint(new ScanThru(from, thru, mSymbolGenerator, this));
}

PabloAST * PabloBlock::createAnd(PabloAST * expr1, PabloAST * expr2) {
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
            return createNot(createOr(not1->getExpr(), not2->getExpr()));
        }
        else if (equals(not1->getExpr(), expr2)) {
            return createZeroes();
        }
    }
    else if (Not * not2 = dyn_cast<Not>(expr2)) {
        if (equals(expr1, not2->getExpr())) {
            return createZeroes();
        }
    }
    if (isa<Not>(expr1)) {
        std::swap(expr1, expr2);
    }
    return insertAtInsertionPoint(new And(expr1, expr2, this));
}


PabloAST * PabloBlock::createOr(PabloAST * expr1, PabloAST * expr2) {
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
        return createNot(createAnd(not1->getExpr(), createNot(expr2)));
    }
    else if (Not * not2 = dyn_cast<Not>(expr2)) {
        // a∨¬b = ¬¬(¬b∨a) = ¬(b ∧ ¬a)
        return createNot(createAnd(not2->getExpr(), createNot(expr1)));
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
                return createAnd(expr1a, createOr(expr1b, expr2b));
            }
            else if (equals(expr1b, expr2b)) {
                return createAnd(expr1b, createOr(expr1a, expr2a));
            }
            else if (equals(expr1a, expr2b)) {
                return createAnd(expr1a, createOr(expr1b, expr2a));
            }
            else if (equals(expr1b, expr2a)) {
                return createAnd(expr1b, createOr(expr1a, expr2b));
            }
        }
    }
    return insertAtInsertionPoint(new Or(expr1, expr2, this));
}

PabloAST * PabloBlock::createXor(PabloAST * expr1, PabloAST * expr2) {
    if (isa<Ones>(expr1)) {
        return createNot(expr2);
    }
    else if (isa<Zeroes>(expr1)){
        return expr2;
    }
    else if (isa<Ones>(expr2)) {
        return createNot(expr1);
    }
    else if (isa<Zeroes>(expr2)){
        return expr1;
    }
    else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return createXor(not1->getExpr(), not2->getExpr());
        }
    }
    return insertAtInsertionPoint(new Xor(expr1, expr2,  this));
}

/// TERNARY CREATE FUNCTION

PabloAST * PabloBlock::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr) {
    assert (condition && trueExpr && falseExpr && pb);

    if (isa<Ones>(condition)) {
        return trueExpr;
    }
    else if (isa<Zeroes>(condition)){
        return falseExpr;
    }
    else if (isa<Ones>(trueExpr)) {
        return createOr(condition, falseExpr);
    }
    else if (isa<Zeroes>(trueExpr)){
        return createAnd(createNot(condition), falseExpr);
    }
    else if (isa<Ones>(falseExpr)) {
        return createOr(createNot(condition), trueExpr);
    }
    else if (isa<Zeroes>(falseExpr)){
        return createAnd(condition, trueExpr);
    }
    else if (equals(trueExpr, falseExpr)) {
        return trueExpr;
    }
    else if (isa<Not>(trueExpr) && equals(cast<Not>(trueExpr)->getExpr(), falseExpr)) {
        return createXor(condition, falseExpr);
    }
    else if (isa<Not>(falseExpr) && equals(trueExpr, cast<Not>(falseExpr)->getExpr())){
        return createXor(condition, falseExpr);
    }
    return insertAtInsertionPoint(new Sel(condition, trueExpr, falseExpr, this));
}

If * PabloBlock::createIf(PabloAST * condition, std::vector<Assign *> && definedVars, PabloBlock & body) {
    return insertAtInsertionPoint(new If(condition, std::move(definedVars), body, this));
}

While * PabloBlock::createWhile(PabloAST * cond, PabloBlock & body) {
    return insertAtInsertionPoint(new While(cond, body, this));
}

/// CONSTRUCTOR

PabloBlock::PabloBlock()
: mZeroes(new Zeroes())
, mOnes(new Ones())
, mSymbolGenerator(new SymbolGenerator())
{

}

PabloBlock::PabloBlock(PabloBlock * predecessor)
: mZeroes(predecessor->mZeroes) // inherit the original "Zeroes" variable for simplicity
, mOnes(predecessor->mOnes) // inherit the original "Ones" variable for simplicity
, mSymbolGenerator(predecessor->mSymbolGenerator)
{

}

}
