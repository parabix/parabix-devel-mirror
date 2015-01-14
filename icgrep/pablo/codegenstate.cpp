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
    OptimizeNot op;
    return op(expr, this);
}

Not * PabloBlock::createNotImm(PabloAST * expr) {
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
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    OptimizeAnd op;
    return op(expr1, expr2, this);
}

And * PabloBlock::createAndImm(PabloAST * expr1, PabloAST * expr2) {
    return insertAtInsertionPoint(new And(expr1, expr2, this));
}

PabloAST * PabloBlock::createOr(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    OptimizeOr op;
    return op(expr1, expr2, this);
}

Or * PabloBlock::createOrImm(PabloAST * expr1, PabloAST * expr2) {
    return insertAtInsertionPoint(new Or(expr1, expr2, this));
}

PabloAST * PabloBlock::createXor(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    OptimizeXor op;
    return op(expr1, expr2, this);
}

Xor * PabloBlock::createXorImm(PabloAST * expr1, PabloAST * expr2) {
    return insertAtInsertionPoint(new Xor(expr1, expr2,  this));
}

/// TERNARY CREATE FUNCTION

PabloAST * PabloBlock::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr) {
    OptimizeSel op;
    return op(condition, trueExpr, falseExpr, this);
}

Sel * PabloBlock::createSelImm(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr) {
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
