/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/codegenstate.h>

namespace pablo {

/// UNARY CREATE FUNCTIONS

Advance * PabloBlock::createAdvance(PabloAST * expr) {
    return mUnary.findOrMake<Advance>(PabloAST::ClassTypeId::Advance, expr);
}

Call * PabloBlock::createCall(const std::string name) {
    return mUnary.findOrMake<Call>(PabloAST::ClassTypeId::Call, mSymbolGenerator.get(name));
}

PabloAST * PabloBlock::createNot(PabloAST * expr) {
    return mUnary.findOrCall<OptimizeNot>(PabloAST::ClassTypeId::Not, expr);
}

Var * PabloBlock::createVar(const std::string name) {
    return mUnary.findOrMake<Var>(PabloAST::ClassTypeId::Var, mSymbolGenerator.get(name));
}

Var * PabloBlock::createVar(String * name) {
    return mUnary.findOrMake<Var>(PabloAST::ClassTypeId::Var, name);
}

/// BINARY CREATE FUNCTIONS

Next * PabloBlock::createNext(Assign * assign, PabloAST * expr) {
    Next * next = mBinary.findOrMake<Next>(PabloAST::ClassTypeId::Next, assign, expr);
    mStatements.push_back(next);
    return next;
}

MatchStar * PabloBlock::createMatchStar(PabloAST * marker, PabloAST * charclass) {
    return mBinary.findOrMake<MatchStar>(PabloAST::ClassTypeId::MatchStar, marker, charclass);
}

ScanThru * PabloBlock::createScanThru(PabloAST * from, PabloAST * thru) {
    return mBinary.findOrMake<ScanThru>(PabloAST::ClassTypeId::ScanThru, from, thru);
}

PabloAST * PabloBlock::createAnd(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    return mBinary.findOrCall<OptimizeAnd>(PabloAST::ClassTypeId::And, expr1, expr2);
}

PabloAST * PabloBlock::createOr(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    return mBinary.findOrCall<OptimizeOr>(PabloAST::ClassTypeId::Or, expr1, expr2);
}

PabloAST * PabloBlock::createXor(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    return mBinary.findOrCall<OptimizeXor>(PabloAST::ClassTypeId::Xor, expr1, expr2);
}

/// TERNARY CREATE FUNCTION

PabloAST *PabloBlock::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr) {
    return mTernary.findOrCall<OptimizeSel>(PabloAST::ClassTypeId::Sel, condition, trueExpr, falseExpr);
}

}
