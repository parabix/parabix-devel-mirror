/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/codegenstate.h>

namespace pablo {

/// UNARY CREATE FUNCTIONS

Advance * CodeGenState::createAdvance(PabloE * expr) {
    return mUnary.findOrMake<Advance>(PabloE::ClassTypeId::Advance, expr);
}

Call * CodeGenState::createCall(const std::string name) {
    return mUnary.findOrMake<Call>(PabloE::ClassTypeId::Call, mSymbolGenerator[name]);
}

CharClass * CodeGenState::createCharClass(const std::string name) {
    return mUnary.findOrMake<CharClass>(PabloE::ClassTypeId::CharClass, mSymbolGenerator[name]);
}

PabloE * CodeGenState::createNot(PabloE * expr) {
    return mUnary.findOrCall<OptimizeNot>(PabloE::ClassTypeId::Not, expr);
}

Var * CodeGenState::createVar(const std::string name) {
    return mUnary.findOrMake<Var>(PabloE::ClassTypeId::Var, mSymbolGenerator[name]);
}

Var * CodeGenState::createVar(Assign * assign) {
    return mUnary.findOrMake<Var>(PabloE::ClassTypeId::Var, const_cast<String *>(assign->mName));
}

/// BINARY CREATE FUNCTIONS

PabloE * CodeGenState::createAnd(PabloE * expr1, PabloE * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    return mBinary.findOrCall<OptimizeAnd>(PabloE::ClassTypeId::And, expr1, expr2);
}

Assign * CodeGenState::createAssign(const std::string name, PabloE * expr) {
    return mBinary.findOrMake<Assign>(PabloE::ClassTypeId::Assign, mSymbolGenerator[name], expr);
}

MatchStar * CodeGenState::createMatchStar(PabloE * expr1, PabloE * expr2) {
    return mBinary.findOrMake<MatchStar>(PabloE::ClassTypeId::MatchStar, expr1, expr2);
}

PabloE * CodeGenState::createOr(PabloE * expr1, PabloE * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    return mBinary.findOrCall<OptimizeOr>(PabloE::ClassTypeId::Or, expr1, expr2);
}

ScanThru * CodeGenState::createScanThru(PabloE * from, PabloE * thru) {
    return mBinary.findOrMake<ScanThru>(PabloE::ClassTypeId::ScanThru, from, thru);
}

PabloE * CodeGenState::createXor(PabloE * expr1, PabloE * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    return mBinary.findOrCall<OptimizeXor>(PabloE::ClassTypeId::Xor, expr1, expr2);
}

/// TERNARY CREATE FUNCTION

PabloE *CodeGenState::createSel(PabloE * condition, PabloE * trueExpr, PabloE * falseExpr) {
    return mTernary.findOrCall<OptimizeSel>(PabloE::ClassTypeId::Sel, condition, trueExpr, falseExpr);
}

}
