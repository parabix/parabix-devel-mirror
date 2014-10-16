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
    return mUnary.findOrMake<Call>(PabloAST::ClassTypeId::Call, mSymbolGenerator[name]);
}

CharClass * PabloBlock::createCharClass(const std::string name) {
    return mUnary.findOrMake<CharClass>(PabloAST::ClassTypeId::CharClass, mSymbolGenerator[name]);
}

PabloAST * PabloBlock::createNot(PabloAST * expr) {
    return mUnary.findOrCall<OptimizeNot>(PabloAST::ClassTypeId::Not, expr);
}

Var * PabloBlock::createVar(const std::string name) {
    return mUnary.findOrMake<Var>(PabloAST::ClassTypeId::Var, mSymbolGenerator[name]);
}

Var * PabloBlock::createVar(Assign * assign) {
    return mUnary.findOrMake<Var>(PabloAST::ClassTypeId::Var, const_cast<String *>(assign->mName));
}

/// BINARY CREATE FUNCTIONS

PabloAST * PabloBlock::createAnd(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    return mBinary.findOrCall<OptimizeAnd>(PabloAST::ClassTypeId::And, expr1, expr2);
}

Assign * PabloBlock::createAssign(const std::string name, PabloAST * expr) {
//    auto key = std::make_tuple(PabloAST::ClassTypeId::Assign, expr);
//    Assign * assign = cast<Assign>(mUnary.find(key));
//    if (assign == nullptr) {
//        assign = new Assign(mSymbolGenerator[name], expr);
//        mUnary.insert(std::move(key), assign);
//    }
//    else {
//        assign = new Assign(mSymbolGenerator[name], createVar(assign));
//    }
    Assign * assign = mBinary.findOrMake<Assign>(PabloAST::ClassTypeId::Assign, mSymbolGenerator[name], expr);
    mStatements.push_back(assign);
    return assign;
}

MatchStar * PabloBlock::createMatchStar(PabloAST * expr1, PabloAST * expr2) {
    return mBinary.findOrMake<MatchStar>(PabloAST::ClassTypeId::MatchStar, expr1, expr2);
}

PabloAST * PabloBlock::createOr(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    return mBinary.findOrCall<OptimizeOr>(PabloAST::ClassTypeId::Or, expr1, expr2);
}

ScanThru * PabloBlock::createScanThru(PabloAST * from, PabloAST * thru) {
    return mBinary.findOrMake<ScanThru>(PabloAST::ClassTypeId::ScanThru, from, thru);
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
