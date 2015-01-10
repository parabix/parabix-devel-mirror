/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/codegenstate.h>

namespace pablo {

/// UNARY CREATE FUNCTIONS

PabloAST * PabloBlock::createAdvance(PabloAST * expr, const int shiftAmount) {
    if (isa<Zeroes>(expr) || shiftAmount == 0) {
        return expr;
    }
    return insertIfNew(mUnaryWithInt.findOrMake<Advance>(PabloAST::ClassTypeId::Advance, expr, shiftAmount, mSymbolGenerator, this));
}

Assign * PabloBlock::createImmutableAssign(const std::string prefix, PabloAST * expr, const int outputIndex) {
    return insertIfNew(mUnary.findOrMake<Assign>(PabloAST::ClassTypeId::Assign, expr, outputIndex, mSymbolGenerator->make(prefix), this));
}

Call * PabloBlock::createCall(const std::string name) {
    return createCall(mSymbolGenerator->get(name));
}

Call * PabloBlock::createCall(String * name) {
    return insertIfNew(mUnary.findOrMake<Call>(PabloAST::ClassTypeId::Call, name, this));
}

PabloAST * PabloBlock::createNot(PabloAST * expr) {
    return mUnary.findOrCall<OptimizeNot>(PabloAST::ClassTypeId::Not, expr, this);
}

Not * PabloBlock::createNotImm(PabloAST * expr) {
    return insertIfNew(mUnary.findOrMake<Not>(PabloAST::ClassTypeId::Not, expr, this));
}

Var * PabloBlock::createVar(const std::string name) {
    return createVar(mSymbolGenerator->get(name));
}

Var * PabloBlock::createVar(String * name) {
    return mUnary.findOrMake<Var>(PabloAST::ClassTypeId::Var, name, this).first;
}

/// BINARY CREATE FUNCTIONS

Next * PabloBlock::createNext(Assign * assign, PabloAST * expr) {
    return insertIfNew(mBinary.findOrMake<Next>(PabloAST::ClassTypeId::Next, assign, expr, this));
}

PabloAST * PabloBlock::createMatchStar(PabloAST * marker, PabloAST * charclass) {
    if (isa<Zeroes>(marker) || isa<Zeroes>(charclass)) {
        return marker;
    }
    return insertIfNew(mBinary.findOrMake<MatchStar>(PabloAST::ClassTypeId::MatchStar, marker, charclass, mSymbolGenerator, this));
}

PabloAST * PabloBlock::createScanThru(PabloAST * from, PabloAST * thru) {
    if (isa<Zeroes>(from) || isa<Zeroes>(thru)) {
        return from;
    }
    return insertIfNew(mBinary.findOrMake<ScanThru>(PabloAST::ClassTypeId::ScanThru, from, thru, mSymbolGenerator, this));
}

PabloAST * PabloBlock::createAnd(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    return mBinary.findOrCall<OptimizeAnd>(PabloAST::ClassTypeId::And, expr1, expr2, this);
}

And * PabloBlock::createAndImm(PabloAST * expr1, PabloAST * expr2) {
    return insertIfNew(mBinary.findOrMake<And>(PabloAST::ClassTypeId::And, expr1, expr2, this));
}

PabloAST * PabloBlock::createOr(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    return mBinary.findOrCall<OptimizeOr>(PabloAST::ClassTypeId::Or, expr1, expr2, this);
}

Or * PabloBlock::createOrImm(PabloAST * expr1, PabloAST * expr2) {
    return insertIfNew(mBinary.findOrMake<Or>(PabloAST::ClassTypeId::Or, expr1, expr2, this));
}

PabloAST * PabloBlock::createXor(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    return mBinary.findOrCall<OptimizeXor>(PabloAST::ClassTypeId::Xor, expr1, expr2, this);
}

Xor * PabloBlock::createXorImm(PabloAST * expr1, PabloAST * expr2) {
    return insertIfNew(mBinary.findOrMake<Xor>(PabloAST::ClassTypeId::Xor, expr1, expr2,  this));
}

/// TERNARY CREATE FUNCTION

PabloAST * PabloBlock::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr) {
    return mTernary.findOrCall<OptimizeSel>(PabloAST::ClassTypeId::Sel, condition, trueExpr, falseExpr, this);
}

Sel * PabloBlock::createSelImm(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr) {
    return insertIfNew(mTernary.findOrMake<Sel>(PabloAST::ClassTypeId::Sel, condition, trueExpr, falseExpr, this));
}

PabloAST * PabloBlock::setOperandOf(Statement * inst, const unsigned index, PabloAST * value) {

    assert (index < inst->getNumOperands() && inst->getOperand(index) != value);

    // get the current insertion point so we can restore it later
    Statement * const ip = getInsertPoint();
    const bool replacingCurrentIP = (ip == inst);

    // set the current ip to the position of the inst we're "updating"
    setInsertPoint(inst);
    // this will move the ip back to inst's prior node
    inst->removeFromParent();

    PabloAST * retVal = inst;

    switch (inst->getClassTypeId()) {
        case PabloAST::ClassTypeId::Advance:
            retVal = createAdvance(value, cast<Advance>(inst)->getAdvanceAmount());
            break;
        case PabloAST::ClassTypeId::Not:
            retVal = createNot(value);
            break;
//        case PabloAST::ClassTypeId::Assign:
//            retVal = createAssign(value, cast<Assign>(inst)->getOutputIndex());
//            break;
        case PabloAST::ClassTypeId::Var:
            retVal = createVar(cast<String>(value));
            break;
        case PabloAST::ClassTypeId::Call:
            retVal = createCall(cast<String>(value));
            break;
        case PabloAST::ClassTypeId::And:
            retVal = createAnd(index == 0 ? value : inst->getOperand(0), index == 1 ? value : inst->getOperand(1));
            break;
        case PabloAST::ClassTypeId::Or:
            retVal = createOr(index == 0 ? value : inst->getOperand(0), index == 1 ? value : inst->getOperand(1));
            break;
        case PabloAST::ClassTypeId::Xor:
            retVal = createXor(index == 0 ? value : inst->getOperand(0), index == 1 ? value : inst->getOperand(1));
            break;
        case PabloAST::ClassTypeId::Next:
            retVal = createNext(cast<Assign>(index == 0 ? value : inst->getOperand(0)), index == 1 ? value : inst->getOperand(1));
            break;
        case PabloAST::ClassTypeId::ScanThru:
            retVal = createScanThru(index == 0 ? value : inst->getOperand(0), index == 1 ? value : inst->getOperand(1));
            break;
        case PabloAST::ClassTypeId::MatchStar:
            retVal = createMatchStar(index == 0 ? value : inst->getOperand(0), index == 1 ? value : inst->getOperand(1));
            break;
        case PabloAST::ClassTypeId::Sel:
            retVal = createSel(index == 0 ? value : inst->getOperand(0), index == 1 ? value : inst->getOperand(1), index == 2 ? value : inst->getOperand(2));
            break;
        default:
            break;
    }

    // restore our insertion point
    setInsertPoint(replacingCurrentIP ? inst : ip);

    return retVal;
}

}
