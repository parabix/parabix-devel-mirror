/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/codegenstate.h>
#include <iostream>

namespace pablo {

/// UNARY CREATE FUNCTIONS

PabloAST * PabloBlock::createAdvance(PabloAST * expr, const int shiftAmount) {
    if (isa<Zeroes>(expr) || shiftAmount == 0) {
        return expr;
    }
    return mUnaryWithInt.findOrMake<Advance>(PabloAST::ClassTypeId::Advance, expr, shiftAmount).first;
}

PabloAST * PabloBlock::createCall(const std::string name) {
    return mUnary.findOrMake<Call>(PabloAST::ClassTypeId::Call, mSymbolGenerator->get(name)).first;
}

PabloAST * PabloBlock::createNot(PabloAST * expr) {
    return mUnary.findOrCall<OptimizeNot>(PabloAST::ClassTypeId::Not, expr).first;
}

Var * PabloBlock::createVar(const std::string name) {
    return mUnary.findOrMake<Var>(PabloAST::ClassTypeId::Var, mSymbolGenerator->get(name)).first;
}

Var * PabloBlock::createVar(Assign * assign) {
    return mUnary.findOrMake<Var>(PabloAST::ClassTypeId::Var, assign).first;
}

Var * PabloBlock::createVar(Next * next) {
    return mUnary.findOrMake<Var>(PabloAST::ClassTypeId::Var, next).first;
}

/// BINARY CREATE FUNCTIONS

Next * PabloBlock::createNext(Assign * assign, PabloAST * expr) {
    Next * next;
    bool added;
    std::tie(next, added) = mBinary.findOrMake<Next>(PabloAST::ClassTypeId::Next, assign, expr, this);
    if (LLVM_LIKELY(added)) {
        push_back(next);
    }
    return next;
}

PabloAST * PabloBlock::createMatchStar(PabloAST * marker, PabloAST * charclass) {
    if (isa<Zeroes>(marker) || isa<Zeroes>(charclass)) {
        return marker;
    }
    return mBinary.findOrMake<MatchStar>(PabloAST::ClassTypeId::MatchStar, marker, charclass).first;
}

PabloAST * PabloBlock::createScanThru(PabloAST * from, PabloAST * thru) {
    if (isa<Zeroes>(from) || isa<Zeroes>(thru)) {
        return from;
    }
    return mBinary.findOrMake<ScanThru>(PabloAST::ClassTypeId::ScanThru, from, thru).first;
}

PabloAST * PabloBlock::createAnd(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    return mBinary.findOrCall<OptimizeAnd>(PabloAST::ClassTypeId::And, expr1, expr2).first;
}

PabloAST * PabloBlock::createOr(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    return mBinary.findOrCall<OptimizeOr>(PabloAST::ClassTypeId::Or, expr1, expr2).first;
}

PabloAST * PabloBlock::createXor(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    return mBinary.findOrCall<OptimizeXor>(PabloAST::ClassTypeId::Xor, expr1, expr2).first;
}

/// TERNARY CREATE FUNCTION

PabloAST * PabloBlock::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr) {
    return mTernary.findOrCall<OptimizeSel>(PabloAST::ClassTypeId::Sel, condition, trueExpr, falseExpr).first;
}

//PabloAST * PabloBlock::replaceUsesOfWith(PabloAST * inst, PabloAST * from, PabloAST * to) {
//    if (from == to) {
//        return inst;
//    }
//    switch (inst->getClassTypeId()) {
//        case PabloAST::ClassTypeId::Advance:
//            {
//                Advance * n = cast<Advance>(inst);
//                if (n->getExpr() == from) {
//                    return createAdvance(to, n->getAdvanceAmount());
//                }
//                return inst;
//            }
//        case PabloAST::ClassTypeId::Assign:
//            {
//                Assign * n = cast<Assign>(inst);
//                if (n->getExpr() == from) {
//                    Assign * assign = createAssign(to);
//                    n->replaceWith(assign);
//                    return assign;
//                }
//                return inst;
//            }
//        case PabloAST::ClassTypeId::Var:
//            {
//                Var * n = cast<Var>(inst);
//                if (n->getVar() == from) {
//                    return createVar(to);
//                }
//                return inst;
//            }
//        case PabloAST::ClassTypeId::Next:
//            {
//                Next * n = cast<Next>(inst);
//                if (n->getInitial() == from || n->getExpr() == from) {
//                    return createNext(n->getInitial() == from ? to : n->getInitial(),
//                                      n->getExpr() == from ? to : n->getExpr());
//                }
//                return inst;
//            }
////        case PabloAST::ClassTypeId::Call:
////            {
////                Call * n = cast<Call>(node);
////                if (n->getCallee() == from) {
////                    return createCall(to, n->getExpr());
////                }
////                return node;
////            }
//        case PabloAST::ClassTypeId::Not:
//            {
//                Not * n = cast<Not>(inst);
//                if (n->getExpr() == from) {
//                    return createNot(to);
//                }
//                return inst;
//            }
//        case PabloAST::ClassTypeId::And:
//            {
//                And * n = cast<And>(inst);
//                if (n->getExpr1() == from || n->getExpr2() == from) {
//                    return createAnd(n->getExpr1() == from ? to : n->getExpr1(),
//                                     n->getExpr2() == from ? to : n->getExpr2());
//                }
//                return inst;
//            }
//        case PabloAST::ClassTypeId::Or:
//            {
//                Or * n = cast<Or>(inst);
//                if (n->getExpr1() == from || n->getExpr2() == from) {
//                    return createOr(n->getExpr1() == from ? to : n->getExpr1(),
//                                    n->getExpr2() == from ? to : n->getExpr2());
//                }
//                return inst;
//            }
//        case PabloAST::ClassTypeId::Xor:
//            {
//                Xor * n = cast<Xor>(inst);
//                if (n->getExpr1() == from || n->getExpr2() == from) {
//                    return createXor(n->getExpr1() == from ? to : n->getExpr1(),
//                                     n->getExpr2() == from ? to : n->getExpr2());
//                }
//                return inst;
//            }
//        case PabloAST::ClassTypeId::ScanThru:
//            {
//                ScanThru * n = cast<ScanThru>(inst);
//                if (n->getScanFrom() == from || n->getScanThru() == from) {
//                    return createScanThru(n->getScanFrom() == from ? to : n->getScanFrom(),
//                                          n->getScanThru() == from ? to : n->getScanThru());
//                }
//                return inst;
//            }
//        case PabloAST::ClassTypeId::MatchStar:
//            {
//                MatchStar * n = cast<MatchStar>(inst);
//                if (n->getMarker() == from || n->getScanThru() == from) {
//                    return createMatchStar(n->getMarker() == from ? to : n->getMarker(),
//                                           n->getScanThru() == from ? to : n->getScanThru());
//                }
//                return inst;
//            }
//        case PabloAST::ClassTypeId::Ones:
//        case PabloAST::ClassTypeId::Zeroes:
//        case PabloAST::ClassTypeId::If:
//        case PabloAST::ClassTypeId::While:
//            return inst;
//        default:
//            throw std::runtime_error("Unhandled node type (" + std::to_string(inst->getClassTypeId()) +
//                                     ") given to PabloBlock::replaceUsesOfWith(...)")
//    }

//}


}
