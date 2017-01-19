#include "builder.hpp"
#include <pablo/boolean.h>
#include <pablo/arithmetic.h>
#include <pablo/branch.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_lookahead.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_infile.h>
#include <pablo/pe_count.h>
#include <pablo/pe_integer.h>
#include <pablo/pe_string.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <pablo/pe_var.h>
#include <pablo/ps_assign.h>

using namespace llvm;

namespace pablo {

#define MAKE_UNARY(NAME, TYPE, ARGS...) \
struct __##NAME { \
    inline PabloAST * operator()(PabloAST * arg) { \
        return mPb->NAME(arg); \
    } \
    inline __##NAME(PabloBlock * pb) : mPb(pb) {} \
private: \
    PabloBlock * mPb; \
}; \
__##NAME functor(mPb); \
PabloAST * result = mExprTable.findUnaryOrCall(std::move(functor), TYPE, ARGS)

#define MAKE_NAMED_UNARY(NAME, TYPE, PREFIX, ARGS...) \
struct __##NAME { \
    inline PabloAST * operator()(PabloAST * arg) { \
        return mPb->NAME(arg, mPrefix); \
    } \
    inline __##NAME(PabloBlock * pb, const std::string & prefix) : mPb(pb), mPrefix(prefix) {} \
private: \
    PabloBlock * mPb; \
    const std::string & mPrefix; \
}; \
__##NAME functor(mPb, prefix); \
PabloAST * result = mExprTable.findUnaryOrCall(std::move(functor), TYPE, ARGS)

#define MAKE_BINARY(NAME, TYPE, ARGS...) \
struct __##NAME { \
    inline PabloAST * operator()(PabloAST * arg1, PabloAST * arg2) { \
        return mPb->NAME(arg1, arg2); \
    } \
    inline __##NAME(PabloBlock * pb) : mPb(pb) {} \
private: \
    PabloBlock * mPb; \
}; \
__##NAME functor(mPb); \
PabloAST * result = mExprTable.findBinaryOrCall(std::move(functor), TYPE, ARGS)

#define MAKE_NAMED_BINARY(NAME, TYPE, PREFIX, ARGS...) \
struct __##NAME { \
    inline PabloAST * operator()(PabloAST * arg1, PabloAST * arg2) { \
        return mPb->NAME(arg1, arg2, mPrefix); \
    } \
    inline __##NAME(PabloBlock * pb, const std::string & prefix) : mPb(pb), mPrefix(prefix) {} \
private: \
    PabloBlock * mPb; \
    const std::string & mPrefix; \
}; \
__##NAME functor(mPb, PREFIX); \
PabloAST * result = mExprTable.findBinaryOrCall(std::move(functor), TYPE, ARGS)

#define MAKE_TERNARY(NAME, TYPE, ARGS...) \
struct __##NAME { \
    inline PabloAST * operator()(PabloAST * arg1, PabloAST * arg2, PabloAST * arg3) { \
        return mPb->NAME(arg1, arg2, arg3); \
    } \
    inline __##NAME(PabloBlock * pb) : mPb(pb) {} \
private: \
    PabloBlock * mPb; \
}; \
__##NAME functor(mPb); \
PabloAST * result = mExprTable.findTernaryOrCall(std::move(functor), TYPE, ARGS)

#define MAKE_NAMED_TERNARY(NAME, TYPE, PREFIX, ARGS...) \
struct __##NAME { \
    inline PabloAST * operator()(PabloAST * arg1, PabloAST * arg2, PabloAST * arg3) { \
        return mPb->NAME(arg1, arg2, arg3, mPrefix); \
    } \
    inline __##NAME(PabloBlock * pb, const std::string & prefix) : mPb(pb), mPrefix(prefix) {} \
private: \
    PabloBlock * mPb; \
    const std::string & mPrefix; \
}; \
__##NAME functor(mPb, PREFIX); \
PabloAST * result = mExprTable.findTernaryOrCall(std::move(functor), TYPE, ARGS)

#define MAKE_VARIABLE(NAME, TYPE, ARGS...) \
struct __##NAME { \
    inline PabloAST * operator()(const std::vector<PabloAST *> & args, PabloAST * prototype) { \
        return mPb->NAME(prototype, args); \
    } \
    inline __##NAME(PabloBlock * pb) : mPb(pb) {} \
private: \
    PabloBlock * mPb; \
}; \
__##NAME functor(mPb); \
PabloAST * result = mExprTable.findVariadicOrCall(std::move(functor), TYPE, ARGS)

template<typename Type>
static inline Type * isBinary(PabloAST * expr) {
    if (isa<Type>(expr) && cast<Type>(expr)->getNumOperands() == 2) {
        return cast<Type>(expr);
    }
    return nullptr;
}

using TypeId = PabloAST::ClassTypeId;

PabloAST * PabloBuilder::createAdvance(PabloAST * expr, PabloAST * shiftAmount) {
    if (isa<Zeroes>(expr) || cast<Integer>(shiftAmount)->value() == 0) {
        return expr;
    }
    MAKE_BINARY(createAdvance, TypeId::Advance, expr, shiftAmount);
    return result;
}

PabloAST * PabloBuilder::createAdvance(PabloAST * expr, PabloAST * shiftAmount, const std::string & prefix) {
    if (isa<Zeroes>(expr) || cast<Integer>(shiftAmount)->value() == 0) {
        return expr;
    }
    MAKE_NAMED_BINARY(createAdvance, TypeId::Advance, prefix, expr, shiftAmount);
    return result;
}

Extract * PabloBuilder::createExtract(PabloAST * value, not_null<PabloAST *> index) {
    MAKE_BINARY(createExtract, TypeId::Extract, value, index);
    return cast<Extract>(result);
}

Extract * PabloBuilder::createExtract(PabloAST * value, not_null<PabloAST *> index, const std::string & prefix) {
    MAKE_NAMED_BINARY(createExtract, TypeId::Extract, prefix, value, index);
    return cast<Extract>(result);
}

PabloAST * PabloBuilder::createLookahead(PabloAST * expr, PabloAST * shiftAmount) {
    if (isa<Zeroes>(expr) || cast<Integer>(shiftAmount)->value() == 0) {
        return expr;
    }
    MAKE_BINARY(createLookahead, TypeId::Lookahead, expr, shiftAmount);
    return result;
}

PabloAST * PabloBuilder::createLookahead(PabloAST * expr, PabloAST * shiftAmount, const std::string & prefix) {
    if (isa<Zeroes>(expr) || cast<Integer>(shiftAmount)->value() == 0) {
        return expr;
    }
    MAKE_NAMED_BINARY(createLookahead, TypeId::Lookahead, prefix, expr, shiftAmount);
    return result;
}

PabloAST * PabloBuilder::createNot(PabloAST * expr) {
    if (isa<Ones>(expr)) {
        return createZeroes(expr->getType());
    }
    else if (isa<Zeroes>(expr)){
        return createOnes(expr->getType());
    }
    else if (Not * not1 = dyn_cast<Not>(expr)) {
        return not1->getOperand(0);
    }
    MAKE_UNARY(createNot, TypeId::Not, expr);
    return result;
}

PabloAST * PabloBuilder::createNot(PabloAST * expr, const std::string & prefix) {
    if (isa<Ones>(expr)) {
        return createZeroes(expr->getType());
    }
    else if (isa<Zeroes>(expr)){
        return createOnes(expr->getType());
    }
    else if (Not * not1 = dyn_cast<Not>(expr)) {
        return not1->getOperand(0);
    }
    MAKE_NAMED_UNARY(createNot, TypeId::Not, prefix, expr);
    return result;
}

PabloAST * PabloBuilder::createCount(PabloAST * expr) {
    MAKE_UNARY(createCount, TypeId::Count, expr);
    return result;
}

PabloAST * PabloBuilder::createCount(PabloAST * expr, const std::string & prefix) {
    MAKE_NAMED_UNARY(createCount, TypeId::Count, prefix, expr);
    return result;
}

PabloAST * PabloBuilder::createAssign(PabloAST * const variable, PabloAST * const value) {
    return mPb->createAssign(variable, value);
}

PabloAST * PabloBuilder::createAnd(PabloAST * expr1, PabloAST * expr2) {
    if (isa<Zeroes>(expr2) || isa<Ones>(expr1)) {
        return expr2;
    } else if (isa<Zeroes>(expr1) || isa<Ones>(expr2) || equals(expr1, expr2)){
        return expr1;
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return createNot(createOr(not1->getOperand(0), not2->getOperand(0)));
        } else if (equals(not1->getOperand(0), expr2)) {
            return createZeroes(expr1->getType());
        }
    } else if (Not * not2 = dyn_cast<Not>(expr2)) {
        if (equals(expr1, not2->getOperand(0))) {
            return createZeroes(expr1->getType());
        }
    } else if (Or * or1 = isBinary<Or>(expr1)) {
        if (equals(or1->getOperand(0), expr2) || equals(or1->getOperand(1), expr2)) {
            return expr2;
        }
    } else if (Or * or2 = isBinary<Or>(expr2)) {
        if (equals(or2->getOperand(0), expr1) || equals(or2->getOperand(1), expr1)) {
            return expr1;
        }
    }
    if (expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    MAKE_BINARY(createAnd, TypeId::And, expr1, expr2);
    return result;
}

PabloAST * PabloBuilder::createAnd(PabloAST * expr1, PabloAST * expr2, const std::string & prefix) {
    if (isa<Zeroes>(expr2) || isa<Ones>(expr1)) {
        return expr2;
    } else if (isa<Zeroes>(expr1) || isa<Ones>(expr2) || equals(expr1, expr2)){
        return expr1;
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return createNot(createOr(not1->getOperand(0), not2->getOperand(0)), prefix);
        } else if (equals(not1->getOperand(0), expr2)) {
            return createZeroes(expr1->getType());
        }
    } else if (Not * not2 = dyn_cast<Not>(expr2)) {
        if (equals(expr1, not2->getOperand(0))) {
            return createZeroes(expr1->getType());
        }
    } else if (Or * or1 = isBinary<Or>(expr1)) {
        if (equals(or1->getOperand(0), expr2) || equals(or1->getOperand(1), expr2)) {
            return expr2;
        }
    } else if (Or * or2 = isBinary<Or>(expr2)) {
        if (equals(or2->getOperand(0), expr1) || equals(or2->getOperand(1), expr1)) {
            return expr1;
        }
    }
    if (expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    MAKE_NAMED_BINARY(createAnd, TypeId::And, prefix, expr1, expr2);
    return result;
}

PabloAST * PabloBuilder::createOr(PabloAST * expr1, PabloAST * expr2) {
    if (isa<Zeroes>(expr1) || isa<Ones>(expr2)){
        return expr2;
    }
    if (isa<Zeroes>(expr2) || isa<Ones>(expr1) || equals(expr1, expr2)) {
        return expr1;
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        // ¬a∨b = ¬¬(¬a ∨ b) = ¬(a ∧ ¬b)
        return createNot(createAnd(not1->getOperand(0), createNot(expr2)));
    } else if (Not * not2 = dyn_cast<Not>(expr2)) {
        // a∨¬b = ¬¬(¬b ∨ a) = ¬(b ∧ ¬a)
        return createNot(createAnd(not2->getOperand(0), createNot(expr1)));
    } else if (equals(expr1, expr2)) {
        return expr1;
    } else if (And * and1 = isBinary<And>(expr1)) {
        PabloAST * const expr1a = and1->getOperand(0);
        PabloAST * const expr1b = and1->getOperand(1);
        if (And * and2 = isBinary<And>(expr2)) {
            PabloAST * const expr2a = and2->getOperand(0);
            PabloAST * const expr2b = and2->getOperand(1);
            //These optimizations factor out common components that can occur when sets are formed by union
            //(e.g., union of [a-z] and [A-Z].
            if (equals(expr1a, expr2a)) {
                return createAnd(expr1a, createOr(expr1b, expr2b));
            } else if (equals(expr1b, expr2b)) {
                return createAnd(expr1b, createOr(expr1a, expr2a));
            } else if (equals(expr1a, expr2b)) {
                return createAnd(expr1a, createOr(expr1b, expr2a));
            } else if (equals(expr1b, expr2a)) {
                return createAnd(expr1b, createOr(expr1a, expr2b));
            }
        } else if (equals(expr1a, expr2) || equals(expr1b, expr2)) {
            // (a ∧ b) ∨ a = a
            return expr2;
        }
    } else if (And * and2 = isBinary<And>(expr2)) {
        if (equals(and2->getOperand(0), expr1) || equals(and2->getOperand(1), expr1)) {
            return expr1;
        }
    }
    if (expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    MAKE_BINARY(createOr, TypeId::Or, expr1, expr2);
    return result;
}

PabloAST * PabloBuilder::createOr(PabloAST * expr1, PabloAST * expr2, const std::string & prefix) {
    if (isa<Zeroes>(expr1) || isa<Ones>(expr2)){
        return expr2;
    }
    if (isa<Zeroes>(expr2) || isa<Ones>(expr1) || equals(expr1, expr2)) {
        return expr1;
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        // ¬a∨b = ¬¬(¬a ∨ b) = ¬(a ∧ ¬b)
        return createNot(createAnd(not1->getOperand(0), createNot(expr2)), prefix);
    } else if (Not * not2 = dyn_cast<Not>(expr2)) {
        // a∨¬b = ¬¬(¬b ∨ a) = ¬(b ∧ ¬a)
        return createNot(createAnd(not2->getOperand(0), createNot(expr1)), prefix);
    } else if (equals(expr1, expr2)) {
        return expr1;
    } else if (And * and1 = isBinary<And>(expr1)) {
        PabloAST * const expr1a = and1->getOperand(0);
        PabloAST * const expr1b = and1->getOperand(1);
        if (And * and2 = isBinary<And>(expr2)) {
            PabloAST * const expr2a = and2->getOperand(0);
            PabloAST * const expr2b = and2->getOperand(1);
            //These optimizations factor out common components that can occur when sets are formed by union
            //(e.g., union of [a-z] and [A-Z].
            if (equals(expr1a, expr2a)) {
                return createAnd(expr1a, createOr(expr1b, expr2b), prefix);
            } else if (equals(expr1b, expr2b)) {
                return createAnd(expr1b, createOr(expr1a, expr2a), prefix);
            } else if (equals(expr1a, expr2b)) {
                return createAnd(expr1a, createOr(expr1b, expr2a), prefix);
            } else if (equals(expr1b, expr2a)) {
                return createAnd(expr1b, createOr(expr1a, expr2b), prefix);
            }
        } else if (equals(expr1a, expr2) || equals(expr1b, expr2)) {
            // (a ∧ b) ∨ a = a
            return expr2;
        }
    } else if (And * and2 = isBinary<And>(expr2)) {
        if (equals(and2->getOperand(0), expr1) || equals(and2->getOperand(1), expr1)) {
            return expr1;
        }
    }
    if (expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    MAKE_NAMED_BINARY(createOr, TypeId::Or, prefix, expr1, expr2);
    return result;
}

PabloAST * PabloBuilder::createXor(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 == expr2) {
        return createZeroes(expr1->getType());
    } else if (isa<Ones>(expr1)) {
        return createNot(expr2);
    } else if (isa<Zeroes>(expr1)){
        return expr2;
    } else if (isa<Ones>(expr2)) {
        return createNot(expr1);
    } else if (isa<Zeroes>(expr2)){
        return expr1;
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return createXor(not1->getOperand(0), not2->getOperand(0));
        }
    }
    if (expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    MAKE_BINARY(createXor, TypeId::Xor, expr1, expr2);
    return result;
}

PabloAST * PabloBuilder::createXor(PabloAST * expr1, PabloAST * expr2, const std::string & prefix) {
    if (expr1 == expr2) {
        return createZeroes(expr1->getType());
    } else if (isa<Ones>(expr1)) {
        return createNot(expr2);
    } else if (isa<Zeroes>(expr1)){
        return expr2;
    } else if (isa<Ones>(expr2)) {
        return createNot(expr1);
    } else if (isa<Zeroes>(expr2)){
        return expr1;
    } else if (Not * not1 = dyn_cast<Not>(expr1)) {
        if (Not * not2 = dyn_cast<Not>(expr2)) {
            return createXor(not1->getOperand(0), not2->getOperand(0), prefix);
        }
    }
    if (expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    MAKE_NAMED_BINARY(createXor, TypeId::Xor, prefix, expr1, expr2);
    return result;
}

PabloAST * PabloBuilder::createAdd(PabloAST * expr1, PabloAST * expr2) {
    if (isa<Integer>(expr1) && isa<Integer>(expr2)) {
        return getInteger(cast<Integer>(expr1)->value() + cast<Integer>(expr2)->value());
    } else if (isa<Integer>(expr1)) {
        if (cast<Integer>(expr1)->value() == 0) {
            return expr2;
        }
    } else if (isa<Integer>(expr2)) {
        if (cast<Integer>(expr2)->value() == 0) {
            return expr1;
        }
    }
    MAKE_BINARY(createAdd, TypeId::Add, expr1, expr2);
    return result;
}

PabloAST * PabloBuilder::createSubtract(PabloAST * expr1, PabloAST * expr2) {
    if (isa<Integer>(expr1) && isa<Integer>(expr2)) {
        return getInteger(cast<Integer>(expr1)->value() - cast<Integer>(expr2)->value());
    } else if (isa<Integer>(expr1)) {
        if (cast<Integer>(expr1)->value() == 0) {
            return expr2;
        }
    } else if (isa<Integer>(expr2)) {
        if (cast<Integer>(expr2)->value() == 0) {
            return expr1;
        }
    }
    MAKE_BINARY(createSubtract, TypeId::Subtract, expr1, expr2);
    return result;
}

PabloAST * PabloBuilder::createLessThan(PabloAST * expr1, PabloAST * expr2) {
    if (isa<Integer>(expr1) && isa<Integer>(expr2)) {
        return getInteger(cast<Integer>(expr1)->value() < cast<Integer>(expr2)->value() ? 1 : 0);
    }
    MAKE_BINARY(createLessThan, TypeId::LessThan, expr1, expr2);
    return result;
}

PabloAST * PabloBuilder::createInFile(PabloAST * expr) {
    MAKE_UNARY(createInFile, TypeId::InFile, expr);
    return result;
}

PabloAST * PabloBuilder::createInFile(PabloAST * expr, const std::string & prefix) {
    MAKE_NAMED_UNARY(createInFile, TypeId::InFile, prefix, expr);
    return result;
}

PabloAST * PabloBuilder::createAtEOF(PabloAST * expr) {
    MAKE_UNARY(createAtEOF, TypeId::AtEOF, expr);
    return result;
}

PabloAST * PabloBuilder::createAtEOF(PabloAST * expr, const std::string & prefix) {
    MAKE_NAMED_UNARY(createAtEOF, TypeId::AtEOF, prefix, expr);
    return result;
}

PabloAST * PabloBuilder::createMatchStar(PabloAST * marker, PabloAST * charclass) {
    if (isa<Zeroes>(marker) || isa<Zeroes>(charclass)) {
        return marker;
    }
    MAKE_BINARY(createMatchStar, TypeId::MatchStar, marker, charclass);
    return result;
}

PabloAST * PabloBuilder::createMatchStar(PabloAST * marker, PabloAST * charclass, const std::string & prefix) {
    if (isa<Zeroes>(marker) || isa<Zeroes>(charclass)) {
        return marker;
    }
    MAKE_NAMED_BINARY(createMatchStar, TypeId::MatchStar, prefix, marker, charclass);
    return result;
}

PabloAST * PabloBuilder::createScanThru(PabloAST * from, PabloAST * thru) {
    if (isa<Zeroes>(from) || isa<Zeroes>(thru)) {
        return from;
    }
    MAKE_BINARY(createScanThru, TypeId::ScanThru, from, thru);
    return result;
}

PabloAST * PabloBuilder::createScanThru(PabloAST * from, PabloAST * thru, const std::string & prefix) {
    if (isa<Zeroes>(from) || isa<Zeroes>(thru)) {
        return from;
    }
    MAKE_NAMED_BINARY(createScanThru, TypeId::ScanThru, prefix, from, thru);
    return result;
}


PabloAST * PabloBuilder::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr) {
    if (isa<Ones>(condition)) {
        return trueExpr;
    } else if (isa<Zeroes>(condition)){
        return falseExpr;
    } else if (isa<Ones>(trueExpr)) {
        return createOr(condition, falseExpr);
    } else if (isa<Zeroes>(trueExpr)){
        return createAnd(createNot(condition), falseExpr);
    } else if (isa<Ones>(falseExpr)) {
        return createOr(createNot(condition), trueExpr);
    } else if (isa<Zeroes>(falseExpr)){
        return createAnd(condition, trueExpr);
    } else if (equals(trueExpr, falseExpr)) {
        return trueExpr;
    } else if (isa<Not>(trueExpr) && equals(cast<Not>(trueExpr)->getOperand(0), falseExpr)) {
        return createXor(condition, falseExpr);
    } else if (isa<Not>(falseExpr) && equals(trueExpr, cast<Not>(falseExpr)->getOperand(0))){
        return createXor(condition, trueExpr);
    }
    MAKE_TERNARY(createSel, TypeId::Sel, condition, trueExpr, falseExpr);
    return result;
}

PabloAST * PabloBuilder::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, const std::string & prefix) {
    if (isa<Ones>(condition)) {
        return trueExpr;
    } else if (isa<Zeroes>(condition)){
        return falseExpr;
    } else if (isa<Ones>(trueExpr)) {
        return createOr(condition, falseExpr);
    } else if (isa<Zeroes>(trueExpr)){
        return createAnd(createNot(condition), falseExpr);
    } else if (isa<Ones>(falseExpr)) {
        return createOr(createNot(condition), trueExpr);
    } else if (isa<Zeroes>(falseExpr)){
        return createAnd(condition, trueExpr);
    } else if (equals(trueExpr, falseExpr)) {
        return trueExpr;
    } else if (isa<Not>(trueExpr) && equals(cast<Not>(trueExpr)->getOperand(0), falseExpr)) {
        return createXor(condition, falseExpr);
    } else if (isa<Not>(falseExpr) && equals(trueExpr, cast<Not>(falseExpr)->getOperand(0))){
        return createXor(condition, trueExpr);
    }
    MAKE_NAMED_TERNARY(createSel, TypeId::Sel, prefix, condition, trueExpr, falseExpr);
    return result;
}

}
