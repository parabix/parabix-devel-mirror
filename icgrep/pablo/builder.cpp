#include "builder.hpp"

namespace pablo {

#define MAKE_UNARY(NAME, TYPE, ARGS...) \
struct __##NAME { \
    inline PabloAST * operator()(PabloAST * arg) { \
        return mPb.NAME(arg); \
    } \
    inline __##NAME(PabloBlock & pb) : mPb(pb) {} \
private: \
    PabloBlock & mPb; \
}; \
__##NAME functor(mPb); \
PabloAST * result = mExprTable.findUnaryOrCall(std::move(functor), TYPE, ARGS)


#define MAKE_BINARY(NAME, TYPE, ARGS...) \
struct __##NAME { \
    inline PabloAST * operator()(PabloAST * arg1, PabloAST * arg2) { \
        return mPb.NAME(arg1, arg2); \
    } \
    inline __##NAME(PabloBlock & pb) : mPb(pb) {} \
private: \
    PabloBlock & mPb; \
}; \
__##NAME functor(mPb); \
PabloAST * result = mExprTable.findBinaryOrCall(std::move(functor), TYPE, ARGS)

#define MAKE_TERNARY(NAME, TYPE, ARGS...) \
struct __##NAME { \
    inline PabloAST * operator()(PabloAST * arg1, PabloAST * arg2, PabloAST * arg3) { \
        return mPb.NAME(arg1, arg2, arg3); \
    } \
    inline __##NAME(PabloBlock & pb) : mPb(pb) {} \
private: \
    PabloBlock & mPb; \
}; \
__##NAME functor(mPb); \
PabloAST * result = mExprTable.findTernaryOrCall(std::move(functor), TYPE, ARGS)


Call * PabloBuilder::createCall(String * name) {
    MAKE_UNARY(createCall, PabloAST::ClassTypeId::Call, name);
    return cast<Call>(result);
}

PabloAST * PabloBuilder::createAdvance(PabloAST * expr, PabloAST * shiftAmount) {
    MAKE_BINARY(createAdvance, PabloAST::ClassTypeId::Advance, expr, shiftAmount);
    return result;
}

PabloAST * PabloBuilder::createNot(PabloAST * expr) {
    MAKE_UNARY(createNot, PabloAST::ClassTypeId::Not, expr);
    return result;
}

PabloAST * PabloBuilder::createAnd(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    MAKE_BINARY(createAnd, PabloAST::ClassTypeId::And, expr1, expr2);
    return result;
}

PabloAST * PabloBuilder::createOr(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    MAKE_BINARY(createOr, PabloAST::ClassTypeId::Or, expr1, expr2);
    return result;
}

PabloAST * PabloBuilder::createXor(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    MAKE_BINARY(createXor, PabloAST::ClassTypeId::Xor, expr1, expr2);
    return result;
}

PabloAST * PabloBuilder::createMatchStar(PabloAST * marker, PabloAST * charclass) {
    MAKE_BINARY(createMatchStar, PabloAST::ClassTypeId::MatchStar, marker, charclass);
    return result;
}

PabloAST * PabloBuilder::createScanThru(PabloAST * from, PabloAST * thru) {
    MAKE_BINARY(createScanThru, PabloAST::ClassTypeId::ScanThru, from, thru);
    return result;
}

PabloAST * PabloBuilder::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr) {
    MAKE_TERNARY(createSel, PabloAST::ClassTypeId::Sel, condition, trueExpr, falseExpr);
    return result;
}

}
