#include "builder.hpp"

namespace pablo {

#define CALL_UNARY_FUNCTOR(NAME, TYPE, ARGS...) \
struct __##NAME { \
    inline PabloAST * operator()(PabloAST * expr) { \
        return mPB->NAME(expr); \
    } \
    inline __##NAME(PabloBlock * pb) : mPB(pb) {} \
private: \
    PabloBlock * mPB; \
}; \
__##NAME functor(this); \
mExprTable.findUnaryOrCall<__##NAME>(std::move(functor), TYPE, ARGS)


#define CALL_BINARY_FUNCTOR(NAME, TYPE, ARGS...) \
struct __##NAME { \
    inline PabloAST * operator()(PabloAST * expr) { \
        return mPB->NAME(expr); \
    } \
    inline __##NAME(PabloBlock * pb) : mPB(pb) {} \
private: \
    PabloBlock * mPB; \
}; \
__##NAME functor(this); \
mExprTable.findBinaryOrCall<__##NAME>(std::move(functor), TYPE, ARGS)

#define CALL_TERNARY_FUNCTOR(NAME, TYPE, ARGS...) \
struct __##NAME { \
    inline PabloAST * operator()(PabloAST * expr) { \
        return mPB->NAME(expr); \
    } \
    inline __##NAME(PabloBlock * pb) : mPB(pb) {} \
private: \
    PabloBlock * mPB; \
}; \
__##NAME functor(this); \
mExprTable.findTernaryOrCall<__##NAME>(std::move(functor), TYPE, ARGS)


Call * PabloBuilder::createCall(String * name) {
    return CALL_UNARY_FUNCTOR(createCall, PabloAST::ClassTypeId::Call, name);
}


Assign * PabloBuilder::createAssign(const std::string prefix, PabloAST * expr, const int outputIndex) {

}

PabloAST * PabloBuilder::createAdvance(PabloAST * expr, PabloAST * shiftAmount) {

}

PabloAST * PabloBuilder::createNot(PabloAST * expr) {
    return CALL_UNARY_FUNCTOR(createNot, PabloAST::ClassTypeId::Not, expr);
}

PabloAST * PabloBuilder::createAnd(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    return CALL_BINARY_FUNCTOR(createAnd, PabloAST::ClassTypeId::And, expr1, expr2);
}

PabloAST * PabloBuilder::createOr(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    return CALL_BINARY_FUNCTOR(createOr, PabloAST::ClassTypeId::Or, expr1, expr2);
}

PabloAST * PabloBuilder::createXor(PabloAST * expr1, PabloAST * expr2) {
    if (expr1 < expr2) {
        std::swap(expr1, expr2);
    }
    return CALL_BINARY_FUNCTOR(createXor, PabloAST::ClassTypeId::Xor, expr1, expr2);
}

PabloAST * PabloBuilder::createMatchStar(PabloAST * marker, PabloAST * charclass) {
    return CALL_BINARY_FUNCTOR(createMatchStar, PabloAST::ClassTypeId::MatchStar, marker, charclass);
}

PabloAST * PabloBuilder::createScanThru(PabloAST * from, PabloAST * thru) {
    return CALL_BINARY_FUNCTOR(createScanThru, PabloAST::ClassTypeId::ScanThru, from, thru);
}

PabloAST * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr) {
    return CALL_TERNARY_FUNCTOR(createSel, PabloAST::ClassTypeId::Sel, condition, trueExpr, falseExpr);
}

}
