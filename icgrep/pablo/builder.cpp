#include <pablo/builder.hpp>

namespace pablo {

#define MAKE_UNARY(NAME, TYPE, ARGS...) \
struct __##NAME { \
    inline PabloAST * operator()(PabloAST * arg) { \
        return mPb->NAME(arg); \
    } \
    inline PabloAST * operator()(PabloAST * arg, const std::string name) { \
        return mPb->NAME(arg, name); \
    } \
    inline __##NAME(PabloBlock * pb) : mPb(pb) {} \
private: \
    PabloBlock * mPb; \
}; \
__##NAME functor(mPb); \
PabloAST * result = mExprTable.findUnaryOrCall(std::move(functor), TYPE, ARGS)

#define MAKE_BINARY(NAME, TYPE, ARGS...) \
struct __##NAME { \
    inline PabloAST * operator()(PabloAST * arg1, PabloAST * arg2) { \
        return mPb->NAME(arg1, arg2); \
    } \
    inline PabloAST * operator()(PabloAST * arg1, PabloAST * arg2, const std::string name) { \
        return mPb->NAME(arg1, arg2, name); \
    } \
    inline __##NAME(PabloBlock * pb) : mPb(pb) {} \
private: \
    PabloBlock * mPb; \
}; \
__##NAME functor(mPb); \
PabloAST * result = mExprTable.findBinaryOrCall(std::move(functor), TYPE, ARGS)


#define MAKE_TERNARY(NAME, TYPE, ARGS...) \
struct __##NAME { \
    inline PabloAST * operator()(PabloAST * arg1, PabloAST * arg2, PabloAST * arg3) { \
        return mPb->NAME(arg1, arg2, arg3); \
    } \
    inline PabloAST * operator()(PabloAST * arg1, PabloAST * arg2, PabloAST * arg3, const std::string name) { \
        return mPb->NAME(arg1, arg2, arg3, name); \
    } \
    inline __##NAME(PabloBlock * pb) : mPb(pb) {} \
private: \
    PabloBlock * mPb; \
}; \
__##NAME functor(mPb); \
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
PabloAST * result = mExprTable.findVariableOrCall(std::move(functor), TYPE, ARGS)

Call * PabloBuilder::createCall(Prototype * prototype, const std::vector<PabloAST *> & args) {
    if (prototype == nullptr) {
        throw std::runtime_error("Call object cannot be created with a Null prototype!");
    }
    if (args.size() != cast<Prototype>(prototype)->getNumOfParameters()) {
        throw std::runtime_error("Invalid number of arguments passed into Call object!");
    }
    MAKE_VARIABLE(createCall, PabloAST::ClassTypeId::Call, prototype->getName(), args, prototype);
    return cast<Call>(result);
}

PabloAST * PabloBuilder::createAdvance(PabloAST * expr, PabloAST * shiftAmount) {
    MAKE_BINARY(createAdvance, PabloAST::ClassTypeId::Advance, expr, shiftAmount);
    return result;
}

PabloAST * PabloBuilder::createAdvance(PabloAST * expr, PabloAST * shiftAmount, const std::string prefix) {
    MAKE_BINARY(createAdvance, PabloAST::ClassTypeId::Advance, expr, shiftAmount, prefix);
    return result;
}

PabloAST * PabloBuilder::createLookahead(PabloAST * expr, PabloAST * shiftAmount) {
    MAKE_BINARY(createLookahead, PabloAST::ClassTypeId::Lookahead, expr, shiftAmount);
    return result;
}

PabloAST * PabloBuilder::createLookahead(PabloAST * expr, PabloAST * shiftAmount, const std::string prefix) {
    MAKE_BINARY(createLookahead, PabloAST::ClassTypeId::Lookahead, expr, shiftAmount, prefix);
    return result;
}

PabloAST * PabloBuilder::createNot(PabloAST * expr) {
    MAKE_UNARY(createNot, PabloAST::ClassTypeId::Not, expr);
    return result;
}

PabloAST * PabloBuilder::createNot(PabloAST * expr, const std::string prefix) {
    MAKE_UNARY(createNot, PabloAST::ClassTypeId::Not, expr, prefix);
    return result;
}

PabloAST * PabloBuilder::createAnd(PabloAST * expr1, PabloAST * expr2) {
    assert (expr1);
    assert (expr2);
    if (isa<Not>(expr1) || expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    MAKE_BINARY(createAnd, PabloAST::ClassTypeId::And, expr1, expr2);
    return result;
}

PabloAST * PabloBuilder::createAnd(PabloAST * expr1, PabloAST * expr2, const std::string prefix) {
    assert (expr1);
    assert (expr2);
    if (isa<Not>(expr1) || expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    MAKE_BINARY(createAnd, PabloAST::ClassTypeId::And, expr1, expr2, prefix);
    return result;
}

PabloAST * PabloBuilder::createOr(PabloAST * expr1, PabloAST * expr2) {
    assert (expr1);
    assert (expr2);
    if (expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    MAKE_BINARY(createOr, PabloAST::ClassTypeId::Or, expr1, expr2);
    return result;
}

PabloAST * PabloBuilder::createOr(PabloAST * expr1, PabloAST * expr2, const std::string prefix) {
    assert (expr1);
    assert (expr2);
    if (expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    MAKE_BINARY(createOr, PabloAST::ClassTypeId::Or, expr1, expr2, prefix);
    return result;
}

PabloAST * PabloBuilder::createXor(PabloAST * expr1, PabloAST * expr2) {
    assert (expr1);
    assert (expr2);
    if (expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    MAKE_BINARY(createXor, PabloAST::ClassTypeId::Xor, expr1, expr2);
    return result;
}

PabloAST * PabloBuilder::createXor(PabloAST * expr1, PabloAST * expr2, const std::string prefix) {
    assert (expr1);
    assert (expr2);
    if (expr1 > expr2) {
        std::swap(expr1, expr2);
    }
    MAKE_BINARY(createXor, PabloAST::ClassTypeId::Xor, expr1, expr2, prefix);
    return result;
}

PabloAST * PabloBuilder::createMatchStar(PabloAST * marker, PabloAST * charclass) {
    MAKE_BINARY(createMatchStar, PabloAST::ClassTypeId::MatchStar, marker, charclass);
    return result;
}

PabloAST * PabloBuilder::createMatchStar(PabloAST * marker, PabloAST * charclass, const std::string prefix) {
    MAKE_BINARY(createMatchStar, PabloAST::ClassTypeId::MatchStar, marker, charclass, prefix);
    return result;
}

PabloAST * PabloBuilder::createScanThru(PabloAST * from, PabloAST * thru) {
    MAKE_BINARY(createScanThru, PabloAST::ClassTypeId::ScanThru, from, thru);
    return result;
}

PabloAST * PabloBuilder::createScanThru(PabloAST * from, PabloAST * thru, const std::string prefix) {
    MAKE_BINARY(createScanThru, PabloAST::ClassTypeId::ScanThru, from, thru, prefix);
    return result;
}


PabloAST * PabloBuilder::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr) {
    MAKE_TERNARY(createSel, PabloAST::ClassTypeId::Sel, condition, trueExpr, falseExpr);
    return result;
}

PabloAST * PabloBuilder::createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, const std::string prefix) {
    MAKE_TERNARY(createSel, PabloAST::ClassTypeId::Sel, condition, trueExpr, falseExpr, prefix);
    return result;
}

PabloAST * PabloBuilder::createCount(PabloAST * expr) {
    MAKE_UNARY(createCount, PabloAST::ClassTypeId::Count, expr);
    return result;
}

PabloAST * PabloBuilder::createCount(PabloAST * expr, const std::string prefix) {
    MAKE_UNARY(createCount, PabloAST::ClassTypeId::Count, expr, prefix);
    return result;
}

}
