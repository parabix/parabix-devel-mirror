#ifndef BUILDER_HPP
#define BUILDER_HPP

#include "codegenstate.h"
#include "expression_map.hpp"

namespace pablo {

class PabloBuilder {
public:

    PabloBuilder(PabloBlock & block) : mPb(block), mParent(nullptr) {}

    PabloBuilder(PabloBlock & block, PabloBuilder & parent) : mPb(block), mParent(&parent), mExprTable(&(parent.mExprTable)) {}

    inline static PabloBuilder && Create(PabloBuilder & parent) {
        return std::move(PabloBuilder(parent.mPb.Create(parent.mPb), parent));
    }

    inline Zeroes * createZeroes() const {
        return mPb.createZeroes();
    }

    inline Ones * createOnes() const {
        return mPb.createOnes();
    }

    inline Var * createVar(const std::string name) {
        return mPb.createVar(name);
    }

    inline Var * createVar(String * const name) {
        return mPb.createVar(name);
    }

    inline Var * createVar(PabloAST * const name) {
        return mPb.createVar(name);
    }

    inline Call * createCall(const std::string name) {
        return createCall(mPb.getName(name));
    }

    Call * createCall(String * name);

    Assign * createAssign(const std::string && prefix, PabloAST * expr, const int outputIndex = -1) {
        return mPb.createAssign(std::move(prefix), expr, outputIndex);
    }

    inline PabloAST * createAdvance(PabloAST * expr, const Integer::integer_t shiftAmount) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createAdvance(expr, mPb.getInteger(shiftAmount));
    }

    PabloAST * createAdvance(PabloAST * expr, PabloAST * shiftAmount);

    inline PabloAST * createAdvance(PabloAST * expr, const Integer::integer_t shiftAmount, const std::string prefix) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createAdvance(expr, mPb.getInteger(shiftAmount), prefix);
    }

    PabloAST * createAdvance(PabloAST * expr, PabloAST * shiftAmount, const std::string prefix);

    inline Next * createNext(Assign * assign, PabloAST * expr) {
        return mPb.createNext(assign, expr);
    }

    PabloAST * createAnd(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createAnd(PabloAST * expr1, PabloAST * expr2, const std::string prefix);

    PabloAST * createNot(PabloAST * expr);

    PabloAST * createNot(PabloAST * expr, const std::string prefix);

    PabloAST * createOr(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createOr(PabloAST * expr1, PabloAST * expr2, const std::string prefix);

    PabloAST * createXor(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createXor(PabloAST * expr1, PabloAST * expr2, const std::string prefix);

    PabloAST * createMatchStar(PabloAST * marker, PabloAST * charclass);

    PabloAST * createMatchStar(PabloAST * marker, PabloAST * charclass, const std::string prefix);

    PabloAST * createScanThru(PabloAST * from, PabloAST * thru);

    PabloAST * createScanThru(PabloAST * from, PabloAST * thru, const std::string prefix);

    PabloAST * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr);

    PabloAST * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, const std::string prefix);

    inline If * createIf(PabloAST * condition, std::initializer_list<Assign *> definedVars, PabloBuilder & body) {
        return mPb.createIf(condition, std::move(definedVars), body.mPb);
    }

    inline If * createIf(PabloAST * condition, std::initializer_list<Assign *> definedVars, PabloBlock & body) {
        return mPb.createIf(condition, std::move(definedVars), body);
    }

    inline If * createIf(PabloAST * condition, std::vector<Assign *> definedVars, PabloBuilder & body) {
        return mPb.createIf(condition, std::move(definedVars), body.mPb);
    }

    inline If * createIf(PabloAST * condition, std::vector<Assign *> definedVars, PabloBlock & body) {
        return mPb.createIf(condition, std::move(definedVars), body);
    }

    inline While * createWhile(PabloAST * condition, PabloBuilder & body) {
        return mPb.createWhile(condition, body.mPb);
    }

    inline While * createWhile(PabloAST * condition, PabloBlock & body) {
        return mPb.createWhile(condition, body);
    }

    inline Statement * front() const {
        return mPb.front();
    }

    inline Statement * back() const {
        return mPb.back();
    }

    inline void setInsertPoint(Statement * const statement) {
        mPb.setInsertPoint(statement);
    }

    inline void setInsertPoint(StatementList * const list) {
        mPb.setInsertPoint(list);
    }

    inline Statement * getInsertPoint() const {
        return mPb.getInsertPoint();
    }

    inline PabloBlock & getPabloBlock() {
        return mPb;
    }

    inline PabloBuilder * getParent() {
        return mParent;
    }

private:

    PabloBlock &        mPb;
    PabloBuilder *      mParent;
    ExpressionTable     mExprTable;
};


}


#endif // BUILDER_HPP
