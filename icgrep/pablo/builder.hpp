#ifndef BUILDER_HPP
#define BUILDER_HPP

#include <pablo/codegenstate.h>
#include <pablo/expression_map.hpp>

namespace pablo {

class PabloBuilder {
public:

    explicit PabloBuilder(PabloBlock * block) : mPb(block), mParent(nullptr), mExprTable(nullptr) {}

    explicit PabloBuilder(PabloBlock * block, PabloBuilder & parent) : mPb(block), mParent(&parent), mExprTable(&(parent.mExprTable)) {}

    PabloBuilder(PabloBuilder && builder) : mPb(builder.mPb), mParent(builder.mParent), mExprTable(std::move(builder.mExprTable)) {}

    PabloBuilder & operator=(PabloBuilder) = delete;

    PabloBuilder & operator=(PabloBuilder &) = delete;

    PabloBuilder & operator=(PabloBuilder && builder) {
        mPb = builder.mPb;
        mParent = builder.mParent;
        mExprTable = std::move(builder.mExprTable);
        return *this;
    }

    using iterator = PabloBlock::iterator;

    using const_iterator = PabloBlock::const_iterator;

    inline static PabloBuilder Create(PabloBlock * block) noexcept {
        return std::move(PabloBuilder(block));
    }

    inline static PabloBuilder Create(PabloBuilder & builder) noexcept {
        return std::move(PabloBuilder(new PabloBlock(builder.mPb->mSymbolGenerator), builder));
    }

    static inline Zeroes * createZeroes() {
        return PabloBlock::createZeroes();
    }

    static inline Ones * createOnes() {
        return PabloBlock::createOnes();
    }

    inline Var * createVar(const std::string name) {
        return mPb->createVar(name);
    }

    inline Var * createVar(String * const name) {
        return mPb->createVar(name);
    }

    inline Var * createVar(PabloAST * const name) {
        return mPb->createVar(name);
    }

    inline Call * createCall(Prototype * prototype, const std::vector<Var *> & args) {
        return createCall(prototype, reinterpret_cast<const std::vector<PabloAST *> &>(args));
    }

    Call * createCall(Prototype * prototype, const std::vector<PabloAST *> &vars);

    Assign * createAssign(const std::string && prefix, PabloAST * expr) {
        return mPb->createAssign(std::move(prefix), expr);
    }

    inline PabloAST * createAdvance(PabloAST * expr, const Integer::Type shiftAmount) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createAdvance(expr, mPb->getInteger(shiftAmount));
    }

    PabloAST * createAdvance(PabloAST * expr, PabloAST * shiftAmount);

    inline PabloAST * createAdvance(PabloAST * expr, const Integer::Type shiftAmount, const std::string prefix) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createAdvance(expr, mPb->getInteger(shiftAmount), prefix);
    }

    PabloAST * createAdvance(PabloAST * expr, PabloAST * shiftAmount, const std::string prefix);

    inline PabloAST * createLookahead(PabloAST * expr, const Integer::Type shiftAmount) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createLookahead(expr, mPb->getInteger(shiftAmount));
    }

    PabloAST * createLookahead(PabloAST * expr, PabloAST * shiftAmount);

    inline PabloAST * createLookahead(PabloAST * expr, const Integer::Type shiftAmount, const std::string prefix) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createLookahead(expr, mPb->getInteger(shiftAmount), prefix);
    }

    PabloAST * createLookahead(PabloAST * expr, PabloAST * shiftAmount, const std::string prefix);

    inline Next * createNext(Assign * assign, PabloAST * expr) {
        return mPb->createNext(assign, expr);
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
    
    PabloAST * createCount(PabloAST * expr);
    
    PabloAST * createCount(PabloAST * expr, const std::string prefix);

    /// CreateIf Wrappers

    inline If * createIf(PabloAST * condition, std::initializer_list<Assign *> definedVars, PabloBlock * body) {
        return mPb->createIf(condition, std::move(definedVars), body);
    }

    inline If * createIf(PabloAST * condition, const std::vector<Assign *> & definedVars, PabloBlock * body) {
        return mPb->createIf(condition, definedVars, body);
    }

    inline If * createIf(PabloAST * condition, std::vector<Assign *> && definedVars, PabloBlock * body) {
        return mPb->createIf(condition, std::move(definedVars), body);
    }

    inline If * createIf(PabloAST * condition, std::initializer_list<Assign *> definedVars, PabloBuilder & builder) {
        return mPb->createIf(condition, std::move(definedVars), builder.mPb);
    }

    inline If * createIf(PabloAST * condition, const std::vector<Assign *> & definedVars, PabloBuilder & builder) {
        return mPb->createIf(condition, definedVars, builder.mPb);
    }

    inline If * createIf(PabloAST * condition, std::vector<Assign *> && definedVars, PabloBuilder & builder) {
        return mPb->createIf(condition, std::move(definedVars), builder.mPb);
    }

    /// CreateWhile Wrappers

    inline While * createWhile(PabloAST * condition, const std::initializer_list<Next *> nextVars, PabloBlock * body) {
        return mPb->createWhile(condition, nextVars, body);
    }

    inline While * createWhile(PabloAST * condition, const std::vector<Next *> & nextVars, PabloBlock * body) {
        return mPb->createWhile(condition, nextVars, body);
    }

    inline While * createWhile(PabloAST * condition, std::vector<Next *> && nextVars, PabloBlock * body) {
        return mPb->createWhile(condition, std::move(nextVars), body);
    }

    inline While * createWhile(PabloAST * condition, const std::initializer_list<Next *> nextVars, PabloBuilder & builder) {
        return mPb->createWhile(condition, std::move(nextVars), builder.mPb);
    }

    inline While * createWhile(PabloAST * condition, const std::vector<Next *> & nextVars, PabloBuilder & builder) {
        return mPb->createWhile(condition, nextVars, builder.mPb);
    }

    inline While * createWhile(PabloAST * condition, std::vector<Next *> && nextVars, PabloBuilder & builder) {
        return mPb->createWhile(condition, std::move(nextVars), builder.mPb);
    }

    /// Statement Iterator Wrappers

    iterator begin() {
        return mPb->begin();
    }

    iterator end() {
        return mPb->end();
    }

    const_iterator begin() const {
        return mPb->cbegin();
    }

    const_iterator end() const {
        return mPb->cend();
    }

    const_iterator cbegin() const {
        return mPb->cbegin();
    }

    const_iterator cend() const {
        return mPb->cend();
    }

    inline Statement * front() const {
        return mPb->front();
    }

    inline Statement * back() const {
        return mPb->back();
    }

    inline String * getName(const std::string name, const bool generated = true) const {
        return mPb->getName(std::move(name), generated);
    }

    inline String * makeName(const std::string prefix, const bool generated = true) const {
        return mPb->makeName(std::move(prefix), generated);
    }

    inline Integer * getInteger(Integer::Type value) {
        return mPb->getInteger(value);
    }


    inline Statement * getInsertPoint() const {
        return mPb->getInsertPoint();
    }

    inline PabloBlock * getPabloBlock() {
        return mPb;
    }

    inline PabloBuilder * getParent() {
        return mParent;
    }

    inline void record(Statement * stmt) {
        mExprTable.findOrAdd(stmt);
    }

private:

    PabloBlock *        mPb;
    PabloBuilder *      mParent;
    ExpressionTable     mExprTable;
};


}


#endif // BUILDER_HPP
