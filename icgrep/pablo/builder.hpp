#ifndef BUILDER_HPP
#define BUILDER_HPP

#include <pablo/codegenstate.h>
#include <pablo/expression_map.hpp>
#include <pablo/pe_var.h>

namespace pablo {

class PabloBuilder {
public:

    template<typename T>
    struct not_null {
        not_null(T const value) : _value(value) { assert(_value); }
        not_null(std::nullptr_t) = delete;
        not_null(unsigned) = delete;
        operator T() const { return _value; }
        T operator-> () const { return _value; }
        T get() const { return _value; }
    private:
        T const  _value;
    };

    explicit PabloBuilder(PabloBlock * block)
    : mPb(block), mParent(nullptr), mExprTable(nullptr) {

    }

    PabloBuilder & operator=(PabloBuilder) = delete;

    PabloBuilder & operator=(PabloBuilder &) = delete;

    PabloBuilder(PabloBuilder && builder)
    : mPb(builder.mPb)
    , mParent(builder.mParent)
    , mExprTable(std::move(builder.mExprTable)) {

    }

    PabloBuilder & operator=(PabloBuilder && builder) {
        mPb = builder.mPb;
        mParent = builder.mParent;
        mExprTable = std::move(builder.mExprTable);
        return *this;
    }

    using iterator = PabloBlock::iterator;

    using const_iterator = PabloBlock::const_iterator;

    static PabloBuilder Create(PabloBlock * block) noexcept {
        return PabloBuilder(block);
    }

    static PabloBuilder Create(PabloBuilder & builder) noexcept {
        return PabloBuilder(PabloBlock::Create(builder.getPabloBlock()->getParent()), builder);
    }

    Zeroes * createZeroes(llvm::Type * const type = nullptr) {
        return mPb->createZeroes(type);
    }

    Ones * createOnes(llvm::Type * const type = nullptr) {
        return mPb->createOnes(type);
    }

    Var * createVar(const llvm::StringRef & name, llvm::Type * const type = nullptr) {
        return createVar(makeName(name), type);
    }

    Var * createVar(const llvm::StringRef & name, PabloAST * value) {
        Var * const var = createVar(name, value->getType());
        createAssign(var, value);
        return var;
    }

    Var * createVar(String * const name, llvm::Type * const type = nullptr) {
        return mPb->createVar(name, type);
    }

    Extract * createExtract(Var * const array, not_null<Integer *> index);

    Extract * createExtract(Var * const array, const int64_t index) {
        return createExtract(array, getInteger(index));
    }

    Var * createExtract(Var * const array, not_null<Integer *> index, const llvm::StringRef & name) {
        return createVar(name, createExtract(array, index));
    }

    Var * createExtract(Var * const array, const int64_t index, const llvm::StringRef & name) {
        return createVar(name, createExtract(array, index));
    }

    PabloAST * createAdvance(PabloAST * expr, const int64_t shiftAmount) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createAdvance(expr, mPb->getInteger(shiftAmount));
    }

    PabloAST * createAdvance(PabloAST * expr, not_null<Integer *> shiftAmount);

    PabloAST * createAdvance(PabloAST * expr, const int64_t shiftAmount, const llvm::StringRef & prefix) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createAdvance(expr, mPb->getInteger(shiftAmount), prefix);
    }

    PabloAST * createAdvance(PabloAST * expr, not_null<Integer *> shiftAmount, const llvm::StringRef & prefix);

    PabloAST * createIndexedAdvance(PabloAST * expr, PabloAST * indexStream, const int64_t shiftAmount) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createIndexedAdvance(expr, indexStream, mPb->getInteger(shiftAmount));
    }
    
    PabloAST * createIndexedAdvance(PabloAST * expr, PabloAST * indexStream, not_null<Integer *> shiftAmount);
    
    PabloAST * createIndexedAdvance(PabloAST * expr, PabloAST * indexStream, const int64_t shiftAmount, const llvm::StringRef & prefix) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createIndexedAdvance(expr, indexStream, mPb->getInteger(shiftAmount), prefix);
    }
    
    PabloAST * createIndexedAdvance(PabloAST * expr, PabloAST * indexStream, not_null<Integer *> shiftAmount, const llvm::StringRef & prefix);
    
    PabloAST * createLookahead(PabloAST * expr, const int64_t shiftAmount) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createLookahead(expr, mPb->getInteger(shiftAmount));
    }

    PabloAST * createLookahead(PabloAST * expr, not_null<Integer *> shiftAmount);

    PabloAST * createLookahead(PabloAST * expr, const int64_t shiftAmount, const llvm::StringRef & prefix) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createLookahead(expr, mPb->getInteger(shiftAmount), prefix);
    }

    PabloAST * createLookahead(PabloAST * expr, not_null<Integer *> shiftAmount, const llvm::StringRef & prefix);

    Assign * createAssign(PabloAST * const variable, PabloAST * const value){
        return mPb->createAssign(variable, value);
    }

    PabloAST * createAnd(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createAnd(PabloAST * expr1, PabloAST * expr2, const llvm::StringRef & prefix);

    PabloAST * createNot(PabloAST * expr);

    PabloAST * createNot(PabloAST * expr, const llvm::StringRef & prefix);

    PabloAST * createRepeat(const int64_t fieldWidth, const int64_t value) {
        return createRepeat(mPb->getInteger(fieldWidth), mPb->getInteger(value));
    }

    PabloAST * createRepeat(const int64_t fieldWidth, PabloAST * value) {
        return createRepeat(mPb->getInteger(fieldWidth), value);
    }

    PabloAST * createRepeat(not_null<Integer *> fieldWidth, PabloAST * value);

    PabloAST * createRepeat(const int64_t fieldWidth, PabloAST * value, const llvm::StringRef & prefix) {
        return createRepeat(mPb->getInteger(fieldWidth), value, prefix);
    }

    PabloAST * createRepeat(const int64_t fieldWidth, const int64_t value, const llvm::StringRef & prefix) {
        return createRepeat(mPb->getInteger(fieldWidth), mPb->getInteger(value), prefix);
    }

    PabloAST * createRepeat(not_null<Integer *> fieldWidth, PabloAST * value, const llvm::StringRef & prefix);

    PabloAST * createOr(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createOr(PabloAST * expr1, PabloAST * expr2, const llvm::StringRef & prefix);

    PabloAST * createXor(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createXor(PabloAST * expr1, PabloAST * expr2, const llvm::StringRef & prefix);

    PabloAST * createMatchStar(PabloAST * marker, PabloAST * charclass);

    PabloAST * createMatchStar(PabloAST * marker, PabloAST * charclass, const llvm::StringRef & prefix);

    PabloAST * createScanThru(PabloAST * from, PabloAST * thru);

    PabloAST * createScanThru(PabloAST * from, PabloAST * thru, const llvm::StringRef & prefix);

    PabloAST * createScanTo(PabloAST * from, PabloAST * to);

    PabloAST * createScanTo(PabloAST * from, PabloAST * to, const llvm::StringRef & prefix);

    PabloAST * createAdvanceThenScanThru(PabloAST * from, PabloAST * thru);

    PabloAST * createAdvanceThenScanThru(PabloAST * from, PabloAST * thru, const llvm::StringRef & prefix);

    PabloAST * createAdvanceThenScanTo(PabloAST * from, PabloAST * to);

    PabloAST * createAdvanceThenScanTo(PabloAST * from, PabloAST * to, const llvm::StringRef & prefix);

    PabloAST * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr);

    PabloAST * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, const llvm::StringRef & prefix);

    PabloAST * createCount(PabloAST * expr);
    
    PabloAST * createCount(PabloAST * expr, const llvm::StringRef & prefix);

    PabloAST * createInFile(PabloAST * expr);
    
    PabloAST * createInFile(PabloAST * expr, const llvm::StringRef & prefix);
    
    PabloAST * createAtEOF(PabloAST * expr);
    
    PabloAST * createAtEOF(PabloAST * expr, const llvm::StringRef & prefix);
    
    PabloAST * createAdd(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createSubtract(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createLessThan(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createEquals(PabloAST * expr1, PabloAST * expr2);

    If * createIf(PabloAST * condition, PabloBlock * body) {
        return mPb->createIf(condition, body);
    }

    If * createIf(PabloAST * condition, PabloBuilder & builder) {
        return mPb->createIf(condition, builder.mPb);
    }

    While * createWhile(PabloAST * condition, PabloBlock * body) {
        return mPb->createWhile(condition, body);
    }

    While * createWhile(PabloAST * condition, PabloBuilder & builder) {
        return mPb->createWhile(condition, builder.mPb);
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

    Statement * front() const {
        return mPb->front();
    }

    Statement * back() const {
        return mPb->back();
    }

    Statement * getInsertPoint() const {
        return mPb->getInsertPoint();
    }

    PabloBlock * getPabloBlock() const {
        return mPb;
    }

    PabloBuilder * getParent() const {
        return mParent;
    }

    String * makeName(const llvm::StringRef & prefix) const {
        return mPb->makeName(prefix);
    }

    Integer * getInteger(const uint64_t value) const {
        return mPb->getInteger(value);
    }

    void print(llvm::raw_ostream & O, const bool expandNested = true) const {
        mPb->print(O, expandNested);
    }

protected:

    explicit PabloBuilder(PabloBlock * block, PabloBuilder & parent)
    : mPb(block), mParent(&parent), mExprTable(&(parent.mExprTable)) {

    }

private:

    PabloBlock *        mPb;
    PabloBuilder *      mParent;
    ExpressionTable     mExprTable;
};


}


#endif // BUILDER_HPP
