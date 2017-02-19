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
        inline not_null(T const value) : _value(value) { assert(_value); }
        inline not_null(std::nullptr_t) = delete;
        inline not_null(unsigned) = delete;
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

    inline static PabloBuilder Create(PabloBlock * block) noexcept {
        return PabloBuilder(block);
    }

    inline static PabloBuilder Create(PabloBuilder & builder) noexcept {
        return PabloBuilder(PabloBlock::Create(builder.getPabloBlock()->getParent()), builder);
    }

    inline Zeroes * createZeroes(llvm::Type * const type = nullptr) {
        return mPb->createZeroes(type);
    }

    inline Ones * createOnes(llvm::Type * const type = nullptr) {
        return mPb->createOnes(type);
    }

    inline Var * createVar(const llvm::StringRef & name, llvm::Type * const type = nullptr) {
        return createVar(makeName(name), type);
    }

    inline Var * createVar(const llvm::StringRef & name, PabloAST * value) {
        Var * var = createVar(name, value->getType());
        createAssign(var, value);
        return var;
    }

    inline Var * createVar(String * const name, llvm::Type * const type = nullptr) {
        return mPb->createVar(name, type);
    }

    Extract * createExtract(PabloAST * value, not_null<PabloAST *> index);

    inline Extract * createExtract(PabloAST * value, const int64_t index) {
        return createExtract(value, getInteger(index));
    }

    Extract * createExtract(PabloAST * value, not_null<PabloAST *> index, const llvm::StringRef & prefix);

    inline Extract * createExtract(PabloAST * value, const int64_t index, const llvm::StringRef & prefix) {
        return createExtract(value, getInteger(index), prefix);
    }

    inline PabloAST * createAdvance(PabloAST * expr, const int64_t shiftAmount) {
        return createAdvance(expr, mPb->getInteger(shiftAmount));
    }

    PabloAST * createAdvance(PabloAST * expr, PabloAST * shiftAmount);

    inline PabloAST * createAdvance(PabloAST * expr, const int64_t shiftAmount, const llvm::StringRef & prefix) {
        return createAdvance(expr, mPb->getInteger(shiftAmount), prefix);
    }

    PabloAST * createAdvance(PabloAST * expr, PabloAST * shiftAmount, const llvm::StringRef & prefix);

    inline PabloAST * createLookahead(PabloAST * expr, const int64_t shiftAmount) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createLookahead(expr, mPb->getInteger(shiftAmount));
    }

    PabloAST * createLookahead(PabloAST * expr, PabloAST * shiftAmount);

    inline PabloAST * createLookahead(PabloAST * expr, const int64_t shiftAmount, const llvm::StringRef & prefix) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createLookahead(expr, mPb->getInteger(shiftAmount), prefix);
    }

    PabloAST * createLookahead(PabloAST * expr, PabloAST * shiftAmount, const llvm::StringRef & prefix);

    PabloAST * createAssign(PabloAST * const variable, PabloAST * const value);

    PabloAST * createAnd(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createAnd(PabloAST * expr1, PabloAST * expr2, const llvm::StringRef & prefix);

    PabloAST * createNot(PabloAST * expr);

    PabloAST * createNot(PabloAST * expr, const llvm::StringRef & prefix);

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

    inline If * createIf(PabloAST * condition, PabloBlock * body) {
        return mPb->createIf(condition, body);
    }

    inline If * createIf(PabloAST * condition, PabloBuilder & builder) {
        return mPb->createIf(condition, builder.mPb);
    }

    inline While * createWhile(PabloAST * condition, PabloBlock * body) {
        return mPb->createWhile(condition, body);
    }

    inline While * createWhile(PabloAST * condition, PabloBuilder & builder) {
        return mPb->createWhile(condition, builder.mPb);
    }

//    llvm::Type * getStreamTy(const unsigned FieldWidth = 1) {
//        return mPb->getStreamTy(FieldWidth);
//    }
    
    llvm::Type * getStreamSetTy(const unsigned NumElements = 1, const unsigned FieldWidth = 1) {
        return mPb->getStreamSetTy(NumElements, FieldWidth);
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

    inline Statement * getInsertPoint() const {
        return mPb->getInsertPoint();
    }

    inline PabloBlock * getPabloBlock() const {
        return mPb;
    }

    inline PabloBuilder * getParent() const {
        return mParent;
    }

    inline String * makeName(const llvm::StringRef & prefix) const {
        return mPb->makeName(prefix);
    }

    inline Integer * getInteger(const uint64_t value) const {
        return mPb->getInteger(value);
    }

    inline void print(llvm::raw_ostream & O, const bool expandNested = true) const {
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
