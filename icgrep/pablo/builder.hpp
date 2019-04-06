#ifndef BUILDER_HPP
#define BUILDER_HPP

#include <pablo/codegenstate.h>
#include <pablo/expression_map.hpp>
#include <pablo/pe_var.h>
#include <util/not_null.h>
#include <sstream>

namespace pablo {

class PabloBuilder {
public:

    using iterator = PabloBlock::iterator;

    using const_iterator = PabloBlock::const_iterator;

    PabloBuilder(not_null<PabloBlock *> block)
    : mPb(block), mParent(nullptr), mExprTable(nullptr) {

    }

    PabloBuilder createScope() noexcept {
        return PabloBuilder(mPb->createScope(), this);
    }

    Zeroes * createZeroes(llvm::Type * const type = nullptr) {
        return mPb->createZeroes(type);
    }

    Ones * createOnes(llvm::Type * const type = nullptr) {
        return mPb->createOnes(type);
    }

    Var * createVar(const llvm::StringRef name, llvm::Type * const type = nullptr) {
        return createVar(makeName(name), type);
    }

    Var * createVar(const llvm::StringRef name, PabloAST * value) {
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

    Var * createExtract(Var * const array, not_null<Integer *> index, const llvm::StringRef name) {
        return createVar(name, createExtract(array, index));
    }

    Var * createExtract(Var * const array, const int64_t index, const llvm::StringRef name) {
        return createVar(name, createExtract(array, index));
    }

    PabloAST * createAdvance(PabloAST * expr, const int64_t shiftAmount) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createAdvance(expr, mPb->getInteger(shiftAmount));
    }

    PabloAST * createAdvance(PabloAST * expr, not_null<Integer *> shiftAmount);

    PabloAST * createAdvance(PabloAST * expr, const int64_t shiftAmount, const llvm::StringRef prefix) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createAdvance(expr, mPb->getInteger(shiftAmount), prefix);
    }

    PabloAST * createAdvance(PabloAST * expr, not_null<Integer *> shiftAmount, const llvm::StringRef prefix);

    PabloAST * createIndexedAdvance(PabloAST * expr, PabloAST * indexStream, const int64_t shiftAmount) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createIndexedAdvance(expr, indexStream, mPb->getInteger(shiftAmount));
    }

    PabloAST * createIndexedAdvance(PabloAST * expr, PabloAST * indexStream, not_null<Integer *> shiftAmount);

    PabloAST * createIndexedAdvance(PabloAST * expr, PabloAST * indexStream, const int64_t shiftAmount, const llvm::StringRef prefix) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createIndexedAdvance(expr, indexStream, mPb->getInteger(shiftAmount), prefix);
    }

    PabloAST * createIndexedAdvance(PabloAST * expr, PabloAST * indexStream, not_null<Integer *> shiftAmount, const llvm::StringRef prefix);

    PabloAST * createLookahead(PabloAST * expr, const int64_t shiftAmount) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createLookahead(expr, mPb->getInteger(shiftAmount));
    }

    PabloAST * createLookahead(PabloAST * expr, not_null<Integer *> shiftAmount);

    PabloAST * createLookahead(PabloAST * expr, const int64_t shiftAmount, const llvm::StringRef prefix) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createLookahead(expr, mPb->getInteger(shiftAmount), prefix);
    }

    PabloAST * createLookahead(PabloAST * expr, not_null<Integer *> shiftAmount, const llvm::StringRef prefix);

    Assign * createAssign(PabloAST * const variable, PabloAST * const value){
        return mPb->createAssign(variable, value);
    }

    PabloAST * createAnd(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createAnd(PabloAST * expr1, PabloAST * expr2, const llvm::StringRef prefix);

    PabloAST * createNot(PabloAST * expr);

    PabloAST * createNot(PabloAST * expr, const llvm::StringRef prefix);

    PabloAST * createRepeat(const int64_t fieldWidth, const int64_t value) {
        std::stringstream name;
        name << "repeating<" << fieldWidth << ">(" << std::hex << value << ")";
        return createRepeat(mPb->getInteger(fieldWidth), mPb->getInteger(value), name.str());
    }

    PabloAST * createRepeat(const int64_t fieldWidth, PabloAST * value) {
        return createRepeat(mPb->getInteger(fieldWidth), value);
    }

    PabloAST * createRepeat(not_null<Integer *> fieldWidth, PabloAST * value);

    PabloAST * createRepeat(const int64_t fieldWidth, PabloAST * value, const llvm::StringRef prefix) {
        return createRepeat(mPb->getInteger(fieldWidth), value, prefix);
    }

    PabloAST * createRepeat(const int64_t fieldWidth, const int64_t value, const llvm::StringRef prefix) {
        return createRepeat(mPb->getInteger(fieldWidth), mPb->getInteger(value), prefix);
    }

    PabloAST * createRepeat(not_null<Integer *> fieldWidth, PabloAST * value, const llvm::StringRef prefix);

    PabloAST * createPackL(const int64_t fieldWidth, PabloAST * value) {
        return createPackL(mPb->getInteger(fieldWidth), value);
    }

    PabloAST * createPackH(const int64_t fieldWidth, PabloAST * value) {
        return createPackH(mPb->getInteger(fieldWidth), value);
    }

    PabloAST * createPackL(not_null<Integer *> fieldWidth, PabloAST * value);

    PabloAST * createPackH(not_null<Integer *> fieldWidth, PabloAST * value);


    PabloAST * createOr(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createOr(PabloAST * expr1, PabloAST * expr2, const llvm::StringRef prefix);

    PabloAST * createXor(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createXor(PabloAST * expr1, PabloAST * expr2, const llvm::StringRef prefix);

    PabloAST * createMatchStar(PabloAST * marker, PabloAST * charclass);

    PabloAST * createMatchStar(PabloAST * marker, PabloAST * charclass, const llvm::StringRef prefix);

    PabloAST * createScanThru(PabloAST * from, PabloAST * thru);

    PabloAST * createScanThru(PabloAST * from, PabloAST * thru, const llvm::StringRef prefix);

    PabloAST * createScanTo(PabloAST * from, PabloAST * to);

    PabloAST * createScanTo(PabloAST * from, PabloAST * to, const llvm::StringRef prefix);

    PabloAST * createAdvanceThenScanThru(PabloAST * from, PabloAST * thru);

    PabloAST * createAdvanceThenScanThru(PabloAST * from, PabloAST * thru, const llvm::StringRef prefix);

    PabloAST * createAdvanceThenScanTo(PabloAST * from, PabloAST * to);

    PabloAST * createAdvanceThenScanTo(PabloAST * from, PabloAST * to, const llvm::StringRef prefix);

    PabloAST * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr);

    PabloAST * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, const llvm::StringRef prefix);

    PabloAST * createAnd3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3);

    PabloAST * createAnd3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const llvm::StringRef prefix);

    PabloAST * createOr3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3);

    PabloAST * createOr3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const llvm::StringRef prefix);

    PabloAST * createXor3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3);

    PabloAST * createXor3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const llvm::StringRef prefix);

    PabloAST * createMajority3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3);

    PabloAST * createMajority3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const llvm::StringRef prefix);

    PabloAST * createAndOr(PabloAST * andExpr1, PabloAST * orExpr1, PabloAST * orExpr2);

    PabloAST * createAndOr(PabloAST * andExpr1, PabloAST * orExpr1, PabloAST * orExpr2, const llvm::StringRef prefix);

    PabloAST * createAndXor(PabloAST * andExpr1, PabloAST * xorExpr1, PabloAST * xorExpr2);

    PabloAST * createAndXor(PabloAST * andExpr1, PabloAST * xorExpr1, PabloAST * xorExpr2, const llvm::StringRef prefix);

    PabloAST * createOrAnd(PabloAST * orExpr1, PabloAST * andExpr1, PabloAST * andExpr2);

    PabloAST * createOrAnd(PabloAST * orExpr1, PabloAST * andExpr1, PabloAST * andExpr2, const llvm::StringRef prefix);

    PabloAST * createOrXor(PabloAST * orExpr1, PabloAST * xorExpr1, PabloAST * xorExpr2);

    PabloAST * createOrXor(PabloAST * orExpr1, PabloAST * xorExpr1, PabloAST * xorExpr2, const llvm::StringRef prefix);

    PabloAST * createXorAnd(PabloAST * xorExpr1, PabloAST * andExpr1, PabloAST * andExpr2);

    PabloAST * createXorAnd(PabloAST * xorExpr1, PabloAST * andExpr1, PabloAST * andExpr2, const llvm::StringRef prefix);

    PabloAST * createXorOr(PabloAST * xorExpr1, PabloAST * orExpr1, PabloAST * orExpr2);

    PabloAST * createXorOr(PabloAST * xorExpr1, PabloAST * orExpr1, PabloAST * orExpr2, const llvm::StringRef prefix);

    PabloAST * createTernary(const uint8_t mask, PabloAST * a, PabloAST * b, PabloAST * c) {
        return createTernary(getInteger(mask), a, b, c);
    }

    PabloAST * createTernary(Integer * mask, PabloAST * a, PabloAST * b, PabloAST * c);

    PabloAST * createTernary(const uint8_t mask, PabloAST * a, PabloAST * b, PabloAST * c, const llvm::StringRef prefix) {
        return createTernary(getInteger(mask), a, b, c, prefix);
    }

    PabloAST * createTernary(Integer * mask, PabloAST * a, PabloAST * b, PabloAST * c, const llvm::StringRef prefix);

    PabloAST * createCount(PabloAST * expr);

    PabloAST * createCount(PabloAST * expr, const llvm::StringRef prefix);

    PabloAST * createInFile(PabloAST * expr);

    PabloAST * createInFile(PabloAST * expr, const llvm::StringRef prefix);

    PabloAST * createAtEOF(PabloAST * expr);

    PabloAST * createAtEOF(PabloAST * expr, const llvm::StringRef prefix);

    PabloAST * createTerminateAt(PabloAST * strm, not_null<Integer *> code);

    PabloAST * createTerminateAt(PabloAST * strm, not_null<Integer *> code, const llvm::StringRef prefix);

    PabloAST * createTerminateAt(PabloAST * strm, int64_t code) {
        return createTerminateAt(strm, mPb->getInteger(code));
    }

    PabloAST * createTerminateAt(PabloAST * strm, int64_t code, const llvm::StringRef prefix) {
        return createTerminateAt(strm, mPb->getInteger(code), prefix);
    }

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

    String * makeName(const llvm::StringRef prefix) const {
        return mPb->makeName(prefix);
    }

    Integer * getInteger(const uint64_t value, unsigned intWidth = 64) const {
        return mPb->getInteger(value, intWidth);
    }

    void print(llvm::raw_ostream & O, const bool expandNested = true) const {
        mPb->print(O, expandNested);
    }

protected:

    PabloBuilder(not_null<PabloBlock *> block, not_null<PabloBuilder *> parent)
    : mPb(block), mParent(parent), mExprTable(&(parent->mExprTable)) {

    }

private:

    PabloBlock * const          mPb;
    PabloBuilder * const        mParent;
    ExpressionTable             mExprTable;
};


}


#endif // BUILDER_HPP
