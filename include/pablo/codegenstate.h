/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_PABLOS_H
#define PS_PABLOS_H

#include <pablo/pabloAST.h>
#include <pablo/pablo_kernel.h>
#include <pablo/pe_integer.h>
namespace llvm { class Type; }
namespace llvm { class raw_ostream; }
namespace pablo { class Add; }
namespace pablo { class Advance; }
namespace pablo { class IndexedAdvance; }
namespace pablo { class AdvanceThenScanThru; }
namespace pablo { class AdvanceThenScanTo; }
namespace pablo { class EveryNth; }
namespace pablo { class And; }
namespace pablo { class Assign; }
namespace pablo { class AtEOF; }
namespace pablo { class Branch; }
namespace pablo { class If; }
namespace pablo { class While; }
namespace pablo { class Count; }
namespace pablo { class Extract; }
namespace pablo { class InFile; }
namespace pablo { enum class Intrinsic; class IntrinsicCall; }
namespace pablo { class LessThan; }
namespace pablo { class Equals; }
namespace pablo { class Lookahead; }
namespace pablo { class MatchStar; }
namespace pablo { class Not; }
namespace pablo { class Ones; }
namespace pablo { class Or; }
namespace pablo { class PabloKernel; }
namespace pablo { class PackH; }
namespace pablo { class PackL; }
namespace pablo { class ScanThru; }
namespace pablo { class ScanTo; }
namespace pablo { class Sel; }
namespace pablo { class Repeat; }
namespace pablo { class String; }
namespace pablo { class Subtract; }
namespace pablo { class TerminateAt; }
namespace pablo { class Ternary; }
namespace pablo { class Var; }
namespace pablo { class Xor; }
namespace pablo { class Zeroes; }

namespace pablo {

class PabloBlock : public PabloAST, public StatementList {
    friend class PabloAST;
    friend class Branch;
    friend class PabloBuilder;
    friend class PabloKernel;
public:

    static bool classof(const PabloBlock *) {
        return true;
    }
    static bool classof(const Statement *) {
        return false;
    }
    static bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Block;
    }
    static bool classof(const void *) {
        return false;
    }

    PabloBlock * createScope() noexcept;

    Advance * createAdvance(PabloAST * expr, Integer * shiftAmount) {
        return createAdvance(expr, shiftAmount, nullptr);
    }

    Advance * createAdvance(PabloAST * expr, Integer * shiftAmount, const llvm::StringRef prefix) {
        return createAdvance(expr, shiftAmount, makeName(prefix));
    }

    Advance * createAdvance(PabloAST * expr, Integer * shiftAmount, const String * const name);

    IndexedAdvance * createIndexedAdvance(PabloAST * expr, PabloAST * indexStream, Integer * shiftAmount) {
        return createIndexedAdvance(expr, indexStream, shiftAmount, nullptr);
    }

    IndexedAdvance * createIndexedAdvance(PabloAST * expr, PabloAST * indexStream, Integer * shiftAmount, const llvm::StringRef prefix) {
        return createIndexedAdvance(expr, indexStream, shiftAmount, makeName(prefix));
    }

    IndexedAdvance * createIndexedAdvance(PabloAST * expr, PabloAST * indexStream, Integer * shiftAmount, const String * const name);

    Lookahead * createLookahead(PabloAST * expr, Integer * shiftAmount) {
        return createLookahead(expr, shiftAmount, nullptr);
    }

    Lookahead * createLookahead(PabloAST * expr, Integer * shiftAmount, const llvm::StringRef prefix) {
        return createLookahead(expr, shiftAmount, makeName(prefix));
    }

    Lookahead * createLookahead(PabloAST * expr, Integer * shiftAmount, const String * const name);

    Zeroes * createZeroes(llvm::Type * const type = nullptr) {
        return mParent->getNullValue(type);
    }

    Ones * createOnes(llvm::Type * const type = nullptr) {
        return mParent->getAllOnesValue(type);
    }

    Not * createNot(PabloAST * expr) {
        return createNot(expr, nullptr);
    }

    Not * createNot(PabloAST * expr, const llvm::StringRef prefix) {
        return createNot(expr, makeName(prefix));
    }

    Not * createNot(PabloAST * expr, const String * const name);

    Var * createVar(const llvm::StringRef name, llvm::Type * const type = nullptr) {
        return createVar(makeName(name), type);
    }

    Var * createVar(const String * const name, llvm::Type * const type = nullptr);

    Count * createCount(PabloAST * expr, const llvm::StringRef prefix) {
        return createCount(expr, makeName(prefix));
    }

    Count * createCount(PabloAST * expr, const String * const name = nullptr);

    EveryNth * createEveryNth(PabloAST * expr, Integer * n, const llvm::StringRef prefix) {
        return createEveryNth(expr, n, makeName(prefix));
    }

    EveryNth * createEveryNth(PabloAST * expr, Integer * n, const String * const name = nullptr);

    InFile * createInFile(PabloAST * expr, const llvm::StringRef prefix) {
        return createInFile(expr, makeName(prefix));
    }

    InFile * createInFile(PabloAST * expr, const String * const name = nullptr);

    AtEOF * createAtEOF(PabloAST * expr, const llvm::StringRef prefix) {
        return createAtEOF(expr, makeName(prefix));
    }

    AtEOF * createAtEOF(PabloAST * expr, const String * const name = nullptr);

    TerminateAt * createTerminateAt(PabloAST * strm, Integer * code) {
        return createTerminateAt(strm, code, nullptr);
    }

    TerminateAt * createTerminateAt(PabloAST * strm, Integer * code, const llvm::StringRef prefix) {
        return createTerminateAt(strm, code, makeName(prefix));
    }

    TerminateAt * createTerminateAt(PabloAST * strm, Integer * code, const String * const name);

    Extract * createExtract(Var * const array, Integer * const index);

    Assign * createAssign(Var * const var, PabloAST * const value);

    And * createAnd(PabloAST * expr1, PabloAST * expr2, const llvm::StringRef prefix) {
        return createAnd(expr1, expr2, makeName(prefix));
    }

    And * createAnd(PabloAST * expr1, PabloAST * expr2, const String * const name = nullptr);

    Or * createOr(PabloAST * expr1, PabloAST * expr2, const llvm::StringRef prefix) {
        return createOr(expr1, expr2, makeName(prefix));
    }

    Or * createOr(PabloAST * expr1, PabloAST * expr2, const String * const name = nullptr);

    Xor * createXor(PabloAST * expr1, PabloAST * expr2, const llvm::StringRef prefix) {
        return createXor(expr1, expr2, makeName(prefix));
    }

    Xor * createXor(PabloAST * expr1, PabloAST * expr2, const String * const name = nullptr);

    Sel * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, const llvm::StringRef prefix) {
        return createSel(condition, trueExpr, falseExpr, makeName(prefix));
    }

    Sel * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, const String * const name = nullptr);

    Ternary * createAnd3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const llvm::StringRef prefix) {
        return createAnd3(expr1, expr2, expr3, makeName(prefix));
    }

    Ternary * createAnd3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const String * const name = nullptr);

    Ternary * createOr3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const llvm::StringRef prefix) {
        return createOr3(expr1, expr2, expr3, makeName(prefix));
    }

    Ternary * createOr3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const String * const name = nullptr);

    Ternary * createXor3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const llvm::StringRef prefix) {
        return createXor3(expr1, expr2, expr3, makeName(prefix));
    }

    Ternary * createXor3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const String * const name = nullptr);

    Ternary * createMajority3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const llvm::StringRef prefix) {
        return createMajority3(expr1, expr2, expr3, makeName(prefix));
    }

    Ternary * createMajority3(PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, const String * const name = nullptr);

    Ternary * createAndOr(PabloAST * andExpr1, PabloAST * orExpr1, PabloAST * orExpr2, const llvm::StringRef prefix) {
        return createAndOr(andExpr1, orExpr1, orExpr2, makeName(prefix));
    }

    Ternary * createAndOr(PabloAST * andExpr1, PabloAST * orExpr1, PabloAST * orExpr2, const String * const name = nullptr);

    Ternary * createAndXor(PabloAST * andExpr1, PabloAST * xorExpr1, PabloAST * xorExpr2, const llvm::StringRef prefix) {
        return createAndXor(andExpr1, xorExpr1, xorExpr2, makeName(prefix));
    }

    Ternary * createAndXor(PabloAST * andExpr1, PabloAST * xorExpr1, PabloAST * xorExpr2, const String * const name = nullptr);

    Ternary * createOrAnd(PabloAST * orExpr1, PabloAST * andExpr1, PabloAST * andExpr2, const llvm::StringRef prefix) {
        return createOrAnd(orExpr1, andExpr1, andExpr2, makeName(prefix));
    }

    Ternary * createOrAnd(PabloAST * orExpr1, PabloAST * andExpr1, PabloAST * andExpr2, const String * const name = nullptr);

    Ternary * createOrXor(PabloAST * orExpr1, PabloAST * xorExpr1, PabloAST * xorExpr2, const llvm::StringRef prefix) {
        return createOrXor(orExpr1, xorExpr1, xorExpr2, makeName(prefix));
    }

    Ternary * createOrXor(PabloAST * orExpr1, PabloAST * xorExpr1, PabloAST * xorExpr2, const String * const name = nullptr);

    Ternary * createXorAnd(PabloAST * xorExpr1, PabloAST * andExpr1, PabloAST * andExpr2, const llvm::StringRef prefix) {
        return createXorAnd(xorExpr1, andExpr1, andExpr2, makeName(prefix));
    }

    Ternary * createXorAnd(PabloAST * xorExpr1, PabloAST * andExpr1, PabloAST * andExpr2, const String * const name = nullptr);

    Ternary * createXorOr(PabloAST * xorExpr1, PabloAST * orExpr1, PabloAST * orExpr2, const llvm::StringRef prefix) {
        return createXorOr(xorExpr1, orExpr1, orExpr2, makeName(prefix));
    }

    Ternary * createXorOr(PabloAST * xorExpr1, PabloAST * orExpr1, PabloAST * orExpr2, const String * const name = nullptr);

    Ternary * createTernary(Integer * mask, PabloAST * a, PabloAST * b, PabloAST * c, const String * const name = nullptr);

    Ternary * createTernary(Integer * mask, PabloAST * a, PabloAST * b, PabloAST * c, const llvm::StringRef prefix) {
        return createTernary(mask, a, b, c, makeName(prefix));
    }

    Add * createAdd(PabloAST * expr1, PabloAST * expr2);

    Subtract * createSubtract(PabloAST * expr1, PabloAST * expr2);

    LessThan * createLessThan(PabloAST * expr1, PabloAST * expr2);

    Equals * createEquals(PabloAST * expr1, PabloAST * expr2);

    MatchStar * createMatchStar(PabloAST * marker, PabloAST * charclass, const llvm::StringRef prefix) {
        return createMatchStar(marker, charclass, makeName(prefix));
    }

    MatchStar * createMatchStar(PabloAST * marker, PabloAST * charclass, const String * const name = nullptr);

    ScanThru * createScanThru(PabloAST * from, PabloAST * thru, const llvm::StringRef prefix) {
        return createScanThru(from, thru, makeName(prefix));
    }

    ScanThru * createScanThru(PabloAST * from, PabloAST * thru, const String * const name = nullptr);

    ScanTo * createScanTo(PabloAST * from, PabloAST * to, const llvm::StringRef prefix) {
        return createScanTo(from, to, makeName(prefix));
    }

    ScanTo * createScanTo(PabloAST * from, PabloAST * to, const String * const name = nullptr);

    AdvanceThenScanThru * createAdvanceThenScanThru(PabloAST * from, PabloAST * thru, const llvm::StringRef prefix) {
        return createAdvanceThenScanThru(from, thru, makeName(prefix));
    }

    AdvanceThenScanThru * createAdvanceThenScanThru(PabloAST * from, PabloAST * thru, const String * const name = nullptr);

    AdvanceThenScanTo * createAdvanceThenScanTo(PabloAST * from, PabloAST * to, const llvm::StringRef prefix) {
        return createAdvanceThenScanTo(from, to, makeName(prefix));
    }

    AdvanceThenScanTo * createAdvanceThenScanTo(PabloAST * from, PabloAST * to, const String * const name = nullptr);

    If * createIf(PabloAST * condition, PabloBlock * body);

    While * createWhile(PabloAST * condition, PabloBlock * body);

    Repeat * createRepeat(Integer * fieldWidth, PabloAST * value, const llvm::StringRef prefix) {
        return createRepeat(fieldWidth, value, makeName(prefix));
    }

    Repeat * createRepeat(Integer * fieldWidth, PabloAST * value, const String * const name = nullptr);

    PackH * createPackH(Integer * width, PabloAST * value, const llvm::StringRef prefix) {
        return createPackH(width, value, makeName(prefix));
    }

    PackH * createPackH(Integer * fieldWidth, PabloAST * value, const String * const name = nullptr);

    PackL * createPackL(Integer * fieldWidth, PabloAST * value, const llvm::StringRef prefix) {
        return createPackL(fieldWidth, value, makeName(prefix));
    }

    PackL * createPackL(Integer * fieldWidth, PabloAST * value, const String * const name = nullptr);

    IntrinsicCall * createIntrinsicCall(Intrinsic intrinsic, llvm::ArrayRef<PabloAST *> argv, const llvm::StringRef prefix) {
        return createIntrinsicCall(intrinsic, std::move(argv), makeName(prefix));
    }

    IntrinsicCall * createIntrinsicCall(Intrinsic intrinsic, llvm::ArrayRef<PabloAST *> argv, const String * name = nullptr) {
        assert (argv.size() > 0);
        return createIntrinsicCall(intrinsic, (*argv.begin())->getType(), argv, name);
    }

    IntrinsicCall * createIntrinsicCall(Intrinsic intrinsic, llvm::Type * type, llvm::ArrayRef<PabloAST *> argv, const String * name = nullptr);

    PabloBlock * getPredecessor() const;

    PabloKernel * getParent() const {
        return mParent;
    }

    void insert(Statement * const statement);

    void eraseFromParent(const bool recursively = false);

    Branch * getBranch() const {
        return mBranch;
    }

    void setBranch(Branch * const branch) {
        mBranch = branch;
    }

    String * makeName(const llvm::StringRef prefix) const {
        return mParent->makeName(prefix);
    }

    Integer * getInteger(const int64_t value, unsigned intWidth = 64) const {
        return mParent->getInteger(value, intWidth);
    }

    llvm::LLVMContext & getContext() const {
        return mParent->getContext();
    }

    void print(llvm::raw_ostream & O, const bool expandNested = true) const;

    virtual ~PabloBlock() {}

protected:

    PabloBlock(PabloKernel * const parent, Allocator & allocator) noexcept
    : PabloAST(PabloAST::ClassTypeId::Block, nullptr, allocator)
    , mParent(parent)
    , mBranch(nullptr)
    , mAllocator(allocator) {

    }

    template<typename Type>
    Type * insertAtInsertionPoint(Type * expr) {
        if (llvm::isa<Statement>(expr)) {
            insert(llvm::cast<Statement>(expr));
        }
        return expr;
    }

private:
    PabloKernel * const         mParent;
    Branch *                    mBranch;
    Allocator &                 mAllocator;
};

}

#endif // PS_PABLOS_H
