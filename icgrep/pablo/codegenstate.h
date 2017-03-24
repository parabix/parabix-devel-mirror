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
namespace pablo { class AdvanceThenScanThru; }
namespace pablo { class AdvanceThenScanTo; }
namespace pablo { class And; }
namespace pablo { class Assign; }
namespace pablo { class AtEOF; }
namespace pablo { class Branch; }
namespace pablo { class If; }
namespace pablo { class While; }
namespace pablo { class Count; }
namespace pablo { class Extract; }
namespace pablo { class InFile; }
namespace pablo { class LessThan; }
namespace pablo { class Lookahead; }
namespace pablo { class MatchStar; }
namespace pablo { class Not; }
namespace pablo { class Ones; }
namespace pablo { class Or; }
namespace pablo { class PabloKernel; }
namespace pablo { class Phi; }
namespace pablo { class ScanThru; }
namespace pablo { class ScanTo; }
namespace pablo { class Sel; }
namespace pablo { class String; }
namespace pablo { class Subtract; }
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

    static inline bool classof(const PabloBlock *) {
        return true;
    }
    static inline bool classof(const Statement *) {
        return false;
    }
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Block;
    }
    static inline bool classof(const void *) {
        return false;
    }

    static PabloBlock * Create(PabloKernel * const parent) noexcept;

    Advance * createAdvance(PabloAST * expr, PabloAST * shiftAmount) {
        return createAdvance(expr, shiftAmount, nullptr);
    }

    Advance * createAdvance(PabloAST * expr, PabloAST * shiftAmount, const llvm::StringRef & prefix) {
        return createAdvance(expr, shiftAmount, makeName(prefix));
    }

    Advance * createAdvance(PabloAST * expr, PabloAST * shiftAmount, String * name);

    Lookahead * createLookahead(PabloAST * expr, PabloAST * shiftAmount) {
        return createLookahead(expr, shiftAmount, nullptr);
    }

    Lookahead * createLookahead(PabloAST * expr, PabloAST * shiftAmount, const llvm::StringRef & prefix) {
        return createLookahead(expr, shiftAmount, makeName(prefix));
    }

    Lookahead * createLookahead(PabloAST * expr, PabloAST * shiftAmount, String * name);

    inline Zeroes * createZeroes(llvm::Type * const type = nullptr) {
        return mParent->getNullValue(type);
    }

    inline Ones * createOnes(llvm::Type * const type = nullptr) {
        return mParent->getAllOnesValue(type);
    }

    Not * createNot(PabloAST * expr) {
        return createNot(expr, nullptr);
    }

    Not * createNot(PabloAST * expr, const llvm::StringRef & prefix) {
        return createNot(expr, makeName(prefix));
    }

    Not * createNot(PabloAST * expr, String * name);

    inline Var * createVar(const llvm::StringRef & name, llvm::Type * const type = nullptr) {
        return createVar(makeName(name), type);
    }

    inline Var * createVar(String * name, llvm::Type * const type = nullptr) {
        return createVar(reinterpret_cast<PabloAST *>(name), type);
    }

    Count * createCount(PabloAST * expr);

    Count * createCount(PabloAST * expr, const llvm::StringRef & prefix);

    InFile * createInFile(PabloAST * expr) {
        return createInFile(expr, nullptr);
    }

    InFile * createInFile(PabloAST * expr, const llvm::StringRef & prefix) {
        return createInFile(expr, makeName(prefix));
    }

    InFile * createInFile(PabloAST * expr, String * name);

    AtEOF * createAtEOF(PabloAST * expr) {
        return createAtEOF(expr, nullptr);
    }

    AtEOF * createAtEOF(PabloAST * expr, const llvm::StringRef & prefix) {
        return createAtEOF(expr, makeName(prefix));
    }

    AtEOF * createAtEOF(PabloAST * expr, String * name);

    inline Extract * createExtract(PabloAST * array, PabloAST * index) {
        return createExtract(array, index, nullptr);
    }

    Extract * createExtract(PabloAST * array, PabloAST * index, const llvm::StringRef & prefix) {
        return createExtract(array, index, makeName(prefix));
    }

    Extract * createExtract(PabloAST * array, const int64_t index) {
        return createExtract(array, getInteger(index), nullptr);
    }

    Extract * createExtract(PabloAST * array, const int64_t index, const llvm::StringRef & prefix) {
        return createExtract(array, getInteger(index), makeName(prefix));
    }

    Extract * createExtract(PabloAST * array, PabloAST * index, String * name);

    Assign * createAssign(PabloAST * const var, PabloAST * const value);

    And * createAnd(PabloAST * expr1, PabloAST * expr2) {
        return createAnd(expr1, expr2, nullptr);
    }

    And * createAnd(PabloAST * expr1, PabloAST * expr2, const llvm::StringRef & prefix) {
        return createAnd(expr1, expr2, makeName(prefix));
    }

    And * createAnd(PabloAST * expr1, PabloAST * expr2, String * name);

    And * createAnd(llvm::Type * const type, const unsigned reserved) {
        return createAnd(type, reserved, nullptr);
    }

    And * createAnd(llvm::Type * const type, const unsigned reserved, String * name);

    Or * createOr(PabloAST * expr1, PabloAST * expr2) {
        return createOr(expr1, expr2, nullptr);
    }

    Or * createOr(PabloAST * expr1, PabloAST * expr2, const llvm::StringRef & prefix) {
        return createOr(expr1, expr2, makeName(prefix));
    }

    Or * createOr(PabloAST * expr1, PabloAST * expr2, String * name);

    Or * createOr(llvm::Type * const type, const unsigned reserved) {
        return createOr(type, reserved, nullptr);
    }

    Or * createOr(llvm::Type * const type, const unsigned reserved, String * name);

    Xor * createXor(PabloAST * expr1, PabloAST * expr2) {
        return createXor(expr1, expr2, nullptr);
    }

    Xor * createXor(PabloAST * expr1, PabloAST * expr2, const llvm::StringRef & prefix) {
        return createXor(expr1, expr2, makeName(prefix));
    }

    Xor * createXor(PabloAST * expr1, PabloAST * expr2, String * name);

    Xor * createXor(llvm::Type * const type, const unsigned reserved) {
        return createXor(type, reserved, nullptr);
    }

    Xor * createXor(llvm::Type * const type, const unsigned reserved, String * name);

    Sel * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr) {
        return createSel(condition, trueExpr, falseExpr, nullptr);
    }

    Sel * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, const llvm::StringRef & prefix) {
        return createSel(condition, trueExpr, falseExpr, makeName(prefix));
    }

    Sel * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, String * name);

    Add * createAdd(PabloAST * expr1, PabloAST * expr2);

    Subtract * createSubtract(PabloAST * expr1, PabloAST * expr2);

    LessThan * createLessThan(PabloAST * expr1, PabloAST * expr2);

    MatchStar * createMatchStar(PabloAST * marker, PabloAST * charclass) {
        return createMatchStar(marker, charclass, nullptr);
    }

    MatchStar * createMatchStar(PabloAST * marker, PabloAST * charclass, const llvm::StringRef & prefix) {
        return createMatchStar(marker, charclass, makeName(prefix));
    }

    MatchStar * createMatchStar(PabloAST * marker, PabloAST * charclass, String * name);

    ScanThru * createScanThru(PabloAST * from, PabloAST * thru) {
        return createScanThru(from, thru, nullptr);
    }

    ScanThru * createScanThru(PabloAST * from, PabloAST * thru, const llvm::StringRef & prefix) {
        return createScanThru(from, thru, makeName(prefix));
    }

    ScanThru * createScanThru(PabloAST * from, PabloAST * thru, String * name);

    ScanTo * createScanTo(PabloAST * from, PabloAST * to) {
        return createScanTo(from, to, nullptr);
    }

    ScanTo * createScanTo(PabloAST * from, PabloAST * to, const llvm::StringRef & prefix) {
        return createScanTo(from, to, makeName(prefix));
    }

    ScanTo * createScanTo(PabloAST * from, PabloAST * to, String * name);

    AdvanceThenScanThru * createAdvanceThenScanThru(PabloAST * from, PabloAST * thru) {
        return createAdvanceThenScanThru(from, thru, nullptr);
    }

    AdvanceThenScanThru * createAdvanceThenScanThru(PabloAST * from, PabloAST * thru, const llvm::StringRef & prefix) {
        return createAdvanceThenScanThru(from, thru, makeName(prefix));
    }

    AdvanceThenScanThru * createAdvanceThenScanThru(PabloAST * from, PabloAST * thru, String * name);

    AdvanceThenScanTo * createAdvanceThenScanTo(PabloAST * from, PabloAST * to) {
        return createAdvanceThenScanTo(from, to, nullptr);
    }

    AdvanceThenScanTo * createAdvanceThenScanTo(PabloAST * from, PabloAST * to, const llvm::StringRef & prefix) {
        return createAdvanceThenScanTo(from, to, makeName(prefix));
    }

    AdvanceThenScanTo * createAdvanceThenScanTo(PabloAST * from, PabloAST * to, String * name);

    If * createIf(PabloAST * condition, PabloBlock * body);

    While * createWhile(PabloAST * condition, PabloBlock * body);

    Phi * createPhi(llvm::Type * type);

    llvm::Type * getStreamSetTy(const unsigned NumElements = 1, const unsigned FieldWidth = 1) {
        return mParent->getStreamSetTy(NumElements, FieldWidth);
    }
    
    PabloBlock * getPredecessor() const;

    inline PabloKernel * getParent() const {
        return mParent;
    }

    void insert(Statement * const statement);

    void eraseFromParent(const bool recursively = false);

    inline Branch * getBranch() const {
        return mBranch;
    }

    inline void setBranch(Branch * const branch) {
        mBranch = branch;
    }

    inline String * makeName(const llvm::StringRef & prefix) const {
        return mParent->makeName(prefix);
    }

    inline Integer * getInteger(const int64_t value) const {
        return mParent->getInteger(value);
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
    inline Type * insertAtInsertionPoint(Type * expr) {
        if (llvm::isa<Statement>(expr)) {
            insert(llvm::cast<Statement>(expr));
        }
        return expr;
    }

    Var * createVar(PabloAST * name, llvm::Type * const type);

private:        
    PabloKernel * const         mParent;
    Branch *                    mBranch;
    Allocator &                 mAllocator;
};

}

#endif // PS_PABLOS_H
