/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_PABLOS_H
#define PS_PABLOS_H

#include <pablo/pabloAST.h>
#include <pablo/symbol_generator.h>
#include <pablo/boolean.h>
#include <pablo/arithmetic.h>
#include <pablo/branch.h>

#include <pablo/pe_advance.h>
#include <pablo/pe_lookahead.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_infile.h>

#include <pablo/pe_count.h>

#include <pablo/pe_integer.h>
#include <pablo/pe_string.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>

#include <pablo/pe_var.h>
#include <pablo/ps_assign.h>

#include <pablo/pe_call.h>

#include <pablo/pablo_kernel.h>

#include <llvm/ADT/ArrayRef.h>
#include <stdexcept>

namespace pablo {

class PabloKernel;

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

    inline static PabloBlock * Create(PabloKernel * const parent) noexcept {
        Allocator & allocator = parent->mAllocator;
        return new (allocator) PabloBlock(parent, allocator);
    }

    Advance * createAdvance(PabloAST * expr, PabloAST * shiftAmount) {
        return createAdvance(expr, shiftAmount, nullptr);
    }

    Advance * createAdvance(PabloAST * expr, PabloAST * shiftAmount, const std::string & prefix) {
        return createAdvance(expr, shiftAmount, makeName(prefix));
    }

    Advance * createAdvance(PabloAST * expr, PabloAST * shiftAmount, String * name);

    Lookahead * createLookahead(PabloAST * expr, PabloAST * shiftAmount) {
        return createLookahead(expr, shiftAmount, nullptr);
    }

    Lookahead * createLookahead(PabloAST * expr, PabloAST * shiftAmount, const std::string & prefix) {
        return createLookahead(expr, shiftAmount, makeName(prefix));
    }

    Lookahead * createLookahead(PabloAST * expr, PabloAST * shiftAmount, String * name);

    inline Zeroes * createZeroes(Type * const type = nullptr) {
        return mParent->getNullValue(type);
    }

    inline Ones * createOnes(Type * const type = nullptr) {
        return mParent->getAllOnesValue(type);
    }

    inline Call * createCall(Prototype * prototype, const std::vector<Var *> & args) {
        return createCall(prototype, reinterpret_cast<const std::vector<PabloAST *> &>(args));
    }

    inline Call * createCall(Prototype * prototype, const std::vector<PabloAST *> & args) {
        if (prototype == nullptr) {
            throw std::runtime_error("Call object cannot be created with a Null prototype!");
        }
        if (args.size() != cast<Prototype>(prototype)->getNumOfParameters()) {
            throw std::runtime_error("Invalid number of arguments passed into Call object!");
        }
        return createCall(static_cast<PabloAST *>(prototype), args);
    }

    Not * createNot(PabloAST * expr) {
        return createNot(expr, nullptr);
    }

    Not * createNot(PabloAST * expr, const std::string & prefix) {
        return createNot(expr, makeName(prefix));
    }

    Not * createNot(PabloAST * expr, String * name);

    inline Var * createVar(const std::string & name, Type * const type = nullptr) {
        return createVar(makeName(name), type);
    }

    inline Var * createVar(String * name, Type * const type = nullptr) {
        return createVar(cast<PabloAST>(name), type);
    }

    Count * createCount(PabloAST * expr);

    Count * createCount(PabloAST * expr, const std::string & prefix);

    InFile * createInFile(PabloAST * expr) {
        return createInFile(expr, nullptr);
    }

    InFile * createInFile(PabloAST * expr, const std::string & prefix) {
        return createInFile(expr, makeName(prefix));
    }

    InFile * createInFile(PabloAST * expr, String * name);

    AtEOF * createAtEOF(PabloAST * expr) {
        return createAtEOF(expr, nullptr);
    }

    AtEOF * createAtEOF(PabloAST * expr, const std::string & prefix) {
        return createAtEOF(expr, makeName(prefix));
    }

    AtEOF * createAtEOF(PabloAST * expr, String * name);

    Extract * createExtract(PabloAST * array, const int64_t index) {
        return createExtract(array, getInteger(index), nullptr);
    }

    inline Extract * createExtract(PabloAST * array, PabloAST * index) {
        return createExtract(array, index, nullptr);
    }

    Extract * createExtract(PabloAST * array, PabloAST * index, const std::string & prefix) {
        return createExtract(array, index, makeName(prefix));
    }

    Extract * createExtract(PabloAST * array, const int64_t index, const std::string & prefix) {
        return createExtract(array, getInteger(index), makeName(prefix));
    }

    Extract * createExtract(PabloAST * array, PabloAST * index, String * name);

    Assign * createAssign(PabloAST * const var, PabloAST * const value);

    And * createAnd(PabloAST * expr1, PabloAST * expr2) {
        return createAnd(expr1, expr2, nullptr);
    }

    And * createAnd(PabloAST * expr1, PabloAST * expr2, const std::string & prefix) {
        return createAnd(expr1, expr2, nullptr);
    }

    And * createAnd(PabloAST * expr1, PabloAST * expr2, String * name);

    And * createAnd(Type * const type, const unsigned reserved) {
        return createAnd(type, reserved, nullptr);
    }

    And * createAnd(Type * const type, const unsigned reserved, String * name);

    Or * createOr(PabloAST * expr1, PabloAST * expr2) {
        return createOr(expr1, expr2, nullptr);
    }

    Or * createOr(PabloAST * expr1, PabloAST * expr2, const std::string & prefix) {
        return createOr(expr1, expr2, makeName(prefix));
    }

    Or * createOr(PabloAST * expr1, PabloAST * expr2, String * name);

    Or * createOr(Type * const type, const unsigned reserved) {
        return createOr(type, reserved, nullptr);
    }

    Or * createOr(Type * const type, const unsigned reserved, String * name);

    Xor * createXor(PabloAST * expr1, PabloAST * expr2) {
        return createXor(expr1, expr2, nullptr);
    }

    Xor * createXor(PabloAST * expr1, PabloAST * expr2, const std::string & prefix) {
        return createXor(expr1, expr2, makeName(prefix));
    }

    Xor * createXor(PabloAST * expr1, PabloAST * expr2, String * name);

    Xor * createXor(Type * const type, const unsigned reserved) {
        return createXor(type, reserved, nullptr);
    }

    Xor * createXor(Type * const type, const unsigned reserved, String * name);

    Sel * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr) {
        return createSel(condition, trueExpr, falseExpr, nullptr);
    }

    Sel * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, const std::string & prefix) {
        return createSel(condition, trueExpr, falseExpr, makeName(prefix));
    }

    Sel * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, String * name);

    Add * createAdd(PabloAST * expr1, PabloAST * expr2);

    Subtract * createSubtract(PabloAST * expr1, PabloAST * expr2);

    LessThan * createLessThan(PabloAST * expr1, PabloAST * expr2);

    MatchStar * createMatchStar(PabloAST * marker, PabloAST * charclass) {
        return createMatchStar(marker, charclass, nullptr);
    }

    MatchStar * createMatchStar(PabloAST * marker, PabloAST * charclass, const std::string & prefix) {
        return createMatchStar(marker, charclass, makeName(prefix));
    }

    MatchStar * createMatchStar(PabloAST * marker, PabloAST * charclass, String * name);

    ScanThru * createScanThru(PabloAST * from, PabloAST * thru) {
        return createScanThru(from, thru, nullptr);
    }

    ScanThru * createScanThru(PabloAST * from, PabloAST * thru, const std::string & prefix) {
        return createScanThru(from, thru, makeName(prefix));
    }

    ScanThru * createScanThru(PabloAST * from, PabloAST * thru, String * name);

    If * createIf(PabloAST * condition, PabloBlock * body);

    While * createWhile(PabloAST * condition, PabloBlock * body);

    Type * getStreamTy(const uint64_t FieldWidth = 1) {
        return mParent->getStreamTy(FieldWidth);
    }
    
    Type * getStreamSetTy(const uint64_t NumElements = 1, const uint64_t FieldWidth = 1) {
        return mParent->getStreamSetTy(NumElements, FieldWidth);
    }
    
    inline PabloBlock * getPredecessor() const {
        return getBranch() ? getBranch()->getParent() : nullptr;
    }

    inline PabloKernel * getParent() const {
        return mParent;
    }

    void insert(Statement * const statement);

    inline void setScopeIndex(const unsigned index) {
        mScopeIndex = index;
    }

    inline unsigned getScopeIndex() const {
        return mScopeIndex;
    }
    
    void eraseFromParent(const bool recursively = false);

    inline Branch * getBranch() const {
        return mBranch;
    }

    inline void setBranch(Branch * const branch) {
        mBranch = branch;
    }

    inline String * getName(const std::string & name) const {
        return mParent->getName(name);
    }

    inline String * makeName(const std::string & prefix) const {
        return mParent->makeName(prefix);
    }

    inline Integer * getInteger(const int64_t value) const {
        return mParent->getInteger(value);
    }

    virtual ~PabloBlock() {}

protected:

    explicit PabloBlock(PabloKernel * const parent, Allocator & allocator) noexcept
    : PabloAST(PabloAST::ClassTypeId::Block, nullptr, nullptr, allocator)
    , mParent(parent)
    , mBranch(nullptr)
    , mScopeIndex(0)
    , mAllocator(allocator) {

    }

    template<typename Type>
    inline Type * insertAtInsertionPoint(Type * expr) {
        if (isa<Statement>(expr)) {
            insert(cast<Statement>(expr));
        }
        return expr;
    }

    Call * createCall(PabloAST * prototype, const std::vector<PabloAST *> &);

    Var * createVar(PabloAST * name, Type * const type);

private:        
    PabloKernel * const         mParent;
    Branch *                    mBranch;
    unsigned                    mScopeIndex;
    Allocator &                 mAllocator;
};

}

#endif // PS_PABLOS_H
