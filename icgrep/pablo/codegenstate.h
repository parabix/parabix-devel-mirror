/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_PABLOS_H
#define PS_PABLOS_H

#include <pablo/pabloAST.h>
#include <pablo/symbol_generator.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_lookahead.h>
#include <pablo/pe_and.h>
#include <pablo/pe_call.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_not.h>
#include <pablo/pe_ones.h>
#include <pablo/pe_or.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_sel.h>
#include <pablo/pe_infile.h>
#include <pablo/pe_integer.h>
#include <pablo/pe_string.h>
#include <pablo/pe_var.h>
#include <pablo/pe_xor.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_count.h>
#include <pablo/ps_assign.h>
#include <pablo/branch.h>
#include <pablo/function.h>
#include <llvm/ADT/ArrayRef.h>
#include <stdexcept>

namespace pablo {

class PabloBlock : public PabloAST, public StatementList {
    friend class PabloAST;
    friend class Branch;
    friend class PabloBuilder;
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

    inline static PabloBlock * Create(PabloFunction & parent) noexcept {
        return new PabloBlock(&parent);
    }

    inline static PabloBlock * Create(PabloBlock * const predecessor) noexcept {
        return new PabloBlock(predecessor->mParent);
    }

    Advance * createAdvance(PabloAST * expr, PabloAST * shiftAmount) {
        return createAdvance(expr, shiftAmount, nullptr);
    }

    Advance * createAdvance(PabloAST * expr, PabloAST * shiftAmount, const std::string & prefix) {
        return createAdvance(expr, shiftAmount, makeName(prefix));
    }

    Advance * createAdvance(PabloAST * expr, PabloAST * shiftAmount, String * const name);

    Lookahead * createLookahead(PabloAST * expr, PabloAST * shiftAmount) {
        return createLookahead(expr, shiftAmount, nullptr);
    }

    Lookahead * createLookahead(PabloAST * expr, PabloAST * shiftAmount, const std::string & prefix) {
        return createLookahead(expr, shiftAmount, makeName(prefix));
    }

    Lookahead * createLookahead(PabloAST * expr, PabloAST * shiftAmount, String * const name);

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

    Not * createNot(PabloAST * expr, String * const name);

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

    InFile * createInFile(PabloAST * expr, String * const name);

    AtEOF * createAtEOF(PabloAST * expr) {
        return createAtEOF(expr, nullptr);
    }

    AtEOF * createAtEOF(PabloAST * expr, const std::string & prefix) {
        return createAtEOF(expr, makeName(prefix));
    }

    AtEOF * createAtEOF(PabloAST * expr, String * const name);

    Extract * createExtract(PabloAST * array, const Integer::Type index) {
        return createExtract(array, getInteger(index), nullptr);
    }

    inline Extract * createExtract(PabloAST * array, PabloAST * index) {
        return createExtract(array, index, nullptr);
    }

    Extract * createExtract(PabloAST * array, PabloAST * index, const std::string & prefix) {
        return createExtract(array, index, makeName(prefix));
    }

    Extract * createExtract(PabloAST * array, const Integer::Type index, const std::string & prefix) {
        return createExtract(array, getInteger(index), makeName(prefix));
    }

    Extract * createExtract(PabloAST * array, PabloAST * index, String * const name);

    Assign * createAssign(PabloAST * const var, PabloAST * const value);

    And * createAnd(PabloAST * expr1, PabloAST * expr2) {
        return createAnd(expr1, expr2, nullptr);
    }

    And * createAnd(PabloAST * expr1, PabloAST * expr2, const std::string & prefix) {
        return createAnd(expr1, expr2, nullptr);
    }

    And * createAnd(PabloAST * expr1, PabloAST * expr2, String * const name);

    And * createAnd(Type * const type, const unsigned reserved) {
        return createAnd(type, reserved, nullptr);
    }

    And * createAnd(Type * const type, const unsigned reserved, String * const name);

    Or * createOr(PabloAST * expr1, PabloAST * expr2) {
        return createOr(expr1, expr2, nullptr);
    }

    Or * createOr(PabloAST * expr1, PabloAST * expr2, const std::string & prefix) {
        return createOr(expr1, expr2, makeName(prefix));
    }

    Or * createOr(PabloAST * expr1, PabloAST * expr2, String * const name);

    Or * createOr(Type * const type, const unsigned reserved) {
        return createOr(type, reserved, nullptr);
    }

    Or * createOr(Type * const type, const unsigned reserved, String * const name);

    Xor * createXor(PabloAST * expr1, PabloAST * expr2) {
        return createXor(expr1, expr2, nullptr);
    }

    Xor * createXor(PabloAST * expr1, PabloAST * expr2, const std::string & prefix) {
        return createXor(expr1, expr2, makeName(prefix));
    }

    Xor * createXor(PabloAST * expr1, PabloAST * expr2, String * const name);

    Xor * createXor(Type * const type, const unsigned reserved) {
        return createXor(type, reserved, nullptr);
    }

    Xor * createXor(Type * const type, const unsigned reserved, String * const name);

    MatchStar * createMatchStar(PabloAST * marker, PabloAST * charclass) {
        return createMatchStar(marker, charclass, nullptr);
    }

    MatchStar * createMatchStar(PabloAST * marker, PabloAST * charclass, const std::string & prefix) {
        return createMatchStar(marker, charclass, makeName(prefix));
    }

    MatchStar * createMatchStar(PabloAST * marker, PabloAST * charclass, String * const name);

    ScanThru * createScanThru(PabloAST * from, PabloAST * thru) {
        return createScanThru(from, thru, nullptr);
    }

    ScanThru * createScanThru(PabloAST * from, PabloAST * thru, const std::string & prefix) {
        return createScanThru(from, thru, makeName(prefix));
    }

    ScanThru * createScanThru(PabloAST * from, PabloAST * thru, String * const name);

    Sel * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr) {
        return createSel(condition, trueExpr, falseExpr, nullptr);
    }

    Sel * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, const std::string & prefix) {
        return createSel(condition, trueExpr, falseExpr, makeName(prefix));
    }

    Sel * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, String * const name);
    
    If * createIf(PabloAST * condition, PabloBlock * body);

    While * createWhile(PabloAST * condition, PabloBlock * body);

    inline PabloBlock * getPredecessor() const {
        return getBranch() ? getBranch()->getParent() : nullptr;
    }

    inline PabloFunction * getParent() const {
        return mParent;
    }

    void setParent(PabloFunction * const parent) {
        mParent = parent;
    }

    void insert(Statement * const statement);

    unsigned enumerateScopes(unsigned baseScopeIndex);
    
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

    inline String * getName(const std::string name) const {
        return getSymbolTable()->get(name);
    }

    inline String * makeName(const std::string & prefix) const {
        return getSymbolTable()->make(prefix);
    }

    inline Integer * getInteger(Integer::Type value) const {
        return getSymbolTable()->getInteger(value);
    }

    SymbolGenerator * getSymbolTable() const {
        return mParent->getSymbolTable();
    }

    virtual ~PabloBlock();

protected:

    explicit PabloBlock(PabloFunction * parent) noexcept;

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
    PabloFunction *                                     mParent;
    Branch *                                            mBranch;
    unsigned                                            mScopeIndex;
};

}

#endif // PS_PABLOS_H
