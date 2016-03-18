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
#include <pablo/pe_next.h>
#include <pablo/pe_not.h>
#include <pablo/pe_ones.h>
#include <pablo/pe_or.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_sel.h>
#include <pablo/pe_integer.h>
#include <pablo/pe_string.h>
#include <pablo/pe_var.h>
#include <pablo/pe_xor.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_count.h>
#include <pablo/ps_assign.h>
#include <pablo/ps_if.h>
#include <pablo/ps_while.h>
#include <pablo/function.h>
#include <llvm/ADT/ArrayRef.h>
#include <stdexcept>

namespace pablo {

class PabloBlock : public PabloAST, public StatementList {
    friend class PabloAST;
    friend class If;
    friend class While;
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

    inline static PabloBlock * Create(PabloFunction & function) noexcept {
        return new PabloBlock(function.mSymbolTable);
    }

    inline static PabloBlock * Create(PabloBlock * const block) noexcept {
        return new PabloBlock(block->mSymbolGenerator);
    }

    PabloAST * createAdvance(PabloAST * expr, const Integer::Type shiftAmount);

    PabloAST * createAdvance(PabloAST * expr, PabloAST * shiftAmount);

    PabloAST * createAdvance(PabloAST * expr, const Integer::Type shiftAmount, const std::string prefix);

    PabloAST * createAdvance(PabloAST * expr, PabloAST * shiftAmount, const std::string prefix);

    PabloAST * createLookahead(PabloAST * expr, const Integer::Type shiftAmount);

    PabloAST * createLookahead(PabloAST * expr, PabloAST * shiftAmount);

    PabloAST * createLookahead(PabloAST * expr, const Integer::Type shiftAmount, const std::string prefix);

    PabloAST * createLookahead(PabloAST * expr, PabloAST * shiftAmount, const std::string prefix);

    static inline Zeroes * createZeroes() {
        return &mZeroes;
    }

    static inline Ones * createOnes() {
        return &mOnes;
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

    Assign * createAssign(const std::string && prefix, PabloAST * const expr);

    inline Var * createVar(const std::string name) {
        return createVar(getName(name, false));
    }

    inline Var * createVar(String * name) {
        return createVar(cast<PabloAST>(name));
    }

    Next * createNext(Assign * assign, PabloAST * expr);

    PabloAST * createAnd(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createAnd(PabloAST * expr1, PabloAST * expr2, const std::string prefix);

    And * createAnd(const unsigned reserved);

    And * createAnd(const unsigned reserved, const std::string prefix);

    And * createAnd(std::vector<PabloAST *>::iterator begin, std::vector<PabloAST *>::iterator end) {
        return insertAtInsertionPoint(new And(begin, end, makeName("and_")));
    }

    And * createAnd(Variadic::iterator begin, Variadic::iterator end) {
        return insertAtInsertionPoint(new And(begin, end, makeName("and_")));
    }

    PabloAST * createNot(PabloAST * expr);

    PabloAST * createNot(PabloAST * expr, const std::string prefix);

    PabloAST * createOr(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createOr(PabloAST * expr1, PabloAST * expr2, const std::string prefix);

    Or * createOr(std::vector<PabloAST *>::iterator begin, std::vector<PabloAST *>::iterator end) {
        return insertAtInsertionPoint(new Or(begin, end, makeName("or_")));
    }

    Or * createOr(Variadic::iterator begin, Variadic::iterator end) {
        return insertAtInsertionPoint(new Or(begin, end, makeName("or_")));
    }

    Or * createOr(const unsigned reserved);

    Or * createOr(const unsigned reserved, const std::string prefix);

    PabloAST * createXor(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createXor(PabloAST * expr1, PabloAST * expr2, const std::string prefix);

    Xor * createXor(std::vector<PabloAST *>::iterator begin, std::vector<PabloAST *>::iterator end) {
        return insertAtInsertionPoint(new Xor(begin, end, makeName("xor_")));
    }

    Xor * createXor(Variadic::iterator begin, Variadic::iterator end) {
        return insertAtInsertionPoint(new Xor(begin, end, makeName("xor_")));
    }

    Xor * createXor(const unsigned reserved);

    Xor * createXor(const unsigned reserved, const std::string prefix);

    PabloAST * createMatchStar(PabloAST * marker, PabloAST * charclass);

    PabloAST * createMatchStar(PabloAST * marker, PabloAST * charclass, const std::string prefix);

    PabloAST * createScanThru(PabloAST * from, PabloAST * thru);

    PabloAST * createScanThru(PabloAST * from, PabloAST * thru, const std::string prefix);

    PabloAST * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr);

    PabloAST * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr, const std::string prefix);

    PabloAST * createCount(PabloAST * expr);
    
    PabloAST * createCount(PabloAST * expr, const std::string prefix);
    
    If * createIf(PabloAST * condition, const std::initializer_list<Assign *> definedVars, PabloBlock * body);

    If * createIf(PabloAST * condition, const std::vector<Assign *> & definedVars, PabloBlock * body);

    If * createIf(PabloAST * condition, std::vector<Assign *> && definedVars, PabloBlock * body);

    While * createWhile(PabloAST * condition, const std::initializer_list<Next *> nextVars, PabloBlock * body);

    While * createWhile(PabloAST * condition, const std::vector<Next *> & nextVars, PabloBlock * body);

    While * createWhile(PabloAST * condition, std::vector<Next *> && nextVars, PabloBlock * body);

    inline StatementList & statements() {
        return *this;
    }

    inline const StatementList & statements() const {
        return *this;
    }

    inline String * getName(const std::string name, const bool generated = true) const {
        return mSymbolGenerator->get(name, generated);
    }

    inline String * makeName(const std::string prefix, const bool generated = true) const {
        return mSymbolGenerator->make(prefix, generated);
    }

    inline Integer * getInteger(Integer::Type value) {
        return mSymbolGenerator->getInteger(value);
    }

    inline PabloBlock * getParent() const {
        return mParent;
    }
    
    void setParent(PabloBlock * parent) {
        mParent = parent;
        // Add test to assert this block is in the same function.
    }

    void insert(Statement * const statement);

    unsigned enumerateScopes(unsigned baseScopeIndex);
    
    inline unsigned getScopeIndex() const {
        return mScopeIndex;
    }
    
    void eraseFromParent(const bool recursively = false);

    inline Statement * getBranch() const {
        return mBranch;
    }

    virtual ~PabloBlock();

protected:

    explicit PabloBlock(SymbolGenerator * symbolGenerator) noexcept;

    PabloAST * renameNonNamedNode(PabloAST * expr, const std::string && prefix);

    template<typename Type>
    inline Type * insertAtInsertionPoint(Type * expr) {
        if (isa<Statement>(expr)) {
            if (LLVM_UNLIKELY(isa<If>(expr) || isa<While>(expr))) {
                PabloBlock * const body = isa<If>(expr) ? cast<If>(expr)->getBody() : cast<While>(expr)->getBody();
                body->setParent(this);
                addUser(body);
            }
            insert(cast<Statement>(expr));
        }
        return expr;
    }

    inline void setBranch(Statement * const branch) {
        mBranch = branch;
    }

private:

    Call * createCall(PabloAST * prototype, const std::vector<PabloAST *> &);

    Var * createVar(PabloAST * name);

private:        
    static Zeroes                                       mZeroes;
    static Ones                                         mOnes;
    SymbolGenerator *                                   mSymbolGenerator; // TODO: need a better way of passing a symbol generator around
    PabloBlock *                                        mParent;
    Statement *                                         mBranch; // What statement branches into this scope block?
    unsigned                                            mScopeIndex;
};

}

#endif // PS_PABLOS_H
