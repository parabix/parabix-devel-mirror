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
#include <pablo/ps_assign.h>
#include <pablo/ps_if.h>
#include <pablo/ps_while.h>
#include <stdexcept>

namespace pablo {

class Assign;
class Advance;
class And;
class Call;
class MatchStar;
class Next;
class Not;
class Or;
class Ones;
class ScanThru;
class Sel;
class String;
class Integer;
class Var;
class Xor;
class Zeroes;
class If;
class While;


class PabloBlock : public PabloAST, public StatementList {
    friend class pablo::PabloAST;
    friend class Builder;
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

    inline static PabloBlock & Create() {
        return *(new PabloBlock());
    }

    inline static PabloBlock & Create(PabloBlock & predecessor) {
        return *(new PabloBlock(&predecessor));
    }

    PabloAST * createAdvance(PabloAST * expr, const int shiftAmount);

    PabloAST * createAdvance(PabloAST * expr, PabloAST * shiftAmount);

    PabloAST * createAdvance(PabloAST * expr, const int shiftAmount, const std::string prefix);

    PabloAST * createAdvance(PabloAST * expr, PabloAST * shiftAmount, const std::string prefix);

    inline Zeroes * createZeroes() const {
        return mZeroes;
    }

    inline Ones * createOnes() const {
        return mOnes;
    }

    inline Call * createCall(const std::string name) {
        return createCall(getName(name, false));
    }

    Call * createCall(String * name);

    Assign * createAssign(const std::string prefix, PabloAST * expr, const int outputIndex = -1);

    inline Var * createVar(const std::string name) {
        return createVar(getName(name, false));
    }

    Var * createVar(String * name);

    PabloAST * createVar(const PabloAST * const) {
        throw std::runtime_error("Var objects should only refer to external Vars (i.e., input basis bit streams). Use Assign objects directly.");
    }

    Next * createNext(Assign * assign, PabloAST * expr);

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

    If * createIf(PabloAST * condition, std::vector<Assign *> && definedVars, PabloBlock & body);

    While * createWhile(PabloAST * condition, PabloBlock & body);

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

    virtual ~PabloBlock();

protected:
    PabloBlock();

    PabloBlock(PabloBlock * predecessor);

    PabloAST * renameNonNamedNode(PabloAST * expr, const std::string && prefix);

    template<typename Type>
    inline Type * insertAtInsertionPoint(Type * expr) {
        if (isa<Statement>(expr)) {
            insert(cast<Statement>(expr));
        }
        return expr;
    }

private:        
    Zeroes * const                                      mZeroes;
    Ones * const                                        mOnes;
    SymbolGenerator * const                             mSymbolGenerator;
    PabloBlock *                                        mPredecessor;
};

}

#endif // PS_PABLOS_H
