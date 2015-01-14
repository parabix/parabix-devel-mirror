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


class PabloBlock : public StatementList {
    friend class pablo::PabloAST;
public:

    inline static PabloBlock & Create() {
        return *(new PabloBlock());
    }

    inline static PabloBlock & Create(PabloBlock & predecessor) {
        return *(new PabloBlock(&predecessor));
    }

    PabloAST * createAdvance(PabloAST * expr, const int shiftAmount);

    inline Zeroes * createZeroes() const {
        return mZeroes;
    }

    inline Ones * createOnes() const {
        return mOnes;
    }

    Call * createCall(const std::string name);

    Call * createCall(String * name);

    Assign * createAssign(const std::string prefix, PabloAST * expr, const int outputIndex = -1);

    Var * createVar(const std::string name);

    Var * createVar(String * name);

    PabloAST * createVar(const PabloAST * const) {
        throw std::runtime_error("Var objects should only refer to external Vars (i.e., input basis bit streams). Use Assign objects directly.");
    }

    Next * createNext(Assign * assign, PabloAST * expr);

    PabloAST * createAnd(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createNot(PabloAST * expr);

    PabloAST * createOr(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createXor(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createMatchStar(PabloAST * marker, PabloAST * charclass);

    PabloAST * createScanThru(PabloAST * from, PabloAST * thru);

    PabloAST * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr);

    If * createIf(PabloAST * condition, std::vector<Assign *> && definedVars, PabloBlock & body);

    While * createWhile(PabloAST * cond, PabloBlock & body);

    inline StatementList & statements() {
        return *this;
    }

    inline const StatementList & statements() const {
        return *this;
    }

    inline String * getName(const std::string name) const {
        return mSymbolGenerator->get(name);
    }

    inline String * makeName(const std::string prefix) const {
        return mSymbolGenerator->make(prefix);
    }

protected:
    PabloBlock();

    PabloBlock(PabloBlock * predecessor);

    void * operator new (std::size_t size) noexcept {
        return PabloAST::mAllocator.allocate(size);
    }

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
};

}

#endif // PS_PABLOS_H
