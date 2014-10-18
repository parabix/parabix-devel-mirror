/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_PABLOS_H
#define PS_PABLOS_H

#include <pablo/pabloAST.h>
#include <pablo/pe_string.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_and.h>
#include <pablo/pe_call.h>
#include <pablo/pe_charclass.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_not.h>
#include <pablo/pe_or.h>
#include <pablo/pabloAST.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_sel.h>
#include <pablo/pe_var.h>
#include <pablo/pe_xor.h>
#include <pablo/ps_assign.h>
#include <pablo/ps_if.h>
#include <pablo/ps_while.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <pablo/symbol_generator.h>
#include <map>
#include <vector>
#include <string>
#include <array>
#include <tuple>

namespace pablo {

class PabloBlock {
public:

    PabloBlock(SymbolGenerator & symgen)
    : mSymbolGenerator(symgen)
    , mZeroes(new Zeroes())
    , mOnes(new Ones())
    , mUnary(nullptr, this)
    , mBinary(nullptr, this)
    , mTernary(nullptr, this)
    {

    }

    PabloBlock(PabloBlock & cg)
    : mSymbolGenerator(cg.mSymbolGenerator)
    , mZeroes(cg.mZeroes) // inherit the original "All" variables for simplicity
    , mOnes(cg.mOnes) // inherit the original "All" variables for simplicity
    , mUnary(&(cg.mUnary), this)
    , mBinary(&(cg.mBinary), this)
    , mTernary(&(cg.mTernary), this)
    {

    }

    Advance * createAdvance(PabloAST * expr);

    inline Zeroes * createZeroes() const {
        return mZeroes;
    }

    inline Ones * createOnes() const {
        return mOnes;
    }

    Assign * createAssign(const std::string name, PabloAST * expr);

    Call * createCall(const std::string name);

    Var * createVar(const std::string name);

    Var * createVar(Assign * assign);

    inline PabloAST * createVarIfAssign(PabloAST * const input) {
        return isa<Assign>(input) ? createVar(cast<Assign>(input)) : input;
    }

    CharClass * createCharClass(const std::string name);

    PabloAST * createAnd(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createNot(PabloAST * expr);

    PabloAST * createOr(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createXor(PabloAST * expr1, PabloAST * expr2);

    MatchStar * createMatchStar(PabloAST * expr1, PabloAST * expr2);

    ScanThru * createScanThru(PabloAST * from, PabloAST * thru);

    PabloAST * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr);

    inline If * createIf(PabloAST * condition, PabloBlock && body) {
        If * statement = new If(condition, std::move(body.mStatements));
        mStatements.push_back(statement);
        return statement;
    }

    inline While * createWhile(PabloAST * cond, PabloBlock && body) {
        While * statement = new While(cond, std::move(body.mStatements));
        mStatements.push_back(statement);
        return statement;
    }

    template<typename... Args>
    struct ExpressionMap {
        typedef ExpressionMap<Args...> MapType;
        typedef std::tuple<PabloAST::ClassTypeId, Args...> Key;

        inline ExpressionMap(MapType * predecessor, PabloBlock * parent)
        : mPredecessor(predecessor)
        , mCodeGenState(*parent)
        {

        }

        template <class Type>
        inline Type * findOrMake(const PabloAST::ClassTypeId type, Args... args) {
            auto key = std::make_tuple(type, args...);
            PabloAST * f = find(key);
            if (f) {
                return cast<Type>(f);
            }
            Type * expr = new Type(args...);
            insert(std::move(key), expr);
            return expr;
        }

        template <class Functor>
        inline PabloAST * findOrCall(const PabloAST::ClassTypeId type, Args... args) {
            auto key = std::make_tuple(type, args...);
            PabloAST * f = find(key);
            if (f) {
                return f;
            }
            Functor mf(mCodeGenState);
            PabloAST * expr = mf(args...);            
            insert(std::move(key), expr);
            return expr;
        }

        inline void insert(Key && key, PabloAST * expr) {
            mMap.insert(std::make_pair(std::move(key), expr));
        }

        inline PabloAST * find(const Key & key) const {
            // check this map to see if we have it
            auto itr = mMap.find(key);
            if (itr != mMap.end()) {
                return itr->second;
            }
            // check any previous maps to see if it exists
            auto * pred = mPredecessor;
            while (pred) {
                itr = pred->mMap.find(key);
                if (itr == pred->mMap.end()) {
                    pred = pred->mPredecessor;
                    continue;
                }
                return itr->second;
            }
            return nullptr;
        }

    private:
        MapType * const         mPredecessor;
        PabloBlock &            mCodeGenState;
        std::map<Key, PabloAST *> mMap;
    };

    inline std::string ssa(std::string prefix) { // Static Single-Assignment
        return mSymbolGenerator.ssa(prefix);
    }

    inline const ExpressionList & expressions() const {
        return mStatements;
    }

private:    
    SymbolGenerator &                               mSymbolGenerator;
    Zeroes *                     		    mZeroes;
    Ones *                     		    mOnes;
    ExpressionMap<PabloAST *>                         mUnary;
    ExpressionMap<PabloAST *, PabloAST *>               mBinary;
    ExpressionMap<PabloAST *, PabloAST *, PabloAST *>     mTernary;
    ExpressionList                                  mStatements;
};

}

#endif // PS_PABLOS_H
