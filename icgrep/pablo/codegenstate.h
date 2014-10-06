/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_PABLOS_H
#define PS_PABLOS_H

#include <pablo/pe_pabloe.h>
#include <pablo/pe_string.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_all.h>
#include <pablo/pe_and.h>
#include <pablo/pe_call.h>
#include <pablo/pe_charclass.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_not.h>
#include <pablo/pe_or.h>
#include <pablo/pe_pabloe.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_sel.h>
#include <pablo/pe_var.h>
#include <pablo/pe_xor.h>
#include <pablo/ps_assign.h>
#include <pablo/ps_if.h>
#include <pablo/ps_while.h>
#include <pablo/symbol_generator.h>
#include <unordered_map>
#include <vector>
#include <string>
#include <array>
#include <tuple>

namespace pablo {

struct CodeGenState {

    CodeGenState(SymbolGenerator & symgen)
    : mSymbolGenerator(symgen)
    , mAll{{new All(0), new All(1)}}
    , mUnary(nullptr, this)
    , mBinary(nullptr, this)
    , mTernary(nullptr, this)
    {

    }

    CodeGenState(CodeGenState & cg)
    : mSymbolGenerator(cg.mSymbolGenerator)
    , mAll(cg.mAll)    // inherit the original "All" variables for simplicity
    , mUnary(&(cg.mUnary), this)
    , mBinary(&(cg.mBinary), this)
    , mTernary(&(cg.mTernary), this)
    {

    }

    Advance * createAdvance(PabloE * expr);

    inline All * createAll(const bool value) const {
        return mAll[value];
    }

    Assign * createAssign(const std::string name, PabloE * expr);

    Call * createCall(const std::string name);

    Var * createVar(const std::string name);

    Var * createVar(Assign * assign);

    inline PabloE * createVarIfAssign(PabloE * const input) {
        return isa<Assign>(input) ? createVar(cast<Assign>(input)) : input;
    }

    CharClass * createCharClass(const std::string name);

    PabloE * createAnd(PabloE * expr1, PabloE * expr2);

    PabloE * createNot(PabloE * expr);

    PabloE * createOr(PabloE * expr1, PabloE * expr2);

    PabloE * createXor(PabloE * expr1, PabloE * expr2);

    MatchStar * createMatchStar(PabloE * expr1, PabloE * expr2);

    ScanThru * createScanThru(PabloE * from, PabloE * thru);

    PabloE * createSel(PabloE * condition, PabloE * trueExpr, PabloE * falseExpr);

    inline If * createIf(PabloE * condition, ExpressionList statements) {
        return new If(condition, std::move(statements));
    }

    inline While * createWhile(PabloE * cond, ExpressionList statements) {
        return new While(cond, std::move(statements));
    }

    template<typename... Args>
    struct ExpressionMap {
        typedef ExpressionMap<Args...> MapType;
        typedef std::tuple<PabloE::ClassTypeId, Args...> Key;

        inline ExpressionMap(MapType * predecessor, CodeGenState * parent)
        : mPredecessor(predecessor)
        , mCodeGenState(*parent)
        {

        }

        template <class Type>
        inline Type * findOrMake(const PabloE::ClassTypeId type, Args... args) {
            auto key = std::make_tuple(type, args...);
            PabloE * f = find(key);
            if (f) {
                return cast<Type>(f);
            }
            Type * expr = new Type(args...);
            mMap.insert(std::make_pair(std::move(key), expr));
            return expr;
        }

        template <class Functor>
        inline PabloE * findOrCall(const PabloE::ClassTypeId type, Args... args) {
            auto key = std::make_tuple(type, args...);
            PabloE * f = find(key);
            if (f) {
                return f;
            }
            Functor mf(mCodeGenState);
            PabloE * expr = mf(args...);
            mMap.insert(std::make_pair(std::move(key), expr));
            return expr;
        }

    private:

        inline PabloE * find(const Key & key) const {
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
        CodeGenState &          mCodeGenState;
        std::map<Key, PabloE *> mMap;
    };

    inline void push_back(PabloE * expr) {
        mExpressions.push_back(expr);
    }

    inline std::string ssa(std::string prefix) { // Static Single-Assignment
        return mSymbolGenerator.ssa(prefix);
    }

    inline const ExpressionList & expressions() const {
        return mExpressions;
    }

private:    
    SymbolGenerator &                               mSymbolGenerator;
    const std::array<All *, 2>                      mAll;
    ExpressionMap<PabloE *>                         mUnary;
    ExpressionMap<PabloE *, PabloE *>               mBinary;
    ExpressionMap<PabloE *, PabloE *, PabloE *>     mTernary;
    ExpressionList                                  mExpressions;
};

}

#endif // PS_PABLOS_H
