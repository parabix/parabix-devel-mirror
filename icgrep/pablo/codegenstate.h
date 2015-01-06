/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_PABLOS_H
#define PS_PABLOS_H

#include <pablo/pabloAST.h>
#include <pablo/pabloAST.h>
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
#include <pablo/pe_string.h>
#include <pablo/pe_var.h>
#include <pablo/pe_xor.h>
#include <pablo/pe_zeroes.h>
#include <pablo/ps_assign.h>
#include <pablo/ps_if.h>
#include <pablo/ps_while.h>
#include <pablo/symbol_generator.h>
#include <map>
#include <vector>
#include <string>
#include <array>
#include <tuple>

namespace pablo {

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

    PabloAST * createCall(const std::string name);

    Assign * createAssign(const std::string prefix, PabloAST * expr, const int outputIndex = -1)  {
        // Note: we cannot just use the findOrMake method to obtain this; an Assign node cannot be considered
        // unique until we prove it has no Next node associated with it. But the Assign node must be created
        // before the Next node. Should we create a "Constant" flag for this?
        Assign * assign = new Assign(mSymbolGenerator->get_ssa(prefix), expr, outputIndex, this);
        push_back(assign);
        return assign;
    }

    Var * createVar(const std::string name);

    Var * createVar(Assign * assign);

    Var * createVar(Next * next);

    inline PabloAST * createVar(PabloAST * const input) {
        switch (input->getClassTypeId()) {
            case PabloAST::ClassTypeId::Assign:
                return createVar(cast<Assign>(input));
            case PabloAST::ClassTypeId::Next:
                return createVar(cast<Next>(input));
            default:
                return input;
        }
    }

    Next * createNext(Assign * assign, PabloAST * expr);

    PabloAST * createAnd(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createNot(PabloAST * expr);

    PabloAST * createOr(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createXor(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createMatchStar(PabloAST * marker, PabloAST * charclass);

    PabloAST * createScanThru(PabloAST * from, PabloAST * thru);

    PabloAST * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr);

    inline If * createIf(PabloAST * condition, std::vector<Assign *> && definedVars, PabloBlock & body) {
        If * statement = new If(condition, std::move(definedVars), body, this);
        push_back(statement);
        return statement;
    }

    inline While * createWhile(PabloAST * cond, PabloBlock & body) {
        While * statement = new While(cond, body, this);
        push_back(statement);
        return statement;
    }

    template<typename... Args>
    struct ExpressionMap {
        enum {N = sizeof...(Args)};
        typedef ExpressionMap<Args...> MapType;
        typedef std::tuple<PabloAST::ClassTypeId, Args...> Key;

        inline ExpressionMap(MapType * predecessor, PabloBlock * parent)
        : mPredecessor(predecessor)
        , mCurrentBlock(*parent)
        {

        }

        template <class Type, typename... Params>
        inline std::pair<Type *, bool> findOrMake(const PabloAST::ClassTypeId type, Args... args, Params... params) {
            Key key = std::make_tuple(type, args...);
            PabloAST * const f = find(key);
            if (f) {
                return std::make_pair(cast<Type>(f), false);
            }
            Type * const expr = new Type(std::forward<Args>(args)..., std::forward<Params>(params)...);
            insert(std::move(key), expr);
            return std::make_pair(expr, true);
        }


        template <class Functor, typename... Params>
        inline std::pair<PabloAST *, bool> findOrCall(const PabloAST::ClassTypeId type, Args... args, Params... params) {
            Key key = std::make_tuple(type, args...);
            PabloAST * const f = find(key);
            if (f) {
                return std::make_pair(f, false);
            }
            Functor mf(mCurrentBlock);
            PabloAST * const expr = mf(std::forward<Args>(args)..., std::forward<Params>(params)...);
            insert(std::move(key), expr);
            return std::make_pair(expr, true);
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
        MapType * const             mPredecessor;
        PabloBlock &                mCurrentBlock;
        std::map<Key, PabloAST *>   mMap;
    };

    inline StatementList & statements() {
        return *this;
    }

    inline const StatementList & statements() const {
        return *this;
    }
protected:
    PabloBlock()
    : mZeroes(new Zeroes())
    , mOnes(new Ones())
    , mSymbolGenerator(new SymbolGenerator())
    , mUnary(nullptr, this)
    , mUnaryWithInt(nullptr, this)
    , mBinary(nullptr, this)
    , mTernary(nullptr, this)
    {

    }

    PabloBlock(PabloBlock * predecessor)
    : mZeroes(predecessor->mZeroes) // inherit the original "Zeroes" variable for simplicity
    , mOnes(predecessor->mOnes) // inherit the original "Ones" variable for simplicity
    , mSymbolGenerator(predecessor->mSymbolGenerator)
    , mUnary(&(predecessor->mUnary), this)
    , mUnaryWithInt(&(predecessor->mUnaryWithInt), this)
    , mBinary(&(predecessor->mBinary), this)
    , mTernary(&(predecessor->mTernary), this)
    {

    }

    void* operator new (std::size_t size) noexcept {
        return PabloAST::mAllocator.allocate(size);
    }
private:        
    Zeroes * const                                      mZeroes;
    Ones * const                                        mOnes;
    SymbolGenerator * const                             mSymbolGenerator;
    ExpressionMap<PabloAST *>                           mUnary;
    ExpressionMap<PabloAST *, int>                      mUnaryWithInt;
    ExpressionMap<PabloAST *, PabloAST *>               mBinary;
    ExpressionMap<PabloAST *, PabloAST *, PabloAST *>   mTernary;
};

}

#endif // PS_PABLOS_H
