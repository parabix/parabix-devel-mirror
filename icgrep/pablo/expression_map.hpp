#ifndef EXPRESSION_MAP_HPP
#define EXPRESSION_MAP_HPP

#include <map>
#include <tuple>
#include <pablo/pabloAST.h>

namespace pablo {

template<typename... Args>
struct ExpressionMap {
    enum {N = sizeof...(Args)};
    typedef ExpressionMap<Args...> MapType;
    typedef std::tuple<PabloAST::ClassTypeId, Args...> Key;

    inline ExpressionMap(MapType * predecessor)
    : mPredecessor(predecessor)
    {

    }

    template <class Functor, typename... Params>
    inline PabloAST * findOrCall(Functor && functor, const PabloAST::ClassTypeId type, Args... args, Params... params) {
        Key key = std::make_tuple(type, args...);
        PabloAST * const f = find(key);
        if (f) {
            return f;
        }
        PabloAST * const object = functor(std::forward<Args>(args)..., std::forward<Params>(params)...);
        mMap.insert(std::make_pair(std::move(key), object));
        return object;
    }

    inline std::pair<PabloAST *, bool> findOrAdd(PabloAST * object, const PabloAST::ClassTypeId type, Args... args) {
        Key key = std::make_tuple(type, args...);
        PabloAST * const entry = find(key);
        if (entry) {
            return std::make_pair(entry, false);
        }
        mMap.insert(std::make_pair(std::move(key), object));
        return std::make_pair(object, true);
    }

    inline bool erase(const PabloAST::ClassTypeId type, Args... args) {
        Key key = std::make_tuple(type, args...);
        auto itr = mMap.find(key);
        if (itr == mMap.end()) {
            return mPredecessor ? mPredecessor->erase(type, args...) : false;
        }
        mMap.erase(itr);
        return true;
    }

    inline PabloAST * find(const PabloAST::ClassTypeId type, Args... args) const {
        Key key = std::make_tuple(type, args...);
        return find(key);
    }

private:

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
    std::map<Key, PabloAST *>   mMap;
};


struct ExpressionTable {

    ExpressionTable()
    : mUnary(nullptr)
    , mBinary(nullptr)
    , mTernary(nullptr)
    {

    }

    ExpressionTable(ExpressionTable * predecessor)
    : mUnary(predecessor ? &(predecessor->mUnary) : nullptr)
    , mBinary(predecessor ? &(predecessor->mBinary) : nullptr)
    , mTernary(predecessor ? &(predecessor->mTernary) : nullptr)
    {

    }

    template <class Functor, typename... Params>
    inline PabloAST * findUnaryOrCall(Functor && functor, const PabloAST::ClassTypeId type, PabloAST * expr, Params... params) {
        return mUnary.findOrCall(std::move(functor), type, expr, std::forward<Params>(params)...);
    }

    template <class Functor, typename... Params>
    inline PabloAST * findBinaryOrCall(Functor && functor, const PabloAST::ClassTypeId type, PabloAST * expr1, PabloAST * expr2, Params... params) {
        return mBinary.findOrCall(std::move(functor), type, expr1, expr2, std::forward<Params>(params)...);
    }

    template <class Functor, typename... Params>
    inline PabloAST * findTernaryOrCall(Functor && functor, const PabloAST::ClassTypeId type, PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, Params... params) {
        return mTernary.findOrCall(std::move(functor), type, expr1, expr2, expr3, std::forward<Params>(params)...);
    }

    std::pair<PabloAST *, bool> findOrAdd(Statement * stmt) {
        switch (stmt->getClassTypeId()) {            
            case PabloAST::ClassTypeId::Assign:
            case PabloAST::ClassTypeId::Call:
            case PabloAST::ClassTypeId::Var:
            case PabloAST::ClassTypeId::Not:
                return mUnary.findOrAdd(stmt, stmt->getClassTypeId(), stmt->getOperand(0));
            case PabloAST::ClassTypeId::And:
            case PabloAST::ClassTypeId::Or:
            case PabloAST::ClassTypeId::Xor:
                // test whether the communative version of this statement exists
                if (PabloAST * commExpr = mBinary.find(stmt->getClassTypeId(), stmt->getOperand(1), stmt->getOperand(0))) {
                    return std::make_pair(commExpr, false);
                }
            case PabloAST::ClassTypeId::Advance:
            case PabloAST::ClassTypeId::ScanThru:
            case PabloAST::ClassTypeId::MatchStar:
            case PabloAST::ClassTypeId::Next:
                return mBinary.findOrAdd(stmt, stmt->getClassTypeId(), stmt->getOperand(0), stmt->getOperand(1));
            case PabloAST::ClassTypeId::Sel:
                return mTernary.findOrAdd(stmt, stmt->getClassTypeId(), stmt->getOperand(0), stmt->getOperand(1), stmt->getOperand(2));
            default:
                return std::make_pair(stmt, true);
        }
    }


private:
    ExpressionMap<PabloAST *>                           mUnary;
    ExpressionMap<PabloAST *, PabloAST *>               mBinary;
    ExpressionMap<PabloAST *, PabloAST *, PabloAST *>   mTernary;
};

}

#endif // EXPRESSION_MAP_HPP
