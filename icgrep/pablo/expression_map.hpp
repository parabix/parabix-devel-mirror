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

    inline std::pair<PabloAST *, bool> insert(PabloAST * object, const PabloAST::ClassTypeId type, Args... args) {
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
    , mUnaryWithInt(nullptr)
    , mBinary(nullptr)
    , mTernary(nullptr)
    {

    }

    ExpressionTable(ExpressionTable * predecessor)
    : mUnary(predecessor ? &(predecessor->mUnary) : nullptr)
    , mUnaryWithInt(predecessor ? &(predecessor->mUnaryWithInt) : nullptr)
    , mBinary(predecessor ? &(predecessor->mBinary) : nullptr)
    , mTernary(predecessor ? &(predecessor->mTernary) : nullptr)
    {

    }

    std::pair<PabloAST *, bool> insert(Statement * stmt) {
        switch (stmt->getClassTypeId()) {
            case PabloAST::ClassTypeId::Advance:
                return mUnaryWithInt.insert(stmt, stmt->getClassTypeId(), stmt->getOperand(0), cast<Advance>(stmt)->getAdvanceAmount());
            case PabloAST::ClassTypeId::Assign:
            case PabloAST::ClassTypeId::Call:
            case PabloAST::ClassTypeId::Var:
            case PabloAST::ClassTypeId::Not:
                return mUnary.insert(stmt, stmt->getClassTypeId(), stmt->getOperand(0));
            case PabloAST::ClassTypeId::And:
            case PabloAST::ClassTypeId::Or:
            case PabloAST::ClassTypeId::Xor:
                // test whether the communative version of this statement exists
                if (PabloAST * commExpr = mBinary.find(stmt->getClassTypeId(), stmt->getOperand(1), stmt->getOperand(0))) {
                    return std::make_pair(commExpr, false);
                }
            case PabloAST::ClassTypeId::ScanThru:
            case PabloAST::ClassTypeId::MatchStar:
            case PabloAST::ClassTypeId::Next:
                return mBinary.insert(stmt, stmt->getClassTypeId(), stmt->getOperand(0), stmt->getOperand(1));
            case PabloAST::ClassTypeId::Sel:
                return mTernary.insert(stmt, stmt->getClassTypeId(), stmt->getOperand(0), stmt->getOperand(1), stmt->getOperand(2));
            default:
                return std::make_pair(stmt, true);
        }
    }


private:
    ExpressionMap<PabloAST *>                           mUnary;
    ExpressionMap<PabloAST *, int>                      mUnaryWithInt;
    ExpressionMap<PabloAST *, PabloAST *>               mBinary;
    ExpressionMap<PabloAST *, PabloAST *, PabloAST *>   mTernary;
};

}

#endif // EXPRESSION_MAP_HPP
