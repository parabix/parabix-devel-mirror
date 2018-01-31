#ifndef EXPRESSION_MAP_HPP
#define EXPRESSION_MAP_HPP

#include <pablo/pabloAST.h>
#include <util/slab_allocator.h>
#include <map>

namespace pablo {

template<typename... Args>
struct FixedArgMap {
    enum {N = sizeof...(Args)};
    typedef FixedArgMap<Args...> Type;
    typedef std::tuple<PabloAST::ClassTypeId, Args...> Key;
    friend struct ExpressionTable;

    explicit FixedArgMap(const Type * predecessor = nullptr) noexcept
    : mPredecessor(predecessor) {

    }

    explicit FixedArgMap(Type && other) noexcept
    : mPredecessor(other.mPredecessor)
    , mMap(std::move(other.mMap)) {

    }

    FixedArgMap & operator=(Type && other) noexcept {
        mPredecessor = other.mPredecessor;
        mMap = std::move(other.mMap);
        return *this;
    }

    template <class Functor, typename... Params>
    PabloAST * findOrCall(Functor && functor, const PabloAST::ClassTypeId typeId, Args... args, Params... params) noexcept {
        Key key = std::make_tuple(typeId, args...);
        PabloAST * const f = find(key);
        if (f) {
            return f;
        }
        PabloAST * const object = functor(std::forward<Args>(args)..., std::forward<Params>(params)...);
        mMap.insert(std::make_pair(std::move(key), object));
        return object;
    }

    std::pair<PabloAST *, bool> findOrAdd(PabloAST * object, const PabloAST::ClassTypeId typeId, Args... args) noexcept {
        Key key = std::make_tuple(typeId, args...);
        PabloAST * const entry = find(key);
        if (entry) {
            return std::make_pair(entry, false);
        }
        mMap.insert(std::make_pair(std::move(key), object));
        return std::make_pair(object, true);
    }

    bool erase(const PabloAST::ClassTypeId type, Args... args) noexcept {
        Key key = std::make_tuple(type, args...);
        for (Type * obj = this; obj; obj = obj->mPredecessor) {
            auto itr = obj->mMap.find(key);
            if (itr != mMap.end()) {
                obj->mMap.erase(itr);
                return true;
            }
        }
        return false;
    }

    inline PabloAST * find(const PabloAST::ClassTypeId type, Args... args) const noexcept {
        return find(std::make_tuple(type, args...));
    }

private:

    PabloAST * find(const Key & key) const {
        // check this map to see if we have it
        auto itr = mMap.find(key);
        if (itr != mMap.end()) {
            return itr->second;
        } else { // check any previous maps to see if it exists
            auto * pred = mPredecessor;
            while (pred) {
                itr = pred->mMap.find(key);
                if (itr == pred->mMap.end()) {
                    pred = pred->mPredecessor;
                    continue;
                }
                return itr->second;
            }
        }
        return nullptr;
    }

private:
    const Type *                mPredecessor;
    std::map<Key, PabloAST *>   mMap;
};

struct ExpressionTable {

    explicit ExpressionTable(ExpressionTable * predecessor = nullptr) noexcept {
        if (predecessor) {
            mUnary.mPredecessor = &(predecessor->mUnary);
            mBinary.mPredecessor = &(predecessor->mBinary);
            mTernary.mPredecessor = &(predecessor->mTernary);
        }
    }

    explicit ExpressionTable(ExpressionTable & other) = delete;

    explicit ExpressionTable(ExpressionTable && other) noexcept
    : mUnary(std::move(other.mUnary))
    , mBinary(std::move(other.mBinary))
    , mTernary(std::move(other.mTernary)) {

    }

    ExpressionTable & operator=(ExpressionTable && other) noexcept {
        mUnary = std::move(other.mUnary);
        mBinary = std::move(other.mBinary);
        mTernary = std::move(other.mTernary);
        return *this;
    }

    template <class Functor, typename... Params>
    inline PabloAST * findUnaryOrCall(Functor && functor, const PabloAST::ClassTypeId typeId, void * expr, Params... params) noexcept {
        return mUnary.findOrCall(std::move(functor), typeId, expr, std::forward<Params>(params)...);
    }

    template <class Functor, typename... Params>
    inline PabloAST * findBinaryOrCall(Functor && functor, const PabloAST::ClassTypeId typeId, void * expr1, void * expr2, Params... params) noexcept {
        return mBinary.findOrCall(std::move(functor), typeId, expr1, expr2, std::forward<Params>(params)...);
    }

    template <class Functor, typename... Params>
    inline PabloAST * findTernaryOrCall(Functor && functor, const PabloAST::ClassTypeId typeId, void * expr1, void * expr2, void * expr3, Params... params) noexcept {
        return mTernary.findOrCall(std::move(functor), typeId, expr1, expr2, expr3, std::forward<Params>(params)...);
    }

    std::pair<PabloAST *, bool> findOrAdd(Statement * stmt) noexcept {
        switch (stmt->getClassTypeId()) {                     
            case PabloAST::ClassTypeId::Var:
            case PabloAST::ClassTypeId::Not:
            case PabloAST::ClassTypeId::Count:            
                return mUnary.findOrAdd(stmt, stmt->getClassTypeId(), stmt->getOperand(0));
            case PabloAST::ClassTypeId::And:
            case PabloAST::ClassTypeId::Or:
            case PabloAST::ClassTypeId::Xor:
            case PabloAST::ClassTypeId::Advance:
            case PabloAST::ClassTypeId::ScanThru:
            case PabloAST::ClassTypeId::MatchStar:
            case PabloAST::ClassTypeId::Assign:
            case PabloAST::ClassTypeId::Extract:
            case PabloAST::ClassTypeId::Repeat:
                return mBinary.findOrAdd(stmt, stmt->getClassTypeId(), stmt->getOperand(0), stmt->getOperand(1));
            case PabloAST::ClassTypeId::Sel:
            case PabloAST::ClassTypeId::IndexedAdvance:
                return mTernary.findOrAdd(stmt, stmt->getClassTypeId(), stmt->getOperand(0), stmt->getOperand(1), stmt->getOperand(2));
            default:
                return std::make_pair(stmt, true);
        }
    }

private:
    FixedArgMap<void *>                   mUnary;
    FixedArgMap<void *, void *>           mBinary;
    FixedArgMap<void *, void *, void *>   mTernary;
};

}

#endif // EXPRESSION_MAP_HPP
