#ifndef EXPRESSION_MAP_HPP
#define EXPRESSION_MAP_HPP

#include <pablo/pabloAST.h>
#include <pablo/pablo_intrinsic.h>
#include <util/slab_allocator.h>
#include <llvm/ADT/ArrayRef.h>
#include <map>

namespace pablo {

inline bool operator < (const llvm::ArrayRef<PabloAST *> & A, const llvm::ArrayRef<PabloAST *> & B) {
    return std::lexicographical_compare(A.begin(), A.end(), B.begin(), B.end());
}

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

    PabloAST * find(const PabloAST::ClassTypeId type, Args... args) const noexcept {
        return find(std::make_tuple(type, args...));
    }

    void clear() {
        mMap.clear();
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
            mQuaternary.mPredecessor = &(predecessor->mQuaternary);
        }
    }

    explicit ExpressionTable(ExpressionTable & other) = delete;

    explicit ExpressionTable(ExpressionTable && other) noexcept
    : mUnary(std::move(other.mUnary))
    , mBinary(std::move(other.mBinary))
    , mTernary(std::move(other.mTernary))
    , mQuaternary(std::move(other.mQuaternary)) {

    }

    ExpressionTable & operator=(ExpressionTable && other) noexcept {
        mUnary = std::move(other.mUnary);
        mBinary = std::move(other.mBinary);
        mTernary = std::move(other.mTernary);
        mQuaternary = std::move(other.mQuaternary);
        return *this;
    }

    template <class Functor, typename... Params>
    PabloAST * findUnaryOrCall(Functor && functor, const PabloAST::ClassTypeId typeId, void * expr, Params... params) noexcept {
        return mUnary.findOrCall(std::move(functor), typeId, expr, std::forward<Params>(params)...);
    }

    template <class Functor, typename... Params>
    PabloAST * findBinaryOrCall(Functor && functor, const PabloAST::ClassTypeId typeId, void * expr1, void * expr2, Params... params) noexcept {
        return mBinary.findOrCall(std::move(functor), typeId, expr1, expr2, std::forward<Params>(params)...);
    }

    template <class Functor, typename... Params>
    PabloAST * findTernaryOrCall(Functor && functor, const PabloAST::ClassTypeId typeId, void * expr1, void * expr2, void * expr3, Params... params) noexcept {
        return mTernary.findOrCall(std::move(functor), typeId, expr1, expr2, expr3, std::forward<Params>(params)...);
    }

    template <class Functor, typename... Params>
    PabloAST * findQuaternaryOrCall(Functor && functor, const PabloAST::ClassTypeId typeId, void * expr1, void * expr2, void * expr3, void * expr4, Params... params) noexcept {
        return mQuaternary.findOrCall(std::move(functor), typeId, expr1, expr2, expr3, expr4, std::forward<Params>(params)...);
    }

    template<class Functor, typename... Params>
    PabloAST * findIntrinsicOrCall(Functor && functor, const PabloAST::ClassTypeId typeId, const Intrinsic intrinsic, llvm::ArrayRef<PabloAST *> argv, Params... params) noexcept {
        return mIntrinsic.findOrCall(std::move(functor), typeId, intrinsic, std::move(argv), std::forward<Params>(params)...);
    }

    void clear() {
        mUnary.clear();
        mBinary.clear();
        mTernary.clear();
        mQuaternary.clear();
        mIntrinsic.clear();
    }

    std::pair<PabloAST *, bool> findOrAdd(Statement * stmt) noexcept {
        const auto typeId = stmt->getClassTypeId();
        switch (typeId) {
            case PabloAST::ClassTypeId::Var:
            case PabloAST::ClassTypeId::Not:
            case PabloAST::ClassTypeId::Count:
                return mUnary.findOrAdd(stmt, typeId, stmt->getOperand(0));
            case PabloAST::ClassTypeId::And:
            case PabloAST::ClassTypeId::Or:
            case PabloAST::ClassTypeId::Xor:
            case PabloAST::ClassTypeId::Advance:
            case PabloAST::ClassTypeId::ScanThru:
            case PabloAST::ClassTypeId::MatchStar:
            case PabloAST::ClassTypeId::Assign:
            case PabloAST::ClassTypeId::PackL:
            case PabloAST::ClassTypeId::PackH:
            case PabloAST::ClassTypeId::Extract:
            case PabloAST::ClassTypeId::Repeat:
                return mBinary.findOrAdd(stmt, typeId, stmt->getOperand(0), stmt->getOperand(1));
            case PabloAST::ClassTypeId::Sel:
            case PabloAST::ClassTypeId::IndexedAdvance:
                return mTernary.findOrAdd(stmt, typeId, stmt->getOperand(0), stmt->getOperand(1), stmt->getOperand(2));
            case PabloAST::ClassTypeId::Ternary:
                return mQuaternary.findOrAdd(stmt, typeId, stmt->getOperand(0), stmt->getOperand(1), stmt->getOperand(2), stmt->getOperand(3));
            case PabloAST::ClassTypeId::IntrinsicCall:
                return mIntrinsic.findOrAdd(stmt, typeId, llvm::cast<IntrinsicCall>(stmt)->getIntrinsic(), llvm::cast<IntrinsicCall>(stmt)->getArgv());
            default:
                return std::make_pair(stmt, true);
        }
    }

private:
    FixedArgMap<void *>                                 mUnary;
    FixedArgMap<void *, void *>                         mBinary;
    FixedArgMap<void *, void *, void *>                 mTernary;
    FixedArgMap<void *, void *, void *, void *>         mQuaternary;
    FixedArgMap<Intrinsic, llvm::ArrayRef<PabloAST *>>  mIntrinsic;
};

}

#endif // EXPRESSION_MAP_HPP
