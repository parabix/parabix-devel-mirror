#ifndef EXPRESSION_MAP_HPP
#define EXPRESSION_MAP_HPP

#include <pablo/pabloAST.h>
#include <pablo/pablo_intrinsic.h>
#include <util/slab_allocator.h>
#include <llvm/ADT/ArrayRef.h>
#include <map>

namespace pablo {

template<typename T>
inline bool operator < (const llvm::ArrayRef<T> & A, const llvm::ArrayRef<T> & B) {
    return std::lexicographical_compare(A.begin(), A.end(), B.begin(), B.end());
}

template <typename T>
inline void __byval(llvm::ArrayRef<T> & t, ProxyAllocator<uint8_t> & alloc) noexcept {
    ProxyAllocator<T> A{alloc};
    t = t.copy(A);
}

template <typename T>
inline void __byval(T &, ProxyAllocator<uint8_t> &) noexcept { }

template<unsigned I, typename Tuple>
struct __make_byval_impl {
    static void doit(Tuple & t, ProxyAllocator<uint8_t> & alloc) noexcept {
        __make_byval_impl<I - 1, Tuple>::doit(t, alloc);
        __byval(std::get<I>(t), alloc);
    }
};

template<typename Tuple>
struct __make_byval_impl<0, Tuple> {
    static void doit(Tuple & t, ProxyAllocator<uint8_t> & alloc) noexcept {
        __byval(std::get<0>(t), alloc);
    }
};

template<typename Tuple>
inline Tuple & make_byval(Tuple & t, ProxyAllocator<uint8_t> & alloc) noexcept {
    __make_byval_impl<std::tuple_size<Tuple>::value - 1, Tuple>::doit(t, alloc);
    return t;
}

template<typename... Args>
struct FixedArgMap {
    enum {N = sizeof...(Args)};
    friend struct ExpressionTable;

    using Type = FixedArgMap<Args...>;
    using Key = std::tuple<PabloAST::ClassTypeId, Args...>;
    using Allocator = SlabAllocator<uint8_t>;
    using MapAllocator = ProxyAllocator<std::pair<Key, PabloAST *>>;
    using Map = std::map<Key, PabloAST *, std::less<Key>, MapAllocator>;

    explicit FixedArgMap(Allocator & allocator, const Type * predecessor = nullptr) noexcept
    : mPredecessor(predecessor)
    , mMap(MapAllocator{allocator}) {

    }

    explicit FixedArgMap(Type && other, Allocator & allocator) noexcept
    : mPredecessor(other.mPredecessor)
    , mMap(MapAllocator{allocator}) {
        assert (other.mMap.empty());
    }

    FixedArgMap & operator=(Type && other) noexcept = delete;

    template <class Functor, typename... Params>
    PabloAST * findOrCall(Functor && functor, const PabloAST::ClassTypeId typeId, Args... args, Params... params) noexcept {
        Key key = std::make_tuple(typeId, args...);
        PabloAST * const f = find(key);
        if (f) {
            return f;
        }
        PabloAST * const object = functor(std::forward<Args>(args)..., std::forward<Params>(params)...);
        insert(std::move(key), object);
        return object;
    }

    std::pair<PabloAST *, bool> findOrAdd(PabloAST * object, const PabloAST::ClassTypeId typeId, Args... args) noexcept {
        Key key = std::make_tuple(typeId, args...);
        PabloAST * const entry = find(key);
        if (entry) {
            return std::make_pair(entry, false);
        }
        insert(std::move(key), object);
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

    void insert(Key && key, PabloAST * const object) noexcept {
        ProxyAllocator<uint8_t> alloc{mMap.get_allocator()};
        mMap.insert(std::make_pair(std::move(make_byval(key, alloc)), object));
    }

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
    const Type * const mPredecessor;
    Map                mMap;
};

struct ExpressionTable {

    using Allocator = SlabAllocator<uint8_t>;
    using UnaryT = FixedArgMap<void *>;
    using BinaryT = FixedArgMap<void *, void *>;
    using TernaryT = FixedArgMap<void *, void *, void *>;
    using QuaternaryT = FixedArgMap<void *, void *, void *, void *>;
    using IntrinsicT = FixedArgMap<Intrinsic, llvm::ArrayRef<PabloAST *>>;

    #define CON(Type) m##Type(mAllocator, predecessor ? &(predecessor->m##Type) : nullptr)
    explicit ExpressionTable(ExpressionTable * predecessor = nullptr) noexcept
    : CON(Unary)
    , CON(Binary)
    , CON(Ternary)
    , CON(Quaternary)
    , CON(Intrinsic) {

    }
    #undef CON

    ExpressionTable(ExpressionTable & other) = delete;

    #define MOVE(Type) m##Type(std::move(other.m##Type), mAllocator)
    ExpressionTable(ExpressionTable && other)
    : MOVE(Unary)
    , MOVE(Binary)
    , MOVE(Ternary)
    , MOVE(Quaternary)
    , MOVE(Intrinsic) {

    }
    #undef MOVE

    ExpressionTable & operator=(ExpressionTable && other) = delete;

    // NOTE: this deconstructor is *required* to ensure the correct order of "deleting"
    // the internal maps before releasing the slab(s).
    ~ExpressionTable() noexcept { clear(); }

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
        mAllocator.Reset();
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

    UnaryT          mUnary;
    BinaryT         mBinary;
    TernaryT        mTernary;
    QuaternaryT     mQuaternary;
    IntrinsicT      mIntrinsic;
    Allocator       mAllocator;
};

}

#endif // EXPRESSION_MAP_HPP
