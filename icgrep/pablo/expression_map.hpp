#ifndef EXPRESSION_MAP_HPP
#define EXPRESSION_MAP_HPP

#include <pablo/pabloAST.h>
#include <pablo/pablo_intrinsic.h>
#include <util/slab_allocator.h>
#include <llvm/ADT/ArrayRef.h>
#include <type_traits>
#include <map>

namespace pablo {

template<typename T>
inline bool operator < (const llvm::ArrayRef<T> & A, const llvm::ArrayRef<T> & B) noexcept {
    return std::lexicographical_compare(A.begin(), A.end(), B.begin(), B.end());
}

namespace { // byval

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

template <typename T, typename = typename std::enable_if<!std::is_same<T, PabloAST *>::value, T>::type>
inline bool __is_var(const T) noexcept {
    return false;
}

template <typename T, typename = T>
inline bool __is_var(const llvm::ArrayRef<T> & A) noexcept {
    for (const T & a : A) {
        if (__is_var<T, T>(a)) return true;
    }
    return false;
}

template <typename = PabloAST *>
inline bool __is_var(const PabloAST * const expr) noexcept {
    return expr->getClassTypeId() == PabloAST::ClassTypeId::Var;
}

template<unsigned I, typename Tuple>
struct __contains_var_impl {
    static bool doit(const Tuple & t) noexcept {
        return __contains_var_impl<I - 1, Tuple>::doit(t) || __is_var(std::get<I>(t));
    }
};

template<typename Tuple>
struct __contains_var_impl<0, Tuple> {
    static bool doit(const Tuple & t) noexcept {
        return __is_var(std::get<0>(t));
    }
};

} // end of anonymous namespace

template<typename... Args>
struct FixedArgMap {
    enum {N = sizeof...(Args)};
    friend struct ExpressionTable;

    using Type = FixedArgMap<Args...>;
    using TypeId = PabloAST::ClassTypeId;
    using Key = std::tuple<TypeId, Args...>;
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
        // This is called due to RVO when returning a new nested builder.
        // If the map is not empty, it's an error.
        assert (other.mMap.empty());
    }

    FixedArgMap & operator=(Type && other) noexcept = delete;

    template <class Functor, typename... Params>
    PabloAST * findOrCall(Functor && functor, const TypeId typeId, Args... args, Params... params) noexcept {
        Key key = std::make_tuple(typeId, args...);
        PabloAST * const f = find(key);
        if (f) {
            return f;
        }
        PabloAST * const object = functor(std::forward<Args>(args)..., std::forward<Params>(params)...);
        insert(std::move(key), object);
        return object;
    }

    std::pair<PabloAST *, bool> findOrAdd(PabloAST * object, const TypeId typeId, Args... args) noexcept {
        Key key = std::make_tuple(typeId, args...);
        PabloAST * const entry = find(key);
        if (entry) {
            return std::make_pair(entry, false);
        }
        insert(std::move(key), object);
        return std::make_pair(object, true);
    }

    void clear() {
        mMap.clear();
    }

private:

    void insert(Key && key, PabloAST * const object) noexcept {
        ProxyAllocator<uint8_t> alloc{mMap.get_allocator()};
        __make_byval_impl<std::tuple_size<Key>::value - 1, Key>::doit(key, alloc);
        mMap.insert(std::make_pair(std::move(key), object));
    }

    PabloAST * find(const Key & key) const noexcept {
        // check this map to see if we have it
        auto itr = mMap.find(key);
        if (itr != mMap.end()) {
            return itr->second;
        } else if (LLVM_LIKELY(allow_recursion(key))) {
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
        }
        return nullptr;
    }

    // TODO: if this map was aware of its "loop depth" (instead of just knowing whether
    // it has any outer scope) we could still safely reuse some Vars so long as we do not
    // go outside of the current loop nest
    static bool allow_recursion(const Key & key) noexcept {
        return !__contains_var_impl<std::tuple_size<Key>::value - 1, Key>::doit(key);
    }

private:
    const Type * const mPredecessor;
    Map                mMap;
};

struct ExpressionTable {

    using Allocator = SlabAllocator<uint8_t>;
    using UnaryT = FixedArgMap<PabloAST *>;
    using BinaryT = FixedArgMap<PabloAST *, PabloAST *>;
    using TernaryT = FixedArgMap<PabloAST *, PabloAST *, PabloAST *>;
    using QuaternaryT = FixedArgMap<PabloAST *, PabloAST *, PabloAST *, PabloAST *>;
    using IntrinsicT = FixedArgMap<Intrinsic, llvm::ArrayRef<PabloAST *>>;
    using TypeId = PabloAST::ClassTypeId;

    #define INIT(Type) m##Type(mAllocator, predecessor ? &(predecessor->m##Type) : nullptr)
    explicit ExpressionTable(ExpressionTable * predecessor = nullptr) noexcept
    : INIT(Unary)
    , INIT(Binary)
    , INIT(Ternary)
    , INIT(Quaternary)
    , INIT(Intrinsic) {

    }
    #undef INIT

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
    PabloAST * findUnaryOrCall(Functor && functor, const TypeId typeId, PabloAST * expr, Params... params) noexcept {
        return mUnary.findOrCall(std::move(functor), typeId, expr, std::forward<Params>(params)...);
    }

    template <class Functor, typename... Params>
    PabloAST * findBinaryOrCall(Functor && functor, const TypeId typeId, PabloAST * expr1, PabloAST * expr2, Params... params) noexcept {
        return mBinary.findOrCall(std::move(functor), typeId, expr1, expr2, std::forward<Params>(params)...);
    }

    template <class Functor, typename... Params>
    PabloAST * findTernaryOrCall(Functor && functor, const TypeId typeId, PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, Params... params) noexcept {
        return mTernary.findOrCall(std::move(functor), typeId, expr1, expr2, expr3, std::forward<Params>(params)...);
    }

    template <class Functor, typename... Params>
    PabloAST * findQuaternaryOrCall(Functor && functor, const TypeId typeId, PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, PabloAST * expr4, Params... params) noexcept {
        return mQuaternary.findOrCall(std::move(functor), typeId, expr1, expr2, expr3, expr4, std::forward<Params>(params)...);
    }

    template<class Functor, typename... Params>
    PabloAST * findIntrinsicOrCall(Functor && functor, const TypeId typeId, const Intrinsic intrinsic, llvm::ArrayRef<PabloAST *> argv, Params... params) noexcept {
        return mIntrinsic.findOrCall(std::move(functor), typeId, intrinsic, std::move(argv), std::forward<Params>(params)...);
    }

    std::pair<PabloAST *, bool> findOrAdd(Statement * stmt) noexcept {
        const auto typeId = stmt->getClassTypeId();
        switch (typeId) {
            case TypeId::Var:
            case TypeId::Not:
            case TypeId::Count: {
                    PabloAST * const expr1 = stmt->getOperand(0);
                    return mUnary.findOrAdd(stmt, typeId, expr1);
                }
            case TypeId::And:
            case TypeId::Or:
            case TypeId::Xor:
            case TypeId::Advance:
            case TypeId::ScanThru:
            case TypeId::MatchStar:
            case TypeId::Assign:
            case TypeId::PackL:
            case TypeId::PackH:
            case TypeId::Extract:
            case TypeId::Repeat: {
                PabloAST * const expr1 = stmt->getOperand(0);
                PabloAST * const expr2 = stmt->getOperand(1);
                return mBinary.findOrAdd(stmt, typeId, expr1, expr2);
            }
            case TypeId::Sel:
            case TypeId::IndexedAdvance: {
                PabloAST * const expr1 = stmt->getOperand(0);
                PabloAST * const expr2 = stmt->getOperand(1);
                PabloAST * const expr3 = stmt->getOperand(2);
                return mTernary.findOrAdd(stmt, typeId, expr1, expr2, expr3);
            }
            case TypeId::Ternary: {
                PabloAST * const expr1 = stmt->getOperand(0);
                PabloAST * const expr2 = stmt->getOperand(1);
                PabloAST * const expr3 = stmt->getOperand(2);
                PabloAST * const expr4 = stmt->getOperand(3);
                return mQuaternary.findOrAdd(stmt, typeId, expr1, expr2, expr3, expr4);
            }
            case TypeId::IntrinsicCall: {
                const auto args = llvm::cast<IntrinsicCall>(stmt)->getArgv();
                const auto intrinsicId = llvm::cast<IntrinsicCall>(stmt)->getIntrinsic();
                return mIntrinsic.findOrAdd(stmt, typeId, intrinsicId, args);
            }
            default:
                return std::make_pair(stmt, true);
        }
    }

    void clear() noexcept {
         mUnary.clear();
         mBinary.clear();
         mTernary.clear();
         mQuaternary.clear();
         mIntrinsic.clear();
         mAllocator.Reset();
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
