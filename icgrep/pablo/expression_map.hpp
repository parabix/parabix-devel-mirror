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

    explicit FixedArgMap(Type * predecessor = nullptr) : mPredecessor(predecessor) { }

    explicit FixedArgMap(Type && other) noexcept
    : mPredecessor(other.mPredecessor)
    , mMap(std::move(other.mMap)) {

    }

    FixedArgMap & operator=(Type && other) {
        mPredecessor = other.mPredecessor;
        mMap = std::move(other.mMap);
        return *this;
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
        for (Type * obj = this; obj; obj = obj->mPredecessor) {
            auto itr = obj->mMap.find(key);
            if (itr != mMap.end()) {
                obj->mMap.erase(itr);
                return true;
            }
        }
        return false;
    }

    inline PabloAST * find(const PabloAST::ClassTypeId type, Args... args) const {
        return find(std::make_tuple(type, args...));
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
    Type *                      mPredecessor;
    std::map<Key, PabloAST *>   mMap;
};


struct VarArgMap {

    friend struct ExpressionTable;

    using Allocator = LLVMAllocator;

    struct Key {

        inline Key(PabloAST::ClassTypeId type, const PabloAST * arg1, const std::vector<PabloAST *> & args, Allocator & allocator)
        : mType(type)
        , mArgs(1 + args.size())
        , mArg(allocator.Allocate<const PabloAST *>(mArgs)) {
            unsigned i = 1;
            mArg[0] = arg1;
            for (PabloAST * arg : args) {
                mArg[i++] = arg;
            }
        }

        inline Key(PabloAST::ClassTypeId type, const Variadic * stmt, Allocator & allocator)
        : mType(type)
        , mArgs(stmt->getNumOperands())
        , mArg(allocator.Allocate<const PabloAST *>(mArgs)) {
            unsigned i = 0;
            for (PabloAST * arg : *stmt) {
                mArg[i++] = arg;
            }
        }

        inline Key(const Key & key) = default;

        inline Key(Key && key) = default;

        inline bool operator < (const Key & other) const {
            if (mType != other.mType) {
                return mType < other.mType;
            } else if (mArgs != other.mArgs) {
                return mArgs < other.mArgs;
            }
            for (unsigned i = 0; i != mArgs; ++i) {
                if (mArg[i] != other.mArg[i]) {
                    return mArg[i] < other.mArg[i];
                }
            }
            return false;
        }

        const PabloAST::ClassTypeId   mType;
        const unsigned                mArgs;
        const PabloAST **             mArg;
    };

    using Map = std::map<Key, PabloAST *>;

    explicit VarArgMap(VarArgMap * predecessor = nullptr)
    : mPredecessor(predecessor) {

    }

    explicit VarArgMap(VarArgMap && other) noexcept
    : mPredecessor(other.mPredecessor)
    , mMap(std::move(other.mMap)) {

    }

    VarArgMap & operator=(VarArgMap && other) {
        mPredecessor = other.mPredecessor;
        mMap = std::move(other.mMap);
        return *this;
    }

    template <class Functor, typename... Params>
    inline PabloAST * findOrCall(Functor && functor, const PabloAST::ClassTypeId type, const PabloAST * arg1, const std::vector<PabloAST *> & args, Params... params) {
        Key key(type, arg1, args, mAllocator);
        PabloAST * const f = find(key);
        if (f) {
            mAllocator.Deallocate<const PabloAST *>(key.mArg);
            return f;
        }
        PabloAST * const object = functor(args, std::forward<Params>(params)...);
        mMap.insert(std::make_pair(std::move(key), object));
        return object;
    }

    inline std::pair<PabloAST *, bool> findOrAdd(Variadic * object, const PabloAST::ClassTypeId type) {
        Key key(type, object, mAllocator);
        PabloAST * const entry = find(key);
        if (entry) {
            mAllocator.Deallocate<const PabloAST *>(key.mArg);
            return std::make_pair(entry, false);
        }
        mMap.insert(std::make_pair(std::move(key), object));
        return std::make_pair(object, true);
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
    VarArgMap *     mPredecessor;
    Map             mMap;
    Allocator       mAllocator;
};

struct ExpressionTable {

    explicit ExpressionTable(ExpressionTable * predecessor = nullptr) noexcept {
        if (predecessor) {
            mUnary.mPredecessor = &(predecessor->mUnary);
            mBinary.mPredecessor = &(predecessor->mBinary);
            mTernary.mPredecessor = &(predecessor->mTernary);
            mVariable.mPredecessor = &(predecessor->mVariable);
        }
    }

    explicit ExpressionTable(ExpressionTable & other) = delete;

    explicit ExpressionTable(ExpressionTable && other) noexcept
    : mUnary(std::move(other.mUnary))
    , mBinary(std::move(other.mBinary))
    , mTernary(std::move(other.mTernary))
    , mVariable(std::move(other.mVariable)) {

    }

    ExpressionTable & operator=(ExpressionTable && other) {
        mUnary = std::move(other.mUnary);
        mBinary = std::move(other.mBinary);
        mTernary = std::move(other.mTernary);
        mVariable = std::move(other.mVariable);
        return *this;
    }

    template <class Functor, typename... Params>
    inline PabloAST * findUnaryOrCall(Functor && functor, const PabloAST::ClassTypeId type, PabloAST * expr, Params... params) {
        return mUnary.findOrCall(std::move(functor), type,  expr , std::forward<Params>(params)...);
    }

    template <class Functor, typename... Params>
    inline PabloAST * findBinaryOrCall(Functor && functor, const PabloAST::ClassTypeId type, PabloAST * expr1, PabloAST * expr2, Params... params) {
        return mBinary.findOrCall(std::move(functor), type, expr1, expr2, std::forward<Params>(params)...);
    }

    template <class Functor, typename... Params>
    inline PabloAST * findTernaryOrCall(Functor && functor, const PabloAST::ClassTypeId type, PabloAST * expr1, PabloAST * expr2, PabloAST * expr3, Params... params) {
        return mTernary.findOrCall(std::move(functor), type, expr1, expr2, expr3, std::forward<Params>(params)...);
    }

    template <class Functor, typename... Params>
    inline PabloAST * findVariableOrCall(Functor && functor, const PabloAST::ClassTypeId type, const PabloAST * arg1, const std::vector<PabloAST *> & args, Params... params) {
        return mVariable.findOrCall(std::move(functor), type, arg1, args, std::forward<Params>(params)...);
    }

    std::pair<PabloAST *, bool> findOrAdd(Statement * stmt) {
        switch (stmt->getClassTypeId()) {            
            case PabloAST::ClassTypeId::Assign:            
            case PabloAST::ClassTypeId::Var:
            case PabloAST::ClassTypeId::Not:
            case PabloAST::ClassTypeId::Count:
                return mUnary.findOrAdd(stmt, stmt->getClassTypeId(), stmt->getOperand(0));
            case PabloAST::ClassTypeId::And:
            case PabloAST::ClassTypeId::Or:
            case PabloAST::ClassTypeId::Xor:
                return mVariable.findOrAdd(cast<Variadic>(stmt), stmt->getClassTypeId());
            case PabloAST::ClassTypeId::Advance:
            case PabloAST::ClassTypeId::ScanThru:
            case PabloAST::ClassTypeId::MatchStar:
            case PabloAST::ClassTypeId::Next:
                return mBinary.findOrAdd(stmt, stmt->getClassTypeId(), stmt->getOperand(0), stmt->getOperand(1));
            case PabloAST::ClassTypeId::Sel:
                return mTernary.findOrAdd(stmt, stmt->getClassTypeId(), stmt->getOperand(0), stmt->getOperand(1), stmt->getOperand(2));
            case PabloAST::ClassTypeId::Call:
                // temporarily ignored
            default:
                return std::make_pair(stmt, true);
        }
    }


private:
    FixedArgMap<PabloAST *>                             mUnary;
    FixedArgMap<PabloAST *, PabloAST *>                 mBinary;
    FixedArgMap<PabloAST *, PabloAST *, PabloAST *>     mTernary;
    VarArgMap                                           mVariable;
};

}

#endif // EXPRESSION_MAP_HPP
