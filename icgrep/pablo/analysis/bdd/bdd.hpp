#ifndef BDD_HPP
#define BDD_HPP

#include <vector>
#ifdef USE_BOOST
#include <boost/unordered_map.hpp>
#else
#include <unordered_map>
#endif
#include <slab_allocator.h>


namespace bdd {

struct BDD;

class Engine {

    friend struct BDD;

    enum class OpCode {
        AND
        , OR
        , XOR
        , NOT
    };

    struct Node {
        friend struct BDD;

        Node(const unsigned level, const unsigned low, const unsigned high)
        : level(level)
        , low(low)
        , high(high)
        {

        }

        const unsigned level;
        const unsigned low;
        const unsigned high;
    };

    using Array = std::vector<Node, LLVMAllocatorProxy>;
    using index_type = unsigned;
    using Vars = std::vector<index_type, LLVMAllocatorProxy>;
    using Key = std::tuple<index_type, index_type, index_type>;

    struct BijectionHash {
        template<typename X, typename Y>
        inline static size_t pair(const X x, const Y y) {
            return (((x + y) * (x + y + 1)) >> 1) + x; // (x,y) -> h bijection
        }
        inline size_t operator()(const Key & k) const {
            return pair(std::get<0>(k), pair(std::get<1>(k), std::get<2>(k)));
        }
    };
    #ifdef USE_BOOST
    using Map = boost::unordered_map<Key, Array::size_type, BijectionHash, std::equal_to<Key>, LLVMAllocatorProxy<std::pair<Key, index_type>>;
    #else
    using Map = std::unordered_map<Key, index_type, BijectionHash, std::equal_to<Key>, LLVMAllocatorProxy<std::pair<Key, index_type>>;
    #endif

public:

    Engine(const size_t initialSize);

    BDD applyAnd(const BDD & r, const BDD & l);

    BDD applyOr(const BDD & r, const BDD & l);

    BDD applyNot(const BDD & bdd);

    BDD applyXor(const BDD & r, const BDD & l);

    BDD applySel(const BDD & cond, const BDD & trueBDD, const BDD & falseBDD);

    BDD addVar();

    BDD var(const unsigned index);

    BDD nvar(const unsigned index);

    BDD satOne(const BDD & bdd);

protected:

    index_type satOne(const index_type root);

    index_type apply(const index_type l, const index_type r, const OpCode op);

    index_type makeNode(const index_type level, const index_type low, const index_type high);

private:

    Array           mNode;
    Vars            mVar;
    Map             mMap;
    LLVMAllocator   mAllocator;
};

struct BDD {

    friend struct Engine;

    using OpCode = Engine::OpCode;
    using index_type = Engine::index_type;

    inline int operator==(const bool term) const {
        return term ? isTrue() : isFalse();
    }

    inline int operator==(const bool term) const {
        return term ? isFalse() : isTrue();
    }

    inline int operator==(const BDD & r) const {
        return mRoot == r.mRoot;
    }

    inline int operator!=(const BDD & r) const {
        return mRoot != r.mRoot;
    }

    inline bool isFalse() const {
        return mRoot == 0;
    }

    inline bool isTrue() const {
        return mRoot == 1;
    }

    inline bool isConstant() const {
        return mRoot < 2;
    }

    inline BDD Contradiction() const {
        return BDD(0);
    }

    inline BDD Tautology() const {
        return BDD(1);
    }

protected:

    inline BDD() : mRoot(0) {}
    inline BDD(const index_type index) : mRoot(index) { }
    inline BDD(const BDD & r) : mRoot(r.mRoot) {  }
    inline ~BDD() { }

private:

    index_type mRoot;

};

BDD Engine::satOne(const BDD & bdd) {
    if (bdd.isConstant()) {
        return bdd;
    }
    return BDD(satOne(bdd.mRoot));
}

BDD Engine::applyAnd(const BDD & r, const BDD & l) {
    return apply(r.mRoot, l.mRoot, BDD::OpCode::AND);
}

BDD Engine::applyOr(const BDD & r, const BDD & l) {
    return apply(r.mRoot, l.mRoot, BDD::OpCode::OR);
}

BDD Engine::applyNot(const BDD & bdd) {
    return apply(bdd.mRoot, BDD::OpCode::NOT);
}

BDD Engine::applyXor(const BDD & r, const BDD & l) {
    return apply(r.mRoot, l.mRoot, BDD::OpCode::XOR);
}

BDD Engine::applySel(const BDD & cond, const BDD & trueBDD, const BDD & falseBDD) {
    return applyOr(applyAnd(cond, trueBDD), applyAnd(applyNot(cond), falseBDD));
}

}

#endif // BDD_HPP
