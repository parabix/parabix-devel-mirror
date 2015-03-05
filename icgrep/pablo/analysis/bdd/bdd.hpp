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
    using Map = boost::unordered_map<Key, Array::size_type, BijectionHash, std::equal_to<Key>, LLVMAllocatorProxy>;
    #else
    using Map = std::unordered_map<Key, index_type, BijectionHash, std::equal_to<Key>, LLVMAllocatorProxy>;
    #endif

public:

    Engine(const size_t initialSize, const size_t varCount);

    BDD applyAND(const BDD & r, const BDD & l);

    BDD applyOR(const BDD & r, const BDD & l);

    BDD applyNOT(const BDD & bdd);

    BDD applyXOR(const BDD & r, const BDD & l);

    BDD applySEL(const BDD & cond, const BDD & trueBDD, const BDD & falseBDD);

    BDD addVar();

    void addVars(const size_t vars);

    BDD var(const unsigned index);

    BDD nvar(const unsigned index);

protected:

    index_type satone(const index_type root);

    index_type apply(const index_type l, const index_type r, const OpCode op);

    index_type makenode(const index_type level, const index_type low, const index_type high);

private:

    Array           mNode;
    Vars            mVar;
    Map             mMap;
    LLVMAllocator   mAllocator;
};

struct BDD {

    using OpCode = Engine::OpCode;
    using index_type = Engine::index_type;

    inline int operator==(const BDD &r) const {
        return mRoot == r.mRoot;
    }

    inline int operator!=(const BDD &r) const {
        return mRoot != r.mRoot;
    }

    inline bool isZero() const {
        return mRoot == 0;
    }

    inline bool isOne() const {
        return mRoot == 1;
    }

    inline bool isConstant() const {
        return mRoot < 2;
    }

    inline BDD() : mRoot(0) {}

protected:

    inline BDD(const index_type index) : mRoot(index) { }
    inline BDD(const BDD & r) : mRoot(r.mRoot) {  }
    inline ~BDD() { }

private:

    index_type mRoot;

};

BDD Engine::satone(const BDD & bdd) {
    if (bdd.isConstant()) {
        return bdd;
    }
    return BDD(satone(bdd.mRoot));
}

BDD Engine::applyAND(const BDD & r, const BDD & l) {
    return apply(r.mRoot, l.mRoot, BDD::OpCode::AND);
}

BDD Engine::applyOR(const BDD & r, const BDD & l) {
    return apply(r.mRoot, l.mRoot, BDD::OpCode::OR);
}

BDD Engine::applyNOT(const BDD & bdd) {
    return apply(bdd.mRoot, BDD::OpCode::NOT);
}

BDD Engine::applyXOR(const BDD & r, const BDD & l) {
    return apply(r.mRoot, l.mRoot, BDD::OpCode::XOR);
}

BDD Engine::applySEL(const BDD & cond, const BDD & trueBDD, const BDD & falseBDD) {
    return applyOR(applyAND(cond, trueBDD), applyAND(applyNOT(cond), falseBDD));
}

}

#endif // BDD_HPP
