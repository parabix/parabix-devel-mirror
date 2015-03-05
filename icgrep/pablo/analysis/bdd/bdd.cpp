#include "bdd.hpp"

namespace bdd {

#define ISONE(a)   ((a) == 1)
#define ISZERO(a)  ((a) == 0)
#define ISCONST(a) ((a) < 2)

Engine::index_type Engine::apply(const index_type l, const OpCode op) {

    switch (op) {
        case OpCode::NOT:
            if (ISCONST(l)) return 1 - l;
            break;
        default:
            throw std::runtime_error("Unsupported unary operation.");
    }

    const Node & n = mNode[l];

    return makenode(n.level, n.low, n.high);
}

Engine::index_type Engine::apply(const index_type l, const index_type r, const OpCode op) {

    switch (op) {
        case OpCode::AND:
            if (l == r) return l;
            if (ISCONST(l) && ISCONST(r)) return l & r;
            if (ISONE(l)) return r;
            if (ISONE(r)) return l;
            break;
        case OpCode::OR:
            if (l == r) return l;
            if (ISCONST(l) && ISCONST(r)) return l | r;
            if (ISONE(l) || ISONE(r)) return 1;
            if (ISZERO(l)) return r;
            if (ISZERO(r)) return l;
            break;
        case OpCode::XOR:
            if (l == r) return 0;
            if (ISCONST(l) && ISCONST(r)) return l ^ r;
            if (ISZERO(l)) return r;
            if (ISZERO(r)) return l;
            break;
        default:
            throw std::runtime_error("Unsupported binary operation.");
    }

    const Node & nl = mNode[l];
    const Node & nr = mNode[l];

    unsigned lowl = l;
    unsigned highl = l;
    unsigned level = nl.level;
    if (nl.level <= nr.level) {
        lowl = nl.low;
        highl = nl.high;
    }

    unsigned lowr = r;
    unsigned highr = r;
    if (nr.level <= nl.level) {
        lowr = nr.low;
        highr = nr.high;
        level = nr.level;
    }

    return makenode(level, apply(lowl, lowr, op), apply(highl, highr, op));
}

Engine::index_type Engine::makenode(const index_type level, const index_type low, const index_type high) {
    if (low == high) {
        return low;
    }
    auto f = mMap.find(std::make_tuple(level, low, high));
    if (f == mMap.end()) {
        const size_t index = mNode.size();
        mNode.emplace_back(level, low, high);
        f = mMap.insert(std::make_pair(std::make_tuple(level, low, high), index)).second;
    }
    return f->second;
}

Engine::index_type Engine::satone(const index_type root) {
    if (ISCONST(root))
       return root;
    const Node & v = mNode[root];
    if (ISZERO(v.low)) {
        return makenode(v.level, 0, satone(v.high).mRoot);
    }
    else {
        return makenode(v.level, satone(v.low).mRoot, 0);
    }
}

BDD Engine::var(const unsigned index) {
    return BDD(mVar[index * 2]);
}

BDD Engine::nvar(const unsigned index) {
    return BDD(mVar[index * 2 + 1]);
}

BDD Engine::addVar() {
    unsigned level = mVar.size() >> 1;
    index_type var = makenode(level, 0, 1);
    mVar.push_back(var);
    mVar.push_back(makenode(level, 1, 0));
    return BDD(var);
}

void Engine::addVars(const size_t vars) {
    unsigned level = mVar.size() >> 1;
    for (size_t i = 0; i != vars; ++i, ++level) {
        mVar.push_back(makenode(level, 0, 1));
        mVar.push_back(makenode(level, 1, 0));
    }
}

Engine::Engine(const size_t initialSize, const size_t varCount)
: mNode(LLVMAllocatorProxy(mAllocator))
, mVar(LLVMAllocatorProxy(mAllocator))
, mMap(LLVMAllocatorProxy(mAllocator)) {
    mNode.reserve(initialSize + 2);
    mMap.bucket_size(initialSize);

    // add the 0 terminal
    assert (mNode.size() == 0);
    mNode.emplace_back(0, 0, 0);
    mMap.insert(std::make_pair(std::make_tuple(0, 0, 0), 0));

    // the 1 terminal
    assert (mNode.size() == 1);
    mNode.emplace_back(0, 1, 1);
    mMap.insert(std::make_pair(std::make_tuple(0, 1, 1), 1));

    // and the variables
    addVars(varCount);
}

}
