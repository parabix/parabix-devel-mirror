#include "bdd.hpp"
#include <iostream>

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

    return makeNode(n.level, n.low, n.high);
}

Engine::index_type Engine::apply(const index_type l, const index_type r, const OpCode op) {

    std::cerr << "Engine::apply(" << l << ',' << r << ',' << (int)(op) << ")\n";

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

    if (l >= mNode.size() || r >= mNode.size()) {
        throw std::runtime_error("Accessing invalid BDD node!");
    }

    const Node & nl = mNode[l];
    const Node & nr = mNode[r];

    unsigned lowl = l;
    unsigned highl = l;
    unsigned lowr = r;
    unsigned highr = r;
    unsigned level = nl.level;

    if (nl.level <= nr.level) {
        lowl = nl.low;
        highl = nl.high;
    }

    if (nr.level <= nl.level) {
        lowr = nr.low;
        highr = nr.high;
        level = nr.level;
    }

    return makeNode(level, apply(lowl, lowr, op), apply(highl, highr, op));
}

Engine::index_type Engine::makeNode(const index_type level, const index_type low, const index_type high) {
    if (low == high) {
        return low;
    }
    auto f = mMap.find(std::make_tuple(level, low, high));
    if (f == mMap.end()) {
        const size_t index = mNode.size();
        mNode.emplace_back(level, low, high);
        f = mMap.insert(std::make_pair(std::make_tuple(level, low, high), index)).first;
    }
    return f->second;
}

Engine::index_type Engine::satOne(const index_type root) {
    if (ISCONST(root))
       return root;
    const Node & v = mNode[root];
    if (ISZERO(v.low)) {
        return makeNode(v.level, 0, satOne(v.high));
    }
    else {
        return makeNode(v.level, satOne(v.low), 0);
    }
}

BDD Engine::addVar() {
    unsigned level = mVar.size() >> 1;
    index_type var = makeNode(level, 0, 1);
    mVar.push_back(var);
    mVar.push_back(makeNode(level, 1, 0));
    return BDD(var);
}

void Engine::init(const size_t variables) {
    mNode.reserve(variables + 2);
    mMap.bucket_size(variables);

    // add the 0 terminal
    assert (mNode.size() == 0);
    mNode.emplace_back(0, 0, 0);
    mMap.insert(std::make_pair(std::make_tuple(0, 0, 0), 0));

    // the 1 terminal
    assert (mNode.size() == 1);
    mNode.emplace_back(0, 1, 1);
    mMap.insert(std::make_pair(std::make_tuple(0, 1, 1), 1));

    // and the variables
    mVar.resize(variables * 2);
    for (auto i = 0; i != variables; ++i) {
        mVar[i * 2] = makeNode(i, 0, 1);
        mVar[i * 2 + 1] = makeNode(i, 1, 0);
    }
}

}
