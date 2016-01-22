#include "booleanreassociationpass.h"
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/topological_sort.hpp>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/analysis/pabloverifier.hpp>
#include <algorithm>
#include <numeric> // std::iota
#ifndef NDEBUG
#include <queue>
#endif
#include <pablo/printer_pablos.h>
#include <iostream>

using namespace boost;
using namespace boost::container;

namespace pablo {

using TypeId = PabloAST::ClassTypeId;
using DependencyGraph = BooleanReassociationPass::Graph;
using Vertex = DependencyGraph::vertex_descriptor;
using VertexData = BooleanReassociationPass::VertexData;
using SinkMap = BooleanReassociationPass::Map;
using DistributionGraph = adjacency_list<hash_setS, vecS, bidirectionalS, Vertex>;
using DistributionMap = flat_map<DependencyGraph::vertex_descriptor, DistributionGraph::vertex_descriptor>;
using VertexSet = std::vector<Vertex>;
using VertexSets = std::vector<VertexSet>;
using Biclique = std::pair<VertexSet, VertexSet>;
using BicliqueSet = std::vector<Biclique>;
using DistributionSet = std::tuple<VertexSet, VertexSet, VertexSet>;
using DistributionSets = std::vector<DistributionSet>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief helper functions
 ** ------------------------------------------------------------------------------------------------------------- */
template<typename Iterator>
inline DependencyGraph::edge_descriptor first(const std::pair<Iterator, Iterator> & range) {
    assert (range.first != range.second);
    return *range.first;
}

#ifndef NDEBUG
static bool no_path(const Vertex u, const Vertex v, const DependencyGraph & G) {
    if (u == v) return false;
    flat_set<Vertex> V;
    std::queue<Vertex> Q;
    Q.push(u);
    for (;;) {
        Vertex w = Q.front();
        if (w == v) {
            return false;
        }
        Q.pop();
        for (auto e : make_iterator_range(out_edges(w, G))) {
            Vertex x = target(e, G);
            if (V.count(x)) continue;
            Q.push(x);
            V.insert(x);
        }
        if (Q.empty()) {
            break;
        }
    }
    return true;
}
#endif

inline void add_edge(PabloAST * expr, const Vertex u, const Vertex v, DependencyGraph & G) {
    assert (no_path(v, u, G));
    // Make sure each edge is unique
    for (auto e : make_iterator_range(out_edges(u, G))) {
        if (LLVM_UNLIKELY(target(e, G) == v && (G[e] == nullptr || G[e] == expr))) {
            G[e] = expr;
            return;
        }
    }
    G[std::get<0>(add_edge(u, v, G))] = expr;
}

static inline bool inCurrentBlock(const Statement * stmt, const PabloBlock * const block) {
    return stmt->getParent() == block;
}

static inline bool inCurrentBlock(const PabloAST * expr, const PabloBlock * const block) {
    return expr ? isa<Statement>(expr) && inCurrentBlock(cast<Statement>(expr), block) : true;
}

inline TypeId & getType(VertexData & data) {
    return std::get<0>(data);
}

inline TypeId getType(const VertexData & data) {
    return std::get<0>(data);
}

inline PabloAST *& getValue(VertexData & data) {
    return std::get<1>(data);
}

inline PabloAST * getValue(const VertexData & data) {
    return std::get<1>(data);
}

inline bool isAssociative(const VertexData & data) {
    switch (getType(data)) {
        case TypeId::And:
        case TypeId::Or:
        case TypeId::Xor:
            return true;
        default:
            return false;
    }
}

inline bool isDistributive(const VertexData & data) {
    switch (getType(data)) {
        case TypeId::And:
        case TypeId::Or:
            return true;
        default:
            return false;
    }
}

inline bool isMutable(const VertexData & data) {
    return getType(data) != TypeId::Var;
}

inline bool isNonEscaping(const VertexData & data) {
    // If these are redundant, the Simplifier pass will eliminate them. Trust that they're necessary.
    switch (getType(data)) {
        case TypeId::Assign:
        case TypeId::Next:
        case TypeId::If:
        case TypeId::While:
            return false;
        default:
            return true;
    }
}

inline bool isSameType(const VertexData & data1, const VertexData & data2) {
    return getType(data1) == getType(data2);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief intersects
 ** ------------------------------------------------------------------------------------------------------------- */
template <class Type>
inline bool intersects(const Type & A, const Type & B) {
    auto first1 = A.begin(), last1 = A.end();
    auto first2 = B.begin(), last2 = B.end();
    while (first1 != last1 && first2 != last2) {
        if (*first1 < *first2) {
            ++first1;
        } else if (*first2 < *first1) {
            ++first2;
        } else {
            return true;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool BooleanReassociationPass::optimize(PabloFunction & function) {   
    BooleanReassociationPass brp;
    brp.processScopes(function);
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-reassociation");
    #endif
    Simplifier::optimize(function);
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScopes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BooleanReassociationPass::processScopes(PabloFunction & function) {
    function.setEntryBlock(processScopes(function, function.getEntryBlock()));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScopes
 ** ------------------------------------------------------------------------------------------------------------- */
PabloBlock * BooleanReassociationPass::processScopes(PabloFunction & f, PabloBlock * const block) {
    for (Statement * stmt : *block) {
        if (If * ifNode = dyn_cast<If>(stmt)) {
            PabloBlock * rewrittenBlock = processScopes(f, ifNode->getBody());
            mResolvedScopes.emplace(rewrittenBlock, stmt);
            ifNode->setBody(rewrittenBlock);
        } else if (While * whileNode = dyn_cast<While>(stmt)) {
            PabloBlock * rewrittenBlock = processScopes(f, whileNode->getBody());
            mResolvedScopes.emplace(rewrittenBlock, stmt);
            whileNode->setBody(rewrittenBlock);
        }
    }    
    return processScope(f, block);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getVertex
 ** ------------------------------------------------------------------------------------------------------------- */
template<typename ValueType, typename GraphType, typename MapType>
static inline Vertex getVertex(const ValueType value, GraphType & G, MapType & M) {
    const auto f = M.find(value);
    if (f != M.end()) {
        return f->second;
    }
    const auto u = add_vertex(value, G);
    M.insert(std::make_pair(value, u));
    return u;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeTree
 ** ------------------------------------------------------------------------------------------------------------- */
inline PabloAST * writeTree(PabloBlock * block, const TypeId typeId, circular_buffer<PabloAST *> & Q) {
    while (Q.size() > 1) {
        PabloAST * e1 = Q.front(); Q.pop_front();
        PabloAST * e2 = Q.front(); Q.pop_front();
        PabloAST * expr = nullptr;
        switch (typeId) {
            case TypeId::And:
                expr = block->createAnd(e1, e2); break;
            case TypeId::Or:
                expr = block->createOr(e1, e2); break;
            case TypeId::Xor:
                expr = block->createXor(e1, e2); break;
            default: break;
        }
        Q.push_back(expr);
    }
    return Q.front();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isReachableAtEntryPoint
 ** ------------------------------------------------------------------------------------------------------------- */
bool isReachableAtEntryPoint(const PabloAST * const expr, const PabloBlock * const block) {
    // if expr is not a statement, it's always reachable
    if (isa<Statement>(expr)) {
        const PabloBlock * parent = cast<Statement>(expr)->getParent();
        // if expr is in the current block, it's not reachable at the entry point of this block
        if (parent == block) {
            return false;
        }
        const PabloBlock * current = block->getParent();
        // If expr is an Assign or Next node, it must escape its block (presuming the Simplifier has eliminated any
        // unnecessary Assign or Next nodes.) We'll want to test whether the parent of its block is an ancestor of
        // the current block.
        if (isa<Assign>(expr) || isa<Next>(expr)) {
            parent = parent->getParent();
        }
        // If we can find expr in a preceeding block, it's reachable
        for (;;) {
            if (parent == current) {
                return true;
            }
            if (current == nullptr) {
                break;
            }
            current = current->getParent();
        }
        // otherwise it must be in a nested block and therefore unreachable
        return false;
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createTree
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * BooleanReassociationPass::createTree(const PabloBlock * const block, PabloBlock * const newScope, const Vertex u, Graph & G) {

    flat_set<PabloAST *> internalVars;

    flat_set<PabloAST *> externalVars;

    assert (in_degree(u, G) > 0);

    for (const auto e : make_iterator_range(in_edges(u, G))) {
        PabloAST * const expr = getValue(G[source(e, G)]);
        assert ("G contains a null input variable!" && (expr != nullptr));
        if (isReachableAtEntryPoint(expr, block)) {
            externalVars.insert(expr);
        } else {
            internalVars.insert(expr);
        }
    }

    circular_buffer<PabloAST *> Q(in_degree(u, G));
    for (auto expr : externalVars) {
        Q.push_back(expr);
    }

    const TypeId typeId = getType(G[u]);
    Statement * stmt = newScope->front();
    if (Q.size() > 1) {
        newScope->setInsertPoint(nullptr);
        writeTree(newScope, typeId, Q);
        assert (Q.size() == 1);
    }

    for (unsigned distance = 0; stmt; ++distance, stmt = stmt->getNextNode()) {

        if (distance > 3 && Q.size() > 1) { // write out the current Q
            writeTree(newScope, typeId, Q);
            assert (Q.size() == 1);
        }

        bool found = false;
        if (isa<If>(stmt)) {
            for (Assign * def : cast<If>(stmt)->getDefined()) {
                if (internalVars.erase(def) != 0) {
                    assert (!Q.full());
                    Q.push_back(def);
                    found = true;
                }
            }
        } else if (isa<While>(stmt)) {
            for (Next * var : cast<While>(stmt)->getVariants()) {
                if (internalVars.erase(var) != 0) {
                    assert (!Q.full());
                    Q.push_back(var);
                    found = true;
                }
            }
        } else if (internalVars.erase(stmt) != 0) {
            Q.push_back(stmt);
            found = true;
        }

        if (found) {
            newScope->setInsertPoint(stmt);
            distance = 0;
        }
    }

    assert (internalVars.empty());

    newScope->setInsertPoint(newScope->back());

    writeTree(newScope, typeId, Q);
    assert (Q.size() == 1);
    PabloAST * const result = Q.front();
    assert (result);
    getValue(G[u]) = result;

    return result;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScope
 ** ------------------------------------------------------------------------------------------------------------- */
inline PabloBlock * BooleanReassociationPass::processScope(PabloFunction & f, PabloBlock * const block) {
    Graph G;
    summarizeAST(block, G);
    redistributeAST(G);
    return rewriteAST(f, block, G);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief rewriteAST
 ** ------------------------------------------------------------------------------------------------------------- */
PabloBlock * BooleanReassociationPass::rewriteAST(PabloFunction & f, PabloBlock * const originalScope, Graph & G) {

    using ReadyPair = std::pair<unsigned, Vertex>;
    using ReadySet = std::vector<ReadyPair>;

    // Determine the bottom-up ordering of G
    std::vector<unsigned> ordering(num_vertices(G));
    circular_buffer<Vertex> Q(num_vertices(G));
    for (const Vertex u : make_iterator_range(vertices(G))) {
        if (out_degree(u, G) == 0 && in_degree(u, G) != 0) {
            ordering[u] = 1;
            Q.push_back(u);
        }
    }

    while (!Q.empty()) {
        const Vertex u = Q.front();
        Q.pop_front();
        assert (ordering[u] > 0);
        for (const auto ei : make_iterator_range(in_edges(u, G))) {
            const Vertex v = source(ei, G);
            if (ordering[v] == 0) {
                unsigned weight = 0;
                bool ready = true;
                for (const auto ej : make_iterator_range(out_edges(v, G))) {
                    const Vertex w = target(ej, G);
                    const unsigned t = ordering[w];
                    if (t == 0) {
                        ready = false;
                        break;
                    }
                    weight = std::max(weight, t);
                }
                if (ready) {
                    assert (weight < std::numeric_limits<unsigned>::max());
                    assert (weight > 0);
                    ordering[v] = (weight + 1);
                    Q.push_back(v);
                }
            }
        }
    }

    // Compute the initial ready set
    ReadySet readySet;
    for (const Vertex u : make_iterator_range(vertices(G))) {
        if (in_degree(u, G) == 0 && out_degree(u, G) != 0) {
            readySet.emplace_back(ordering[u], u);
        }
    }

    struct {
        bool operator()(const ReadyPair a, const ReadyPair b) {
            return std::get<0>(a) > std::get<0>(b);
        }
    } readyComparator;

    std::sort(readySet.begin(), readySet.end(), readyComparator);

    flat_set<Vertex> tested;

    PabloBlock * const newScope = PabloBlock::Create(f);

    // This should hold back Assign and Next nodes then write them in after any reassociated statements
    // are rewritten at the earliest use.

    // Rewrite the AST using the bottom-up ordering
    while (!readySet.empty()) {

        // Scan through the ready set to determine which one 'kills' the greatest number of inputs
        // NOTE: the readySet is kept in sorted "distance to sink" order; thus those closest to a
        // sink will be evaluated first.
        double best = 0;
        auto chosen = readySet.begin();
        for (auto ri = readySet.begin(); ri != readySet.end(); ++ri) {
            const Vertex u = std::get<1>(*ri);
            const PabloAST * const expr = getValue(G[u]);
            if (expr && (isa<Assign>(expr) || isa<Next>(expr))) {
                chosen = ri;
                break;
            }
            double progress = 0;
            for (auto ei : make_iterator_range(in_edges(u, G))) {
                const auto v = source(ei, G);
                const auto totalUsesOfIthOperand = out_degree(v, G);
                if (LLVM_UNLIKELY(totalUsesOfIthOperand == 0)) {
                    progress += 1.0;
                } else {
                    unsigned unscheduledUsesOfIthOperand = 0;
                    for (auto ej : make_iterator_range(out_edges(v, G))) {
                        if (ordering[target(ej, G)]) { // if this edge leads to an unscheduled statement
                            ++unscheduledUsesOfIthOperand;
                        }
                    }
                    progress += std::pow((double)(totalUsesOfIthOperand - unscheduledUsesOfIthOperand + 1) / (double)(totalUsesOfIthOperand), 2);
                }
            }
            if (progress > best) {
                chosen = ri;
                best = progress;
            }
        }

        Vertex u; unsigned weight;
        std::tie(weight, u) = *chosen;
        readySet.erase(chosen);

        assert (weight > 0);

        if (LLVM_LIKELY(isMutable(G[u]))) {
            PabloAST * expr = nullptr;
            if (isAssociative(G[u])) {
                expr = createTree(originalScope, newScope, u, G);
            } else {
                expr = getValue(G[u]);
            }
            assert (expr);
            for (auto e : make_iterator_range(out_edges(u, G))) {
                if (G[e] && G[e] != expr) {
                    if (PabloAST * user = getValue(G[target(e, G)])) {
                        cast<Statement>(user)->replaceUsesOfWith(G[e], expr);
                    }
                }
            }
            // Make sure that optimization doesn't reduce this to an already written statement
            if (LLVM_LIKELY(isa<Statement>(expr) && cast<Statement>(expr)->getParent() == originalScope)) {
                newScope->insert(cast<Statement>(expr));
            }
        }

        // mark this instruction as written
        ordering[u] = 0;
        // Now check whether any new instructions are ready
        for (auto ei : make_iterator_range(out_edges(u, G))) {
            bool ready = true;
            const auto v = target(ei, G);
            // since G may be a multigraph, we need to check whether we've already tested v
            if (tested.insert(v).second) {
                for (auto ej : make_iterator_range(in_edges(v, G))) {
                    if (ordering[source(ej, G)]) {
                        ready = false;
                        break;
                    }
                }
                if (ready) {
                    auto entry = std::make_pair(ordering[v], v);
                    auto position = std::lower_bound(readySet.begin(), readySet.end(), entry, readyComparator);
                    readySet.insert(position, entry);
                    assert (std::is_sorted(readySet.cbegin(), readySet.cend(), readyComparator));
                }
            }
        }
        tested.clear();
    }
    originalScope->eraseFromParent();
    return newScope;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief summarizeAST
 *
 * This function scans through a scope block and computes a DAG G in which any sequences of AND, OR or XOR functions
 * are "flattened" (i.e., allowed to have any number of inputs.)
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::summarizeAST(PabloBlock * const block, Graph & G) const {
    Map M;
    // Compute the base def-use graph ...
    for (Statement * stmt : *block) {
        const Vertex u = getSummaryVertex(stmt, G, M, block);
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            PabloAST * const op = stmt->getOperand(i);
            if (LLVM_UNLIKELY(isa<Integer>(op) || isa<String>(op))) {
                continue;
            }
            const Vertex v = getSummaryVertex(op, G, M, block);
            add_edge(op, v, u, G);
            // If this operand is a Not operation that is not in this PabloBlock,
            // pull its input operand in. It may lead to future optimizations.
            if (LLVM_UNLIKELY(isa<Not>(op) && !inCurrentBlock(cast<Not>(op), block))) {
                PabloAST * const negatedValue = cast<Not>(op)->getExpr();
                add_edge(negatedValue, getSummaryVertex(negatedValue, G, M, block), v, G);
            }
        }
        if (isa<If>(stmt)) {
            for (Assign * def : cast<const If>(stmt)->getDefined()) {
                const Vertex v = getSummaryVertex(def, G, M, block);
                add_edge(def, u, v, G);
                resolveNestedUsages(v, def, block, G, M, stmt);
            }
        } else if (isa<While>(stmt)) {
            // To keep G a DAG, we need to do a bit of surgery on loop variants because
            // the next variables it produces can be used within the condition. Instead,
            // we make the loop dependent on the original value of each Next node and
            // the Next node dependent on the loop.
            for (Next * var : cast<const While>(stmt)->getVariants()) {
                const Vertex v = getSummaryVertex(var, G, M, block);
                assert (in_degree(v, G) == 1);
                add_edge(nullptr, source(first(in_edges(v, G)), G), u, G);
                remove_edge(v, u, G);
                add_edge(var, u, v, G);
                resolveNestedUsages(v, var, block, G, M, stmt);
            }
        } else {
            resolveNestedUsages(u, stmt, block, G, M);
        }
    }
    std::vector<Vertex> mapping(num_vertices(G));
    summarizeGraph(G, mapping);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resolveNestedUsages
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::resolveNestedUsages(const Vertex u, PabloAST * expr, PabloBlock * const block, Graph & G, Map & M, const Statement * const ignoreIfThis) const {
    for (PabloAST * user : expr->users()) {
        assert (user);
        if (LLVM_LIKELY(user != expr && isa<Statement>(user))) {
            PabloBlock * parent = cast<Statement>(user)->getParent();
            assert (parent);
            if (LLVM_UNLIKELY(parent != block)) {
                for (;;) {
                    if (LLVM_UNLIKELY(parent == nullptr)) {
                        assert (isa<Assign>(expr) || isa<Next>(expr));
                        break;
                    } else if (parent->getParent() == block) {
                        const auto f = mResolvedScopes.find(parent);
                        if (LLVM_UNLIKELY(f == mResolvedScopes.end())) {
                            throw std::runtime_error("Failed to resolve scope block!");
                        }
                        Statement * const branch = f->second;
                        if (LLVM_UNLIKELY(branch == ignoreIfThis)) {
                            break;
                        }
                        // Add in a Var denoting the user of this expression so that it can be updated if expr changes.
                        const Vertex v = getSummaryVertex(user, G, M, block);
                        add_edge(expr, u, v, G);

                        const Vertex w = getSummaryVertex(branch, G, M, block);
                        add_edge(nullptr, v, w, G);
                        break;
                    }
                    parent = parent->getParent();
                }
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief summarizeGraph
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BooleanReassociationPass::summarizeGraph(Graph & G, std::vector<Vertex> & mapping) {
    std::vector<Vertex> reverse_topological_ordering;
    reverse_topological_ordering.reserve(num_vertices(G));

    topological_sort(G, std::back_inserter(reverse_topological_ordering));
    assert(mapping.size() >= num_vertices(G));
    for (const Vertex u : reverse_topological_ordering) {
        if (in_degree(u, G) == 0 && out_degree(u, G) == 0) {
            continue;
        } else if (LLVM_LIKELY(out_degree(u, G) > 0)) {
            if (isAssociative(G[u])) {
                if (LLVM_UNLIKELY(in_degree(u, G) == 1)) {
                    // We have a redundant node here that'll simply end up being a duplicate
                    // of the input value. Remove it and add any of its outgoing edges to its
                    // input node.
                    const auto ei = first(in_edges(u, G));
                    const Vertex v = source(ei, G);
                    for (auto ej : make_iterator_range(out_edges(u, G))) {
                        const Vertex w = target(ej, G);
                        add_edge(G[ei], v, w, G);
                        if (mapping[w] == mapping[u]) {
                            mapping[w] = v;
                        }
                    }
                    mapping[u] = v;
                    clear_vertex(u, G);
                    getType(G[u]) = TypeId::Var;
                    getValue(G[u]) = nullptr;
                    continue;
                } else if (LLVM_UNLIKELY(out_degree(u, G) == 1)) {
                    // Otherwise if we have a single user, we have a similar case as above but
                    // we can only merge this vertex into the outgoing instruction if they are
                    // of the same type.
                    const auto ei = first(out_edges(u, G));
                    const Vertex v = target(ei, G);
                    if (LLVM_UNLIKELY(isSameType(G[v], G[u]))) {
                        for (auto ej : make_iterator_range(in_edges(u, G))) {
                            add_edge(G[ei], source(ej, G), v, G);
                        }
                        mapping[u] = v;
                        clear_vertex(u, G);
                        getType(G[u]) = TypeId::Var;
                        getValue(G[u]) = nullptr;
                        continue;
                    }
                }
            }
        } else if (isNonEscaping(G[u])) {
            clear_in_edges(u, G);
            getType(G[u]) = TypeId::Var;
            getValue(G[u]) = nullptr;
            continue;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerateBicliques
 *
 * Adaptation of the MICA algorithm as described in "Consensus algorithms for the generation of all maximal
 * bicliques" by Alexe et. al. (2003). Note: this implementation considers all verticies in set A to be in
 * bipartition A and their adjacencies to be in B.
  ** ------------------------------------------------------------------------------------------------------------- */
template <class Graph>
static BicliqueSet enumerateBicliques(const Graph & G, const VertexSet & A) {
    using IntersectionSets = std::set<VertexSet>;

    IntersectionSets B1;
    for (auto u : A) {
        if (in_degree(u, G) > 0) {
            VertexSet incomingAdjacencies;
            incomingAdjacencies.reserve(in_degree(u, G));
            for (auto e : make_iterator_range(in_edges(u, G))) {
                incomingAdjacencies.push_back(source(e, G));
            }
            std::sort(incomingAdjacencies.begin(), incomingAdjacencies.end());
            B1.insert(std::move(incomingAdjacencies));
        }
    }

    IntersectionSets B(B1);

    IntersectionSets Bi;

    VertexSet clique;
    for (auto i = B1.begin(); i != B1.end(); ++i) {
        for (auto j = i; ++j != B1.end(); ) {
            std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(clique));
            if (clique.size() > 0) {
                if (B.count(clique) == 0) {
                    Bi.insert(clique);
                }
                clique.clear();
            }
        }
    }

    for (;;) {
        if (Bi.empty()) {
            break;
        }
        B.insert(Bi.begin(), Bi.end());
        IntersectionSets Bk;
        for (auto i = B1.begin(); i != B1.end(); ++i) {
            for (auto j = Bi.begin(); j != Bi.end(); ++j) {
                std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(clique));
                if (clique.size() > 0) {
                    if (B.count(clique) == 0) {
                        Bk.insert(clique);
                    }
                    clique.clear();
                }
            }
        }
        Bi.swap(Bk);
    }

    BicliqueSet cliques;
    cliques.reserve(B.size());
    for (auto Bi = B.begin(); Bi != B.end(); ++Bi) {
        VertexSet Ai(A);
        for (const Vertex u : *Bi) {
            VertexSet Aj;
            Aj.reserve(out_degree(u, G));
            for (auto e : make_iterator_range(out_edges(u, G))) {
                Aj.push_back(target(e, G));
            }
            std::sort(Aj.begin(), Aj.end());
            VertexSet Ak;
            Ak.reserve(std::min(Ai.size(), Aj.size()));
            std::set_intersection(Ai.begin(), Ai.end(), Aj.begin(), Aj.end(), std::back_inserter(Ak));
            Ai.swap(Ak);
        }
        assert (Ai.size() > 0); // cannot happen if this algorithm is working correctly
        cliques.emplace_back(std::move(Ai), std::move(*Bi));
    }
    return std::move(cliques);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief independentCliqueSets
 ** ------------------------------------------------------------------------------------------------------------- */
template <unsigned side>
inline static BicliqueSet && independentCliqueSets(BicliqueSet && cliques, const unsigned minimum) {
    using IndependentSetGraph = adjacency_list<hash_setS, vecS, undirectedS, unsigned>;

    const auto l = cliques.size();
    IndependentSetGraph I(l);

    // Initialize our weights
    for (unsigned i = 0; i != l; ++i) {
        I[i] = std::pow(std::get<side>(cliques[i]).size(), 2);
    }

    // Determine our constraints
    for (unsigned i = 0; i != l; ++i) {
        for (unsigned j = i + 1; j != l; ++j) {
            if (intersects(std::get<side>(cliques[i]), std::get<side>(cliques[j]))) {
                add_edge(i, j, I);
            }
        }
    }

    // Use the greedy algorithm to choose our independent set
    VertexSet selected;
    for (;;) {
        unsigned w = 0;
        Vertex u = 0;
        for (unsigned i = 0; i != l; ++i) {
            if (I[i] > w) {
                w = I[i];
                u = i;
            }
        }
        if (w < minimum) break;
        selected.push_back(u);
        I[u] = 0;
        for (auto v : make_iterator_range(adjacent_vertices(u, I))) {
            I[v] = 0;
        }
    }

    // Sort the selected list and then remove the unselected cliques
    std::sort(selected.begin(), selected.end(), std::greater<Vertex>());
    auto end = cliques.end();
    for (const unsigned offset : selected) {
        end = cliques.erase(cliques.begin() + offset + 1, end) - 1;
    }
    cliques.erase(cliques.begin(), end);

    return std::move(cliques);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief removeUnhelpfulBicliques
 *
 * An intermediary vertex could have more than one outgoing edge but if any that are not directed to vertices in
 * the lower biclique, we'll need to compute that specific value anyway. Remove them from the clique set and if
 * there are not enough vertices in the biclique to make distribution profitable, eliminate the clique.
 ** ------------------------------------------------------------------------------------------------------------- */
static BicliqueSet && removeUnhelpfulBicliques(BicliqueSet && cliques, const DependencyGraph & G, DistributionGraph & H) {
    for (auto ci = cliques.begin(); ci != cliques.end(); ) {
        const auto cardinalityA = std::get<0>(*ci).size();
        VertexSet & B = std::get<1>(*ci);
        for (auto bi = B.begin(); bi != B.end(); ) {
            if (out_degree(H[*bi], G) == cardinalityA) {
                ++bi;
            } else {
                bi = B.erase(bi);
            }
        }
        if (B.size() > 1) {
            ++ci;
        } else {
            ci = cliques.erase(ci);
        }
    }
    return std::move(cliques);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief safeDistributionSets
 ** ------------------------------------------------------------------------------------------------------------- */
static DistributionSets safeDistributionSets(const DependencyGraph & G, DistributionGraph & H) {

    VertexSet sinks;
    for (const Vertex u : make_iterator_range(vertices(H))) {
        if (out_degree(u, H) == 0 && in_degree(u, H) != 0) {
            sinks.push_back(u);
        }
    }
    std::sort(sinks.begin(), sinks.end());

    DistributionSets T;
    BicliqueSet lowerSet = independentCliqueSets<1>(removeUnhelpfulBicliques(enumerateBicliques(H, sinks), G, H), 1);
    for (Biclique & lower : lowerSet) {
        BicliqueSet upperSet = independentCliqueSets<0>(enumerateBicliques(H, std::get<1>(lower)), 2);
        for (Biclique & upper : upperSet) {
            T.emplace_back(std::move(std::get<1>(upper)), std::move(std::get<0>(upper)), std::get<0>(lower));
        }
    }
    return std::move(T);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateDistributionGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void generateDistributionGraph(const DependencyGraph & G, DistributionGraph & H) {
    DistributionMap M;
    for (const Vertex u : make_iterator_range(vertices(G))) {
        if (in_degree(u, G) == 0 && out_degree(u, G) == 0) {
            continue;
        } else if (isDistributive(G[u])) {
            const TypeId outerTypeId = getType(G[u]);
            const TypeId innerTypeId = (outerTypeId == TypeId::And) ? TypeId::Or : TypeId::And;
            flat_set<Vertex> distributable;
            for (auto e : make_iterator_range(in_edges(u, G))) {
                const Vertex v = source(e, G);
                if (LLVM_UNLIKELY(getType(G[v]) == innerTypeId)) {
                    bool safe = true;
                    for (const auto e : make_iterator_range(out_edges(v, G))) {
                        if (getType(G[target(e, G)]) != outerTypeId) {
                            safe = false;
                            break;
                        }
                    }
                    if (safe) {
                        distributable.insert(v);
                    }
                }
            }
            if (LLVM_LIKELY(distributable.size() > 1)) {
                // We're only interested in computing a subgraph of G in which every source has an out-degree
                // greater than 1. Otherwise we'd only end up permuting the AND/OR sequence with no logical
                // benefit. (Note: this does not consider register pressure / liveness.)
                flat_map<Vertex, bool> observedMoreThanOnce;
                bool anyOpportunities = false;
                for (const Vertex v : distributable) {
                    for (auto e : make_iterator_range(in_edges(v, G))) {
                        const Vertex w = source(e, G);
                        auto ob = observedMoreThanOnce.find(w);
                        if (ob == observedMoreThanOnce.end()) {
                            observedMoreThanOnce.emplace(w, false);
                        } else {
                            ob->second = true;
                            anyOpportunities = true;
                        }
                    }
                }
                if (anyOpportunities) {
                    for (const auto ob : observedMoreThanOnce) {
                        if (std::get<1>(ob)) {
                            const Vertex w = std::get<0>(ob);
                            for (auto e : make_iterator_range(out_edges(w, G))) {
                                const Vertex v = target(e, G);
                                if (distributable.count(v)) {
                                    const Vertex y = getVertex(v, H, M);
                                    add_edge(y, getVertex(u, H, M), H);
                                    add_edge(getVertex(w, H, M), y, H);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief redistributeAST
 *
 * Apply the distribution law to reduce computations whenever possible.
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::redistributeAST(Graph & G) const {

    std::vector<Vertex> mapping(num_vertices(G) + 16);
    std::iota(mapping.begin(), mapping.end(), 0); // 0,1,.....n

    for (;;) {

        DistributionGraph H;

        generateDistributionGraph(G, H);

        // If we found no potential opportunities then we cannot apply the distribution law to any part of G.
        if (num_vertices(H) == 0) {
            break;
        }

        const DistributionSets distributionSets = safeDistributionSets(G, H);

        if (LLVM_UNLIKELY(distributionSets.empty())) {
            break;
        }

        for (const DistributionSet & set : distributionSets) {

            // Each distribution tuple consists of the sources, intermediary, and sink nodes.
            const VertexSet & sources = std::get<0>(set);
            const VertexSet & intermediary = std::get<1>(set);
            const VertexSet & sinks = std::get<2>(set);

            const TypeId outerTypeId = getType(G[mapping[H[sinks.front()]]]);
            assert (outerTypeId == TypeId::And || outerTypeId == TypeId::Or);
            const TypeId innerTypeId = (outerTypeId == TypeId::Or) ? TypeId::And : TypeId::Or;

            // Update G to match the desired change
            const Vertex x = addSummaryVertex(outerTypeId, G);
            const Vertex y = addSummaryVertex(innerTypeId, G);
            mapping.resize(num_vertices(G));
            mapping[x] = x;
            mapping[y] = y;

            for (const Vertex i : intermediary) {
                const auto u = mapping[H[i]];
                assert (getType(G[u]) == innerTypeId);
                for (const Vertex t : sinks) {
                    assert (getType(G[mapping[H[t]]]) == outerTypeId);
                    remove_edge(u, mapping[H[t]], G);
                }
                add_edge(nullptr, u, x, G);
            }

            for (const Vertex s : sources) {
                const auto u = mapping[H[s]];
                for (const Vertex i : intermediary) {
                    remove_edge(u, mapping[H[i]], G);
                }
                add_edge(nullptr, u, y, G);
            }
            add_edge(nullptr, x, y, G);

            for (const Vertex t : sinks) {
                const auto v = mapping[H[t]];
                add_edge(getValue(G[v]), y, v, G);
            }

            summarizeGraph(G, mapping);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addSummaryVertex
 ** ------------------------------------------------------------------------------------------------------------- */
inline Vertex BooleanReassociationPass::addSummaryVertex(const TypeId typeId, Graph & G) {
    return add_vertex(std::make_pair(typeId, nullptr), G);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getVertex
 ** ------------------------------------------------------------------------------------------------------------- */
Vertex BooleanReassociationPass::getSummaryVertex(PabloAST * expr, Graph & G, Map & M, const PabloBlock * const block) {
    const auto f = M.find(expr);
    if (f != M.end()) {
        return f->second;
    }
    // To simplify future analysis, every statement not in the current block will be recorded as a Var.
    const TypeId typeId = inCurrentBlock(expr, block) ? expr->getClassTypeId() : TypeId::Var;
    const auto u = add_vertex(std::make_pair(typeId, expr), G);
    M.insert(std::make_pair(expr, u));
    return u;
}

}
