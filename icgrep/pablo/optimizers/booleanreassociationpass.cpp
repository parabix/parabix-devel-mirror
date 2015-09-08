#include "booleanreassociationpass.h"
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/topological_sort.hpp>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <algorithm>
#include <queue>
#include <set>
#include <iostream>
#include <pablo/printer_pablos.h>
#include "graph-facade.hpp"

using namespace boost;
using namespace boost::container;

namespace pablo {

using Graph = BooleanReassociationPass::Graph;
using Vertex = Graph::vertex_descriptor;
using VertexQueue = circular_buffer<Vertex>;
using Map = BooleanReassociationPass::Map;
using EdgeQueue = std::queue<std::pair<Vertex, Vertex>>;
using TypeId = PabloAST::ClassTypeId;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool BooleanReassociationPass::optimize(PabloFunction & function) {
    BooleanReassociationPass brp;

    raw_os_ostream out(std::cerr);

    out << "BEFORE:\n";
    PabloPrinter::print(function.getEntryBlock().statements(), out);
    out << "----------------------------------------------------------\n";
    out.flush();

    brp.resolveScopes(function);
    brp.processScopes(function);
    Simplifier::optimize(function);

    out << "AFTER:\n";
    PabloPrinter::print(function.getEntryBlock().statements(), out);

    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resolveScopes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BooleanReassociationPass::resolveScopes(PabloFunction &function) {
    mResolvedScopes.emplace(&function.getEntryBlock(), nullptr);
    resolveScopes(function.getEntryBlock());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resolveScopes
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::resolveScopes(PabloBlock & block) {
    for (Statement * stmt : block) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            PabloBlock & nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
            mResolvedScopes.emplace(&nested, stmt);
            resolveScopes(nested);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScopes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BooleanReassociationPass::processScopes(PabloFunction & function) {
    std::vector<Statement *> terminals;
    for (unsigned i = 0; i != function.getNumOfResults(); ++i) {
        terminals.push_back(function.getResult(i));
    }
    processScopes(function.getEntryBlock(), std::move(terminals));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScopes
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::processScopes(PabloBlock & block, std::vector<Statement *> && terminals) {
    processScope(block, std::move(terminals));
    for (Statement * stmt : block) {
        if (isa<If>(stmt)) {
            const auto & defs = cast<const If>(stmt)->getDefined();
            std::vector<Statement *> terminals(defs.begin(), defs.end());
            processScopes(cast<If>(stmt)->getBody(), std::move(terminals));
        } else if (isa<While>(stmt)) {
            const auto & vars = cast<const While>(stmt)->getVariants();
            std::vector<Statement *> terminals(vars.begin(), vars.end());
            processScopes(cast<While>(stmt)->getBody(), std::move(terminals));
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isOptimizable
 *
 * And, Or and Xor instructions are all associative, commutative and distributive operations. Thus we can
 * safely rearrange expressions such as "((((a ∨ b) ∨ c) ∨ d) ∨ e) ∨ f" into "((a ∨ b) ∨ (c ∨ d)) ∨ (e ∨ f)".
 ** ------------------------------------------------------------------------------------------------------------- */
static inline bool isOptimizable(const PabloAST * const expr) {
    assert (expr);
    switch (expr->getClassTypeId()) {
        case PabloAST::ClassTypeId::And:
        case PabloAST::ClassTypeId::Or:
        case PabloAST::ClassTypeId::Xor:
            return true;
        default:
            return false;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief inCurrentBlock
 ** ------------------------------------------------------------------------------------------------------------- */
static inline bool inCurrentBlock(const Statement * stmt, const PabloBlock & block) {
    return stmt->getParent() == &block;
}

static inline bool inCurrentBlock(const PabloAST * expr, const PabloBlock & block) {
    return isa<Statement>(expr) && inCurrentBlock(cast<Statement>(expr), block);
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
 * @brief createTree
 ** ------------------------------------------------------------------------------------------------------------- */
static PabloAST * createTree(PabloBlock & block, const PabloAST::ClassTypeId typeId, circular_buffer<PabloAST *> & Q) {
    assert (!Q.empty());
    while (Q.size() > 1) {
        PabloAST * e1 = Q.front(); Q.pop_front();
        PabloAST * e2 = Q.front(); Q.pop_front();
        PabloAST * expr = nullptr;
        // Note: this switch ought to compile to an array of function pointers to the appropriate method.
        switch (typeId) {
            case PabloAST::ClassTypeId::And:
                expr = block.createAnd(e1, e2); break;
            case PabloAST::ClassTypeId::Or:
                expr = block.createOr(e1, e2); break;
            case PabloAST::ClassTypeId::Xor:
                expr = block.createXor(e1, e2); break;
            default: break;
        }
        Q.push_back(expr);
    }
    PabloAST * r = Q.front();
    Q.clear();
    return r;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printGraph
 ** ------------------------------------------------------------------------------------------------------------- */
static void printGraph(PabloBlock & block, const Graph & G, const std::string name) {
    raw_os_ostream out(std::cerr);
    out << "digraph " << name << " {\n";
    for (auto u : make_iterator_range(vertices(G))) {
//        if (in_degree(u, G) == 0 && out_degree(u, G) == 0) {
//            continue;
//        }
        out << "v" << u << " [label=\"";
        PabloAST * expr = G[u];
        if (isa<Statement>(expr)) {
            if (LLVM_UNLIKELY(isa<If>(expr))) {
                out << "If ";
                PabloPrinter::print(cast<If>(expr)->getOperand(0), out);
                out << ":";
            } else if (LLVM_UNLIKELY(isa<While>(expr))) {
                out << "While ";
                PabloPrinter::print(cast<While>(expr)->getOperand(0), out);
                out << ":";
            } else {
                PabloPrinter::print(cast<Statement>(expr), "", out);
            }
        } else {
            PabloPrinter::print(expr, out);
        }
        out << "\"";
        if (!inCurrentBlock(expr, block)) {
            out << " style=dashed";
        }
        out << "];\n";
    }
    for (auto e : make_iterator_range(edges(G))) {
        out << "v" << source(e, G) << " -> v" << target(e, G) << ";\n";
    }

    if (num_vertices(G) > 0) {

        out << "{ rank=same;";
        for (auto u : make_iterator_range(vertices(G))) {
            if (in_degree(u, G) == 0 && out_degree(u, G) != 0) {
                out << " v" << u;
            }
        }
        out << "}\n";

        out << "{ rank=same;";
        for (auto u : make_iterator_range(vertices(G))) {
            if (out_degree(u, G) == 0 && in_degree(u, G) != 0) {
                out << " v" << u;
            }
        }
        out << "}\n";

    }

    out << "}\n\n";
    out.flush();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printGraph
 ** ------------------------------------------------------------------------------------------------------------- */
template<typename SubgraphType>
static void printGraph(PabloBlock & block, const SubgraphType & S, const Graph & G, const std::string name) {
    raw_os_ostream out(std::cerr);
    out << "digraph " << name << " {\n";
    for (auto u : make_iterator_range(vertices(S))) {
        if (in_degree(u, S) == 0 && out_degree(u, S) == 0) {
            continue;
        }
        out << "v" << u << " [label=\"";
        PabloAST * expr = G[S[u]];
        if (isa<Statement>(expr)) {
            if (LLVM_UNLIKELY(isa<If>(expr))) {
                out << "If ";
                PabloPrinter::print(cast<If>(expr)->getOperand(0), out);
                out << ":";
            } else if (LLVM_UNLIKELY(isa<While>(expr))) {
                out << "While ";
                PabloPrinter::print(cast<While>(expr)->getOperand(0), out);
                out << ":";
            } else {
                PabloPrinter::print(cast<Statement>(expr), "", out);
            }
        } else {
            PabloPrinter::print(expr, out);
        }
        out << "\"";
        if (!inCurrentBlock(expr, block)) {
            out << " style=dashed";
        }
        out << "];\n";
    }
    for (auto e : make_iterator_range(edges(S))) {
        out << "v" << source(e, S) << " -> v" << target(e, S) << ";\n";
    }

    if (num_vertices(S) > 0) {

        out << "{ rank=same;";
        for (auto u : make_iterator_range(vertices(S))) {
            if (in_degree(u, S) == 0 && out_degree(u, S) != 0) {
                out << " v" << u;
            }
        }
        out << "}\n";

        out << "{ rank=same;";
        for (auto u : make_iterator_range(vertices(S))) {
            if (out_degree(u, S) == 0 && in_degree(u, S) != 0) {
                out << " v" << u;
            }
        }
        out << "}\n";

    }

    out << "}\n\n";
    out.flush();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScope
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BooleanReassociationPass::processScope(PabloBlock & block, std::vector<Statement *> && terminals) {
    Graph G;

    summarizeAST(block, G);
    redistributeAST(block, G);

    circular_buffer<Vertex> Q(num_vertices(G));
    topological_sort(G, std::front_inserter(Q));
    assert (Q.size() == num_vertices(G));

    circular_buffer<PabloAST *> nodes;
    block.setInsertPoint(block.back());

    while (!Q.empty()) {
        const Vertex u = Q.front(); Q.pop_front();
        PabloAST * expr = G[u];
        if (LLVM_LIKELY(inCurrentBlock(expr, block))) {
            if (isOptimizable(expr)) {
                if (in_degree(u, G) == 0) {
                    cast<Statement>(expr)->eraseFromParent(false);
                    continue;
                } else {
                    if (LLVM_UNLIKELY(nodes.capacity() < in_degree(u, G))) {
                        nodes.set_capacity(in_degree(u, G));
                    }
                    for (auto e : make_iterator_range(in_edges(u, G))) {
                        nodes.push_back(G[source(e, G)]);
                    }
                    assert (nodes.size() == in_degree(u, G));
                    PabloAST * replacement = createTree(block, expr->getClassTypeId(), nodes);

                    cast<Statement>(expr)->replaceWith(replacement, true, false);
                    expr = replacement;
                    G[u] = replacement;
                }
            }
            block.insert(cast<Statement>(expr));
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief summarizeAST
 *
 * This function scans through a basic block (starting by its terminals) and computes a DAG in which any sequences
 * of AND, OR or XOR functions are "flattened" (i.e., allowed to have any number of inputs.) This allows us to
 * reassociate them in the most efficient way possible.
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::summarizeAST(PabloBlock & block, Graph & G) const {

    Map M;

    // Compute the base def-use graph ...
    for (Statement * stmt : block) {
        const Vertex u = getVertex(stmt, G, M);
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            PabloAST * const op = stmt->getOperand(i);
            if (LLVM_UNLIKELY(isa<Integer>(op) || isa<String>(op))) {
                continue;
            }
            add_edge(getVertex(op, G, M), u, G);
        }
        if (isa<If>(stmt)) {
            for (Assign * def : cast<const If>(stmt)->getDefined()) {
                const Vertex v = getVertex(def, G, M);
                add_edge(u, v, G);
                resolveUsages(v, def, block, G, M);
            }
        } else if (isa<While>(stmt)) {
            for (Next * var : cast<const While>(stmt)->getVariants()) {
                const Vertex v = getVertex(var, G, M);
                add_edge(u, v, G);
                resolveUsages(v, var, block, G, M);
            }
        } else {
            resolveUsages(u, stmt, block, G, M);
        }
    }

    std::vector<Vertex> ordering;
    ordering.reserve(num_vertices(G));
    topological_sort(G, std::back_inserter(ordering));

    for (auto i = ordering.rbegin(); i != ordering.rend(); ++i) {
        const Vertex u = *i;
        if (isOptimizable(G[u])) {
            std::vector<Vertex> removable;
            for (auto e : make_iterator_range(in_edges(u, G))) {
                const Vertex v = source(e, G);
                if (out_degree(v, G) == 1 && in_degree(v, G) != 0 && G[u]->getClassTypeId() == G[v]->getClassTypeId()) {
                    removable.push_back(v);
                    for (auto e : make_iterator_range(in_edges(v, G))) {
                        add_edge(source(e, G), u, G);
                    }
                }
            }
            for (auto v : removable) {
                clear_vertex(v, G);
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resolveUsages
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::resolveUsages(const Vertex u, PabloAST * expr, PabloBlock & block, Graph & G, Map & M) const {
    for (PabloAST * user : expr->users()) {
        assert (user);
        if (LLVM_LIKELY(user != expr && isa<Statement>(user))) {
            PabloBlock * parent = cast<Statement>(user)->getParent();
            assert (parent);
            if (LLVM_UNLIKELY(parent != &block)) {
                for (;;) {
                    if (LLVM_UNLIKELY(parent == nullptr)) {
                        assert (isa<Assign>(expr) || isa<Next>(expr));
                        break;
                    } else if (parent->getParent() == &block) {
                        const auto f = mResolvedScopes.find(parent);
                        if (LLVM_UNLIKELY(f == mResolvedScopes.end())) {
                            throw std::runtime_error("Failed to resolve scope block!");
                        }
                        add_edge(u, getVertex(f->second, G, M), G);
                        break;
                    }
                    parent = parent->getParent();
                }
            }
        }
    }
}

using VertexSet = std::vector<Vertex>;
using VertexSets = std::vector<VertexSet>;

template <class Graph>
static VertexSet incomingVertexSet(const Vertex u, const Graph & G) {
    VertexSet V;
    V.reserve(G.in_degree(u));
    for (auto e : make_iterator_range(G.in_edges(u))) {
        V.push_back(G.source(e));
    }
    std::sort(V.begin(), V.end());
    return std::move(V);
}

template <class Graph>
static VertexSet outgoingVertexSet(const Vertex u, const Graph & G) {
    VertexSet V;
    V.reserve(G.out_degree(u));
    for (auto e : make_iterator_range(G.out_edges(u))) {
        V.push_back(G.target(e));
    }
    std::sort(V.begin(), V.end());
    return std::move(V);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerateBicliques
 *
 * Adaptation of the MICA algorithm as described in "Consensus algorithms for the generation of all maximal
 * bicliques" by Alexe et. al. (2003). Note: this implementation considers all verticies with an in-degree of 0
 * to be in bipartition A and their adjacencies to be in B.
  ** ------------------------------------------------------------------------------------------------------------- */
template <class Graph>
static VertexSets enumerateBicliques(const Graph & G) {
    using IntersectionSets = std::set<VertexSet>;

    IntersectionSets B1;
    for (auto u : make_iterator_range(G.vertices())) {
        if (G.in_degree(u) == 0) {
            B1.insert(std::move(outgoingVertexSet(u, G)));
        }
    }

    IntersectionSets C(B1.begin(), B1.end());

    IntersectionSets Bi;
    VertexSet clique;
    for (auto i = B1.begin(); i != B1.end(); ++i) {
        for (auto j = i; ++j != B1.end(); ) {
            std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(clique));
            if (clique.size() > 0) {
                if (C.count(clique) == 0) {
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
        C.insert(Bi.begin(), Bi.end());
        IntersectionSets Bk;
        for (auto i = B1.begin(); i != B1.end(); ++i) {
            for (auto j = Bi.begin(); j != Bi.end(); ++j) {
                std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(clique));
                if (clique.size() > 0) {
                    if (C.count(clique) == 0) {
                        Bk.insert(clique);
                    }
                    clique.clear();
                }
            }
        }
        Bi.swap(Bk);
    }
    return VertexSets(C.begin(), C.end());
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
 * @brief maximalIndependentSet
 ** ------------------------------------------------------------------------------------------------------------- */
static VertexSets && maximalIndependentSet(VertexSets && V) {
    using IndependentSetGraph = adjacency_list<hash_setS, vecS, undirectedS, unsigned>;
    const auto l = V.size();
    IndependentSetGraph I(l);
    // Initialize our weights
    for (unsigned i = 0; i != l; ++i) {
        I[i] = V[i].size();
    }
    // Determine our constraints
    for (unsigned i = 0; i != l; ++i) {
        for (unsigned j = i + 1; j != l; ++j) {
            if (intersects(V[i], V[j])) {
                add_edge(i, j, I);
            }
        }
    }
    // Use the greedy algorithm to choose our independent set (TODO: try the similar randomized approach)
    VertexSet selected;
    std::vector<bool> ignored(l, false);
    for (;;) {
        unsigned w = 0;
        Vertex u = 0;
        for (unsigned i = 0; i != l; ++i) {
            if (!ignored[i] && I[i] > w) {
                w = I[i];
                u = i;
            }
        }
        if (w == 0) break;
        selected.push_back(u);
        ignored[u] = true;
        for (auto v : make_iterator_range(adjacent_vertices(u, I))) {
            ignored[v] = true;
        }
    }
    // Sort the selected list and then remove the unselected sets from V
    std::sort(selected.begin(), selected.end(), std::greater<Vertex>());
    auto end = V.end();
    for (const unsigned offset : selected) {
        end = V.erase(V.begin() + offset + 1, end) - 1;
    }
    V.erase(V.begin(), end);
    return std::move(V);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief sinkIndependentMaximalBicliques
 ** ------------------------------------------------------------------------------------------------------------- */
template <class Graph>
static VertexSets sinkIndependentMaximalBicliques(Graph && G) {
    VertexSets B = maximalIndependentSet(std::move(enumerateBicliques(G)));
    VertexSets A;
    A.reserve(B.size());
    for (const VertexSet & Bi : B) {
        // Compute our A set
        auto bi = Bi.begin();
        VertexSet Ai = incomingVertexSet(*bi, G);
        while (++bi != Bi.end()) {
            VertexSet Ai = incomingVertexSet(*bi, G);
            VertexSet Ak;
            std::set_intersection(Ai.begin(), Ai.end(), Ai.begin(), Ai.end(), std::back_inserter(Ak));
            Ai.swap(Ak);
        }
        A.emplace_back(std::move(Ai));
    }
    std::vector<Vertex> sources;
    std::vector<Vertex> intermediary;
    for (auto u : make_iterator_range(G.vertices())) {
        if (G.in_degree(u) == 0) {
            sources.push_back(u);
        } else if (G.out_degree(u) != 0) {
            intermediary.push_back(u);
        }
    }
    for (auto u : sources) {
        G.clear_out_edges(u);
    }
    for (unsigned i = 0; i != B.size(); ++i) {
        for (auto u : A[i]) {
            for (auto v : B[i]) {
                G.add_edge(u, v);
            }
        }
    }
    for (auto u : intermediary) {
        if (G.in_degree(u) == 0) {
            G.clear_out_edges(u);
        }
    }
    return std::move(A);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief safeDistributionSets
 ** ------------------------------------------------------------------------------------------------------------- */
template <class Graph>
static VertexSets safeDistributionSets(Graph & G) {
    VertexSets sinks = sinkIndependentMaximalBicliques(makeTransposedGraphFacade(G));
    // Scan through G and replicate any source that has more than one sink until G is broken
    // into weakly connected components in which each has exactly one sink.
    if (sinks.size() > 1) {
        std::vector<unsigned> component(num_vertices(G), 0);
        unsigned components = 0;
        for (const VertexSet & S : sinks) {
            ++components;
            for (auto e : make_iterator_range(in_edges(S.front(), G))) {
                component[source(e, G)] = components;
            }
        }
        for (const Vertex u : make_iterator_range(vertices(G))) {
            if (LLVM_UNLIKELY(in_degree(u, G) == 0)) {
                flat_set<unsigned> membership;
                for (auto e : make_iterator_range(out_edges(u, G))) {
                    membership.insert(component[target(e, G)]);
                }
                if (LLVM_UNLIKELY(membership.size() > 1)) {
                    VertexSet adjacent;
                    adjacent.reserve(out_degree(u, G));
                    for (auto e : make_iterator_range(out_edges(u, G))) {
                        adjacent.push_back(target(e, G));
                    }
                    clear_out_edges(u, G);
                    auto mi = membership.begin();
                    for (Vertex w = u; ;) {
                        const unsigned m = *mi;
                        for (auto v : adjacent) {
                            if (component[v] == m) {
                                add_edge(w, v, G);
                            }
                        }
                        if (++mi == membership.end()) {
                            break;
                        }
                        w = add_vertex(G[u], G);
                    }
                }
            }
        }
    }
    return std::move(sinkIndependentMaximalBicliques(makeGraphFacade(G)));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief segmentInto3LevelGraph
 *
 * Ensure that every source-to-sink path in G has an edge-distance of exactly 2. The safeDistributionSets algorithm
 * expects that G exibits this property but if an input to a distributable function is also the output of another
 * distributable function, this complicates the analysis process. Thus method resolves that by replicating the
 * appropriate vertices into input-only and output-only vertices.
 ** ------------------------------------------------------------------------------------------------------------- */
template <class Graph>
Graph & segmentInto3LevelGraph(Graph & G) {
    std::vector<unsigned> distance(num_vertices(G), 0);
    std::queue<Vertex> Q;
    for (const Vertex u : make_iterator_range(vertices(G))) {
        if (in_degree(u, G) == 0) {
            distance[u] = 1;
            for (const auto e : make_iterator_range(out_edges(u, G))) {
                Q.push(target(e, G));
            }
        }
    }
    for (;;) {
        const Vertex u = Q.front(); Q.pop();
        unsigned dist = 0;
        bool ready = true;
        for (const auto e : make_iterator_range(in_edges(u, G))) {
            const Vertex v = source(e, G);
            if (LLVM_UNLIKELY(distance[v] == 0)) {
                ready = false;
                break;
            }
            dist = std::max(dist, distance[v]);
        }
        assert (dist <= 4);
        if (ready) {
            if (LLVM_UNLIKELY(dist == 4)) {
                for (const auto e : make_iterator_range(in_edges(u, G))) {
                    const Vertex v = source(e, G);
                    if (distance[v] == 3) {
                        const Vertex w = add_vertex(G[v], G);
                        for (const auto e : make_iterator_range(out_edges(v, G))) {
                            add_edge(w, target(e, G), G);
                        }
                        clear_out_edges(v, G);
                        assert (w == distance.size());
                        distance.push_back(0);
                        Q.push(w);
                    }
                }
            } else { // update the distance and add in the adjacent vertices to Q
                distance[u] = dist + 1;
                for (const auto e : make_iterator_range(out_edges(u, G))) {
                    Q.push(target(e, G));
                }
            }
        }
        if (Q.empty()) {
            break;
        }
    }
    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief redistributeAST
 *
 * Apply the distribution law to reduce computations whenever possible.
 ** ------------------------------------------------------------------------------------------------------------- */
bool BooleanReassociationPass::redistributeAST(PabloBlock & block, Graph & G) const {
    bool anyChanges = false;
    for (;;) {

        adjacency_list<hash_setS, vecS, bidirectionalS, Vertex> H;
        flat_map<Vertex, Vertex> M;

        for (const Vertex u : make_iterator_range(vertices(G))) {
            const TypeId outerTypeId = G[u]->getClassTypeId();
            if (outerTypeId == TypeId::And || outerTypeId == TypeId::Or) {
                if (inCurrentBlock(cast<Statement>(G[u]), block)) {
                    const TypeId innerTypeId = (outerTypeId == TypeId::And) ? TypeId::Or : TypeId::And;
                    flat_set<Vertex> distributable;
                    for (auto e : make_iterator_range(in_edges(u, G))) {
                        const Vertex v = source(e, G);
                        if (LLVM_UNLIKELY(G[v]->getClassTypeId() == innerTypeId) && LLVM_LIKELY(inCurrentBlock(cast<Statement>(G[v]), block))) {
                            bool safe = true;
                            for (const auto e : make_iterator_range(out_edges(v, G))) {
                                if (G[target(e, G)]->getClassTypeId() != outerTypeId) {
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
                        flat_map<Vertex, unsigned> observed;
                        bool anyOpportunities = false;
                        for (const Vertex v : distributable) {
                            for (auto e : make_iterator_range(in_edges(v, G))) {
                                const Vertex w = source(e, G);
                                const auto f = observed.find(w);
                                if (f == observed.end()) {
                                    observed.emplace(w, 0);
                                } else {
                                    f->second += 1;
                                    anyOpportunities = true;
                                }
                            }
                        }
                        if (anyOpportunities) {
                            for (auto ob : observed) {
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

        // If we found no potential opportunities then we cannot apply the distribution law to any part of G.
        if (num_vertices(H) == 0) {
            break;
        }

        printGraph(block, H, G, "H1");

        const VertexSets distributionSets = safeDistributionSets(segmentInto3LevelGraph(H));

        printGraph(block, H, G, "H2");

        // If no sources remain, no bicliques were found that would have a meaningful impact on the AST.
        if (LLVM_UNLIKELY(distributionSets.size() == 0)) {
            break;
        }

        for (const VertexSet & sources : distributionSets) {
            assert (sources.size() > 0);
            const VertexSet intermediary = outgoingVertexSet(sources.front(), makeGraphFacade(H));
            assert (intermediary.size() > 0);
            const VertexSet sinks = outgoingVertexSet(intermediary.front(), makeGraphFacade(H));
            assert (sinks.size() > 0);

            // Now begin transforming the AST
            const TypeId typeId = G[H[sinks.front()]]->getClassTypeId();
            assert (typeId == TypeId::And || typeId == TypeId::Or);

            circular_buffer<PabloAST *> Q(std::max(sources.size(), intermediary.size() + 1));
            for (const Vertex u : intermediary) {
                Q.push_back(G[H[u]]);
            }
            PabloAST * merged = createTree(block, typeId, Q);
            for (const Vertex s : sources) {
                Q.push_back(G[H[s]]);
            }
            Q.push_back(merged);
            PabloAST * masked = createTree(block, typeId == TypeId::Or ? TypeId::And : TypeId::Or, Q);

            // Eliminate the edges from our original graph
            for (const Vertex u : intermediary) {
                for (const Vertex s : sources) {
                    remove_edge(H[s], H[u], G);
                }
                for (const Vertex t : sinks) {
                    remove_edge(H[u], H[t], G);
                }
            }

            // Finally update G to match the desired changes
            const Vertex x = add_vertex(merged, G);
            const Vertex y = add_vertex(masked, G);
            for (const Vertex u : intermediary) {
                add_edge(H[u], x, G);
            }
            add_edge(x, y, G);
            for (const Vertex s : sources) {
                add_edge(H[s], y, G);
            }
            for (const Vertex t : sinks) {
                add_edge(y, H[t], G);
            }
            for (const Vertex u : intermediary) {
                if (LLVM_UNLIKELY(in_degree(H[u], G) == 0)) {
                    clear_out_edges(H[u], G);
                }
            }
        }
        anyChanges = true;
    }
    return anyChanges;
}

}
