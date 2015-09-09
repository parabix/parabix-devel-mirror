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
using DistributionGraph = adjacency_list<hash_setS, vecS, bidirectionalS, Vertex>;
using DistributionMap = flat_map<Graph::vertex_descriptor, DistributionGraph::vertex_descriptor>;
using VertexSet = std::vector<Vertex>;
using VertexSets = std::vector<VertexSet>;
using Biclique = std::pair<VertexSet, VertexSet>;
using BicliqueSet = std::vector<Biclique>;
using DistributionSet = std::tuple<std::vector<Vertex>, std::vector<Vertex>, std::vector<Vertex>>;
using DistributionSets = std::vector<DistributionSet>;

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
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool BooleanReassociationPass::optimize(PabloFunction & function) {
    BooleanReassociationPass brp;
    brp.resolveScopes(function);
    brp.processScopes(function);
    Simplifier::optimize(function);
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
    processScopes(function.getEntryBlock());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScopes
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::processScopes(PabloBlock & block) {
    for (Statement * stmt : block) {
        if (isa<If>(stmt)) {
            processScopes(cast<If>(stmt)->getBody());
        } else if (isa<While>(stmt)) {
            processScopes(cast<While>(stmt)->getBody());
        }
    }
    processScope(block);
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
        // assert (isa<Statement>(e1) ? cast<Statement>(e1)->getParent() != nullptr : true);
        PabloAST * e2 = Q.front(); Q.pop_front();
        // assert (isa<Statement>(e2) ? cast<Statement>(e2)->getParent() != nullptr : true);
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
        if (in_degree(u, G) == 0 && out_degree(u, G) == 0) {
            continue;
        }
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
inline void BooleanReassociationPass::processScope(PabloBlock & block) {
    Graph G;

    summarizeAST(block, G);
    redistributeAST(block, G);

    circular_buffer<Vertex> Q(num_vertices(G));
    topological_sort(G, std::front_inserter(Q));
    assert (Q.size() == num_vertices(G));

    circular_buffer<PabloAST *> nodes;
    block.setInsertPoint(nullptr);

    while (!Q.empty()) {
        const Vertex u = Q.front();
        Q.pop_front();
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
                    G[u] = createTree(block, expr->getClassTypeId(), nodes);
                    cast<Statement>(expr)->replaceWith(G[u], true, true);
                    if (LLVM_UNLIKELY(isa<Var>(G[u]))) {
                        continue;
                    }
                    expr = G[u];
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerateBicliques
 *
 * Adaptation of the MICA algorithm as described in "Consensus algorithms for the generation of all maximal
 * bicliques" by Alexe et. al. (2003). Note: this implementation considers all verticies with an in-degree of 0
 * to be in bipartition A and their adjacencies to be in B.
  ** ------------------------------------------------------------------------------------------------------------- */
template <class Graph>
static BicliqueSet enumerateBicliques(const Graph & G, const VertexSet & A) {
    using IntersectionSets = std::set<VertexSet>;

    IntersectionSets B1;
    for (auto u : A) {
        B1.insert(std::move(incomingVertexSet(u, G)));
    }

    IntersectionSets B(B1.begin(), B1.end());

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
        // Compute our A set
        auto bi = Bi->begin();
        VertexSet Ai = outgoingVertexSet(*bi, G);
        while (++bi != Bi->end()) {
            VertexSet Aj = outgoingVertexSet(*bi, G);
            VertexSet Ak;
            std::set_intersection(Ai.begin(), Ai.end(), Aj.begin(), Aj.end(), std::back_inserter(Ak));
            Ai.swap(Ak);
        }
        cliques.emplace_back(std::move(Ai), std::move(*Bi));
    }

    return std::move(cliques);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief safeDistributionSets
 ** ------------------------------------------------------------------------------------------------------------- */
static DistributionSets safeDistributionSets(DistributionGraph & H, const Graph & G) {

    VertexSet terminals;
    for (const Vertex u : make_iterator_range(vertices(H))) {
        if (out_degree(u, H) == 0) {
            terminals.push_back(u);
        }
    }

    BicliqueSet lowerSet = enumerateBicliques(makeGraphFacade(H), terminals);
    // An intermediary vertex could have more than one outgoing edge but if any edge
    // is not directed to a vertex in our biclique, we'll need to compute that specific
    // value anyway. Remove them from the clique set and if there are not enough vertices
    // in the biclique to make distribution profitable, eliminate the clique.
    for (auto ci = lowerSet.begin(); ci != lowerSet.end(); ) {
        VertexSet & A = std::get<0>(*ci);
        VertexSet & B = std::get<1>(*ci);
        const auto cardinalityA = A.size();
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
            ci = lowerSet.erase(ci);
        }
    }

    // Each distribution tuple consists of the sources, intermediary, and sink nodes.
    DistributionSets T;
    for (const Biclique & lower : lowerSet) {
        BicliqueSet upperSet = enumerateBicliques(makeGraphFacade(H), std::get<1>(lower));
        for (Biclique upper : upperSet) {
            T.emplace_back(std::get<1>(upper), std::get<0>(upper), std::get<0>(lower));
        }
    }

    return std::move(T);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateDistributionGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void generateDistributionGraph(PabloBlock & block, Graph & G, DistributionGraph & H) {
    DistributionMap M;
    for (const Vertex u : make_iterator_range(vertices(G))) {
        const TypeId outerTypeId = G[u]->getClassTypeId();
        if (outerTypeId == TypeId::And || outerTypeId == TypeId::Or) {
            if (inCurrentBlock(cast<Statement>(G[u]), block)) {
                const TypeId innerTypeId = (outerTypeId == TypeId::And) ? TypeId::Or : TypeId::And;
                flat_set<Vertex> distributable;
                for (auto e : make_iterator_range(in_edges(u, G))) {
                    const Vertex v = source(e, G);
                    if (LLVM_UNLIKELY(G[v]->getClassTypeId() == innerTypeId && inCurrentBlock(cast<Statement>(G[v]), block))) {
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
                            if (ob.second) {
                                const Vertex w = ob.first;
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
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief redistributeAST
 *
 * Apply the distribution law to reduce computations whenever possible.
 ** ------------------------------------------------------------------------------------------------------------- */
bool BooleanReassociationPass::redistributeAST(PabloBlock & block, Graph & G) const {
    bool anyChanges = false;
    for (;;) {

        DistributionGraph H;

        generateDistributionGraph(block, G, H);

        // If we found no potential opportunities then we cannot apply the distribution law to any part of G.
        if (num_vertices(H) == 0) {
            break;
        }

        DistributionSets distributionSets = safeDistributionSets(H, G);

        if (LLVM_UNLIKELY(distributionSets.empty())) {
            break;
        }


        break;



//        for (const Vertex s : make_iterator_range(vertices(H))) {

//            if (in_degree(s, H) == 0) {

//                assert (sources.size() > 0);
//                const VertexSet intermediary = outgoingVertexSet(sources.front(), makeGraphFacade(H));
//                assert (intermediary.size() > 0);
//                const VertexSet sinks = outgoingVertexSet(intermediary.front(), makeGraphFacade(H));
//                assert (sinks.size() > 0);

//                // Now begin transforming the AST
//                circular_buffer<PabloAST *> Q(std::max(sources.size(), intermediary.size() + 1));
//                for (const Vertex u : intermediary) {
//                    Q.push_back(G[H[u]]);
//                }
//                const TypeId typeId = G[H[sinks.front()]]->getClassTypeId();
//                assert (typeId == TypeId::And || typeId == TypeId::Or);
//                PabloAST * merged = createTree(block, typeId, Q);
//                for (const Vertex s : sources) {
//                    Q.push_back(G[H[s]]);
//                }
//                Q.push_back(merged);
//                PabloAST * masked = createTree(block, (typeId == TypeId::Or) ? TypeId::And : TypeId::Or, Q);

//                // Eliminate the edges from our original graph G
//                for (const Vertex u : intermediary) {
//                    const auto uu = H[u];
//                    for (const Vertex s : sources) {
//                        assert(edge(H[s], uu, G).second);
//                        remove_edge(H[s], uu, G);
//                    }
//                    for (const Vertex t : sinks) {
//                        assert(edge(uu, H[t], G).second);
//                        remove_edge(uu, H[t], G);
//                    }
//                }

//                // Finally update G to match the desired changes
//                const Vertex x = add_vertex(merged, G);
//                const Vertex y = add_vertex(masked, G);
//                for (const Vertex u : intermediary) {
//                    add_edge(H[u], x, G);
//                }
//                add_edge(x, y, G);
//                for (const Vertex s : sources) {
//                    add_edge(H[s], y, G);
//                }
//                for (const Vertex t : sinks) {
//                    add_edge(y, H[t], G);
//                }

//            }

//        }
//        anyChanges = true;

    }

    return anyChanges;
}

}
