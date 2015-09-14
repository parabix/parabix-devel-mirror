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
#include <queue>
#include <set>
#include <iostream>
#include <pablo/printer_pablos.h>
#include "graph-facade.hpp"

using namespace boost;
using namespace boost::container;

namespace pablo {

using TypeId = PabloAST::ClassTypeId;
using Graph = BooleanReassociationPass::Graph;
using Vertex = Graph::vertex_descriptor;
using VertexQueue = circular_buffer<Vertex>;
using Map = BooleanReassociationPass::Map;
using EdgeQueue = std::queue<std::pair<Vertex, Vertex>>;
using DistributionGraph = adjacency_list<hash_setS, vecS, bidirectionalS, Vertex>;
using DistributionMap = flat_map<Graph::vertex_descriptor, DistributionGraph::vertex_descriptor>;
using VertexSet = std::vector<Vertex>;
using VertexSets = std::vector<VertexSet>;
using Biclique = std::pair<VertexSet, VertexSet>;
using BicliqueSet = std::vector<Biclique>;
using DistributionSet = std::tuple<VertexSet, VertexSet, VertexSet>;
using DistributionSets = std::vector<DistributionSet>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief helper functions
 ** ------------------------------------------------------------------------------------------------------------- */
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

template <class Graph>
static VertexSet sinks(const Graph & G) {
    VertexSet V;
    for (const Vertex u : make_iterator_range(vertices(G))) {
        if (out_degree(u, G) == 0) {
            V.push_back(u);
        }
    }
    std::sort(V.begin(), V.end());
    return std::move(V);
}

template<typename Iterator>
inline Graph::edge_descriptor first(const std::pair<Iterator, Iterator> & range) {
    assert (range.first != range.second);
    return *range.first;
}

inline void add_edge(PabloAST * expr, const Vertex u, const Vertex v, Graph & G) {
    assert (u != v);
    G[add_edge(u, v, G).first] = expr;
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
    processScopes(function, function.getEntryBlock());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScopes
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::processScopes(PabloFunction & function, PabloBlock & block) {
    for (Statement * stmt : block) {
        if (isa<If>(stmt)) {
            processScopes(function, cast<If>(stmt)->getBody());
        } else if (isa<While>(stmt)) {
            processScopes(function, cast<While>(stmt)->getBody());
        }
    }
    processScope(function, block);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief inCurrentBlock
 ** ------------------------------------------------------------------------------------------------------------- */
static inline bool inCurrentBlock(const Statement * stmt, const PabloBlock & block) {
    return stmt->getParent() == &block;
}

static inline bool inCurrentBlock(const PabloAST * expr, const PabloBlock & block) {
    return expr ? isa<Statement>(expr) && inCurrentBlock(cast<Statement>(expr), block) : true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isOptimizable
 *
 * And, Or and Xor instructions are all associative, commutative and distributive operations. Thus we can
 * safely rearrange expressions such as "((((a ∨ b) ∨ c) ∨ d) ∨ e) ∨ f" into "((a ∨ b) ∨ (c ∨ d)) ∨ (e ∨ f)".
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool BooleanReassociationPass::isOptimizable(const VertexData & data) {
    switch (data.first) {
        case PabloAST::ClassTypeId::And:
        case PabloAST::ClassTypeId::Or:
        case PabloAST::ClassTypeId::Xor:
            return true;
        default:
            return false;
    }
}

inline bool isNegated(const BooleanReassociationPass::VertexData & data) {
    return (data.first == TypeId::Not) && (data.second != nullptr);
}

inline bool BooleanReassociationPass::isMutable(const VertexData & data, const PabloBlock &) {
    return (data.first != TypeId::Var);
}

inline bool BooleanReassociationPass::isNonEscaping(const VertexData & data) {
    return data.first != TypeId::Assign && data.first != TypeId::Next;
}

inline bool BooleanReassociationPass::isSameType(const VertexData & data1, const VertexData & data2) {
    return data1.first == data2.first;
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
 * @brief printGraph
 ** ------------------------------------------------------------------------------------------------------------- */
static void printGraph(const PabloBlock & block, const Graph & G, const std::string name) {
    raw_os_ostream out(std::cerr);
    std::vector<unsigned> visible(num_vertices(G), false);
    out << "digraph " << name << " {\n";
    for (auto u : make_iterator_range(vertices(G))) {
        if (in_degree(u, G) == 0 && out_degree(u, G) == 0) {
            continue;
        }
        visible[u] = true;

        out << "v" << u << " [label=\"" << u << ": ";
        PabloAST * expr;
        TypeId typeId;
        std::tie(typeId, expr) = G[u];
        bool temporary = false;
        bool error = false;
        if (expr == nullptr) {
            temporary = true;
            switch (typeId) {
                case TypeId::And:
                    out << "And";
                    break;
                case TypeId::Or:
                    out << "Or";
                    break;
                case TypeId::Xor:
                    out << "Xor";
                    break;
                default:
                    out << "???";
                    error = true;
                    break;
            }
        } else if (G[u].first != TypeId::Var) {
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
        if (!BooleanReassociationPass::isMutable(G[u], block)) {
            out << " style=dashed";
        }
        if (error) {
            out << " color=red";
        } else if (temporary) {
            out << " color=blue";
        }
        out << "];\n";
    }
    for (auto e : make_iterator_range(edges(G))) {
        if (visible[source(e, G)] && visible[target(e, G)]) {
            out << "v" << source(e, G) << " -> v" << target(e, G);
        }
        if (G[e]) {
             out << " [label=\"";
             PabloPrinter::print(G[e], out);
             out << "\"]";
        }
        out << ";\n";
    }

    if (num_vertices(G) > 0) {

        out << "{ rank=same;";
        for (auto u : make_iterator_range(vertices(G))) {
            if (visible[u] && in_degree(u, G) == 0 && out_degree(u, G) != 0) {
                out << " v" << u;
            }
        }
        out << "}\n";

        out << "{ rank=same;";
        for (auto u : make_iterator_range(vertices(G))) {
            if (visible[u] && out_degree(u, G) == 0 && in_degree(u, G) != 0) {
                out << " v" << u;
            }
        }
        out << "}\n";

    }

    out << "}\n\n";
    out.flush();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createTree
 ** ------------------------------------------------------------------------------------------------------------- */
static PabloAST * createTree(PabloBlock & block, const Vertex u, Graph & G) {
    flat_set<PabloAST *> sources;
    for (const auto e : make_iterator_range(in_edges(u, G))) {
        PabloAST * expr = G[source(e, G)].second;
        assert ("G contains a null input variable!" && (expr != nullptr));
        sources.insert(expr);
    }
    circular_buffer<PabloAST *> Q(sources.begin(), sources.end());
    const TypeId typeId = G[u].first;
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
    PabloAST * const result = Q.front();
    assert (result);
    G[u].second = result;
    return result;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScope
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BooleanReassociationPass::processScope(PabloFunction & function, PabloBlock & block) {
    Graph G;

//    raw_os_ostream out(std::cerr);
//    out << "============================================================\n";
//    PabloPrinter::print(block.statements(), " > ", out);
//    out << "------------------------------------------------------------\n";
//    out.flush();

    summarizeAST(block, G);
    redistributeAST(block, G);

    circular_buffer<Vertex> Q(num_vertices(G));
    topological_sort(G, std::back_inserter(Q));
    block.setInsertPoint(nullptr);

    while (!Q.empty()) {
        const Vertex u = Q.back(); Q.pop_back();
        if (LLVM_LIKELY(isMutable(G[u], block))) {
//            out << "Mutable: " << u << ": ";
//            PabloPrinter::print(G[u].second, out);
//            out << '\n';
//            out.flush();
            Statement * stmt = nullptr;
            if (isOptimizable(G[u])) {
                PabloAST * replacement = createTree(block, u, G);
                if (LLVM_LIKELY(inCurrentBlock(replacement, block))) {
                    stmt = cast<Statement>(replacement);
                } else { // optimization reduced this to a Constant, Var or an outer-scope statement
                    continue;
                }
            } else { // update any potential mappings
                stmt = cast<Statement>(G[u].second);
            }
            assert (stmt);
            assert (inCurrentBlock(stmt, block));
            for (auto e : make_iterator_range(out_edges(u, G))) {
                if (G[e] && G[e] != stmt) {
                    PabloAST * expr = G[target(e, G)].second;
                    if (expr) { // processing a yet-to-be created value
                        cast<Statement>(expr)->replaceUsesOfWith(G[e], stmt);
                    }
                }
            }
            block.insert(stmt);
        }
    }

//    Simplifier::deadCodeElimination(block);

//    out << "------------------------------------------------------------\n";
//    PabloPrinter::print(block.statements(), " < ", out);
//    out.flush();

//    PabloVerifier::verify(function);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getVertex
 ** ------------------------------------------------------------------------------------------------------------- */
static inline Vertex getVertex(PabloAST * expr, Graph & G, Map & M, const PabloBlock & block) {
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief summarizeAST
 *
 * This function scans through a scope blockand computes a DAG G in which any sequences of AND, OR or XOR functions
 * are "flattened" (i.e., allowed to have any number of inputs.)
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::summarizeAST(PabloBlock & block, Graph & G) const {
    Map M;
    // Compute the base def-use graph ...
    for (Statement * stmt : block) {
        const Vertex u = getVertex(stmt, G, M, block);
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            PabloAST * const op = stmt->getOperand(i);
            if (LLVM_UNLIKELY(isa<Integer>(op) || isa<String>(op))) {
                continue;
            }
            const Vertex v = getVertex(op, G, M, block);
            if (!edge(v, u, G).second) {
                add_edge(op, v, u, G);
            }
            // If this operand is a Not operation that is not in this PabloBlock,
            // pull its input operand in. It may lead to future optimizations.
            if (LLVM_UNLIKELY(isa<Not>(op) && !inCurrentBlock(cast<Not>(op), block))) {
                PabloAST * const neg = cast<Not>(op)->getExpr();
                const Vertex w = getVertex(neg, G, M, block);
                if (!edge(w, v, G).second) {
                    add_edge(neg, w, v, G);
                }
            }
        }
        if (isa<If>(stmt)) {
            for (Assign * def : cast<const If>(stmt)->getDefined()) {
                const Vertex v = getVertex(def, G, M, block);
                add_edge(def, u, v, G);
                resolveUsages(v, def, block, G, M, stmt);
            }
        } else if (isa<While>(stmt)) {
            for (Next * var : cast<const While>(stmt)->getVariants()) {
                const Vertex v = getVertex(var, G, M, block);
                add_edge(var, u, v, G);
                resolveUsages(v, var, block, G, M, stmt);
            }
        } else {
            resolveUsages(u, stmt, block, G, M);
        }
    }
    std::vector<Vertex> mapping(num_vertices(G));
    summarizeGraph(G, mapping);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resolveUsages
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::resolveUsages(const Vertex u, PabloAST * expr, PabloBlock & block, Graph & G, Map & M, Statement * ignoreIfThis) const {
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
                        Statement * branch = f->second;
                        if (LLVM_UNLIKELY(branch == ignoreIfThis)) {
                            break;
                        }
                        // Add in a Var denoting the user of this expression so that it can be updated if expr changes.
                        const Vertex v = getVertex(user, G, M, block);
                        add_edge(expr, u, v, G);
                        const Vertex w = getVertex(branch, G, M, block);
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
        if (LLVM_LIKELY(out_degree(u, G) > 0)) {
            if (isOptimizable(G[u])) {
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
                    clear_vertex(u, G);
                    G[u].first = TypeId::Var;
                    mapping[u] = v;
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
                        clear_vertex(u, G);
                        G[u].first = TypeId::Var;
                        mapping[u] = v;
                    }
                }
            }
        } else if (isNonEscaping(G[u])) {
            clear_in_edges(u, G);
            G[u].first = TypeId::Var;
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
            VertexSet Aj = outgoingVertexSet(u, G);
            VertexSet Ak;
            std::set_intersection(Ai.begin(), Ai.end(), Aj.begin(), Aj.end(), std::back_inserter(Ak));
            Ai.swap(Ak);
        }
        cliques.emplace_back(std::move(Ai), std::move(*Bi));
    }
    return std::move(cliques);
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
 * An intermediary vertex could have more than one outgoing edge but if any edge is not directed to a vertex in our
 * biclique, we'll need to compute that specific value anyway. Remove them from the clique set and if there are not
 * enough vertices in the biclique to make distribution profitable, eliminate the clique.
 ** ------------------------------------------------------------------------------------------------------------- */
static BicliqueSet && removeUnhelpfulBicliques(BicliqueSet && cliques, const Graph & G, DistributionGraph & H) {
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
static DistributionSets safeDistributionSets(const Graph & G, DistributionGraph & H) {
    const auto F = makeGraphFacade(H);
    DistributionSets T;
    BicliqueSet lowerSet = independentCliqueSets<1>(removeUnhelpfulBicliques(enumerateBicliques(F, sinks(H)), G, H), 1);
    for (Biclique & lower : lowerSet) {
        BicliqueSet upperSet = independentCliqueSets<0>(enumerateBicliques(F, std::get<1>(lower)), 2);
        for (Biclique & upper : upperSet) {
            T.emplace_back(std::move(std::get<1>(upper)), std::move(std::get<0>(upper)), std::get<0>(lower));
        }
    }
    return std::move(T);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateDistributionGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void generateDistributionGraph(const Graph & G, DistributionGraph & H) {
    DistributionMap M;
    for (const Vertex u : make_iterator_range(vertices(G))) {
        const TypeId outerTypeId = G[u].first;
        if (outerTypeId == TypeId::And || outerTypeId == TypeId::Or) {
            const TypeId innerTypeId = (outerTypeId == TypeId::And) ? TypeId::Or : TypeId::And;
            flat_set<Vertex> distributable;
            for (auto e : make_iterator_range(in_edges(u, G))) {
                const Vertex v = source(e, G);
                if (LLVM_UNLIKELY(G[v].first == innerTypeId)) {
                    bool safe = true;
                    for (const auto e : make_iterator_range(out_edges(v, G))) {
                        if (G[target(e, G)].first != outerTypeId) {
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



/** ------------------------------------------------------------------------------------------------------------- *
 * @brief redistributeAST
 *
 * Apply the distribution law to reduce computations whenever possible.
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::redistributeAST(const PabloBlock & block, Graph & G) const {

//    printGraph(block, G, "G0");

//    unsigned count = 0;

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

//        raw_os_ostream out(std::cerr);
//        out << "DISTRIBUTION SETS: " << distributionSets.size() << '\n';

//        ++count;

//        unsigned subcount = 0;

        for (const DistributionSet & set : distributionSets) {

            // Each distribution tuple consists of the sources, intermediary, and sink nodes.
            const VertexSet & sources = std::get<0>(set);
            const VertexSet & intermediary = std::get<1>(set);
            const VertexSet & sinks = std::get<2>(set);

//            out << "SOURCES:";
//            for (const Vertex u : sources) {
//                const Vertex x = mapping[H[u]];
//                out << " " << x << ": "; PabloPrinter::print(G[x].second, out);
//            }
//            out << "\n";
//            out << "INTERMEDIARY:";
//            for (const Vertex u : intermediary) {
//                const Vertex x = mapping[H[u]];
//                out << " " << x << ": "; PabloPrinter::print(G[x].second, out);
//            }
//            out << "\n";
//            out << "SINKS:";
//            for (const Vertex u : sinks) {
//                const Vertex x = mapping[H[u]];
//                out << " " << x << ": "; PabloPrinter::print(G[x].second, out);
//            }
//            out << "\n";
//            out.flush();

            const TypeId outerTypeId = G[mapping[H[sinks.front()]]].first;
            assert (outerTypeId == TypeId::And || outerTypeId == TypeId::Or);
            const TypeId innerTypeId = (outerTypeId == TypeId::Or) ? TypeId::And : TypeId::Or;

            // Update G to match the desired changes (TODO: modify this to reuse a discarded vertex instead)
            const Vertex x = add_vertex(std::make_pair(outerTypeId, nullptr), G);
            const Vertex y = add_vertex(std::make_pair(innerTypeId, nullptr), G);
            mapping.resize(num_vertices(G));
            mapping[x] = x;
            mapping[y] = y;

            for (const Vertex i : intermediary) {
                const auto u = mapping[H[i]];
                assert (G[u].first == innerTypeId);
                for (const Vertex t : sinks) {
                    const auto v = mapping[H[t]];
                    assert (G[v].first == outerTypeId);
                    for (auto e : make_iterator_range(out_edges(u, G))) {
                        if (target(e, G) == v) {
                            assert (u != v);
                            remove_edge(e, G);
                        }
                    }
                }
                add_edge(nullptr, u, x, G);
            }            

            for (const Vertex s : sources) {
                const auto u = mapping[H[s]];
                for (const Vertex i : intermediary) {
                    const auto v = mapping[H[i]];
                    for (auto e : make_iterator_range(out_edges(u, G))) {
                        if (target(e, G) == v) {
                            assert (u != v);
                            remove_edge(e, G);
                        }
                    }
                }
                add_edge(nullptr, u, y, G);
            }

            add_edge(nullptr, x, y, G);

            for (const Vertex t : sinks) {
                const auto v = mapping[H[t]];
                add_edge(G[v].second, y, v, G);
            }

            summarizeGraph(G, mapping);

//            printGraph(block, G, "G" + std::to_string(count) + "_" + std::to_string(++subcount));
        }
    }
}

}
