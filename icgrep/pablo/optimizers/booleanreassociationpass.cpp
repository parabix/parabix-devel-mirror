#include "booleanreassociationpass.h"
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/strong_components.hpp>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/analysis/pabloverifier.hpp>
#include <algorithm>
#include <numeric> // std::iota
#include <pablo/printer_pablos.h>
#include <iostream>
#include <llvm/Support/CommandLine.h>
#include "maxsat.hpp"

using namespace boost;
using namespace boost::container;


static cl::OptionCategory ReassociationOptions("Reassociation Optimization Options", "These options control the Pablo Reassociation optimization pass.");

static cl::opt<unsigned> LoadEarly("Reassociation-Load-Early", cl::init(false),
                                  cl::desc("When recomputing an Associative operation, load values from preceeding blocks at the beginning of the "
                                           "Scope Block rather than at the point of first use."),
                                  cl::cat(ReassociationOptions));

namespace pablo {

using TypeId = PabloAST::ClassTypeId;
using Graph = BooleanReassociationPass::Graph;
using Vertex = Graph::vertex_descriptor;
using VertexData = BooleanReassociationPass::VertexData;
using DistributionGraph = BooleanReassociationPass::DistributionGraph;
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
template<typename Iterator>
inline Graph::edge_descriptor first(const std::pair<Iterator, Iterator> & range) {
    assert (range.first != range.second);
    return *range.first;
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

inline Z3_ast & getDefinition(VertexData & data) {
    return std::get<2>(data);
}

inline PabloAST * getValue(const VertexData & data) {
    return std::get<1>(data);
}

inline PabloAST *& getValue(VertexData & data) {
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

void add_edge(PabloAST * expr, const Vertex u, const Vertex v, Graph & G) {
    // Make sure each edge is unique
    assert (u < num_vertices(G) && v < num_vertices(G));
    assert (u != v);

    // Just because we've supplied an expr doesn't mean it's useful. Check it.
    if (expr) {
        if (isAssociative(G[v])) {
            expr = nullptr;
        } else {
            bool clear = true;
            if (const Statement * dest = dyn_cast_or_null<Statement>(getValue(G[v]))) {
                for (unsigned i = 0; i < dest->getNumOperands(); ++i) {
                    if (dest->getOperand(i) == expr) {
                        clear = false;
                        break;
                    }
                }
            }
            if (LLVM_LIKELY(clear)) {
                expr = nullptr;
            }
        }
    }

    for (auto e : make_iterator_range(out_edges(u, G))) {
        if (LLVM_UNLIKELY(target(e, G) == v)) {
            if (expr) {
                if (G[e] == nullptr) {
                    G[e] = expr;
                } else if (G[e] != expr) {
                    continue;
                }
            }
            return;
        }
    }
    G[boost::add_edge(u, v, G).first] = expr;
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
 * @brief printGraph
 ** ------------------------------------------------------------------------------------------------------------- */
static void printGraph(const Graph & G, const std::string name) {
    raw_os_ostream out(std::cerr);

    std::vector<unsigned> c(num_vertices(G));
    strong_components(G, make_iterator_property_map(c.begin(), get(vertex_index, G), c[0]));

    out << "digraph " << name << " {\n";
    for (auto u : make_iterator_range(vertices(G))) {
        if (in_degree(u, G) == 0 && out_degree(u, G) == 0) {
            continue;
        }
        out << "v" << u << " [label=\"" << u << ": ";
        TypeId typeId;
        PabloAST * expr;
        Z3_ast node;
        std::tie(typeId, expr, node) = G[u];
        bool temporary = false;
        bool error = false;
        if (expr == nullptr || (typeId != expr->getClassTypeId() && typeId != TypeId::Var)) {
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
                case TypeId::Not:
                    out << "Not";
                    break;
                default:
                    out << "???";
                    error = true;
                    break;
            }
            if (expr) {
                out << " ("; PabloPrinter::print(expr, out); out << ")";
            }
        } else {
            PabloPrinter::print(expr, out);
        }
        if (node == nullptr) {
            out << " (*)";
            error = true;
        }
        out << "\"";
        if (typeId == TypeId::Var) {
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
        const auto s = source(e, G);
        const auto t = target(e, G);
        out << "v" << s << " -> v" << t;
        bool cyclic = (c[s] == c[t]);
        if (G[e] || cyclic) {
            out << " [";
             if (G[e]) {
                out << "label=\"";
                PabloPrinter::print(G[e], out);
                out << "\" ";
             }
             if (cyclic) {
                out << "color=red ";
             }
             out << "]";
        }
        out << ";\n";
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
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool BooleanReassociationPass::optimize(PabloFunction & function) {

    Z3_config cfg = Z3_mk_config();
    Z3_context ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);
    Z3_solver solver = Z3_mk_solver(ctx);
    Z3_solver_inc_ref(ctx, solver);

    BooleanReassociationPass brp(ctx, solver, function);
    brp.processScopes(function);

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);

    Simplifier::optimize(function);

    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScopes
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool BooleanReassociationPass::processScopes(PabloFunction & function) {
    PabloBlock * const entry = function.getEntryBlock();
    CharacterizationMap map;
    // Map the constants and input variables
    map.add(entry->createZeroes(), Z3_mk_false(mContext));
    map.add(entry->createOnes(), Z3_mk_true(mContext));
    for (unsigned i = 0; i < mFunction.getNumOfParameters(); ++i) {
        map.add(mFunction.getParameter(i), makeVar());
    }
    mInFile = makeVar();
    processScopes(entry, map);
    return mModified;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScopes
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::processScopes(PabloBlock * const block, CharacterizationMap & map) {
    Z3_solver_push(mContext, mSolver);
    for (Statement * stmt = block->front(); stmt; ) {
        if (LLVM_UNLIKELY(isa<If>(stmt))) {
            if (LLVM_UNLIKELY(isa<Zeroes>(cast<If>(stmt)->getCondition()))) {
                stmt = stmt->eraseFromParent(true);
            } else {
                CharacterizationMap nested(map);
                processScopes(cast<If>(stmt)->getBody(), nested);
                for (Assign * def : cast<If>(stmt)->getDefined()) {
                    map.add(def, makeVar());
                }
                stmt = stmt->getNextNode();
            }
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            if (LLVM_UNLIKELY(isa<Zeroes>(cast<While>(stmt)->getCondition()))) {
                stmt = stmt->eraseFromParent(true);
            } else {
                CharacterizationMap nested(map);
                processScopes(cast<While>(stmt)->getBody(), nested);
                for (Next * var : cast<While>(stmt)->getVariants()) {
                    map.add(var, makeVar());
                }
                stmt = stmt->getNextNode();
            }
        } else { // characterize this statement then check whether it is equivalent to any existing one.
            stmt = characterize(stmt, map);
        }
    }
    distributeScope(block, map);
    Z3_solver_pop(mContext, mSolver, 1);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
inline Statement * BooleanReassociationPass::characterize(Statement * const stmt, CharacterizationMap & map) {
    Z3_ast node = nullptr;
    const size_t n = stmt->getNumOperands(); assert (n > 0);
    if (isa<Variadic>(stmt)) {
        Z3_ast operands[n];
        for (size_t i = 0; i < n; ++i) {
            operands[i] = map.get(stmt->getOperand(i)); assert (operands[i]);
        }
        if (isa<And>(stmt)) {
            node = Z3_mk_and(mContext, n, operands);
        } else if (isa<Or>(stmt)) {
            node = Z3_mk_or(mContext, n, operands);
        } else if (isa<Xor>(stmt)) {
            node = Z3_mk_xor(mContext, operands[0], operands[1]);
            for (unsigned i = 2; LLVM_UNLIKELY(i < n); ++i) {
                node = Z3_mk_xor(mContext, node, operands[i]);
            }
        }
    } else if (isa<Not>(stmt)) {
        Z3_ast op = map.get(stmt->getOperand(0)); assert (op);
        node = Z3_mk_not(mContext, op);
    } else if (isa<Sel>(stmt)) {
        Z3_ast operands[3];
        for (size_t i = 0; i < 3; ++i) {
            operands[i] = map.get(stmt->getOperand(i)); assert (operands[i]);
        }
        node = Z3_mk_ite(mContext, operands[0], operands[1], operands[2]);
    } else if (LLVM_UNLIKELY(isa<InFile>(stmt) || isa<AtEOF>(stmt))) {
        assert (stmt->getNumOperands() == 1);
        Z3_ast check[2];
        check[0] = map.get(stmt->getOperand(0)); assert (check[0]);
        check[1] = isa<InFile>(stmt) ? mInFile : Z3_mk_not(mContext, mInFile); assert (check[1]);
        node = Z3_mk_and(mContext, 2, check);
    } else {
        if (LLVM_UNLIKELY(isa<Assign>(stmt) || isa<Next>(stmt))) {
            Z3_ast op = map.get(stmt->getOperand(0)); assert (op);
            map.add(stmt, op, true);
        } else {
            map.add(stmt, makeVar());
        }
        return stmt->getNextNode();
    }
    node = Z3_simplify(mContext, node); assert (node);
    PabloAST * const replacement = map.findKey(node);
    if (LLVM_LIKELY(replacement == nullptr)) {
        map.add(stmt, node);
        return stmt->getNextNode();
    } else {
        return stmt->replaceWith(replacement);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScope
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BooleanReassociationPass::distributeScope(PabloBlock * const block, CharacterizationMap & map) {
    Graph G;
    try {
        transformAST(block, map, G);
    } catch (std::runtime_error err) {
        printGraph(G, "G");
        throw err;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief summarizeAST
 *
 * This function scans through a scope block and computes a DAG G in which any sequences of AND, OR or XOR functions
 * are "flattened" (i.e., allowed to have any number of inputs.)
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::transformAST(PabloBlock * const block, CharacterizationMap & C, Graph & G) {
    StatementMap S;
    // Compute the base def-use graph ...
    for (Statement * stmt : *block) {

        const Vertex u = makeVertex(stmt->getClassTypeId(), stmt, S, G, C.get(stmt));

        for (unsigned i = 0; i < stmt->getNumOperands(); ++i) {
            PabloAST * const op = stmt->getOperand(i);
            if (LLVM_LIKELY(isa<Statement>(op) || isa<Var>(op))) {
                add_edge(op, makeVertex(TypeId::Var, op, C, S, G), u, G);
            }
        }

        if (LLVM_UNLIKELY(isa<If>(stmt))) {
            for (Assign * def : cast<const If>(stmt)->getDefined()) {
                const Vertex v = makeVertex(TypeId::Var, def, C, S, G);
                add_edge(def, u, v, G);
                resolveNestedUsages(block, def, v, C, S, G, stmt);
            }
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            // To keep G a DAG, we need to do a bit of surgery on loop variants because
            // the next variables it produces can be used within the condition. Instead,
            // we make the loop dependent on the original value of each Next node and
            // the Next node dependent on the loop.
            for (Next * var : cast<const While>(stmt)->getVariants()) {
                const Vertex v = makeVertex(TypeId::Var, var, C, S, G);
                assert (in_degree(v, G) == 1);
                auto e = first(in_edges(v, G));
                add_edge(G[e], source(e, G), u, G);
                remove_edge(v, u, G);
                add_edge(var, u, v, G);
                resolveNestedUsages(block, var, v, C, S, G, stmt);
            }
        } else {            
            resolveNestedUsages(block, stmt, u, C, S, G, stmt);
        }
    }

    if (redistributeGraph(C, S, G)) {
        factorGraph(G);
        rewriteAST(block, G);
        mModified = true;
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resolveNestedUsages
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::resolveNestedUsages(PabloBlock * const block, PabloAST * const expr, const Vertex u,
                                                   CharacterizationMap & C, StatementMap & S, Graph & G,
                                                   const Statement * const ignoreIfThis) const {
    assert ("Cannot resolve nested usages of a null expression!" && expr);
    for (PabloAST * user : expr->users()) { assert (user);
        if (LLVM_LIKELY(user != expr && isa<Statement>(user))) {
            PabloBlock * parent = cast<Statement>(user)->getParent(); assert (parent);
            if (LLVM_UNLIKELY(parent != block)) {
                for (;;) {
                    if (parent->getParent() == block) {
                        Statement * const branch = parent->getBranch();
                        if (LLVM_UNLIKELY(branch != ignoreIfThis)) {
                            // Add in a Var denoting the user of this expression so that it can be updated if expr changes.
                            const Vertex v = makeVertex(TypeId::Var, user, C, S, G);
                            add_edge(expr, u, v, G);
                            const Vertex w = makeVertex(branch->getClassTypeId(), branch, S, G);
                            add_edge(user, v, w, G);
                        }
                        break;
                    }
                    parent = parent->getParent();
                    if (LLVM_UNLIKELY(parent == nullptr)) {
                        assert (isa<Assign>(expr) || isa<Next>(expr));
                        break;
                    }
                }
            }
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
            VertexSet adjacencies;
            adjacencies.reserve(in_degree(u, G));
            for (auto e : make_iterator_range(in_edges(u, G))) {
                adjacencies.push_back(source(e, G));
            }
            std::sort(adjacencies.begin(), adjacencies.end());
            assert(std::unique(adjacencies.begin(), adjacencies.end()) == adjacencies.end());
            B1.insert(std::move(adjacencies));
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
            assert(std::unique(Aj.begin(), Aj.end()) == Aj.end());
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
 * @brief generateDistributionGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void generateDistributionGraph(const Graph & G, DistributionGraph & H) {
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
                flat_set<Vertex> observed;
                for (const Vertex v : distributable) {
                    for (auto e : make_iterator_range(in_edges(v, G))) {
                        const auto v = source(e, G);
                        observed.insert(v);
                    }
                }
                for (const Vertex w : observed) {
                    for (auto e : make_iterator_range(out_edges(w, G))) {
                        const Vertex v = target(e, G);
                        if (distributable.count(v)) {
                            const Vertex y = getVertex(v, H, M);
                            boost::add_edge(y, getVertex(u, H, M), H);
                            boost::add_edge(getVertex(w, H, M), y, H);
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
bool BooleanReassociationPass::redistributeGraph(CharacterizationMap & C, StatementMap & M, Graph & G) const {

    bool modified = false;

    DistributionGraph H;

    contractGraph(M, G);

    for (;;) {

        for (;;) {

            generateDistributionGraph(G, H);

            // If we found no potential opportunities then we cannot apply the distribution law to any part of G.
            if (num_vertices(H) == 0) {
                break;
            }

            const DistributionSets distributionSets = safeDistributionSets(G, H);

            if (LLVM_UNLIKELY(distributionSets.empty())) {
                break;
            }

            modified = true;

            for (const DistributionSet & set : distributionSets) {

                // Each distribution tuple consists of the sources, intermediary, and sink nodes.
                const VertexSet & sources = std::get<0>(set);
                const VertexSet & intermediary = std::get<1>(set);
                const VertexSet & sinks = std::get<2>(set);

                const TypeId outerTypeId = getType(G[H[sinks.front()]]);
                assert (outerTypeId == TypeId::And || outerTypeId == TypeId::Or);
                const TypeId innerTypeId = (outerTypeId == TypeId::Or) ? TypeId::And : TypeId::Or;

                const Vertex x = makeVertex(outerTypeId, nullptr, G);
                const Vertex y = makeVertex(innerTypeId, nullptr, G);

                // Update G to reflect the distributed operations (including removing the subgraph of
                // the to-be distributed edges.)

                add_edge(nullptr, x, y, G);

                for (const Vertex i : sources) {
                    const auto u = H[i];
                    for (const Vertex j : intermediary) {
                        const auto v = H[j];
                        const auto e = edge(u, v, G); assert (e.second);
                        remove_edge(e.first, G);
                    }
                    add_edge(nullptr, u, y, G);
                }

                for (const Vertex i : intermediary) {
                    const auto u = H[i];
                    for (const Vertex j : sinks) {
                        const auto v = H[j];
                        const auto e = edge(u, v, G); assert (e.second);
                        add_edge(G[e.first], y, v, G);
                        remove_edge(e.first, G);
                    }
                    add_edge(nullptr, u, x, G);
                    getDefinition(G[u]) = nullptr;
                }

            }

            H.clear();

            contractGraph(M, G);
        }

        // Although exceptionally unlikely, it's possible that if we can reduce the graph, we could
        // further simplify it. Restart the process if and only if we succeed.
        if (LLVM_UNLIKELY(reduceGraph(C, M, G))) {
            if (LLVM_UNLIKELY(contractGraph(M, G))) {
                H.clear();
                continue;
            }
        }

        break;
    }

    return modified;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isNonEscaping
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool isNonEscaping(const VertexData & data) {
    // If these are redundant, the Simplifier pass will eliminate them. Trust that they're necessary.
    switch (getType(data)) {
        case TypeId::Assign:
        case TypeId::Next:
        case TypeId::If:
        case TypeId::While:
        case TypeId::Count:
            return false;
        default:
            return true;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief contractGraph
 ** ------------------------------------------------------------------------------------------------------------- */
bool BooleanReassociationPass::contractGraph(StatementMap & M, Graph & G) const {

    bool contracted = false;

    circular_buffer<Vertex> ordering(num_vertices(G));

    topological_sort(G, std::back_inserter(ordering)); // reverse topological ordering

    // first contract the graph
    for (const Vertex u : ordering) {
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
                    }
                    removeVertex(u, M, G);
                    contracted = true;
                } else if (LLVM_UNLIKELY(out_degree(u, G) == 1)) {
                    // Otherwise if we have a single user, we have a similar case as above but
                    // we can only merge this vertex into the outgoing instruction if they are
                    // of the same type.
                    const auto ei = first(out_edges(u, G));
                    const Vertex v = target(ei, G);
                    if (LLVM_UNLIKELY(getType(G[v]) == getType(G[u]))) {
                        for (auto ej : make_iterator_range(in_edges(u, G))) {
                            add_edge(G[ei], source(ej, G), v, G);
                        }
                        removeVertex(u, M, G);
                        contracted = true;
                    }                    
                }
            }
        } else if (LLVM_UNLIKELY(isNonEscaping(G[u]))) {
            removeVertex(u, M, G);
            contracted = true;
        }
    }
    return contracted;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isReducible
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool isReducible(const VertexData & data) {
    switch (getType(data)) {
        case TypeId::Var:
        case TypeId::If:
        case TypeId::While:
            return false;
        default:
            return true;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief reduceGraph
 ** ------------------------------------------------------------------------------------------------------------- */
bool BooleanReassociationPass::reduceGraph(CharacterizationMap & C, StatementMap & S, Graph & G) const {

    bool reduced = false;

    VertexMap M;
    circular_buffer<Vertex> ordering(num_vertices(G));

    topological_sort(G, std::front_inserter(ordering)); // topological ordering

    // first contract the graph
    for (const Vertex u : ordering) {
        if (isReducible(G[u])) {
            Z3_ast & node = getDefinition(G[u]);
            if (isAssociative(G[u])) {
                const TypeId typeId = getType(G[u]);
                if (node == nullptr) {
                    const auto n = in_degree(u, G); assert (n > 1);
                    Z3_ast operands[n];
                    unsigned i = 0;
                    for (auto e : make_iterator_range(in_edges(u, G))) {
                        const Vertex v = source(e, G);
                        assert (getDefinition(G[v]));
                        operands[i++] = getDefinition(G[v]);
                    }
                    switch (typeId) {
                        case TypeId::And:
                            node = Z3_mk_and(mContext, n, operands);
                            break;
                        case TypeId::Or:
                            node = Z3_mk_or(mContext, n, operands);
                            break;
                        case TypeId::Xor:
                            node = Z3_mk_xor(mContext, operands[0], operands[1]);
                            for (unsigned i = 2; LLVM_UNLIKELY(i < n); ++i) {
                                node = Z3_mk_xor(mContext, node, operands[i]);
                            }
                            break;
                        default: llvm_unreachable("unexpected type id");
                    }

                    assert (node);
                    node = Z3_simplify(mContext, node);
                }
                graph_traits<Graph>::in_edge_iterator begin, end;
restart:        if (in_degree(u, G) > 1) {
                    std::tie(begin, end) = in_edges(u, G);
                    for (auto i = begin; ++i != end; ) {
                        const auto v = source(*i, G);
                        for (auto j = begin; j != i; ++j) {
                            const auto w = source(*j, G);
                            Z3_ast operands[2] = { getDefinition(G[v]), getDefinition(G[w]) };
                            assert (operands[0]);
                            assert (operands[1]);
                            Z3_ast test = nullptr;
                            switch (typeId) {
                                case TypeId::And:
                                    test = Z3_mk_and(mContext, 2, operands); break;
                                case TypeId::Or:
                                    test = Z3_mk_or(mContext, 2, operands); break;
                                case TypeId::Xor:
                                    test = Z3_mk_xor(mContext, operands[0], operands[1]); break;
                                default:
                                    llvm_unreachable("impossible type id");
                            }
                            assert (test);
                            test = Z3_simplify(mContext, test);
                            PabloAST * const factor = C.findKey(test);
                            if (LLVM_UNLIKELY(factor != nullptr)) {
                                const Vertex a = makeVertex(TypeId::Var, factor, S, G, test);
                                // note: unless both edges carry an Pablo AST replacement value, they will converge into a single edge.
                                PabloAST * const r1 = G[*i];
                                PabloAST * const r2 = G[*j];

                                remove_edge(*i, G);
                                remove_edge(*j, G);

                                if (LLVM_UNLIKELY(r1 && r2)) {
                                    add_edge(r1, a, u, G);
                                    add_edge(r2, a, u, G);
                                } else {
                                    add_edge(r1 ? r1 : r2, a, u, G);
                                }

//                                errs() << " -- subsituting (" << a << ',' << u << ")=" << Z3_ast_to_string(mContext, test)
//                                       << " for (" << v << ',' << u << ")=" << Z3_ast_to_string(mContext, getDefinition(G[v]));

//                                switch (typeId) {
//                                    case TypeId::And:
//                                        errs() << " ∧ "; break;
//                                    case TypeId::Or:
//                                        errs() << " ∨ ";  break;
//                                    case TypeId::Xor:
//                                        errs() << " ⊕ ";  break;
//                                    default:
//                                        llvm_unreachable("impossible type id");
//                                }

//                                errs() << "(" << w << ',' << u << ")=" << Z3_ast_to_string(mContext, getDefinition(G[w]))
//                                       << "\n";

                                reduced = true;
                                goto restart;
                            }
                        }
                    }
                }
            }

            if (LLVM_UNLIKELY(node == nullptr)) {
                throw std::runtime_error("No Z3 characterization for vertex " + std::to_string(u));
            }

            auto f = M.find(node);
            if (LLVM_LIKELY(f == M.end())) {
                M.emplace(node, u);
            } else if (isAssociative(G[u])) {
                const Vertex v = f->second;

//                errs() << " -- subsituting " << u << ":=\n" << Z3_ast_to_string(mContext, getDefinition(G[u]))
//                       << "\n for " << v << ":=\n" << Z3_ast_to_string(mContext, getDefinition(G[v])) << "\n";

                for (auto e : make_iterator_range(out_edges(u, G))) {
                    add_edge(G[e], v, target(e, G), G);
                }
                removeVertex(u, S, G);
                reduced = true;
            }
        }
    }
    return reduced;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief factorGraph
 ** ------------------------------------------------------------------------------------------------------------- */
bool BooleanReassociationPass::factorGraph(const TypeId typeId, Graph & G, std::vector<Vertex> & factors) const {

    if (LLVM_UNLIKELY(factors.empty())) {
        return false;
    }

    std::vector<Vertex> I, J, K;

    bool modified = false;

    for (auto i = factors.begin(); ++i != factors.end(); ) {
        assert (getType(G[*i]) == typeId);
        for (auto ei : make_iterator_range(in_edges(*i, G))) {
            I.push_back(source(ei, G));
        }
        std::sort(I.begin(), I.end());
        for (auto j = factors.begin(); j != i; ++j) {
            for (auto ej : make_iterator_range(in_edges(*j, G))) {
                J.push_back(source(ej, G));
            }
            std::sort(J.begin(), J.end());
            // get the pairwise intersection of each set of inputs (i.e., their common subexpression)
            std::set_intersection(I.begin(), I.end(), J.begin(), J.end(), std::back_inserter(K));
            assert (std::is_sorted(K.begin(), K.end()));
            // if the intersection contains at least two elements
            const auto n = K.size();
            if (n > 1) {
                Vertex a = *i;
                Vertex b = *j;
                if (LLVM_UNLIKELY(in_degree(a, G) == n || in_degree(b, G) == n)) {
                    if (in_degree(a, G) != n) {
                        assert (in_degree(b, G) == n);
                        std::swap(a, b);
                    }
                    assert (in_degree(a, G) == n);
                    if (in_degree(b, G) == n) {
                        for (auto e : make_iterator_range(out_edges(b, G))) {
                            add_edge(G[e], a, target(e, G), G);
                        }
                        removeVertex(b, G);
                    } else {
                        for (auto u : K) {
                            remove_edge(u, b, G);
                        }
                        add_edge(nullptr, a, b, G);
                    }
                } else {
                    Vertex v = makeVertex(typeId, nullptr, G);
                    for (auto u : K) {
                        remove_edge(u, a, G);
                        remove_edge(u, b, G);
                        add_edge(nullptr, u, v, G);
                    }
                    add_edge(nullptr, v, a, G);
                    add_edge(nullptr, v, b, G);
                    factors.push_back(v);
                }
                modified = true;
            }
            K.clear();
            J.clear();
        }
        I.clear();
    }
    return modified;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief factorGraph
 ** ------------------------------------------------------------------------------------------------------------- */
bool BooleanReassociationPass::factorGraph(Graph & G) const {
    // factor the associative vertices.
    std::vector<Vertex> factors;
    bool factored = false;
    for (unsigned i = 0; i < 3; ++i) {
        TypeId typeId[3] = { TypeId::And, TypeId::Or, TypeId::Xor};
        for (auto j : make_iterator_range(vertices(G))) {
            if (getType(G[j]) == typeId[i]) {
                factors.push_back(j);
            }
        }
        if (factorGraph(typeId[i], G, factors)) {
            factored = true;
        }
        factors.clear();
    }
    return factored;
}


inline bool isMutable(const Vertex u, const Graph & G) {
    return getType(G[u]) != TypeId::Var;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief rewriteAST
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::rewriteAST(PabloBlock * const block, Graph & G) {

    using line_t = long long int;

    enum : line_t { MAX_INT = std::numeric_limits<line_t>::max() };

    Z3_config cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "MODEL", "true");
    Z3_context ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);
    Z3_solver solver = Z3_mk_solver(ctx);
    Z3_solver_inc_ref(ctx, solver);

    std::vector<Z3_ast> mapping(num_vertices(G), nullptr);

    flat_map<PabloAST *, Z3_ast> M;

    // Generate the variables
    const auto ty = Z3_mk_int_sort(ctx);
    Z3_ast ZERO = Z3_mk_int(ctx, 0, ty);

    for (const Vertex u : make_iterator_range(vertices(G))) {
        const auto var = Z3_mk_fresh_const(ctx, nullptr, ty);
        Z3_ast constraint = nullptr;
        if (in_degree(u, G) == 0) {
            constraint = Z3_mk_eq(ctx, var, ZERO);
        } else {
            constraint = Z3_mk_gt(ctx, var, ZERO);
        }
        Z3_solver_assert(ctx, solver, constraint);

        PabloAST * const expr = getValue(G[u]);
        if (expr) {
            const bool added = M.emplace(expr, var).second;
            assert ("G contains duplicate vertices for the same statement!" && added);
        }
        mapping[u] = var;
    }

    // Add in the dependency constraints
    for (const Vertex u : make_iterator_range(vertices(G))) {
        Z3_ast const t = mapping[u];
        for (auto e : make_iterator_range(in_edges(u, G))) {
            Z3_ast const s = mapping[source(e, G)];
            Z3_solver_assert(ctx, solver, Z3_mk_lt(ctx, s, t));
        }
    }

    // Compute the soft ordering constraints
    std::vector<Z3_ast> ordering(0);
    ordering.reserve(2 * M.size() - 1);

    Z3_ast prior = nullptr;
    unsigned gap = 1;
    for (Statement * stmt : *block) {
        auto f = M.find(stmt);
        if (f != M.end()) {
            Z3_ast const node = f->second;
            if (prior) {
                ordering.push_back(Z3_mk_lt(ctx, prior, node));
                Z3_ast ops[2] = { node, prior };
                ordering.push_back(Z3_mk_le(ctx, Z3_mk_sub(ctx, 2, ops), Z3_mk_int(ctx, gap, ty)));
            } else {
                ordering.push_back(Z3_mk_eq(ctx, node, Z3_mk_int(ctx, gap, ty)));

            }
            prior = node;
            gap = 0;
        }
        ++gap;
    }

    if (LLVM_UNLIKELY(maxsat(ctx, solver, ordering) < 0)) {
        throw std::runtime_error("Unable to construct a topological ordering during reassociation!");
    }

    Z3_model model = Z3_solver_get_model(ctx, solver);
    Z3_model_inc_ref(ctx, model);

    std::vector<Vertex> S(0);
    S.reserve(num_vertices(G));

    std::vector<line_t> L(num_vertices(G));



    for (const Vertex u : make_iterator_range(vertices(G))) {
        line_t line = LoadEarly ? 0 : MAX_INT;
        if (isMutable(u, G)) {
            Z3_ast value;
            if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, mapping[u], Z3_L_TRUE, &value) != Z3_L_TRUE)) {
                throw std::runtime_error("Unexpected Z3 error when attempting to obtain value from model!");
            }
            if (LLVM_UNLIKELY(Z3_get_numeral_int64(ctx, value, &line) != Z3_L_TRUE)) {
                throw std::runtime_error("Unexpected Z3 error when attempting to convert model value to integer!");
            }
            S.push_back(u);
        }
        L[u] = line;
    }

    Z3_model_dec_ref(ctx, model);

    std::sort(S.begin(), S.end(), [&L](const Vertex u, const Vertex v){ return L[u] < L[v]; });

    block->setInsertPoint(nullptr);

    std::vector<Vertex> T;

    line_t count = 1;

    for (auto u : S) {
        PabloAST *& stmt = getValue(G[u]);

        assert (isMutable(u, G));
        assert (L[u] > 0 && L[u] < MAX_INT);

        if (isAssociative(G[u])) {

            if (in_degree(u, G) == 0 || out_degree(u, G) == 0) {
                throw std::runtime_error("Vertex " + std::to_string(u) + " is either a source or sink node but marked as associative!");
            }

            Statement * ip = block->getInsertPoint();
            ip = ip ? ip->getNextNode() : block->front();

            const auto typeId = getType(G[u]);

            T.reserve(in_degree(u, G));
            for (const auto e : make_iterator_range(in_edges(u, G))) {
                T.push_back(source(e, G));
            }

            // Then sort them by their line position (noting any incoming value will either be 0 or MAX_INT)
            std::sort(T.begin(), T.end(), [&L](const Vertex u, const Vertex v){ return L[u] < L[v]; });
            if (LoadEarly) {
                block->setInsertPoint(nullptr);
            }

            circular_buffer<PabloAST *> Q(2); // in_degree(u, G));
            for (auto u : T) {
                PabloAST * expr = getValue(G[u]);
                if (LLVM_UNLIKELY(expr == nullptr)) {
                    throw std::runtime_error("Vertex " + std::to_string(u) + " does not have an expression!");
                }
                Q.push_back(expr);
                if (Q.size() > 1) {

                    PabloAST * e1 = Q.front(); Q.pop_front();
                    PabloAST * e2 = Q.front(); Q.pop_front();

                    if (in_degree(u, G) > 0) {
                        if (LLVM_UNLIKELY(!dominates(e1, e2))) {
                            std::string tmp;
                            raw_string_ostream out(tmp);
                            out << "e1: ";
                            PabloPrinter::print(e1, out);
                            out << " does not dominate e2: ";
                            PabloPrinter::print(e2, out);
                            throw std::runtime_error(out.str());
                        }
                        assert (dominates(e1, e2));
                        assert (dominates(e2, ip));
                        Statement * dom = cast<Statement>(expr);
                        for (;;) {
                            PabloBlock * const parent = dom->getParent();
                            if (parent == block) {
                                block->setInsertPoint(dom);
                                break;
                            }
                            dom = parent->getBranch(); assert(dom);
                        }
                        assert (dominates(dom, ip));
                    }

                    switch (typeId) {
                        case TypeId::And:
                            expr = block->createAnd(e1, e2); break;
                        case TypeId::Or:
                            expr = block->createOr(e1, e2); break;
                        case TypeId::Xor:
                            expr = block->createXor(e1, e2); break;
                        default: break;
                    }
                    Q.push_front(expr);
                }
            }

            assert (Q.size() == 1);

            T.clear();

            block->setInsertPoint(ip->getPrevNode());

            PabloAST * const replacement = Q.front(); assert (replacement);
            for (auto e : make_iterator_range(out_edges(u, G))) {
                if (G[e]) {
                    if (PabloAST * user = getValue(G[target(e, G)])) {
                        cast<Statement>(user)->replaceUsesOfWith(G[e], replacement);
                    }
                }
            }

            stmt = replacement;
        }

        assert (stmt);
        assert (inCurrentBlock(stmt, block));

        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            for (auto e : make_iterator_range(out_edges(u, G))) {
                const auto v = target(e, G);
                assert (L[v] == std::numeric_limits<line_t>::max());
                L[v] = count;
            }
        }

        block->insert(cast<Statement>(stmt));
        L[u] = count++; // update the line count with the actual one.
    }

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);

    Statement * const end = block->getInsertPoint(); assert (end);
    for (;;) {
        Statement * const next = end->getNextNode();
        if (next == nullptr) {
            break;
        }
        next->eraseFromParent(true);
    }

    #ifndef NDEBUG
    PabloVerifier::verify(mFunction, "post-reassociation");
    #endif

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addSummaryVertex
 ** ------------------------------------------------------------------------------------------------------------- */
Vertex BooleanReassociationPass::makeVertex(const TypeId typeId, PabloAST * const expr, Graph & G, Z3_ast node) {
//    for (Vertex u : make_iterator_range(vertices(G))) {
//        if (LLVM_UNLIKELY(in_degree(u, G) == 0 && out_degree(u, G) == 0)) {
//            std::get<0>(G[u]) = typeId;
//            std::get<1>(G[u]) = expr;
//            return u;
//        }
//    }
    return add_vertex(std::make_tuple(typeId, expr, node), G);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addSummaryVertex
 ** ------------------------------------------------------------------------------------------------------------- */
Vertex BooleanReassociationPass::makeVertex(const TypeId typeId, PabloAST * const expr, StatementMap & M, Graph & G, Z3_ast node) {
    assert (expr);
    const auto f = M.find(expr);
    if (f != M.end()) {
        assert (getValue(G[f->second]) == expr);
        return f->second;
    }
    const Vertex u = makeVertex(typeId, expr, G, node);
    M.emplace(expr, u);
    return u;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addSummaryVertex
 ** ------------------------------------------------------------------------------------------------------------- */
Vertex BooleanReassociationPass::makeVertex(const TypeId typeId, PabloAST * const expr, CharacterizationMap & C, StatementMap & M, Graph & G) {
    assert (expr);
    const auto f = M.find(expr);
    if (f != M.end()) {
        assert (getValue(G[f->second]) == expr);
        return f->second;
    }
    const Vertex u = makeVertex(typeId, expr, G, C.get(expr));
    M.emplace(expr, u);
    return u;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief removeSummaryVertex
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BooleanReassociationPass::removeVertex(const Vertex u, StatementMap & M, Graph & G) const {
    VertexData & ref = G[u];
    if (std::get<1>(ref)) {
        auto f = M.find(std::get<1>(ref));
        assert (f != M.end());
        M.erase(f);
    }
    removeVertex(u, G);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief removeSummaryVertex
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BooleanReassociationPass::removeVertex(const Vertex u, Graph & G) const {
    VertexData & ref = G[u];
    clear_vertex(u, G);
    std::get<0>(ref) = TypeId::Var;
    std::get<1>(ref) = nullptr;
    std::get<2>(ref) = nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief make
 ** ------------------------------------------------------------------------------------------------------------- */
inline Z3_ast BooleanReassociationPass::makeVar() const {
    Z3_ast node = Z3_mk_fresh_const(mContext, nullptr, Z3_mk_bool_sort(mContext));
//    Z3_inc_ref(mContext, node);
    return node;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief add
 ** ------------------------------------------------------------------------------------------------------------- */
inline Z3_ast BooleanReassociationPass::CharacterizationMap::add(PabloAST * const expr, Z3_ast node, const bool forwardOnly) {
    assert (expr && node);
    mForward.emplace(expr, node);
    if (!forwardOnly) {
        mBackward.emplace(node, expr);
    }
    return node;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief get
 ** ------------------------------------------------------------------------------------------------------------- */
inline Z3_ast BooleanReassociationPass::CharacterizationMap::get(PabloAST * const expr) const {
    assert (expr);
    auto f = mForward.find(expr);
    if (LLVM_UNLIKELY(f == mForward.end())) {
        if (mPredecessor == nullptr) {
            return nullptr;
        }
        return mPredecessor->get(expr);
    }
    return f->second;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief get
 ** ------------------------------------------------------------------------------------------------------------- */
inline PabloAST * BooleanReassociationPass::CharacterizationMap::findKey(Z3_ast const node) const {
    assert (node);
    auto f = mBackward.find(node);
    if (LLVM_UNLIKELY(f == mBackward.end())) {
        if (mPredecessor == nullptr) {
            return nullptr;
        }
        return mPredecessor->findKey(node);
    }
    return f->second;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
inline BooleanReassociationPass::BooleanReassociationPass(Z3_context ctx, Z3_solver solver, PabloFunction & f)
: mContext(ctx)
, mSolver(solver)
, mFunction(f)
, mModified(false) {

}

}
