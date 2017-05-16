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

//  noif-dist-mult-dist-50 \p{Cham}(?<!\p{Mc})

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
inline bool intersects(Type & A, Type & B) {
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
static void printGraph(const Graph & G, const std::string & name) {
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
    Z3_context ctx = Z3_mk_context_rc(cfg);
    Z3_del_config(cfg);

    Z3_params params = Z3_mk_params(ctx);
    Z3_params_inc_ref(ctx, params);
    Z3_params_set_bool(ctx, params, Z3_mk_string_symbol(ctx, "pull_cheap_ite"), true);
    Z3_params_set_bool(ctx, params, Z3_mk_string_symbol(ctx, "local_ctx"), true);

    Z3_tactic ctx_solver_simplify = Z3_mk_tactic(ctx, "ctx-solver-simplify");
    Z3_tactic_inc_ref(ctx, ctx_solver_simplify);

    BooleanReassociationPass brp(ctx, params, ctx_solver_simplify, function);
    brp.processScopes(function);

    Z3_params_dec_ref(ctx, params);
    Z3_tactic_dec_ref(ctx, ctx_solver_simplify);
    Z3_del_context(ctx);

    PabloVerifier::verify(function, "post-reassociation");

    Simplifier::optimize(function);

    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScopes
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool BooleanReassociationPass::processScopes(PabloFunction & function) {
    CharacterizationMap C;
    PabloBlock * const entry = function.getEntryBlock();
    // Map the constants and input variables
    C.add(entry->createZeroes(), Z3_mk_false(mContext));
    C.add(entry->createOnes(), Z3_mk_true(mContext));
    for (unsigned i = 0; i < mFunction.getNumOfParameters(); ++i) {
        C.add(mFunction.getParameter(i), makeVar());
    }
    mInFile = makeVar();
    processScopes(entry, C);
    for (auto i = mRefs.begin(); i != mRefs.end(); ++i) {
        Z3_dec_ref(mContext, *i);
    }
    mRefs.clear();
    return mModified;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScopes
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::processScopes(PabloBlock * const block, CharacterizationMap & C) {
    const auto offset = mRefs.size();
    for (Statement * stmt = block->front(); stmt; ) {
        if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
            if (LLVM_UNLIKELY(isa<Zeroes>(cast<Branch>(stmt)->getCondition()))) {
                stmt = stmt->eraseFromParent(true);
            } else {
                CharacterizationMap nested(C);
                processScopes(cast<Branch>(stmt)->getBody(), nested);
                for (Var * def : cast<Branch>(stmt)->getEscaped()) {
                    C.add(def, makeVar());
                }
                stmt = stmt->getNextNode();
            }
        } else { // characterize this statement then check whether it is equivalent to any existing one.
            PabloAST * const folded = Simplifier::fold(stmt, block);
            if (LLVM_UNLIKELY(folded != nullptr)) {
                stmt = stmt->replaceWith(folded);
            } else {
                stmt = characterize(stmt, C);
            }
        }
    }    
    distributeScope(block, C);
    for (auto i = mRefs.begin() + offset; i != mRefs.end(); ++i) {
        Z3_dec_ref(mContext, *i);
    }
    mRefs.erase(mRefs.begin() + offset, mRefs.end());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
inline Statement * BooleanReassociationPass::characterize(Statement * const stmt, CharacterizationMap & C) {

    Z3_ast node = nullptr;
    const size_t n = stmt->getNumOperands(); assert (n > 0);
    bool use_expensive_simplification = false;
    if (isa<Variadic>(stmt)) {
        Z3_ast operands[n];
        for (size_t i = 0; i < n; ++i) {
            PabloAST * const op = stmt->getOperand(i);
            if (isa<Not>(op)) {
                use_expensive_simplification = true;
            }
            operands[i] = C.get(op); assert (operands[i]);
        }
        if (isa<And>(stmt)) {
            node = Z3_mk_and(mContext, n, operands);
        } else if (isa<Or>(stmt)) {
            node = Z3_mk_or(mContext, n, operands);
        } else if (isa<Xor>(stmt)) {
            node = Z3_mk_xor(mContext, operands[0], operands[1]);
            for (unsigned i = 2; LLVM_UNLIKELY(i < n); ++i) {
                Z3_inc_ref(mContext, node);
                Z3_ast temp = Z3_mk_xor(mContext, node, operands[i]);
                Z3_inc_ref(mContext, temp);
                Z3_dec_ref(mContext, node);
                node = temp;
            }
        }
    } else if (isa<Not>(stmt)) {
        Z3_ast op = C.get(stmt->getOperand(0)); assert (op);
        node = Z3_mk_not(mContext, op);
    } else if (isa<Sel>(stmt)) {
        Z3_ast operands[3];
        for (size_t i = 0; i < 3; ++i) {
            operands[i] = C.get(stmt->getOperand(i)); assert (operands[i]);
        }
        node = Z3_mk_ite(mContext, operands[0], operands[1], operands[2]);
    } else if (LLVM_UNLIKELY(isa<InFile>(stmt) || isa<AtEOF>(stmt))) {
        assert (stmt->getNumOperands() == 1);
        Z3_ast check[2];
        check[0] = C.get(stmt->getOperand(0)); assert (check[0]);
        check[1] = isa<InFile>(stmt) ? mInFile : Z3_mk_not(mContext, mInFile); assert (check[1]);
        node = Z3_mk_and(mContext, 2, check);
    } else if (LLVM_UNLIKELY(isa<Assign>(stmt))) {
        return stmt->getNextNode();
    }  else {
        C.add(stmt, makeVar());
        return stmt->getNextNode();
    }
    node = simplify(node, use_expensive_simplification);
    PabloAST * const replacement = C.findKey(node);
    if (LLVM_LIKELY(replacement == nullptr)) {
        C.add(stmt, node);
        mRefs.push_back(node);
        return stmt->getNextNode();
    } else {
        Z3_dec_ref(mContext, node);
        return stmt->replaceWith(replacement);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief processScope
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BooleanReassociationPass::distributeScope(PabloBlock * const block, CharacterizationMap & C) {
    Graph G;
    try {
        mBlock = block;
        transformAST(C, G);
    } catch (std::exception &) {
        printGraph(G, "E");
        throw;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief summarizeAST
 *
 * This function scans through a scope block and computes a DAG G in which any sequences of AND, OR or XOR functions
 * are "flattened" (i.e., allowed to have any number of inputs.)
 ** ------------------------------------------------------------------------------------------------------------- */
Vertex BooleanReassociationPass::transcribeSel(Sel * const stmt, CharacterizationMap & C, StatementMap & S, VertexMap & M, Graph & G) {

    Z3_ast args[2];

    const Vertex c = makeVertex(TypeId::Var, stmt->getCondition(), C, S, M, G);
    const Vertex t = makeVertex(TypeId::Var, cast<Sel>(stmt)->getTrueExpr(), C, S, M, G);
    const Vertex f = makeVertex(TypeId::Var, cast<Sel>(stmt)->getFalseExpr(), C, S, M, G);

    args[0] = getDefinition(G[c]);
    args[1] = getDefinition(G[t]);

    Z3_ast trueExpr = Z3_mk_and(mContext, 2, args);
    Z3_inc_ref(mContext, trueExpr);
    mRefs.push_back(trueExpr);

    const Vertex x = makeVertex(TypeId::And, nullptr, G, trueExpr);
    add_edge(nullptr, c, x, G);
    add_edge(nullptr, t, x, G);

    Z3_ast notCond = Z3_mk_not(mContext, args[0]);
    Z3_inc_ref(mContext, notCond);
    mRefs.push_back(notCond);

    args[0] = notCond;
    args[1] = getDefinition(G[f]);

    Z3_ast falseExpr = Z3_mk_and(mContext, 2, args);
    Z3_inc_ref(mContext, falseExpr);
    mRefs.push_back(falseExpr);

    const Vertex n = makeVertex(TypeId::Not, nullptr, G, notCond);

    add_edge(nullptr, c, n, G);

    const Vertex y = makeVertex(TypeId::And, nullptr, G, falseExpr);
    add_edge(nullptr, n, y, G);
    add_edge(nullptr, f, y, G);

    const Vertex u = makeVertex(TypeId::Or, stmt, C, S, M, G);
    add_edge(nullptr, x, u, G);
    add_edge(nullptr, y, u, G);

    return u;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief summarizeAST
 *
 * This function scans through a scope block and computes a DAG G in which any sequences of AND, OR or XOR functions
 * are "flattened" (i.e., allowed to have any number of inputs.)
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::transformAST(CharacterizationMap & C, Graph & G) {

    StatementMap S;

    VertexMap M;

    // Compute the base def-use graph ...
    for (Statement * stmt : *mBlock) {
        if (LLVM_UNLIKELY(isa<Sel>(stmt))) {

            const Vertex u = transcribeSel(cast<Sel>(stmt), C, S, M, G);

            resolveNestedUsages(stmt, u, C, S, M, G, stmt);

        } else {


            const Vertex u = makeVertex(stmt->getClassTypeId(), stmt, C, S, M, G);
            for (unsigned i = 0; i < stmt->getNumOperands(); ++i) {
                PabloAST * const op = stmt->getOperand(i);
                if (LLVM_LIKELY(isa<Statement>(op) || isa<Var>(op))) {
                    add_edge(op, makeVertex(TypeId::Var, op, C, S, M, G), u, G);
                }
            }
            if (LLVM_UNLIKELY(isa<If>(stmt))) {
                for (Var * def : cast<If>(stmt)->getEscaped()) {
                    const Vertex v = makeVertex(TypeId::Var, def, C, S, M, G);
                    add_edge(def, u, v, G);
                    resolveNestedUsages(def, v, C, S, M, G, stmt);
                }
                continue;
            } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
                // To keep G a DAG, we need to do a bit of surgery on loop variants because
                // the next variables it produces can be used within the condition. Instead,
                // we make the loop dependent on the original value of each Next node and
                // the Next node dependent on the loop.
                for (Var * var : cast<While>(stmt)->getEscaped()) {
                    const Vertex v = makeVertex(TypeId::Var, var, C, S, M, G);
                    assert (in_degree(v, G) == 1);
                    auto e = first(in_edges(v, G));
                    add_edge(G[e], source(e, G), u, G);
                    remove_edge(v, u, G);
                    add_edge(var, u, v, G);
                    resolveNestedUsages(var, v, C, S, M, G, stmt);
                }
                continue;
            } else {
                resolveNestedUsages(stmt, u, C, S, M, G, stmt);
            }
        }

    }

    if (redistributeGraph(C, M, G)) {
        factorGraph(G);
        rewriteAST(G);
        mModified = true;
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resolveNestedUsages
 ** ------------------------------------------------------------------------------------------------------------- */
void BooleanReassociationPass::resolveNestedUsages(PabloAST * const expr, const Vertex u,
                                                   CharacterizationMap & C, StatementMap & S, VertexMap & M, Graph & G,
                                                   const Statement * const ignoreIfThis) const {
    assert ("Cannot resolve nested usages of a null expression!" && expr);
    for (PabloAST * user : expr->users()) { assert (user);
        if (LLVM_LIKELY(user != expr && isa<Statement>(user))) {
            PabloBlock * parent = cast<Statement>(user)->getParent(); assert (parent);
            if (LLVM_UNLIKELY(parent != mBlock)) {
                for (;;) {
                    if (parent->getPredecessor() == mBlock) {
                        Statement * const branch = parent->getBranch();
                        if (LLVM_UNLIKELY(branch != ignoreIfThis)) {
                            // Add in a Var denoting the user of this expression so that it can be updated if expr changes.
                            const Vertex v = makeVertex(TypeId::Var, user, C, S, M, G);
                            add_edge(expr, u, v, G);
                            const Vertex w = makeVertex(branch->getClassTypeId(), branch, S, G);
                            add_edge(user, v, w, G);
                        }
                        break;
                    }
                    parent = parent->getPredecessor();
                    if (LLVM_UNLIKELY(parent == nullptr)) {
                        assert (isa<Assign>(expr));
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

    return cliques;
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
            TypeId outerTypeId = getType(G[u]);
            TypeId innerTypeId = (outerTypeId == TypeId::And) ? TypeId::Or : TypeId::And;
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
 * @brief recomputeDefinition
 ** ------------------------------------------------------------------------------------------------------------- */
inline Z3_ast BooleanReassociationPass::computeDefinition(TypeId typeId, const Vertex u, Graph & G, const bool use_expensive_minimization) const {
    const unsigned n = in_degree(u, G);
    Z3_ast operands[n];
    unsigned k = 0;
    for (const auto e : make_iterator_range(in_edges(u, G))) {
        const auto v = source(e, G);
        if (LLVM_UNLIKELY(getDefinition(G[v]) == nullptr)) {
            throw std::runtime_error("No definition for " + std::to_string(v));
        }
        operands[k++] = getDefinition(G[v]);
    }
    assert (k == n);
    Z3_ast const node = (typeId == TypeId::And) ? Z3_mk_and(mContext, n, operands) : Z3_mk_or(mContext, n, operands);
    return simplify(node, use_expensive_minimization);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateDefinition
 *
 * Apply the distribution law to reduce computations whenever possible.
 ** ------------------------------------------------------------------------------------------------------------- */
Vertex BooleanReassociationPass::updateIntermediaryDefinition(TypeId typeId, const Vertex u, VertexMap & M, Graph & G) {

    Z3_ast def = computeDefinition(typeId, u, G);
    Z3_ast orig = getDefinition(G[u]); assert (orig);

    Z3_dec_ref(mContext, orig);

    const auto g = M.find(orig);
    if (LLVM_LIKELY(g != M.end())) {
        M.erase(g);
    }

    const auto f = std::find(mRefs.rbegin(), mRefs.rend(), orig);
    assert (f != mRefs.rend());
    *f = def;

    const auto h = M.find(def);
    if (LLVM_UNLIKELY(h != M.end())) {
        const auto v = h->second;
        if (v != u) {
            for (auto e : make_iterator_range(out_edges(u, G))) {
                add_edge(G[e], v, target(e, G), G);
            }
            removeVertex(u, G);
            return v;
        }
    }

    getDefinition(G[u]) = def;
    M.emplace(def, u);
    return u;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateDefinition
 *
 * Apply the distribution law to reduce computations whenever possible.
 ** ------------------------------------------------------------------------------------------------------------- */
Vertex BooleanReassociationPass::updateSinkDefinition(TypeId typeId, const Vertex u, CharacterizationMap & C, VertexMap & M, Graph & G) {

    Z3_ast const def = computeDefinition(typeId, u, G);

    auto f = M.find(def);

    if (LLVM_UNLIKELY(f != M.end())) {
        Z3_dec_ref(mContext, def);
        Vertex v = f->second; assert (v != u);
        for (auto e : make_iterator_range(out_edges(u, G))) {
            add_edge(G[e], v, target(e, G), G);
        }
        removeVertex(u, G);
        return v;
    } else if (LLVM_LIKELY(C.predecessor() != nullptr)) {
        PabloAST * const factor = C.predecessor()->findKey(def);
        if (LLVM_UNLIKELY(factor != nullptr)) {
            getValue(G[u]) = factor;
            getType(G[u]) = TypeId::Var;
            clear_in_edges(u, G);
        }
    }

    getDefinition(G[u]) = def;
    mRefs.push_back(def);

    graph_traits<Graph>::in_edge_iterator begin, end;

restart:

    if (in_degree(u, G) > 1) {
        std::tie(begin, end) = in_edges(u, G);
        for (auto i = begin; ++i != end; ) {
            const auto v = source(*i, G);
            for (auto j = begin; j != i; ++j) {
                const auto w = source(*j, G);
                Z3_ast operands[2] = { getDefinition(G[v]), getDefinition(G[w]) };
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
                test = simplify(test, true);

                bool replacement = false;
                Vertex x = 0;
                const auto f = M.find(test);
                if (LLVM_UNLIKELY(f != M.end())) {
                    x = f->second;
                    Z3_ast orig = getDefinition(G[x]);
                    if (LLVM_UNLIKELY(orig != test)) {
                        std::string tmp;
                        raw_string_ostream out(tmp);
                        out << "vertex " << x << " is mapped to:\n"
                            << Z3_ast_to_string(mContext, test)
                            << "\n\nBut is recorded as:\n\n";
                        if (orig) {
                            out << Z3_ast_to_string(mContext, orig);
                        } else {
                            out << "<null>";
                        }
                        throw std::runtime_error(out.str());
                    }
                    Z3_dec_ref(mContext, test);
                    replacement = true;
                } else if (LLVM_LIKELY(C.predecessor() != nullptr)) {
                    PabloAST * const factor = C.predecessor()->findKey(test);
                    if (LLVM_UNLIKELY(factor != nullptr)) {
                        x = makeVertex(TypeId::Var, factor, G, test);
                        M.emplace(test, x);
                        replacement = true;
                        mRefs.push_back(test);
                    }
                }

                if (LLVM_UNLIKELY(replacement)) {

                    assert (G[*i] == nullptr);
                    assert (G[*j] == nullptr);

                    remove_edge(*i, G);
                    remove_edge(*j, G);

                    add_edge(nullptr, x, u, G);

                    goto restart;
                }

                Z3_dec_ref(mContext, test);
            }
        }
    }

    M.emplace(def, u);

    return u;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief redistributeAST
 *
 * Apply the distribution law to reduce computations whenever possible.
 ** ------------------------------------------------------------------------------------------------------------- */
bool BooleanReassociationPass::redistributeGraph(CharacterizationMap & C, VertexMap & M, Graph & G) {

    bool modified = false;

//    errs() << "=====================================================\n";

    DistributionGraph H;

    for (;;) {

        contractGraph(M, G);

//        printGraph(G, "G");

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

        mRefs.reserve(distributionSets.size() * 2);

        for (const DistributionSet & set : distributionSets) {

            // Each distribution tuple consists of the sources, intermediary, and sink nodes.
            const VertexSet & sources = std::get<0>(set);
            const VertexSet & intermediary = std::get<1>(set);
            const VertexSet & sinks = std::get<2>(set);

            TypeId outerTypeId = getType(G[H[sinks.front()]]);
            assert (outerTypeId == TypeId::And || outerTypeId == TypeId::Or);
            TypeId innerTypeId = (outerTypeId == TypeId::Or) ? TypeId::And : TypeId::Or;

            const Vertex x = makeVertex(outerTypeId, nullptr, G);
            const Vertex y = makeVertex(innerTypeId, nullptr, G);

            // Update G to reflect the distributed operations (including removing the subgraph of
            // the to-be distributed edges.)

            add_edge(nullptr, x, y, G);

            for (const Vertex i : sources) {
                const auto u = H[i];
                for (const Vertex j : intermediary) {
                    const auto v = H[j];
                    assert (getType(G[v]) == innerTypeId);
                    const auto e = edge(u, v, G); assert (e.second);
                    remove_edge(e.first, G);
                }
                add_edge(nullptr, u, y, G);
            }

            for (const Vertex i : intermediary) {

                const auto u = updateIntermediaryDefinition(innerTypeId, H[i], M, G);

                for (const Vertex j : sinks) {
                    const auto v = H[j];
                    assert (getType(G[v]) == outerTypeId);
                    const auto e = edge(u, v, G); assert (e.second);
                    add_edge(G[e.first], y, v, G);
                    remove_edge(e.first, G);
                }
                add_edge(nullptr, u, x, G);
            }

            updateSinkDefinition(outerTypeId, x, C, M, G);

            updateSinkDefinition(innerTypeId, y, C, M, G);

        }

        H.clear();

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
        case TypeId::If:
        case TypeId::While:
        case TypeId::Count:
            return false;
        default:
            return true;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief unique_source
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool has_unique_source(const Vertex u, const Graph & G) {
    if (in_degree(u, G) > 0) {
        graph_traits<Graph>::in_edge_iterator i, end;
        std::tie(i, end) = in_edges(u, G);
        const Vertex v = source(*i, G);
        while (++i != end) {
            if (source(*i, G) != v) {
                return false;
            }
        }
        return true;
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief unique_target
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool has_unique_target(const Vertex u, const Graph & G) {
    if (out_degree(u, G) > 0) {
        graph_traits<Graph>::out_edge_iterator i, end;
        std::tie(i, end) = out_edges(u, G);
        const Vertex v = target(*i, G);
        while (++i != end) {
            if (target(*i, G) != v) {
                return false;
            }
        }
        return true;
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief contractGraph
 ** ------------------------------------------------------------------------------------------------------------- */
bool BooleanReassociationPass::contractGraph(VertexMap & M, Graph & G) const {

    bool contracted = false;

    circular_buffer<Vertex> ordering(num_vertices(G));

    topological_sort(G, std::back_inserter(ordering)); // reverse topological ordering

    // first contract the graph
    for (const Vertex u : ordering) {
        if (in_degree(u, G) == 0 && out_degree(u, G) == 0) {
            continue;
        } else if (LLVM_LIKELY(out_degree(u, G) > 0)) {
            if (isAssociative(G[u])) {
                if (LLVM_UNLIKELY(has_unique_source(u, G))) {
                    // We have a redundant node here that'll simply end up being a duplicate
                    // of the input value. Remove it and add any of its outgoing edges to its
                    // input node.
                    const auto ei = first(in_edges(u, G));
                    const Vertex v = source(ei, G);
                    for (auto ej : make_iterator_range(out_edges(u, G))) {
                        add_edge(G[ej], v, target(ej, G), G);
                    }
                    removeVertex(u, M, G);
                    contracted = true;
                } else if (LLVM_UNLIKELY(has_unique_target(u, G))) {
                    // Otherwise if we have a single user, we have a similar case as above but
                    // we can only merge this vertex into the outgoing instruction if they are
                    // of the same type.
                    const auto ei = first(out_edges(u, G));
                    const Vertex v = target(ei, G);
                    if (LLVM_UNLIKELY(getType(G[v]) == getType(G[u]))) {
                        for (auto ej : make_iterator_range(in_edges(u, G))) {
                            add_edge(G[ej], source(ej, G), v, G);
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
 * @brief factorGraph
 ** ------------------------------------------------------------------------------------------------------------- */
bool BooleanReassociationPass::factorGraph(TypeId typeId, Graph & G, std::vector<Vertex> & factors) const {

    if (LLVM_UNLIKELY(factors.empty())) {
        return false;
    }

    std::vector<Vertex> I, J, K;

    bool modified = false;

    for (unsigned i = 1; i < factors.size(); ++i) {
        assert (getType(G[factors[i]]) == typeId);
        for (auto ei : make_iterator_range(in_edges(factors[i], G))) {
            I.push_back(source(ei, G));
        }
        std::sort(I.begin(), I.end());
        for (unsigned j = 0; j < i; ++j) {
            for (auto ej : make_iterator_range(in_edges(factors[j], G))) {
                J.push_back(source(ej, G));
            }
            std::sort(J.begin(), J.end());
            // get the pairwise intersection of each set of inputs (i.e., their common subexpression)
            std::set_intersection(I.begin(), I.end(), J.begin(), J.end(), std::back_inserter(K));
            assert (std::is_sorted(K.begin(), K.end()));
            // if the intersection contains at least two elements
            const auto n = K.size();
            if (n > 1) {
                Vertex a = factors[i];
                Vertex b = factors[j];
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isMutable
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool isMutable(const Vertex u, const Graph & G) {
    return getType(G[u]) != TypeId::Var;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief rewriteAST
 ** ------------------------------------------------------------------------------------------------------------- */
bool BooleanReassociationPass::rewriteAST(Graph & G) {

    using line_t = long long int;

    enum : line_t { MAX_INT = std::numeric_limits<line_t>::max() };

    // errs() << "---------------------------------------------------------\n";

    // printGraph(G, "X");

    Z3_config cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "model", "true");
    Z3_context ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);
    Z3_solver solver = Z3_mk_solver(ctx);
    Z3_solver_inc_ref(ctx, solver);

    std::vector<Z3_ast> mapping(num_vertices(G), nullptr);

    flat_map<PabloAST *, Z3_ast> V;

    // Generate the variables
    const auto ty = Z3_mk_int_sort(ctx);
    Z3_ast ZERO = Z3_mk_int(ctx, 0, ty);

    for (const Vertex u : make_iterator_range(vertices(G))) {
        const auto var = Z3_mk_fresh_const(ctx, nullptr, ty);
        Z3_ast constraint = nullptr;
        if (in_degree(u, G) > 0) {
            constraint = Z3_mk_gt(ctx, var, ZERO);
            Z3_solver_assert(ctx, solver, constraint);
        }        
        PabloAST * const expr = getValue(G[u]);
        if (inCurrentBlock(expr, mBlock)) {
            V.emplace(expr, var);
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
    ordering.reserve(V.size() - 1);

    Z3_ast prior = nullptr;
    unsigned gap = 1;
    for (Statement * stmt : *mBlock) {
        auto f = V.find(stmt);
        if (f != V.end()) {
            Z3_ast const node = f->second;
            if (prior) {
//                ordering.push_back(Z3_mk_lt(ctx, prior, node)); // increases the cost by 6 - 10x
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
    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);

    std::sort(S.begin(), S.end(), [&L](const Vertex u, const Vertex v){ return L[u] < L[v]; });

    mBlock->setInsertPoint(nullptr);

    std::vector<Vertex> T;

    line_t count = 1;

    for (auto u : S) {
        PabloAST *& stmt = getValue(G[u]);

        assert (isMutable(u, G));
        assert (L[u] > 0 && L[u] < MAX_INT);

        bool append = true;

        if (isAssociative(G[u])) {

            if (in_degree(u, G) == 0 || out_degree(u, G) == 0) {
                throw std::runtime_error("Vertex " + std::to_string(u) + " is either a source or sink node but marked as associative!");
            }

            Statement * ip = mBlock->getInsertPoint();
            ip = ip ? ip->getNextNode() : mBlock->front();

            const auto typeId = getType(G[u]);

            T.clear();
            T.reserve(in_degree(u, G));
            for (const auto e : make_iterator_range(in_edges(u, G))) {
                T.push_back(source(e, G));
            }

            // Then sort them by their line position (noting any incoming value will either be 0 or MAX_INT)
            std::sort(T.begin(), T.end(), [&L](const Vertex u, const Vertex v){ return L[u] < L[v]; });

            if (LoadEarly) {
                mBlock->setInsertPoint(nullptr);
            }

            PabloAST * expr = nullptr;
            PabloAST * join = nullptr;

            for (auto v : T) {
                expr = getValue(G[v]);
                if (LLVM_UNLIKELY(expr == nullptr)) {
                    throw std::runtime_error("Vertex " + std::to_string(v) + " does not have an expression!");
                }
                if (join) {

                    if (in_degree(v, G) > 0) {

                        assert (L[v] > 0 && L[v] < MAX_INT);

                        Statement * dom = cast<Statement>(expr);
                        for (;;) {
                            PabloBlock * const parent = dom->getParent();
                            if (parent == mBlock) {
                                break;
                            }
                            dom = parent->getBranch(); assert(dom);
                        }
                        mBlock->setInsertPoint(dom);

                        assert (dominates(join, expr));
                        assert (dominates(expr, ip));
                        assert (dominates(dom, ip));
                    }

                    switch (typeId) {
                        case TypeId::And:
                            expr = mBlock->createAnd(join, expr); break;
                        case TypeId::Or:
                            expr = mBlock->createOr(join, expr); break;
                        case TypeId::Xor:
                            expr = mBlock->createXor(join, expr); break;
                        default:
                            llvm_unreachable("Invalid TypeId!");
                    }
                }
                join = expr;
            }

            assert (expr);

            Statement * const currIP = mBlock->getInsertPoint();

            mBlock->setInsertPoint(ip->getPrevNode());

            stmt = expr;

            // If the insertion point isn't the statement we just attempted to create,
            // we must have unexpectidly reused a prior statement (or var.)
            if (LLVM_UNLIKELY(expr != currIP)) {
                L[u] = 0;
                // If that statement happened to be in the current block, locate which
                // statement it "duplicated" (if any) and set the duplicate's line # to
                // the line of the original statement to maintain a consistent view of
                // the program.
                if (inCurrentBlock(expr, mBlock)) {
                    for (auto v : make_iterator_range(vertices(G))) {
                        if (LLVM_UNLIKELY(getValue(G[v]) == expr)) {
                            L[u] = L[v];
                            break;
                        }
                    }
                }
                append = false;
            }
        } else if (stmt == nullptr) {
            assert (getType(G[u]) == TypeId::Not);
            assert (in_degree(u, G) == 1);
            PabloAST * op = getValue(G[source(first(in_edges(u, G)), G)]); assert (op);
            stmt = mBlock->createNot(op);
        } else if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            for (auto e : make_iterator_range(out_edges(u, G))) {
                const auto v = target(e, G);
                assert (L[v] == std::numeric_limits<line_t>::max());
                L[v] = count;
            }
        }

        for (auto e : make_iterator_range(out_edges(u, G))) {
            if (G[e]) {
                if (PabloAST * user = getValue(G[target(e, G)])) {
                    cast<Statement>(user)->replaceUsesOfWith(G[e], stmt);
                }
            }
        }

        if (LLVM_LIKELY(append)) {
            mBlock->insert(cast<Statement>(stmt));
            L[u] = count++; // update the line count with the actual one.
        }
    }

    Statement * const end = mBlock->getInsertPoint(); assert (end);
    for (;;) {
        Statement * const next = end->getNextNode();
        if (next == nullptr) {
            break;
        }

        #ifndef NDEBUG
        for (PabloAST * user : next->users()) {
            if (isa<Statement>(user) && dominates(user, next)) {
                std::string tmp;
                raw_string_ostream out(tmp);
                out << "Erasing ";
                PabloPrinter::print(next, out);
                out << " erroneously modifies live statement ";
                PabloPrinter::print(cast<Statement>(user), out);
                throw std::runtime_error(out.str());
            }
        }
        #endif
        next->eraseFromParent(true);
    }

    #ifndef NDEBUG
    PabloVerifier::verify(mFunction, "mid-reassociation");
    #endif

    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
* @brief makeVertex
** ------------------------------------------------------------------------------------------------------------- */
Vertex BooleanReassociationPass::makeVertex(TypeId typeId, PabloAST * const expr, CharacterizationMap & C, StatementMap & S, VertexMap & M, Graph & G) {
    assert (expr);
    const auto f = S.find(expr);
    if (f != S.end()) {
        assert (getValue(G[f->second]) == expr);
        return f->second;
    }
    const auto node = C.get(expr);   
    const Vertex u = makeVertex(typeId, expr, G, node);
    S.emplace(expr, u);
    if (node) {
        M.emplace(node, u);
    }
    return u;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeVertex
 ** ------------------------------------------------------------------------------------------------------------- */
Vertex BooleanReassociationPass::makeVertex(TypeId typeId, PabloAST * const expr, StatementMap & M, Graph & G, Z3_ast node) {
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
 * @brief makeVertex
 ** ------------------------------------------------------------------------------------------------------------- */
Vertex BooleanReassociationPass::makeVertex(TypeId typeId, PabloAST * const expr, Graph & G, Z3_ast node) {
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
 * @brief removeVertex
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BooleanReassociationPass::removeVertex(const Vertex u, VertexMap & M, Graph & G) const {
    VertexData & ref = G[u];
    Z3_ast def = getDefinition(ref); assert (def);
    auto f = M.find(def); assert (f != M.end());
    M.erase(f);
    removeVertex(u, G);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief removeVertex
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
inline Z3_ast BooleanReassociationPass::makeVar() {
    Z3_ast node = Z3_mk_fresh_const(mContext, nullptr, Z3_mk_bool_sort(mContext));
    Z3_inc_ref(mContext, node);
    mRefs.push_back(node);
    return node;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplify
 ** ------------------------------------------------------------------------------------------------------------- */
inline Z3_ast BooleanReassociationPass::simplify(Z3_ast const node, bool use_expensive_minimization) const {
    assert (node);
    Z3_inc_ref(mContext, node);
    Z3_ast result = nullptr;
    if (use_expensive_minimization) {

        Z3_goal g = Z3_mk_goal(mContext, true, false, false); assert (g);
        Z3_goal_inc_ref(mContext, g);
        Z3_goal_assert(mContext, g, node);

        Z3_apply_result r = Z3_tactic_apply(mContext, mTactic, g); assert (r);
        Z3_apply_result_inc_ref(mContext, r);
        Z3_goal_dec_ref(mContext, g);

        assert (Z3_apply_result_get_num_subgoals(mContext, r) == 1);

        Z3_goal h = Z3_apply_result_get_subgoal(mContext, r, 0); assert (h);
        Z3_goal_inc_ref(mContext, h);
        Z3_apply_result_dec_ref(mContext, r);

        const unsigned n = Z3_goal_size(mContext, h);

        if (n == 1) {
            result = Z3_goal_formula(mContext, h, 0); assert (result);
            Z3_inc_ref(mContext, result);
        } else if (n > 1) {
            Z3_ast operands[n];
            for (unsigned i = 0; i < n; ++i) {
                operands[i] = Z3_goal_formula(mContext, h, i);
                Z3_inc_ref(mContext, operands[i]);
            }
            result = Z3_mk_and(mContext, n, operands); assert (result);
            Z3_inc_ref(mContext, result);
            for (unsigned i = 0; i < n; ++i) {
                Z3_dec_ref(mContext, operands[i]);
            }
        } else {
            result = Z3_mk_true(mContext); assert (result);
        }
        Z3_goal_dec_ref(mContext, h);

    } else {        
        result = Z3_simplify_ex(mContext, node, mParams); assert (result);
        Z3_inc_ref(mContext, result);
    }    
    Z3_dec_ref(mContext, node);
    assert (result);
    return result;
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
inline BooleanReassociationPass::BooleanReassociationPass(Z3_context ctx, Z3_params params, Z3_tactic tactic, PabloFunction & f)
: mBlock(nullptr)
, mContext(ctx)
, mParams(params)
, mTactic(tactic)
, mFunction(f)
, mModified(false) {

}

}
