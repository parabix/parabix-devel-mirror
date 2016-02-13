#include "distributivepass.h"

#include <pablo/codegenstate.h>
#include <pablo/analysis/pabloverifier.hpp>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/passes/flattenassociativedfg.h>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <pablo/printer_pablos.h>
#include <iostream>

using namespace boost;
using namespace boost::container;

namespace pablo {

using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, PabloAST *>;
using Vertex = Graph::vertex_descriptor;
using Map = flat_map<PabloAST *, Vertex>;
using VertexSet = std::vector<Vertex>;
using Biclique = std::pair<VertexSet, VertexSet>;
using BicliqueSet = std::vector<Biclique>;
using DistributionSet = std::tuple<VertexSet, VertexSet, VertexSet>;
using DistributionSets = std::vector<DistributionSet>;
using TypeId = PabloAST::ClassTypeId;

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
static BicliqueSet && independentCliqueSets(BicliqueSet && bicliques, const bool uppsetSet) {
    using IndependentSetGraph = adjacency_list<hash_setS, vecS, undirectedS, unsigned>;

    const auto l = bicliques.size();
    IndependentSetGraph I(l);


    // Initialize our weights and determine the constraints
    if (uppsetSet) {
        for (auto i = bicliques.begin(); i != bicliques.end(); ++i) {
            I[std::distance(bicliques.begin(), i)] = std::pow(std::get<0>(*i).size(), 2);
            for (auto j = i; ++j != bicliques.end(); ) {
                if (intersects(i->first, j->first)) {
                    add_edge(std::distance(bicliques.begin(), i), std::distance(bicliques.begin(), j), I);
                }
            }
        }
    } else {
        for (auto i = bicliques.begin(); i != bicliques.end(); ++i) {
            I[std::distance(bicliques.begin(), i)] = std::pow(std::get<1>(*i).size(), 2);
            for (auto j = i; ++j != bicliques.end(); ) {
                if (intersects(i->first, j->first) && intersects(i->second, j->second)) {
                    add_edge(std::distance(bicliques.begin(), i), std::distance(bicliques.begin(), j), I);
                }
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
        if (w < (uppsetSet ? 2 : 1)) break;
        selected.push_back(u);
        I[u] = 0;
        for (auto v : make_iterator_range(adjacent_vertices(u, I))) {
            I[v] = 0;
        }
    }

    // Sort the selected list and then remove the unselected cliques
    std::sort(selected.begin(), selected.end(), std::greater<Vertex>());
    auto end = bicliques.end();
    for (const unsigned offset : selected) {
        end = bicliques.erase(bicliques.begin() + offset + 1, end) - 1;
    }
    bicliques.erase(bicliques.begin(), end);

    return std::move(bicliques);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerateBicliques
 *
 * Adaptation of the MICA algorithm as described in "Consensus algorithms for the generation of all maximal
 * bicliques" by Alexe et. al. (2003). Note: this implementation considers all verticies in set A to be in
 * bipartition A and their adjacencies to be in B.
  ** ------------------------------------------------------------------------------------------------------------- */
static BicliqueSet enumerateBicliques(const Graph & G, const VertexSet & A, const unsigned min) {
    using IntersectionSets = std::set<VertexSet>;

    IntersectionSets B1;
    VertexSet tmp;
    for (auto u : A) {
        if (in_degree(u, G) > 0) {
            tmp.reserve(in_degree(u, G));
            for (auto e : make_iterator_range(in_edges(u, G))) {
                tmp.push_back(source(e, G));
            }
            if (tmp.size() >= min) {
                std::sort(tmp.begin(), tmp.end());
                B1.emplace(tmp.begin(), tmp.end());
            }
            tmp.clear();
        }
    }

    IntersectionSets B(B1);

    IntersectionSets Bi;
    for (auto i = B1.begin(); i != B1.end(); ++i) {
        for (auto j = i; ++j != B1.end(); ) {
            std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(tmp));
            if (tmp.size() >= min) {
                if (B.count(tmp) == 0) {
                    Bi.emplace(tmp.begin(), tmp.end());
                }
            }
            tmp.clear();
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
                std::set_intersection(i->begin(), i->end(), j->begin(), j->end(), std::back_inserter(tmp));
                if (tmp.size() >= min) {
                    if (B.count(tmp) == 0) {
                        Bk.emplace(tmp.begin(), tmp.end());
                    }
                }
                tmp.clear();
            }
        }
        Bi.swap(Bk);
    }

    BicliqueSet cliques;
    cliques.reserve(B.size());
    for (auto Bi = B.begin(); Bi != B.end(); ++Bi) {
        assert (Bi->size() >= min);
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
 * @brief removeUnhelpfulBicliques
 *
 * An intermediary vertex could have more than one outgoing edge but if any that are not directed to vertices in
 * the lower biclique, we'll need to compute that specific value anyway. Remove them from the clique set and if
 * there are not enough vertices in the biclique to make distribution profitable, eliminate the clique.
 ** ------------------------------------------------------------------------------------------------------------- */
static BicliqueSet && removeUnhelpfulBicliques(BicliqueSet && cliques, const Graph & G) {
    for (auto ci = cliques.begin(); ci != cliques.end(); ) {
        const auto cardinalityA = std::get<0>(*ci).size();
        VertexSet & B = std::get<1>(*ci);
        for (auto bi = B.begin(); bi != B.end(); ) {
            if (G[*bi]->getNumUses() == cardinalityA) {
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
inline static DistributionSets safeDistributionSets(const Graph & G, const VertexSet & A) {
    DistributionSets T;
    BicliqueSet lowerSet = independentCliqueSets(removeUnhelpfulBicliques(enumerateBicliques(G, A, 1), G), false);
    for (Biclique & lower : lowerSet) {
        BicliqueSet upperSet = independentCliqueSets(enumerateBicliques(G, std::get<1>(lower), 2), true);
        for (Biclique & upper : upperSet) {
            T.emplace_back(std::move(std::get<1>(upper)), std::move(std::get<0>(upper)), std::get<0>(lower));
        }
    }
    return std::move(T);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief scopeDepthOf
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned scopeDepthOf(const PabloBlock * block) {
    unsigned depth = 0;
    for (; block; ++depth, block = block->getParent());
    return depth;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief findInsertionPoint
 ** ------------------------------------------------------------------------------------------------------------- */
inline PabloBlock * findInsertionPoint(const VertexSet & users, const Graph & G) {
    std::vector<PabloBlock *> scopes(0);
    scopes.reserve(users.size());
    for (Vertex u : users) {
        PabloBlock * const scope = cast<Statement>(G[u])->getParent(); assert (scope);
        if (std::find(scopes.begin(), scopes.end(), scope) == scopes.end()) {
            scopes.push_back(scope);
        }
    }
    while (scopes.size() > 1) {
        // Find the LCA of both scopes then add the LCA back to the list of scopes.
        PabloBlock * scope1 = scopes.back();
        scopes.pop_back();
        PabloBlock * scope2 = scopes.back();
        scopes.pop_back();
        assert (scope1 != scope2);
        unsigned depth1 = scopeDepthOf(scope1);
        unsigned depth2 = scopeDepthOf(scope2);
        // If one of these scopes is nested deeper than the other, scan upwards through
        // the scope tree until both scopes are at the same depth.
        while (depth1 > depth2) {
            scope1 = scope1->getParent();
            --depth1;
        }
        while (depth1 < depth2) {
            scope2 = scope2->getParent();
            --depth2;
        }
        assert (depth1 == depth2);
        // Then iteratively step backwards until we find a matching scopes; this must be
        // the LCA of our original pair.
        while (scope1 != scope2) {
            scope1 = scope1->getParent();
            scope2 = scope2->getParent();
        }
        assert (scope1 && scope2);
        if (std::find(scopes.begin(), scopes.end(), scope1) == scopes.end()) {
            scopes.push_back(scope1);
        }
    }
    assert (scopes.size() == 1);
    PabloBlock * const root = scopes.front();
    // Now that we know the common scope of these users, test which statement is the first to require it.
    flat_set<Statement *> usages;
    usages.reserve(users.size());
    for (Vertex u : users) {
        Statement * user = cast<Statement>(G[u]);
        PabloBlock * scope = user->getParent();
        while (scope != root) {
            assert (scope);
            user = scope->getBranch();
            scope = scope->getParent();
        }
        usages.insert(user);
    }
    Statement * ip = nullptr;
    for (Statement * stmt : *root) {
        if (usages.count(stmt)) {
            ip = stmt->getPrevNode();
            break;
        }
    }
    assert (ip);
    root->setInsertPoint(ip);
    return root;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeDistributionGraph
 ** ------------------------------------------------------------------------------------------------------------- */
static inline void computeDistributionGraph(Variadic * const expr, Graph & G, VertexSet & A) {

    const TypeId outerTypeId = expr->getClassTypeId();
    const TypeId innerTypeId = (outerTypeId == TypeId::And) ? TypeId::Or : TypeId::And;

    assert (isa<And>(expr) || isa<Or>(expr));

    Map M;
    for (unsigned i = 0; i != expr->getNumOperands(); ++i) {
        PabloAST * const op = expr->getOperand(i);
        if (op->getClassTypeId() == innerTypeId) {
            bool distributable = true;
            for (PabloAST * user : op->users()) {
                // Early check to see whether it'd be beneficial to distribute it. If this fails, we'd have
                // to compute the operand's value anyway, so just ignore this operand.
                if (user->getClassTypeId() != outerTypeId) {
                    distributable = false;
                    break;
                }
            }
            if (distributable) {
                const Vertex u = add_vertex(op, G);
                for (PabloAST * user : op->users()) {
                    const auto f = M.find(user);
                    Vertex v = 0;
                    if (LLVM_LIKELY(f != M.end())) {
                        v = f->second;
                    } else {
                        v = add_vertex(user, G);
                        M.emplace(user, v);
                        A.push_back(v);
                    }
                    add_edge(u, v, G);
                }
                for (PabloAST * input : *cast<Variadic>(op)) {
                    const auto f = M.find(input);
                    Vertex v = 0;
                    if (f != M.end()) {
                        v = f->second;
                    } else {
                        v = add_vertex(input, G);
                        M.emplace(input, v);
                    }
                    add_edge(v, u, G);
                }
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief distribute
 *
 * Based on the knowledge that:
 *
 *   (P ∧ Q) ∨ (P ∧ R) ⇔ P ∧ (Q ∨ R) and (P ∨ Q) ∧ (P ∨ R) ⇔ P ∨ (Q ∧ R)
 *
 * Try to eliminate some of the unnecessary operations from the current Variadic expression.
 ** ------------------------------------------------------------------------------------------------------------- */
inline void DistributivePass::distribute(Variadic * const var) {



    assert (isa<And>(var) || isa<Or>(var));

    Graph G;
    VertexSet A;

    std::vector<Variadic *> Q = {var};

    while (Q.size() > 0) {
        Variadic * expr = CanonicalizeDFG::canonicalize(Q.back()); Q.pop_back();
        PabloAST * const replacement = Simplifier::fold(expr, expr->getParent());
        if (LLVM_UNLIKELY(replacement != nullptr)) {
            expr->replaceWith(replacement, true, true);
            if (LLVM_UNLIKELY(isa<Variadic>(replacement))) {
                Q.push_back(cast<Variadic>(replacement));
            }
            continue;
        }

        if (LLVM_LIKELY(isa<And>(expr) || isa<Or>(expr))) {

            computeDistributionGraph(expr, G, A);

            // If we found no potential opportunities then we cannot apply the distribution law to any part of G.
            if (num_vertices(G) == 0) {
                assert (A.empty());
                continue;
            }

            const auto S = safeDistributionSets(G, A);
            if (S.empty()) {
                G.clear();
                A.clear();
                continue;
            }

            Q.push_back(expr);

            for (const DistributionSet & set : S) {

                // Each distribution tuple consists of the sources, intermediary, and sink nodes.
                const VertexSet & sources = std::get<0>(set);
                assert (sources.size() > 0);
                const VertexSet & intermediary = std::get<1>(set);
                assert (intermediary.size() > 1);
                const VertexSet & sinks = std::get<2>(set);
                assert (sinks.size() > 0);

                for (const Vertex u : intermediary) {
                    for (const Vertex v : sources) {
                        cast<Variadic>(G[u])->deleteOperand(G[v]);
                    }
                }
                for (const Vertex u : sinks) {
                    for (const Vertex v : intermediary) {
                        cast<Variadic>(G[u])->deleteOperand(G[v]);
                    }
                }

                PabloBlock * const block = findInsertionPoint(sinks, G);
                Variadic * innerOp = nullptr;
                Variadic * outerOp = nullptr;
                if (isa<And>(expr)) {
                    outerOp = block->createAnd(intermediary.size());
                    innerOp = block->createOr(sources.size() + 1);
                } else {
                    outerOp = block->createOr(intermediary.size());
                    innerOp = block->createAnd(sources.size() + 1);
                }
                for (const Vertex v : intermediary) {
                    outerOp->addOperand(G[v]);
                }
                for (const Vertex v : sources) {
                    innerOp->addOperand(G[v]);
                }
                for (const Vertex u : sinks) {
                    cast<Variadic>(G[u])->addOperand(innerOp);
                }
                innerOp->addOperand(outerOp);
                // Push our newly constructed ops into the Q
                Q.push_back(innerOp);
                Q.push_back(outerOp);

                unmodified = false;
            }

            G.clear();
            A.clear();
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief distribute
 ** ------------------------------------------------------------------------------------------------------------- */
void DistributivePass::distribute(PabloBlock * const block) {
    Statement * stmt = block->front();
    while (stmt) {
        Statement * next = stmt->getNextNode();
        if (isa<If>(stmt) || isa<While>(stmt)) {
            distribute(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        } else if (isa<And>(stmt) || isa<Or>(stmt)) {
            distribute(cast<Variadic>(stmt));
        }
        stmt = next;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool DistributivePass::optimize(PabloFunction & function) {
    DistributivePass dp;
    unsigned rounds = 0;
    for (;;) {
        dp.unmodified = true;
        dp.distribute(function.getEntryBlock());
        if (dp.unmodified) {
            break;
        }
        ++rounds;
        #ifndef NDEBUG
        PabloVerifier::verify(function, "post-distribution");
        #endif
        Simplifier::optimize(function);
    }
    return rounds > 0;
}


}
