#include "distributivepass.h"

#include <pablo/codegenstate.h>
#include <pablo/analysis/pabloverifier.hpp>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/passes/flattenassociativedfg.h>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/graph/adjacency_list.hpp>

using namespace boost;
using namespace boost::container;

namespace pablo {

using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, PabloAST *>;
using Vertex = Graph::vertex_descriptor;
using SinkMap = flat_map<PabloAST *, Vertex>;
using VertexSet = std::vector<Vertex>;
using Biclique = std::pair<VertexSet, VertexSet>;
using BicliqueSet = std::vector<Biclique>;
using DistributionSet = std::tuple<VertexSet, VertexSet, VertexSet>;
using DistributionSets = std::vector<DistributionSet>;

using TypeId = PabloAST::ClassTypeId;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getVertex
 ** ------------------------------------------------------------------------------------------------------------- */
static inline Vertex getVertex(PabloAST * value, Graph & G, SinkMap & M) {
    const auto f = M.find(value);
    if (f != M.end()) {
        return f->second;
    }
    const auto u = add_vertex(value, G);
    M.emplace(value, u);
    return u;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateDistributionGraph
 *
 * Generate a graph G describing the potential applications of the distributive law for the given block.
 ** ------------------------------------------------------------------------------------------------------------- */
VertexSet generateDistributionGraph(PabloBlock * block, Graph & G) {
    SinkMap M;
    VertexSet distSet;
    for (Statement * stmt : *block) {
        if (isa<And>(stmt) || isa<Or>(stmt)) {
            const TypeId outerTypeId = stmt->getClassTypeId();
            const TypeId innerTypeId = (outerTypeId == TypeId::And) ? TypeId::Or : TypeId::And;
            flat_set<PabloAST *> distributable;
            for (PabloAST * const expr : *cast<Variadic>(stmt)) {
                if (LLVM_UNLIKELY(expr->getClassTypeId() == innerTypeId)) {
                    bool safe = true;
                    for (PabloAST * const user : expr->users()) {
                        if (user->getClassTypeId() != outerTypeId) {
                            safe = false;
                            break;
                        }
                    }
                    if (safe) {
                        distributable.insert(expr);
                    }
                }
            }
            if (LLVM_LIKELY(distributable.size() > 1)) {
                flat_map<PabloAST *, bool> observedMoreThanOnce;
                bool anyOpportunities = false;
                for (const PabloAST * distOperation : distributable) {
                    for (PabloAST * const distVar : *cast<Variadic>(distOperation)) {
                        auto ob = observedMoreThanOnce.find(distVar);
                        if (ob == observedMoreThanOnce.end()) {
                            observedMoreThanOnce.emplace(distVar, false);
                        } else {
                            ob->second = true;
                            anyOpportunities = true;
                        }
                    }
                }
                if (anyOpportunities) {
                    for (const auto ob : observedMoreThanOnce) {
                        PabloAST * distVar = nullptr;
                        bool observedTwice = false;
                        std::tie(distVar, observedTwice) = ob;
                        if (observedTwice) {
                            const Vertex z = getVertex(stmt, G, M);
                            distSet.push_back(z);
                            for (PabloAST * const distOperation : distVar->users()) {
                                if (distributable.count(distOperation)) {
                                    const Vertex y = getVertex(distOperation, G, M);
                                    add_edge(getVertex(distVar, G, M), y, G);
                                    add_edge(y, z, G);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return distSet;
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
 * @brief enumerateBicliques
 *
 * Adaptation of the MICA algorithm as described in "Consensus algorithms for the generation of all maximal
 * bicliques" by Alexe et. al. (2003). Note: this implementation considers all verticies in set A to be in
 * bipartition A and their adjacencies to be in B.
  ** ------------------------------------------------------------------------------------------------------------- */
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
 * @brief removeUnhelpfulBicliques
 *
 * An intermediary vertex could have more than one outgoing edge but if any that are not directed to vertices in
 * the lower biclique, we'll need to compute that specific value anyway. Remove them from the clique set and if
 * there are not enough vertices in the biclique to make distribution profitable, eliminate the clique.
 ** ------------------------------------------------------------------------------------------------------------- */
static BicliqueSet && removeUnhelpfulBicliques(BicliqueSet && cliques, Graph & G) {
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
static DistributionSets safeDistributionSets(Graph & G, const VertexSet & distSet) {
    DistributionSets T;
    BicliqueSet lowerSet = independentCliqueSets<1>(removeUnhelpfulBicliques(enumerateBicliques(G, distSet), G), 1);
    for (Biclique & lower : lowerSet) {
        BicliqueSet upperSet = independentCliqueSets<0>(enumerateBicliques(G, std::get<1>(lower)), 2);
        for (Biclique & upper : upperSet) {
            T.emplace_back(std::move(std::get<1>(upper)), std::move(std::get<0>(upper)), std::get<0>(lower));
        }
    }
    return std::move(T);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief process
 ** ------------------------------------------------------------------------------------------------------------- */
inline void DistributivePass::process(PabloBlock * const block) {

    for (;;) {

        // FlattenAssociativeDFG::coalesce(block, false);

        Graph G;

        const VertexSet distSet = generateDistributionGraph(block, G);

        // If we found no potential opportunities then we cannot apply the distribution law to any part of G.
        if (LLVM_UNLIKELY(distSet.empty())) {
            break;
        }

        const DistributionSets distributionSets = safeDistributionSets(G, distSet);

        if (LLVM_UNLIKELY(distributionSets.empty())) {
            break;
        }

        for (const DistributionSet & set : distributionSets) {

            // Each distribution tuple consists of the sources, intermediary, and sink nodes.
            const VertexSet & sources = std::get<0>(set);
            const VertexSet & intermediary = std::get<1>(set);
            const VertexSet & sinks = std::get<2>(set);

            // Find the first sink and set the insert point immediately before that.
            Variadic * innerOp = nullptr;
            Variadic * outerOp = nullptr;

            block->setInsertPoint(cast<Variadic>(G[sinks.front()])->getPrevNode());
            if (isa<And>(G[sinks.front()])) {
                outerOp = block->createAnd(intermediary.size());
                innerOp = block->createOr(sources.size() + 1);
            } else {
                outerOp = block->createOr(intermediary.size());
                innerOp = block->createAnd(sources.size() + 1);
            }

            for (const Vertex u : intermediary) {
                for (const Vertex v : sinks) {
                    cast<Variadic>(G[v])->deleteOperand(G[u]);
                }
                outerOp->addOperand(G[u]);
            }
            FlattenAssociativeDFG::coalesce(outerOp);

            for (const Vertex u : sources) {
                for (const Vertex v : intermediary) {
                    cast<Variadic>(G[v])->deleteOperand(G[u]);
                }
                innerOp->addOperand(G[u]);
            }
            innerOp->addOperand(outerOp);
            FlattenAssociativeDFG::coalesce(innerOp);

            for (const Vertex u : sinks) {
                Variadic * const resultOp = cast<Variadic>(G[u]);
                resultOp->addOperand(innerOp);
                FlattenAssociativeDFG::coalesce(resultOp);
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief distribute
 ** ------------------------------------------------------------------------------------------------------------- */
void DistributivePass::distribute(PabloBlock * const block) {
    for (Statement * stmt : *block) {
        if (isa<If>(stmt) || isa<While>(stmt)) {
            distribute(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        }
    }
    process(block);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
void DistributivePass::optimize(PabloFunction & function) {
    DistributivePass::distribute(function.getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-distribution");
    #endif
    Simplifier::optimize(function);
//    FlattenAssociativeDFG::deMorgansReduction(function.getEntryBlock());
//    #ifndef NDEBUG
//    PabloVerifier::verify(function, "post-demorgans-reduction");
//    #endif
//    Simplifier::optimize(function);
}


}
