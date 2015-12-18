#include "flattenassociativedfg.h"
#include <pablo/codegenstate.h>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/analysis/pabloverifier.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <pablo/printer_pablos.h>
#include <iostream>

using namespace boost;
using namespace boost::container;

namespace pablo {

using TypeId = PabloAST::ClassTypeId;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief coalesce
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FlattenAssociativeDFG::coalesce(Variadic * const var) {
    const TypeId typeId = var->getClassTypeId();
    for (unsigned i = 0; i < var->getNumOperands(); ) {
        PabloAST * const op = var->getOperand(i);
        if (op->getClassTypeId() == typeId) {
            Variadic * removedVar = cast<Variadic>(var->removeOperand(i));
            for (unsigned j = 0; j != cast<Variadic>(op)->getNumOperands(); ++j) {
                var->addOperand(cast<Variadic>(op)->getOperand(j));
            }
            if (removedVar->getNumOperands() == 1) {
                removedVar->replaceWith(removedVar->getOperand(0));
            } else if (removedVar->getNumUses() == 0) {
                removedVar->eraseFromParent(true);
            }
            continue;
        }
        ++i;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief coalesce
 ** ------------------------------------------------------------------------------------------------------------- */
void FlattenAssociativeDFG::coalesce(PabloBlock * const block, const bool traverse) {
    Statement * stmt = block->front();
    while (stmt) {
        Statement * next = stmt->getNextNode();
        if (LLVM_UNLIKELY((isa<If>(stmt) || isa<While>(stmt)) && traverse)) {
            coalesce(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody(), true);
        } else if (isa<And>(stmt) || isa<Or>(stmt) || isa<Xor>(stmt)) {
            coalesce(cast<Variadic>(stmt));
        } else if (isa<Not>(stmt)) {
            deMorgansExpansion(cast<Not>(stmt), block);
        }
        stmt = next;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deMorgansExpansion
 *
 * Apply the De Morgans' law to any negated And or Or statement with the intent of further coalescing its operands
 * thereby allowing the Simplifier to check for tautologies and contradictions.
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FlattenAssociativeDFG::deMorgansExpansion(Not * const var, PabloBlock * const block) {
    PabloAST * const negatedVar = var->getOperand(0);
    if (isa<And>(negatedVar) || isa<Or>(negatedVar)) {
        Variadic * src = cast<Variadic>(negatedVar);
        const unsigned operands = src->getNumOperands();
        Variadic * replacement = nullptr;
        block->setInsertPoint(var->getPrevNode());
        if (isa<And>(negatedVar)) {
            replacement = block->createOr(operands);
        } else {
            replacement = block->createAnd(operands);
        }
        block->setInsertPoint(replacement->getPrevNode());
        for (unsigned i = 0; i != operands; ++i) {
            replacement->addOperand(block->createNot(src->getOperand(i)));
        }
        coalesce(replacement);
        var->replaceWith(replacement, true, true);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deMorgansReduction
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FlattenAssociativeDFG::deMorgansReduction(Variadic * const var, PabloBlock * const block) {
    unsigned negations = 0;
    for (unsigned i = 0; i < var->getNumOperands(); ++i) {
        if (isa<Not>(var->getOperand(i))) {
            ++negations;
        }
    }
    if (negations > 1) {
        PabloAST * negated[negations];
        for (unsigned i = var->getNumOperands(), j = negations; i && j; ) {
            if (isa<Not>(var->getOperand(--i))) {
                negated[--j] = cast<Not>(var->removeOperand(i))->getOperand(0);
            }
        }
        block->setInsertPoint(var->getPrevNode());
        Variadic * extractedVar = nullptr;
        if (isa<And>(var)) {
            extractedVar = block->createOr(negations);
        } else { // if (isa<Or>(var)) {
            extractedVar = block->createAnd(negations);
        }
        for (unsigned i = 0; i != negations; ++i) {
            extractedVar->addOperand(negated[i]);
        }
        var->addOperand(block->createNot(extractedVar));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deMorgansReduction
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FlattenAssociativeDFG::deMorgansReduction(PabloBlock * const block) {
    for (Statement * stmt : *block) {
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            deMorgansReduction(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        } else if (isa<And>(stmt) || isa<Or>(stmt)) {
            deMorgansReduction(cast<Variadic>(stmt), block);
        }
    }
}

union VertexData {
    Assign * def;
    TypeId   typeId;
    explicit VertexData() : def(nullptr) { }
    explicit VertexData(Assign * def) : def(def) { }
    explicit VertexData(TypeId typeId) : typeId(typeId) { }
};
using Graph = adjacency_list<vecS, vecS, bidirectionalS, VertexData, Variadic *>;
using Vertex = Graph::vertex_descriptor;
using Map = flat_map<TypeId, Vertex>;
using VertexSet = std::vector<Vertex>;
using Biclique = std::pair<VertexSet, VertexSet>;
using BicliqueSet = std::vector<Biclique>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addToVariadicGraph
 ** ------------------------------------------------------------------------------------------------------------- */
static bool addToVariadicGraph(Assign * const def, Graph & G, Map & M, VertexSet & A) {

    // Test if its valid to transform this statement
    for (PabloAST * user : def->users()) {
        if (isa<Variadic>(user) == 0) {
            if (isa<If>(user)) {
                const auto defs = cast<If>(user)->getDefined();
                if (LLVM_LIKELY(std::find(defs.begin(), defs.end(), def) != defs.end())) {
                    continue;
                }
            }
            return false;
        }
    }

    // Add the statement and its users to G
    const Vertex u = add_vertex(VertexData(def), G);
    A.push_back(u);
    for (PabloAST * user : def->users()) {
        if (isa<Variadic>(user)) {
            auto f = M.find(user->getClassTypeId());
            if (f == M.end()) {
                f = M.emplace(user->getClassTypeId(), add_vertex(VertexData(user->getClassTypeId()), G)).first;
            }
            assert (f != M.end());
            G[add_edge(u, f->second, G).first] = cast<Variadic>(user);
        }
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief matches
 ** ------------------------------------------------------------------------------------------------------------- */
inline static bool matches(const PabloAST * const a, const PabloAST * const b) {
    return (isa<Assign>(b)) && (cast<Assign>(a)->getParent() == cast<Assign>(b)->getParent());
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
        flat_set<Vertex> adj;
        adj.reserve(out_degree(u, G));
        for (auto e : make_iterator_range(out_edges(u, G))) {
            adj.insert(target(e, G));
        }
        B1.emplace(adj.begin(), adj.end());
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

    BicliqueSet S;
    for (auto Bi = B.begin(); Bi != B.end(); ++Bi) {
        VertexSet Ai(A);
        for (const Vertex u : *Bi) {
            VertexSet Aj;
            Aj.reserve(in_degree(u, G));
            for (auto e : make_iterator_range(in_edges(u, G))) {
                Aj.push_back(source(e, G));
            }
            std::sort(Aj.begin(), Aj.end());
            VertexSet Ak;
            Ak.reserve(std::min(Ai.size(), Aj.size()));
            std::set_intersection(Ai.begin(), Ai.end(), Aj.begin(), Aj.end(), std::back_inserter(Ak));
            Ai.swap(Ak);
        }
        assert (Ai.size() > 0); // cannot happen if this algorithm is working correctly
        // If |Ai| > |Bi|, removing Ai from of the Variadic and sinking it into the nested scope will
        // reduce the number of values stored.
        if (Ai.size() > Bi->size()) {
            S.emplace_back(std::move(Ai), std::move(*Bi));
        }
    }
    return S;
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
inline static BicliqueSet independentCliqueSets(BicliqueSet && cliques, const unsigned minimum) {
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
 * @brief tryToPartiallyExtractVariadic
 ** ------------------------------------------------------------------------------------------------------------- */
void FlattenAssociativeDFG::tryToPartiallyExtractVariadic(Variadic * const var) {

    for (unsigned i = 0; i < var->getNumOperands(); ++i) {
        PabloAST * const op = var->getOperand(i);
        if (isa<Assign>(op)) {
            // Have we found a variadic operation that can sunk into a nested scope?
            for (unsigned j = i + 1; j != var->getNumOperands(); ++j) {
                bool invalid = false;
                if (LLVM_UNLIKELY(matches(op, var->getOperand(j)))) {
                    Graph G;
                    Map M;
                    VertexSet A;
                    if (addToVariadicGraph(cast<Assign>(op), G, M, A) == 0) {
                        invalid = true;
                        break;
                    }
                    addToVariadicGraph(cast<Assign>(var->getOperand(j)), G, M, A);
                    for (++j; j != var->getNumOperands(); ++j) {
                        if (LLVM_UNLIKELY(matches(op, var->getOperand(j)))) {
                            addToVariadicGraph(cast<Assign>(var->getOperand(j)), G, M, A);
                        }
                    }
                    if (A.size() > 1) {
                        const auto S = independentCliqueSets<0>(std::move(enumerateBicliques(G, A)), 2);
                        for (const Biclique & C : S) {
                            const VertexSet & sources = std::get<0>(C);
                            const VertexSet & variadics = std::get<1>(C);
                            PabloBlock * const block = cast<Assign>(op)->getParent();
                            block->setInsertPoint(block->back());
                            for (const auto v : variadics) {
                                Variadic * joiner = nullptr;
                                switch (G[v].typeId) {
                                    case TypeId::And:
                                        joiner = block->createAnd(sources.size());
                                        break;
                                    case TypeId::Or:
                                        joiner = block->createOr(sources.size());
                                        break;
                                    case TypeId::Xor:
                                        joiner = block->createXor(sources.size());
                                        break;
                                    default:
                                        break;
                                }
                                assert (joiner);
                                flat_set<Variadic *> vars;
                                for (const auto u : sources) {
                                    Assign * const def = G[u].def;
                                    joiner->addOperand(def->getOperand(0));
                                    for (auto e : make_iterator_range(out_edges(u, G))) {
                                        if (LLVM_LIKELY(target(e, G) == v)) {
                                            Variadic * const var = cast<Variadic>(G[e]);
                                            vars.insert(var);
                                            var->deleteOperand(def);
                                        }
                                    }
                                    assert (def->getNumUses() == 1);
                                    def->eraseFromParent();
                                }
                                coalesce(joiner);
                                Assign * const def = block->createAssign("m", joiner);
                                cast<If>(block->getBranch())->addDefined(def);
                                for (Variadic * var : vars) {
                                    var->addOperand(def);
                                }
                            }
                        }
                    }
                    break;
                }
                if (invalid) {
                    break;
                }
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief removeFalseScopeDependencies
 ** ------------------------------------------------------------------------------------------------------------- */
inline void FlattenAssociativeDFG::removeFalseScopeDependencies(Assign * const def) {
    if (isa<Variadic>(def->getOperand(0))) {
        Variadic * const var = cast<Variadic>(def->getOperand(0));
        for (unsigned i = 0; i != var->getNumOperands(); ++i) {
            if (isa<Assign>(var->getOperand(i))) {
                Assign * const nestedDef = cast<Assign>(var->getOperand(i));
                if (LLVM_LIKELY(nestedDef->getOperand(0)->getClassTypeId() == var->getClassTypeId())) {
                    if (LLVM_LIKELY(nestedDef->getNumUses() == 2)) { // The If node that produces it and the "var"
                        Variadic * const nestedVar = cast<Variadic>(nestedDef->getOperand(0));
                        if (LLVM_LIKELY(nestedVar->getNumUses() == 1 && nestedVar->getNumOperands() > 0)) {
                            for (unsigned i = 0, j = 0; ; ) {
                                if (var->getOperand(i) < nestedVar->getOperand(j)) {
                                    if (++i == var->getNumOperands()) {
                                        break;
                                    }
                                } else {
                                    if (var->getOperand(i) > nestedVar->getOperand(j)) {
                                        ++j;
                                    } else {
                                        nestedVar->removeOperand(j);
                                    }
                                    if (j == nestedVar->getNumOperands()) {
                                        break;
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
 * @brief removeFalseScopeDependencies
 *
 * After coalescing the AST, we may find that a result of some If statement is added to a result of a subsequent
 * If statement. Unless necessary for correctness, eliminate it as we can potentially schedule the If nodes
 * better without the sequential dependency.
 ** ------------------------------------------------------------------------------------------------------------- */
void FlattenAssociativeDFG::removeFalseScopeDependencies(PabloBlock * const block) {
    for (Statement * stmt = block->back(); stmt; stmt = stmt->getPrevNode()) {
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            removeFalseScopeDependencies(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        } else if (LLVM_UNLIKELY(isa<Assign>(stmt))) {
            removeFalseScopeDependencies(cast<Assign>(stmt));
        } else if (isa<Variadic>(stmt)) {
            tryToPartiallyExtractVariadic(cast<Variadic>(stmt));
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief transform
 ** ------------------------------------------------------------------------------------------------------------- */
void FlattenAssociativeDFG::transform(PabloFunction & function) {

    FlattenAssociativeDFG::coalesce(function.getEntryBlock(), true);
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-coalescence");
    #endif

    Simplifier::optimize(function);

    FlattenAssociativeDFG::deMorgansReduction(function.getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-demorgans-reduction");
    #endif

    Simplifier::optimize(function);

    FlattenAssociativeDFG::removeFalseScopeDependencies(function.getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-remove-false-scope-dependencies");
    #endif

    Simplifier::optimize(function);
}

}
