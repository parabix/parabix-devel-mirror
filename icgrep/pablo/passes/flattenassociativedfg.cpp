#include "flattenassociativedfg.h"
#include <pablo/codegenstate.h>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/analysis/pabloverifier.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <queue>

#include <pablo/optimizers/distributivepass.h>

using namespace boost;
using namespace boost::container;

namespace pablo {

using TypeId = PabloAST::ClassTypeId;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief canonicalize
 ** ------------------------------------------------------------------------------------------------------------- */
Variadic * CanonicalizeDFG::canonicalize(Variadic * var) {
    assert (isa<And>(var) || isa<Or>(var) || isa<Xor>(var));
    for (unsigned i = 0; i < var->getNumOperands(); ) {
        if (var->getOperand(i)->getClassTypeId() == var->getClassTypeId()) {
            Variadic * const removedVar = cast<Variadic>(var->removeOperand(i));
            for (unsigned j = 0; j != removedVar->getNumOperands(); ++j) {
                var->addOperand(removedVar->getOperand(j));
            }
            if (removedVar->getNumUses() == 0) {
                removedVar->eraseFromParent(true);
            }
            continue;
        }
        ++i;
    }
    return var;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief coalesce
 ** ------------------------------------------------------------------------------------------------------------- */
void CanonicalizeDFG::canonicalize(PabloBlock * const block) {
    Statement * stmt = block->front();
    while (stmt) {
        Statement * next = stmt->getNextNode();
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            canonicalize(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        } else if (isa<Variadic>(stmt)) {
            canonicalize(cast<Variadic>(stmt));
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
inline void CanonicalizeDFG::deMorgansExpansion(Not * const negation, PabloBlock * const block) {
    PabloAST * const negatedVar = negation->getOperand(0);
    if (isa<And>(negatedVar) || isa<Or>(negatedVar)) {
        const TypeId typeId = isa<And>(negatedVar) ? TypeId::Or : TypeId::And;
        bool expandable = false;
        for (PabloAST * user : negation->users()) {
            if (user->getClassTypeId() == typeId) {
                expandable = true;
                break;
            }
        }
        if (expandable) {

            const unsigned operands = cast<Variadic>(negatedVar)->getNumOperands();
            PabloAST * negations[operands];
            for (unsigned i = 0; i != operands; ++i) {
                PabloAST * op = cast<Variadic>(negatedVar)->getOperand(i);
                Statement * ip = negation;
                if (LLVM_LIKELY(isa<Statement>(op))) {
                    Statement * br = cast<Statement>(op);
                    PabloBlock * scope = br->getParent();
                    for (;;) {
                        if (LLVM_UNLIKELY(scope == nullptr)) {
                            break;
                        } else if (LLVM_UNLIKELY(scope == negation->getParent())) {
                            ip = br;
                            break;
                        }
                        br = scope->getBranch();
                        scope = scope->getParent();
                    }
                }
                block->setInsertPoint(ip);
                negations[i] = block->createNot(op);
            }

            for (PabloAST * user : negation->users()) {
                if (user->getClassTypeId() == typeId) {
                    cast<Variadic>(user)->deleteOperand(negation);
                    for (unsigned i = 0; i != operands; ++i) {
                        cast<Variadic>(user)->addOperand(negations[i]);
                    }
                }
            }

        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deMorgansExpansion
 ** ------------------------------------------------------------------------------------------------------------- */
void CanonicalizeDFG::deMorgansExpansion(PabloBlock * const block) {
    for (Statement * stmt : *block) {
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            deMorgansExpansion(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        } else if (isa<Not>(stmt)) {
            deMorgansExpansion(cast<Not>(stmt), block);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deMorgansReduction
 ** ------------------------------------------------------------------------------------------------------------- */
inline void CanonicalizeDFG::deMorgansReduction(Variadic * const var, PabloBlock * const block) {
    assert (isa<And>(var) || isa<Or>(var));
    unsigned negations = 0;
    const unsigned operands = var->getNumOperands();
    PabloAST * negated[operands];
    for (unsigned i = 0; i < operands; ++i) {
        PabloAST * const op = var->getOperand(i);
        if (isa<Not>(op)) {
            negated[negations++] = cast<Not>(op)->getOperand(0);
        }
    }
    if (negations > 1) {
        for (unsigned i = operands; i; ) {
            PabloAST * const op = var->getOperand(--i);
            if (isa<Not>(op)) {
                var->removeOperand(i);
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
void CanonicalizeDFG::deMorgansReduction(PabloBlock * const block) {
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
using DependencyGraph = adjacency_list<vecS, vecS, bidirectionalS, VertexData, Variadic *>;
using Vertex = DependencyGraph::vertex_descriptor;
using SourceMap = flat_map<Assign *, Vertex>;
using SinkMap = flat_map<TypeId, Vertex>;
using VertexSet = std::vector<Vertex>;
using Biclique = std::pair<VertexSet, VertexSet>;
using BicliqueSet = std::vector<Biclique>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addToVariadicGraph
 ** ------------------------------------------------------------------------------------------------------------- */
static bool addToVariadicGraph(Assign * const def, DependencyGraph & G, SourceMap & A, SinkMap & B) {

    if (LLVM_LIKELY(A.count(def) == 0)) {
        // Test if its valid to transform this statement
        for (PabloAST * user : def->users()) {
            if (isa<Variadic>(user) == 0) {
                if (isa<If>(user)) {
                    if (LLVM_LIKELY(cast<If>(user)->getCondition() != def)) {
                        continue;
                    }
                }
                return false;
            }
        }

        // Add the statement and its users to G
        const Vertex u = add_vertex(VertexData(def), G);
        A.emplace(def, u);
        for (PabloAST * user : def->users()) {
            if (isa<Variadic>(user)) {
                auto f = B.find(user->getClassTypeId());
                if (f == B.end()) {
                    f = B.emplace(user->getClassTypeId(), add_vertex(VertexData(user->getClassTypeId()), G)).first;
                }
                assert (f != B.end());
                G[add_edge(u, f->second, G).first] = cast<Variadic>(user);
            }
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
static BicliqueSet enumerateBicliques(const DependencyGraph & G, const VertexSet & A) {
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
    assert (std::is_sorted(first1, last1));
    assert (std::is_sorted(first2, last2));
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
inline static BicliqueSet independentCliqueSets(BicliqueSet && bicliques, const unsigned minimum) {
    using IndependentSetGraph = adjacency_list<hash_setS, vecS, undirectedS, unsigned>;

    const auto l = bicliques.size();
    IndependentSetGraph I(l);

    // Initialize our weights and determine the constraints
    for (auto i = bicliques.begin(); i != bicliques.end(); ++i) {
        I[std::distance(bicliques.begin(), i)] = std::pow(std::get<side>(*i).size(), 2);
        for (auto j = i; ++j != bicliques.end(); ) {
            if (intersects(i->second, j->second) && intersects(i->first, j->first)) {
                add_edge(std::distance(bicliques.begin(), i), std::distance(bicliques.begin(), j), I);
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
    auto end = bicliques.end();
    for (const unsigned offset : selected) {
        end = bicliques.erase(bicliques.begin() + offset + 1, end) - 1;
    }
    bicliques.erase(bicliques.begin(), end);

    return std::move(bicliques);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief tryToPartiallyExtractVariadic
 ** ------------------------------------------------------------------------------------------------------------- */
inline void CanonicalizeDFG::tryToPartiallyExtractVariadic(Variadic * const var) {
    for (unsigned i = 0; i < var->getNumOperands(); ++i) {
        PabloAST * const op = var->getOperand(i);
        if (isa<Assign>(op)) {
            // Look for two Assign statements in this variadic; we don't want to generate a dependency graph unless
            // we expect some optimization can occur.
            for (unsigned j = i + 1; j != var->getNumOperands(); ++j) {
                bool invalid = false;
                if (LLVM_UNLIKELY(matches(op, var->getOperand(j)))) {
                    DependencyGraph G;
                    SourceMap A;
                    SinkMap B;
                    if (addToVariadicGraph(cast<Assign>(op), G, A, B) == 0) {
                        invalid = true;
                        break;
                    }
                    addToVariadicGraph(cast<Assign>(var->getOperand(j)), G, A, B);
                    for (++j; j != var->getNumOperands(); ++j) {
                        if (LLVM_UNLIKELY(matches(op, var->getOperand(j)))) {
                            addToVariadicGraph(cast<Assign>(var->getOperand(j)), G, A, B);
                        }
                    }

                    if (A.size() > 1) {

                        VertexSet H;
                        H.reserve(A.size());
                        for (auto a : A) {
                            H.push_back(a.second);
                        }

                        const auto S = independentCliqueSets<0>(std::move(enumerateBicliques(G, H)), 2);
                        if (LLVM_UNLIKELY(S.empty())) {
                            break;
                        }
                        for (const Biclique & C : S) {
                            const VertexSet & sources = std::get<0>(C);
                            const VertexSet & variadics = std::get<1>(C);
                            assert (variadics.size() > 0);
                            assert (sources.size() > variadics.size());
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
                                    default: llvm_unreachable("Unexpected!");
                                }
                                assert (joiner);
                                flat_set<Assign *> defs;
                                for (const auto u : sources) {
                                    defs.insert(G[u].def);
                                }
                                for (Assign * def : defs) {
                                    joiner->addOperand(def->getOperand(0));                                    
                                }
                                Assign * const joined = block->createAssign("m", canonicalize(joiner));
                                for (Assign * def : defs) {
                                    def->replaceWith(joined);
                                    assert (def->getNumUses() == 0);
                                }
                            }
                        }
                        --i;
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
 * @brief tryToPartiallyExtractVariadic
 ** ------------------------------------------------------------------------------------------------------------- */
void CanonicalizeDFG::tryToPartiallyExtractVariadic(PabloBlock * const block) {
    for (Statement * stmt = block->back(); stmt; stmt = stmt->getPrevNode()) {
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            tryToPartiallyExtractVariadic(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        } else if (isa<Variadic>(stmt)) {
            tryToPartiallyExtractVariadic(cast<Variadic>(stmt));
        }
    }
}

using ScopeDependencyGraph = adjacency_list<hash_setS, vecS, bidirectionalS, PabloAST *>;
using ScopeDependencyMap = flat_map<PabloAST *, ScopeDependencyGraph::vertex_descriptor>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief find
 ** ------------------------------------------------------------------------------------------------------------- */
inline ScopeDependencyGraph::vertex_descriptor find(PabloAST * expr, ScopeDependencyGraph & G, ScopeDependencyMap & M) {
    auto f = M.find(expr);
    if (f == M.end()) {
        f = M.emplace(expr, add_vertex(expr, G)).first;
    }
    return f->second;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief buildScopeDependencyGraph
 ** ------------------------------------------------------------------------------------------------------------- */
ScopeDependencyGraph::vertex_descriptor buildScopeDependencyGraph(Variadic * const var, ScopeDependencyGraph & G, ScopeDependencyMap & M) {
    auto f = M.find(var);
    if (f != M.end()) {
        return f->second;
    }
    auto u = add_vertex(var, G);
    M.emplace(var, u);
    for (unsigned i = 0; i != var->getNumOperands(); ++i) {
        PabloAST * expr = var->getOperand(i);
        PabloAST * value = var;
        while (isa<Assign>(expr)) {            
            value = expr;
            expr = cast<Assign>(expr)->getExpression();
        }
        if ((expr->getClassTypeId() == var->getClassTypeId()) && (expr->getNumUses() == 1)) {
            const auto v = find(value, G, M);
            add_edge(v, u, G);
            add_edge(buildScopeDependencyGraph(cast<Variadic>(expr), G, M), v, G);
        }
    }
    return u;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief analyzeScopeDependencies
 ** ------------------------------------------------------------------------------------------------------------- */
inline void analyzeScopeDependencies(Assign * const def, ScopeDependencyGraph & G, ScopeDependencyMap & M) {
    if (LLVM_LIKELY(isa<Variadic>(def->getExpression()))) {
        buildScopeDependencyGraph(cast<Variadic>(def->getExpression()), G, M);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief analyzeScopeDependencies
 ** ------------------------------------------------------------------------------------------------------------- */
void analyzeScopeDependencies(PabloBlock * const block, ScopeDependencyGraph & G, ScopeDependencyMap & M) {
    for (Statement * stmt : *block) {
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            analyzeScopeDependencies(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody(), G, M);
        } else if (LLVM_UNLIKELY(isa<Assign>(stmt))) {
            analyzeScopeDependencies(cast<Assign>(stmt), G, M);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief removeDependenciesWithUnresolvedUses
 ** ------------------------------------------------------------------------------------------------------------- */
void removeDependenciesWithUnresolvedUses(ScopeDependencyGraph & G) {
    for (auto u : make_iterator_range(vertices(G))) {
        const PabloAST * const expr = G[u];
        unsigned uses = 0;
        if (isa<Assign>(expr)) {
            for (const PabloAST * user : cast<Assign>(expr)->users()) {
                if (!isa<If>(user) || cast<If>(user)->getCondition() == expr) {
                    ++uses;
                }
            }
        } else {
            uses = expr->getNumUses();
        }
        if (uses != out_degree(u, G)) {
            clear_out_edges(u, G);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief eliminateUnecessaryDependencies
 ** ------------------------------------------------------------------------------------------------------------- */
void eliminateUnecessaryDependencies(ScopeDependencyGraph & G) {
    using Vertex = ScopeDependencyGraph::vertex_descriptor;
    std::vector<bool> visited(num_vertices(G), false);
    std::queue<Vertex> Q;
    for (auto u : make_iterator_range(vertices(G))) {
        if (out_degree(u, G) == 0 && in_degree(u, G) != 0) {
            Q.push(u);
        }
    }
    while (Q.size() > 0) {
        const auto u = Q.front(); Q.pop();
        visited[u] = true;
        for (auto e : make_iterator_range(in_edges(u, G))) {
            bool usersHaveBeenVisited = true;
            const auto v = source(e, G);
            for (auto e : make_iterator_range(out_edges(v, G))) {
                if (visited[target(e, G)] == 0) {
                    usersHaveBeenVisited = false;
                    break;
                }
            }
            if (usersHaveBeenVisited) {
                Q.push(v);
                for (auto e : make_iterator_range(in_edges(u, G))) {
                    const auto w = source(e, G);
                    if (w != v) {
                        auto f = add_edge(w, v, G);
                        if (f.second == 0) {
                            cast<Variadic>(G[v])->deleteOperand(G[w]);
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
 *
 * TODO: make this only iterate over the scope blocks and test the scope branch.
 ** ------------------------------------------------------------------------------------------------------------- */
inline void CanonicalizeDFG::removeFalseScopeDependencies(PabloFunction & function) {
    ScopeDependencyGraph G;
    {
        ScopeDependencyMap M;
        analyzeScopeDependencies(function.getEntryBlock(), G, M);
    }
    removeDependenciesWithUnresolvedUses(G);
    eliminateUnecessaryDependencies(G);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief transform
 ** ------------------------------------------------------------------------------------------------------------- */
void CanonicalizeDFG::transform(PabloFunction & function) {

    CanonicalizeDFG::canonicalize(function.getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-canonicalize");
    #endif

    Simplifier::optimize(function);

//    CanonicalizeDFG::deMorgansReduction(function.getEntryBlock());
//    #ifndef NDEBUG
//    PabloVerifier::verify(function, "post-demorgans-reduction");
//    #endif

//    Simplifier::optimize(function);

    CanonicalizeDFG::removeFalseScopeDependencies(function);
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-remove-false-scope-dependencies");
    #endif

    Simplifier::optimize(function);

    CanonicalizeDFG::tryToPartiallyExtractVariadic(function.getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-partial-variadic-extraction");
    #endif

    Simplifier::optimize(function);

//    CanonicalizeDFG::deMorgansExpansion(function.getEntryBlock());
//    #ifndef NDEBUG
//    PabloVerifier::verify(function, "post-demorgans-expansion");
//    #endif

//    Simplifier::optimize(function);


}

}
