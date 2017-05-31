#include "distributivepass.h"

#include <pablo/pablo_kernel.h>
#include <pablo/codegenstate.h>
#include <pablo/branch.h>
#include <pablo/pe_string.h>
#include <pablo/pe_integer.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <pablo/boolean.h>
#include <pablo/pe_var.h>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/function_output_iterator.hpp>
#include <set>

#include <boost/graph/strong_components.hpp>
#include <llvm/Support/raw_ostream.h>

using namespace boost;
using namespace boost::container;
using namespace llvm;

using TypeId = pablo::PabloAST::ClassTypeId;
using VertexData = std::pair<pablo::PabloAST *, TypeId>;

using Graph = adjacency_list<vecS, vecS, bidirectionalS, VertexData, pablo::PabloAST *>;
using Vertex = Graph::vertex_descriptor;
using in_edge_iterator = graph_traits<Graph>::in_edge_iterator;
using out_edge_iterator = graph_traits<Graph>::out_edge_iterator;

using VertexSet = std::vector<Vertex>;
using Biclique = std::pair<VertexSet, VertexSet>;
using BicliqueSet = std::vector<Biclique>;
using DistributionSet = std::tuple<VertexSet, VertexSet, VertexSet>;
using DistributionSets = std::vector<DistributionSet>;

using IndependentSetGraph = adjacency_list<hash_setS, vecS, undirectedS, unsigned>;

namespace pablo {


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printGraph
 ** ------------------------------------------------------------------------------------------------------------- */
static void printGraph(const Graph & G, const std::string & name, llvm::raw_ostream & out) {

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
        std::tie(expr, typeId) = G[u];
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
                out << " ("; expr->print(out); out << ")";
            }
        } else {
            expr->print(out);
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
            PabloAST * const expr = G[e];
            if (expr) {
                out << "label=\"";
                expr->print(out);
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

struct PassContainer {

    enum Modification {
        None, Trivial, Structural
    };

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief run
     *
     * Based on the knowledge that:
     *
     *  (ASSOCIATIVITY)    A ∧ (B ∧ C) ⇔ (A ∧ B) ∧ C ⇔ A ∧ B ∧ C   and   A ∨ (B ∨ C) ⇔ (A ∨ B) ∨ C ⇔ A ∨ B ∨ C
     *
     *  (IDENTITY)         A ∨ 0 ⇔ A   and   A ∧ 1 = A
     *
     *  (ANNULMENT)        A ∧ 0 ⇔ 0   and   A ∨ 1 = 1
     *
     *  (IDEMPOTENT)       A ∨ (A ∨ B) ⇔ A ∨ B   and   A ∧ (A ∧ B) ⇔ A ∧ B
     *
     *  (ABSORBTION)       A ∨ (A ∧ B) ⇔ A ∧ (A ∨ B) ⇔ A
     *
     *  (COMPLEMENT)       A ∨ ¬A ⇔ 1   and   A ∧ ¬A = 0
     *
     *  (DISTRIBUTIVITY)   (A ∧ B) ∨ (A ∧ C) ⇔ A ∧ (B ∨ C)   and   (A ∨ B) ∧ (A ∨ C) ⇔ A ∨ (B ∧ C)
     *
     * Try to eliminate some of the unnecessary operations from the current Variadic expressions
     ** ------------------------------------------------------------------------------------------------------------- */
    void run(PabloKernel * const kernel) {
        run(kernel->getEntryBlock());

        printGraph(G, "G", errs());
        if (simplifyGraph() == Structural) {
            // rewriteAST(first, stmt);
            printGraph(G, "O", errs());
        }

    }

    void run(PabloBlock * const block) {
        for (Statement * stmt : *block) {            
            if (isa<Branch>(stmt)) {
                addBranch(stmt);
                run(cast<Branch>(stmt)->getBody());
            } else {
                addStatement(stmt);
            }
        }

//        G.clear();
//        M.clear();
//        for (Statement * stmt : *block) {
//            addStatement(stmt);
//        }

//        printGraph(G, "G", errs());
//        if (simplifyGraph() == Structural) {
//            // rewriteAST(first, stmt);
//            printGraph(G, "O", errs());
//        }

    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief simplifyGraph
     ** ------------------------------------------------------------------------------------------------------------- */
    Modification simplifyGraph() {
        Modification modified = None;
        for (;;) {
            const auto p1 = applyAssociativeIdentityAnnulmentLaws();
            const auto p2 = applyAbsorbtionComplementIdempotentLaws();
            const auto p3 = applyDistributivityLaw();
            if (std::max(std::max(p1, p2), p3) != Structural) {
                break;
            }
            modified = Structural;
        }
        return modified;
    }

protected:

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief applyAssociativeIdentityAnnulmentLaws
     ** ------------------------------------------------------------------------------------------------------------- */
    Modification applyAssociativeIdentityAnnulmentLaws() {

        auto identityComparator = [this](const Vertex u, const Vertex v) -> bool {
            const auto typeA = getType(u);
            assert (typeA != TypeId::Var);
            const auto typeB = getType(v);
            assert (typeB != TypeId::Var);
            if (LLVM_LIKELY(typeA != typeB)) {
                using value_of = std::underlying_type<TypeId>::type;
                return static_cast<value_of>(typeA) < static_cast<value_of>(typeB);
            } else {
                const auto degA = in_degree(u, G);
                const auto degB = in_degree(v, G);
                if (degA != degB) {
                    return degA < degB;
                } else {
                    Vertex adjA[degA];
                    Vertex adjB[degA];
                    in_edge_iterator ei, ej;
                    std::tie(ei, std::ignore) = in_edges(u, G);
                    std::tie(ej, std::ignore) = in_edges(v, G);
                    for (size_t i = 0; i < degA; ++i, ++ei, ++ej) {
                        adjA[i] = source(*ei, G);
                        adjB[i] = source(*ej, G);
                    }
                    std::sort(adjA, adjA + degA);
                    std::sort(adjB, adjB + degA);
                    for (size_t i = 0; i < degA; ++i) {
                        if (adjA[i] < adjB[i]) {
                            return true;
                        }
                    }
                    return false;
                }
            }
        };

        flat_set<Vertex, decltype(identityComparator)> V(identityComparator);
        V.reserve(num_vertices(G));

        VertexSet ordering;
        ordering.reserve(num_vertices(G));

        topological_sort(G, std::back_inserter(ordering)); // note: ordering is in reverse topological order

        Modification modified = None;

        for (const auto u : boost::adaptors::reverse(ordering)) {
            const TypeId typeId = getType(u);
            if (isImmutable(typeId)) {
                continue;
            } else if (LLVM_UNLIKELY(typeId == TypeId::Zeroes || typeId == TypeId::Ones)) {
                for(;;) {
                    bool done = true;
                    for (auto e : make_iterator_range(out_edges(u, G))) {
                        const auto v = target(e, G);
                        const auto targetTypeId = getType(v);
                        if (LLVM_UNLIKELY(isAssociative(targetTypeId))) {

                            errs() << " -- identity relationship\n";

                            if (isIdentityRelation(typeId, targetTypeId)) {
                                remove_edge(e, G);
                            } else {
                                setType(v, typeId == TypeId::And ? TypeId::Zeroes : TypeId::Ones);
                                clear_in_edges(v, G);
                            }
                            done = false;
                            modified = Structural;
                            break;
                        }
                    }
                    if (done) {
                        break;
                    }
                }
            } else if (isAssociative(typeId)) {
                if (LLVM_UNLIKELY(in_degree(u, G) == 0)) {
                    setType(u, TypeId::Zeroes);
                } else {
                    // An associative operation with only one element is always equivalent to the element
                    bool contractable = true;
                    if (LLVM_LIKELY(in_degree(u, G) > 1)) {
                        for (auto e : make_iterator_range(out_edges(u, G))) {
                            if (LLVM_LIKELY(typeId != getType(target(e, G)))) {
                                contractable = false;
                                break;
                            }
                        }
                    }
                    if (LLVM_UNLIKELY(contractable)) {
                        for (auto ei : make_iterator_range(in_edges(u, G))) {
                            for (auto ej : make_iterator_range(out_edges(u, G))) {
                                addEdge(source(ei, G), target(ej, G), G[ei]);
                            }
                        }
                        removeVertex(u);
                        modified = std::max(modified, Trivial);
                        continue;
                    }

                    if (LLVM_UNLIKELY(typeId == TypeId::Xor)) {
                        // TODO:: (A ⊕ ¬B) = (A ⊕ (B ⊕ 1)) = ¬(A ⊕ B)

                    }



                }
            }

            assert (getType(u) != TypeId::Var);

            const auto f = V.insert(u);
            if (LLVM_UNLIKELY(!f.second)) {
                const auto v = *f.first;

                errs() << " -- replacing " << u << " with " << v << "\n";

                for (auto e : make_iterator_range(out_edges(u, G))) {
                    addEdge(v, target(e, G), G[e]);
                }
                removeVertex(u);
                modified = Structural;
            }
        }
        return modified;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief applyAbsorbtionComplementIdempotentLaws
     ** ------------------------------------------------------------------------------------------------------------- */
    Modification applyAbsorbtionComplementIdempotentLaws() {
        Modification modified = None;
        for (const Vertex u : make_iterator_range(vertices(G))) {
            const TypeId typeId = getType(u);
            if (isDistributive(typeId)) {
restart_loop:   in_edge_iterator ei_begin, ei_end;
                std::tie(ei_begin, ei_end) = in_edges(u, G);
                for (auto ei = ei_begin; ei != ei_end; ++ei) {
                    const auto v = source(*ei, G);
                    const auto innerTypeId = getType(v);
                    if (isDistributive(innerTypeId) || innerTypeId == TypeId::Not) {
                        in_edge_iterator ek_begin, ek_end;
                        std::tie(ek_begin, ek_end) = in_edges(v, G);
                        for (auto ej = ei_begin; ej != ei_end; ++ej) {
                            for (auto ek = ek_begin; ek != ek_end; ++ek) {
                                if (LLVM_UNLIKELY(source(*ej, G) == source(*ek, G))) {
                                    modified = Structural;
                                    if (LLVM_UNLIKELY(innerTypeId == TypeId::Not)) {
                                        // complement
                                        setType(u, typeId == TypeId::And ? TypeId::Zeroes : TypeId::Ones);
                                        clear_in_edges(u, G);
                                        goto abort_loop;
                                    } else {
                                        if (LLVM_LIKELY(innerTypeId != typeId)) {
                                            // idempotent
                                            remove_edge(*ei, G);
                                        } else {
                                            // absorbtion
                                            remove_edge(*ej, G);
                                        }                                        
                                        // this seldom occurs so if it does, just restart the process rather than
                                        // trying to keep the iterators valid.
                                        goto restart_loop;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            abort_loop:;
        }
        return modified;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief identifyDistributableVertices
     *
     * Let (u) ∈ V(G) be a conjunction ∧ or disjunction ∨ and (v) be a ∧ or ∨ and the opposite type of (u). If (u,v) ∈
     * E(G) and all outgoing edges of (v) lead to a vertex of the same type, add (u), (v) and any vertex (w) in which
     * (w,v) ∈ E(G) to the distribution graph H as well as the edges indicating their relationships within G.
     *
     *                  (?) (?) (?) <-- w1, w2, ...
     *                     \ | /
     *                      (v)   <-- v
     *                     /   \
     *            u --> (∧)     (∧)
     *
     ** ------------------------------------------------------------------------------------------------------------- */
    void identifyDistributableVertices() {

        assert (D.empty() && L.empty());

        for (const Vertex u : make_iterator_range(vertices(G))) {
            const TypeId outerTypeId = getType(u);
            if (isDistributive(outerTypeId)) {
                bool beneficial = true;
                const TypeId innerTypeId = oppositeTypeId(outerTypeId);
                for (auto e : make_iterator_range(out_edges(u, G))) {
                    const Vertex v = target(e, G);
                    if (LLVM_UNLIKELY(getType(v) != innerTypeId)) {
                        beneficial = false;
                        break;
                    }
                }
                if (beneficial) {
                    for (auto e : make_iterator_range(out_edges(u, G))) {
                        const auto v = target(e, G);
                        const auto f = std::lower_bound(D.begin(), D.end(), v);
                        if (f == D.end() || *f != v) {
                            D.insert(f, v);
                            assert (std::is_sorted(D.begin(), D.end()));
                        }
                    }
                    for (auto e : make_iterator_range(in_edges(u, G))) {
                        const auto v = source(e, G);
                        const auto f = std::lower_bound(L.begin(), L.end(), v);
                        if (f == L.end() || *f != v) {
                            L.insert(f, v);
                            assert (std::is_sorted(L.begin(), L.end()));
                        }
                    }
                }
            }
        }

        // D = D - L

        if (!L.empty()) {
            if (!D.empty()) {
                auto di = D.begin(), li = L.begin(), li_end = L.end();
                for (;;) {
                    if (*li < *di) {
                        if (++li == li_end) {
                            break;
                        }
                    } else {
                        if (*di < *li) {
                            ++di;
                        } else {
                            di = D.erase(di);
                        }
                        if (di == D.end()) {
                            break;
                        }
                    }
                }
            }
            L.clear();
        }
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief applyDistributivityLaw
     ** ------------------------------------------------------------------------------------------------------------- */
    Modification applyDistributivityLaw() {

        identifyDistributableVertices();

        // If we found no potential opportunities then we cannot apply the distribution law to any part of G.
        if (D.empty()) {
            return None;
        }

        Modification modified = None;

        const auto lowerSet = independentCliqueSets<1>(removeUnhelpfulBicliques(enumerateBicliques(D)), 1);

        for (auto & lower : lowerSet) {
            const auto upperSet = independentCliqueSets<0>(enumerateBicliques(std::get<1>(lower)), 2);
            for (const auto & upper : upperSet) {

                const auto & sources = std::get<1>(upper);
                const auto & intermediary = std::get<0>(upper);
                const auto & sinks = std::get<0>(lower);



                const auto outerTypeId = getType(sinks.front());
                const auto innerTypeId = oppositeTypeId(outerTypeId);

                errs() << " -- distributing\n";

                // Update G to match the desired change
                const auto x = makeVertex(outerTypeId);
                const auto y = makeVertex(innerTypeId);

                for (const auto i : intermediary) {
                    assert (getType(i) == innerTypeId);
                    for (const Vertex t : sinks) {
                        assert (getType(t) == outerTypeId);
                        remove_edge(i, t, G);
                    }
                    addEdge(i, x);
                }

                for (const Vertex s : sources) {
                    for (const Vertex i : intermediary) {
                        remove_edge(s, i, G);
                    }
                    addEdge(s, y);
                }
                addEdge(x, y);

                for (const Vertex t : sinks) {
                    addEdge(y, t, std::get<0>(G[t]));
                }

                modified = Structural;
            }
        }

        D.clear();

        return modified;
    }


    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief enumerateBicliques
     *
     * Adaptation of the MICA algorithm as described in "Consensus algorithms for the generation of all maximal
     * bicliques" by Alexe et. al. (2003). Note: this implementation considers all verticies in set A to be in
     * bipartition A and their adjacencies to be in B.
      ** ------------------------------------------------------------------------------------------------------------- */

    BicliqueSet enumerateBicliques(const VertexSet & A) {
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
    BicliqueSet && independentCliqueSets(BicliqueSet && cliques, const unsigned minimum) {


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
                    boost::add_edge(i, j, I);
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
    BicliqueSet && removeUnhelpfulBicliques(BicliqueSet && cliques) {
        for (auto ci = cliques.begin(); ci != cliques.end(); ) {
            const auto cardinalityA = std::get<0>(*ci).size();
            VertexSet & B = std::get<1>(*ci);
            for (auto bi = B.begin(); bi != B.end(); ) {
                if (out_degree(*bi, G) == cardinalityA) {
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
     * @brief makeVertex
     ** ------------------------------------------------------------------------------------------------------------- */
    Vertex makeVertex(const TypeId typeId, PabloAST * const expr = nullptr) {
        return add_vertex(std::make_pair(expr, typeId), G);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief addExpression
     ** ------------------------------------------------------------------------------------------------------------- */
    Vertex addExpression(PabloAST * const expr) {
        const auto f = M.find(expr);
        if (LLVM_LIKELY(f != M.end())) {
            return f->second;
        }
        TypeId typeId = TypeId::Var;
        if (isa<Zeroes>(expr)) {
            typeId = TypeId::Zeroes;
        } else if (isa<Ones>(expr)) {
            typeId = TypeId::Ones;
        }
        const auto u = makeVertex(typeId, expr);
        M.emplace(expr, u);
        if (LLVM_UNLIKELY(isa<Not>(expr))) {
            PabloAST * const negated = cast<Not>(expr)->getExpr();
            addEdge(addExpression(negated), u, negated);
        }
        return u;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief addStatement
     ** ------------------------------------------------------------------------------------------------------------- */
    Vertex addStatement(Statement * const stmt) {
        assert (M.count(stmt) == 0);
        const auto typeId = stmt->getClassTypeId();
        if (LLVM_UNLIKELY(typeId == TypeId::Sel)) {

            // expand Sel (C,T,F) statements into (C ∧ T) ∨ (C ∧ F)

            const auto c = addExpression(cast<Sel>(stmt)->getCondition());
            const auto t = addExpression(cast<Sel>(stmt)->getTrueExpr());
            const auto l = makeVertex(TypeId::And);
            addEdge(c, l);
            addEdge(t, l);
            const auto n = makeVertex(TypeId::Not);
            addEdge(c, n);
            const auto r = makeVertex(TypeId::And);
            const auto f = addExpression(cast<Sel>(stmt)->getFalseExpr());
            addEdge(n, r);
            addEdge(f, r);
            const auto u = makeVertex(TypeId::Or, stmt);
            M.emplace(stmt, u);
            addEdge(l, u);
            addEdge(r, u);

            return u;

        } else {

            const auto u = makeVertex(typeId, stmt);
            M.emplace(stmt, u);
            for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                PabloAST * const op = stmt->getOperand(i);
                if (LLVM_UNLIKELY(isa<String>(op))) {
                    continue;
                }
                addEdge(addExpression(op), u, op);
            }

            return u;
        }

    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief addBranch
     ** ------------------------------------------------------------------------------------------------------------- */
    void addBranch(Statement * const br) {
        const auto u = addStatement(br);
        for (auto escaped : cast<Branch>(br)->getEscaped()) {
            addEdge(u, addExpression(escaped), escaped);
        }
    }


    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief addEdge
     ** ------------------------------------------------------------------------------------------------------------- */
    void addEdge(const Vertex u, const Vertex v, PabloAST * const value = nullptr) {
        const auto typeId = getType(v);
        if (isAssociative(typeId)) {
            for (auto e : make_iterator_range(in_edges(u, G))) {
                if (LLVM_UNLIKELY(source(e, G) == u)) {
                    if (LLVM_LIKELY(isDistributive(typeId))) {
                        G[e] = std::max(G[e], value);
                    } else {
                        remove_edge(e, G);
                    }
                    return;
                }
            }
        }
        boost::add_edge(u, v, value, G);
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief removeVertex
     ** ------------------------------------------------------------------------------------------------------------- */
    void removeVertex(const Vertex u) {
        clear_vertex(u, G);
        setType(u, TypeId::Var);
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

    TypeId getType(const Vertex u) {
        return std::get<1>(G[u]);
    }

    void setType(const Vertex u, const TypeId typeId) {
        std::get<1>(G[u]) = typeId;
    }

    static bool isIdentityRelation(const TypeId a, const TypeId b) {
        return !((a == TypeId::Zeroes) ^ (b == TypeId::Or));
    }

    static bool isAssociative(const TypeId typeId) {
        return (isDistributive(typeId) || typeId == TypeId::Xor);
    }

    static bool isDistributive(const TypeId typeId) {
        return (typeId == TypeId::And || typeId == TypeId::Or);
    }

    static bool isImmutable(const TypeId typeId) {
        return (typeId == TypeId::Var || typeId == TypeId::Assign || typeId == TypeId::Extract);
    }

    static TypeId oppositeTypeId(const TypeId typeId) {
        assert (isDistributive(typeId));
        return (typeId == TypeId::And) ? TypeId::Or : TypeId::And;
    }

private:

    Graph G;
    flat_map<pablo::PabloAST *, Vertex> M;
    VertexSet D;
    VertexSet L;

};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool DistributivePass::optimize(PabloKernel * const kernel) {
    #ifdef NDEBUG
    report_fatal_error("DistributivePass is unsupported");
    #endif
    PassContainer C;
    C.run(kernel);
    return true;
}

}
