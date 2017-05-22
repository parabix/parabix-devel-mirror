#include "distributivepass.h"

#include <pablo/pablo_kernel.h>
#include <pablo/codegenstate.h>
#include <pablo/branch.h>
#include <pablo/pe_string.h>
#include <pablo/pe_integer.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <pablo/boolean.h>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/function_output_iterator.hpp>

#include <boost/graph/strong_components.hpp>
#include <llvm/Support/raw_ostream.h>

using namespace boost;
using namespace boost::container;
using namespace llvm;

using TypeId = pablo::PabloAST::ClassTypeId;
using VertexData = std::pair<pablo::PabloAST *, TypeId>;

using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, VertexData, pablo::PabloAST *>;
using Vertex = Graph::vertex_descriptor;

using DistributionGraph = adjacency_list<hash_setS, vecS, bidirectionalS, Vertex>;
using DistributionVertex = DistributionGraph::vertex_descriptor;

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
    void run(PabloBlock * const block) {
        Statement * stmt = block->front();
        // Statement * first = stmt;
        while (stmt) {
            Statement * next = stmt->getNextNode();
            if (isa<Branch>(stmt)) {
                // addUsageInformation();
                if (simplifyGraph()) {
                    // rewriteAST(first, stmt);

                    // printGraph(G, "G", errs());
                }



                G.clear();
                M.clear();
                run(cast<Branch>(stmt)->getBody());
                G.clear();
                M.clear();
            } else {
                addStatement(stmt);
            }
            stmt = next;
        }
    }

protected:

//    /** ------------------------------------------------------------------------------------------------------------- *
//     * @brief addUsageInformation
//     *
//     * Populate G with the user information of each statement so that we can determine whether it'll be cost effective
//     * to distribute an operation.
//     ** ------------------------------------------------------------------------------------------------------------- */
//    void addUsageInformation() {
//        for (const Vertex u : make_iterator_range(vertices(G))) {
//            PabloAST * expr; TypeId typeId;
//            std::tie(expr, typeId) = G[u];
//            if (LLVM_LIKELY(typeId != TypeId::Var)) {
//                for (PabloAST * user : expr->users()) {
//                    add_edge(u, addExpression(user), user, G);
//                }
//            }
//        }
//    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief applyAssociativeIdentityAnnulmentLaws
     ** ------------------------------------------------------------------------------------------------------------- */
    bool applyAssociativeIdentityAnnulmentLaws() {

        bool modified = false;

        std::function<void(const Vertex)> apply = [&](const Vertex u) {
            PabloAST * expr; TypeId typeId;
            std::tie(expr, typeId) = G[u];



            if (LLVM_UNLIKELY(typeId == TypeId::Zeroes || typeId == TypeId::Ones)) {
repeat:         for (auto e : make_iterator_range(out_edges(u, G))) {
                    const auto v = target(e, G);
                    const auto targetTypeId = getType(v);
                    if (LLVM_UNLIKELY(isAssociative(targetTypeId))) {
                        if (isIdentityRelation(typeId, targetTypeId)) {
                            remove_edge(e, G);
                        } else {
                            setType(v, typeId == TypeId::And ? TypeId::Zeroes : TypeId::Ones);
                            clear_in_edges(v, G);
                            apply(v);
                        }
                        modified = true;
                        goto repeat;
                    }
                }
            } else if (isAssociative(typeId)) {

                bool contractable = true;
                // an associative operation with only one element is always equivalent to the element
                if (LLVM_LIKELY(in_degree(u, G) != 1)) {
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
                            add_edge(source(ei, G), target(ej, G), expr, G);
                        }
                    }
                    clear_vertex(u, G);
                    setType(u, TypeId::Var);
                    modified = true;
                }
            }
        };

        // note: calls "apply" on each vertex in reverse topological order
        topological_sort(G, boost::make_function_output_iterator(apply));

        return modified;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief applyAbsorbtionComplementIdempotentLaws
     ** ------------------------------------------------------------------------------------------------------------- */
    bool applyAbsorbtionComplementIdempotentLaws() {
        using in_edge_iterator = graph_traits<Graph>::in_edge_iterator;
        bool modified = false;
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
                                        modified = true;
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
     * @brief contractGraph
     *
     * This function scans through a scope block and computes a DAG G in which any sequences of AND, OR or XOR functions
     * are "flattened" (i.e., allowed to have any number of inputs.)
     ** ------------------------------------------------------------------------------------------------------------- */
    bool contractGraph() {
        bool modified = false;
        bool alreadyGoneOnce = false;
        for (;;) {
            if (applyAssociativeIdentityAnnulmentLaws()) {
                modified = true;
            } else if (alreadyGoneOnce) {
                break;
            }
            if (applyAbsorbtionComplementIdempotentLaws()) {
                modified = true;
            } else { // if (alreadyGoneOnce) {
                break;
            }
            alreadyGoneOnce = true;
        }
        return modified;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief addVertex
     ** ------------------------------------------------------------------------------------------------------------- */
    DistributionVertex addVertex(const Vertex u) {
        const auto f = D.find(u);
        if (LLVM_LIKELY(f != D.end())) {
            return f->second;
        }
        const auto v = add_vertex(u, H);
        D.emplace(u, v);
        return v;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief generateDistributionGraph
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
    void generateDistributionGraph() {

        assert (D.empty());

        flat_set<Vertex> distributable;
        flat_set<Vertex> observed;

        for (const Vertex u : make_iterator_range(vertices(G))) {
            const TypeId outerTypeId = getType(u);
            if (isDistributive(outerTypeId)) {
                const TypeId innerTypeId = oppositeTypeId(outerTypeId);
                for (auto e : make_iterator_range(in_edges(u, G))) {
                    const Vertex v = source(e, G);
                    if (LLVM_UNLIKELY(std::get<1>(G[v]) == innerTypeId)) {
                        bool beneficial = true;
                        for (const auto e : make_iterator_range(out_edges(v, G))) {
                            if (std::get<1>(G[target(e, G)]) != outerTypeId) {
                                beneficial = false;
                                break;
                            }
                        }
                        if (beneficial) {
                            distributable.insert(v);
                        }
                    }
                }
                if (LLVM_LIKELY(distributable.size() > 1)) {
                    for (const Vertex v : distributable) {
                        for (auto e : make_iterator_range(in_edges(v, G))) {
                            observed.insert(source(e, G));
                        }
                    }
                    for (const Vertex w : observed) {
                        for (auto e : make_iterator_range(out_edges(w, G))) {
                            const Vertex v = target(e, G);
                            if (distributable.count(v)) {
                                const Vertex y = addVertex(v);
                                boost::add_edge(y, addVertex(u), H);
                                boost::add_edge(addVertex(w), y, H);
                            }
                        }
                    }
                    observed.clear();
                }
                distributable.clear();
            }
        }

        D.clear();
    }


    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief redistributeAST
     *
     * Apply the distribution law to reduce computations whenever possible.
     ** ------------------------------------------------------------------------------------------------------------- */
    bool simplifyGraph() {

        VertexSet sinks;

        bool modified = false;

        for (;;) {

            assert (num_vertices(H) == 0 && num_edges(H) == 0);

            if (contractGraph()) {
                modified = true;
            }

            generateDistributionGraph();

            // If we found no potential opportunities then we cannot apply the distribution law to any part of G.
            if (num_vertices(H) == 0) {
                break;
            }

            for (const Vertex u : make_iterator_range(vertices(H))) {
                if (out_degree(u, H) == 0 && in_degree(u, H) != 0) {
                    sinks.push_back(u);
                }
            }
            std::sort(sinks.begin(), sinks.end());

            bool done = true;

            const auto lowerSet = independentCliqueSets<1>(removeUnhelpfulBicliques(enumerateBicliques(sinks)), 1);

            for (auto & lower : lowerSet) {
                const auto upperSet = independentCliqueSets<0>(enumerateBicliques(std::get<1>(lower)), 2);
                for (const auto & upper : upperSet) {

                    const auto & sources = std::get<1>(upper);
                    const auto & intermediary = std::get<0>(upper);
                    const auto & sinks = std::get<0>(lower);

                    const auto outerTypeId = getType(H[sinks.front()]);
                    const auto innerTypeId = oppositeTypeId(outerTypeId);

                    // Update G to match the desired change
                    const auto x = makeVertex(outerTypeId);
                    const auto y = makeVertex(innerTypeId);

                    for (const auto i : intermediary) {
                        const auto u = H[i];
                        assert (getType(u) == innerTypeId);
                        for (const Vertex t : sinks) {
                            assert (getType(H[t]) == outerTypeId);
                            remove_edge(u, H[t], G);
                        }
                        add_edge(u, x, nullptr, G);
                    }

                    for (const Vertex s : sources) {
                        const auto u = H[s];
                        for (const Vertex i : intermediary) {
                            remove_edge(u, H[i], G);
                        }
                        add_edge(u, y, nullptr, G);
                    }
                    add_edge(x, y, nullptr, G);

                    for (const Vertex t : sinks) {
                        const auto v = H[t];
                        add_edge(y, v, std::get<0>(G[v]), G);
                    }

                    done = false;
                }
            }

            H.clear();

            if (done) {
                break;
            } else {
                sinks.clear();
                modified = true;
            }
        }

        assert (num_vertices(H) == 0 && num_edges(H) == 0);

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
            if (in_degree(u, H) > 0) {
                VertexSet incomingAdjacencies;
                incomingAdjacencies.reserve(in_degree(u, H));
                for (auto e : make_iterator_range(in_edges(u, H))) {
                    incomingAdjacencies.push_back(source(e, H));
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
                Aj.reserve(out_degree(u, H));
                for (auto e : make_iterator_range(out_edges(u, H))) {
                    Aj.push_back(target(e, H));
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
     * @brief addTemporary
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
        return u;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief addStatement
     ** ------------------------------------------------------------------------------------------------------------- */
    void addStatement(Statement * const stmt) {
        assert (M.count(stmt) == 0);
        const auto typeId = stmt->getClassTypeId();
        const auto u = makeVertex(typeId, stmt);
        M.emplace(stmt, u);
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            PabloAST * const op = stmt->getOperand(i);
            if (LLVM_UNLIKELY(isa<Integer>(op) || isa<String>(op))) {
                continue;
            }
            const auto v = addExpression(op);
            add_edge(v, u, op, G);
            if (LLVM_UNLIKELY(isa<Not>(op))) {
                PabloAST * const negated = cast<Not>(op)->getExpr();
                add_edge(addExpression(negated), v, negated, G);
            }
        }
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

    static TypeId oppositeTypeId(const TypeId typeId) {
        assert (isDistributive(typeId));
        return (typeId == TypeId::And) ? TypeId::Or : TypeId::And;
    }

    void add_edge(const Vertex u, const Vertex v, PabloAST * const value, Graph & G) {
        G[std::get<0>(boost::add_edge(u, v, G))] = value;
    }

private:

    Graph G;
    flat_map<pablo::PabloAST *, Vertex> M;

    DistributionGraph H;
    flat_map<Vertex, DistributionVertex> D;

};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool DistributivePass::optimize(PabloKernel * const kernel) {
    PassContainer C;
    C.run(kernel->getEntryBlock());
    return true;
}

}
