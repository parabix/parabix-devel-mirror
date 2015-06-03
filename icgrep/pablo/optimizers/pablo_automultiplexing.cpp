#include "pablo_automultiplexing.hpp"
#include <pablo/codegenstate.h>
#include <llvm/ADT/SmallVector.h>
#include <llvm/ADT/DenseMap.h>
#include <llvm/ADT/DenseSet.h>
#include <llvm/ADT/BitVector.h>
#include <stack>
#include <queue>
#include <unordered_set>
#include <boost/container/flat_set.hpp>
#include <unordered_map>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/circular_buffer.hpp>
#include <include/simd-lib/builtins.hpp>
// #include <pablo/expression_map.hpp>
#include <pablo/printer_pablos.h>
#include <iostream>
#include <cudd.h>
#include <util.h>

using namespace llvm;
using namespace boost;
using namespace boost::container;
using namespace boost::numeric::ublas;

namespace pablo {

void AutoMultiplexing::optimize(const std::vector<Var *> & input, PabloBlock & entry) {
    AutoMultiplexing am;
    am.initialize(input, entry);
    am.characterize(entry);
    am.shutdown();
    am.createMultiplexSetGraph();
    std::random_device rd;
    RNG rng(rd());
    if (am.generateMultiplexSets(rng)) {
        am.approxMaxWeightIndependentSet(rng);
        am.applySubsetConstraints();
        am.multiplexSelectedIndependentSets();
        am.topologicalSort(entry);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initialize
 * @param vars the input vars for this program
 * @param entry the entry block
 *
 * Scan through the program to identify any advances and calls then initialize the BDD engine with
 * the proper variable ordering.
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::initialize(const std::vector<Var *> & vars, const PabloBlock & entry) {

    flat_map<const PabloAST *, unsigned> map;
    std::vector<const Statement *> complex;
    std::stack<const Statement *> scope;

    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    unsigned n = 0, m = 0;
    for (const Statement * stmt = entry.front(); ; ) {
        while ( stmt ) {
            ++n;
            if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                // Set the next statement to be the first statement of the inner scope and push the
                // next statement of the current statement into the scope stack.
                const PabloBlock & nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                scope.push(stmt->getNextNode());
                stmt = nested.front();
                assert (stmt);
                continue;
            }

            assert ("Run the Simplifer pass prior to this!" && (stmt->getNumUses() != 0 || (isa<Assign>(stmt) ? !cast<Assign>(stmt)->superfluous() : false)));

            switch (stmt->getClassTypeId()) {
                case PabloAST::ClassTypeId::Advance:                    
                    mAdvance.push_back(const_cast<Advance*>(cast<Advance>(stmt)));
                    map.emplace(stmt, m++);
                case PabloAST::ClassTypeId::Call:
                case PabloAST::ClassTypeId::ScanThru:
                case PabloAST::ClassTypeId::MatchStar:
                    complex.push_back(stmt);
                    break;
                default:
                    break;
            }
            stmt = stmt->getNextNode();
        }
        if (scope.empty()) {
            break;
        }
        stmt = scope.top();
        scope.pop();
    }

    // Create the transitive closure matrix of graph. From this we'll construct
    // two graphs restricted to the relationships between advances. The first will
    // be a path graph, which is used to bypass testing for mutual exclusivity of
    // advances that cannot be multiplexed. The second is a transitive reduction
    // of that graph, which forms the basis of our constraint graph when deciding
    // which advances ought to be multiplexed.

    matrix<bool> G(n, m); // Let G be a matrix with n rows of m (Advance) elements
    G.clear();
    for (unsigned i = 0; i != m; ++i) {
        G(i, i) = true;
    }

    n = m;
    m = 0;

    const Statement * stmt = entry.front();
    for (;;) {
        while ( stmt ) {

            unsigned u;
            if (isa<Advance>(stmt)) {
                assert (mAdvance[m] == stmt);
                u = m++;
            }
            else {
                u = n++;
                map.emplace(stmt, u);
            }

            for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                const PabloAST * const op = stmt->getOperand(i);
                if (LLVM_LIKELY(isa<Statement>(op))) {
                    const unsigned v = map[op];
                    for (unsigned w = 0; w != m; ++w) {
                        G(u, w) |= G(v, w);
                    }
                }
            }
            if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                // Set the next statement to be the first statement of the inner scope
                // and push the next statement of the current statement into the stack.
                const PabloBlock & nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                scope.push(stmt->getNextNode());
                stmt = nested.front();
                assert (stmt);
                continue;
            }
            stmt = stmt->getNextNode();
        }
        if (scope.empty()) {
            break;
        }
        stmt = scope.top();
        scope.pop();
    }

    // Record our transitive closure in the path graph and remove any reflexive-loops
    mPathGraph = PathGraph(m);
    for (unsigned i = 0; i != m; ++i) {
        G(i, i) = false;
        for (unsigned j = 0; j != m; ++j) {
            if (G(i, j)) {
                add_edge(i, j, mPathGraph);
            }
        }        
    }

    // Now take the transitive reduction of G
    for (unsigned j = 0; j != m; ++j) {
        for (unsigned i = 0; i != m; ++i) {
            if (G(i, j)) {
                // Eliminate any unecessary arcs not covered by a longer path
                for (unsigned k = 0; k != m; ++k) {
                    G(i, k) = G(j, k) ? false : G(i, k);
                }
            }
        }
    }

    // And now transcribe it into our base constraint graph. Since G is a use-def
    // graph and we want our constraint graph to be a def-use graph, reverse the
    // edges as we're writing them to obtain the transposed graph.
    mConstraintGraph = ConstraintGraph(m);
    for (unsigned j = 0; j != m; ++j) {
        for (unsigned i = 0; i != m; ++i) {
            if (G(i, j)) {
                add_edge(j, i, mConstraintGraph);
            }
        }
    }

    // Initialize the BDD engine ...
    mManager = Cudd_Init((complex.size() + vars.size()), 0, CUDD_UNIQUE_SLOTS, CUDD_CACHE_SLOTS, 0);

    // Map the predefined 0/1 entries
    mCharacterizationMap.emplace(entry.createZeroes(), Cudd_ReadZero(mManager));
    mCharacterizationMap.emplace(entry.createOnes(), Cudd_ReadOne(mManager));

    // Order the variables so the input Vars are pushed to the end; they ought to
    // be the most complex to resolve.
    unsigned i = 0;
    for (const Statement * stmt : complex) {
        mCharacterizationMap.emplace(stmt, Cudd_bddIthVar(mManager, i++));
    }
    for (const Var * var : vars) {
        mCharacterizationMap.emplace(var, Cudd_bddIthVar(mManager, i++));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 * @param vars the input vars for this program
 *
 * Scan through the program and iteratively compute the BDDs for each statement.
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::characterize(PabloBlock & entry) {

    std::vector<std::pair<DdNode *, DdNode *>> advances; // the input BDD and the BDD variable of the i-th Advance
    std::stack<Statement *> scope;

    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    for (Statement * stmt = entry.front(); ; ) {
        while ( stmt ) {

            if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                // Set the next statement to be the first statement of the inner scope and push the
                // next statement of the current statement into the scope stack.
                const PabloBlock & nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                scope.push(stmt->getNextNode());
                stmt = nested.front();
                assert (stmt);
                continue;
            }

            DdNode * bdd = nullptr;

            // Map our operands to the computed BDDs
            std::array<DdNode *, 3> input;
            for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                PabloAST * const op = stmt->getOperand(i);
                if (LLVM_UNLIKELY(isa<Integer>(op) || isa<String>(op))) {
                    continue;
                }
                auto f = mCharacterizationMap.find(op);
                if (LLVM_UNLIKELY(f == mCharacterizationMap.end())) {
                    throw std::runtime_error("Uncharacterized operand " + std::to_string(i) + " of " + stmt->getName()->to_string());
                }
                input[i] = f->second;
            }

            bool testContradiction = true;

            switch (stmt->getClassTypeId()) {
                case PabloAST::ClassTypeId::Assign:
                    bdd = input[0];
                    break;
                case PabloAST::ClassTypeId::And:
                    bdd = And(input[0], input[1]);
                    break;
                case PabloAST::ClassTypeId::Next:
                    // The next instruction is almost identical to an OR; however, a Next cannot be
                    // an operand of a Statement. Instead it updates the Initial operand's value.
                    testContradiction = false;
                case PabloAST::ClassTypeId::Or:
                    bdd = Or(input[0], input[1]);
                    break;
                case PabloAST::ClassTypeId::Xor:
                    bdd = Xor(input[0], input[1]);
                    break;
                case PabloAST::ClassTypeId::Not:
                    bdd = Not(input[0]);
                    break;
                case PabloAST::ClassTypeId::Sel:
                    bdd = Ite(input[0], input[1], input[2]);
                    break;
                case PabloAST::ClassTypeId::ScanThru:
                    // It may be possible use "mEngine.applyNot(input[1])" for ScanThrus if we rule out the
                    // possibility of a contradition being erroneously calculated later. An example of this
                    // would be ¬(ScanThru(c,m) ∨ m) but are there others that would be harder to predict?
                case PabloAST::ClassTypeId::MatchStar:
                    if (LLVM_UNLIKELY(isZero(input[0]) || isZero(input[1]))) {
                        bdd = Cudd_ReadZero(mManager);
                        break;
                    }
                case PabloAST::ClassTypeId::Call:
                    testContradiction = false;
                    assert (mCharacterizationMap.count(stmt));
                    bdd = mCharacterizationMap[stmt];
                    break;
                case PabloAST::ClassTypeId::Advance:

                    assert (stmt == mAdvance[advances.size()]);

                    if (LLVM_UNLIKELY(isZero(input[0]))) {
                        bdd = Cudd_ReadZero(mManager);
                        // mark this advance as having an input and output of 0
                        advances.emplace_back(bdd, bdd);
                    }
                    else {

                        // When we built the path graph, we constructed it in the same order; hense the row/column of
                        // the path graph is equivalent to the index.

                        assert (mCharacterizationMap.count(stmt));
                        DdNode * const Ik = input[0];
                        DdNode * const Vk = bdd = mCharacterizationMap[stmt];
                        const unsigned k = advances.size();

                        for (unsigned i = 0; i != k; ++i) {

                            Advance * adv = mAdvance[i];
                            DdNode * Ii = std::get<0>(advances[i]);
                            DdNode * Vi = std::get<1>(advances[i]);

                            bool constrained = true;

                            // If these advances are "shifting" their values by the same amount and aren't transitively dependant ...
                            if ((stmt->getOperand(1) == adv->getOperand(1)) && (!edge(k, i, mPathGraph).second)) {

                                // Test this with Cudd_bddIntersect(mManager, Ik, Ii);
                                DdNode * const tmp = And(Ik, Ii); // do we need to simplify this to identify subsets?

                                if (Ik == tmp) {
                                    // If Ik = C and C = Ii ∩ Ik then Ik (the input to the k-th advance) is a *subset* of Ii (the input of the
                                    // i-th advance). Record this in the subset graph with the arc (k,i). These edges will be moved into the
                                    // multiplex set graph if Advance_i and Advance_k happen to be in the same mutually exclusive set.
                                    mSubsetGraph.emplace_back(k, i);
                                    constrained = false;
                                }
                                else if (Ii == tmp) {
                                    mSubsetGraph.emplace_back(i, k);
                                    constrained = false;
                                }
                                // Is there any satisfying truth assignment? If not, these streams are mutually exclusive.
                                else if (noSatisfyingAssignment(tmp)) {

                                    assert (mCharacterizationMap.count(adv));

                                    DdNode *& other = mCharacterizationMap[adv];
                                    // mark the i-th and k-th Advances as being mutually exclusive
                                    bdd = And(bdd, Not(Vi));
                                    other = And(other, Not(Vk));
                                    constrained = false;
                                }

                                Cudd_RecursiveDeref(mManager, tmp);
                            }

                            // Advances must be in the same scope or they cannot be safely multiplexed unless one is moved.
                            if (constrained || (stmt->getParent() != adv->getParent())) {
                                // We want the constraint graph to be acyclic; since the dependencies are already in topological order,
                                // adding an arc from a lesser to greater numbered vertex won't induce a cycle.
                                add_edge(i, k, mConstraintGraph);
                            }

                        }

                        // Append this advance to the list of known advances in this "basic block"; add on the input's BDD to
                        // eliminate the need for looking it up again.
                        advances.emplace_back(Ik, Vk);
                        testContradiction = false;
                    }


                    break;
                default:
                    throw std::runtime_error("Unexpected statement " + stmt->getName()->to_string());
            }
            if (LLVM_UNLIKELY(testContradiction && noSatisfyingAssignment(bdd))) {
                Cudd_RecursiveDeref(mManager, bdd);                
                stmt = stmt->replaceWith(entry.createZeroes());
            }
            else {                
                mCharacterizationMap[stmt] = bdd;
                stmt = stmt->getNextNode();
            }
        }

        if (scope.empty()) {
            break;
        }
        stmt = scope.top();
        scope.pop();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CUDD wrappers
 ** ------------------------------------------------------------------------------------------------------------- */

inline bool AutoMultiplexing::isZero(DdNode * x) const {
    return x == Cudd_ReadZero(mManager);
}

inline DdNode * AutoMultiplexing::And(DdNode * x, DdNode * y) {
    DdNode * r = Cudd_bddAnd(mManager, x, y);
    Cudd_Ref(r);
    return r;
}

inline DdNode * AutoMultiplexing::Or(DdNode * x, DdNode * y) {
    DdNode * r = Cudd_bddOr(mManager, x, y);
    Cudd_Ref(r);
    return r;
}

inline DdNode * AutoMultiplexing::Xor(DdNode * x, DdNode * y) {
    DdNode * r = Cudd_bddXor(mManager, x, y);
    Cudd_Ref(r);
    return r;
}

inline DdNode * AutoMultiplexing::Not(DdNode * x) {
    return Cudd_Not(x);
}

inline DdNode * AutoMultiplexing::Ite(DdNode * x, DdNode * y, DdNode * z) {
    DdNode * r = Cudd_bddIte(mManager, x, y, z);
    Cudd_Ref(r);
    return r;
}

inline bool AutoMultiplexing::noSatisfyingAssignment(DdNode * x) {
    // TODO: since we're only interested in knowing whether there is no such cube,
    // write an optimized function for that if one does not exist.
    int* cube;
    CUDD_VALUE_TYPE dummy;
    auto gen = Cudd_FirstCube(mManager, x, &cube, &dummy);
    bool r = Cudd_IsGenEmpty(gen);
    Cudd_GenFree(gen);
    return r;
}

inline void AutoMultiplexing::shutdown() {
    Cudd_Quit(mManager);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createMappingGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::createMultiplexSetGraph() {
    mMultiplexSetGraph = MultiplexSetGraph(num_vertices(mConstraintGraph));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateMultiplexSets
 * @param RNG random number generator
 ** ------------------------------------------------------------------------------------------------------------- */
bool AutoMultiplexing::generateMultiplexSets(RNG & rng) {

    using Vertex = ConstraintGraph::vertex_descriptor;
    using VertexIterator = graph_traits<ConstraintGraph>::vertex_iterator;
    using EdgeIterator = graph_traits<ConstraintGraph>::out_edge_iterator;
    using DegreeType = graph_traits<ConstraintGraph>::degree_size_type;

    // push all source nodes into the active independent set S
    IndependentSet S;
    unsigned remainingVerticies = num_vertices(mConstraintGraph);
    std::vector<DegreeType> D(remainingVerticies);

    VertexIterator vi, vi_end;
    for (std::tie(vi, vi_end) = vertices(mConstraintGraph); vi != vi_end; ++vi) {
        auto d = in_degree(*vi, mConstraintGraph);
        D[*vi] = d;
        if (d == 0) {
            S.push_back(*vi);
        }
    }

    assert (!S.empty());

    for ( ; remainingVerticies >= 3; --remainingVerticies) {
        if (S.size() >= 3) {
            addMultiplexSet(S);
        }        
        // Randomly choose a vertex in S and discard it.
        assert (!S.empty());
        const auto i = S.begin() + RNGDistribution(0, S.size() - 1)(rng);
        const Vertex u = *i;
        S.erase(i);
        EdgeIterator ei, ei_end;
        for (std::tie(ei, ei_end) = out_edges(u, mConstraintGraph); ei != ei_end; ++ei) {
            const Vertex v = target(*ei, mConstraintGraph);
            if ((--D[v]) == 0) {
                S.push_back(v);
            }
        }
    }

    return num_vertices(mMultiplexSetGraph) > num_vertices(mConstraintGraph);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addMultiplexSet
 * @param S an independent set
 ** ------------------------------------------------------------------------------------------------------------- */
inline void AutoMultiplexing::addMultiplexSet(const IndependentSet & S) {

    // At this stage, the multiplex set graph is a directed bipartite graph that is used to show relationships
    // between the "set vertex" and its members. We obtain these from "generateMultiplexSets".

    // Question: should we enumerate the power set of S?

    const auto v = add_vertex(mMultiplexSetGraph);
    for (auto u : S) {
        add_edge(v, u, mMultiplexSetGraph);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief approxMaxWeightIndependentSet
 * @param RNG random number generator
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::approxMaxWeightIndependentSet(RNG & rng) {

    // Compute our independent set graph from our multiplex set graph; effectively it's the graph resulting
    // from contracting one advance node edge with an adjacent vertex

    const unsigned m = num_vertices(mConstraintGraph);
    const unsigned n = num_vertices(mMultiplexSetGraph) - m;

    IndependentSetGraph G(n);

    std::unordered_map<MultiplexSetGraph::vertex_descriptor, IndependentSetGraph::vertex_descriptor> M;
    MultiplexSetGraph::vertex_descriptor i = m;
    graph_traits<IndependentSetGraph>::vertex_iterator vi, vi_end;
    for (std::tie(vi, vi_end) = vertices(G); vi != vi_end; ++vi, ++i) {
        M.emplace(i, *vi);
        G[*vi] = std::make_pair(out_degree(i, mMultiplexSetGraph), i);
    }

    // Let M be mMultiplexSetGraph. Each vertex in G corresponds to a set vertex in M.
    // If an advance vertex in M (i.e., one of the first m vertices of M) is adjacent
    // to two or more set vertices, add an edge between the corresponding vertices in G.
    // I.e., make all vertices in G adjacent if their corresponding sets intersect.

    for (i = 0; i != m; ++i) {
        if (in_degree(i, mMultiplexSetGraph) > 1) {
            graph_traits<MultiplexSetGraph>::in_edge_iterator ei_begin, ei_end;
            std::tie(ei_begin, ei_end) = in_edges(i, mMultiplexSetGraph);
            for (auto ei = ei_begin; ei != ei_end; ++ei) {
                for (auto ej = ei_begin; ej != ei; ++ej) {
                    // Note: ei and ej are incoming edges. The source is the set vertex,
                    add_edge(M[source(*ei, mMultiplexSetGraph)], M[source(*ej, mMultiplexSetGraph)], G);
                }
            }
        }
    }

    // Using WMIN from "A note on greedy algorithms for the maximum weighted independent set problem"
    // (Note: look into minimum independent dominating set algorithms. It fits better with the ceil log2 + subset cost model.)

    std::vector<IndependentSetGraph::vertex_descriptor> A;

    while (num_vertices(G) > 0) {

        // Select a vertex vi whose weight as greater than the average weight of its neighbours ...

        auto i = RNGDistribution(0, num_vertices(G) - 1)(rng);
        for (std::tie(vi, vi_end) = vertices(G); i && vi != vi_end; ++vi, --i);

        const unsigned Vw = G[*vi].first * (degree(*vi, G) + 1);

        unsigned Nw = 0;
        graph_traits<IndependentSetGraph>::adjacency_iterator ai, ai_end;
        std::tie(ai, ai_end) = adjacent_vertices(*vi, G);
        for (; ai != ai_end; ++ai) {
            Nw += G[*ai].first;
        }

        // Then add it to our set and remove it and its neighbourhood from G

        if (Nw <= Vw) {
            A.reserve(degree(*vi, G) + 1);
            // Note: by clearing the adjacent set vertices from the multiplex set graph, this will effectively
            // choose the set refered to by vi as one of our chosen independent sets.
            graph_traits<IndependentSetGraph>::adjacency_iterator ai, ai_end;
            A.push_back(*vi);
            for (std::tie(ai, ai_end) = adjacent_vertices(*vi, G); ai != ai_end; ++ai) {
                clear_out_edges(G[*ai].second, mMultiplexSetGraph);
                A.push_back(*ai);
            }
            for (auto u : A) {
                clear_vertex(u, G);
                remove_vertex(u, G);
            }
            A.clear();
        }


    }

    #ifndef NDEBUG
    for (unsigned i = 0; i != m; ++i) {
        assert (in_degree(i, mMultiplexSetGraph) <= 1);
    }
    for (unsigned i = m; i != (m + n); ++i) {
        assert (out_degree(i, mMultiplexSetGraph) == 0 || out_degree(i, mMultiplexSetGraph) >= 3);
    }
    #endif
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief choose
 ** ------------------------------------------------------------------------------------------------------------- */

inline Statement * choose(PabloAST * x, PabloAST * y, Statement * z) {
    return isa<Statement>(x) ? cast<Statement>(x) : (isa<Statement>(y) ? cast<Statement>(y) : z);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief applySubsetConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::applySubsetConstraints() {

    // When entering thus function, the multiplex set graph M is a bipartite DAG with edges denoting the set
    // relationships of our independent sets.
    const unsigned m = num_vertices(mConstraintGraph);
    const unsigned n = num_vertices(mMultiplexSetGraph);

    // Add in any edges from our subset constraints to M that are within the same set.
    bool hasSubsetConstraint = false;
    for (const auto & ei : mSubsetGraph) {
        const auto u = ei.first;
        const auto v = ei.second;
        graph_traits<MultiplexSetGraph>::in_edge_iterator ej, ej_end;
        // If both the source and target of ei are adjacent to the same vertex, that vertex must be the
        // "set vertex". Add the edge between the vertices.
        for (std::tie(ej, ej_end) = in_edges(u, mMultiplexSetGraph); ej != ej_end; ++ej) {
            auto w = target(*ej, mMultiplexSetGraph);
            // Only check (v, w) if w is a "set vertex".
            if (w >= (n - m) && edge(v, w, mMultiplexSetGraph).second) {
                add_edge(u, v, mMultiplexSetGraph);
                hasSubsetConstraint = true;
            }
        }
    }

    if (LLVM_UNLIKELY(hasSubsetConstraint)) {
        // At this point, M is still a DAG but no longer bipartite. We're going to scan through the set vertices
        // in M, and use a DFS to scan through M and eliminate any subset relationships in the AST.
        // That way, "multiplexSelectedIndependentSets" only needs to consider muxing and demuxing of the streams.

        std::vector<MultiplexSetGraph::vertex_descriptor> V;
        std::stack<MultiplexSetGraph::vertex_descriptor> S;
        std::queue<PabloAST *> Q;
        BitVector D(n - m, 0);

        for (auto i = m; i != n; ++i) {
            graph_traits<MultiplexSetGraph>::out_edge_iterator ei, ei_end;
            // For each member of a "set vertex" ...
            std::tie(ei, ei_end) = out_edges(i, mMultiplexSetGraph);
            for (; ei != ei_end; ++ei) {
                const auto s = source(*ei, mMultiplexSetGraph);
                if (out_degree(s, mMultiplexSetGraph) != 0) {
                    // First scan through the subgraph of vertices in M dominated by s and build up the set T,
                    // consisting of all sinks w.r.t. vertex s.
                    auto u = s;
                    for (;;) {
                        graph_traits<MultiplexSetGraph>::out_edge_iterator ej, ej_end;
                        for (std::tie(ej, ej_end) = out_edges(u, mMultiplexSetGraph); ej != ej_end; ++ej) {
                            auto v = target(*ej, mMultiplexSetGraph);
                            if (D.test(v)) {
                                continue;
                            }
                            D.set(v);
                            if (out_degree(v, mMultiplexSetGraph) == 0) {
                                V.push_back(v);
                            }
                            else {
                                S.push(v);
                            }
                        }
                        if (S.empty()) {
                            break;
                        }
                        u = S.top();
                        S.pop();
                    }
                    D.clear();
                    // Now in order for these advances to be mutually exclusive, the input to A_s must be masked
                    // with the complement of each advance indicated by V.

                    Advance * adv = mAdvance[s];
                    PabloBlock * pb = adv->getParent();

                    for (auto i : V) {
                        Q.push(mAdvance[i]->getOperand(0));
                    }                    
                    while (Q.size() > 1) {
                        PabloAST * a1 = Q.front(); Q.pop();
                        PabloAST * a2 = Q.front(); Q.pop();
                        pb->setInsertPoint(cast<Statement>(a2));
                        Q.push(pb->createOr(a1, a2));
                    }
                    assert (Q.size() == 1);

                    PabloAST * const mask = pb->createNot(Q.front()); Q.pop();
                    adv->setOperand(0, pb->createAnd(adv->getOperand(0), mask, "subset_in"));

                    // Similar to the above, we're going to OR together the result of each advance,
                    // including s. This will restore the advanced variable back to its original state.

                    // Gather the original users to this advance. We'll be manipulating it shortly.
                    Statement::Users U(adv->users());

                    // Add s to V and sort the list; it'll be closer to being in topological order.
                    V.push_back(s);
                    std::sort(V.begin(), V.end());
                    for (auto i : V) {
                        Q.push(mAdvance[i]);
                    }
                    while (Q.size() > 1) {
                        PabloAST * a1 = Q.front(); Q.pop();
                        PabloAST * a2 = Q.front(); Q.pop();
                        pb->setInsertPoint(choose(a2, a1, adv));
                        Q.push(pb->createOr(a1, a2));
                    }
                    assert (Q.size() == 1);

                    PabloAST * const input = Q.front(); Q.pop();
                    for (PabloAST * use : U) {
                        if (LLVM_LIKELY(isa<Statement>(use))) {
                            cast<Statement>(use)->replaceUsesOfWith(adv, input);
                        }
                    }

                    pb->setInsertPoint(pb->back());

                    V.clear();

                }
            }
        }
    }
}


static inline size_t smallest_multiplexed_set(const size_t x) {
    return std::log2<size_t>(x) + 1; // use a faster builtin function for this?
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief multiplexSelectedIndependentSets
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::multiplexSelectedIndependentSets() const {

    const unsigned f = num_vertices(mConstraintGraph);
    const unsigned l = num_vertices(mMultiplexSetGraph);

    // Preallocate the structures based on the size of the largest multiplex set
    size_t max_n = 3;
    for (unsigned s = f; s != l; ++s) {
        max_n = std::max<unsigned>(max_n, out_degree(s, mMultiplexSetGraph));
    }
    const size_t max_m = smallest_multiplexed_set(max_n);

    std::vector<MultiplexSetGraph::vertex_descriptor> I(max_n);
    std::vector<Advance *> V(max_n);
    std::vector<PabloAST *> muxed(max_m);
    circular_buffer<PabloAST *> Q(max_n);

    // When entering thus function, the multiplex set graph M is a DAG with edges denoting the set
    // relationships of our independent sets.

    for (unsigned s = f; s != l; ++s) {
        const size_t n = out_degree(s, mMultiplexSetGraph);
        if (n) {

            const size_t m = smallest_multiplexed_set(n);

            graph_traits<MultiplexSetGraph>::out_edge_iterator ei, ei_end;
            std::tie(ei, ei_end) = out_edges(s, mMultiplexSetGraph);
            for (unsigned i = 0; i != n; ++ei, ++i) {
                I[i] = target(*ei, mMultiplexSetGraph);
                assert (I[i] < mAdvance.size());
            }
            std::sort(I.begin(), I.begin() + n);

            for (unsigned i = 0; i != n; ++i) {
                V[i] = mAdvance[I[i]];
            }

            PabloBlock * const pb = V[0]->getParent();
            assert (pb);

            // Sanity test to make sure every advance is in the same scope.
            #ifndef NDEBUG
            for (unsigned i = 1; i != n; ++i) {
                assert (I[i - 1] < I[i]);
                assert (V[i - 1] != V[i]);
                assert (V[i]->getParent() == pb);
            }
            #endif

            /// Perform n-to-m Multiplexing
            for (size_t j = 0; j != m; ++j) {

                assert (Q.empty());

                std::stringstream name;

                name << "mux";

                for (size_t i = 1; i <= n; ++i) {
                    if ((i & (static_cast<size_t>(1) << j)) != 0) {
                        assert (!Q.full());
                        PabloAST * tmp = V[i - 1]->getOperand(0); assert (tmp);
                        Q.push_back(tmp);
                        name << "_" << V[i - 1]->getName()->to_string();
                    }
                }

                assert (Q.size() >= 1);

                Advance * const adv = V[j];
                // TODO: figure out a way to determine whether we're creating a duplicate value below.
                // The expression map findOrCall ought to work conceptually but the functors method
                // doesn't really work anymore with the current API.
                while (Q.size() > 1) {
                    PabloAST * a1 = Q.front(); Q.pop_front(); assert (a1);
                    PabloAST * a2 = Q.front(); Q.pop_front(); assert (a2);
                    assert (!Q.full());
                    pb->setInsertPoint(choose(a2, a1, adv));
                    PabloAST * tmp = pb->createOr(a1, a2); assert (tmp);
                    Q.push_back(tmp);
                }
                assert (Q.size() == 1);

                PabloAST * mux = Q.front(); Q.pop_front(); assert (mux);
                muxed[j] = pb->createAdvance(mux, adv->getOperand(1), name.str());
            }


            /// Perform m-to-n Demultiplexing
            // Now construct the demuxed values and replaces all the users of the original advances with them.
            for (size_t i = 1; i <= n; ++i) {

                Advance * const adv = V[i - 1];

                assert (Q.empty());
                for (size_t j = 0; j != m; ++j) {
                    if ((i & (static_cast<size_t>(1) << j)) == 0) {
                        Q.push_back(muxed[j]);
                    }
                }

                assert (Q.size() <= m);
                PabloAST * neg = nullptr;
                if (LLVM_LIKELY(Q.size() > 0)) {
                    while (Q.size() > 1) {
                        PabloAST * a1 = Q.front(); Q.pop_front(); assert (a1);
                        PabloAST * a2 = Q.front(); Q.pop_front(); assert (a2);
                        pb->setInsertPoint(choose(a2, a1, adv));
                        assert (!Q.full());
                        PabloAST * tmp = pb->createOr(a1, a2); assert (tmp);
                        Q.push_back(tmp);
                    }
                    assert (Q.size() == 1);
                    neg = pb->createNot(Q.front()); Q.pop_front(); assert (neg);
                }

                assert (Q.empty());
                for (unsigned j = 0; j != m; ++j) {
                    if ((i & (static_cast<unsigned>(1) << j)) != 0) {
                        assert (!Q.full());
                        Q.push_back(muxed[j]);
                    }
                }

                assert (Q.size() <= m);
                assert (Q.size() >= 1);

                while (Q.size() > (1 + ((neg == nullptr) ? 1 : 0))) {
                    PabloAST * a1 = Q.front(); Q.pop_front(); assert (a1);
                    PabloAST * a2 = Q.front(); Q.pop_front(); assert (a2);

                    assert (!Q.full());
                    PabloAST * tmp = pb->createAnd(a1, a2); assert (tmp);
                    Q.push_back(tmp);
                }

                PabloAST * a1 = Q.front(); Q.pop_front(); assert (demux);
                PabloAST * a2 = neg;
                if (LLVM_UNLIKELY(Q.size() == 2)) {
                    a2 = Q.front(); Q.pop_front(); assert (a2);
                }
                pb->setInsertPoint(choose(a2, a1, adv));
                PabloAST * demux = pb->createAnd(a1, a2, "demux_" + V[i - 1]->getName()->to_string());

                V[i - 1]->replaceWith(demux, false, true);
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief topologicalSort
 *
 * After transforming the IR, we need to run this in order to always have a valid program. Each multiplex set
 * contains vertices corresponding to an Advance in the IR. While we know each Advance within a set is independent
 * w.r.t. the transitive closure of their dependencies in the IR, the position of each Advance's dependencies and
 * users within the IR isn't taken into consideration. Thus while there must be a valid ordering for the program,
 * it's not necessarily the original ordering.
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::topologicalSort(PabloBlock & entry) const {

    // Note: not a real topological sort. I expect the original order to be very close to the resulting one.

    std::unordered_set<PabloAST *> encountered;
    std::stack<Statement *> scope;
    for (Statement * stmt = entry.front(); ; ) {

        while ( stmt ) {

            bool moved = false;

            for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                PabloAST * const op = stmt->getOperand(i);
                if (LLVM_LIKELY(isa<Statement>(op))) {
                    if (LLVM_UNLIKELY(encountered.count(op) == 0)) {
                        Statement * next = stmt->getNextNode();
                        stmt->insertAfter(cast<Statement>(op));
                        stmt = next;
                        moved = true;
                        break;
                    }
                }
            }

            if (moved) {
                continue;
            }

            encountered.insert(stmt);

            if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                // Set the next statement to be the first statement of the inner scope and push the
                // next statement of the current statement into the scope stack.
                const PabloBlock & nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                scope.push(stmt->getNextNode());
                stmt = nested.front();
                assert (stmt);
                continue;
            }

            stmt = stmt->getNextNode();
        }

        if (scope.empty()) {
            break;
        }
        stmt = scope.top();
        scope.pop();
    }

}

} // end of namespace pablo

