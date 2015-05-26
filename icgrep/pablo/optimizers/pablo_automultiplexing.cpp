#include "pablo_automultiplexing.hpp"
#include <pablo/codegenstate.h>
#include <llvm/ADT/SmallVector.h>
#include <llvm/ADT/DenseMap.h>
#include <llvm/ADT/DenseSet.h>
#include <llvm/ADT/BitVector.h>
#include <stack>
#include <queue>
#include <unordered_set>
#include <pablo/analysis/bdd/bdd.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <include/simd-lib/builtins.hpp>
#include <expression_map.hpp>

using namespace llvm;
using namespace bdd;
using namespace boost;
using namespace boost::container;
using namespace boost::numeric::ublas;

namespace pablo {

static void AutoMultiplexing::optimize(PabloBlock & block) {
    AutoMultiplexing am;
    Engine eng = am.initialize(vars, block);
    am.characterize(eng, block);
    am.createMappingGraph();
    RNG rng(std::random_device()());
    if (am.generateMultiplexSets(rng)) {
        am.approxMaxWeightIndependentSet();
        am.applySubsetConstraints();
        am.multiplexSelectedIndependentSets();
        am.topologicalSort();
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
Engine AutoMultiplexing::initialize(const std::vector<Var *> & vars, const PabloBlock & entry) {

    unsigned m = 1;
    unsigned variables = 0;

    flat_map<PabloAST *, unsigned> map;
    for (const Var * var : vars) {
        map.emplace(var, 0);
    }

    std::stack<const Statement *> scope;

    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    unsigned n = 0;
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
            switch (stmt->getClassTypeId()) {
                case PabloAST::ClassTypeId::Advance:
                    mAdvance.emplace(stmt, m);
                    map.emplace(stmt, ++m);
                case PabloAST::ClassTypeId::Call:
                case PabloAST::ClassTypeId::ScanThru:
                case PabloAST::ClassTypeId::MatchStar:
                    ++variables;
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

    matrix<bool> G(n, m);
    G.clear();
    for (unsigned i = 0; i != m; ++i) {
        G(i, i) = true;
    }

    for (const Statement * stmt = entry.front(), n = 0; ; ) {
        while ( stmt ) {
            // Fill in the transitive closure G ...
            unsigned u;
            if (isa<Advance>(stmt)) {                
                u = ++n;
                mAdvance.push_back(cast<Advance>(stmt));
                assert (mAdvance.size() == n);
            }
            else {
                u = ++m;
                map.emplace(stmt, u);
            }
            for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                PabloAST * const op = stmt->getOperand(i);
                if (LLVM_UNLIKELY(isa<Integer>(op) || isa<String>(op))) {
                    continue;
                }
                const unsigned v = map[op];
                for (unsigned w = 0; w != n; ++w) {
                    G(v, w) |= G(u, w);
                }
            }
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

    // Record our transitive closure in the path graph and remove any reflexive-loops
    mPathGraph = PathGraph(n);
    for (unsigned i = 0; i != n; ++i) {
        for (unsigned j = 0; j != n; ++j) {
            if (G(i, j)) {
                add_edge(i, j, mPathGraph);
            }
        }
        G(i, i) = false;
    }

    // Now take the transitive reduction of G
    for (unsigned j = 1; j != n; ++j) {
        for (unsigned i = 1; i != n; ++i) {
            if (G(i, j)) {
                // Eliminate any unecessary arcs not covered by a longer path
                for (unsigned k = 0; k != m; ++k) {
                    G(i, k) = G(j, k) ? false : G(i, k);
                }
            }
        }
    }

    // And now transcribe it into our base constraint graph
    mConstraintGraph = ConstraintGraph(m - 1);
    for (unsigned j = 1; j != n; ++j) {
        for (unsigned i = 1; i != n; ++i) {
            if (G(i, j)) {
                add_edge(i - 1, j - 1, mConstraintGraph);
            }
        }
    }

    // initialize the BDD engine ...
    Engine engine(variables + vars.size());
    mCharacterizationMap.emplace(entry.createZeroes(), BDD::Contradiction());
    mCharacterizationMap.emplace(entry.createOnes(), BDD::Tautology());
    // order the variables so that our inputs are pushed to the end; they ought to
    // be the most complex to resolve.
    for (const Var * var : vars) {
        mCharacterizationMap.emplace(var, engine.var(variables++));
    }
    return engine;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 * @param engine the BDD engine
 * @param vars the input vars for this program
 *
 * Scan through the program and iteratively compute the BDDs for each statement.
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::characterize(Engine & engine, const PabloBlock & entry) {

    std::vector<std::pair<Advance *, BDD>> advances;
    std::stack<const Statement *> scope;

    advances.reserve(mAdvance.size());

    unsigned variables = 0;

    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    for (const Statement * stmt = entry.front(); ; ) {
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

            BDD bdd;

            // Map our operands to the computed BDDs
            std::array<BDD, 3> input;
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
                    bdd = engine.applyAnd(input[0], input[1]);
                    break;
                case PabloAST::ClassTypeId::Or:
                case PabloAST::ClassTypeId::Next:
                    bdd = engine.applyOr(input[0], input[1]);
                    break;
                case PabloAST::ClassTypeId::Xor:
                    bdd = engine.applyXor(input[0], input[1]);
                    break;
                case PabloAST::ClassTypeId::Not:
                    bdd = engine.applyNot(input[0]);
                    break;
                case PabloAST::ClassTypeId::Sel:
                    bdd = engine.applySel(input[0], input[1], input[2]);
                    break;
                case PabloAST::ClassTypeId::ScanThru:
                    // It may be possible use "engine.applyNot(input[1])" for ScanThrus if we rule out the
                    // possibility of a contradition being erroneously calculated later. An example of this
                    // would be ¬(ScanThru(c,m) ∨ m) but are there others that would be harder to predict?
                case PabloAST::ClassTypeId::MatchStar:
                    if (LLVM_UNLIKELY(input[0] == false || input[1] == false)) {
                        bdd = false;
                        break;
                    }
                case PabloAST::ClassTypeId::Call:
                    testContradiction = false;
                    bdd = engine.var(variables++);
                    break;
                case PabloAST::ClassTypeId::Advance:
                    if (LLVM_UNLIKELY(input[0] == false)) {
                        bdd = false;
                    }
                    else {

                        const BDD var = engine.var(variables++);

                        bdd = var;

                        // When we built the path graph, we constructed it in the same order; hense the row/column of
                        // the path graph is equivalent to the index.

                        const BDD A = input[0];
                        const unsigned k = advances.size();

                        assert (stmt == mAdvance[k - 1]);

                        for (unsigned i = 0; i != k; ++i) {

                            Advance * adv;
                            BDD B;
                            std::tie(adv, B) = advances[i];

                            bool mutuallyExclusive = false;
                            bool constrained = false;

                            // If these advances are "shifting" their values by the same amount and aren't transitively dependant ...
                            if ((stmt->getOperand(1) != adv->getOperand(1)) && !edge(k, i, mPathGraph).second) {
                                const BDD C = engine.applyAnd(A, B); // do we need to simplify this to identify subsets?
                                // Is there any satisfying truth assignment? If not, these streams are mutually exclusive.
                                mutuallyExclusive = engine.satOne(C).isFalse();

                                constrained = !mutuallyExclusive || (stmt->getParent() != adv->getParent()) ;
                            }

                            if (mutuallyExclusive) {
                                auto f = mCharacterizationMap.find(adv);
                                // build up our BDD equation for the k-th advance
                                bdd = engine.applyAnd(bdd, engine.applyNot(f->second));
                                // but only mark the other advance as being mutually exclusive with the k-th *variable*
                                f->second = engine.applyAnd(f->second, engine.applyNot(var));
                            }

                            if (constrained) {
                                // We want the constraint graph to be acyclic; since the dependencies are already in topological order,
                                // adding an arc from a lesser to greater numbered vertex won't induce a cycle.
                                add_edge(i, k, mConstraintGraph);
                            }
                            else if (LLVM_UNLIKELY(A == C)) {
                                // If A = C and C = A ∩ B then A (the input to the k-th advance) is a *subset* of B (the input of the
                                // i-th advance). Record this in the subset graph with the arc (k,i).
                                add_edge(k, i, mSubsetGraph);
                                continue;
                            }
                            else if (LLVM_UNLIKELY(B == C)) {
                                add_edge(i, k, mSubsetGraph);
                                continue;
                            }
                        }

                        // Append this advance to the list of known advances in this "basic block"; add on the input's BDD to
                        // eliminate the need for looking it up again.
                        advances.emplace_back(cast<Advance>(stmt), A);

                        testContradiction = false;
                    }
                    break;
            }
            if (LLVM_UNLIKELY(testContradiction && engine.satOne(bdd) == false)) {
                stmt = stmt->replaceWith(entry.createZeroes());
            }
            else {
                mCharacterizationMap.insert(std::make_pair(stmt, bdd));
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
 * @brief createMappingGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::createMappingGraph() {
    mMappingGraph = MappingGraph(num_vertices(mConstraintGraph));
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

    bool canMultiplex = false;

    VertexIterator vi, vi_end;
    for (std::tie(vi, vi_end) = vertices(mConstraintGraph); vi != vi_end; ++vi) {
        auto d = in_degree(*vi, mConstraintGraph);
        D[*vi] = d;
        if (d == 0) {
            S.push_back(*vi);
        }
    }

    for ( ; remainingVerticies >= 3; --remainingVerticies) {
        if (S.size() >= 3) {
            addMultiplexSet(S);
            canMultiplex = true;
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

    return canMultiplex;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addMultiplexSet
 * @param set an independent set
 ** ------------------------------------------------------------------------------------------------------------- */
inline void AutoMultiplexing::addMultiplexSet(const IndependentSet & S) {

    // At this stage, the mapping graph is a directed bipartite graph that is used to show relationships between
    // the "set vertex" for each independent set and its members. We obtain these from "generateMultiplexSets".

    // Question: should we enumerate the power set of S?
    const auto v = add_vertex(mMappingGraph);
    for (auto u : S) {
        add_edge(v, u, mMappingGraph);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief approxMaxWeightIndependentSet
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::approxMaxWeightIndependentSet(RNG & rng) {

    // Compute our independent set graph from our mapping graph; effectively it's the graph resulting
    // from contracting one advance node edge with an adjacent vertex

    const unsigned m = num_vertices(mConstraintGraph);
    const unsigned n = num_vertices(mMappingGraph) - m;

    IndependentSetGraph G(n);

    // Record the "weight" of this independent set vertex
    for (IndependentSetGraph::vertex_descriptor i = 0; i != n; ++i) {
        G[i] = out_degree(m + i, mMappingGraph);
    }
    // Add in any edges based on the adjacencies in the mapping graph
    for (MappingGraph::vertex_descriptor i = 0; i != m; ++i) {
        graph_traits<MappingGraph>::in_edge_iterator ei, ei_end;
        std::tie(ei, ei_end) = in_edges(i, mMappingGraph);
        for (; ei != ei_end; ++ei) {
            for (auto ej = ei; ej != ei; ++ej) {
                add_edge(*ei - m, *ej - m, G);
            }
        }
    }
    // Process the graph using the greedy method of "Approximation Algorithms for the Weighted Independent Set Problem" (2005)
    // (Note: look into minimum independent dominating set algorithms. It fits better with the log2 + subset relationship cost model.)
    std::vector<IndependentSetGraph::vertex_descriptor> S;
    std::vector<bool> removed(n, false);
    for (;;) {
        // Let S be the set of verticies of miminal weight
        graph_traits<IndependentSetGraph>::vertex_iterator vi, vi_end;
        std::tie(vi, vi_end) = vertices(G);
        unsigned W = std::numeric_limits<unsigned>::max();
        for (; vi != vi_end; ++vi) {
            if (removed[*vi]) {
                continue;
            }
            const unsigned w = G[*vi];
            if (w <= W) {
                if (w < W) {
                    W = w;
                    S.clear();
                }
                S.push_back(*vi);
            }
        }
        if (S.empty()) {
            break;
        }
        // Select u from S
        const auto u = S[S.size() == 1 ? 0 : RNGDistribution(0, S.size() - 1)(rng)];
        // Remove u and its adjacencies from G; clear the adjacent set vertices from the mapping graph. This will effectively
        // choose the set refered to by u as one of our chosen independent sets and discard any other sets that contain one
        // of the vertices in the chosen set.
        graph_traits<IndependentSetGraph>::adjacency_iterator ai, ai_end;
        std::tie(ai, ai_end) = adjacent_vertices(u, G);
        for (; ai != ai_end; ++ai) {
            removed[*ai] = true;
            clear_out_edges(m + *ai, mMappingGraph);
        }
        removed[u] = true;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief applySubsetConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::applySubsetConstraints() {

    // When entering thus function, the mapping graph M is a bipartite DAG with edges denoting the set
    // relationships of our independent sets.
    const unsigned m = num_vertices(mConstraintGraph);
    const unsigned n = num_vertices(mMappingGraph);

    // Add in any edges from our subset constraints to M that are within the same set.
    bool hasSubsetConstraint = false;
    graph_traits<SubsetGraph>::edge_iterator ei, ei_end;
    for (std::tie(ei, ei_end) = edges(mSubsetGraph); ei != ei_end; ++ei) {
        const auto u = source(*ei, mSubsetGraph);
        const auto v = target(*ei, mSubsetGraph);
        graph_traits<MappingGraph>::in_edge_iterator ej, ej_end;
        // If both the source and target of ei are adjacent to the same vertex, that vertex must be the
        // "set vertex". Add the edge between the vertices.
        for (std::tie(ej, ej_end) = in_edges(u, mMappingGraph); ej != ej_end; ++ej) {
            auto w = target(*ej, mMappingGraph);
            // Only check (v, w) if w is a "set vertex".
            if (w >= (n - m) && edge(v, w).second) {
                add_edge(u, v, mMappingGraph);
                hasSubsetConstraint = true;
            }
        }
    }

    if (LLVM_UNLIKELY(hasSubsetConstraint)) {
        // At this point, M is still a DAG but no longer bipartite. We're going to scan through the set vertices
        // in M, and use a DFS to scan through M and eliminate any subset relationships in the AST.
        // That way, "multiplexSelectedIndependentSets" only needs to consider muxing and demuxing of the streams.

        std::vector<MappingGraph::vertex_descriptor> V;
        std::stack<MappingGraph::vertex_descriptor> S;
        std::queue<PabloAST *> Q;
        BitVector D(n - m, 0);

        for (auto i = m; i != n; ++i) {
            graph_traits<MappingGraph>::out_edge_iterator ei, ei_end;
            // For each member of a "set vertex" ...
            std::tie(ei, ei_end) = out_edges(i, mMappingGraph);
            for (; ei != ei_end; ++ei) {
                const auto s = source(*ei, mMappingGraph);
                if (out_degree(s, mMappingGraph) != 0) {
                    // First scan through the subgraph of vertices in M dominated by s and build up the set T,
                    // consisting of all sinks w.r.t. vertex s.
                    auto u = s;
                    for (;;) {
                        graph_traits<MappingGraph>::out_edge_iterator ej, ej_end;
                        for (std::tie(ej, ej_end) = out_edges(u, mMappingGraph); ej != ej_end; ++ej) {
                            auto v = target(*ej, mMappingGraph);
                            if (D.test(v)) {
                                continue;
                            }
                            D.set(v);
                            if (out_degree(v, mMappingGraph) == 0) {
                                V.push(v);
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
                    pb->setInsertPoint(adv->getPrevNode());
                    while (Q.size() > 1) {
                        PabloAST * a1 = Q.front(); Q.pop();
                        PabloAST * a2 = Q.front(); Q.pop();
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
                        pb->setInsertPoint(cast<Statement>(a2));
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

//#define FAST_LOG2(x) ((sizeof(unsigned long) * 8 - 1) - ScanReverseIntrinsic((unsigned long)(x)))

//#define FAST_CEIL_LOG2(x) (FAST_LOG_2(x) + ((x & (x - 1) != 0) ? 1 : 0))

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief multiplexSelectedIndependentSets
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::multiplexSelectedIndependentSets() {

    // When entering thus function, the mapping graph M is a bipartite DAG with edges denoting the set
    // relationships of our independent sets.

    unsigned N = 3;
    for (unsigned s = num_vertices(mConstraintGraph), e = num_vertices(mMappingGraph); s != e; ++s) {
        N = std::max<unsigned>(N, out_degree(s, mMappingGraph));
    }
    unsigned M = static_cast<unsigned>(std::ceil(std::log2(N))); // use a faster builtin function for this.

    std::vector<MappingGraph::vertex_descriptor> V(N);
    std::queue<PabloAST *> T(M);
    std::queue<PabloAST *> F(M);
    std::vector<SmallVector<User *, 4>> users(N);

    for (unsigned s = num_vertices(mConstraintGraph), e = num_vertices(mMappingGraph); s != e; ++s) {
        const unsigned n = out_degree(s, mMappingGraph);
        if (n) {

            const unsigned m = static_cast<unsigned>(std::ceil(std::log2(n))); // use a faster builtin function for this.

            graph_traits<MappingGraph>::out_edge_iterator ei_begin, ei_end;
            std::tie(ei_begin, ei_end) = out_edges(s, mMappingGraph);
            for (auto ei = ei_begin; ei != ei_end; ++ei) {
                V[std::distance(ei_begin, ei)] = target(*ei, mMappingGraph);
            }
            std::sort(V.begin(), V.begin() + n);

            PabloBlock * const pb = mAdvance[V[0]]->getParent();
            // Sanity test to make sure every advance is in the same scope.
            #ifndef NDEBUG
            for (auto i = 1; i != n; ++i) {
                assert (mAdvance[V[i]]->getParent() == pb);
            }
            #endif

            /// Perform n-to-m Multiplexing
            for (unsigned j = 0; j != m; ++j) {
                for (unsigned i = 0; i != (1 << m); ++i) {
                    if (((i + 1) & (1 << j)) != 0) {
                        T.push(mAdvance[V[i]]->getOperand(0));
                    }
                }
                // TODO: figure out a way to determine whether we're creating a duplicate value below.
                // The expression map findOrCall ought to work conceptually but the functors method
                // doesn't really work anymore with the current API.
                while (T.size() > 1) {
                    PabloAST * a1 = T.front(); T.pop();
                    PabloAST * a2 = T.front(); T.pop();
                    pb->setInsertPoint(cast<Statement>(a2));
                    T.push(pb->createOr(a1, a2));
                }
                mAdvance[V[j]]->setOperand(0, T.front()); T.pop();
            }

            /// Perform m-to-n Demultiplexing
            // Store the original users of our advances; we'll be modifying these extensively shortly.
            for (unsigned i = 0; i != n; ++i) {
                const Advance * const adv = mAdvance[V[i]];
                users[i].insert(users[i].begin(), adv->user_begin(), adv->user_end()) ;
            }

            // Now construct the demuxed values and replaces all the users of the original advances with them.
            for (unsigned i = 0; i != n; ++i) {
                for (unsigned j = 0; j != m; ++j) {
                    if (((i + 1) & (1 << j)) != 0) {
                        T.push(mAdvance[V[j]]);
                    }
                    else {
                        F.push(mAdvance[V[j]]);
                    }
                }
                while (T.size() > 1) {
                    PabloAST * a1 = T.front(); T.pop();
                    PabloAST * a2 = T.front(); T.pop();
                    pb->setInsertPoint(cast<Statement>(a2));
                    T.push(pb->createAnd(a1, a2));
                }
                assert (T.size() == 1);

                while (F.size() > 1) {
                    PabloAST * a1 = T.front(); T.pop();
                    PabloAST * a2 = T.front(); T.pop();
                    pb->setInsertPoint(cast<Statement>(a2));
                    F.push(pb->createOr(a1, a2));
                }
                assert (F.size() == 1);

                PabloAST * const demux = pb->createAnd(T.front(), pb->createNot(F.front()), "demux"); T.pop(); F.pop();
                for (PabloAST * use : users[i]) {
                    cast<Statement>(use)->replaceUsesOfWith(mAdvance[V[j]], demux);
                }
            }

            /// Clean up the unneeded advances ...
            for (unsigned i = m; i != n; ++i) {
                mAdvance[V[i]]->eraseFromParent(true);
            }
            for (unsigned i = 0; i != n; ++i) {
                users[i].clear();
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
 * users within the IR isn't taken into consideration. This while there must be a valid ordering for the program
 * it is not necessarily the original ordering.
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::topologicalSort() {


}


}
