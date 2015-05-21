#include "pablo_automultiplexing.hpp"
#include <pablo/codegenstate.h>
#include <llvm/ADT/SmallVector.h>
#include <llvm/ADT/DenseMap.h>
#include <llvm/ADT/DenseSet.h>
#include <stack>
#include <queue>
#include <unordered_set>
#include <pablo/analysis/bdd/bdd.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/numeric/ublas/matrix.hpp>

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
    RNG rng(std::random_device()());
    am.generateMultiplexSets(rng);
    am.approxMaxWeightIndependentSet();
    am.multiplexSelectedIndependentSets();
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
                    mIndexMap.emplace(stmt, m);
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
                assert (mIndexMap.count(stmt) != 0 && mIndexMap[stmt] == (u - 1));
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

    
    
    unsigned variables = 0;

    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    for (const Statement * stmt = entry.front(); ; ) {
        for (; stmt; stmt = stmt->getNextNode()) {

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

            switch (stmt->getClassTypeId()) {
                case PabloAST::ClassTypeId::Assign:
                    bdd = input[0];
                    break;
                case PabloAST::ClassTypeId::And:
                    bdd = engine.applyAnd(input[0], input[1]);
                    if (LLVM_UNLIKELY(engine.satOne(bdd).isFalse())) {
                        bdd = BDD::Contradiction();
                        // Look into making a "replaceAllUsesWithZero" method that can recursively apply
                        // the implication of having a Zero output.
                        stmt->replaceAllUsesWith(entry.createZeroes());
                        stmt->eraseFromParent(true);
                    }
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
                    // possibility of a contradition being calculated later. An example of this would be
                    // ¬(ScanThru(c,m) ∨ m) but are there others that would be harder to predict?
                case PabloAST::ClassTypeId::Call:
                case PabloAST::ClassTypeId::MatchStar:
                    bdd = engine.var(variables++);
                    break;
                case PabloAST::ClassTypeId::Advance: {
                        const BDD var = engine.var(variables++);

                        bdd = var;

                        // when we built the path graph, we constructed it in the same order; hense the row/column of
                        // the path graph is equivalent to the index.

                        const unsigned k = advances.size();

                        assert (mIndexMap.count(stmt) != 0 && mIndexMap[stmt] == k);

                        for (unsigned i = 0; i != k; ++i) {

                            Advance * adv;
                            BDD eq;
                            std::tie(adv, eq) = advances[i];

                            // If these advances are "shifting" their values by the same amount and aren't transitively dependant ...
                            if ((stmt->getOperand(1) != adv->getOperand(1)) && !edge(k, i, mPathGraph).second) {

                                const BDD C = engine.applyAnd(input[0], eq); // do we need to simplify this to identify subsets?
                                // Is there any satisfying truth assignment? If not, these streams are mutually exclusive.
                                if (engine.satOne(C).isFalse()) {
                                    auto f = mCharacterizationMap.find(adv);
                                    // build up our BDD equation for the k-th Advance
                                    bdd = engine.applyAnd(bdd, engine.applyNot(f->second));
                                    // but only
                                    f->second = engine.applyAnd(f->second, engine.applyNot(var));
                                    continue;
                                }
                                else if (LLVM_UNLIKELY(input[0] == C)) {
                                    add_edge(k, i, mSubsetGraph);
                                    continue;
                                }
                                else if (LLVM_UNLIKELY(eq == C)) {
                                    add_edge(i, k, mSubsetGraph);
                                    continue;
                                }
                            }
                            // We want the constraint graph to be acyclic; since the dependencies are already in topological order,
                            // adding an arc from a lesser to greater numbered vertex won't induce a cycle.
                            add_edge(i, k, mConstraintGraph);
                            // Append this advance to the list of known advances in this "basic block"; add on the input's BDD to
                            // eliminate the need for looking it up again.
                            advances.emplace_back(cast<Advance>(stmt), input[0]);
                        }
                    }
                    break;
            }
            mCharacterizationMap.insert(std::make_pair(stmt, bdd));
        }

        if (scope.empty()) {
            break;
        }
        stmt = scope.top();
        scope.pop();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateMultiplexSets
 * @param RNG random number generator
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::generateMultiplexSets(RNG & rng) {

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
            S.insert(*vi);
        }
    }

    for ( ; remainingVerticies >= 3; --remainingVerticies) {
        if (S.size() > 2) {
            addMultiplexSet(S);
        }
        // Randomly choose a vertex in S and discard it.
        const auto i = S.begin() + RNGDistribution(0, S.size() - 1)(rng);
        const Vertex u = *i;
        S.erase(i);
        EdgeIterator ei, ei_end;
        for (std::tie(ei, ei_end) = out_edges(u, mConstraintGraph); ei != ei_end; ++ei) {
            const Vertex v = target(*ei, mConstraintGraph);
            if ((--D[v]) == 0) {
                S.insert(v);
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addMultiplexSet
 * @param set an independent set
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::addMultiplexSet(const IndependentSet & set) {


}


AutoMultiplexing::AutoMultiplexing()
{

}


}
