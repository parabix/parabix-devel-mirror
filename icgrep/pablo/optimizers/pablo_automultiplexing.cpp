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

    Engine eng = am.initialize(block, vars);
    am.characterize(eng, vars);


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

    std::vector<const Statement *> advances;
    std::stack<const Statement *> scope;

    unsigned variables = 0;

    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    unsigned n = 1;
    for (const Statement * stmt = entry.front(); ; ) {
        for (; stmt; stmt = stmt->getNextNode()) {
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
                    advances.push_back(stmt);
                case PabloAST::ClassTypeId::Call:
                case PabloAST::ClassTypeId::ScanThru:
                case PabloAST::ClassTypeId::MatchStar:
                    ++variables;
                    break;
                default:
                    break;
            }
        }
        if (scope.empty()) {
            break;
        }
        stmt = scope.top();
        scope.pop();
    }

    // create the transitive closure matrix of our advances; we'll use
    const unsigned m = advances.size() + 1;
    flat_map<PabloAST *, unsigned> map;
    for (const Var * var : vars) {
        map.emplace(var, 0);
    }
    for (unsigned i = 1; i != m; ++i) {
        map.emplace(advances[i], i);
    }
    matrix<bool> G(n, m);
    G.clear();
    for (unsigned i = 0; i != m; ++i) {
        G(i, i) = true;
    }

    for (const Statement * stmt = entry.front(), n = 1; ; ) {
        for (; stmt; stmt = stmt->getNextNode()) {
            // Fill in the transitive closure G ...
            const unsigned u = n++;
            for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                const unsigned v = map[stmt->getOperand(i)];
                for (unsigned w = 0; w != m; ++w) {
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
        for (unsigned j = 0; j != m; ++j) {
            if (G(i, j)) {
                add_edge(i, j, mPathGraph);
            }
        }
        G(i, i) = false;
    }

    // Now transcribe the transitive reduction of G into our constraint graph
    mConstraintGraph = ConstraintGraph(m);
    for (unsigned j = 0; j != m; ++j) {
        for (unsigned i = 0; i != m; ++i) {
            if (G(i, j)) {
                add_edge(i, j, mConstraintGraph);
                // Eliminate any unecessary arcs not covered by a longer path
                for (unsigned k = 0; k != m; ++k) {
                    G(i, k) = G(j, k) ? false : G(i, k);
                }
            }
        }
    }

    // initialize the BDD engine ...
    Engine engine(variables + vars.size());
    mCharacterizationMap.emplace(entry.createZeroes(), BDD::Contradiction());
    mCharacterizationMap.emplace(entry.createOnes(), BDD::Tautology());
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
    unsigned offset = 0;

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
                    if (LLVM_UNLIKELY(mTestForConjunctionContradictions && engine.satOne(bdd).isFalse())) {
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
                    // some optimization may be possible use not op2 for scan thrus if we rule out the possibility
                    // of a contradition being calculated later.
                case PabloAST::ClassTypeId::Call:
                case PabloAST::ClassTypeId::MatchStar:
                    bdd = engine.var(variables++);
                    break;
                case PabloAST::ClassTypeId::Advance:
                    {
                        BDD var = engine.var(variables++);

                        bdd = var;

                        // when we built the path graph, we constructed it in the same order; hense the row/column of
                        // the path graph is equivalent to the index.

                        const unsigned k = advances.size();
                        const BDD A = input[0];

                        for (unsigned i = 0; i != k; ++i) {

                            Advance * adv;
                            BDD B;
                            std::tie(adv, B) = advances[i];

                            // Are these two advances advancing their values by the same amount?
                            if (stmt->getOperand(1) == adv->getOperand(1)) {
                                continue;
                            }

                            // Are these advances transitively dependant?
                            if (edge(offset + i, offset + k, mPathGraph).second) {
                                continue;
                            }

                            const BDD C = engine.applyAnd(A, B); // do we need to simplify this to identify subsets?
                            // Is there any satisfying truth assignment? If not, these streams are mutually exclusive.
                            if (engine.satOne(C).isFalse()) {
                                auto f = mCharacterizationMap.find(adv);
                                bdd = engine.applyAnd(bdd, engine.applyNot(f->second));
                                f->second = engine.applyAnd(f->second, engine.applyNot(var));
                            }
                            else if (LLVM_UNLIKELY(A == C)) {
                                add_edge(k, i, mSubsetGraph);
                            }
                            else if (LLVM_UNLIKELY(B == C)) {
                                add_edge(i, k, mSubsetGraph);
                            }
                            else {
                                // We want the constraint graph to be acyclic; since the dependencies are already in topological order,
                                // adding an arc from a lesser to greater numbered vertex won't induce a cycle.
                                add_edge(k > i ? i : k, k > i ? k : i, mConstraintGraph);
                            }

                            advances.emplace_back(cast<Advance>(stmt), A);
                        }
                    }
                    break;
                }
                mCharacterizationMap.insert(std::make_pair(stmt, bdd));
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
 * @brief findIndependentSets
 * @param bb
 * @param tli
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::findIndependentSets() {

    typedef DependencyGraph::vertex_descriptor                  Vertex;
    typedef graph_traits<DependencyGraph>::vertex_iterator      VertexIterator;
    typedef graph_traits<DependencyGraph>::out_edge_iterator    EdgeIterator;

    // push all source nodes into the active independent set S
    Vertices S;
    VertexIterator vi, vi_end;
    for (std::tie(vi, vi_end) = vertices(G); vi != vi_end; ++vi) {
        if (in_degree(*vi, G) == 0) {
            S.push_back(*vi);
        }
    }

    while (!S.empty()) {
        if (S.size() >= 3) {
            independentSets.push_back(std::move(Vertices(S)));
        }
        if (num_edges(G) == 0) {
            break;
        }
        // discard the node of maximum degree from S.
        auto max_vi = S.begin();
        auto max_timestamp = timestamp[*max_vi];
        auto max_degree = out_degree(*max_vi, G);
        for (auto vi = max_vi + 1; vi != S.end(); ++vi) {
            const auto d = out_degree(*vi, G);
            const auto t = timestamp[*vi];
             if (max_degree < d || (max_degree == d && max_timestamp < t)) {
             // if (max_timestamp < t || (max_timestamp == t && max_degree < d)) {
                max_degree = d;
                max_timestamp = t;
                max_vi = vi;
            }
        }

//        auto max_m = metric[*max_vi];
//        for (auto vi = max_vi + 1; vi != S.end(); ++vi) {
//            const auto m = metric[*max_vi];
//            const auto t = timestamp[*vi];
//            if (max_m < m || (max_m == m && max_timestamp < t)) {
//                max_m = m;
//                max_timestamp = t;
//                max_vi = vi;
//            }
//        }

        const unsigned v = *max_vi;
        S.erase(max_vi);
        EdgeIterator ei, ei_end;
        for (std::tie(ei, ei_end) = out_edges(v, G); ei != ei_end; ++ei) {
            const Vertex u = target(*ei, G);
            if (in_degree(u, G) == 1) {
                S.push_back(u);
            }
        }
        clear_out_edges(v, G);
    }
}




AutoMultiplexing::AutoMultiplexing()
{

}


}
