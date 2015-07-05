#include "pablo_automultiplexing.hpp"
#include <pablo/codegenstate.h>
#include <llvm/ADT/BitVector.h>
#include <stack>
#include <queue>
#include <unordered_set>
#include <boost/container/flat_set.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/circular_buffer.hpp>
#include <include/simd-lib/builtins.hpp>
#include <pablo/builder.hpp>
#include <boost/range/iterator_range.hpp>
#include <pablo/printer_pablos.h>
#include <cudd.h>
#include <util.h>

using namespace llvm;
using namespace boost;
using namespace boost::container;
using namespace boost::numeric::ublas;

// #define PRINT_DEBUG_OUTPUT

#if !defined(NDEBUG) && !defined(PRINT_DEBUG_OUTPUT)
#define PRINT_DEBUG_OUTPUT
#endif

#ifdef PRINT_DEBUG_OUTPUT

#include <iostream>

using namespace pablo;
typedef uint64_t timestamp_t;

static inline timestamp_t read_cycle_counter() {
#ifdef __GNUC__
timestamp_t ts;
#ifdef __x86_64__
  unsigned int eax, edx;
  asm volatile("rdtsc" : "=a" (eax), "=d" (edx));
  ts = ((timestamp_t) eax) | (((timestamp_t) edx) << 32);
#else
  asm volatile("rdtsc\n" : "=A" (ts));
#endif
  return(ts);
#endif
#ifdef _MSC_VER
  return __rdtsc();
#endif
}

#define LOG(x) std::cerr << x << std::endl;
#define RECORD_TIMESTAMP(Name) const timestamp_t Name = read_cycle_counter()
#define LOG_GRAPH(Name, G) \
    LOG(Name << " |V|=" << num_vertices(G) << ", |E|="  << num_edges(G) << \
                " (" << (((double)num_edges(G)) / ((double)(num_vertices(G) * (num_vertices(G) - 1) / 2))) << ')')

unsigned __count_advances(const PabloBlock & entry) {

    std::stack<const Statement *> scope;
    unsigned advances = 0;

    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    for (const Statement * stmt = entry.front(); ; ) {
        while ( stmt ) {
            if (isa<Advance>(stmt)) {
                ++advances;
            }
            else if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
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
    return advances;
}

#define LOG_NUMBER_OF_ADVANCES(entry) LOG("|Advances|=" << __count_advances(entry))

#else
#define LOG(x)
#define RECORD_TIMESTAMP(Name)
#define LOG_GRAPH(Name, G)
#define LOG_NUMBER_OF_ADVANCES(entry)
#endif


namespace pablo {

bool AutoMultiplexing::optimize(const std::vector<Var *> & input, PabloBlock & entry) {

    // std::random_device rd;
    const auto seed = 2938639837; // rd();
    RNG rng(seed);

    LOG("Seed:                    " << seed);

    AutoMultiplexing am(input);
    RECORD_TIMESTAMP(start_initialize);
    am.initialize(entry);
    RECORD_TIMESTAMP(end_initialize);

    LOG("Initialize:              " << (end_initialize - start_initialize));

    LOG_NUMBER_OF_ADVANCES(entry);

    RECORD_TIMESTAMP(start_characterize);
    am.characterize(entry);
    RECORD_TIMESTAMP(end_characterize);

    LOG("Characterize:            " << (end_characterize - start_characterize));

    RECORD_TIMESTAMP(start_create_multiplex_graph);
    const bool multiplex = am.generateMultiplexSets(rng, 1);
    RECORD_TIMESTAMP(end_create_multiplex_graph);
    LOG("GenerateMultiplexSets:   " << (end_create_multiplex_graph - start_create_multiplex_graph));

    if (multiplex) {

        RECORD_TIMESTAMP(start_select_multiplex_sets);
        am.selectMultiplexSets(rng);
        RECORD_TIMESTAMP(end_select_multiplex_sets);
        LOG("SelectMultiplexSets:     " << (end_select_multiplex_sets - start_select_multiplex_sets));

        RECORD_TIMESTAMP(start_subset_constraints);
        am.applySubsetConstraints();
        RECORD_TIMESTAMP(end_subset_constraints);
        LOG("ApplySubsetConstraints:  " << (end_subset_constraints - start_subset_constraints));

        RECORD_TIMESTAMP(start_select_independent_sets);
        am.multiplexSelectedIndependentSets();
        RECORD_TIMESTAMP(end_select_independent_sets);
        LOG("MultiplexSelectedSets:   " << (end_select_independent_sets - start_select_independent_sets));

        RECORD_TIMESTAMP(start_topological_sort);
        am.topologicalSort(entry);
        RECORD_TIMESTAMP(end_topological_sort);
        LOG("TopologicalSort:         " << (end_topological_sort - start_topological_sort));
    }

    RECORD_TIMESTAMP(start_shutdown);
    am.Shutdown();
    RECORD_TIMESTAMP(end_shutdown);
    LOG("Shutdown:                " << (end_shutdown - start_shutdown));

    LOG_NUMBER_OF_ADVANCES(entry);

    return multiplex;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initialize
 * @param vars the input vars for this program
 * @param entry the entry block
 *
 * Scan through the program to identify any advances and calls then initialize the BDD engine with
 * the proper variable ordering.
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::initialize(PabloBlock & entry) {

    flat_map<const PabloAST *, unsigned> map;    
    std::stack<Statement *> scope;
    unsigned complexStatements = 0; // number of statements that cannot always be categorized without generating a new variable

    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    unsigned n = 0, m = 0;
    for (Statement * stmt = entry.front(); ; ) {
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
                    mAdvanceMap.emplace(stmt, m);
                    map.emplace(stmt, m++);
                case PabloAST::ClassTypeId::Call:
                case PabloAST::ClassTypeId::ScanThru:
                case PabloAST::ClassTypeId::MatchStar:
                    complexStatements++;
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
                assert (mAdvanceMap.count(stmt) && mAdvanceMap[stmt] == m);
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

    // Record the path / base constraint graph after removing any reflexive-loops.
    // Since G is a use-def graph and we want our constraint graph to be a def-use graph,
    // reverse the edges as we're writing them to obtain the transposed graph.
    mConstraintGraph = ConstraintGraph(m);
    mSubsetGraph = SubsetGraph(m);

    for (unsigned i = 0; i != m; ++i) {
        G(i, i) = false;
        for (unsigned j = 0; j != m; ++j) {
            if (G(i, j)) {
                add_edge(j, i, mConstraintGraph);
            }
        }        
    }

    // Initialize the BDD engine ...
    mManager = Cudd_Init((complexStatements + mBaseVariables.size()), 0, CUDD_UNIQUE_SLOTS, CUDD_CACHE_SLOTS, 0);
    Cudd_AutodynDisable(mManager);

    // Map the predefined 0/1 entries
    mCharacterizationMap[entry.createZeroes()] = Zero();
    mCharacterizationMap[entry.createOnes()] = One();

    // Order the variables so the input Vars are pushed to the end; they ought to
    // be the most complex to resolve.
    unsigned i = complexStatements;
    for (const Var * var : mBaseVariables) {
        mCharacterizationMap[var] = Cudd_bddIthVar(mManager, i++);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 * @param vars the input vars for this program
 *
 * Scan through the program and iteratively compute the BDDs for each statement.
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::characterize(PabloBlock & block) {
    for (Statement * stmt : block) {
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            // Set the next statement to be the first statement of the inner scope and push the
            // next statement of the current statement into the scope stack.
            characterize(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
            continue;
        }
        mCharacterizationMap[stmt] = characterize(stmt);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
inline DdNode * AutoMultiplexing::characterize(Statement * const stmt) {

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
            std::string tmp;
            llvm::raw_string_ostream msg(tmp);
            msg << "Uncharacterized operand " << std::to_string(i);
            PabloPrinter::print(stmt, " of ", msg);
            throw std::runtime_error(msg.str());
        }
        input[i] = f->second;
    }

    switch (stmt->getClassTypeId()) {
        case PabloAST::ClassTypeId::Assign:
            return input[0];
        case PabloAST::ClassTypeId::And:
            bdd = And(input[0], input[1]);
            break;
        case PabloAST::ClassTypeId::Next:
        case PabloAST::ClassTypeId::Or:
            return Or(input[0], input[1]);
        case PabloAST::ClassTypeId::Xor:
            return Xor(input[0], input[1]);
        case PabloAST::ClassTypeId::Not:
            return Not(input[0]);
        case PabloAST::ClassTypeId::Sel:
            bdd = Ite(input[0], input[1], input[2]);
            break;
        case PabloAST::ClassTypeId::ScanThru:
            // It may be possible use "Not(input[1])" for ScanThrus if we rule out the possibility
            // of a contradition being erroneously calculated later. An example of this
            // would be ¬(ScanThru(c,m) ∨ m)
        case PabloAST::ClassTypeId::MatchStar:
            if (LLVM_UNLIKELY(isZero(input[0]) || isZero(input[1]))) {
                return Zero();
            }
        case PabloAST::ClassTypeId::Call:
            return NewVar();
        case PabloAST::ClassTypeId::Advance:
            return characterize(cast<Advance>(stmt), input[0]);
        default:
            throw std::runtime_error("Unexpected statement type " + stmt->getName()->to_string());
    }

    return bdd;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
inline DdNode * AutoMultiplexing::characterize(Advance * adv, DdNode * input) {
    DdNode * Ik, * Ck, * Nk;
    if (LLVM_UNLIKELY(isZero(input))) {
        Ik = Ck = Nk = Zero();
    }
    else {

        Ik = input;
        Ck = NewVar();
        Nk = Not(Ck);

        assert (mAdvanceMap.count(adv));

        const auto k = mAdvanceMap[adv];

        std::vector<bool> unconstrained(k , false);

        // Can we use a transposed copy of the subset graph to determine an ordering of variables?
        for (unsigned i = 0; i != k; ++i) {
            // Have we already proven that these are unconstrained by the subset relationships?
            if (unconstrained[i]) continue;
            Advance * tmp = std::get<0>(mAdvance[i]);
            // If these advances are "shifting" their values by the same amount and aren't transitively dependant ...
            if ((adv->getOperand(1) == tmp->getOperand(1)) && (notTransitivelyDependant(i, k))) {
                DdNode * Ii = std::get<1>(mAdvance[i]);
                DdNode * Ni = std::get<2>(mAdvance[i]);
                DdNode * const IiIk = And(Ik, Ii);
                // Is there any satisfying truth assignment? If not, these streams are mutually exclusive.
                if (NoSatisfyingAssignment(IiIk)) {
                    assert (mCharacterizationMap.count(tmp));
                    DdNode *& Ci = mCharacterizationMap[tmp];
                    // Mark the i-th and k-th Advances as being mutually exclusive
                    Ck = And(Ck, Ni);
                    Ci = And(Ci, Nk);
                    // If Ai ∩ Ak = ∅ and Aj ⊂ Ai, Aj ∩ Ak = ∅.
                    graph_traits<SubsetGraph>::in_edge_iterator ei, ei_end;
                    for (std::tie(ei, ei_end) = in_edges(i, mSubsetGraph); ei != ei_end; ++ei) {
                        const auto j = source(*ei, mSubsetGraph);
                        if (notTransitivelyDependant(k, j)) {
                            Advance * adv = std::get<0>(mAdvance[j]);
                            assert (mCharacterizationMap.count(adv));
                            DdNode *& Cj = mCharacterizationMap[adv];
                            DdNode * Nj = std::get<2>(mAdvance[j]);
                            // Mark the i-th and j-th Advances as being mutually exclusive
                            Ck = And(Ck, Nj);
                            Cj = And(Cj, Nk);
                            unconstrained[j] = true;
                        }
                    }
                    unconstrained[i] = true;
                }
                else if (Ik == IiIk) {
                    // If Ik = Ii ∩ Ik then Ik ⊂ Ii. Record this in the subset graph with the arc (k,i).
                    // These edges will be moved into the multiplex set graph if Ai and Ak happen to be
                    // in the same mutually exclusive set.
                    add_edge(k, i, mSubsetGraph);
                    // If Ak ⊂ Ai and Ai ⊂ Aj, Ak ⊂ Aj.
                    graph_traits<SubsetGraph>::out_edge_iterator ei, ei_end;
                    for (std::tie(ei, ei_end) = out_edges(i, mSubsetGraph); ei != ei_end; ++ei) {
                        const auto j = target(*ei, mSubsetGraph);
                        add_edge(k, j, mSubsetGraph);
                        unconstrained[j] = true;
                    }
                    unconstrained[i] = true;
                }
                else if (Ii == IiIk) {
                    // If Ii = Ii ∩ Ik then Ii ⊂ Ik. Record this in the subset graph with the arc (i,k).
                    add_edge(i, k, mSubsetGraph);
                    // If Ai ⊂ Ak and Aj ⊂ Ai, Aj ⊂ Ak.
                    graph_traits<SubsetGraph>::in_edge_iterator ei, ei_end;
                    for (std::tie(ei, ei_end) = in_edges(i, mSubsetGraph); ei != ei_end; ++ei) {
                        const auto j = source(*ei, mSubsetGraph);
                        add_edge(j, k, mSubsetGraph);
                        unconstrained[j] = true;
                    }
                    unconstrained[i] = true;
                }
                Cudd_RecursiveDeref(mManager, IiIk);
            }
        }

        for (unsigned i = 0; i != k; ++i) {
            const Advance * const tmp = std::get<0>(mAdvance[i]);
            // Even if these Advances are mutually exclusive, they must be in the same scope or they cannot be safely multiplexed.
            if (!unconstrained[i] || (adv->getParent() != tmp->getParent())) {
                // We want the constraint graph to be acyclic; since the dependencies are already in topological order,
                // adding an arc from a lesser to greater numbered vertex won't induce a cycle.
                add_edge(i, k, mConstraintGraph);
            }
        }
    }

    mAdvance.emplace_back(adv, Ik, Nk);

    return Ck;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief notTransitivelyDependant
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool AutoMultiplexing::notTransitivelyDependant(const ConstraintVertex i, const ConstraintVertex j) const {
    assert (i < num_vertices(mConstraintGraph) && j < num_vertices(mConstraintGraph));
    return (mConstraintGraph.get_edge(i, j) == 0);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CUDD wrappers
 ** ------------------------------------------------------------------------------------------------------------- */

inline DdNode * AutoMultiplexing::Zero() const {
    return Cudd_ReadLogicZero(mManager);
}

inline DdNode * AutoMultiplexing::One() const {
    return Cudd_ReadOne(mManager);
}

inline DdNode * AutoMultiplexing::NewVar() {
    return Cudd_bddIthVar(mManager, mVariables++);
}

inline bool AutoMultiplexing::isZero(DdNode * const x) const {
    return x == Zero();
}

inline DdNode * AutoMultiplexing::And(DdNode * const x, DdNode * const y) {
    DdNode * r = Cudd_bddAnd(mManager, x, y);
    Cudd_Ref(r);
    return r;
}

inline DdNode * AutoMultiplexing::Or(DdNode * const x, DdNode * const y) {
    DdNode * r = Cudd_bddOr(mManager, x, y);
    Cudd_Ref(r);
    return r;
}

inline DdNode * AutoMultiplexing::Xor(DdNode * const x, DdNode * const y) {
    DdNode * r = Cudd_bddXor(mManager, x, y);
    Cudd_Ref(r);
    return r;
}

inline DdNode * AutoMultiplexing::Not(DdNode * const x) const {
    return Cudd_Not(x);
}

inline DdNode * AutoMultiplexing::Ite(DdNode * const x, DdNode * const y, DdNode * const z) {
    DdNode * r = Cudd_bddIte(mManager, x, y, z);
    Cudd_Ref(r);
    return r;
}

inline bool AutoMultiplexing::NoSatisfyingAssignment(DdNode * const x) {
    return Cudd_bddLeq(mManager, x, Zero());
}

inline void AutoMultiplexing::Shutdown() {
    #ifdef PRINT_DEBUG_OUTPUT
    Cudd_PrintInfo(mManager, stderr);
    #endif
    Cudd_Quit(mManager);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateMultiplexSets
 * @param RNG random number generator
 ** ------------------------------------------------------------------------------------------------------------- */
bool AutoMultiplexing::generateMultiplexSets(RNG & rng, unsigned k) {

    using vertex_t = ConstraintGraph::vertex_descriptor;
    using degree_t = graph_traits<ConstraintGraph>::degree_size_type;

    // What if we generated a "constraint free" graph instead? By taking each connected component of it
    // and computing the complement of it (with the same lesser to greater index ordering), we should
    // have the same problem here but decomposed into subgraphs.

    IndependentSet M, N;
    auto remainingVerticies = num_vertices(mConstraintGraph);
    std::vector<degree_t> D(remainingVerticies);
    M.reserve(15);
    N.reserve(15);

    mMultiplexSetGraph = MultiplexSetGraph(remainingVerticies);

    while (k) {

        // Push all source nodes into the new independent set N
        for (auto v : make_iterator_range(vertices(mConstraintGraph))) {
            const auto d = in_degree(v, mConstraintGraph);
            D[v] = d;
            if (d == 0) {
                N.push_back(v);
            }
        }

        for (;;) {

            addMultiplexSet(N, M);

            if (remainingVerticies == 0) {
                break;
            }

            assert (N.size() > 0);

            // Move all of our "newly" uncovered vertices in S into the "known" set M. By always choosing
            // at least one element from N, this will prevent us from adding the same multiplexing set again.
            M.insert(M.end(), N.begin(), N.end()); N.clear();

            do {
                // Randomly choose a vertex in S and discard it.
                assert (!M.empty());
                const auto i = M.begin() + IntDistribution(0, M.size() - 1)(rng);
                const vertex_t u = *i;
                M.erase(i);
                --remainingVerticies;
                for (auto e : make_iterator_range(out_edges(u, mConstraintGraph))) {
                    const vertex_t v = target(e, mConstraintGraph);
                    if ((--D[v]) == 0) {
                        N.push_back(v);
                    }
                }
            }
            while (N.empty() && remainingVerticies != 0);
        }

        if (--k == 0) {
            break;
        }

        N.clear();
        M.clear();
    }

    return num_vertices(mMultiplexSetGraph) > num_vertices(mConstraintGraph);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addMultiplexSet
 * @param N an independent set
 * @param M an independent set
 ** ------------------------------------------------------------------------------------------------------------- */
inline void AutoMultiplexing::addMultiplexSet(const IndependentSet & N, const IndependentSet & M) {

    // At this stage, the multiplex set graph is a directed bipartite graph that is used to show relationships
    // between the "set vertex" and its members. We obtain these from "generateMultiplexSets".

    if ((N.size() + M.size()) >= 3) {
        const auto u = add_vertex(mMultiplexSetGraph);
        for (const auto x : N) {
            add_edge(u, x, mMultiplexSetGraph);
        }
        for (const auto y : M) {
            add_edge(u, y, mMultiplexSetGraph);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief is_power_of_2
 * @param n an integer
 ** ------------------------------------------------------------------------------------------------------------- */
static inline bool is_power_of_2(const size_t n) {
    return ((n & (n - 1)) == 0) ;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief log2_plus_one
 ** ------------------------------------------------------------------------------------------------------------- */
static inline size_t log2_plus_one(const size_t n) {
    return std::log2<size_t>(n) + 1; // use a faster builtin function for this?
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief selectMultiplexSets
 * @param RNG random number generator
 *
 * This algorithm is simply computes a greedy set cover. We want an exact max-weight set cover but can generate new
 * sets by taking a subset of any existing set. With a few modifications, the greedy approach seems to work well
 * enough but more analysis is needed.
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::selectMultiplexSets(RNG &) {


    using OutEdgeIterator = graph_traits<MultiplexSetGraph>::out_edge_iterator;
    using InEdgeIterator = graph_traits<MultiplexSetGraph>::in_edge_iterator;
    using degree_t = MultiplexSetGraph::degree_size_type;
    using vertex_t = MultiplexSetGraph::vertex_descriptor;

    const size_t m = num_vertices(mConstraintGraph);
    const size_t n = num_vertices(mMultiplexSetGraph) - m;

    std::vector<degree_t> remaining(n, 0);
    std::vector<vertex_t> chosen_set(m, 0);

    for (unsigned i = 0; i != n; ++i) {
        remaining[i] = out_degree(i + m, mMultiplexSetGraph);
    }

    for (;;) {

        // Choose the set with the greatest number of vertices not already included in some other set.
        vertex_t k = 0;
        degree_t w = 0;
        for (vertex_t i = 0; i != n; ++i) {
            const degree_t r = remaining[i];
            if (w < r) {
                k = i;
                w = r;
            }
        }

        // Multiplexing requires at least 3 elements; if the best set contains fewer than 3, abort.
        if (w < 3) {
            break;
        }

        OutEdgeIterator ei, ei_end;
        for (std::tie(ei, ei_end) = out_edges(k + m, mMultiplexSetGraph); ei != ei_end; ++ei) {
            const vertex_t j = target(*ei, mMultiplexSetGraph);
            if (chosen_set[j] == 0) {
                chosen_set[j] = (k + m);
                InEdgeIterator ej, ej_end;
                for (std::tie(ej, ej_end) = in_edges(j, mMultiplexSetGraph); ej != ej_end; ++ej) {
                    remaining[source(*ej, mMultiplexSetGraph) - m]--;
                }
            }
        }

        assert (remaining[k] == 0);

        // If this contains 2^n elements for any n, discard the member that is most likely to be added
        // to some future set.
        if (is_power_of_2(w)) {
            vertex_t j = 0;
            degree_t w = 0;
            for (vertex_t i = 0; i != m; ++i) {
                if (chosen_set[i] == (k + m)) {
                    InEdgeIterator ej, ej_end;
                    degree_t r = 1;
                    for (std::tie(ej, ej_end) = in_edges(i, mMultiplexSetGraph); ej != ej_end; ++ej) {
                        // strongly prefer adding weight to unvisited sets that have more remaining vertices
                        r += std::pow(remaining[source(*ej, mMultiplexSetGraph) - m], 2);
                    }
                    if (w < r) {
                        j = i;
                        w = r;
                    }
                }
            }
            assert (w > 0);
            chosen_set[j] = 0;
            InEdgeIterator ej, ej_end;
            for (std::tie(ej, ej_end) = in_edges(j, mMultiplexSetGraph); ej != ej_end; ++ej) {
                remaining[source(*ej, mMultiplexSetGraph) - m]++;
            }
        }
    }

    for (unsigned i = 0; i != m; ++i) {
        InEdgeIterator ei, ei_end;
        std::tie(ei, ei_end) = in_edges(i, mMultiplexSetGraph);
        for (auto next = ei; ei != ei_end; ei = next) {
            ++next;
            if (source(*ei, mMultiplexSetGraph) != chosen_set[i]) {
                remove_edge(*ei, mMultiplexSetGraph);
            }
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
 * @brief applySubsetConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::applySubsetConstraints() {

    // When entering thus function, the multiplex set graph M is a bipartite DAG with edges denoting the set
    // relationships of our independent sets.
    const unsigned m = num_vertices(mConstraintGraph);
    const unsigned n = num_vertices(mMultiplexSetGraph);

    // Add in any edges from our subset constraints to M that are within the same set.
    bool hasSubsetConstraint = false;

    graph_traits<SubsetGraph>::edge_iterator ei, ei_end;
    for (std::tie(ei, ei_end) = edges(mSubsetGraph); ei != ei_end; ++ei) {
        const auto u = source(*ei, mSubsetGraph);
        const auto v = target(*ei, mSubsetGraph);
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

                    Advance * adv = std::get<0>(mAdvance[s]);
                    PabloBlock * pb = adv->getParent();

                    for (auto i : V) {
                        Q.push(std::get<0>(mAdvance[i])->getOperand(0));
                    }                    
                    pb->setInsertPoint(adv);
                    while (Q.size() > 1) {
                        PabloAST * a1 = Q.front(); Q.pop();
                        PabloAST * a2 = Q.front(); Q.pop();                        
                        Q.push(pb->createOr(a1, a2, "subset"));
                    }
                    assert (Q.size() == 1);

                    PabloAST * const mask = pb->createNot(Q.front()); Q.pop();
                    adv->setOperand(0, pb->createAnd(adv->getOperand(0), mask, "subset"));

                    // Similar to the above, we're going to OR together the result of each advance,
                    // including s. This will restore the advanced variable back to its original state.

                    // Gather the original users to this advance. We'll be manipulating it shortly.
                    Statement::Users U(adv->users());

                    // Add s to V and sort the list; it'll be closer to being in topological order.
                    V.push_back(s);
                    std::sort(V.begin(), V.end());
                    for (auto i : V) {
                        Q.push(std::get<0>(mAdvance[i]));
                    }
                    pb->setInsertPoint(adv);
                    while (Q.size() > 1) {
                        PabloAST * a1 = Q.front(); Q.pop();
                        PabloAST * a2 = Q.front(); Q.pop();                        
                        Q.push(pb->createOr(a1, a2, "subset"));
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
    const size_t max_m = log2_plus_one(max_n);

    std::vector<MultiplexSetGraph::vertex_descriptor> I(max_n);
    std::vector<Advance *> input(max_n);
    std::vector<PabloAST *> muxed(max_m);
    circular_buffer<PabloAST *> Q(max_n);

    // When entering thus function, the multiplex set graph M is a DAG with edges denoting the set
    // relationships of our independent sets.

    for (unsigned s = f; s != l; ++s) {
        const size_t n = out_degree(s, mMultiplexSetGraph);
        if (n) {

            const size_t m = log2_plus_one(n);

            graph_traits<MultiplexSetGraph>::out_edge_iterator ei, ei_end;
            std::tie(ei, ei_end) = out_edges(s, mMultiplexSetGraph);
            for (unsigned i = 0; i != n; ++ei, ++i) {
                I[i] = target(*ei, mMultiplexSetGraph);
            }
            std::sort(I.begin(), I.begin() + n);

            for (unsigned i = 0; i != n; ++i) {
                input[i] = std::get<0>(mAdvance[I[i]]);
            }

            PabloBlock * const block = input[0]->getParent();
            block->setInsertPoint(block->back());
            PabloBuilder builder(*block);
            Advance * const adv = input[0];

            /// Perform n-to-m Multiplexing
            for (size_t j = 0; j != m; ++j) {

                std::ostringstream prefix;
                prefix << "mux" << n << "to" << m << '_';
                for (size_t i = 1; i <= n; ++i) {
                    if ((i & (static_cast<size_t>(1) << j)) != 0) {
                        Q.push_back(input[i - 1]->getOperand(0));
                    }
                }

                while (Q.size() > 1) {
                    PabloAST * a1 = Q.front(); Q.pop_front(); assert (a1);
                    PabloAST * a2 = Q.front(); Q.pop_front(); assert (a2);
                    assert (!Q.full());                                        
                    Q.push_back(builder.createOr(a2, a1, "muxing"));
                }

                PabloAST * mux = Q.front(); Q.pop_front(); assert (mux);
                muxed[j] = builder.createAdvance(mux, adv->getOperand(1), prefix.str());
            }


            /// Perform m-to-n Demultiplexing
            // Now construct the demuxed values and replaces all the users of the original advances with them.
            for (size_t i = 1; i <= n; ++i) {

                for (size_t j = 0; j != m; ++j) {
                    if ((i & (static_cast<size_t>(1) << j)) == 0) {
                        Q.push_back(muxed[j]);
                    }
                }

                if (LLVM_LIKELY(Q.size() > 0)) {
                    while (Q.size() > 1) {
                        PabloAST * a1 = Q.front(); Q.pop_front(); assert (a1);
                        PabloAST * a2 = Q.front(); Q.pop_front(); assert (a2);
                        assert (!Q.full());
                        Q.push_back(builder.createOr(a2, a1, "demuxing"));
                    }
                    assert (Q.size() == 1);
                    PabloAST * neg = Q.front(); Q.pop_front();
                    Q.push_back(builder.createNot(neg, "demuxing")); assert (neg);
                }

                for (unsigned j = 0; j != m; ++j) {
                    if ((i & (static_cast<unsigned>(1) << j)) != 0) {
                        assert (!Q.full());
                        Q.push_back(muxed[j]);
                    }
                }

                while (Q.size() > 1) {
                    PabloAST * a1 = Q.front(); Q.pop_front(); assert (a1);
                    PabloAST * a2 = Q.front(); Q.pop_front(); assert (a2);
                    assert (!Q.full());
                    Q.push_back(builder.createAnd(a1, a2, "demuxing"));
                }

                PabloAST * demuxed = Q.front(); Q.pop_front(); assert (demux);
                input[i - 1]->replaceWith(demuxed, true, true);
            }

            // simplify(muxed, m, builder);
        }
    }
}



/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplify
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::simplify(const std::vector<PabloAST *> & variables, const unsigned n, PabloBuilder & builder) const {

    std::queue<PabloAST *> Q;
    llvm::DenseMap<PabloAST *, DdNode *> characterization;
    boost::container::flat_set<PabloAST *> uncharacterized;

    DdManager * manager = Cudd_Init(n + mBaseVariables.size(), 0, CUDD_UNIQUE_SLOTS, CUDD_CACHE_SLOTS, 0);
    Cudd_AutodynDisable(manager);

    unsigned i = 0;
    for (; i != n; ++i) {
        PabloAST * const var = variables[i];
        Q.push(var);
        characterization[var] = Cudd_bddIthVar(manager, i);
    }

    for (Var * var : mBaseVariables) {
        characterization[var] = Cudd_bddIthVar(manager, i++);
    }

    std::array<DdNode *, 3> input;

    while (!Q.empty()) {
        PabloAST * const var = Q.front(); Q.pop();
        for (PabloAST * user : var->users()) {
            Statement * stmt = nullptr;
            // If this user is a boolean operation ...
            switch (user->getClassTypeId()) {
                case PabloAST::ClassTypeId::And:
                case PabloAST::ClassTypeId::Or:
                case PabloAST::ClassTypeId::Not:
                case PabloAST::ClassTypeId::Xor:
                case PabloAST::ClassTypeId::Sel:
                    // And it is within the same scope ...
                    if ((stmt = cast<Statement>(user))->getParent() == &builder.getPabloBlock()) {
                        bool characterized = true;
                        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                            auto f = characterization.find(stmt->getOperand(i));
                            if (f == characterization.end()) {
                                characterized = false;
                                break;
                            }
                            input[i] = f->second;
                        }
                        // And every input operand is characterizied ...
                        if (characterized) {
                            DdNode * bdd = nullptr;
                            switch (user->getClassTypeId()) {
                                case PabloAST::ClassTypeId::And:
                                    bdd = Cudd_bddAnd(manager, input[0], input[1]);
                                    break;
                                case PabloAST::ClassTypeId::Or:
                                    bdd = Cudd_bddOr(manager, input[0], input[1]);
                                    break;
                                case PabloAST::ClassTypeId::Not:
                                    bdd = Cudd_Not(input[0]);
                                    break;
                                case PabloAST::ClassTypeId::Xor:
                                    bdd = Cudd_bddXor(manager, input[0], input[1]);
                                    break;
                                case PabloAST::ClassTypeId::Sel:
                                    bdd = Cudd_bddIte(manager, input[0], input[1], input[2]);
                                    break;
                                default: __builtin_unreachable();
                            }
                            characterization[stmt] = bdd;
                            uncharacterized.erase(stmt);
                            for (PabloAST * next : stmt->users()) {
                                if (characterization.count(next) == 0) {
                                    Q.push(next);
                                }
                            }
                            break;
                        }
                    }
                default:
                    // Otherwise we probably apply Quine McCluskey to one of this statements operands
                    // presuming they aren't characterized properly later.
                    uncharacterized.insert(user);
                    break;
            }
        }
    }
    // Gether up the output statements from the characterized inputs of the uncharacterized statements
    llvm::DenseMap<PabloAST *, DdNode *> outputs;
    for (PabloAST * var : uncharacterized) {
        Statement * stmt = cast<Statement>(var);
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            auto f = characterization.find(stmt->getOperand(i));
            if (f != characterization.end()) {
                outputs.insert(std::make_pair(f->first, f->second));
            }
        }
    }

    circular_buffer<PabloAST *> S(n);

    CUDD_VALUE_TYPE value;
    int * cube = nullptr;
    for (auto itr : outputs) {
        // Look into doing some more analysis here to see if a Xor or Sel operation is possible.
        DdGen * gen = Cudd_FirstCube(manager, itr.second, &cube, &value);
        while (!Cudd_IsGenEmpty(gen)) {
            // cube[0 ... n - 1] = { 0 : false, 1: true, 2: don't care }
            for (unsigned i = n - 1; i; --i) {
                if (cube[i] == 0) {
                    S.push_back(variables[i]);
                }
            }

            if (S.size() > 0) {
                while(S.size() > 1) {
                    PabloAST * A = S.front(); S.pop_front();
                    PabloAST * B = S.front(); S.pop_front();
                    S.push_back(builder.createOr(A, B));
                }
                PabloAST * C = S.front(); S.pop_front();
                S.push_back(builder.createNot(C));
            }

            for (unsigned i = n - 1; i; --i) {
                if (cube[i] == 1) {
                    S.push_back(variables[i]);
                }
            }

            while(S.size() > 1) {
                PabloAST * A = S.front(); S.pop_front();
                PabloAST * B = S.front(); S.pop_front();
                S.push_back(builder.createAnd(A, B));
            }

            Q.push(S.front()); S.pop_front();

            Cudd_NextCube(gen, &cube, &value);
        }
        Cudd_GenFree(gen);

        while (Q.size() > 1) {
            PabloAST * A = Q.front(); Q.pop();
            PabloAST * B = Q.front(); Q.pop();
            Q.push(builder.createOr(A, B));
        }

        Statement * stmt = cast<Statement>(itr.first);
        stmt->replaceWith(Q.front(), true, true);
        Q.pop();
    }
    Cudd_Quit(manager);


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
    std::unordered_set<const PabloAST *> encountered;
    std::stack<Statement *> scope;
    for (Statement * stmt = entry.front(); ; ) { restart:
        while ( stmt ) {
            for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                PabloAST * const op = stmt->getOperand(i);
                if (LLVM_LIKELY(isa<Statement>(op))) {
                    if (LLVM_UNLIKELY(encountered.count(op) == 0)) {
                        if (LLVM_UNLIKELY(isa<While>(stmt) && isa<Next>(op))) {
                            if (encountered.count(cast<Next>(op)->getInitial()) != 0) {
                                continue;
                            }
                        }
                        Statement * const next = stmt->getNextNode();
                        stmt->insertAfter(cast<Statement>(op));
                        stmt = next;
                        goto restart;
                    }
                }
            }
            if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                // Set the next statement to be the first statement of the inner scope and push the
                // next statement of the current statement into the scope stack.
                const PabloBlock & nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                scope.push(stmt->getNextNode());
                stmt = nested.front();
                continue;
            }
            encountered.insert(stmt);
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

