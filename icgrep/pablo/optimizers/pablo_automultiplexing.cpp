#include "pablo_automultiplexing.hpp"
#include <include/simd-lib/builtins.hpp>
#include <pablo/builder.hpp>
#include <pablo/function.h>
#include <pablo/printer_pablos.h>
#include <boost/container/flat_set.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/range/iterator_range.hpp>
#include <pablo/analysis/pabloverifier.hpp>
#include <stack>
#include <queue>
#include <unordered_set>
#include <bdd.h>

/// TODO: Investigate why ./icgrep -c -multiplexing-window-size=13,14...,20 "^\p{l}$" causes segfault in BuDDy.

using namespace llvm;
using namespace boost;
using namespace boost::container;
using namespace boost::numeric::ublas;

#define PRINT_DEBUG_OUTPUT

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

unsigned __count_advances(const PabloBlock * const entry) {

    std::stack<const Statement *> scope;
    unsigned advances = 0;

    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    for (const Statement * stmt = entry->front(); ; ) {
        while ( stmt ) {
            if (isa<Advance>(stmt)) {
                ++advances;
            }
            else if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                // Set the next statement to be the first statement of the inner scope and push the
                // next statement of the current statement into the scope stack.
                const PabloBlock * const nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                scope.push(stmt->getNextNode());
                stmt = nested->front();
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

using TypeId = PabloAST::ClassTypeId;

bool MultiplexingPass::optimize(PabloFunction & function, const unsigned limit, const unsigned maxSelections, const unsigned windowSize, const bool independent) {

//    std::random_device rd;
//    const auto seed = rd();
    const auto seed = 83234827342;

    LOG("Seed:                    " << seed);

    MultiplexingPass mp(seed, limit, maxSelections, windowSize);
    RECORD_TIMESTAMP(start_initialize);
    const unsigned advances = mp.initialize(function, independent);
    RECORD_TIMESTAMP(end_initialize);

    LOG("Initialize:              " << (end_initialize - start_initialize));

    LOG_NUMBER_OF_ADVANCES(function.getEntryBlock());

    if (advances == 0) {
        return false;
    }

    RECORD_TIMESTAMP(start_characterize);
    mp.characterize(function.getEntryBlock());
    RECORD_TIMESTAMP(end_characterize);

    LOG("Characterize:             " << (end_characterize - start_characterize));

    LOG("Nodes in BDD:             " << bdd_getnodenum() << " of " << bdd_getallocnum());

    RECORD_TIMESTAMP(start_shutdown);
    bdd_done();
    RECORD_TIMESTAMP(end_shutdown);
    LOG("Shutdown:                 " << (end_shutdown - start_shutdown));

    RECORD_TIMESTAMP(start_create_multiplex_graph);
    const bool multiplex = mp.generateCandidateSets();
    RECORD_TIMESTAMP(end_create_multiplex_graph);
    LOG("GenerateCandidateSets:    " << (end_create_multiplex_graph - start_create_multiplex_graph));

    if (multiplex) {

        RECORD_TIMESTAMP(start_usage_weighting);
        mp.generateUsageWeightingGraph();
        RECORD_TIMESTAMP(end_usage_weighting);
        LOG("GenerateUsageWeighting:   " << (end_usage_weighting - start_usage_weighting));

        RECORD_TIMESTAMP(start_select_multiplex_sets);
        mp.selectMultiplexSets();
        RECORD_TIMESTAMP(end_select_multiplex_sets);
        LOG("SelectMultiplexSets:      " << (end_select_multiplex_sets - start_select_multiplex_sets));

        RECORD_TIMESTAMP(start_subset_constraints);
        mp.eliminateSubsetConstraints();
        RECORD_TIMESTAMP(end_subset_constraints);
        LOG("ApplySubsetConstraints:   " << (end_subset_constraints - start_subset_constraints));

        RECORD_TIMESTAMP(start_select_independent_sets);
        mp.multiplexSelectedSets(function);
        RECORD_TIMESTAMP(end_select_independent_sets);
        LOG("SelectedIndependentSets:  " << (end_select_independent_sets - start_select_independent_sets));

        RECORD_TIMESTAMP(start_topological_sort);
        MultiplexingPass::topologicalSort(function);
        RECORD_TIMESTAMP(end_topological_sort);
        LOG("TopologicalSort:          " << (end_topological_sort - start_topological_sort));

        #ifndef NDEBUG
        PabloVerifier::verify(function, "post-multiplexing");
        #endif
    }

    LOG_NUMBER_OF_ADVANCES(function.getEntryBlock());

    return multiplex;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initialize
 * @param function the function to optimize
 * @returns true if there are fewer than three advances in this function
 *
 * Scan through the program to identify any advances and calls then initialize the BDD engine with
 * the proper variable ordering.
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned MultiplexingPass::initialize(PabloFunction & function, const bool independent) {

    std::stack<Statement *> scope;
    unsigned variableCount = 0; // number of statements that cannot always be categorized without generating a new variable

    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    unsigned statements = 0, advances = 0;
    for (Statement * stmt = function.getEntryBlock()->front(); ; ) {
        while ( stmt ) {
            ++statements;
            if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                scope.push(stmt->getNextNode());
                const PabloBlock * const nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();                
                stmt = nested->front();
                assert (stmt);
                continue;
            }
            switch (stmt->getClassTypeId()) {
                case TypeId::Advance:
                    ++advances;
                case TypeId::ScanThru:
                case TypeId::Call:
                case TypeId::MatchStar:
                    ++variableCount;
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

    // If there are fewer than three Advances in this program, just abort. We cannot reduce it.
    if (advances < 3) {
        return 0;
    }

    initializeBaseConstraintGraph(function.getEntryBlock(), statements, advances);

    mSubsetGraph = SubsetGraph(advances);

    initializeAdvanceDepth(function.getEntryBlock(), advances);

    // Initialize the BDD engine ...
    bdd_init(10000000, 100000);
    bdd_setvarnum(variableCount + function.getNumOfParameters());
    bdd_setcacheratio(64);
    bdd_setmaxincrease(10000000);
    bdd_autoreorder(BDD_REORDER_SIFT);

    // Map the constants and input variables
    mCharacterization[PabloBlock::createZeroes()] = bdd_zero();
    mCharacterization[PabloBlock::createOnes()] = bdd_one();
    mVariables = function.getNumOfParameters();

    // TODO: record information in the function to indicate which pairs of input variables are independent
    if (independent) {
        for (unsigned i = 0; i != mVariables; ++i) {
            BDD Vi = bdd_ithvar(i);
            BDD Ni = bdd_zero();
            for (unsigned j = 0; j != i; ++j) {
                Ni = bdd_addref(bdd_or(Ni, bdd_ithvar(j)));
            }
            for (unsigned j = i + 1; j != mVariables; ++j) {
                Ni = bdd_addref(bdd_or(Ni, bdd_ithvar(j)));
            }
            Ni = bdd_addref(bdd_not(Ni));
            mCharacterization[function.getParameter(i)] = bdd_addref(bdd_imp(Vi, Ni));
        }
    } else {
        for (unsigned i = 0; i != mVariables; ++i) {
            mCharacterization[function.getParameter(i)] = bdd_ithvar(i);
        }
    }

    return advances;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeBaseConstraintGraph
 ** ------------------------------------------------------------------------------------------------------------- */
inline void MultiplexingPass::initializeBaseConstraintGraph(PabloBlock * const block, const unsigned statements, const unsigned advances) {

    std::stack<Statement *> scope;
    flat_map<const PabloAST *, unsigned> M;
    M.reserve(statements);
    matrix<bool> G(statements, advances, false);
    for (unsigned i = 0; i != advances; ++i) {
        G(i, i) = true;
    }

    unsigned n = advances;
    unsigned k = 0;
    for (const Statement * stmt = block->front();;) {
        while ( stmt ) {
            unsigned u = 0;
            if (LLVM_UNLIKELY(isa<Advance>(stmt))) {
                u = k++;
            } else {
                u = n++;
            }
            M.emplace(stmt, u);
            for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                const PabloAST * const op = stmt->getOperand(i);
                if (LLVM_LIKELY(isa<Statement>(op))) {
                    const unsigned v = M[op];
                    for (unsigned w = 0; w != k; ++w) {
                        G(u, w) |= G(v, w);
                    }
                }
            }
            if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                scope.push(stmt->getNextNode());
                const PabloBlock * const nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                stmt = nested->front();
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

    assert (k == advances);

    // Initialize the base constraint graph by effectively transposing G and removing reflective loops
    mConstraintGraph = ConstraintGraph(advances);
    for (unsigned i = 0; i != advances; ++i) {
        for (unsigned j = 0; j < i; ++j) {
            if (G(i, j)) {
                add_edge(j, i, mConstraintGraph);
            }
        }
        for (unsigned j = i + 1; j < advances; ++j) {
            if (G(i, j)) {
                add_edge(j, i, mConstraintGraph);
            }
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeAdvanceDepth
 ** ------------------------------------------------------------------------------------------------------------- */
inline void MultiplexingPass::initializeAdvanceDepth(PabloBlock * const entryBlock, const unsigned advances) {

    std::stack<Statement *> scope;
    unsigned k = 0;
    int maxDepth = 0;
    const PabloBlock * advanceScope[advances];
    mAdvance.resize(advances, nullptr);
    mAdvanceDepth.resize(advances, 0);
    mAdvanceNegatedVariable.reserve(advances);
    for (Statement * stmt = entryBlock->front(); ; ) {
        while ( stmt ) {
            if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                scope.push(stmt->getNextNode());
                const PabloBlock * const nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                stmt = nested->front();
                assert (stmt);
                continue;
            } else if (LLVM_UNLIKELY(isa<Advance>(stmt))) {
                int depth = 0;
                mAdvance[k] = cast<Advance>(stmt);
                advanceScope[k] = cast<Advance>(stmt)->getParent();
                for (unsigned i = 0; i != k; ++i) {
                    if (edge(i, k, mConstraintGraph).second || (advanceScope[i] != advanceScope[k])) {
                        depth = std::max<int>(depth, mAdvanceDepth[i]);
                    }
                }
                mAdvanceDepth[k++] = ++depth;
                maxDepth = std::max(maxDepth, depth);
            }
            stmt = stmt->getNextNode();
        }
        if (scope.empty()) {
            break;
        }
        stmt = scope.top();
        scope.pop();
    }
    assert (k == advances);

    LOG("Window Size / Max Depth: " << mWindowSize << " of " << maxDepth);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 * @param vars the input vars for this program
 *
 * Scan through the program and iteratively compute the BDDs for each statement.
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::characterize(PabloBlock * const block) {
    for (Statement * stmt : *block) {
        if (LLVM_UNLIKELY(isa<If>(stmt))) {
            characterize(cast<If>(stmt)->getBody());
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            characterize(cast<While>(stmt)->getBody());
            for (const Next * var : cast<While>(stmt)->getVariants()) {
                BDD & assign = get(var->getInitial());
                assign = bdd_addref(bdd_or(assign, get(var)));
            }
        } else {
            mCharacterization.insert(std::make_pair(stmt, characterize(stmt)));
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief throwUnexpectedStatementTypeError
 ** ------------------------------------------------------------------------------------------------------------- */
static void throwUnexpectedStatementTypeError(const Statement * const stmt) {
    std::string tmp;
    raw_string_ostream err(tmp);
    err << "Unexpected statement type ";
    PabloPrinter::print(stmt, err);
    throw std::runtime_error(err.str());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
inline BDD MultiplexingPass::characterize(Statement * const stmt) {
    assert (stmt->getNumOperands() > 0);
    BDD bdd = get(stmt->getOperand(0));
    switch (stmt->getClassTypeId()) {
        case TypeId::Assign:
        case TypeId::Next:
            break;
        case TypeId::And:
            for (unsigned i = 1; i != stmt->getNumOperands(); ++i) {
                bdd = bdd_and(bdd, get(stmt->getOperand(i)));
            }
            break;
        case TypeId::Or:
            for (unsigned i = 1; i != stmt->getNumOperands(); ++i) {
                bdd = bdd_or(bdd, get(stmt->getOperand(i)));
            }
            break;
        case TypeId::Xor:
            for (unsigned i = 1; i != stmt->getNumOperands(); ++i) {
                bdd = bdd_xor(bdd, get(stmt->getOperand(i)));
            }
            break;
        case TypeId::Not:
            bdd = bdd_not(bdd);
            break;
        case TypeId::Sel:
            bdd = bdd_ite(bdd, get(stmt->getOperand(1)), get(stmt->getOperand(2)));
            break;
        case TypeId::ScanThru:
            // ScanThru(c, m) := (c + m) ∧ ¬m. Thus we can conservatively represent this statement using the BDD
            // for ¬m --- provided no derivative of this statement is negated in any fashion.
        case TypeId::MatchStar:
        case TypeId::Call:
            return bdd_ithvar(mVariables++);
        case TypeId::Advance:
            return characterize(cast<Advance>(stmt), bdd);
        default:
            throwUnexpectedStatementTypeError(stmt);
    }
    return bdd_addref(bdd);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
inline BDD MultiplexingPass::characterize(Advance * const adv, const BDD Ik) {
    const auto k = mAdvanceNegatedVariable.size();
    assert (mAdvance[k] == adv);
    std::vector<bool> unconstrained(k , false);
    for (unsigned i = 0; i != k; ++i) {

        // Are we interested in testing these streams to see whether they are mutually exclusive?
        if (exceedsWindowSize(i, k)) continue;

        // Have we already proven that they are unconstrained by their subset relationship?
        if (unconstrained[i]) continue;

        // If these Advances are mutually exclusive, in the same scope, transitively independent, and shift their
        // values by the same amount, we can safely multiplex them. Otherwise mark the constraint in the graph.
        const Advance * const ithAdv = mAdvance[i];
        if ((mTestConstrainedAdvances || independent(i, k)) && (ithAdv->getOperand(1) == adv->getOperand(1))) {
            const BDD Ii = get(ithAdv->getOperand(0));
            const BDD IiIk = bdd_addref(bdd_and(Ii, Ik));
            // Is there any satisfying truth assignment? If not, these streams are mutually exclusive.
            if (bdd_satone(IiIk) == bdd_zero()) {
                // If Ai ∩ Ak = ∅ and Aj ⊂ Ai, Aj ∩ Ak = ∅.
                for (auto e : make_iterator_range(in_edges(i, mSubsetGraph))) {
                    unconstrained[source(e, mSubsetGraph)] = true;
                }
                unconstrained[i] = true;
            } else if (Ii == IiIk) {
                // If Ii = Ii ∩ Ik then Ii ⊂ Ik. Record this in the subset graph with the arc (i, k).
                // Note: the AST will be modified to make these mutually exclusive if Ai and Ak end up in
                // the same multiplexing set.
                add_edge(i, k, mSubsetGraph);
                // If Ai ⊂ Ak and Aj ⊂ Ai, Aj ⊂ Ak.
                for (auto e : make_iterator_range(in_edges(i, mSubsetGraph))) {
                    const auto j = source(e, mSubsetGraph);
                    add_edge(j, k, mSubsetGraph);
                    unconstrained[j] = true;
                }
                unconstrained[i] = true;
            } else if (Ik == IiIk) {
                // If Ik = Ii ∩ Ik then Ik ⊂ Ii. Record this in the subset graph with the arc (k, i).
                add_edge(k, i, mSubsetGraph);
                // If Ak ⊂ Ai and Ai ⊂ Aj, Ak ⊂ Aj.
                for (auto e : make_iterator_range(out_edges(i, mSubsetGraph))) {
                    const auto j = target(e, mSubsetGraph);
                    add_edge(k, j, mSubsetGraph);
                    unconstrained[j] = true;
                }
                unconstrained[i] = true;
            }
            bdd_delref(IiIk);
        }
    }

    BDD Ak = bdd_ithvar(mVariables++);
    const BDD Nk = bdd_addref(bdd_not(Ak));
    for (unsigned i = 0; i != k; ++i) {
        if (unconstrained[i]) {
            // Note: this algorithm deems two streams are mutually exclusive if and only if the conjuntion of their BDDs results
            // in a contradiction. To generate a contradiction when comparing Advances, the BDD of each Advance is represented by
            // the conjunction of variable representing that Advance and the negation of all variables for the Advances in which
            // the inputs are deemed to be mutually exclusive. For example, if the input of the i-th Advance is mutually exclusive
            // with the input of the j-th and k-th Advance, the BDD of the i-th Advance is Ai ∧ ¬Aj ∧ ¬Ak.
            const Advance * const ithAdv = mAdvance[i];
            const BDD Ni = mAdvanceNegatedVariable[i];
            BDD & Ai = get(ithAdv);
            Ai = bdd_addref(bdd_and(Ai, Nk));
            Ak = bdd_addref(bdd_and(Ak, Ni));
            if (independent(i, k) && (adv->getParent() == ithAdv->getParent())) {
                continue;
            }
        }
        add_edge(i, k, mConstraintGraph);
    }
    // To minimize the number of BDD computations, store the negated variable instead of negating it each time.
    mAdvanceNegatedVariable.emplace_back(Nk);
    return Ak;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief independent
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool MultiplexingPass::independent(const ConstraintVertex i, const ConstraintVertex j) const {
    assert (i < num_vertices(mConstraintGraph) && j < num_vertices(mConstraintGraph));
    return (mConstraintGraph.get_edge(i, j) == 0);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief exceedsWindowSize
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool MultiplexingPass::exceedsWindowSize(const ConstraintVertex i, const ConstraintVertex j) const {
    assert (i < mAdvanceDepth.size() && j < mAdvanceDepth.size());
    return (std::abs<int>(mAdvanceDepth[i] - mAdvanceDepth[j]) > mWindowSize);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief is_power_of_2
 * @param n an integer
 ** ------------------------------------------------------------------------------------------------------------- */
static inline bool is_power_of_2(const size_t n) {
    return ((n & (n - 1)) == 0);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateUsageWeightingGraph
 *
 * Prior to generating our candidate sets, scan through the constraint graph and generate a clique in the usage
 * weighting graph each set of constraint-free Advance nodes that have the same users. We may be able optimize
 * the demultiplexing calculations using range expressions.
 *
 * Note: it'd be preferable to contract vertices in the constraint graph prior to scanning through it but that
 * leaves us with a more difficult problem. Namely, Advance nodes may belong to more than one clique but it'd be
 * useless to compute a value twice; furthermore, we want to avoid generating a multiplexing set whose size is 2^n
 * for any n ∈ ℤ* but don't want to needlessly limit the size of any clique. Look into this further later.
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::generateUsageWeightingGraph() {
    const auto n = num_vertices(mConstraintGraph); assert (n > 2);
    // Let G be the complement of the constraint graph ∪ the subset graph restricted to only the edges corresponding
    // to pairs of Advances with the same users.
    CliqueGraph G(n);
    for (unsigned i = 0; i != (n - 1); ++i) {
        const Advance * const advI = mAdvance[i];
        for (unsigned j = i + 1; j != n; ++j) {
            const Advance * const advJ = mAdvance[j];
            if (LLVM_UNLIKELY(advI->getNumUses() == advJ->getNumUses()) && independent(i, j)) {
                if (LLVM_UNLIKELY(std::equal(advI->user_begin(), advI->user_end(), advJ->user_begin()))) {
                    // INVESTIGATE: we should be able to ignore subset relations if these are going to become a
                    // range expression. Look into making a proof for it once the range expression calculation
                    // is finished.
                    if (!(edge(i, j, mSubsetGraph).second || edge(j, i, mSubsetGraph).second)) {
                        add_edge(i, j, G);
                    }
                }
            }
        }
    }
    if (num_edges(G) > 0) {
        const CliqueSets S = findMaximalCliques(G);
        for (unsigned i = 0; i != n; ++i) {
            clear_vertex(i, G);
        }
        for (const std::vector<CliqueGraph::vertex_descriptor> & C : S) {
            const unsigned m = C.size(); assert (m > 1);
            for (unsigned i = 1; i != m; ++i) {
                for (unsigned j = 0; j != i; ++j) {
                    add_edge(C[j], C[i], G);
                }
            }
        }
    }
    mUsageGraph = G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateMultiplexSets
 ** ------------------------------------------------------------------------------------------------------------- */
bool MultiplexingPass::generateCandidateSets() {

    // What if we generated a "constraint free" graph instead? By taking each connected component of it
    // and computing the complement of it (with the same lesser to greater index ordering), we should
    // have the same problem here but decomposed into subgraphs.

    VertexVector S;
    std::vector<ConstraintGraph::degree_size_type> D(num_vertices(mConstraintGraph));
    S.reserve(15);

    mMultiplexSetGraph = MultiplexSetGraph(num_vertices(mConstraintGraph));

    // Push all source nodes into the (initial) independent set S
    for (auto v : make_iterator_range(vertices(mConstraintGraph))) {
        const auto d = in_degree(v, mConstraintGraph);
        D[v] = d;
        if (d == 0) {
            S.push_back(v);
        }
    }

    assert (S.size() > 0);

    auto remainingVerticies = num_vertices(mConstraintGraph) - S.size();

    do {

        addCandidateSet(S);

        bool noNewElements = true;
        do {
            assert (S.size() > 0);
            // Randomly choose a vertex in S and discard it.
            const auto i = S.begin() + IntDistribution(0, S.size() - 1)(mRNG);
            assert (i != S.end());
            const ConstraintVertex u = *i;
            S.erase(i);

            for (auto e : make_iterator_range(out_edges(u, mConstraintGraph))) {
                const ConstraintVertex v = target(e, mConstraintGraph);
                if ((--D[v]) == 0) {
                    S.push_back(v);
                    --remainingVerticies;
                    noNewElements = false;
                }
            }
        }
        while (noNewElements && remainingVerticies);
    }
    while (remainingVerticies);

    return num_vertices(mMultiplexSetGraph) > num_vertices(mConstraintGraph);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief choose
 *
 * Compute n choose k
 ** ------------------------------------------------------------------------------------------------------------- */
__attribute__ ((const)) inline unsigned long choose(const unsigned n, const unsigned k) {
    if (n < k)
        return 0;
    if (n == k || k == 0)
        return 1;
    unsigned long delta = k;
    unsigned long max = n - k;
    if (delta < max) {
        std::swap(delta, max);
    }
    unsigned long result = delta + 1;
    for (unsigned i = 2; i <= max; ++i) {
        result = (result * (delta + i)) / i;
    }
    return result;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief select
 *
 * James McCaffrey's algorithm for "Generating the mth Lexicographical Element of a Mathematical Combination"
 ** ------------------------------------------------------------------------------------------------------------- */
void select(const unsigned n, const unsigned k, const unsigned m, unsigned * element) {
    unsigned long a = n;
    unsigned long b = k;
    unsigned long x = (choose(n, k) - 1) - m;
    for (unsigned i = 0; i != k; ++i) {
        unsigned long y = 0;
        while ((y = choose(--a, b)) > x);
        x = x - y;
        b = b - 1;
        element[i] = (n - 1) - a;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addCandidateSet
 * @param S an independent set
 ** ------------------------------------------------------------------------------------------------------------- */
inline void MultiplexingPass::addCandidateSet(const VertexVector & S) {
    if (S.size() >= 3) {
        if (S.size() <= mMultiplexingSetSizeLimit) {
            const auto u = add_vertex(mMultiplexSetGraph);
            for (const auto v : S) {
                add_edge(u, v, mMultiplexSetGraph);
            }
        } else {
            const auto max = choose(S.size(), mMultiplexingSetSizeLimit);
            unsigned element[mMultiplexingSetSizeLimit];
            if (LLVM_UNLIKELY(max <= mMaxMultiplexingSetSelections)) {
                for (unsigned i = 0; i != max; ++i) {
                    select(S.size(), mMultiplexingSetSizeLimit, i, element);
                    const auto u = add_vertex(mMultiplexSetGraph);
                    for (unsigned j = 0; j != mMultiplexingSetSizeLimit; ++j) {
                        add_edge(u, S[element[j]], mMultiplexSetGraph);
                    }
                }
            } else { // take m random samples
                for (unsigned i = 0; i != mMaxMultiplexingSetSelections; ++i) {
                    select(S.size(), mMultiplexingSetSizeLimit, mRNG() % max, element);
                    const auto u = add_vertex(mMultiplexSetGraph);
                    for (unsigned j = 0; j != mMultiplexingSetSizeLimit; ++j) {
                        add_edge(u, S[element[j]], mMultiplexSetGraph);
                    }
                }
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief log2_plus_one
 ** ------------------------------------------------------------------------------------------------------------- */
static inline size_t log2_plus_one(const size_t n) {
    return std::log2<size_t>(n) + 1;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief selectMultiplexSets
 *
 * This algorithm is simply computes a greedy set cover. We want an exact max-weight set cover but can generate new
 * sets by taking a subset of any existing set. With a few modifications, the greedy approach seems to work well
 * enough but can be shown to produce a suboptimal solution if there are three candidate sets labelled A, B and C,
 * in which A ∩ B = ∅, |A| ≤ |B| < |C|, and C ⊂ (A ∪ B).
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::selectMultiplexSets() {

    using SetIterator = graph_traits<MultiplexSetGraph>::in_edge_iterator;
    using ElementIterator = graph_traits<MultiplexSetGraph>::out_edge_iterator;
    using degree_t = MultiplexSetGraph::degree_size_type;
    using vertex_t = MultiplexSetGraph::vertex_descriptor;

    const size_t m = num_vertices(mConstraintGraph);
    const size_t n = num_vertices(mMultiplexSetGraph) - m;

    degree_t remaining[n];
    vertex_t chosen_set[m];

    for (unsigned i = 0; i != n; ++i) {
        remaining[i] = out_degree(i + m, mMultiplexSetGraph);
    }
    for (unsigned i = 0; i != m; ++i) {
        chosen_set[i] = 0;
    }

    for (;;) {

        // Choose the set with the greatest number of vertices not already included in some other set.
        vertex_t k = 0;
        degree_t w = 0;
        for (vertex_t i = 0; i != n; ++i) {
            degree_t r = remaining[i];
            if (r > 2) { // if this set has at least 3 elements.
                r *= r;
                ElementIterator begin, end;
                std::tie(begin, end) = out_edges(i + m, mMultiplexSetGraph);
                for (auto ei = begin; ei != end; ++ei) {
                    for (auto ej = ei; ++ej != end; ) {
                        if (edge(target(*ei, mMultiplexSetGraph), target(*ej, mMultiplexSetGraph), mUsageGraph).second) {
                            ++r;
                        }
                    }
                }
                if (w < r) {
                    k = i;
                    w = r;
                }
            }
        }

        // Multiplexing requires 3 or more elements; if no set contains at least 3, abort.
        if (w == 0) {
            break;
        }

        for (const auto ei : make_iterator_range(out_edges(k + m, mMultiplexSetGraph))) {
            const vertex_t j = target(ei, mMultiplexSetGraph);
            if (chosen_set[j] == 0) {
                chosen_set[j] = (k + m);
                for (const auto ej : make_iterator_range(in_edges(j, mMultiplexSetGraph))) {
                    remaining[source(ej, mMultiplexSetGraph) - m]--;
                }
            }
        }

        assert (remaining[k] == 0);

        // If this contains 2^n elements for any n, discard the member that is most likely to be added
        // to some future set.
        if (LLVM_UNLIKELY(is_power_of_2(w))) {
            vertex_t j = 0;
            degree_t w = 0;
            for (vertex_t i = 0; i != m; ++i) {
                if (chosen_set[i] == (k + m)) {
                    degree_t r = 1;
                    for (const auto ej : make_iterator_range(in_edges(i, mMultiplexSetGraph))) {
                        // strongly prefer adding weight to unvisited sets that have more remaining vertices
                        r += std::pow(remaining[source(ej, mMultiplexSetGraph) - m], 2);
                    }
                    if (w < r) {
                        j = i;
                        w = r;
                    }
                }
            }
            assert (w > 0);
            chosen_set[j] = 0;
            for (const auto ej : make_iterator_range(in_edges(j, mMultiplexSetGraph))) {
                remaining[source(ej, mMultiplexSetGraph) - m]++;
            }
        }
    }

    for (unsigned i = 0; i != m; ++i) {
        SetIterator ei, ei_end;
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
 * @brief eliminateSubsetConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::eliminateSubsetConstraints() {
    // If Ai ⊂ Aj then the subset graph will contain the arc (i, j). Remove all arcs corresponding to vertices
    // that are not elements of the same multiplexing set.
    SubsetEdgeIterator ei, ei_end, ei_next;
    std::tie(ei, ei_end) = edges(mSubsetGraph);
    for (ei_next = ei; ei != ei_end; ei = ei_next) {
        ++ei_next;
        const auto u = source(*ei, mSubsetGraph);
        const auto v = target(*ei, mSubsetGraph);
        if (in_degree(u, mMultiplexSetGraph) != 0 && in_degree(v, mMultiplexSetGraph) != 0) {
            assert (in_degree(u, mMultiplexSetGraph) == 1);
            const auto su = source(*(in_edges(u, mMultiplexSetGraph).first), mMultiplexSetGraph);
            assert (in_degree(v, mMultiplexSetGraph) == 1);
            const auto sv = source(*(in_edges(v, mMultiplexSetGraph).first), mMultiplexSetGraph);
            if (su == sv) {
                continue;
            }
        }
        remove_edge(*ei, mSubsetGraph);
    }

    if (num_edges(mSubsetGraph) != 0) {

        // At least one subset constraint exists; perform a transitive reduction on the graph to ensure that
        // we perform the minimum number of AST modifications for the selected multiplexing sets.

        doTransitiveReductionOfSubsetGraph();

        // Afterwards modify the AST to ensure that multiplexing algorithm can ignore any subset constraints
        for (auto e : make_iterator_range(edges(mSubsetGraph))) {
            Advance * const adv1 = mAdvance[source(e, mSubsetGraph)];
            Advance * const adv2 = mAdvance[target(e, mSubsetGraph)];
            assert (adv1->getParent() == adv2->getParent());
            PabloBlock * const pb = adv1->getParent();
            pb->setInsertPoint(adv2->getPrevNode());
            adv2->setOperand(0, pb->createAnd(adv2->getOperand(0), pb->createNot(adv1->getOperand(0)), "subset"));
            pb->setInsertPoint(adv2);
            adv2->replaceAllUsesWith(pb->createOr(adv1, adv2, "merge"));
        }

    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief multiplexSelectedSets
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::multiplexSelectedSets(PabloFunction &) {
    const auto first_set = num_vertices(mConstraintGraph);
    const auto last_set = num_vertices(mMultiplexSetGraph);
    for (auto idx = first_set; idx != last_set; ++idx) {
        const size_t n = out_degree(idx, mMultiplexSetGraph);
        if (n) {
            const size_t m = log2_plus_one(n);
            Advance * input[n];
            Advance * muxed[m];
            // The multiplex set graph is a DAG with edges denoting the set relationships of our independent sets.
            unsigned i = 0;
            for (const auto u : orderMultiplexSet(idx)) {
                input[i++] = mAdvance[u];
            }
            Advance * const adv = input[0];
            PabloBlock * const block = adv->getParent(); assert (block);
            block->setInsertPoint(nullptr);
            /// Perform n-to-m Multiplexing
            for (size_t j = 0; j != m; ++j) {
                std::ostringstream prefix;
                prefix << "mux" << n << "to" << m << '.' << (j + 1);
                Or * muxing = block->createOr(n);
                for (size_t i = 0; i != n; ++i) {
                    if (((i + 1) & (1UL << j)) != 0) {
                        assert (input[i]->getParent() == block);
                        muxing->addOperand(input[i]->getOperand(0));
                    }
                }
                muxed[j] = cast<Advance>(block->createAdvance(muxing, adv->getOperand(1), prefix.str()));
            }
            /// Perform m-to-n Demultiplexing
            for (size_t i = 0; i != n; ++i) {
                // Construct the demuxed values and replaces all the users of the original advances with them.                
                PabloAST * demuxing[m];
                for (size_t j = 0; j != m; ++j) {
                    demuxing[j] = muxed[j];
                    if (((i + 1) & (1UL << j)) == 0) {
                        demuxing[j] = block->createNot(muxed[j]);
                    }
                }
                And * demuxed = block->createAnd(m);
                for (size_t j = 0; j != m; ++j) {
                    demuxed->addOperand(demuxing[j]);
                }
                input[i]->replaceWith(demuxed, true, true);
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief orderMultiplexSet
 ** ------------------------------------------------------------------------------------------------------------- */
inline MultiplexingPass::MultiplexVector MultiplexingPass::orderMultiplexSet(const MultiplexSetGraph::vertex_descriptor u) {
    MultiplexVector set;
    set.reserve(out_degree(u, mMultiplexSetGraph));
    for (const auto e : make_iterator_range(out_edges(u, mMultiplexSetGraph))) {
        set.push_back(target(e, mMultiplexSetGraph));
    }
    std::sort(set.begin(), set.end());
    MultiplexVector clique;
    MultiplexVector result;
    result.reserve(out_degree(u, mMultiplexSetGraph));
    while (set.size() > 0) {
        const auto v = *set.begin();
        clique.push_back(v);
        set.erase(set.begin());
        for (const auto w : make_iterator_range(adjacent_vertices(v, mUsageGraph))) {
            auto f = std::lower_bound(set.begin(), set.end(), w);
            // Is w in our multiplexing set?
            if (f == set.end() || *f != w) {
                continue;
            }
            // Is our subgraph still a clique after adding w to it?
            bool valid = true;
            for (const auto y : clique) {
                if (!edge(w, y, mUsageGraph).second) {
                    valid = false;
                    break;
                }
            }
            if (valid) {
                clique.push_back(w);
                set.erase(f);
            }
        }
        result.insert(result.end(), clique.begin(), clique.end());
        clique.clear();
    }
    return result;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief OrderingVerifier
 ** ------------------------------------------------------------------------------------------------------------- */
struct OrderingVerifier {
    OrderingVerifier() : mParent(nullptr) {}
    OrderingVerifier(const OrderingVerifier & parent) : mParent(&parent) {}
    bool count(const PabloAST * expr) const {
        if (mSet.count(expr)) {
            return true;
        } else if (mParent) {
            return mParent->count(expr);
        }
        return false;
    }
    void insert(const PabloAST * expr) {
        mSet.insert(expr);
    }
private:
    const OrderingVerifier * const mParent;
    std::unordered_set<const PabloAST *> mSet;
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief topologicalSort
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::topologicalSort(PabloFunction & function) {
    OrderingVerifier set;
    set.insert(PabloBlock::createZeroes());
    set.insert(PabloBlock::createOnes());
    for (unsigned i = 0; i != function.getNumOfParameters(); ++i) {
        set.insert(function.getParameter(i));
    }
    topologicalSort(function.getEntryBlock(), set);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief topologicalSort
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::topologicalSort(PabloBlock * block, OrderingVerifier & parent) {
    OrderingVerifier encountered(parent);
    for (Statement * stmt = block->front(); stmt; ) {
        if (LLVM_UNLIKELY(isa<If>(stmt))) {
            topologicalSort(cast<If>(stmt)->getBody(), encountered);
            for (Assign * def : cast<If>(stmt)->getDefined()) {
                encountered.insert(def);
            }
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            topologicalSort(cast<While>(stmt)->getBody(), encountered);
            for (Next * var : cast<While>(stmt)->getVariants()) {
                encountered.insert(var);
            }
        }
        bool unmodified = true;
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            PabloAST * const op = stmt->getOperand(i);
            if (LLVM_UNLIKELY(encountered.count(op) == 0 && isa<Statement>(op))) {
                Statement * const next = stmt->getNextNode();
                Statement * pos = cast<Statement>(op);
                if (cast<Statement>(op)->getParent() != block) {
                    // If we haven't already encountered the Assign or Next node, it must come from a If or
                    // While node that we haven't processed yet. Scan ahead and try to locate it.
                    pos = nullptr;
                    if (isa<Assign>(pos)) {
                        for (pos = cast<Statement>(op); pos; pos = pos->getNextNode()) {
                            if (LLVM_UNLIKELY(isa<If>(pos))) {
                                const auto & defs = cast<If>(pos)->getDefined();
                                if (LLVM_LIKELY(std::find(defs.begin(), defs.end(), op) != defs.end())) {
                                    break;
                                }
                            }
                        }
                    } else if (isa<Next>(pos)) {
                        for (pos = cast<Statement>(op); pos; pos = pos->getNextNode()) {
                            if (LLVM_UNLIKELY(isa<While>(pos))) {
                                const auto & vars = cast<While>(pos)->getVariants();
                                if (LLVM_LIKELY(std::find(vars.begin(), vars.end(), op) != vars.end())) {
                                    break;
                                }
                            }
                        }
                    }
                    if (LLVM_UNLIKELY(pos == nullptr)) {
                        throw std::runtime_error("Unexpected error: MultiplexingPass failed to topologically sort the function!");
                    }
                }
                stmt->insertAfter(pos);
                stmt = next;
                unmodified = false;
                break;
            }
        }
        if (unmodified) {
            encountered.insert(stmt);
            stmt = stmt->getNextNode();
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief doTransitiveReductionOfSubsetGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::doTransitiveReductionOfSubsetGraph() {
    std::vector<SubsetGraph::vertex_descriptor> Q;
    for (auto u : make_iterator_range(vertices(mSubsetGraph))) {
        if (in_degree(u, mSubsetGraph) == 0 && out_degree(u, mSubsetGraph) != 0) {
            Q.push_back(u);
        }
    }
    flat_set<SubsetGraph::vertex_descriptor> targets;
    flat_set<SubsetGraph::vertex_descriptor> visited;
    do {
        const auto u = Q.back(); Q.pop_back();
        for (auto ei : make_iterator_range(out_edges(u, mSubsetGraph))) {
            for (auto ej : make_iterator_range(out_edges(target(ei, mSubsetGraph), mSubsetGraph))) {
                targets.insert(target(ej, mSubsetGraph));
            }
        }
        for (auto v : targets) {
            remove_edge(u, v, mSubsetGraph);
        }
        for (auto e : make_iterator_range(out_edges(u, mSubsetGraph))) {
            const auto v = target(e, mSubsetGraph);
            if (visited.insert(v).second) {
                Q.push_back(v);
            }
        }
    } while (Q.size() > 0);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief findMaximalCliques
 *
 * Adaptation of the Bron-Kerbosch algorithm.
 ** ------------------------------------------------------------------------------------------------------------- */
inline MultiplexingPass::CliqueSets MultiplexingPass::findMaximalCliques(const CliqueGraph & G) {
    CliqueSets S;
    const auto n = num_vertices(G);
    std::vector<CliqueGraph::vertex_descriptor> ordering;
    ordering.reserve(n);
    for (auto u : make_iterator_range(vertices(G))) {
        if (degree(u, G)) {
            ordering.push_back(u);
        }
    }
    CliqueSet R;
    CliqueSet P(ordering.begin(), ordering.end());    
    CliqueSet X;
    X.reserve(ordering.size());
    // compute a degeneracy ordering of G
    std::sort(ordering.begin(), ordering.end(), [&G](const CliqueGraph::vertex_descriptor i, const CliqueGraph::vertex_descriptor j){ return degree(i, G) < degree(j, G); });
    for (auto v : ordering) {
        R.insert(v);
        CliqueSet PN, XN;
        for (const auto u : make_iterator_range(adjacent_vertices(v, G))) {
            if (P.count(u)) PN.insert(u);
            if (X.count(u)) XN.insert(u);
        }
        findMaximalCliques(G, R, std::move(PN), std::move(XN), S); // ({v}, P ∩ N(v), X ∩ N(v))
        R.clear();
        P.erase(v);
        X.insert(v);
    }
    return S;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief findMaximalCliques
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::findMaximalCliques(const CliqueGraph & G, CliqueSet & R, CliqueSet && P, CliqueSet && X, CliqueSets & S) {
    if (LLVM_UNLIKELY(P.empty() && X.empty())) { // Report R as a maximal clique
        S.emplace(R.begin(), R.end());
    } else {
        // choose the pivot vertex u in P ∪ X as the vertex with the highest number of neighbors in P (Tomita et al. 2006.)
        CliqueSet N;
        CliqueGraph::degree_size_type size = 0;
        for (const CliqueGraph::vertex_descriptor u : P) {
            if (degree(u, G) >= size) {
                CliqueGraph::degree_size_type neighbours = 0;
                for (const CliqueGraph::vertex_descriptor v : make_iterator_range(adjacent_vertices(u, G))) {
                    neighbours += P.count(v);
                }
                if (size <= neighbours) {
                    if (size < neighbours) {
                        size = neighbours;
                        N.clear();
                    }
                    N.insert(u);
                }
            }
        }
        for (const CliqueGraph::vertex_descriptor u : X) {
            if (degree(u, G) >= size) {
                CliqueGraph::degree_size_type neighbours = 0;
                for (const CliqueGraph::vertex_descriptor v : make_iterator_range(adjacent_vertices(u, G))) {
                    neighbours += P.count(v);
                }
                if (size <= neighbours) {
                    if (size < neighbours) {
                        size = neighbours;
                        N.clear();
                    }
                    N.insert(u);
                }
            }
        }
        const CliqueGraph::vertex_descriptor u = *(N.nth(IntDistribution(0, N.size() - 1)(mRNG)));
        // for each vertex v in P \ N(u):
        for (auto v = P.begin(); v != P.end(); v = P.erase(v)) {
            if (edge(u, *v, G).second) continue;
            const bool added = R.insert(*v).second;
            CliqueSet PN, XN;
            for (const CliqueGraph::vertex_descriptor u : make_iterator_range(adjacent_vertices(*v, G))) {
                if (P.count(u)) PN.insert(u);
                if (X.count(u)) XN.insert(u);
            }
            findMaximalCliques(G, R, std::move(PN), std::move(XN), S); // (R ∪ {v}, P ∩ N(v), X ∩ N(v))
            if (LLVM_LIKELY(added)) R.erase(*v);
            X.insert(*v);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief get
 ** ------------------------------------------------------------------------------------------------------------- */
inline BDD & MultiplexingPass::get(const PabloAST * const expr) {
    assert (expr);
    auto f = mCharacterization.find(expr);
    assert (f != mCharacterization.end());
    return f->second;
}

} // end of namespace pablo
