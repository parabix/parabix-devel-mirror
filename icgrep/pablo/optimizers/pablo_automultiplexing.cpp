#include "pablo_automultiplexing.hpp"
#include <pablo/builder.hpp>
#include <pablo/function.h>
#include <pablo/printer_pablos.h>
#include <boost/container/flat_set.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/range/iterator_range.hpp>
#include <pablo/analysis/pabloverifier.hpp>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/builder.hpp>
#include <stack>
#include <queue>
#include <unordered_set>
#include <bdd.h>
#include <functional>

#include <llvm/Support/CommandLine.h>

using namespace llvm;
using namespace boost;
using namespace boost::container;
using namespace boost::numeric::ublas;

/// Interesting test cases:
/// ./icgrep -c -multiplexing '[\p{Lm}\p{Meetei_Mayek}]' -disable-if-hierarchy-strategy

/// ./icgrep -c -multiplexing '\p{Imperial_Aramaic}(?<!\p{Sm})' -disable-if-hierarchy-strategy


static cl::OptionCategory MultiplexingOptions("Multiplexing Optimization Options", "These options control the Pablo Multiplexing optimization pass.");

#ifdef NDEBUG
#define INITIAL_SEED_VALUE (std::random_device()())
#else
#define INITIAL_SEED_VALUE (83234827342)
#endif

static cl::opt<std::mt19937::result_type> Seed("multiplexing-seed", cl::init(INITIAL_SEED_VALUE),
                                        cl::desc("randomization seed used when performing any non-deterministic operations."),
                                        cl::cat(MultiplexingOptions));

#undef INITIAL_SEED_VALUE

static cl::opt<unsigned> SetLimit("multiplexing-set-limit", cl::init(std::numeric_limits<unsigned>::max()),
                                        cl::desc("maximum size of any candidate set."),
                                        cl::cat(MultiplexingOptions));

static cl::opt<unsigned> SelectionLimit("multiplexing-selection-limit", cl::init(100),
                                        cl::desc("maximum number of selections from any partial candidate set."),
                                        cl::cat(MultiplexingOptions));

static cl::opt<unsigned> WindowSize("multiplexing-window-size", cl::init(1),
                                        cl::desc("maximum depth difference for computing mutual exclusion of Advance nodes."),
                                        cl::cat(MultiplexingOptions));

static cl::opt<unsigned> Samples("multiplexing-samples", cl::init(1),
                                 cl::desc("number of times the Advance constraint graph is sampled to find multiplexing opportunities."),
                                 cl::cat(MultiplexingOptions));


enum SelectionStrategy {Greedy, WorkingSet};

static cl::opt<SelectionStrategy> Strategy(cl::desc("Choose set selection strategy:"),
                                             cl::values(
                                             clEnumVal(Greedy, "choose the largest multiplexing sets possible (w.r.t. the multiplexing-set-limit)."),
                                             clEnumVal(WorkingSet, "choose multiplexing sets that share common input values."),
                                             clEnumValEnd),
                                           cl::init(Greedy),
                                           cl::cat(MultiplexingOptions));

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

#ifdef PRINT_TIMING_INFORMATION
MultiplexingPass::seed_t MultiplexingPass::SEED = 0;
unsigned MultiplexingPass::NODES_ALLOCATED = 0;
unsigned MultiplexingPass::NODES_USED = 0;
#endif

using TypeId = PabloAST::ClassTypeId;

template<typename Graph>
static Graph construct(PabloBlock * const block);

template<typename Graph, typename Map>
static Graph construct(PabloBlock * const block, Map & M);

bool MultiplexingPass::optimize(PabloFunction & function, const bool independent) {

    if (LLVM_UNLIKELY(Samples < 1)) {
        return false;
    }


    LOG("Seed:                    " << Seed);

    #ifdef PRINT_TIMING_INFORMATION
    MultiplexingPass::SEED = Seed;
    MultiplexingPass::NODES_ALLOCATED = 0;
    MultiplexingPass::NODES_USED = 0;
    #endif

    MultiplexingPass mp(Seed);
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

    #ifdef PRINT_TIMING_INFORMATION
    MultiplexingPass::NODES_ALLOCATED = bdd_getallocnum();
    MultiplexingPass::NODES_USED = bdd_getnodenum();
    #endif

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
        if (Strategy == SelectionStrategy::Greedy) {
            mp.selectMultiplexSetsGreedy();
        } else if (Strategy == SelectionStrategy::WorkingSet) {
            mp.selectMultiplexSetsWorkingSet();
        }
        RECORD_TIMESTAMP(end_select_multiplex_sets);
        LOG("SelectMultiplexSets:      " << (end_select_multiplex_sets - start_select_multiplex_sets));

        RECORD_TIMESTAMP(start_subset_constraints);
        mp.eliminateSubsetConstraints();
        RECORD_TIMESTAMP(end_subset_constraints);
        LOG("ApplySubsetConstraints:   " << (end_subset_constraints - start_subset_constraints));

        RECORD_TIMESTAMP(start_select_independent_sets);
        mp.multiplexSelectedSets(function);
        RECORD_TIMESTAMP(end_select_independent_sets);
        LOG("MultiplexSelectedSets:    " << (end_select_independent_sets - start_select_independent_sets));

        #ifndef NDEBUG
        PabloVerifier::verify(function, "post-multiplexing");
        #endif

        Simplifier::optimize(function);
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
    bdd_autoreorder(BDD_REORDER_SIFT); // BDD_REORDER_SIFT

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
                add_edge(j, i, true, mConstraintGraph);
            }
        }
        for (unsigned j = i + 1; j < advances; ++j) {
            if (G(i, j)) {
                add_edge(j, i, true, mConstraintGraph);
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
    mAdvanceRank.resize(advances, 0);
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
                        depth = std::max<int>(depth, mAdvanceRank[i]);
                    }
                }
                mAdvanceRank[k++] = ++depth;
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

    LOG("Window Size / Max Depth: " << WindowSize << " of " << maxDepth);
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
            for (const Assign * def : cast<If>(stmt)->getDefined()) {
                if (LLVM_LIKELY(escapes(def))) {
                    bdd_addref(get(def));
                }
            }
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            characterize(cast<While>(stmt)->getBody());
            for (const Next * var : cast<While>(stmt)->getVariants()) {
                if (LLVM_LIKELY(escapes(var))) {
                    BDD & initial = get(var->getInitial());
                    BDD & escaped = get(var);
                    initial = bdd_addref(bdd_or(initial, escaped));
                    escaped = initial;
                }
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
                    const auto j = source(e, mSubsetGraph);
                    if (mSubsetImplicationsAdhereToWindowingSizeConstraint && exceedsWindowSize(j, k)) {
                        continue;
                    }
                    unconstrained[j] = true;
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
                    if (mSubsetImplicationsAdhereToWindowingSizeConstraint && exceedsWindowSize(j, k)) {
                        continue;
                    }
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
                    if (mSubsetImplicationsAdhereToWindowingSizeConstraint && exceedsWindowSize(j, k)) {
                        continue;
                    }
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
            // This algorithm deems two streams mutually exclusive if and only if the conjuntion of their BDDs is a contradiction.
            // To generate a contradiction when comparing Advances, the BDD of each Advance is represented by the conjunction of
            // variables representing the k-th Advance and the negation of all variables for the Advances whose inputs are mutually
            // exclusive with the k-th input.

            // For example, if the input of the i-th Advance is mutually exclusive with the input of the j-th and k-th Advance, the
            // BDD of the i-th Advance is Ai ∧ ¬Aj ∧ ¬Ak. Similarly, the j- and k-th Advance is Aj ∧ ¬Ai and Ak ∧ ¬Ai, respectively
            // (assuming that the j-th and k-th Advance are not mutually exclusive.)

            const BDD Ni = mAdvanceNegatedVariable[i];
            BDD & Ai = get(mAdvance[i]);
            Ai = bdd_addref(bdd_and(Ai, Nk));
            Ak = bdd_addref(bdd_and(Ak, Ni));
            if (independent(i, k) && (adv->getParent() == mAdvance[i]->getParent())) {
                continue;
            }
        }
        add_edge(i, k, false, mConstraintGraph);
    }
    // To minimize the number of BDD computations, we store the negated variable instead of negating it each time.
    mAdvanceNegatedVariable.emplace_back(Nk);
    return Ak;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief independent
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool MultiplexingPass::independent(const ConstraintVertex i, const ConstraintVertex j) const {
    assert (i < num_vertices(mConstraintGraph) && j < num_vertices(mConstraintGraph));
    return mConstraintGraph.get_edge(i, j).first == false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief exceedsWindowSize
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool MultiplexingPass::exceedsWindowSize(const ConstraintVertex i, const ConstraintVertex j) const {
    assert (i < mAdvanceRank.size() && j < mAdvanceRank.size());
    return (std::abs<int>(mAdvanceRank[i] - mAdvanceRank[j]) > WindowSize);
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
 * @brief generateCandidateSets
 ** ------------------------------------------------------------------------------------------------------------- */
bool MultiplexingPass::generateCandidateSets() {

    Constraints S;

    ConstraintGraph::degree_size_type D[num_vertices(mConstraintGraph)];

    mCandidateGraph = CandidateGraph(num_vertices(mConstraintGraph));

    for (unsigned r = Samples; r; --r) {

        // Push all source nodes into the (initial) independent set S
        for (const auto v : make_iterator_range(vertices(mConstraintGraph))) {
            const auto d = in_degree(v, mConstraintGraph);
            D[v] = d;
            if (d == 0) {
                S.push_back(v);
            }
        }

        auto remaining = num_vertices(mConstraintGraph) - S.size();

        for (;;) {
            assert (S.size() > 0);
            addCandidateSet(S);
            if (LLVM_UNLIKELY(remaining == 0)) {
                break;
            }
            for (;;) {
                assert (S.size() > 0);
                // Randomly choose a vertex in S and discard it.
                const auto i = S.begin() + IntDistribution(0, S.size() - 1)(mRNG);
                assert (i != S.end());
                const auto u = *i;
                S.erase(i);
                bool checkCandidate = false;
                for (auto e : make_iterator_range(out_edges(u, mConstraintGraph))) {
                    const auto v = target(e, mConstraintGraph);
                    assert ("Constraint set degree subtraction error!" && (D[v] != 0));
                    if ((--D[v]) == 0) {
                        assert ("Error v is already in S!" && std::count(S.begin(), S.end(), v) == 0);
                        S.push_back(v);
                        assert (remaining != 0);
                        --remaining;
                        if (LLVM_LIKELY(S.size() >= 3)) {
                            checkCandidate = true;
                        }
                    }
                }
                if (checkCandidate || LLVM_UNLIKELY(remaining == 0)) {
                    break;
                }
            }
        }

        S.clear();
    }

    #ifdef PRINT_DEBUG_OUTPUT
    const auto n = num_vertices(mConstraintGraph);
    const auto m = num_vertices(mCandidateGraph);
    unsigned sets = 0;
    for (auto i = n; i < m; ++i) {
        if (degree(i, mCandidateGraph) > 0) {
            ++sets;
        }
    }
    LOG("Unique Candidate Sets:    " << (sets));
    #endif

    return num_vertices(mCandidateGraph) > num_vertices(mConstraintGraph);
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
void MultiplexingPass::selectCandidateSet(const unsigned n, const unsigned k, const unsigned m, const Constraints & S, ConstraintVertex * const element) {
    unsigned long a = n;
    unsigned long b = k;
    unsigned long x = (choose(n, k) - 1) - m;
    for (unsigned i = 0; i != k; ++i) {
        unsigned long y = 0;
        while ((y = choose(--a, b)) > x);
        x = x - y;
        b = b - 1;
        element[i] = S[(n - 1) - a];
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateCandidateSet
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::updateCandidateSet(ConstraintVertex * const begin, ConstraintVertex * const end) {

    using Vertex = CandidateGraph::vertex_descriptor;

    const auto n = num_vertices(mConstraintGraph);
    const auto m = num_vertices(mCandidateGraph);
    const auto d = end - begin;

    std::sort(begin, end);

    Vertex u = 0;

    for (Vertex i = n; i != m; ++i) {

        if (LLVM_UNLIKELY(degree(i, mCandidateGraph) == 0)) {
            u = i;
            continue;
        }

        const auto adj = adjacent_vertices(i, mCandidateGraph);
        if (degree(i, mCandidateGraph) < d) {
            // set_i can only be a subset of the new set
            if (LLVM_UNLIKELY(std::includes(begin, end, adj.first, adj.second))) {
                clear_vertex(i, mCandidateGraph);
                u = i;
            }
        } else if (LLVM_UNLIKELY(std::includes(adj.first, adj.second, begin, end))) {
            // the new set is a subset of set_i; discard it.
            return;
        }

    }

    if (LLVM_LIKELY(u == 0)) { // n must be at least 3 so u is 0 if and only if we're not reusing a set vertex.
        u = add_vertex(mCandidateGraph);
    }

    for (ConstraintVertex * i = begin; i != end; ++i) {
        add_edge(u, *i, mCandidateGraph);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addCandidateSet
 * @param S an independent set
 ** ------------------------------------------------------------------------------------------------------------- */
inline void MultiplexingPass::addCandidateSet(const Constraints & S) {
    if (S.size() >= 3) {
        const unsigned setLimit = SetLimit;
        if (S.size() <= setLimit) {
            ConstraintVertex E[S.size()];
            std::copy(S.cbegin(), S.cend(), E);
            updateCandidateSet(E, E + S.size());
        } else {
            assert (setLimit > 0);
            ConstraintVertex E[setLimit];
            const auto max = choose(S.size(), setLimit);
            if (LLVM_UNLIKELY(max <= SelectionLimit)) {
                for (unsigned i = 0; i != max; ++i) {
                    selectCandidateSet(S.size(), setLimit, i, S, E);
                    updateCandidateSet(E, E + setLimit);
                }
            } else { // take m random samples
                for (unsigned i = 0; i != SelectionLimit; ++i) {
                    selectCandidateSet(S.size(), setLimit, mRNG() % max, S, E);
                    updateCandidateSet(E, E + setLimit);
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
 * @brief selectMultiplexSetsGreedy
 *
 * This algorithm is simply computes a greedy set cover. We want an exact max-weight set cover but can generate new
 * sets by taking a subset of any existing set. With a few modifications, the greedy approach seems to work well
 * enough but can be shown to produce a suboptimal solution if there are three candidate sets labelled A, B and C,
 * in which A ∩ B = ∅, |A| ≤ |B| < |C|, and C ⊂ (A ∪ B).
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::selectMultiplexSetsGreedy() {

    using AdjIterator = graph_traits<CandidateGraph>::adjacency_iterator;
    using degree_t = CandidateGraph::degree_size_type;
    using vertex_t = CandidateGraph::vertex_descriptor;

    const size_t m = num_vertices(mConstraintGraph);
    const size_t n = num_vertices(mCandidateGraph) - m;

    std::vector<bool> chosen(n, false);

    for (;;) {

        // Choose the set with the greatest number of vertices not already included in some other set.
        vertex_t u = 0;
        degree_t w = 0;
        for (vertex_t i = 0; i != n; ++i) {
            if (chosen[i]) continue;
            const auto t = i + m;
            degree_t r = degree(t, mCandidateGraph);
            if (LLVM_LIKELY(r >= 3)) { // if this set has at least 3 elements.
                r *= r;
                AdjIterator begin, end;
                std::tie(begin, end) = adjacent_vertices(t, mCandidateGraph);
                for (auto ei = begin; ei != end; ++ei) {
                    for (auto ej = ei; ++ej != end; ) {
                        if (edge(*ei, *ej, mUsageGraph).second) {
                            ++r;
                        }
                    }
                }
                if (w < r) {
                    u = t;
                    w = r;
                }
            } else if (r) {
                clear_vertex(t, mCandidateGraph);
            }
        }

        // Multiplexing requires 3 or more elements; if no set contains at least 3, abort.
        if (LLVM_UNLIKELY(w == 0)) {
            break;
        }

        chosen[u - m] = true;

        // If this contains 2^n elements for any n, discard the member that is most likely to be added
        // to some future set.
        if (LLVM_UNLIKELY(is_power_of_2(degree(u, mCandidateGraph)))) {
            vertex_t x = 0;
            degree_t w = 0;
            for (const auto v : make_iterator_range(adjacent_vertices(u, mCandidateGraph))) {
                if (degree(v, mCandidateGraph) > w) {
                    x = v;
                    w = degree(v, mCandidateGraph);
                }
            }
            remove_edge(u, x, mCandidateGraph);
        }

        AdjIterator begin, end;
        std::tie(begin, end) = adjacent_vertices(u, mCandidateGraph);
        for (auto vi = begin; vi != end; ) {
            const auto v = *vi++;
            clear_vertex(v, mCandidateGraph);
            add_edge(v, u, mCandidateGraph);
        }

        if (Samples > 1) {
            removePotentialCycles(u);
        }
    }

    #ifndef NDEBUG
    for (unsigned i = 0; i != m; ++i) {
        assert (degree(i, mCandidateGraph) <= 1);
    }
    for (unsigned i = m; i != (m + n); ++i) {
        assert (degree(i, mCandidateGraph) == 0 || degree(i, mCandidateGraph) >= 3);
    }
    #endif
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief selectMultiplexSetsWorkingSet
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::selectMultiplexSetsWorkingSet() {

    // The inputs to each Advance must be different; otherwise the SimplificationPass would consider all but
    // one of the Advances redundant. However, if the input is short lived, we can ignore it in favour of its
    // operands, which *may* be shared amongst more than one of the Advances (or may be short lived themselves,
    // in which we can consider their operands instead.) Ideally, if we can keep the set of live values small,
    // we may be able to reduce register pressure.

//    using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, Statement *, unsigned>;
//    using Map = flat_map<const Statement *, typename Graph::vertex_descriptor>;

//    const size_t m = num_vertices(mConstraintGraph);
//    const size_t n = num_vertices(mMultiplexSetGraph) - m;

//    for (unsigned i = 0; i != n; ++i) {

//        Map M;
//        Graph G = construct<Graph>(block, M);







//    }






}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief removePotentialCycles
 *
 * If Samples > 1, our candidate sets were generated by more than one traversal through the constraint DAG.
 * Multiplexing disjoint sets generated by differing traversals can induce a cycle in the AST. For example,
 * suppose sets {A,B} and {C,D} and A is dependent on C and D on B; multiplexing both will result in a cycle.
 *
 * Eliminating all potential cycles will likely lead to the removal of many candidate sets. Instead we "fix"
 * the candidate sets after the selection of a particular candidate set.
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::removePotentialCycles(const CandidateGraph::vertex_descriptor i) {

    using AdjIterator = graph_traits<CandidateGraph>::adjacency_iterator;

    const auto m = num_vertices(mConstraintGraph);
    const auto n = num_vertices(mCandidateGraph);

    // Suppose we construct a graph G that indicates whether selecting candidate set V will induce a cycle, given
    // that we've already chosen candidate set U. This can occur here only because some elements of V are dependent
    // on U and vice versa.

    // We want the minimal minimum weight feedback arc set of G; however, we also know any edge will either have
    //

    for (auto j = m; j < n; ++j) {
        if (LLVM_UNLIKELY(i == j)) continue;
        AdjIterator begin, end;
        std::tie(begin, end) = adjacent_vertices(j, mCandidateGraph);
        for (auto ui = begin; ui != end; )  {
            const auto u = *ui++;
            unsigned outgoing = 0;
            unsigned incoming = 0;
            for (auto v : make_iterator_range(adjacent_vertices(i, mCandidateGraph)))  {
                if (dependent(u, v)) {
                    ++outgoing;
                } else if (dependent(v, u)) {
                    ++incoming;
                }
            }
            if (LLVM_UNLIKELY(outgoing > 0 && incoming > 0)) {
                remove_edge(j, u, mCandidateGraph);
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief dependent
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool MultiplexingPass::dependent(const ConstraintVertex i, const ConstraintVertex j) const {
    const auto e = mConstraintGraph.get_edge(i, j);
    return (e.second && e.first);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief eliminateSubsetConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::eliminateSubsetConstraints() {
    using SubsetEdgeIterator = graph_traits<SubsetGraph>::edge_iterator;
    // If Ai ⊂ Aj then the subset graph will contain the arc (i, j). Remove all arcs corresponding to vertices
    // that are not elements of the same multiplexing set.
    SubsetEdgeIterator ei, ei_end, ei_next;
    std::tie(ei, ei_end) = edges(mSubsetGraph);
    for (ei_next = ei; ei != ei_end; ei = ei_next) {
        ++ei_next;
        const auto u = source(*ei, mSubsetGraph);
        const auto v = target(*ei, mSubsetGraph);
        if (degree(u, mCandidateGraph) != 0 && degree(v, mCandidateGraph) != 0) {
            assert (degree(u, mCandidateGraph) == 1);
            assert (degree(v, mCandidateGraph) == 1);
            const auto su = *(adjacent_vertices(u, mCandidateGraph).first);
            const auto sv = *(adjacent_vertices(v, mCandidateGraph).first);
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
void MultiplexingPass::multiplexSelectedSets(PabloFunction & function) {
    flat_set<PabloBlock *> modified;
    const auto first_set = num_vertices(mConstraintGraph);
    const auto last_set = num_vertices(mCandidateGraph);
    for (auto idx = first_set; idx != last_set; ++idx) {
        const size_t n = degree(idx, mCandidateGraph);
        assert (n == 0 || n > 2);
        if (n) {
            const size_t m = log2_plus_one(n);
            Advance * input[n];
            PabloAST * muxed[m];
            PabloAST * muxed_n[m];
            // The multiplex set graph is a DAG with edges denoting the set relationships of our independent sets.
            unsigned i = 0;
            for (const auto u : orderMultiplexSet(idx)) {
                input[i++] = mAdvance[u];
            }
            Advance * const adv = input[0];
            PabloBlock * const block = adv->getParent(); assert (block);
            modified.insert(block);

            circular_buffer<PabloAST *> Q(n);

            PabloBuilder builder(block);
            block->setInsertPoint(nullptr);
            /// Perform n-to-m Multiplexing            
            for (size_t j = 0; j != m; ++j) {                
                std::ostringstream prefix;
                prefix << "mux" << n << "to" << m << '.' << (j);
                assert (Q.empty());
                for (size_t i = 0; i != n; ++i) {
                    if (((i + 1) & (1UL << j)) != 0) {
                        Q.push_back(input[i]->getOperand(0));
                    }
                }
                while (Q.size() > 1) {
                    PabloAST * a = Q.front(); Q.pop_front();
                    PabloAST * b = Q.front(); Q.pop_front();
                    Q.push_back(builder.createOr(a, b));
                }
                PabloAST * const muxing =  Q.front(); Q.clear();
                muxed[j] = builder.createAdvance(muxing, adv->getOperand(1), prefix.str());
                muxed_n[j] = builder.createNot(muxed[j]);
            }
            /// Perform m-to-n Demultiplexing
            block->setInsertPoint(block->back());
            for (size_t i = 0; i != n; ++i) {
                // Construct the demuxed values and replaces all the users of the original advances with them.
                assert (Q.empty());
                for (size_t j = 0; j != m; ++j) {
                    Q.push_back((((i + 1) & (1UL << j)) != 0) ? muxed[j] : muxed_n[j]);
                }
                while (Q.size() > 1) {
                    PabloAST * const a = Q.front(); Q.pop_front();
                    PabloAST * const b = Q.front(); Q.pop_front();
                    Q.push_back(builder.createAnd(a, b));
                }
                PabloAST * const demuxed =  Q.front(); Q.clear();
                input[i]->replaceWith(demuxed, true, true);
            }
        }
    }
    for (PabloBlock * block : modified) {
        rewriteAST(block);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief orderMultiplexSet
 ** ------------------------------------------------------------------------------------------------------------- */
inline MultiplexingPass::Candidates MultiplexingPass::orderMultiplexSet(const CandidateGraph::vertex_descriptor u) {
    Candidates set;
    set.reserve(degree(u, mCandidateGraph));
    for (const auto e : make_iterator_range(adjacent_vertices(u, mCandidateGraph))) {
        set.push_back(e);
    }
    std::sort(set.begin(), set.end());
    Candidates clique;
    Candidates result;
    result.reserve(degree(u, mCandidateGraph));
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
 * @brief rewriteAST
 *
 * Multiplexing ignores def-use ordering when muxing and demuxing the Advances; this will likely lead to an illegal
 * ordering but, by virtue of the multiplexing algorithm, some ordering of the IR must be legal. However, an
 * arbritary topological ordering will likely lead to poor performance due to excessive register spills; this
 * algorithm attempts to mitigate this by using a simple bottom-up ordering scheme.
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::rewriteAST(PabloBlock * const block) {

    using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, Statement *>;
    using Vertex = Graph::vertex_descriptor;
    using ReadySet = std::vector<Vertex>;
    using TypeId = PabloAST::ClassTypeId;

    Graph G = construct<Graph>(block);


    std::vector<unsigned> rank(num_vertices(G), 0);

    {
        circular_buffer<Vertex> Q(num_vertices(G));
        // Compute the rank of each statement
        for (const Vertex u : make_iterator_range(vertices(G))) {
            if (out_degree(u, G) == 0 && rank[u] == 0) {
                Q.push_back(u);
            }
        }

        while (Q.size() > 0) {

            const Vertex u = Q.front();
            Q.pop_front();

            assert (rank[u] == 0);

            unsigned work = 0;
            switch (G[u]->getClassTypeId()) {
                case TypeId::And:
                case TypeId::Or:
                case TypeId::Xor:
                    work = 2;
                    break;
                case TypeId::Not:
                case TypeId::Assign:
                case TypeId::Next:
                    work = 1;
                    break;
                case TypeId::Sel:
                    work = 6;
                    break;
                case TypeId::Advance:
                case TypeId::ScanThru:
                    work = 33;
                    break;
                case TypeId::MatchStar:
                    work = 51;
                    break;
                case TypeId::If:
                case TypeId::While:
                case TypeId::Call:
                    work = 10000; // <-- try to push If, While and Call nodes as high as possible
                    break;
                default: break;
            }

            unsigned r = 0;
            for (const auto e : make_iterator_range(out_edges(u, G))) {
                r = std::max(r, rank[target(e, G)]);
            }

            rank[u] = work + r;

            for (const auto ei : make_iterator_range(in_edges(u, G))) {
                const auto v = source(ei, G);
                assert (rank[v] == 0);
                bool ready = true;
                for (const auto ej : make_iterator_range(out_edges(v, G))) {
                    if (rank[target(ej, G)] == 0) {
                        ready = false;
                        break;
                    }
                }
                if (ready) {
                    Q.push_back(v);
                }
            }

        }
    }

    // Compute the initial ready set
    ReadySet readySet;
    for (const Vertex u : make_iterator_range(vertices(G))) {
        if (in_degree(u, G) == 0) {
            readySet.emplace_back(u);
        }
    }

    auto by_nonincreasing_rank = [&rank](const Vertex u, const Vertex v) -> bool {
        return rank[u] > rank[v];
    };

    std::sort(readySet.begin(), readySet.end(), by_nonincreasing_rank);

    block->setInsertPoint(nullptr);
    // Rewrite the AST using the bottom-up ordering
    while (readySet.size() > 0) {
        // Scan through the ready set to determine which one 'kills' the greatest number of inputs
        double best = 0.0;
        auto rk = readySet.begin();
        for (auto ri = readySet.begin(); ri != readySet.end(); ++ri) {
            double p = 0.0;
            assert (rank[*ri] != 0);
            for (auto ei : make_iterator_range(in_edges(*ri, G))) {
                const auto v = source(ei, G);
                unsigned unscheduled = 0;
                for (auto ej : make_iterator_range(out_edges(v, G))) {
                    if (rank[target(ej, G)] != 0) { // if this edge leads to an unscheduled statement
                        ++unscheduled;
                    }
                }
                assert (unscheduled > 0);
                assert (unscheduled <= out_degree(v, G));
                const double uses = out_degree(v, G);
                p += std::pow((uses - (double)(unscheduled - 1)) / uses, 2);
            }
            if (p > best) {
                rk = ri;
                best = p;
            }
        }
        const auto u = *rk;
        readySet.erase(rk);
        // Write the statement back to the AST ...
        block->insert(G[u]);
        // ... and mark it as written
        rank[u] = 0;
        // Now check whether any new statements are ready
        for (auto ei : make_iterator_range(out_edges(u, G))) {
            bool ready = true;
            const auto v = target(ei, G);
            assert (rank[v] != 0);
            for (auto ej : make_iterator_range(in_edges(v, G))) {
                if (rank[source(ej, G)] != 0) {
                    ready = false;
                    break;
                }
            }
            if (ready) {
                readySet.insert(std::lower_bound(readySet.begin(), readySet.end(), v, by_nonincreasing_rank), v);
                assert (std::is_sorted(readySet.cbegin(), readySet.cend(), by_nonincreasing_rank));
            }
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeDAG
 ** ------------------------------------------------------------------------------------------------------------- */
template<typename Graph, typename Map>
Graph construct(PabloBlock * const block, Map & M) {

    using Vertex = typename Graph::vertex_descriptor;

    const auto size = std::distance(block->begin(), block->end());

    Graph G(size);
    M.reserve(size);

    Vertex u = 0;
    for (Statement * stmt : *block ) {
        G[u] = stmt;
        M.emplace(stmt, u);
        if (LLVM_UNLIKELY(isa<If>(stmt))) {
            for (Assign * def : cast<If>(stmt)->getDefined()) {
                M.emplace(def, u);
            }
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            for (Next * var : cast<While>(stmt)->getVariants()) {
                M.emplace(var, u);
            }
        }
        ++u;
    }

    /// The following is a lamda function that adds any users of "stmt" to the graph after resolving
    /// which vertex it maps to w.r.t. the current block.
    auto addUsers = [&](const Vertex u, const Statement * const stmt) -> void {
        for (const PabloAST * user : stmt->users()) {
            if (LLVM_LIKELY(isa<Statement>(user))) {
                const Statement * use = cast<Statement>(user);
                auto f = M.find(use);
                if (LLVM_UNLIKELY(f == M.end())) {
                    const PabloBlock * parent = use->getParent();
                    for (;;) {
                        if (parent == block) {
                            break;
                        }
                        use = parent->getBranch();
                        parent = parent->getParent();
                        if (parent == nullptr) {
                            return;
                        }
                    }
                    f = M.find(use);
                    assert (f != M.end());
                    M.emplace(use, f->second);
                }
                const auto v = f->second;
                if (LLVM_UNLIKELY(u != v)) {
                    add_edge(u, v, G);
                }
            }
        }
    };

    u = 0;
    for (Statement * stmt : *block ) {

        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            PabloAST * const op = stmt->getOperand(i);
            if (isa<Statement>(op)) {
                auto f = M.find(cast<Statement>(op));
                if (f != M.end()) {
                    add_edge(f->second, u, G);
                }
            }
        }

        if (LLVM_UNLIKELY(isa<If>(stmt))) {
            for (Assign * def : cast<If>(stmt)->getDefined()) {
                addUsers(u, def);
            }
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            for (Next * var : cast<While>(stmt)->getVariants()) {
                addUsers(u, var);
            }
        } else {
            addUsers(u, stmt);
        }

        ++u;
    }

    #ifndef NDEBUG
    std::vector<Vertex> nothing;
    topological_sort(G, std::back_inserter(nothing));
    #endif

    return G;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeDAG
 ** ------------------------------------------------------------------------------------------------------------- */
template<typename Graph>
Graph construct(PabloBlock * const block) {
    using Map = flat_map<const Statement *, typename Graph::vertex_descriptor>;
    Map M;
    return construct<Graph, Map>(block, M);
}

} // end of namespace pablo
