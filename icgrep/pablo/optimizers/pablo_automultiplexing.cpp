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

namespace pablo {

using TypeId = PabloAST::ClassTypeId;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 * @param function the function to optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool MultiplexingPass::optimize(PabloFunction & function) {

    if (LLVM_UNLIKELY(Samples < 1)) {
        return false;
    }

    PabloVerifier::verify(function, "pre-multiplexing");

    Z3_config cfg = Z3_mk_config();
    Z3_context ctx = Z3_mk_context_rc(cfg);
    Z3_del_config(cfg);
    Z3_solver solver = Z3_mk_solver(ctx);
    Z3_solver_inc_ref(ctx, solver);

    MultiplexingPass mp(function, Seed, ctx, solver);

    mp.characterize(function);

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);

    PabloVerifier::verify(function, "post-multiplexing");

    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 * @param function the function to optimize
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::characterize(PabloFunction & function) {
    // Map the constants and input variables
    Z3_sort boolTy = Z3_mk_bool_sort(mContext);

    Z3_ast F = Z3_mk_const(mContext, Z3_mk_int_symbol(mContext, 0), boolTy);
    Z3_inc_ref(mContext, F);
    add(PabloBlock::createZeroes(), F);

    Z3_ast T = Z3_mk_const(mContext, Z3_mk_int_symbol(mContext, 1), boolTy);
    Z3_inc_ref(mContext, T);
    add(PabloBlock::createOnes(), T);

    for (unsigned i = 0; i < function.getNumOfParameters(); ++i) {
        make(function.getParameter(i));
    }

    characterize(function.getEntryBlock());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::characterize(PabloBlock * const block) {
    Statement * end = initialize(block->front());
    for (Statement * stmt : *block) {
        if (LLVM_UNLIKELY(stmt == end)) {
            Statement * const next = stmt->getNextNode();
            multiplex(block);
            if (isa<If>(stmt)) {
                characterize(cast<If>(stmt)->getBody());
            } else if (isa<While>(stmt)) {
                for (const Next * var : cast<While>(stmt)->getVariants()) {
                    Z3_inc_ref(mContext, get(var->getInitial()));
                }
                characterize(cast<While>(stmt)->getBody());
                // since we cannot be certain that we'll always execute at least one iteration of a loop, we must
                // assume that the variants could either be their initial value or their resulting value.
                for (const Next * var : cast<While>(stmt)->getVariants()) {
                    Z3_ast v0 = get(var->getInitial());
                    Z3_ast & v1 = get(var);
                    Z3_ast merge[2] = { v0, v1 };
                    Z3_ast r = Z3_mk_or(mContext, 2, merge);
                    Z3_inc_ref(mContext, r);
                    Z3_dec_ref(mContext, v0);
                    Z3_dec_ref(mContext, v1);
                    v1 = r;
                    assert (get(var) == r);
                }
            }
            end = initialize(next);
        } else {
            characterize(stmt);
        }
    }
    multiplex(block);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief multiplex
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::multiplex(PabloBlock * const block) {
    if (generateCandidateSets()) {
        selectMultiplexSetsGreedy();
        eliminateSubsetConstraints();
        multiplexSelectedSets(block);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief equals
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool MultiplexingPass::equals(Z3_ast a, Z3_ast b) {
    Z3_solver_push(mContext, mSolver);
    Z3_ast test = Z3_mk_eq(mContext, a, b);
    Z3_inc_ref(mContext, test);
    Z3_solver_assert(mContext, mSolver, test);
    const auto r = Z3_solver_check(mContext, mSolver);
    Z3_dec_ref(mContext, test);
    Z3_solver_pop(mContext, mSolver, 1);
    return (r == Z3_L_TRUE);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief handle_unexpected_statement
 ** ------------------------------------------------------------------------------------------------------------- */
static void handle_unexpected_statement(Statement * const stmt) {
    std::string tmp;
    raw_string_ostream err(tmp);
    err << "Unexpected statement type: ";
    PabloPrinter::print(stmt, err);
    throw std::runtime_error(err.str());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
inline Z3_ast MultiplexingPass::characterize(Statement * const stmt) {

    const size_t n = stmt->getNumOperands(); assert (n > 0);
    Z3_ast operands[n] = {};
    for (size_t i = 0; i < n; ++i) {
        PabloAST * op = stmt->getOperand(i);
        if (LLVM_LIKELY(isa<Statement>(op) || isa<Var>(op))) {
            operands[i] = get(op, true);
        }
    }

    Z3_ast node = operands[0];
    switch (stmt->getClassTypeId()) {
        case TypeId::Assign:
        case TypeId::Next:
        case TypeId::AtEOF:
        case TypeId::InFile:
            node = operands[0]; break;
        case TypeId::And:
            node = Z3_mk_and(mContext, n, operands); break;
        case TypeId::Or:
            node = Z3_mk_or(mContext, n, operands); break;
        case TypeId::Xor:
            node = Z3_mk_xor(mContext, operands[0], operands[1]);
            Z3_inc_ref(mContext, node);
            for (unsigned i = 2; LLVM_UNLIKELY(i < n); ++i) {
                Z3_ast temp = Z3_mk_xor(mContext, node, operands[i]);
                Z3_inc_ref(mContext, temp);
                Z3_dec_ref(mContext, node);
                node = temp;
            }
            return add(stmt, node);
        case TypeId::Not:
            node = Z3_mk_not(mContext, node);
            break;
        case TypeId::Sel:
            node = Z3_mk_ite(mContext, operands[0], operands[1], operands[2]);
            break;
        case TypeId::Advance:
            return characterize(cast<Advance>(stmt), operands[0]);
        case TypeId::ScanThru:
            // ScanThru(c, m) := (c + m) ∧ ¬m. Thus we can conservatively represent this statement using the BDD
            // for ¬m --- provided no derivative of this statement is negated in any fashion.
        case TypeId::MatchStar:
        case TypeId::Count:
            return make(stmt);
        default:
            handle_unexpected_statement(stmt);
    }
    Z3_inc_ref(mContext, node);
    return add(stmt, node);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
inline Z3_ast MultiplexingPass::characterize(Advance * const adv, Z3_ast Ik) {
    const auto k = mAdvanceNegatedVariable.size();

    assert (adv);
    assert (mConstraintGraph[k] == adv);

    bool unconstrained[k] = {};

    Z3_solver_push(mContext, mSolver);

    for (size_t i = 0; i < k; ++i) {

        // Have we already proven that they are unconstrained by their subset relationship?
        if (unconstrained[i]) continue;

        // If these Advances are mutually exclusive, in the same scope, transitively independent, and shift their
        // values by the same amount, we can safely multiplex them. Otherwise mark the constraint in the graph.
        const Advance * const ithAdv = mConstraintGraph[i];
        if (ithAdv->getOperand(1) == adv->getOperand(1)) {

            Z3_ast Ii = get(ithAdv->getOperand(0));

            // Is there any satisfying truth assignment? If not, these streams are mutually exclusive.

            Z3_solver_push(mContext, mSolver);
            Z3_ast conj[2] = { Ii, Ik };
            Z3_ast IiIk = Z3_mk_and(mContext, 2, conj);
            Z3_inc_ref(mContext, IiIk);
            Z3_solver_assert(mContext, mSolver, IiIk);
            if (Z3_solver_check(mContext, mSolver) == Z3_L_FALSE) {
                // If Ai ∩ Ak = ∅ and Aj ⊂ Ai, Aj ∩ Ak = ∅.
                for (auto e : make_iterator_range(in_edges(i, mSubsetGraph))) {
                    unconstrained[source(e, mSubsetGraph)] = true;
                }
                unconstrained[i] = true;

            } else if (equals(Ii, IiIk)) {
                // If Ii = Ii ∩ Ik then Ii ⊆ Ik. Record this in the subset graph with the arc (i, k).
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

            } else if (equals(Ik, IiIk)) {
                // If Ik = Ii ∩ Ik then Ik ⊆ Ii. Record this in the subset graph with the arc (k, i).
                add_edge(k, i, mSubsetGraph);
                // If Ak ⊂ Ai and Ai ⊂ Aj, Ak ⊂ Aj.
                for (auto e : make_iterator_range(out_edges(i, mSubsetGraph))) {
                    const auto j = target(e, mSubsetGraph);
                    add_edge(k, j, mSubsetGraph);
                    unconstrained[j] = true;
                }
                unconstrained[i] = true;
            }

            Z3_dec_ref(mContext, IiIk);
            Z3_solver_pop(mContext, mSolver, 1);
        }
    }

    Z3_solver_pop(mContext, mSolver, 1);

    Z3_ast Ak0 = make(adv);
    Z3_inc_ref(mContext, Ak0);
    Z3_ast Nk = Z3_mk_not(mContext, Ak0);
    Z3_inc_ref(mContext, Nk);

    Z3_ast vars[k + 1];
    vars[0] = Ak0;

    unsigned m = 1;
    for (unsigned i = 0; i < k; ++i) {
        if (unconstrained[i]) {
            // This algorithm deems two streams mutually exclusive if and only if their conjuntion is a contradiction.
            // To generate a contradiction when comparing Advances, the BDD of each Advance is represented by the conjunction of
            // variables representing the k-th Advance and the negation of all variables for the Advances whose inputs are mutually
            // exclusive with the k-th input.

            // For example, if the input of the i-th Advance is mutually exclusive with the input of the j-th and k-th Advance, the
            // BDD of the i-th Advance is Ai ∧ ¬Aj ∧ ¬Ak. Similarly, the j- and k-th Advance is Aj ∧ ¬Ai and Ak ∧ ¬Ai, respectively
            // (assuming that the j-th and k-th Advance are not mutually exclusive.)

            Z3_ast & Ai0 = get(mConstraintGraph[i]);
            Z3_ast conj[2] = { Ai0, Nk };
            Z3_ast Ai = Z3_mk_and(mContext, 2, conj);
            Z3_inc_ref(mContext, Ai);
            Z3_dec_ref(mContext, Ai0); // if this doesn't work, we'll have to scan from the output variables.
            Ai0 = Ai;
            assert (get(mConstraintGraph[i]) == Ai);

            vars[m++] = mAdvanceNegatedVariable[i];

            continue; // note: if these Advances aren't transtively independent, an edge will still exist.
        }
        add_edge(i, k, mConstraintGraph);
    }
    // To minimize the number of BDD computations, we store the negated variable instead of negating it each time.
    mAdvanceNegatedVariable.emplace_back(Nk);
    Z3_ast Ak = Z3_mk_and(mContext, m, vars);
    if (LLVM_UNLIKELY(Ak != Ak0)) {
        Z3_inc_ref(mContext, Ak);
        Z3_dec_ref(mContext, Ak0);
    }
    return add(adv, Ak);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initialize
 ** ------------------------------------------------------------------------------------------------------------- */
Statement * MultiplexingPass::initialize(Statement * const initial) {

    // clean up any unneeded refs / characterizations.
    for (auto i = mCharacterization.begin(); i != mCharacterization.end(); ) {
        const CharacterizationRef & r = std::get<1>(*i);
        if (LLVM_UNLIKELY(std::get<1>(r) == 0)) {
            Z3_dec_ref(mContext, std::get<0>(r));
            auto j = i++;
            mCharacterization.erase(j);
        } else {
            ++i;
        }
    }

    for (Z3_ast var : mAdvanceNegatedVariable) {
        Z3_dec_ref(mContext, var);
    }
    mAdvanceNegatedVariable.clear();

    // Scan through and count all the advances and statements ...
    unsigned statements = 0, advances = 0;
    Statement * last = nullptr;
    for (Statement * stmt = initial; stmt; stmt = stmt->getNextNode()) {
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            last = stmt;
            break;
        } else if (LLVM_UNLIKELY(isa<Advance>(stmt))) {
            ++advances;
        }
        ++statements;
    }

    flat_map<const PabloAST *, unsigned> M;
    M.reserve(statements);
    matrix<bool> G(statements, advances, false);
    for (unsigned i = 0; i != advances; ++i) {
        G(i, i) = true;
    }

    mConstraintGraph = ConstraintGraph(advances);
    unsigned n = advances;
    unsigned k = 0;
    for (Statement * stmt = initial; stmt != last; stmt = stmt->getNextNode()) {
        assert (!isa<If>(stmt) && !isa<While>(stmt));
        unsigned u = 0;
        if (LLVM_UNLIKELY(isa<Advance>(stmt))) {
            mConstraintGraph[k] = cast<Advance>(stmt);
            u = k++;
        } else {
            u = n++;
        }
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            const PabloAST * const op = stmt->getOperand(i);
            if (LLVM_LIKELY(isa<Statement>(op))) {
                auto f = M.find(op);
                if (f != M.end()) {
                    const unsigned v = std::get<1>(*f);
                    for (unsigned w = 0; w != k; ++w) {
                        G(u, w) |= G(v, w);
                    }
                }
            }
        }
        M.emplace(stmt, u);
    }

    assert (k == advances);

    // Initialize the base constraint graph by transposing G and removing reflective loops
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

    mSubsetGraph = SubsetGraph(advances);
    mAdvanceNegatedVariable.reserve(advances);

    return last;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief is_power_of_2
 * @param n an integer
 ** ------------------------------------------------------------------------------------------------------------- */
static inline bool is_power_of_2(const size_t n) {
    return ((n & (n - 1)) == 0);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateCandidateSets
 ** ------------------------------------------------------------------------------------------------------------- */
bool MultiplexingPass::generateCandidateSets() {

    const auto n = mAdvanceNegatedVariable.size();
    if (n < 3) {
        return false;
    }
    assert (num_vertices(mConstraintGraph) == n);

    Constraints S;

    ConstraintGraph::degree_size_type D[n];

    mCandidateGraph = CandidateGraph(n);

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

    bool chosen[n] = {};

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
            Advance * const adv1 = mConstraintGraph[source(e, mSubsetGraph)];
            Advance * const adv2 = mConstraintGraph[target(e, mSubsetGraph)];
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
 * @brief dominates
 *
 * does Statement a dominate Statement b?
 ** ------------------------------------------------------------------------------------------------------------- */
bool dominates(const Statement * const a, const Statement * const b) {

    if (LLVM_UNLIKELY(b == nullptr)) {
        return true;
    } else if (LLVM_UNLIKELY(a == nullptr)) {
        return false;
    }

    assert (a->getParent());
    assert (b->getParent());

    const PabloBlock * const parent = a->getParent();
    if (LLVM_LIKELY(parent == b->getParent())) {
        for (const Statement * t : *parent) {
            if (t == a) {
                return true;
            } else if (t == b) {
                break;
            }
        }
        return false;
    } else {
        const PabloBlock * block = b->getParent();
        for (;;) {
            Statement * br = block->getBranch();
            if (br == nullptr) {
                return dominates(parent->getBranch(), b);
            }
            block = br->getParent();
            if (block == parent) {
                return dominates(a, br);
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief multiplexSelectedSets
 ** ------------------------------------------------------------------------------------------------------------- */
inline void MultiplexingPass::multiplexSelectedSets(PabloBlock * const block) {


//    Z3_config cfg = Z3_mk_config();
//    Z3_context ctx = Z3_mk_context_rc(cfg);
//    Z3_del_config(cfg);
//    Z3_solver solver = Z3_mk_solver(ctx);
//    Z3_solver_inc_ref(ctx, solver);


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
            for (const auto u : make_iterator_range(adjacent_vertices(idx, mCandidateGraph))) { // orderMultiplexSet(idx)) {
                input[i++] = mConstraintGraph[u];
            }
            Advance * const adv = input[0];
            assert (block == adv->getParent());

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

    flat_set<PabloAST *> encountered;
    for (Statement * stmt = block->front(); stmt; ) {

        assert (stmt->getParent() == block);
        Statement * const next = stmt->getNextNode();

        bool unmoved = true;
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            PabloAST * const op = stmt->getOperand(i);
            if (isa<Statement>(op)) {
                Statement * ip = cast<Statement>(op);
                if (ip->getParent() != block) {
                    // If we haven't already encountered the Assign or Next node, it must come from a If or
                    // While node that we haven't processed yet. Scan ahead and try to locate it.
                    if (isa<Assign>(op)) {
                        for (PabloAST * user : cast<Assign>(op)->users()) {
                            if (isa<If>(user) && cast<If>(user)->getParent() == block) {
                                const auto & defs = cast<If>(user)->getDefined();
                                if (LLVM_LIKELY(std::find(defs.begin(), defs.end(), op) != defs.end())) {
                                    ip = cast<If>(user);
                                    break;
                                }
                            }
                        }
                    } else if (isa<Next>(op)) {
                        for (PabloAST * user : cast<Next>(op)->users()) {
                            if (isa<While>(user) && cast<While>(user)->getParent() == block) {
                                const auto & vars = cast<While>(user)->getVariants();
                                if (LLVM_LIKELY(std::find(vars.begin(), vars.end(), op) != vars.end())) {
                                    ip = cast<While>(user);
                                    break;
                                }
                            }
                        }
                    }
                }
                if (encountered.count(ip) == 0) {
                    if (dominates(ip, stmt)) {
                        encountered.insert(ip);
                    } else {
                        assert (ip->getParent() == block);
                        stmt->insertAfter(ip);
                        unmoved = false;
                        break;
                    }
                }
            }
        }
        if (unmoved) {
            encountered.insert(stmt);
        }
        stmt = next;
    }

//    Z3_solver_dec_ref(ctx, solver);
//    Z3_del_context(ctx);


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
    } while (!Q.empty());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief get
 ** ------------------------------------------------------------------------------------------------------------- */
inline Z3_ast & MultiplexingPass::get(const PabloAST * const expr, const bool deref) {
    assert (expr);
    auto f = mCharacterization.find(expr);
    assert (f != mCharacterization.end());
    auto & val = f->second;
    if (deref) {
        unsigned & refs = std::get<1>(val);
        assert (refs > 0);
        --refs;
    }
    return std::get<0>(val);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief make
 ** ------------------------------------------------------------------------------------------------------------- */
inline Z3_ast MultiplexingPass::make(const PabloAST * const expr) {
    assert (expr);
    Z3_sort ty = Z3_mk_bool_sort(mContext);
    Z3_symbol s = Z3_mk_string_symbol(mContext, nullptr); // expr->getName()->to_string().c_str()
    Z3_ast node = Z3_mk_const(mContext, s, ty);
    Z3_inc_ref(mContext, node);
    return add(expr, node);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief add
 ** ------------------------------------------------------------------------------------------------------------- */
inline Z3_ast MultiplexingPass::add(const PabloAST * const expr, Z3_ast node) {    
    mCharacterization.insert(std::make_pair(expr, std::make_pair(node, expr->getNumUses())));
    return node;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
inline MultiplexingPass::MultiplexingPass(PabloFunction & f, const RNG::result_type seed, Z3_context context, Z3_solver solver)
: mContext(context)
, mSolver(solver)
, mFunction(f)
, mRNG(seed)
, mConstraintGraph(0)
{

}


} // end of namespace pablo
