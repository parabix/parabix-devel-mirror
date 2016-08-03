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

    #ifndef NDEBUG
    PabloVerifier::verify(function, "pre-multiplexing");
    #endif

    Z3_config cfg = Z3_mk_config();
    Z3_context ctx = Z3_mk_context_rc(cfg);
    Z3_del_config(cfg);
    Z3_solver solver = Z3_mk_solver(ctx);
    Z3_solver_inc_ref(ctx, solver);

    MultiplexingPass mp(function, Seed, ctx, solver);

    mp.optimize();

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);

    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-multiplexing");
    #endif

    Simplifier::optimize(function);

    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 * @param function the function to optimize
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::optimize() {
    // Map the constants and input variables

    add(PabloBlock::createZeroes(), Z3_mk_false(mContext));
    add(PabloBlock::createOnes(), Z3_mk_true(mContext));
    for (unsigned i = 0; i < mFunction.getNumOfParameters(); ++i) {
        make(mFunction.getParameter(i));
    }

    optimize(mFunction.getEntryBlock());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::optimize(PabloBlock * const block) {
    Statement * begin = block->front();
    Statement * end = initialize(begin);
    for (Statement * stmt : *block) {
        if (LLVM_UNLIKELY(stmt == end)) {
            Statement * const next = stmt->getNextNode();
            multiplex(block, begin, stmt);
            if (isa<If>(stmt)) {
                optimize(cast<If>(stmt)->getBody());
            } else if (isa<While>(stmt)) {
                for (const Next * var : cast<While>(stmt)->getVariants()) {
                    Z3_inc_ref(mContext, get(var->getInitial()));
                }
                optimize(cast<While>(stmt)->getBody());
                // since we cannot be certain that we'll always execute at least one iteration of a loop, we must
                // assume that the variants could either be their initial or resulting value.
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
            end = initialize(begin = next);
        } else {
            characterize(stmt);
        }
    }
    multiplex(block, begin, nullptr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief multiplex
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::multiplex(PabloBlock * const block, Statement * const begin, Statement * const end) {
    if (generateCandidateSets()) {
        selectMultiplexSetsGreedy();
        eliminateSubsetConstraints();
        multiplexSelectedSets(block, begin, end);
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
        const auto e = i++;
        if (LLVM_UNLIKELY(std::get<1>(r) == 0)) {
            Z3_dec_ref(mContext, std::get<0>(r));
            mCharacterization.erase(e);
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

///** ------------------------------------------------------------------------------------------------------------- *
// * Fu & Malik procedure for MaxSAT. This procedure is based on unsat core extraction and the at-most-one constraint.
// ** ------------------------------------------------------------------------------------------------------------- */
//inline bool fu_malik_maxsat_step(Z3_context ctx, Z3_solver s, const std::vector<Z3_ast> & soft)
//{
//    // create assumptions
//    const auto n = soft.size();
//    Z3_ast assumptions[n];
//    for (size_t i = 0; i < n; ++i) {
//        assumptions[i] = Z3_mk_not(ctx, soft[i]);
//    }

//    if (Z3_solver_check_assumptions(ctx, s, n, assumptions) != Z3_L_FALSE) {
//        return true; // done
//    }

//    const auto core = Z3_solver_get_unsat_core(ctx, s);
//    const auto m = Z3_ast_vector_size(ctx, core);
//    Z3_ast block_vars[m];
//    unsigned k = 0;
//    // update soft-constraints and aux_vars
//    for (unsigned i = 0; i < num_soft_cnstrs; i++) {
//        // check whether assumption[i] is in the core or not
//        for (unsigned j = 0; j < m; j++) {
//            if (assumptions[i] == Z3_ast_vector_get(ctx, core, j)) {
//                // assumption[i] is in the unsat core... so soft_cnstrs[i] is in the unsat core
//                Z3_ast block_var   = Z3_mk_fresh_const(ctx, nullptr, Z3_mk_bool_sort(ctx));
//                Z3_ast new_aux_var = Z3_mk_fresh_const(ctx, nullptr, Z3_mk_bool_sort(ctx));
//                soft_cnstrs[i]     = mk_binary_or(ctx, soft_cnstrs[i], block_var);
//                aux_vars[i]        = new_aux_var;
//                block_vars[k]      = block_var;
//                k++;
//                // Add new constraint containing the block variable.
//                // Note that we are using the new auxiliary variable to be able to use it as an assumption.
//                Z3_solver_assert(ctx, s, mk_binary_or(ctx, soft_cnstrs[i], new_aux_var));
//                break;
//            }
//        }
//    }


//    assert_at_most_one(ctx, s, k, block_vars);
//    return 0; // not done.

//}

///** ------------------------------------------------------------------------------------------------------------- *
// * Fu & Malik procedure for MaxSAT. This procedure is based on unsat core extraction and the at-most-one constraint.
// ** ------------------------------------------------------------------------------------------------------------- */
//inline bool fu_malik_maxsat(Z3_context ctx, Z3_solver s, const std::vector<Z3_ast> & soft) {
//    assert(Z3_solver_check(ctx, s) != Z3_L_FALSE);
//    for (size_t k = 0; k < soft.size(); ++k) {
//        if (fu_malik_maxsat_step(ctx, s, soft)) {
//            return true;
//        }
//    }
//    return false;
//}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addWithHardConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
Z3_ast addWithHardConstraints(Z3_context ctx, Z3_solver solver, PabloBlock * const block, Statement * stmt, flat_map<Statement *, Z3_ast> & M) {
    assert (M.count(stmt) == 0 && stmt->getParent() == block);
    // compute the hard dependency constraints
    Z3_symbol symbol = Z3_mk_string_symbol(ctx, stmt->getName()->value().data()); assert (symbol);
    Z3_ast node = Z3_mk_const(ctx, symbol, Z3_mk_int_sort(ctx)); assert (node);
    for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
        PabloAST * const op = stmt->getOperand(i);
        if (isa<Statement>(op) && cast<Statement>(op)->getParent() == block) {
            const auto f = M.find(cast<Statement>(op));
            if (f != M.end()) {
                Z3_ast depedency = Z3_mk_lt(ctx, f->second, node);
                Z3_solver_assert(ctx, solver, depedency);
            }
        }
    }
    M.emplace(stmt, node);
    return node;

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief dominates
 *
 * does Statement a dominate Statement b?
 ** ------------------------------------------------------------------------------------------------------------- */
bool dominates(const Statement * const a, const Statement * const b) {
    assert (a);
    if (LLVM_UNLIKELY(b == nullptr)) {
        return false;
    }
    assert (a->getParent() == b->getParent());
    for (const Statement * t : *a->getParent()) {
        if (t == a) {
            return true;
        } else if (t == b) {
            return false;
        }
    }
    llvm_unreachable("Neither a nor b are in their reported block!");
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addWithHardConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
Z3_ast addWithHardConstraints(Z3_context ctx, Z3_solver solver, PabloBlock * const block, PabloAST * expr, flat_map<Statement *, Z3_ast> & M, Statement * const ip) {
    if (isa<Statement>(expr)) {
        Statement * const stmt = cast<Statement>(expr);
        if (stmt->getParent() == block) {
            const auto f = M.find(stmt);
            if (LLVM_UNLIKELY(f != M.end())) {
                return f->second;
            } else if (!dominates(stmt, ip)) {
                for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                    addWithHardConstraints(ctx, solver, block, stmt->getOperand(i), M, ip);
                }
                return addWithHardConstraints(ctx, solver, block, stmt, M);
            }
        }
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief multiplexSelectedSets
 ** ------------------------------------------------------------------------------------------------------------- */
inline void MultiplexingPass::multiplexSelectedSets(PabloBlock * const block, Statement * const begin, Statement * const end) {

    assert ("begin cannot be null!" && begin);
    assert (begin->getParent() == block);
    assert (!end || end->getParent() == block);
    assert (!end || isa<If>(end) || isa<While>(end));

    Statement * const ip = begin->getPrevNode(); // save our insertion point prior to modifying the AST

    Z3_config cfg = Z3_mk_config();
    // Z3_set_param_value(cfg, "MODEL", "true");
    Z3_context ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);
    Z3_solver solver = Z3_mk_solver(ctx);

    const auto first_set = num_vertices(mConstraintGraph);
    const auto last_set = num_vertices(mCandidateGraph);

    // Compute the hard and soft constraints for any part of the AST that we are not intending to modify.
    flat_map<Statement *, Z3_ast> M;

//    Z3_ast prior = nullptr;

//    Z3_ast one = Z3_mk_int(ctx, 1, Z3_mk_int_sort(ctx));

//    std::vector<Z3_ast> soft; // call check_with_assumptions!!!

    for (Statement * stmt = begin; stmt != end; stmt = stmt->getNextNode()) {
        Z3_ast node = addWithHardConstraints(ctx, solver, block, stmt, M);
//        // add in the soft ordering constraints
//        if (prior) {
//            Z3_ast constraint[2];
//            if (gap) {
//                constraint[0] = Z3_mk_lt(ctx, prior, node);
//                gap = false;
//            } else {
//                Z3_ast prior_plus_one[2] = { prior, one };
//                Z3_ast num = Z3_mk_add(ctx, 2, prior_plus_one);
//                constraint[0] = Z3_mk_eq(ctx, node, num);
//            }
//            Z3_ast ordering = Z3_mk_fresh_const(ctx, nullptr, Z3_mk_bool_sort(ctx));
//            constraint[1] = ordering;
//            Z3_solver_assert(ctx, solver, Z3_mk_or(ctx, 2, constraint));
//            soft.push_back(ordering);
//        }
//        prior = node;
    }


    block->setInsertPoint(block->back());

    for (auto idx = first_set; idx != last_set; ++idx) {
        const size_t n = degree(idx, mCandidateGraph);
        if (n) {
            const size_t m = log2_plus_one(n); assert (n > 2 && m < n);
            Advance * input[n];
            PabloAST * muxed[m];
            PabloAST * muxed_n[m];
            // The multiplex set graph is a DAG with edges denoting the set relationships of our independent sets.
            unsigned i = 0;
            for (const auto u : make_iterator_range(adjacent_vertices(idx, mCandidateGraph))) {
                input[i] = mConstraintGraph[u];
                assert ("Algorithm failure! not all inputs are in the same block!" && (input[i]->getParent() == block));
                assert ("Algorithm failure! not all inputs advance by the same amount!" && (input[i]->getOperand(1) == input[0]->getOperand(1)));
                ++i;
            }

            circular_buffer<PabloAST *> Q(n);

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
                    PabloAST * expr = block->createOr(a, b);
                    addWithHardConstraints(ctx, solver, block, expr, M, ip);
                    Q.push_back(expr);
                }
                PabloAST * const muxing = Q.front(); Q.clear();
                muxed[j] = block->createAdvance(muxing, input[0]->getOperand(1), prefix.str());
                addWithHardConstraints(ctx, solver, block, muxed[j], M, ip);
                muxed_n[j] = block->createNot(muxed[j]);
                addWithHardConstraints(ctx, solver, block, muxed_n[j], M, ip);
            }

            /// Perform m-to-n Demultiplexing
            for (size_t i = 0; i != n; ++i) {
                // Construct the demuxed values and replaces all the users of the original advances with them.
                assert (Q.empty());
                for (size_t j = 0; j != m; ++j) {
                    Q.push_back((((i + 1) & (1UL << j)) != 0) ? muxed[j] : muxed_n[j]);
                }
                Z3_ast replacement = nullptr;
                while (Q.size() > 1) {
                    PabloAST * const a = Q.front(); Q.pop_front();
                    PabloAST * const b = Q.front(); Q.pop_front();
                    PabloAST * expr = block->createAnd(a, b);
                    replacement = addWithHardConstraints(ctx, solver, block, expr, M, ip);
                    Q.push_back(expr);
                }
                assert (replacement);
                PabloAST * const demuxed = Q.front(); Q.clear();

                const auto f = M.find(input[i]);
                assert (f != M.end());
                Z3_solver_assert(ctx, solver, Z3_mk_eq(ctx, f->second, replacement));
                M.erase(f);

                input[i]->replaceWith(demuxed);
                assert (M.count(input[i]) == 0);
            }
        }
    }

    assert (M.count(ip) == 0);

    if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) == Z3_L_FALSE)) {
        throw std::runtime_error("Unexpected Z3 failure when attempting to topologically sort the AST!");
    }

    Z3_model m = Z3_solver_get_model(ctx, solver);
    Z3_model_inc_ref(ctx, m);

    std::vector<std::pair<long long int, Statement *>> Q;

    for (const auto i : M) {
        Z3_ast value;
        if (Z3_model_eval(ctx, m, std::get<1>(i), Z3_L_TRUE, &value) != Z3_L_TRUE) {
            throw std::runtime_error("Unexpected Z3 error when attempting to obtain value from model!");
        }
        long long int line;
        if (Z3_get_numeral_int64(ctx, value, &line) != Z3_L_TRUE) {
            throw std::runtime_error("Unexpected Z3 error when attempting to convert model value to integer!");
        }
        Q.emplace_back(line, std::get<0>(i));
    }

    Z3_model_dec_ref(ctx, m);
    Z3_del_context(ctx);

    std::sort(Q.begin(), Q.end());

    block->setInsertPoint(ip);
    for (auto i : Q) {
        block->insert(std::get<1>(i));
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
    const String * name = nullptr;
    if (LLVM_LIKELY(isa<Statement>(expr))) {
        name = cast<Statement>(expr)->getName();
    } else if (LLVM_UNLIKELY(isa<Var>(expr))) {
        name = cast<Var>(expr)->getName();
    }
    assert (name);
    Z3_symbol s = Z3_mk_string_symbol(mContext, name->value().data());
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
