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

static cl::opt<unsigned> WindowSize("multiplexing-window-size", cl::init(100),
                                        cl::desc("maximum sequence distance to consider for candidate set."),
                                        cl::cat(MultiplexingOptions));


namespace pablo {

Z3_bool maxsat(Z3_context ctx, Z3_solver solver, std::vector<Z3_ast> & soft);

using TypeId = PabloAST::ClassTypeId;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 * @param function the function to optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool MultiplexingPass::optimize(PabloFunction & function) {

    PabloVerifier::verify(function, "pre-multiplexing");

    errs() << "PRE-MULTIPLEXING\n==============================================\n";
    PabloPrinter::print(function, errs());

    Z3_config cfg = Z3_mk_config();
    Z3_context ctx = Z3_mk_context_rc(cfg);
    Z3_del_config(cfg);
    Z3_solver solver = Z3_mk_solver(ctx);
    Z3_solver_inc_ref(ctx, solver);

    MultiplexingPass mp(function, Seed, ctx, solver);

    mp.optimize();

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);

    PabloVerifier::verify(function, "post-multiplexing");

    Simplifier::optimize(function);

    errs() << "POST-MULTIPLEXING\n==============================================\n";
    PabloPrinter::print(function, errs());

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
    if (generateCandidateSets(begin, end)) {
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
    Z3_ast test = Z3_mk_eq(mContext, a, b); // try using check assumption instead?
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
    const auto k = mNegatedAdvance.size();

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
            Z3_dec_ref(mContext, Ai0);
            Ai0 = Ai;
            assert (get(mConstraintGraph[i]) == Ai);

            vars[m++] = mNegatedAdvance[i];

            continue; // note: if these Advances are transitively dependent, an edge will still exist.
        }
        add_edge(i, k, mConstraintGraph);
    }
    // To minimize the number of BDD computations, we store the negated variable instead of negating it each time.
    mNegatedAdvance.emplace_back(Nk);
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

    for (Z3_ast var : mNegatedAdvance) {
        Z3_dec_ref(mContext, var);
    }
    mNegatedAdvance.clear();

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
    mNegatedAdvance.reserve(advances);

    return last;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateCandidateSets
 ** ------------------------------------------------------------------------------------------------------------- */
bool MultiplexingPass::generateCandidateSets(Statement * const begin, Statement * const end) {

    const auto n = mNegatedAdvance.size();
    if (LLVM_UNLIKELY(n < 3)) {
        return false;
    }
    assert (num_vertices(mConstraintGraph) == n);

    // The naive way to handle this would be to compute a DNF formula consisting of the
    // powerset of all independent (candidate) sets of G, assign a weight to each, and
    // try to maximally satisfy the clauses. However, this would be extremely costly to
    // compute let alone solve as we could easily generate O(2^100) clauses for a complex
    // problem. Further the vast majority of clauses would be false in the end.

    // Moreover, for every set that can Advance is contained in would need a unique
    // variable and selector. In other words:

    // Suppose Advance A has a selector variable I. If I is true, then A must be in ONE set.
    // Assume A could be in m sets. To enforce this, there are m(m - 1)/2 clauses:

    //   (¬A_1 ∨ ¬A_2 ∨ ¬I), (¬A_1 ∨ ¬A_3 ∨ ¬I), ..., (¬A_m-1 ∨ ¬A_m ∨ ¬I)

    // m here is be equivalent to number of independent sets in the constraint graph G
    // that contains A.

    // If two sets have a DEPENDENCY constraint between them, it will introduce a cyclic
    // relationship even if those sets are legal on their own. Thus we'd also need need
    // hard constraints between all constrained variables related to the pair of Advances.

    // Instead, we only try to solve for one set at a time. This eliminate the need for
    // the above constraints and computing m but this process to be closer to a simple
    // greedy search.

    // We do want to weight whether to include or exclude an item in a set but what should
    // this be? The weight must be related to the elements already in the set. If our goal
    // is to balance the perturbation of the AST with the reduction in # of Advances, the
    // cost of inclusion / exclusion could be proportional to the # of instructions that
    // it increases / decreases the span by --- but how many statements is an Advance worth?

    // What if instead we maintain a queue of advances and discard any that are outside of
    // the current window?

    mCandidateGraph = CandidateGraph(n);

    Z3_config cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "MODEL", "true");
    Z3_context ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);
    Z3_solver solver = Z3_mk_solver(ctx);
    Z3_solver_inc_ref(ctx, solver);
    std::vector<Z3_ast> N(n);
    for (unsigned i = 0; i < n; ++i) {
        N[i] = Z3_mk_fresh_const(ctx, nullptr, Z3_mk_bool_sort(ctx)); assert (N[i]);
    }
    std::vector<std::pair<unsigned, unsigned>> S;
    S.reserve(n);

    unsigned line = 0;
    unsigned i = 0;
    for (Statement * stmt = begin; stmt != end; stmt = stmt->getNextNode()) {
        if (LLVM_UNLIKELY(isa<Advance>(stmt))) {
            assert (S.empty() || line > std::get<0>(S.back()));
            assert (cast<Advance>(stmt) == mConstraintGraph[i]);
            if (S.size() > 0 && (line - std::get<0>(S.front())) > WindowSize) {
                // try to compute a maximal set for this given set of Advances
                if (S.size() > 2) {
                    generateCandidateSets(ctx, solver, S, N);
                }
                // erase any that preceed our window
                for (auto i = S.begin();;) {
                    if (++i == S.end() || (line - std::get<0>(*i)) <= WindowSize) {
                        S.erase(S.begin(), i);
                        break;
                    }
                }
            }
            for (unsigned j : make_iterator_range(adjacent_vertices(i, mConstraintGraph))) {
                Z3_ast disj[2] = { Z3_mk_not(ctx, N[j]), Z3_mk_not(ctx, N[i]) };
                Z3_solver_assert(ctx, solver, Z3_mk_or(ctx, 2, disj));
            }
            S.emplace_back(line, i++);
        }
        ++line;
    }
    if (S.size() > 2) {
        generateCandidateSets(ctx, solver, S, N);
    }

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);

    return num_vertices(mCandidateGraph) > n;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateCandidateSets
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::generateCandidateSets(Z3_context ctx, Z3_solver solver, const std::vector<std::pair<unsigned, unsigned>> & S, const std::vector<Z3_ast> & N) {
    assert (S.size() > 2);
    assert (std::get<0>(S.front()) < std::get<0>(S.back()));
    assert ((std::get<0>(S.back()) - std::get<0>(S.front())) <= WindowSize);
    Z3_solver_push(ctx, solver);
    const auto n = N.size();
    std::vector<Z3_ast> assumptions(S.size());
    for (unsigned i = 0, j = 0; i < n; ++i) {
        if (LLVM_UNLIKELY(j < S.size() && std::get<1>(S[j]) == i)) { // in our window range
            assumptions[j++] = N[i];
        } else {
            Z3_solver_assert(ctx, solver, Z3_mk_not(ctx, N[i]));
        }
    }
    if (maxsat(ctx, solver, assumptions) != Z3_L_FALSE) {
        Z3_model m = Z3_solver_get_model(ctx, solver);
        Z3_model_inc_ref(ctx, m);
        const auto k = add_vertex(mCandidateGraph); assert(k >= N.size());
        Z3_ast TRUE = Z3_mk_true(ctx);
        Z3_ast FALSE = Z3_mk_false(ctx);
        for (const auto i : S) {
            Z3_ast value;
            if (LLVM_UNLIKELY(Z3_model_eval(ctx, m, N[std::get<1>(i)], 1, &value) != Z3_TRUE)) {
                throw std::runtime_error("Unexpected Z3 error when attempting to obtain value from constraint model!");
            }
            if (value == TRUE) {
                add_edge(std::get<1>(i), k, mCandidateGraph);
            } else if (LLVM_UNLIKELY(value != FALSE)) {
                throw std::runtime_error("Unexpected Z3 error constraint model value is a non-terminal!");
            }
        }
        Z3_model_dec_ref(ctx, m);
    }
    Z3_solver_pop(ctx, solver, 1);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief is_power_of_2
 * @param n an integer
 ** ------------------------------------------------------------------------------------------------------------- */
static inline bool is_power_of_2(const size_t n) {
    return ((n & (n - 1)) == 0);
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
// * Topologically sort the sequence of instructions whilst trying to adhere as best as possible to the original
// * program sequence.
// ** ------------------------------------------------------------------------------------------------------------- */
//inline bool topologicalSort(Z3_context ctx, Z3_solver solver, const std::vector<Z3_ast> & nodes, const int limit) {
//    const auto n = nodes.size();
//    if (LLVM_UNLIKELY(n == 0)) {
//        return true;
//    }
//    if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) == Z3_L_FALSE)) {
//        return false;
//    }

//    Z3_ast aux_vars[n];
//    Z3_ast assumptions[n];
//    Z3_ast ordering[n];
//    int increments[n];

//    Z3_sort boolTy = Z3_mk_bool_sort(ctx);
//    Z3_sort intTy = Z3_mk_int_sort(ctx);
//    Z3_ast one = Z3_mk_int(ctx, 1, intTy);

//    for (unsigned i = 0; i < n; ++i) {
//        aux_vars[i] = Z3_mk_fresh_const(ctx, nullptr, boolTy);
//        assumptions[i] = Z3_mk_not(ctx, aux_vars[i]);
//        Z3_ast num = one;
//        if (i > 0) {
//            Z3_ast prior_plus_one[2] = { nodes[i - 1], one };
//            num = Z3_mk_add(ctx, 2, prior_plus_one);
//        }
//        ordering[i] = Z3_mk_eq(ctx, nodes[i], num);
//        increments[i] = 1;
//    }

//    unsigned unsat = 0;

//    for (;;) {
//        Z3_solver_push(ctx, solver);
//        for (unsigned i = 0; i < n; ++i) {
//            Z3_ast constraint[2] = {ordering[i], aux_vars[i]};
//            Z3_solver_assert(ctx, solver, Z3_mk_or(ctx, 2, constraint));
//        }
//        if (LLVM_UNLIKELY(Z3_solver_check_assumptions(ctx, solver, n, assumptions) != Z3_L_FALSE)) {
//            errs() << " SATISFIABLE!  (" << unsat << " of " << n << ")\n";
//            return true; // done
//        }
//        Z3_ast_vector core = Z3_solver_get_unsat_core(ctx, solver); assert (core);
//        unsigned m = Z3_ast_vector_size(ctx, core); assert (m > 0);

//        errs() << " UNSATISFIABLE " << m << "  (" << unsat << " of " << n <<")\n";

//        for (unsigned j = 0; j < m; j++) {
//            // check whether assumption[i] is in the core or not
//            bool not_found = true;
//            for (unsigned i = 0; i < n; i++) {
//                if (assumptions[i] == Z3_ast_vector_get(ctx, core, j)) {

//                    const auto k = increments[i];

//                    errs() << " -- " << i << " @k=" << k << "\n";

//                    if (k < limit) {
//                        Z3_ast gap = Z3_mk_int(ctx, 1UL << k, intTy);
//                        Z3_ast num = gap;
//                        if (LLVM_LIKELY(i > 0)) {
//                            Z3_ast prior_plus_gap[2] = { nodes[i - 1], gap };
//                            num = Z3_mk_add(ctx, 2, prior_plus_gap);
//                        }
//                        Z3_dec_ref(ctx, ordering[i]);
//                        ordering[i] = Z3_mk_le(ctx, num, nodes[i]);
//                    } else if (k == limit && i > 0) {
//                        ordering[i] = Z3_mk_le(ctx, nodes[i - 1], nodes[i]);
//                    } else {
//                        assumptions[i] = aux_vars[i]; // <- trivially satisfiable
//                        ++unsat;
//                    }
//                    increments[i] = k + 1;
//                    not_found = false;
//                    break;
//                }
//            }
//            if (LLVM_UNLIKELY(not_found)) {
//                throw std::runtime_error("Unexpected Z3 failure when attempting to locate unsatisfiable ordering constraint!");
//            }
//        }
//        Z3_solver_pop(ctx, solver, 1);
//    }
//    llvm_unreachable("maxsat wrongly reported this being unsatisfiable despite being able to satisfy the hard constraints!");
//    return false;
//}

///** ------------------------------------------------------------------------------------------------------------- *
// * Topologically sort the sequence of instructions whilst trying to adhere as best as possible to the original
// * program sequence.
// ** ------------------------------------------------------------------------------------------------------------- */
//inline bool topologicalSort(Z3_context ctx, Z3_solver solver, const std::vector<Z3_ast> & nodes, const int limit) {
//    const auto n = nodes.size();
//    if (LLVM_UNLIKELY(n == 0)) {
//        return true;
//    }
//    if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) == Z3_L_FALSE)) {
//        return false;
//    }

//    Z3_ast aux_vars[n];
//    Z3_ast assumptions[n];

//    Z3_sort boolTy = Z3_mk_bool_sort(ctx);
//    Z3_ast one = Z3_mk_int(ctx, 1, Z3_mk_int_sort(ctx));

//    for (unsigned i = 0; i < n; ++i) {
//        aux_vars[i] = Z3_mk_fresh_const(ctx, nullptr, boolTy);
//        assumptions[i] = Z3_mk_not(ctx, aux_vars[i]);
//        Z3_ast num = one;
//        if (i > 0) {
//            Z3_ast prior_plus_one[2] = { nodes[i - 1], one };
//            num = Z3_mk_add(ctx, 2, prior_plus_one);
//        }
//        Z3_ast ordering = Z3_mk_eq(ctx, nodes[i], num);
//        Z3_ast constraint[2] = {ordering, aux_vars[i]};
//        Z3_solver_assert(ctx, solver, Z3_mk_or(ctx, 2, constraint));
//    }

//    for (unsigned k = 0; k < n; ) {
//        if (LLVM_UNLIKELY(Z3_solver_check_assumptions(ctx, solver, n, assumptions) != Z3_L_FALSE)) {
//            errs() << " SATISFIABLE!\n";
//            return true; // done
//        }
//        Z3_ast_vector core = Z3_solver_get_unsat_core(ctx, solver); assert (core);
//        unsigned m = Z3_ast_vector_size(ctx, core); assert (m > 0);

//        k += m;

//        errs() << " UNSATISFIABLE " << m << " (" << k << ")\n";

//        for (unsigned j = 0; j < m; j++) {
//            // check whether assumption[i] is in the core or not
//            bool not_found = true;
//            for (unsigned i = 0; i < n; i++) {
//                if (assumptions[i] == Z3_ast_vector_get(ctx, core, j)) {
//                    assumptions[i] = aux_vars[i];
//                    not_found = false;
//                    break;
//                }
//            }
//            if (LLVM_UNLIKELY(not_found)) {
//                throw std::runtime_error("Unexpected Z3 failure when attempting to locate unsatisfiable ordering constraint!");
//            }
//        }
//    }
//    llvm_unreachable("maxsat wrongly reported this being unsatisfiable despite being able to satisfy the hard constraints!");
//    return false;
//}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addWithHardConstraints
 ** ------------------------------------------------------------------------------------------------------------- */
Z3_ast addWithHardConstraints(Z3_context ctx, Z3_solver solver, PabloBlock * const block, Statement * const stmt, flat_map<Statement *, Z3_ast> & M) {
    assert (M.count(stmt) == 0 && stmt->getParent() == block);
    // compute the hard dependency constraints
    Z3_ast node = Z3_mk_fresh_const(ctx, nullptr, Z3_mk_int_sort(ctx)); assert (node);
    // we want all numbers to be positive so that the soft assertion that the first statement ought to stay at the first location
    // whenever possible isn't satisfied by making preceeding numbers negative.
    Z3_solver_assert(ctx, solver, Z3_mk_gt(ctx, node, Z3_mk_int(ctx, 0, Z3_mk_int_sort(ctx))));
    for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
        PabloAST * const op = stmt->getOperand(i);
        if (isa<Statement>(op) && cast<Statement>(op)->getParent() == block) {
            const auto f = M.find(cast<Statement>(op));
            if (f != M.end()) {
                Z3_solver_assert(ctx, solver, Z3_mk_lt(ctx, f->second, node));
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

    Z3_config cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "MODEL", "true");
    Z3_context ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);
    Z3_solver solver = Z3_mk_solver(ctx);
    Z3_solver_inc_ref(ctx, solver);

    const auto first_set = num_vertices(mConstraintGraph);
    const auto last_set = num_vertices(mCandidateGraph);

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
                assert ("Not all inputs are in the same block!" && (input[i]->getParent() == block));
                assert ("Not all inputs advance by the same amount!" && (input[i]->getOperand(1) == input[0]->getOperand(1)));
                assert ("Inputs are not in sequential order!" && (i == 0 || (i > 0 && dominates(input[i - 1], input[i]))));
                ++i;
            }

            Statement * const A1 = input[0];
            Statement * const An = input[n - 1]->getNextNode();

            Statement * const ip = A1->getPrevNode(); // save our insertion point prior to modifying the AST

            Z3_solver_push(ctx, solver);

            // Compute the hard and soft constraints for any part of the AST that we are not intending to modify.
            flat_map<Statement *, Z3_ast> M;

            Z3_ast prior = nullptr;
            Z3_ast one = Z3_mk_int(ctx, 1, Z3_mk_int_sort(ctx));
            std::vector<Z3_ast> ordering;
//            std::vector<Z3_ast> nodes;

            for (Statement * stmt = A1; stmt != An; stmt = stmt->getNextNode()) { assert (stmt != ip);
                Z3_ast node = addWithHardConstraints(ctx, solver, block, stmt, M);
                // compute the soft ordering constraints
                Z3_ast num = one;
                if (prior) {
                    Z3_ast prior_plus_one[2] = { prior, one };
                    num = Z3_mk_add(ctx, 2, prior_plus_one);
                }
                ordering.push_back(Z3_mk_eq(ctx, node, num));
                if (prior) {
                    ordering.push_back(Z3_mk_lt(ctx, prior, node));
                }


//                for (Z3_ast prior : nodes) {
//                    Z3_solver_assert(ctx, solver, Z3_mk_not(ctx, Z3_mk_eq(ctx, prior, node)));
//                }
 //               nodes.push_back(node);


                prior = node;
            }

            // assert (nodes.size() <= WindowSize);

            block->setInsertPoint(block->back()); // <- necessary for domination check!

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

            assert (M.count(ip) == 0);

            if (LLVM_UNLIKELY(maxsat(ctx, solver, ordering) != Z3_L_TRUE)) {
                throw std::runtime_error("Unexpected Z3 failure when attempting to topologically sort the AST!");
            }

            Z3_model model = Z3_solver_get_model(ctx, solver);
            Z3_model_inc_ref(ctx, model);

            std::vector<std::pair<long long int, Statement *>> I;

            for (const auto i : M) {
                Z3_ast value;
                if (LLVM_UNLIKELY(Z3_model_eval(ctx, model, std::get<1>(i), Z3_L_TRUE, &value) != Z3_L_TRUE)) {
                    throw std::runtime_error("Unexpected Z3 error when attempting to obtain value from model!");
                }
                long long int line;
                if (LLVM_UNLIKELY(Z3_get_numeral_int64(ctx, value, &line) != Z3_L_TRUE)) {
                    throw std::runtime_error("Unexpected Z3 error when attempting to convert model value to integer!");
                }
                I.emplace_back(line, std::get<0>(i));
            }

            Z3_model_dec_ref(ctx, model);

            std::sort(I.begin(), I.end());

            block->setInsertPoint(ip);
            for (auto i : I) {
                block->insert(std::get<1>(i));
            }

            Z3_solver_pop(ctx, solver, 1);
        }
    }

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);

}

///** ------------------------------------------------------------------------------------------------------------- *
// * @brief multiplexSelectedSets
// ** ------------------------------------------------------------------------------------------------------------- */
//inline void MultiplexingPass::multiplexSelectedSets(PabloBlock * const block, Statement * const begin, Statement * const end) {

//    assert ("begin cannot be null!" && begin);
//    assert (begin->getParent() == block);
//    assert (!end || end->getParent() == block);
//    assert (!end || isa<If>(end) || isa<While>(end));

//    Statement * const ip = begin->getPrevNode(); // save our insertion point prior to modifying the AST

//    Z3_config cfg = Z3_mk_config();
//    Z3_set_param_value(cfg, "MODEL", "true");
//    Z3_context ctx = Z3_mk_context(cfg);
//    Z3_del_config(cfg);
//    Z3_solver solver = Z3_mk_solver(ctx);
//    Z3_solver_inc_ref(ctx, solver);

//    const auto first_set = num_vertices(mConstraintGraph);
//    const auto last_set = num_vertices(mCandidateGraph);

//    // Compute the hard and soft constraints for any part of the AST that we are not intending to modify.
//    flat_map<Statement *, Z3_ast> M;

//    Z3_ast prior = nullptr;
//    Z3_ast one = Z3_mk_int(ctx, 1, Z3_mk_int_sort(ctx));
//    std::vector<Z3_ast> ordering;

//    for (Statement * stmt = begin; stmt != end; stmt = stmt->getNextNode()) { assert (stmt != ip);
//        Z3_ast node = addWithHardConstraints(ctx, solver, block, stmt, M);
//        // compute the soft ordering constraints
//        Z3_ast num = one;
//        if (prior) {
//            Z3_ast prior_plus_one[2] = { prior, one };
//            num = Z3_mk_add(ctx, 2, prior_plus_one);
//        }
//        ordering.push_back(Z3_mk_eq(ctx, node, num));
//        prior = node;
//    }

//    block->setInsertPoint(block->back()); // <- necessary for domination check!

//    errs() << "---------------------------------------------\n";

//    for (auto idx = first_set; idx != last_set; ++idx) {
//        const size_t n = degree(idx, mCandidateGraph);
//        if (n) {
//            const size_t m = log2_plus_one(n); assert (n > 2 && m < n);
//            Advance * input[n];
//            PabloAST * muxed[m];
//            PabloAST * muxed_n[m];

//            errs() << n << " -> " << m << "\n";

//            // The multiplex set graph is a DAG with edges denoting the set relationships of our independent sets.
//            unsigned i = 0;
//            for (const auto u : make_iterator_range(adjacent_vertices(idx, mCandidateGraph))) {
//                input[i] = mConstraintGraph[u];
//                assert ("Not all inputs are in the same block!" && (input[i]->getParent() == block));
//                assert ("Not all inputs advance by the same amount!" && (input[i]->getOperand(1) == input[0]->getOperand(1)));
//                ++i;
//            }

//            circular_buffer<PabloAST *> Q(n);

//            /// Perform n-to-m Multiplexing
//            for (size_t j = 0; j != m; ++j) {
//                std::ostringstream prefix;
//                prefix << "mux" << n << "to" << m << '.' << (j);
//                assert (Q.empty());
//                for (size_t i = 0; i != n; ++i) {
//                    if (((i + 1) & (1UL << j)) != 0) {
//                        Q.push_back(input[i]->getOperand(0));
//                    }
//                }
//                while (Q.size() > 1) {
//                    PabloAST * a = Q.front(); Q.pop_front();
//                    PabloAST * b = Q.front(); Q.pop_front();
//                    PabloAST * expr = block->createOr(a, b);
//                    addWithHardConstraints(ctx, solver, block, expr, M, ip);
//                    Q.push_back(expr);
//                }
//                PabloAST * const muxing = Q.front(); Q.clear();
//                muxed[j] = block->createAdvance(muxing, input[0]->getOperand(1), prefix.str());
//                addWithHardConstraints(ctx, solver, block, muxed[j], M, ip);
//                muxed_n[j] = block->createNot(muxed[j]);
//                addWithHardConstraints(ctx, solver, block, muxed_n[j], M, ip);
//            }

//            /// Perform m-to-n Demultiplexing
//            for (size_t i = 0; i != n; ++i) {
//                // Construct the demuxed values and replaces all the users of the original advances with them.
//                assert (Q.empty());
//                for (size_t j = 0; j != m; ++j) {
//                    Q.push_back((((i + 1) & (1UL << j)) != 0) ? muxed[j] : muxed_n[j]);
//                }
//                Z3_ast replacement = nullptr;
//                while (Q.size() > 1) {
//                    PabloAST * const a = Q.front(); Q.pop_front();
//                    PabloAST * const b = Q.front(); Q.pop_front();
//                    PabloAST * expr = block->createAnd(a, b);
//                    replacement = addWithHardConstraints(ctx, solver, block, expr, M, ip);
//                    Q.push_back(expr);
//                }
//                assert (replacement);
//                PabloAST * const demuxed = Q.front(); Q.clear();

//                const auto f = M.find(input[i]);
//                assert (f != M.end());
//                Z3_solver_assert(ctx, solver, Z3_mk_eq(ctx, f->second, replacement));
//                M.erase(f);

//                input[i]->replaceWith(demuxed);
//                assert (M.count(input[i]) == 0);
//            }
//        }
//    }

//    assert (M.count(ip) == 0);

//    // if (LLVM_UNLIKELY(maxsat(ctx, solver, ordering) == Z3_L_FALSE)) {
//    if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) != Z3_L_TRUE)) {
//        throw std::runtime_error("Unexpected Z3 failure when attempting to topologically sort the AST!");
//    }

//    Z3_model m = Z3_solver_get_model(ctx, solver);
//    Z3_model_inc_ref(ctx, m);

//    std::vector<std::pair<long long int, Statement *>> Q;

//    errs() << "-----------------------------------------------------------\n";

//    for (const auto i : M) {
//        Z3_ast value;
//        if (Z3_model_eval(ctx, m, std::get<1>(i), Z3_L_TRUE, &value) != Z3_L_TRUE) {
//            throw std::runtime_error("Unexpected Z3 error when attempting to obtain value from model!");
//        }
//        long long int line;
//        if (Z3_get_numeral_int64(ctx, value, &line) != Z3_L_TRUE) {
//            throw std::runtime_error("Unexpected Z3 error when attempting to convert model value to integer!");
//        }
//        Q.emplace_back(line, std::get<0>(i));
//    }

//    Z3_model_dec_ref(ctx, m);
//    Z3_solver_dec_ref(ctx, solver);
//    Z3_del_context(ctx);

//    std::sort(Q.begin(), Q.end());

//    block->setInsertPoint(ip);
//    for (auto i : Q) {
//        block->insert(std::get<1>(i));
//    }
//}

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
    Z3_ast node = Z3_mk_fresh_const(mContext, nullptr, Z3_mk_bool_sort(mContext));
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


inline Z3_ast mk_binary_or(Z3_context ctx, Z3_ast in_1, Z3_ast in_2) {
    Z3_ast args[2] = { in_1, in_2 };
    return Z3_mk_or(ctx, 2, args);
}

inline Z3_ast mk_ternary_or(Z3_context ctx, Z3_ast in_1, Z3_ast in_2, Z3_ast in_3) {
    Z3_ast args[3] = { in_1, in_2, in_3 };
    return Z3_mk_or(ctx, 3, args);
}

inline Z3_ast mk_binary_and(Z3_context ctx, Z3_ast in_1, Z3_ast in_2) {
    Z3_ast args[2] = { in_1, in_2 };
    return Z3_mk_and(ctx, 2, args);
}

///**
//   \brief Create a full adder with inputs \c in_1, \c in_2 and \c cin.
//   The output of the full adder is stored in \c out, and the carry in \c c_out.
//*/
//inline std::pair<Z3_ast, Z3_ast> mk_full_adder(Z3_context ctx, Z3_ast in_1, Z3_ast in_2, Z3_ast cin) {
//    Z3_ast out = Z3_mk_xor(ctx, Z3_mk_xor(ctx, in_1, in_2), cin);
//    Z3_ast cout = mk_ternary_or(ctx, mk_binary_and(ctx, in_1, in_2), mk_binary_and(ctx, in_1, cin), mk_binary_and(ctx, in_2, cin));
//    return std::make_pair(out, cout);
//}

/**
   \brief Create an adder for inputs of size \c num_bits.
   The arguments \c in1 and \c in2 are arrays of bits of size \c num_bits.

   \remark \c result must be an array of size \c num_bits + 1.
*/
void mk_adder(Z3_context ctx, const unsigned num_bits, Z3_ast * in_1, Z3_ast * in_2, Z3_ast * result) {
    Z3_ast cin = Z3_mk_false(ctx);
    for (unsigned i = 0; i < num_bits; i++) {
        result[i] = Z3_mk_xor(ctx, Z3_mk_xor(ctx, in_1[i], in_2[i]), cin);
        cin = mk_ternary_or(ctx, mk_binary_and(ctx, in_1[i], in_2[i]), mk_binary_and(ctx, in_1[i], cin), mk_binary_and(ctx, in_2[i], cin));
    }
    result[num_bits] = cin;
}

/**
   \brief Given \c num_ins "numbers" of size \c num_bits stored in \c in.
   Create floor(num_ins/2) adder circuits. Each circuit is adding two consecutive "numbers".
   The numbers are stored one after the next in the array \c in.
   That is, the array \c in has size num_bits * num_ins.
   Return an array of bits containing \c ceil(num_ins/2) numbers of size \c (num_bits + 1).
   If num_ins/2 is not an integer, then the last "number" in the output, is the last "number" in \c in with an appended "zero".
*/
unsigned mk_adder_pairs(Z3_context ctx, const unsigned num_bits, const unsigned num_ins, Z3_ast * in, Z3_ast * out) {
    unsigned out_num_bits = num_bits + 1;
    Z3_ast * _in          = in;
    Z3_ast * _out         = out;
    unsigned out_num_ins  = (num_ins % 2 == 0) ? (num_ins / 2) : (num_ins / 2) + 1;
    for (unsigned i = 0; i < num_ins / 2; i++) {
        mk_adder(ctx, num_bits, _in, _in + num_bits, _out);
        _in  += num_bits;
        _in  += num_bits;
        _out += out_num_bits;
    }
    if (num_ins % 2 != 0) {
        for (unsigned i = 0; i < num_bits; i++) {
            _out[i] = _in[i];
        }
        _out[num_bits] = Z3_mk_false(ctx);
    }
    return out_num_ins;
}

/**
   \brief Return the \c idx bit of \c val.
*/
inline bool get_bit(unsigned val, unsigned idx) {
    return (val & (1U << (idx & 31))) != 0;
}

/**
   \brief Given an integer val encoded in n bits (boolean variables), assert the constraint that val <= k.
*/
void assert_le_one(Z3_context ctx, Z3_solver s, unsigned n, Z3_ast * val)
{
    Z3_ast i1, i2;
    Z3_ast not_val = Z3_mk_not(ctx, val[0]);
    assert (get_bit(1, 0));
    Z3_ast out = Z3_mk_true(ctx);
    for (unsigned i = 1; i < n; i++) {
        not_val = Z3_mk_not(ctx, val[i]);
        if (get_bit(1, i)) {
            i1 = not_val;
            i2 = out;
        }
        else {
            i1 = Z3_mk_false(ctx);
            i2 = Z3_mk_false(ctx);
        }
        out = mk_ternary_or(ctx, i1, i2, mk_binary_and(ctx, not_val, out));
    }
    Z3_solver_assert(ctx, s, out);
}

/**
   \brief Create a counter circuit to count the number of "ones" in lits.
   The function returns an array of bits (i.e. boolean expressions) containing the output of the circuit.
   The size of the array is stored in out_sz.
*/
void mk_counter_circuit(Z3_context ctx, Z3_solver solver, unsigned n, Z3_ast * lits) {
    unsigned k = 1;
    assert (n != 0);
    Z3_ast aux_array_1[n + 1];
    Z3_ast aux_array_2[n + 1];
    Z3_ast * aux_1 = aux_array_1;
    Z3_ast * aux_2 = aux_array_2;
    std::memcpy(aux_1, lits, sizeof(Z3_ast) * n);
    while (n > 1) {
        assert (aux_1 != aux_2);
        n = mk_adder_pairs(ctx, k++, n, aux_1, aux_2);
        std::swap(aux_1, aux_2);
    }
    assert_le_one(ctx, solver, k, aux_1);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * Fu & Malik procedure for MaxSAT. This procedure is based on unsat core extraction and the at-most-one constraint.
 ** ------------------------------------------------------------------------------------------------------------- */
Z3_bool maxsat(Z3_context ctx, Z3_solver solver, std::vector<Z3_ast> & soft) {
    if (LLVM_UNLIKELY(Z3_solver_check(ctx, solver) == Z3_L_FALSE)) {
        return Z3_L_FALSE;
    }
    if (LLVM_UNLIKELY(soft.empty())) {
        return true;
    }

    const auto n = soft.size();
    const auto ty = Z3_mk_bool_sort(ctx);
    Z3_ast aux_vars[n];
    Z3_ast assumptions[n];

    for (unsigned i = 0; i < n; ++i) {
        aux_vars[i] = Z3_mk_fresh_const(ctx, nullptr, ty);
        Z3_solver_assert(ctx, solver, mk_binary_or(ctx, soft[i], aux_vars[i]));
    }

    for (;;) {
        // create assumptions
        for (unsigned i = 0; i < n; i++) {
            // Recall that we asserted (soft_cnstrs[i] \/ aux_vars[i])
            // So using (NOT aux_vars[i]) as an assumption we are actually forcing the soft_cnstrs[i] to be considered.
            assumptions[i] = Z3_mk_not(ctx, aux_vars[i]);
        }
        if (Z3_solver_check_assumptions(ctx, solver, n, assumptions) != Z3_L_FALSE) {
            return Z3_L_TRUE; // done
        } else {
            Z3_ast_vector core = Z3_solver_get_unsat_core(ctx, solver);
            unsigned m = Z3_ast_vector_size(ctx, core);
            Z3_ast block_vars[m];
            unsigned k = 0;
            // update soft-constraints and aux_vars
            for (unsigned i = 0; i < n; i++) {
                // check whether assumption[i] is in the core or not
                for (unsigned j = 0; j < m; j++) {
                    if (assumptions[i] == Z3_ast_vector_get(ctx, core, j)) {
                        // assumption[i] is in the unsat core... so soft_cnstrs[i] is in the unsat core
                        Z3_ast block_var = Z3_mk_fresh_const(ctx, nullptr, ty);
                        Z3_ast new_aux_var = Z3_mk_fresh_const(ctx, nullptr, ty);
                        soft[i] = mk_binary_or(ctx, soft[i], block_var);
                        aux_vars[i] = new_aux_var;
                        block_vars[k] = block_var;
                        ++k;
                        // Add new constraint containing the block variable.
                        // Note that we are using the new auxiliary variable to be able to use it as an assumption.
                        Z3_solver_assert(ctx, solver, mk_binary_or(ctx, soft[i], new_aux_var) );
                        break;
                    }
                }

            }
            if (k > 1) {
                mk_counter_circuit(ctx, solver, k, block_vars);
            }
        }
    }
    llvm_unreachable("unreachable");
    return Z3_L_FALSE;
}

} // end of namespace pablo
