#include <pablo/pablo_automultiplexing.hpp>

#include <pablo/builder.hpp>
#include <pablo/printer_pablos.h>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/range/iterator_range.hpp>
#ifndef NDEBUG
#include <pablo/analysis/pabloverifier.hpp>
#endif
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/builder.hpp>
#include <stack>
#include <queue>
#include <unordered_set>
#include <functional>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/ErrorHandling.h>
#include <util/maxsat.hpp>

using namespace llvm;
using namespace boost;
using namespace boost::container;
using namespace boost::numeric::ublas;

static cl::OptionCategory MultiplexingOptions("Multiplexing Optimization Options", "These options control the Pablo Multiplexing optimization pass.");

static cl::opt<unsigned> WindowSize("multiplexing-window-size", cl::init(100),
                                        cl::desc("maximum sequence distance to consider for candidate set."),
                                        cl::cat(MultiplexingOptions));


namespace pablo {

using TypeId = PabloAST::ClassTypeId;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 * @param function the function to optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool MultiplexingPass::optimize(PabloFunction & function) {


    Z3_config cfg = Z3_mk_config();
    Z3_context ctx = Z3_mk_context_rc(cfg);
    Z3_del_config(cfg);
    Z3_solver solver = Z3_mk_solver(ctx);
    Z3_solver_inc_ref(ctx, solver);

    MultiplexingPass mp(function, ctx, solver);

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
    PabloBlock * entryBlock = mFunction.getEntryScope();
    add(entryBlock->createZeroes(), Z3_mk_false(mContext), -1);
    add(entryBlock->createOnes(), Z3_mk_true(mContext), -1);
    for (unsigned i = 0; i < mFunction.getNumOfParameters(); ++i) {
        add(mFunction.getParameter(i), makeVar(), -1);
    }
    optimize(mFunction.getEntryScope());
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
                for (const Var * var : cast<While>(stmt)->getEscaped()) {
                    Z3_inc_ref(mContext, get(var));
                }
                optimize(cast<While>(stmt)->getBody());
                // since we cannot be certain that we'll always execute at least one iteration of a loop, we must
                // assume that the variants could either be their initial or resulting value.
                for (const Var * var : cast<While>(stmt)->getEscaped()) {
                    Z3_ast v0 = get(var);
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
        // eliminateSubsetConstraints();
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
static void handle_unexpected_statement(const Statement * const stmt) {
    std::string tmp;
    raw_string_ostream err(tmp);
    err << "Unexpected statement type: ";
    PabloPrinter::print(stmt, err);
    throw std::runtime_error(err.str());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
Z3_ast MultiplexingPass::characterize(const Statement * const stmt, const bool deref) {

    const size_t n = stmt->getNumOperands(); assert (n > 0);
    Z3_ast operands[n];
    for (size_t i = 0; i < n; ++i) {
        PabloAST * op = stmt->getOperand(i);
        if (LLVM_UNLIKELY(isa<Integer>(op) || isa<String>(op))) {
            continue;
        }
        operands[i] = get(op, deref);
    }

    Z3_ast node = operands[0];
    switch (stmt->getClassTypeId()) {
        case TypeId::Assign:
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
            return add(stmt, node, stmt->getNumUses());
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
            node = makeVar();
            break;
        default:
            handle_unexpected_statement(stmt);
    }
    Z3_inc_ref(mContext, node);
    return add(stmt, node, stmt->getNumUses());
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
inline Z3_ast MultiplexingPass::characterize(const Advance * const adv, Z3_ast Ik) {
    const auto k = mNegatedAdvance.size();

    assert (adv);
    assert (mConstraintGraph[k] == adv);
    std::vector<bool> unconstrained(k);

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
//                for (auto e : make_iterator_range(in_edges(i, mSubsetGraph))) {
//                    unconstrained[source(e, mSubsetGraph)] = true;
//                }
                unconstrained[i] = true;
            }/* else if (equals(Ii, IiIk)) {
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
            }*/
            Z3_dec_ref(mContext, IiIk);
            Z3_solver_pop(mContext, mSolver, 1);
        }
    }

    Z3_solver_pop(mContext, mSolver, 1);

    Z3_ast Ak0 = makeVar();
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
        const auto ei = add_edge(i, k, mConstraintGraph);
        // if this is not a new edge, it must have a dependency constraint.
        if (ei.second) {
            mConstraintGraph[ei.first] = ConstraintType::Inclusive;
        }
    }
    // To minimize the number of BDD computations, we store the negated variable instead of negating it each time.
    mNegatedAdvance.emplace_back(Nk);
    Z3_ast Ak = Z3_mk_and(mContext, m, vars);
    if (LLVM_UNLIKELY(Ak != Ak0)) {
        Z3_inc_ref(mContext, Ak);
        Z3_dec_ref(mContext, Ak0);
    }
    return add(adv, Ak, -1);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initialize
 ** ------------------------------------------------------------------------------------------------------------- */
Statement * MultiplexingPass::initialize(Statement * const initial) {

    // clean up any unneeded refs / characterizations.
    for (auto i = mCharacterization.begin(); i != mCharacterization.end(); ) {
        const auto ref = i->second;
        auto next = i; ++next;
        if (LLVM_UNLIKELY(ref.second == 0)) {
            assert (isa<Statement>(i->first));
            Z3_dec_ref(mContext, ref.first);
            mCharacterization.erase(i);
        }
        i = next;
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
                mConstraintGraph[add_edge(j, i, mConstraintGraph).first] = ConstraintType::Dependency;
            }
        }
        for (unsigned j = i + 1; j < advances; ++j) {
            if (G(i, j)) {
                mConstraintGraph[add_edge(j, i, mConstraintGraph).first] = ConstraintType::Dependency;
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

    mCandidateGraph = CandidateGraph(n);

    Z3_config cfg = Z3_mk_config();
    Z3_set_param_value(cfg, "MODEL", "true");
    Z3_context ctx = Z3_mk_context(cfg);
    Z3_del_config(cfg);
    Z3_solver solver = Z3_mk_solver(ctx);
    Z3_solver_inc_ref(ctx, solver);

    std::vector<Z3_ast> V(n);
    for (unsigned i = 0; i < n; ++i) {
        V[i] = Z3_mk_fresh_const(ctx, nullptr, Z3_mk_bool_sort(ctx)); assert (V[i]);
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
                    generateCandidateSets(ctx, solver, S, V);
                }
                // erase any that preceed our window
                auto end = S.begin();
                while (++end != S.end() && ((line - std::get<0>(*end)) > WindowSize));
                S.erase(S.begin(), end);
            }
            for (unsigned j : make_iterator_range(adjacent_vertices(i, mConstraintGraph))) {
                Z3_ast disj[2] = { Z3_mk_not(ctx, V[j]), Z3_mk_not(ctx, V[i]) };
                Z3_solver_assert(ctx, solver, Z3_mk_or(ctx, 2, disj));
            }
            S.emplace_back(line, i++);
        }
        ++line;
    }
    if (S.size() > 2) {
        generateCandidateSets(ctx, solver, S, V);
    }

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);

    return num_vertices(mCandidateGraph) > n;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateCandidateSets
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiplexingPass::generateCandidateSets(Z3_context ctx, Z3_solver solver, const std::vector<std::pair<unsigned, unsigned>> & S, const std::vector<Z3_ast> & V) {
    assert (S.size() > 2);
    assert (std::get<0>(S.front()) < std::get<0>(S.back()));
    assert ((std::get<0>(S.back()) - std::get<0>(S.front())) <= WindowSize);

    Z3_solver_push(ctx, solver);

    const auto n = V.size();
    std::vector<unsigned> M(S.size());
    for (unsigned i = 0; i < S.size(); ++i) {
        M[i] = std::get<1>(S[i]);
    }

    for (;;) {

        std::vector<Z3_ast> assumptions(M.size());
        unsigned j = 0;
        for (unsigned i = 0; i < n; ++i) {
            if (LLVM_UNLIKELY((j < M.size()) && (M[j] == i))) { // in our window range
                assumptions[j++] = V[i]; assert (V[i]);
            } else {
                Z3_solver_assert(ctx, solver, Z3_mk_not(ctx, V[i]));
            }
        }
        assert (j == M.size());

        if (Z3_maxsat(ctx, solver, assumptions) >= 0) {
            Z3_model m = Z3_solver_get_model(ctx, solver);
            Z3_model_inc_ref(ctx, m);
            const auto k = add_vertex(mCandidateGraph); assert(k >= V.size());
            Z3_ast TRUE = Z3_mk_true(ctx);
            for (auto i = M.begin(); i != M.end(); ) {
                Z3_ast value;
                if (LLVM_UNLIKELY(Z3_model_eval(ctx, m, V[*i], 1, &value) != Z3_TRUE)) {
                    throw std::runtime_error("Unexpected Z3 error when attempting to obtain value from constraint model!");
                }
                if (value == TRUE) {
                    add_edge(*i, k, mCandidateGraph);
                    Z3_solver_assert(ctx, solver, Z3_mk_not(ctx, V[*i]));
                    i = M.erase(i);
                } else {
                    ++i;
                }
            }
            Z3_model_dec_ref(ctx, m);
            if (M.size() > 2) {
                continue;
            }
        }
        break;
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

    std::vector<bool> chosen(n);

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

    // TODO: should we test whether sets overlap and merge the computations together?

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
            PabloAST * demuxed[n];

            // The multiplex set graph is a DAG with edges denoting the set relationships of our independent sets.
            unsigned i = 0;
            for (const auto u : make_iterator_range(adjacent_vertices(idx, mCandidateGraph))) {
                input[i] = mConstraintGraph[u];
                assert ("Not all inputs are in the same block!" && (input[i]->getParent() == block));
                assert ("Not all inputs advance by the same amount!" && (input[i]->getOperand(1) == input[0]->getOperand(1)));
                ++i;
            }

            // We can't trust the AST will be in the original order as we can multiplex a region of the program
            // more than once.
            Statement * initial = nullptr, * sentinal = nullptr;
            for (Statement * stmt : *block) {
                if (isa<Advance>(stmt)) {
                    for (unsigned i = 0; i < n; ++i) {
                        if (stmt == input[i]) {
                            initial = initial ? initial : stmt;
                            sentinal = stmt;
                            break;
                        }
                    }
                }
            }
            assert (initial);

            Statement * const ip = initial->getPrevNode(); // save our insertion point prior to modifying the AST
            sentinal = sentinal->getNextNode();

            Z3_solver_push(ctx, solver);

            // Compute the hard and soft constraints for any part of the AST that we are not intending to modify.
            flat_map<Statement *, Z3_ast> M;

            Z3_ast prior = nullptr;
            Z3_ast one = Z3_mk_int(ctx, 1, Z3_mk_int_sort(ctx));
            std::vector<Z3_ast> ordering;

            for (Statement * stmt = initial; stmt != sentinal; stmt = stmt->getNextNode()) { assert (stmt != ip);
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
                prior = node;
            }

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
                demuxed[i] = Q.front(); Q.clear();

                const auto f = M.find(input[i]);
                assert (f != M.end());
                Z3_solver_assert(ctx, solver, Z3_mk_eq(ctx, f->second, replacement));
                M.erase(f);
            }

            assert (M.count(ip) == 0);

            const auto satisfied = Z3_maxsat(ctx, solver, ordering);

            if (LLVM_UNLIKELY(satisfied >= 0)) {

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

                for (unsigned i = 0; i < n; ++i) {
                    input[i]->replaceWith(demuxed[i], true, true);
                    auto ref = mCharacterization.find(input[i]);
                    assert (ref != mCharacterization.end());
                    add(demuxed[i], std::get<0>(ref->second), -1);
                }

            } else { // fatal error; delete any statements we created.

                for (unsigned i = 0; i < n; ++i) {
                    if (LLVM_LIKELY(isa<Statement>(demuxed[i]))) {
                        cast<Statement>(demuxed[i])->eraseFromParent(true);
                    }
                }

            }

            Z3_solver_pop(ctx, solver, 1);
        }
    }

    Z3_solver_dec_ref(ctx, solver);
    Z3_del_context(ctx);
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
    if (LLVM_UNLIKELY(f == mCharacterization.end())) {
        characterize(cast<Statement>(expr), false);
        f = mCharacterization.find(expr);
        assert (f != mCharacterization.end());
    }
    CharacterizationRef & ref = f->second;
    if (deref) {
        if (LLVM_LIKELY(std::get<1>(ref)) > 0) {
            std::get<1>(ref) -= 1;
        }
    }
    return std::get<0>(ref);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief make
 ** ------------------------------------------------------------------------------------------------------------- */
inline Z3_ast MultiplexingPass::makeVar() {
    Z3_ast node = Z3_mk_fresh_const(mContext, nullptr, Z3_mk_bool_sort(mContext));
    Z3_inc_ref(mContext, node);
    return node;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief add
 ** ------------------------------------------------------------------------------------------------------------- */
inline Z3_ast MultiplexingPass::add(const PabloAST * const expr, Z3_ast node, const size_t refs) {
    mCharacterization.insert(std::make_pair(expr, std::make_pair(node, refs)));
    return node;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
inline MultiplexingPass::MultiplexingPass(PabloFunction & f, Z3_context context, Z3_solver solver)
: mContext(context)
, mSolver(solver)
, mFunction(f)
, mConstraintGraph(0)
{

}

} // end of namespace pablo
