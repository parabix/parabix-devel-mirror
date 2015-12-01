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

bool AutoMultiplexing::optimize(PabloFunction & function, const unsigned limit, const unsigned maxSelections, const bool independent) {

//    std::random_device rd;
//    const auto seed = rd();
    const auto seed = 83234827342;
    RNG rng(seed);

    LOG("Seed:                    " << seed);

    AutoMultiplexing am(limit, maxSelections);
    RECORD_TIMESTAMP(start_initialize);
    const unsigned advances = am.initialize(function, independent);
    RECORD_TIMESTAMP(end_initialize);

    LOG("Initialize:              " << (end_initialize - start_initialize));

    LOG_NUMBER_OF_ADVANCES(function.getEntryBlock());

    if (advances == 0) {
        return false;
    }

    RECORD_TIMESTAMP(start_characterize);
    am.characterize(function.getEntryBlock());
    RECORD_TIMESTAMP(end_characterize);

    LOG("Characterize:            " << (end_characterize - start_characterize));

    LOG("Nodes in BDD:            " << bdd_getnodenum() << " of " << bdd_getallocnum());

    RECORD_TIMESTAMP(start_shutdown);
    bdd_done();
    RECORD_TIMESTAMP(end_shutdown);
    LOG("Shutdown:                " << (end_shutdown - start_shutdown));

    RECORD_TIMESTAMP(start_create_multiplex_graph);
    const bool multiplex = am.generateCandidateSets(rng);
    RECORD_TIMESTAMP(end_create_multiplex_graph);
    LOG("GenerateCandidateSets:   " << (end_create_multiplex_graph - start_create_multiplex_graph));

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
        am.multiplexSelectedIndependentSets(function);
        RECORD_TIMESTAMP(end_select_independent_sets);
        LOG("SelectedIndependentSets: " << (end_select_independent_sets - start_select_independent_sets));

        AutoMultiplexing::topologicalSort(function);
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
unsigned AutoMultiplexing::initialize(PabloFunction & function, const bool independent) {

    flat_map<const PabloAST *, unsigned> map;
    std::stack<Statement *> scope;
    unsigned variableCount = 0; // number of statements that cannot always be categorized without generating a new variable

    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    unsigned statements = 0, advances = 0;
    mResolvedScopes.emplace(function.getEntryBlock(), nullptr);
    for (Statement * stmt = function.getEntryBlock()->front(); ; ) {
        while ( stmt ) {
            ++statements;
            if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                // Set the next statement to be the first statement of the inner scope and push the
                // next statement of the current statement into the scope stack.
                const PabloBlock * const nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                mResolvedScopes.emplace(nested, stmt);
                scope.push(stmt->getNextNode());
                stmt = nested->front();
                assert (stmt);
                continue;
            }

            assert ("Run the Simplifer pass prior to this!" && (stmt->getNumUses() > 0));

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

    // Create the transitive closure matrix of graph. From this we'll construct
    // two graphs restricted to the relationships between advances. The first will
    // be a path graph, which is used to bypass testing for mutual exclusivity of
    // advances that cannot be multiplexed. The second is a transitive reduction
    // of that graph, which forms the basis of our constraint graph when deciding
    // which advances ought to be multiplexed.

    matrix<bool> G(statements, advances, false);
    for (unsigned i = 0; i != advances; ++i) {
        G(i, i) = true;
    }

    unsigned n = advances;
    unsigned m = 0;

    for (const Statement * stmt = function.getEntryBlock()->front();;) {
        while ( stmt ) {

            unsigned u = 0;
            if (isa<Advance>(stmt)) {
                u = m++;
            } else {
                u = n++;
            }
            map.emplace(stmt, u);

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

    // Can I use the data in the matrix to indicate whether an Advance is dependent on a particular instruction and only
    // for which there is still a use left of it?

    // Record the path / base constraint graph after removing any reflexive-loops.
    // Since G is a use-def graph and we want our constraint graph to be a def-use graph,
    // reverse the edges as we're writing them to obtain the transposed graph.

    mConstraintGraph = ConstraintGraph(advances);
    mSubsetGraph = SubsetGraph(advances);

    for (unsigned i = 0; i != advances; ++i) {
        G(i, i) = false;
        for (unsigned j = 0; j != advances; ++j) {
            if (G(i, j)) {
                add_edge(j, i, mConstraintGraph);
            }
        }
    }

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
 * @brief characterize
 * @param vars the input vars for this program
 *
 * Scan through the program and iteratively compute the BDDs for each statement.
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::characterize(PabloBlock * const block) {
    for (Statement * stmt : *block) {
        if (LLVM_UNLIKELY(isa<If>(stmt))) {
            characterize(cast<If>(stmt)->getBody());
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            const auto & variants = cast<While>(stmt)->getVariants();
            std::vector<BDD> assignments(variants.size());
            for (unsigned i = 0; i != variants.size(); ++i) {
                assignments[i] = get(variants[i]->getInitial());
            }
            characterize(cast<While>(stmt)->getBody());
            for (unsigned i = 0; i != variants.size(); ++i) {
                BDD & var = get(variants[i]->getInitial());
                var = bdd_addref(bdd_or(var, assignments[i]));
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
inline BDD AutoMultiplexing::characterize(Statement * const stmt) {
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
            // ScanThru(c, m) := (c + m) ∧ ¬m. We can conservatively represent this statement using the BDD for ¬m --- provided
            // no derivative of this statement is negated in any fashion.
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
inline BDD AutoMultiplexing::characterize(Advance * const adv, const BDD Ik) {

    const auto k = mAdvanceAttributes.size();
    std::vector<bool> unconstrained(k , false);

    for (unsigned i = 0; i != k; ++i) {
        // Have we already proven that these are unconstrained by the subset relationships?
        if (unconstrained[i]) continue;

        // If these advances are "shifting" their values by the same amount ...
        const Advance * const ithAdv = std::get<0>(mAdvanceAttributes[i]);
        if (independent(i, k) && adv->getOperand(1) == ithAdv->getOperand(1)) {
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

    const BDD Vk = bdd_addref(bdd_not(bdd_ithvar(mVariables++)));
    BDD Ck = bdd_one();
    for (unsigned i = 0; i != k; ++i) {
        const Advance * const ithAdv = std::get<0>(mAdvanceAttributes[i]);
        BDD & Ci = get(ithAdv);
        const BDD Vi = std::get<1>(mAdvanceAttributes[i]);
        if (unconstrained[i]) {
            const BDD exclusionConstraint = bdd_addref(bdd_or(Vi, Vk));
            Ci = bdd_addref(bdd_and(Ci, exclusionConstraint));
            Ck = bdd_addref(bdd_and(Ck, exclusionConstraint));
            // If these Advances are mutually exclusive, in the same scope and transitively independent,
            // we can safely multiplex them. Otherwise mark the constraint edge in the graph.
            if (adv->getParent() == ithAdv->getParent()) {
                continue;
            }
        }
        add_edge(i, k, mConstraintGraph);
    }

    mAdvanceAttributes.emplace_back(adv, Vk);

    return Ck;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief independent
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool AutoMultiplexing::independent(const ConstraintVertex i, const ConstraintVertex j) const {
    assert (i < num_vertices(mConstraintGraph) && j < num_vertices(mConstraintGraph));
    return (mConstraintGraph.get_edge(i, j) == 0);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateMultiplexSets
 * @param RNG random number generator
 ** ------------------------------------------------------------------------------------------------------------- */
bool AutoMultiplexing::generateCandidateSets(RNG & rng) {

    using degree_t = graph_traits<ConstraintGraph>::degree_size_type;

    // What if we generated a "constraint free" graph instead? By taking each connected component of it
    // and computing the complement of it (with the same lesser to greater index ordering), we should
    // have the same problem here but decomposed into subgraphs.

    VertexVector S;
    std::vector<degree_t> D(num_vertices(mConstraintGraph));
    S.reserve(15);

    mMultiplexSetGraph = MultiplexSetGraph(num_vertices(mConstraintGraph));

    // Push all source nodes into the new independent set N
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

        addCandidateSet(S, rng);

        bool noNewElements = true;
        do {
            assert (S.size() > 0);
            // Randomly choose a vertex in S and discard it.
            const auto i = S.begin() + IntDistribution(0, S.size() - 1)(rng);
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
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned long choose(const unsigned n, const unsigned k) {
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
        while (choose(--a, b) > x);
        x = x - choose(a, b);
        b = b - 1;
        element[i] = (n - 1) - a;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addCandidateSet
 * @param S an independent set
 ** ------------------------------------------------------------------------------------------------------------- */
inline void AutoMultiplexing::addCandidateSet(const VertexVector & S, RNG & rng) {
    if (S.size() >= 3) {
        if (S.size() <= mLimit) {
            const auto u = add_vertex(mMultiplexSetGraph);
            for (const auto v : S) {
                add_edge(u, v, mMultiplexSetGraph);
            }
        } else {
            const auto max = choose(S.size(), mLimit);
            unsigned element[mLimit];
            if (LLVM_UNLIKELY(max <= mMaxSelections)) {
                for (unsigned i = 0; i != max; ++i) {
                    select(S.size(), mLimit, i, element);
                    const auto u = add_vertex(mMultiplexSetGraph);
                    for (unsigned j = 0; j != mLimit; ++j) {
                        add_edge(u, S[element[j]], mMultiplexSetGraph);
                    }
                }
            } else { // take m random samples
                for (unsigned i = 0; i != mMaxSelections; ++i) {
                    select(S.size(), mLimit, rng() % max, element);
                    const auto u = add_vertex(mMultiplexSetGraph);
                    for (unsigned j = 0; j != mLimit; ++j) {
                        add_edge(u, S[element[j]], mMultiplexSetGraph);
                    }
                }
            }
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
 * enough but can be trivially shown to produce a suboptimal solution if there are three (or more) sets in which
 * two, labelled A and B, are disjoint and the third larger set, C, that consists of elements of A and B.
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::selectMultiplexSets(RNG &) {

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
        if (is_power_of_2(w)) {
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

    using SubsetEdgeIterator = graph_traits<SubsetGraph>::edge_iterator;

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
        // we perform the minimum number of AST modifications for the given multiplexing sets.

        doTransitiveReductionOfSubsetGraph();

        // Afterwards modify the AST to ensure that multiplexing algorithm can ignore any subset constraints
        for (auto e : make_iterator_range(edges(mSubsetGraph))) {
            Advance * adv1 = std::get<0>(mAdvanceAttributes[source(e, mSubsetGraph)]);
            Advance * adv2 = std::get<0>(mAdvanceAttributes[target(e, mSubsetGraph)]);
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
 * @brief multiplexSelectedIndependentSets
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::multiplexSelectedIndependentSets(PabloFunction &) {

    const unsigned first_set = num_vertices(mConstraintGraph);
    const unsigned last_set = num_vertices(mMultiplexSetGraph);

    // Preallocate the structures based on the size of the largest multiplex set
    size_t max_n = 3;
    for (unsigned idx = first_set; idx != last_set; ++idx) {
        max_n = std::max<unsigned>(max_n, out_degree(idx, mMultiplexSetGraph));
    }

    circular_buffer<PabloAST *> Q(max_n);

    // When entering thus function, the multiplex set graph M is a DAG with edges denoting the set
    // relationships of our independent sets.

    for (unsigned idx = first_set; idx != last_set; ++idx) {
        const size_t n = out_degree(idx, mMultiplexSetGraph);
        if (n) {
            const size_t m = log2_plus_one(n);
            Advance * input[n];
            Advance * muxed[m];

            unsigned i = 0;
            for (const auto e : make_iterator_range(out_edges(idx, mMultiplexSetGraph))) {
                input[i++] = std::get<0>(mAdvanceAttributes[target(e, mMultiplexSetGraph)]);
            }

            Advance * const adv = input[0];
            PabloBlock * const block = adv->getParent(); assert (block);
            PabloBuilder builder(block);
            block->setInsertPoint(block->back());

            /// Perform n-to-m Multiplexing
            for (size_t j = 0; j != m; ++j) {

                std::ostringstream prefix;
                prefix << "mux" << n << "to" << m << '.' << (j + 1);
                for (size_t i = 0; i != n; ++i) {
                    if (((i + 1) & (1UL << j)) != 0) {
                        assert (input[i]->getParent() == block);
                        Q.push_back(input[i]->getOperand(0));
                    }
                }

                while (Q.size() > 1) {
                    PabloAST * a1 = Q.front(); Q.pop_front(); assert (a1);
                    PabloAST * a2 = Q.front(); Q.pop_front(); assert (a2);
                    assert (!Q.full());
                    Q.push_back(builder.createOr(a2, a1, "muxing"));
                }

                PabloAST * mux = Q.front(); Q.pop_front(); assert (mux);
                // The only way this did not return an Advance statement would be if either the mux or shift amount
                // is zero. Since these cases would have been eliminated earlier, we are safe to cast here.
                muxed[j] = cast<Advance>(builder.createAdvance(mux, adv->getOperand(1), prefix.str()));
            }

            /// Perform m-to-n Demultiplexing
            for (size_t i = 0; i != n; ++i) {

                // Construct the demuxed values and replaces all the users of the original advances with them.
                for (size_t j = 0; j != m; ++j) {
                    if (((i + 1) & (1UL << j)) == 0) {
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
                    PabloAST * neg = Q.front(); Q.pop_front(); assert (neg);
                    Q.push_back(builder.createNot(neg, "demuxing"));
                }

                for (unsigned j = 0; j != m; ++j) {
                    if (((i + 1) & (1ULL << j)) != 0) {
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

                PabloAST * demuxed = Q.front(); Q.pop_front(); assert (demuxed);
                input[i]->replaceWith(demuxed, true, true);
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
 * users within the IR isn't taken into consideration. Thus while there must be a valid ordering for the program,
 * it's not necessarily the original ordering.
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::topologicalSort(PabloFunction & function) {
    // Note: not a real topological sort. I expect the original order to be very close to the resulting one.
    std::unordered_set<const PabloAST *> encountered;
    std::stack<Statement *> scope;
    for (Statement * stmt = function.getEntryBlock()->front(); ; ) { restart:
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
                const PabloBlock * const nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                scope.push(stmt->getNextNode());
                stmt = nested->front();
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief doTransitiveReductionOfSubsetGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::doTransitiveReductionOfSubsetGraph() {
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
 * @brief get
 ** ------------------------------------------------------------------------------------------------------------- */
inline BDD & AutoMultiplexing::get(const PabloAST * const expr) {
    auto f = mCharacterization.find(expr);
    assert (f != mCharacterization.end());
    return f->second;
}

} // end of namespace pablo
