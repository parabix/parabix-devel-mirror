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
#include <llvm/ADT/BitVector.h>
#include <cudd.h>
#include <cuddInt.h>
#include <util.h>
#include <stack>
#include <queue>
#include <unordered_set>
#include <pablo/optimizers/pablo_simplifier.hpp>

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

bool AutoMultiplexing::optimize(PabloFunction & function) {

    // std::random_device rd;
    const auto seed = 83234827342;
    RNG rng(seed);

    LOG("Seed:                    " << seed);

    AutoMultiplexing am;
    RECORD_TIMESTAMP(start_initialize);
    const bool abort = am.initialize(function);
    RECORD_TIMESTAMP(end_initialize);

    LOG("Initialize:              " << (end_initialize - start_initialize));

    LOG_NUMBER_OF_ADVANCES(function.getEntryBlock());

    if (abort) {
        return false;
    }

    RECORD_TIMESTAMP(start_characterize);
    am.characterize(function.getEntryBlock());
    RECORD_TIMESTAMP(end_characterize);

    LOG("Characterize:            " << (end_characterize - start_characterize));

    RECORD_TIMESTAMP(start_create_multiplex_graph);
    const bool multiplex = am.generateCandidateSets(rng);
    RECORD_TIMESTAMP(end_create_multiplex_graph);
    LOG("GenerateCandidateSets:   " << (end_create_multiplex_graph - start_create_multiplex_graph));

    RECORD_TIMESTAMP(start_shutdown);
    am.Shutdown();
    RECORD_TIMESTAMP(end_shutdown);
    LOG("Shutdown:                " << (end_shutdown - start_shutdown));

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
        LOG("SelectedIndependentSets: " << (end_select_independent_sets - start_select_independent_sets));

        RECORD_TIMESTAMP(start_topological_sort);
        am.topologicalSort(function.getEntryBlock());
        RECORD_TIMESTAMP(end_topological_sort);
        LOG("TopologicalSort (1):     " << (end_topological_sort - start_topological_sort));

//        RECORD_TIMESTAMP(start_simplify_ast);
//        am.simplifyAST(function);
//        RECORD_TIMESTAMP(end_simplify_ast);
//        LOG("SimplifyAST:             " << (end_simplify_ast - start_simplify_ast));

//        RECORD_TIMESTAMP(start_topological_sort2);
//        am.topologicalSort(function.getEntryBlock());
//        RECORD_TIMESTAMP(end_topological_sort2);
//        LOG("TopologicalSort (2):     " << (end_topological_sort2 - start_topological_sort2));

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
bool AutoMultiplexing::initialize(PabloFunction & function) {

    flat_map<const PabloAST *, unsigned> map;    
    std::stack<Statement *> scope;
    unsigned variableCount = 0; // number of statements that cannot always be categorized without generating a new variable

    // Scan through and collect all the advances, calls, scanthrus and matchstars ...
    unsigned n = 0, m = 0;
    for (Statement * stmt = function.getEntryBlock().front(); ; ) {
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

            assert ("Run the Simplifer pass prior to this!" && (stmt->getNumUses() > 0));

            switch (stmt->getClassTypeId()) {
                case PabloAST::ClassTypeId::Advance:
                    mAdvanceMap.emplace(stmt, m);
                    map.emplace(stmt, m++);
                case PabloAST::ClassTypeId::Call:
                case PabloAST::ClassTypeId::ScanThru:
                case PabloAST::ClassTypeId::MatchStar:
                    variableCount++;
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
    if (m < 3) {
        return true;
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

    const Statement * stmt = function.getEntryBlock().front();
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
    mManager = Cudd_Init((variableCount + function.getNumOfParameters()), 0, CUDD_UNIQUE_SLOTS, CUDD_CACHE_SLOTS, 0);
    Cudd_AutodynDisable(mManager);

    // Map the predefined 0/1 entries
    mCharacterizationMap[function.getEntryBlock().createZeroes()] = Zero();
    mCharacterizationMap[function.getEntryBlock().createOnes()] = One();

    // Order the variables so the input Vars are pushed to the end; they ought to
    // be the most complex to resolve.
    for (auto i = 0; i != function.getNumOfParameters(); ++i) {
        mCharacterizationMap[function.getParameter(i)] = Cudd_bddIthVar(mManager, variableCount++);
    }

    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 * @param vars the input vars for this program
 *
 * Scan through the program and iteratively compute the BDDs for each statement.
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::characterize(PabloBlock &block) {
    for (Statement * stmt : block) {
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            const auto start = mRecentCharacterizations.size();
            characterize(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
            assert (mRecentCharacterizations.size() >= start);
            if (isa<If>(stmt)) {
                const auto & defined = cast<const If>(stmt)->getDefined();
                for (auto pair = mRecentCharacterizations.begin() + start; pair != mRecentCharacterizations.end(); ++pair) {
                    const PabloAST * expr = nullptr;
                    DdNode * bdd = nullptr;
                    std::tie(expr, bdd) = *pair;
                    if (LLVM_UNLIKELY(isa<Assign>(expr))) {
                        if (LLVM_LIKELY(std::find(defined.begin(), defined.end(), expr) != defined.end())) {
                            continue;
                        }
                    }
                    mCharacterizationMap.erase(mCharacterizationMap.find(expr));
                    if (LLVM_UNLIKELY(Cudd_IsConstant(bdd))) {
                        continue;
                    }
                    Deref(bdd);
                }
            }
            else { // if isa<While>(stmt)
                const auto & variants = cast<const While>(stmt)->getVariants();
                for (auto pair = mRecentCharacterizations.begin() + start; pair != mRecentCharacterizations.end(); ++pair) {
                    const PabloAST * expr = nullptr;
                    DdNode * bdd = nullptr;
                    std::tie(expr, bdd) = *pair;
                    if (LLVM_UNLIKELY(isa<Next>(expr))) {
                        if (LLVM_LIKELY(std::find(variants.begin(), variants.end(), expr) != variants.end())) {
                            DdNode *& next = mCharacterizationMap[expr];
                            next = Or(next, bdd);
                            Ref(next);
                            continue;
                        }
                    }
                    mCharacterizationMap.erase(mCharacterizationMap.find(expr));
                    if (LLVM_UNLIKELY(Cudd_IsConstant(bdd))) {
                        continue;
                    }
                    Deref(bdd);
                }
            }
            mRecentCharacterizations.erase(mRecentCharacterizations.begin() + start, mRecentCharacterizations.end());
            continue;
        }

        DdNode * var = characterize(stmt);
        mCharacterizationMap[stmt] = var;        
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
inline DdNode * AutoMultiplexing::characterize(Statement * const stmt) {

    DdNode * bdd = nullptr;
    // Map our operands to the computed BDDs
    std::array<DdNode *, 3> input;
    unsigned count = 0;
    for (; count != stmt->getNumOperands(); ++count) {
        PabloAST * const op = stmt->getOperand(count);
        if (LLVM_UNLIKELY(isa<Integer>(op) || isa<String>(op))) {
            continue;
        }
        auto f = mCharacterizationMap.find(op);
        if (LLVM_UNLIKELY(f == mCharacterizationMap.end())) {
            std::string tmp;
            llvm::raw_string_ostream msg(tmp);
            msg << "Uncharacterized operand " << std::to_string(count);
            PabloPrinter::print(stmt, " of ", msg);
            throw std::runtime_error(msg.str());
        }
        input[count] = f->second;
    }

    switch (stmt->getClassTypeId()) {
        case PabloAST::ClassTypeId::Assign:
        case PabloAST::ClassTypeId::Next:
            bdd = input[0];
            break;
        case PabloAST::ClassTypeId::And:
            bdd = And(input[0], input[1]);
            break;        
        case PabloAST::ClassTypeId::Or:
            bdd = Or(input[0], input[1]);
            break;
        case PabloAST::ClassTypeId::Xor:
            bdd = Xor(input[0], input[1]);
            break;
        case PabloAST::ClassTypeId::Not:
            bdd = Not(input[0]);
            break;
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
            bdd = NewVar();
            mRecentCharacterizations.emplace_back(stmt, bdd);
            return bdd;
        case PabloAST::ClassTypeId::Advance:
            // This returns so that it doesn't mistakeningly replace the Advance with 0 or add it
            // to the list of recent characterizations.
            return characterize(cast<Advance>(stmt), input[0]);
        default:
            throw std::runtime_error("Unexpected statement type " + stmt->getName()->to_string());
    }

    Ref(bdd);
    if (LLVM_UNLIKELY(NoSatisfyingAssignment(bdd))) {
        Deref(bdd);
        // If there is no satisfing assignment for this bdd, the statement will always produce 0.
        // We can safely replace this statement with 0 unless it is an Advance, Assign or Next node.
        // Those must be handled specially or we may end up producing a non-equivalent function.
        if (LLVM_UNLIKELY(isa<Advance>(stmt) || isa<Assign>(stmt) || isa<Next>(stmt))) {
            stmt->setOperand(0, stmt->getParent()->createZeroes());
            bdd = Zero();
        }
        else {
            stmt->replaceWith(stmt->getParent()->createZeroes());
            return nullptr;
        }
    }
    mRecentCharacterizations.emplace_back(stmt, bdd);
    return bdd;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
inline DdNode * AutoMultiplexing::characterize(Advance *adv, DdNode * input) {
    DdNode * Ik, * Ck, * Nk;
    if (LLVM_UNLIKELY(isZero(input))) {
        Ik = Ck = Nk = Zero();
    }
    else {

        Ik = input;
        Ref(input);
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
                DdNode * const IiIk = And(Ik, Ii);
                Ref(IiIk);
                // Is there any satisfying truth assignment? If not, these streams are mutually exclusive.
                if (NoSatisfyingAssignment(IiIk)) {
                    assert (mCharacterizationMap.count(tmp));
                    DdNode *& Ci = mCharacterizationMap[tmp];
                    // Mark the i-th and k-th Advances as being mutually exclusive
                    DdNode * Ni = std::get<2>(mAdvance[i]);
                    Ck = And(Ck, Ni); Ref(Ck);
                    Ci = And(Ci, Nk); Ref(Ci);
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
                            Ck = And(Ck, Nj); Ref(Ck);
                            Cj = And(Cj, Nk); Ref(Cj);
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
                Deref(IiIk);
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
    DdNode * var = Cudd_bddIthVar(mManager, mVariables++);
    return var;
}

inline bool AutoMultiplexing::isZero(DdNode * const x) const {
    return x == Zero();
}

inline DdNode * AutoMultiplexing::And(DdNode * const x, DdNode * const y) {
    return Cudd_bddAnd(mManager, x, y);
}

inline DdNode * AutoMultiplexing::Or(DdNode * const x, DdNode * const y) {
    return Cudd_bddOr(mManager, x, y);
}

inline DdNode * AutoMultiplexing::Xor(DdNode * const x, DdNode * const y) {
    return Cudd_bddXor(mManager, x, y);
}

inline DdNode * AutoMultiplexing::Not(DdNode * const x) const {
    return Cudd_Not(x);
}

inline DdNode * AutoMultiplexing::Ite(DdNode * const x, DdNode * const y, DdNode * const z) {
    return Cudd_bddIte(mManager, x, y, z);
}

inline bool AutoMultiplexing::NoSatisfyingAssignment(DdNode * const x) {
    return Cudd_bddLeq(mManager, x, Zero());
}

inline void AutoMultiplexing::Ref(DdNode * const x) {
    assert (x);
    Cudd_Ref(x);
}

inline void AutoMultiplexing::Deref(DdNode * const x) {
    assert (x);
    Cudd_RecursiveDeref(mManager, x);
}

inline void AutoMultiplexing::Shutdown() {
//    #ifdef PRINT_DEBUG_OUTPUT
//    Cudd_PrintInfo(mManager, stderr);
//    #endif
    Cudd_Quit(mManager);
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

    auto remainingVerticies = num_vertices(mConstraintGraph) - S.size();

    do {

        addCandidateSet(S);

        bool noNewElements = true;
        do {
            // Randomly choose a vertex in S and discard it.
            const auto i = S.begin() + IntDistribution(0, S.size() - 1)(rng);
            const ConstraintVertex u = *i;
            S.erase(i);
            --remainingVerticies;

            for (auto e : make_iterator_range(out_edges(u, mConstraintGraph))) {
                const ConstraintVertex v = target(e, mConstraintGraph);
                if ((--D[v]) == 0) {
                    S.push_back(v);
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
 * @brief addCandidateSet
 * @param S an independent set
 * @param T the trie in which to encode this new set into
 * @param roots the roots (source nodes) for each tree in T
 ** ------------------------------------------------------------------------------------------------------------- */
inline void AutoMultiplexing::addCandidateSet(const VertexVector & S) {
    if (S.size() >= 3) {
        const auto u = add_vertex(mMultiplexSetGraph);
        for (const auto v : S) {
            add_edge(u, v, mMultiplexSetGraph);
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
                    std::vector<PabloAST *> U(adv->users().begin(), adv->users().end());

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
void AutoMultiplexing::multiplexSelectedIndependentSets() {

    const unsigned f = num_vertices(mConstraintGraph);
    const unsigned l = num_vertices(mMultiplexSetGraph);

    // Preallocate the structures based on the size of the largest multiplex set
    size_t max_n = 3;
    for (unsigned s = f; s != l; ++s) {
        max_n = std::max<unsigned>(max_n, out_degree(s, mMultiplexSetGraph));
    }

    std::vector<MultiplexSetGraph::vertex_descriptor> I(max_n);
    circular_buffer<PabloAST *> Q(max_n);

    // When entering thus function, the multiplex set graph M is a DAG with edges denoting the set
    // relationships of our independent sets.

    for (unsigned s = f; s != l; ++s) {
        const size_t n = out_degree(s, mMultiplexSetGraph);
        if (n) {
            const size_t m = log2_plus_one(n);
            Advance * input[n];
            std::vector<Advance *> muxed(m);

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
                // The only way this did not return an Advance statement would be if either the mux or shift amount
                // is zero. Since these cases would have been eliminated earlier, we are safe to cast here.               
                muxed[j] = cast<Advance>(builder.createAdvance(mux, adv->getOperand(1), prefix.str()));
            }

            /// Perform m-to-n Demultiplexing            
            for (size_t i = 1; i <= n; ++i) {

                // Construct the demuxed values and replaces all the users of the original advances with them.
                for (size_t j = 0; j != m; ++j) {
                    if ((i & (1ULL << j)) == 0) {
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
                    if ((i & (1ULL << j)) != 0) {
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
                input[i - 1]->replaceWith(demuxed, true, true);
            }

            mMuxedVariables.push_back(std::move(muxed));
        }        
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplifyAST
 ** ------------------------------------------------------------------------------------------------------------- */
void AutoMultiplexing::simplifyAST(const PabloFunction & function) {

    // first do a BFS to build a topological ordering of statements we're going to end up visiting
    // and determine which of those will be terminals in the BDD
    using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, PabloAST *>;
    using Vertex = Graph::vertex_descriptor;

    raw_os_ostream out(std::cerr);

    // TODO: this should build a single graph and iterate by connected component instead.
    for (const auto & muxed : mMuxedVariables) {

        Graph G;
        std::unordered_map<PabloAST *, unsigned> M;
        std::queue<Statement *> Q;

        out << "-----------------------------------------------------------------------------------\n";
        PabloPrinter::print(function.getEntryBlock().statements(), out);
        out << "-----------------------------------------------------------------------------------\n"; out.flush();


        for (unsigned i = 0; i != muxed.size(); ++i) {

            const auto u = add_vertex(muxed[i], G);
            M.insert(std::make_pair(muxed[i], u));
            for (PabloAST * user : muxed[i]->users()) {
                auto f = M.find(user);
                Vertex v = 0;
                if (f == M.end()) {
                    v = add_vertex(user, G);
                    M.insert(std::make_pair(user, v));
                    switch (user->getClassTypeId()) {
                        case PabloAST::ClassTypeId::And:
                        case PabloAST::ClassTypeId::Or:
                        case PabloAST::ClassTypeId::Not:
                        case PabloAST::ClassTypeId::Sel:
                            Q.push(cast<Statement>(user));
                        default: break;
                    }
                } else { // if (f != M.end()) {
                    v = f->second;
                }
                add_edge(u, v, G);
            }
        }

        while (!Q.empty()) {
            Statement * const var = Q.front(); Q.pop();


            const Vertex u = M[var];
            for (unsigned i = 0; i != var->getNumOperands(); ++i) {
                PabloAST * operand = var->getOperand(i);
                auto f = M.find(operand);
                Vertex v = 0;
                if (LLVM_UNLIKELY(f == M.end())) {
                    v = add_vertex(operand, G);
                    M.insert(std::make_pair(operand, v));
                } else { // if (f != M.end()) {
                    v = f->second;
                }
                add_edge(v, u, G);
            }

            for (PabloAST * user : var->users()) {
                auto f = M.find(user);
                Vertex v = 0;
                if (LLVM_LIKELY(f == M.end())) {
                    v = add_vertex(user, G);
                    M.insert(std::make_pair(user, v));
                    switch (user->getClassTypeId()) {
                        case PabloAST::ClassTypeId::And:
                        case PabloAST::ClassTypeId::Or:
                        case PabloAST::ClassTypeId::Not:
                        case PabloAST::ClassTypeId::Sel:
                            Q.push(cast<Statement>(user));
                        default: break;
                    }
                } else { // if (f != M.end()) {
                    v = f->second;
                }
                add_edge(u, v, G);
            }
        }

        for (auto u : make_iterator_range(vertices(G))) {
            PabloAST * expr = G[u];
            switch (expr->getClassTypeId()) {
                case PabloAST::ClassTypeId::And:
                case PabloAST::ClassTypeId::Or:
                case PabloAST::ClassTypeId::Not:
                case PabloAST::ClassTypeId::Sel:
                    break;
                default: // this vertex corresponds to a non-Boolean function. It needs to be split.
                    if (in_degree(u, G) > 0 && out_degree(u, G) > 0) {
                        Vertex v = add_vertex(expr, G);
                        for (auto e : make_iterator_range(out_edges(u, G))) {
                            add_edge(v, target(e, G), G);
                        }
                        clear_out_edges(u, G);
                    }
            }
        }

        out << "\ndigraph x {\n";
        for (auto u : make_iterator_range(vertices(G))) {
            std::string tmp;
            raw_string_ostream name(tmp);
            PabloPrinter::print(G[u], name);
            out << "v" << u << " [label=\"" << name.str() << "\"];\n";
        }
        for (auto e : make_iterator_range(edges(G))) {
            out << "v" << source(e, G) << " -> v" << target(e, G) << '\n';
        }

        out << " { rank=same;";
        for (auto u : make_iterator_range(vertices(G))) {
            if (in_degree(u, G) == 0) {
                out << " v" << u;
            }
        }
        out << "}\n";

        out << " { rank=same;";
        for (auto u : make_iterator_range(vertices(G))) {
            if (out_degree(u, G) == 0) {
                out << " v" << u;
            }
        }
        out << "}\n";

        out << "}\n\n";
        out.flush();

        // count the number of sources (sinks) so we know how many variables (terminals) will exist in the BDD
        std::vector<Vertex> inputs;
        flat_set<Vertex> terminals;

        for (auto u : make_iterator_range(vertices(G))) {
            if (in_degree(u, G) == 0) {
                inputs.push_back(u);
            }
            if (out_degree(u, G) == 0) {
                // the inputs to the sinks become the terminals in the BDD
                for (auto e : make_iterator_range(in_edges(u, G))) {
                    terminals.insert(source(e, G));
                }
            }
        }




        const auto n = inputs.size();

        mManager = Cudd_Init(n, 0, CUDD_UNIQUE_SLOTS, CUDD_CACHE_SLOTS, 0);
        Cudd_AutodynEnable(mManager, CUDD_REORDER_LAZY_SIFT);

        std::vector<PabloAST *> nodes(n);
        std::vector<DdNode *> characterization(num_vertices(G), nullptr);
        for (unsigned i = 0; i != n; ++i) {
            nodes[i] = G[inputs[i]];
            assert (nodes[i]);
            characterization[inputs[i]] = Cudd_bddIthVar(mManager, i);
        }

        std::vector<Vertex> ordering;
        ordering.reserve(num_vertices(G));
        topological_sort(G, std::back_inserter(ordering));

        for (auto ui = ordering.rbegin(); ui != ordering.rend(); ++ui) {

            const Vertex u = *ui;

            if (characterization[u]) {
                continue;
            }

            std::array<DdNode *, 3> input;

            unsigned i = 0;
            for (const auto e : make_iterator_range(in_edges(u, G))) {                
                input[i] = characterization[source(e, G)];
                if (input[i] == nullptr) {
                    std::string tmp;
                    raw_string_ostream out(tmp);
                    out << "Uncharacterized input! ";
                    PabloPrinter::print(G[source(e, G)], out);
                    throw std::runtime_error(out.str());
                }
                ++i;
            }

            DdNode * bdd = nullptr;
            bool characterized = true;
            switch (G[u]->getClassTypeId()) {
                case PabloAST::ClassTypeId::And:
                    bdd = And(input[0], input[1]);
                    break;
                case PabloAST::ClassTypeId::Or:
                    bdd = Or(input[0], input[1]);
                    break;
                case PabloAST::ClassTypeId::Not:
                    bdd = Not(input[0]);
                    break;
                case PabloAST::ClassTypeId::Sel:
                    bdd = Ite(input[0], input[1], input[2]);
                    break;
                default: characterized = false; break;
            }

            if (characterized) {
                Ref(bdd);
                characterization[u] = bdd;
            }
        }

        assert (Cudd_DebugCheck(mManager) == 0);
        Cudd_ReduceHeap(mManager, CUDD_REORDER_SIFT, 0);

        assert (mManager->size == nodes.size());

        for (Vertex t : terminals) {

            PabloPrinter::print(cast<Statement>(G[t]), " >> ", out);
            out << '\n';
            out.flush();

            Statement * stmt = cast<Statement>(G[t]);

            PabloBuilder builder(*(stmt->getParent()));

            DdNode * const f = characterization[t];
            Cudd_Ref(f);

            PabloAST * expr = simplifyAST(f, nodes, builder);
            if (expr) {

                PabloPrinter::print(cast<Statement>(expr), "    replacement: ", out);
                out << '\n';
                out.flush();

                stmt->replaceWith(expr, false, true);
            }
            Cudd_RecursiveDeref(mManager, f);
        }

        Cudd_Quit(mManager);

    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplifyAST
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * AutoMultiplexing::simplifyAST(DdNode * const f, const std::vector<PabloAST *> & variables, PabloBuilder & builder) {
    assert (!NoSatisfyingAssignment(f));
    DdNode * g = Cudd_FindEssential(mManager, f);
    if (g && Cudd_SupportSize(mManager, g) > 0) {
        if (g == f) { // every variable is essential
            return makeCoverAST(f, variables, builder);
        }
        Cudd_Ref(g);
        PabloAST * c0 = makeCoverAST(g, variables, builder);
        if (LLVM_UNLIKELY(c0 == nullptr)) {
            Cudd_RecursiveDeref(mManager, g);
            return nullptr;
        }
        DdNode * h = Cudd_Cofactor(mManager, f, g);
        Cudd_Ref(h);
        PabloAST * c1 = simplifyAST(h, variables, builder);
        if (LLVM_UNLIKELY(c1 == nullptr)) {
            Cudd_RecursiveDeref(mManager, g);
            Cudd_RecursiveDeref(mManager, h);
            cast<Statement>(c0)->eraseFromParent(true);
            return nullptr;
        }
        assert (And(g, h) == f);
        Cudd_RecursiveDeref(mManager, g);
        Cudd_RecursiveDeref(mManager, h);
        return builder.createAnd(c0, c1, "escf");
    }

    DdNode ** disjunct = nullptr;
    int disjuncts = Cudd_bddIterDisjDecomp(mManager, f, &disjunct);
    assert (disjuncts < 2 || Or(disjunct[0], disjunct[1]) == f);

    DdNode ** conjunct = nullptr;
    int conjuncts = Cudd_bddIterConjDecomp(mManager, f, &conjunct);
    assert (conjuncts < 2 || And(conjunct[0], conjunct[1]) == f);

    if (LLVM_LIKELY(disjuncts == 2 && conjuncts == 2)) {
        if (Cudd_SharingSize(disjunct, 2) > Cudd_SharingSize(conjunct, 2)) {
            disjuncts = 0;
        }
    }

    DdNode * decomp[] = { nullptr, nullptr };
    if (disjuncts == 2) {
        memcpy(decomp, disjunct, sizeof(DdNode *) * 2);
    } else if (conjuncts == 2) {
        memcpy(decomp, conjunct, sizeof(DdNode *) * 2);
    }

    FREE(disjunct);
    FREE(conjunct);

    if ((decomp[0] != decomp[1]) && (decomp[0] != f) && (decomp[1] != f)) {
        Cudd_Ref(decomp[0]);
        Cudd_Ref(decomp[1]);
        PabloAST * d0 = simplifyAST(decomp[0], variables, builder);
        Cudd_RecursiveDeref(mManager, decomp[0]);
        if (LLVM_UNLIKELY(d0 == nullptr)) {
            Cudd_RecursiveDeref(mManager, decomp[1]);
            return nullptr;
        }

        PabloAST * d1 = simplifyAST(decomp[1], variables, builder);
        Cudd_RecursiveDeref(mManager, decomp[1]);
        if (LLVM_UNLIKELY(d1 == nullptr)) {
            cast<Statement>(d0)->eraseFromParent(true);
            return nullptr;
        }

        if (disjuncts == 2) {
            return builder.createOr(d0, d1, "disj");
        } else {
            return builder.createAnd(d0, d1, "conj");
        }
    }
    return makeCoverAST(f, variables, builder);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeCoverAST
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * AutoMultiplexing::makeCoverAST(DdNode * const f, const std::vector<PabloAST *> & variables, PabloBuilder & builder) {

    std::queue<PabloAST *> SQ;
    const unsigned n = variables.size();
    circular_buffer<PabloAST *> CQ(n);
    circular_buffer<PabloAST *> DQ(n);

    assert (mManager->size == variables.size());

    int cube[n];

    DdNode * g = f;

    Cudd_Ref(g);

    while (g != Cudd_ReadLogicZero(mManager)) {
        int length = 0;
        DdNode * implicant = Cudd_LargestCube(mManager, g, &length);
        if (LLVM_UNLIKELY(implicant == nullptr)) {
            Cudd_RecursiveDeref(mManager, g);
            return nullptr;
        }
        Cudd_Ref(implicant);
        DdNode * prime = Cudd_bddMakePrime(mManager, implicant, f);
        if (LLVM_UNLIKELY(prime == nullptr)) {
            Cudd_RecursiveDeref(mManager, g);
            Cudd_RecursiveDeref(mManager, implicant);
            return nullptr;
        }
        Cudd_Ref(prime);
        Cudd_RecursiveDeref(mManager, implicant);

        DdNode * h = Cudd_bddAnd(mManager, g, Cudd_Not(prime));
        if (LLVM_UNLIKELY(h == nullptr)) {
            Cudd_RecursiveDeref(mManager, g);
            Cudd_RecursiveDeref(mManager, prime);
            return nullptr;
        }
        Cudd_Ref(h);
        Cudd_RecursiveDeref(mManager, g);

        g = h;
        if (LLVM_UNLIKELY(Cudd_BddToCubeArray(mManager, prime, cube) == 0)) {
            Cudd_RecursiveDeref(mManager, g);
            Cudd_RecursiveDeref(mManager, prime);
            return nullptr;
        }
        Cudd_RecursiveDeref(mManager, prime);

        assert (DQ.empty() && CQ.empty());

        for (auto i = 0; i != n; ++i) {
            assert (cube[i] >= 0 && cube[i] <= 2);
            if (cube[i] == 0) {
                assert (!DQ.full());
                DQ.push_back(variables[i]);
            } else if (cube[i] == 1) {
                assert (!CQ.full());
                CQ.push_back(variables[i]);
            }
        }

        if (LLVM_UNLIKELY(DQ.empty() && CQ.empty())) {
            throw std::runtime_error("Error! statement contains no elements!");
        }

        if (DQ.size() > 0) {
            while (DQ.size() > 1) {
                PabloAST * v1 = DQ.front(); DQ.pop_front();
                PabloAST * v2 = DQ.front(); DQ.pop_front();
                DQ.push_back(builder.createOr(v1, v2));
            }
            CQ.push_back(builder.createNot(DQ.front()));
            DQ.pop_front();
        }

        assert (!CQ.empty());
        while (CQ.size() > 1) {
            PabloAST * v1 = CQ.front(); CQ.pop_front();
            PabloAST * v2 = CQ.front(); CQ.pop_front();
            CQ.push_back(builder.createAnd(v1, v2));
        }
        SQ.push(CQ.front()); CQ.pop_front();
    }
    Cudd_RecursiveDeref(mManager, g);
    if (LLVM_UNLIKELY(SQ.empty())) {
        throw std::runtime_error("Error! statement queue empty!");
    }
    while (SQ.size() > 1) {
        PabloAST * v1 = SQ.front(); SQ.pop();
        PabloAST * v2 = SQ.front(); SQ.pop();
        SQ.push(builder.createOr(v1, v2));
    }
    return SQ.front();
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
    // Note: not a real topological sort. I expect the original graph to be close to the resulting one. Thus cost
    // of constructing a graph in which to perform the sort takes longer than potentially moving an instruction
    // multiple times.
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

