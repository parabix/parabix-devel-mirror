#include "pablo_bddminimization.h"
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

#if !defined(NDEBUG) || defined(PRINT_DEBUG_OUTPUT)
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


#else
#define LOG(x)
#define RECORD_TIMESTAMP(Name)
#define LOG_GRAPH(Name, G)
#define LOG_NUMBER_OF_ADVANCES(entry)
#endif


namespace pablo {

bool BDDMinimizationPass::optimize(PabloFunction & function) {

    BDDMinimizationPass am;
    RECORD_TIMESTAMP(start_initialize);
    am.initialize(function);
    RECORD_TIMESTAMP(end_initialize);

    LOG("Initialize:              " << (end_initialize - start_initialize));

    RECORD_TIMESTAMP(start_characterize);
    am.characterize(entry);
    RECORD_TIMESTAMP(end_characterize);

    LOG("Characterize:            " << (end_characterize - start_characterize));

    RECORD_TIMESTAMP(start_minimization);
    am.eliminateLogicallyEquivalentStatements(entry);
    RECORD_TIMESTAMP(end_minimization);
    LOG("Minimize:                " << (end_minimization - start_minimization));

    RECORD_TIMESTAMP(start_minimization);
    am.simplify(entry);
    RECORD_TIMESTAMP(end_minimization);
    LOG("Minimize:                " << (end_minimization - start_minimization));

    RECORD_TIMESTAMP(start_shutdown);
    am.shutdown();
    RECORD_TIMESTAMP(end_shutdown);
    LOG("Shutdown:                " << (end_shutdown - start_shutdown));



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
void BDDMinimizationPass::initialize(PabloFunction &function) {

    std::stack<Statement *> scope;
    unsigned variableCount = 0; // number of statements that cannot always be categorized without generating a new variable

    for (Statement * stmt = function.getEntryBlock().front(); ; ) {
        while ( stmt ) {

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
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 * @param vars the input vars for this program
 *
 * Scan through the program and iteratively compute the BDDs for each statement.
 ** ------------------------------------------------------------------------------------------------------------- */
void BDDMinimizationPass::characterize(const PabloBlock & block) {
    for (Statement * stmt : block) {
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            // Set the next statement to be the first statement of the inner scope and push the
            // next statement of the current statement into the scope stack.
            characterize(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
            continue;
        }
        mCharacterizationMap[stmt] =  characterize(stmt);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
inline DdNode * BDDMinimizationPass::characterize(const Statement * const stmt) {

    DdNode * bdd = nullptr;
    // Map our operands to the computed BDDs
    std::array<DdNode *, 3> input;
    for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
        PabloAST * const op = stmt->getOperand(i);
        if (LLVM_UNLIKELY(isa<Integer>(op) || isa<String>(op))) {
            continue;
        }
        input[i] = mCharacterizationMap[op];
    }

    switch (stmt->getClassTypeId()) {
        case PabloAST::ClassTypeId::Assign:
            return input[0];
        case PabloAST::ClassTypeId::And:
            bdd = And(input[0], input[1]);
            break;
        case PabloAST::ClassTypeId::Next:
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
        case PabloAST::ClassTypeId::MatchStar:
        case PabloAST::ClassTypeId::Call:
        case PabloAST::ClassTypeId::Advance:
            return NewVar();
        default:
            throw std::runtime_error("Unexpected statement type " + stmt->getName()->to_string());
    }

    if (LLVM_UNLIKELY(noSatisfyingAssignment(bdd))) {
        Cudd_RecursiveDeref(mManager, bdd);
        bdd = Zero();
    }

    return bdd;

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief eliminateLogicallyEquivalentStatements
 * @param entry the entry block of the program
 *
 * Scan through the program using the precomputed BDDs and replace any statements with equivalent BDDs with the
 * earlier one (assuming its in scope) and replace any contradictions with Zero.
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BDDMinimizationPass::eliminateLogicallyEquivalentStatements(PabloBlock & entry) {
    SubsitutionMap baseMap;
    baseMap.insert(Zero(), entry.createZeroes());
    baseMap.insert(One(), entry.createOnes());
    eliminateLogicallyEquivalentStatements(entry, baseMap);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief eliminateLogicallyEquivalentStatements
 ** ------------------------------------------------------------------------------------------------------------- */
void BDDMinimizationPass::eliminateLogicallyEquivalentStatements(PabloBlock & block, SubsitutionMap & parent) {
    SubsitutionMap subsitutionMap(&parent);
    Statement * stmt = block.front();
    while (stmt) {
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            // Set the next statement to be the first statement of the inner scope and push the
            // next statement of the current statement into the scope stack.
            eliminateLogicallyEquivalentStatements(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody(), subsitutionMap);
            continue;
        }

        if (LLVM_UNLIKELY(isa<Assign>(stmt) || isa<Next>(stmt))) {
            continue;
        }

        DdNode * bdd = mCharacterizationMap[stmt];
        PabloAST * replacement = subsitutionMap[bdd];
        if (replacement) {
            Cudd_RecursiveDeref(mManager, bdd);
            stmt = stmt->replaceWith(replacement, false, true);
        }
        else {
            stmt = stmt->getNextNode();
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplify
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * BDDMinimizationPass::simplify(DdNode * bdd) const {

    CUDD_VALUE_TYPE value;
    int * cube = nullptr;

    DdGen * gen = Cudd_FirstCube(mManager, bdd, &cube, &value);
    while (!Cudd_IsGenEmpty(gen)) {
        // cube[0 ... n - 1] = { 0 : false, 1: true, 2: don't care }




        Cudd_NextCube(gen, &cube, &value);
    }
    Cudd_GenFree(gen);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CUDD wrappers
 ** ------------------------------------------------------------------------------------------------------------- */

inline DdNode * BDDMinimizationPass::Zero() const {
    return Cudd_ReadLogicZero(mManager);
}

inline DdNode * BDDMinimizationPass::One() const {
    return Cudd_ReadOne(mManager);
}

inline DdNode * BDDMinimizationPass::NewVar() {
    return Cudd_bddIthVar(mManager, mVariables++);
}

inline bool BDDMinimizationPass::isZero(DdNode * const x) const {
    return x == Zero();
}

inline DdNode * BDDMinimizationPass::And(DdNode * const x, DdNode * const y) {
    DdNode * r = Cudd_bddAnd(mManager, x, y);
    Cudd_Ref(r);
    return r;
}

inline DdNode * BDDMinimizationPass::Intersect(DdNode * const x, DdNode * const y) {
    DdNode * r = Cudd_bddIntersect(mManager, x, y); Cudd_Ref(r);
    return r;
}

inline DdNode * BDDMinimizationPass::Or(DdNode * const x, DdNode * const y) {
    DdNode * r = Cudd_bddOr(mManager, x, y);
    Cudd_Ref(r);
    return r;
}

inline DdNode * BDDMinimizationPass::Xor(DdNode * const x, DdNode * const y) {
    DdNode * r = Cudd_bddXor(mManager, x, y);
    Cudd_Ref(r);
    return r;
}

inline DdNode * BDDMinimizationPass::Not(DdNode * const x) const {
    return Cudd_Not(x);
}

inline DdNode * BDDMinimizationPass::Ite(DdNode * const x, DdNode * const y, DdNode * const z) {
    DdNode * r = Cudd_bddIte(mManager, x, y, z);
    Cudd_Ref(r);
    return r;
}

inline bool BDDMinimizationPass::noSatisfyingAssignment(DdNode * const x) {
    return Cudd_bddLeq(mManager, x, Zero());
}

inline void BDDMinimizationPass::shutdown() {
    Cudd_Quit(mManager);
}



} // end of namespace pablo

