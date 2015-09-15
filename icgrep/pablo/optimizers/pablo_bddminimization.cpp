#include "pablo_bddminimization.h"
#include <pablo/codegenstate.h>
#include <pablo/builder.hpp>
#include <stack>
#include <iostream>
#include <pablo/printer_pablos.h>
#include <cudd.h>
#include <cuddInt.h>
#include <util.h>
#include <queue>
#include <boost/circular_buffer.hpp>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>

using namespace llvm;
using namespace boost;
using namespace boost::container;

namespace pablo {

using TypeId = PabloAST::ClassTypeId;

bool BDDMinimizationPass::optimize(PabloFunction & function) {
    BDDMinimizationPass am;
    am.eliminateLogicallyEquivalentStatements(function);

    am.shutdown();
    return Simplifier::optimize(function);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initialize
 * @param vars the input vars for this program
 * @param entry the entry block
 *
 * Scan through the program to identify any advances and calls then initialize the BDD engine with
 * the proper variable ordering.
 ** ------------------------------------------------------------------------------------------------------------- */
void BDDMinimizationPass::eliminateLogicallyEquivalentStatements(PabloFunction & function) {

    std::stack<Statement *> scope;
    unsigned variableCount = 0; // number of statements that cannot always be categorized without generating a new variable

    for (const Statement * stmt = function.getEntryBlock().front(); ; ) {
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
                case TypeId::Assign:
                case TypeId::Next:
                case TypeId::Advance:
                case TypeId::Call:
                case TypeId::MatchStar:
                case TypeId::ScanThru:
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
    mManager = Cudd_Init(variableCount + function.getNumOfParameters() - function.getNumOfResults(), 0, CUDD_UNIQUE_SLOTS, CUDD_CACHE_SLOTS, 0);
    mVariables = 0;
    Cudd_MakeTreeNode(mManager, 0, function.getNumOfParameters(), MTR_DEFAULT);
    // Map the predefined 0/1 entries
    mCharacterizationMap[function.getEntryBlock().createZeroes()] = Zero();
    mCharacterizationMap[function.getEntryBlock().createOnes()] = One();    
    // Order the variables so the input Vars are pushed to the end; they ought to
    // be the most complex to resolve.    
    for (auto i = 0; i != function.getNumOfParameters(); ++i) {
        mCharacterizationMap[function.getParameter(i)] = NewVar(function.getParameter(i));
    }

    SubsitutionMap baseMap;
    baseMap.insert(Zero(), function.getEntryBlock().createZeroes());
    baseMap.insert(One(), function.getEntryBlock().createOnes());

    Cudd_SetSiftMaxVar(mManager, 1000000);
    Cudd_SetSiftMaxSwap(mManager, 1000000000);
    Cudd_AutodynEnable(mManager, CUDD_REORDER_LAZY_SIFT);

    eliminateLogicallyEquivalentStatements(function.getEntryBlock(), baseMap);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 * @param vars the input vars for this program
 *
 * Scan through the program and iteratively compute the BDDs for each statement.
 ** ------------------------------------------------------------------------------------------------------------- */
void BDDMinimizationPass::eliminateLogicallyEquivalentStatements(PabloBlock & block, SubsitutionMap & parent) {
    SubsitutionMap map(&parent);
    Statement * stmt = block.front();
    while (stmt) {
        if (LLVM_UNLIKELY(isa<If>(stmt))) {
            eliminateLogicallyEquivalentStatements(cast<If>(stmt)->getBody(), map);
            for (Assign * def : cast<const If>(stmt)->getDefined()) {
                map.insert(mCharacterizationMap[def], def);
            }
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            eliminateLogicallyEquivalentStatements(cast<While>(stmt)->getBody(), map);
            for (Next * var : cast<const While>(stmt)->getVariants()) {
                map.insert(mCharacterizationMap[var], var);
            }
        } else { // attempt to characterize this statement and replace it if we've encountered an equivalent one
            DdNode * bdd = nullptr;
            bool test = false;
            std::tie(bdd, test) = characterize(stmt);
            if (test) {
                PabloAST * replacement = map.get(bdd);
                if (LLVM_UNLIKELY(replacement != nullptr)) {
                    Cudd_RecursiveDeref(mManager, bdd);
                    stmt = stmt->replaceWith(replacement, false, true);
                    continue;
                }
            } else if (LLVM_LIKELY(nonConstant(bdd))) {
                map.insert(bdd, stmt);
            }
            mCharacterizationMap.insert(std::make_pair(stmt, bdd));
        }
        stmt = stmt->getNextNode();
    }   
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
inline std::pair<DdNode *, bool> BDDMinimizationPass::characterize(Statement * const stmt) {

    DdNode * bdd = nullptr;
    // Map our operands to the computed BDDs
    std::array<DdNode *, 3> input;
    for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
        PabloAST * const op = stmt->getOperand(i);
        if (op == nullptr) {
            throw std::runtime_error("Statement has Null operand!");
        }
        if (LLVM_UNLIKELY(isa<Integer>(op) || isa<String>(op))) {
            continue;
        }
        auto f = mCharacterizationMap.find(op);
        if (LLVM_UNLIKELY(f == mCharacterizationMap.end())) {
            std::string tmp;
            llvm::raw_string_ostream msg(tmp);
            msg << "BDDMinimizationPass: uncharacterized operand " << std::to_string(i);
            PabloPrinter::print(stmt, " of ", msg);
            throw std::runtime_error(msg.str());
        }
        input[i] = f->second;
    }

    switch (stmt->getClassTypeId()) {
        case TypeId::Assign:
        case TypeId::Next:
            return std::make_pair(input[0], false);
        case TypeId::And:
            bdd = And(input[0], input[1]);
            break;
        case TypeId::Or:
            bdd = Or(input[0], input[1]);
            break;
        case TypeId::Xor:
            bdd = Xor(input[0], input[1]);
            break;
        case TypeId::Not:
            bdd = Not(input[0]);
            break;
        case TypeId::Sel:
            bdd = Ite(input[0], input[1], input[2]);
            break;
        case TypeId::MatchStar:
        case TypeId::ScanThru:
            if (LLVM_UNLIKELY(input[1] == Zero())) {
                return std::make_pair(Zero(), true);
            }
        case TypeId::Advance:
            if (LLVM_UNLIKELY(input[0] == Zero())) {
                return std::make_pair(Zero(), true);
            }
        case TypeId::Call:
            // TODO: we may have more than one output. Need to fix call class to allow for it.
            return std::make_pair(NewVar(stmt), false);
        default:
            throw std::runtime_error("Unexpected statement type " + stmt->getName()->to_string());
    }
    Cudd_Ref(bdd);
    if (LLVM_UNLIKELY(noSatisfyingAssignment(bdd))) {
        Cudd_RecursiveDeref(mManager, bdd);
        bdd = Zero();
    }
    return std::make_pair(bdd, true);
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

inline DdNode * BDDMinimizationPass::NewVar(const PabloAST *) {
    return Cudd_bddIthVar(mManager, mVariables++);
}

inline bool BDDMinimizationPass::nonConstant(DdNode * const x) const {
    return x != Zero() && x != One();
}

inline DdNode * BDDMinimizationPass::And(DdNode * const x, DdNode * const y) {
    DdNode * r = Cudd_bddAnd(mManager, x, y);
    return r;
}

inline DdNode * BDDMinimizationPass::Intersect(DdNode * const x, DdNode * const y) {
    DdNode * r = Cudd_bddIntersect(mManager, x, y); Cudd_Ref(r);
    return r;
}

inline DdNode * BDDMinimizationPass::Or(DdNode * const x, DdNode * const y) {
    DdNode * r = Cudd_bddOr(mManager, x, y);
    return r;
}

inline DdNode * BDDMinimizationPass::Xor(DdNode * const x, DdNode * const y) {
    DdNode * r = Cudd_bddXor(mManager, x, y);
    return r;
}

inline DdNode * BDDMinimizationPass::Not(DdNode * const x) const {
    return Cudd_Not(x);
}

inline DdNode * BDDMinimizationPass::Ite(DdNode * const x, DdNode * const y, DdNode * const z) {
    DdNode * r = Cudd_bddIte(mManager, x, y, z);
    return r;
}

inline bool BDDMinimizationPass::noSatisfyingAssignment(DdNode * const x) {
    return Cudd_bddLeq(mManager, x, Zero());
}

inline void BDDMinimizationPass::shutdown() {
    Cudd_Quit(mManager);
    mCharacterizationMap.clear();
}

} // end of namespace pablo

