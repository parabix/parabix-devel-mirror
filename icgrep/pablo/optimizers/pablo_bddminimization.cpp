#include "pablo_bddminimization.h"
#include <pablo/codegenstate.h>
#include <pablo/builder.hpp>
#include <stack>
#include <iostream>
#include <pablo/printer_pablos.h>
#include <cudd.h>
#include <util.h>
#include <mtr.h>

using namespace llvm;

namespace pablo {

bool BDDMinimizationPass::optimize(PabloFunction & function) {
    BDDMinimizationPass am;
    am.initialize(function);
    am.characterize(function.getEntryBlock());
    am.eliminateLogicallyEquivalentStatements(function.getEntryBlock());
    am.simplifyAST(function);
    am.shutdown();
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initialize
 * @param vars the input vars for this program
 * @param entry the entry block
 *
 * Scan through the program to identify any advances and calls then initialize the BDD engine with
 * the proper variable ordering.
 ** ------------------------------------------------------------------------------------------------------------- */
void BDDMinimizationPass::initialize(const PabloFunction & function) {

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
                case PabloAST::ClassTypeId::MatchStar:
                case PabloAST::ClassTypeId::ScanThru:                
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
    unsigned maxVariableCount = variableCount + function.getNumOfParameters();
    mManager = Cudd_Init(maxVariableCount, 0, CUDD_UNIQUE_SLOTS, CUDD_CACHE_SLOTS, 0);
    Cudd_MakeTreeNode(mManager, variableCount, function.getNumOfParameters(), MTR_DEFAULT);
    Cudd_AutodynEnable(mManager, CUDD_REORDER_LAZY_SIFT);
    // Map the predefined 0/1 entries
    mCharacterizationMap[function.getEntryBlock().createZeroes()] = Zero();
    mCharacterizationMap[function.getEntryBlock().createOnes()] = One();

    mVariables = 0;

    mStatementVector.resize(maxVariableCount, nullptr);
    // Order the variables so the input Vars are pushed to the end; they ought to
    // be the most complex to resolve.    
    for (auto i = 0; i != function.getNumOfParameters(); ++i) {
        mStatementVector[variableCount] = const_cast<Var *>(function.getParameter(i));
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
    for (const Statement * stmt : block) {
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            // Set the next statement to be the first statement of the inner scope and push the
            // next statement of the current statement into the scope stack.
            characterize(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());            
            continue;
        }
        mCharacterizationMap.insert(std::make_pair(stmt, characterize(stmt)));
    }
    Cudd_ReduceHeap(mManager, CUDD_REORDER_SIFT, 0);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
inline DdNode * BDDMinimizationPass::characterize(const Statement * const stmt) {

//    llvm::raw_os_ostream out(std::cerr);
//    PabloPrinter::print(stmt, "> ", out); out << "\n"; out.flush();

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
            throw std::runtime_error("Error in AST: attempted to characterize statement with unknown operand!");
        }
        input[i] = f->second;
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
        case PabloAST::ClassTypeId::Advance:
        case PabloAST::ClassTypeId::Call:
        case PabloAST::ClassTypeId::MatchStar:
        case PabloAST::ClassTypeId::ScanThru:
            return NewVar(const_cast<Statement *>(stmt));
        default:
            throw std::runtime_error("Unexpected statement type " + stmt->getName()->to_string());
    }

    Cudd_Ref(bdd);

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
    Cudd_AutodynDisable(mManager);
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
            eliminateLogicallyEquivalentStatements(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody(), subsitutionMap);
        } else if (LLVM_LIKELY(!isa<Assign>(stmt) && !isa<Next>(stmt))) {
            DdNode * bdd = mCharacterizationMap[stmt];
            PabloAST * replacement = subsitutionMap[bdd];
            if (replacement) {
                Cudd_RecursiveDeref(mManager, bdd);
                stmt = stmt->replaceWith(replacement, false, true);
                continue;
            }
            subsitutionMap.insert(bdd, stmt);
        }
        stmt = stmt->getNextNode();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplifyAST
 *
 * Assignment and Next statements are unique in the AST in that they are the only way a value can escape its
 * local block. By trying to simplify those aggressively
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BDDMinimizationPass::simplifyAST(PabloFunction & function) {
    Cudd_ReduceHeap(mManager, CUDD_REORDER_EXACT, 0);
    Cudd_zddVarsFromBddVars(mManager, 1);
    simplifyAST(function.getEntryBlock());
    Cudd_PrintInfo(mManager, stderr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplifyAST
 ** ------------------------------------------------------------------------------------------------------------- */
void BDDMinimizationPass::simplifyAST(PabloBlock & block) {
    for (Statement * stmt : block) {
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            simplifyAST(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
        } else if (isa<Advance>(stmt) || isa<Assign>(stmt) || isa<Next>(stmt)) {
            simplifyAST(block, stmt->getOperand(0));
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplifyAST
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BDDMinimizationPass::simplifyAST(PabloBlock & block, PabloAST * const stmt) {

    DdNode * const bdd = mCharacterizationMap[stmt];

    llvm::raw_os_ostream out(std::cerr);
    out << " >>>>>>>> "; PabloPrinter::print(stmt, out); out << " <<<<<<<<"; out.flush();

//    unsigned count = 0;
//    // TODO: look into 0/1/x dominators?
//    CUDD_VALUE_TYPE value;
//    int * cube = nullptr;
//    DdGen * gen = Cudd_FirstCube(mManager, bdd, &cube, &value);
//    while (!Cudd_IsGenEmpty(gen)) {
//        ++count;
//        // cube[0 ... n - 1] = { 0 : false, 1: true, 2: don't care }
//        Cudd_NextCube(gen, &cube, &value);
//    }
//    Cudd_GenFree(gen);

//    out << count;

    out << "\n"; out.flush();




//    block.setInsertPoint(stmt->getPrevNode());

//    DdNode * zdd = Cudd_zddPortFromBdd(mManager, bdd);
//    Cudd_zddPrintCover(mManager, zdd);
//    Cudd_RecursiveDerefZdd(mManager, zdd);
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

inline DdNode * BDDMinimizationPass::NewVar(Statement * const stmt) {
    mStatementVector[mVariables] = stmt;
    return Cudd_bddIthVar(mManager, mVariables++);
}

inline bool BDDMinimizationPass::isZero(DdNode * const x) const {
    return x == Zero();
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
}

} // end of namespace pablo

