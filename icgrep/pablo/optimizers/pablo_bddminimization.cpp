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
    am.characterizeAndEliminateLogicallyEquivalentStatements(function);
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

    for (const Statement * stmt = function.getEntryBlock().front(); ; ) {
        while ( stmt ) {

            if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                // Set the next statement to be the first statement of the inner scope and push the
                // next statement of the current statement into the scope stack.
                const PabloBlock & nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                variableCount += isa<If>(stmt) ? cast<If>(stmt)->getDefined().size() : cast<While>(stmt)->getVariants().size();
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
    Cudd_MakeTreeNode(mManager, 0, function.getNumOfParameters(), MTR_DEFAULT);
    // Map the predefined 0/1 entries
    mCharacterizationMap[function.getEntryBlock().createZeroes()] = Zero();
    mCharacterizationMap[function.getEntryBlock().createOnes()] = One();    
    // Order the variables so the input Vars are pushed to the end; they ought to
    // be the most complex to resolve.    
    mVariables = 0;
    for (auto i = 0; i != function.getNumOfParameters(); ++i) {
        mCharacterizationMap[function.getParameter(i)] = NewVar();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 * @param vars the input vars for this program
 *
 * Scan through the program and iteratively compute the BDDs for each statement.
 ** ------------------------------------------------------------------------------------------------------------- */
void BDDMinimizationPass::characterizeAndEliminateLogicallyEquivalentStatements(PabloFunction & function) {
    SubsitutionMap baseMap;
    baseMap.insert(Zero(), function.getEntryBlock().createZeroes());
    baseMap.insert(One(), function.getEntryBlock().createOnes());
    Cudd_AutodynEnable(mManager, CUDD_REORDER_LAZY_SIFT);
    characterizeAndEliminateLogicallyEquivalentStatements(function.getEntryBlock(), baseMap);
    Cudd_AutodynDisable(mManager);
    Cudd_ReduceHeap(mManager, CUDD_REORDER_EXACT, 0);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 * @param vars the input vars for this program
 *
 * Scan through the program and iteratively compute the BDDs for each statement.
 ** ------------------------------------------------------------------------------------------------------------- */
void BDDMinimizationPass::characterizeAndEliminateLogicallyEquivalentStatements(PabloBlock & block, SubsitutionMap & parent) {
    SubsitutionMap subsitutionMap(&parent);
    Statement * stmt = block.front();
    while (stmt) {
        if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
            characterizeAndEliminateLogicallyEquivalentStatements(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody(), subsitutionMap);
            // The defined vars of If statements and variants of While statements are unique in the AST in that
            // they are the only way a value can escape its local block. When we simplify this code later, we
            // do not want to recompute whatever was found within a nested block so instead they are marked as
            // new variables.
            if (isa<If>(stmt)) {
                for (const Assign * defVar : cast<const If>(stmt)->getDefined()) {
                    mCharacterizationMap[defVar] = NewVar();
                }
            } else { // if (isa<While>(stmt)) {
                for (const Next * variant : cast<const While>(stmt)->getVariants()) {
                    mCharacterizationMap[variant] = NewVar();
                }
            }
        } else { // attempt to characterize this statement and replace it if
            DdNode * bdd = characterizeAndEliminateLogicallyEquivalentStatements(stmt);
            if (LLVM_LIKELY(!isa<Assign>(stmt) && !isa<Next>(stmt))) {
                PabloAST * replacement = subsitutionMap[bdd];
                if (replacement) {
                    Cudd_RecursiveDeref(mManager, bdd);
                    stmt = stmt->replaceWith(replacement, false, true);
                    continue;
                }
                subsitutionMap.insert(bdd, stmt);
            }
            mCharacterizationMap.insert(std::make_pair(stmt, bdd));
        }
        stmt = stmt->getNextNode();
    }
    Cudd_ReduceHeap(mManager, CUDD_REORDER_SIFT, 0);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
inline DdNode * BDDMinimizationPass::characterizeAndEliminateLogicallyEquivalentStatements(const Statement * const stmt) {

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
        case PabloAST::ClassTypeId::Next:
            return input[0];
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
        case PabloAST::ClassTypeId::Call:
            // TODO: we may have more than one output. Need to fix call class to allow for it.
        case PabloAST::ClassTypeId::Advance:
        case PabloAST::ClassTypeId::MatchStar:
        case PabloAST::ClassTypeId::ScanThru:
            return NewVar();
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
 * @brief simplifyAST
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BDDMinimizationPass::simplifyAST(PabloFunction & function) {
    simplifyAST(function.getEntryBlock());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplifyAST
 ** ------------------------------------------------------------------------------------------------------------- */
void BDDMinimizationPass::simplifyAST(PabloBlock & block) {
    for (Statement * stmt : block) {
        switch (stmt->getClassTypeId()) {
            case PabloAST::ClassTypeId::If:
            case PabloAST::ClassTypeId::While:
                simplifyAST(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody());
                break;
            case PabloAST::ClassTypeId::ScanThru:
            case PabloAST::ClassTypeId::MatchStar:
                simplifyAST(stmt->getOperand(1), block, stmt->getPrevNode());
            case PabloAST::ClassTypeId::Advance:
            case PabloAST::ClassTypeId::Assign:
            case PabloAST::ClassTypeId::Next:
                simplifyAST(stmt->getOperand(0), block, stmt->getPrevNode());
            default: break;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplifyAST
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BDDMinimizationPass::simplifyAST(PabloAST * const expr, PabloBlock & block, Statement * const insertionPoint) {

    DdNode * bdd = mCharacterizationMap[expr];

    llvm::raw_os_ostream out(std::cerr);
    PabloPrinter::print(expr, out); out << " : " << Cudd_SupportSize(mManager, bdd) << '\n';
    out.flush();

    Cudd_bddPrintCover(mManager, bdd, bdd);

//    // TODO: look into 0/1/x dominators?
//    CUDD_VALUE_TYPE value;
//    int * cube = nullptr;

//    char output[3] = { '0', '1', '.' };

//    DdGen * gen = Cudd_FirstCube(mManager, bdd, &cube, &value);
//    while (!Cudd_IsGenEmpty(gen)) {
//        // cube[0 ... n - 1] = { 0 : false, 1: true, 2: don't care }
//        for (unsigned i = 0; i != mVariables; ++i) {
//            out << output[cube[i]];
//        }
//        out << '\n';
//        Cudd_NextCube(gen, &cube, &value);
//    }
//    Cudd_GenFree(gen);

//    out.flush();

    block.setInsertPoint(insertionPoint);

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

inline DdNode * BDDMinimizationPass::NewVar() {
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

