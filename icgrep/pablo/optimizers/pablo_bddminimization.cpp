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

using namespace llvm;
using namespace boost;

namespace pablo {

bool BDDMinimizationPass::optimize(PabloFunction & function) {
    raw_os_ostream out(std::cerr);
    PabloPrinter::print(function.getEntryBlock(), "", out);
    out << "\n****************************************************************\n\n";
    out.flush();

    promoteCrossBlockReachingDefs(function);

    BDDMinimizationPass am;
    am.initialize(function);
    am.characterizeAndEliminateLogicallyEquivalentStatements(function);
    am.simplifyAST(function);
    am.shutdown();

    return Simplifier::optimize(function);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief promoteCrossBlockReachingDefs
 * @param function
  *
 * Scan through the function and promote any cross-block statements into Assign nodes to simplify the BDD node
 * generation.
 ** ------------------------------------------------------------------------------------------------------------- */
void BDDMinimizationPass::promoteCrossBlockReachingDefs(const PabloFunction & function) {

    std::unordered_map<const PabloAST *, unsigned> block;
    std::stack<Statement *> scope;

    unsigned blockIndex = 0;

    for (const Statement * stmt = function.getEntryBlock().front(); ; ) {
        while ( stmt ) {

            if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                // Set the next statement to be the first statement of the inner scope and push the
                // next statement of the current statement into the scope stack.
                const PabloBlock & nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                scope.push(stmt->getNextNode());
                stmt = nested.front();
                assert (stmt);
                ++blockIndex;
                continue;
            }

            for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                Statement * const op = dyn_cast<Statement>(stmt->getOperand(i));
                if (op == nullptr || isa<Assign>(op) || isa<Next>(op) || block[op] == blockIndex) {
                    continue;
                }
                PabloBlock * const block = op->getParent();
                block->setInsertPoint(op);

                op->replaceAllUsesWith(block->createAssign("t", op));
            }

            block[stmt] = blockIndex;

            stmt = stmt->getNextNode();
        }
        if (scope.empty()) {
            break;
        }
        stmt = scope.top();
        scope.pop();
        ++blockIndex;
    }
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
                scope.push(stmt->getNextNode());
                stmt = nested.front();
                assert (stmt);
                continue;
            }
            switch (stmt->getClassTypeId()) {
                case PabloAST::ClassTypeId::Assign:
                case PabloAST::ClassTypeId::Next:
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
    mManager = Cudd_Init(variableCount + function.getNumOfParameters() - function.getNumOfResults(), 0, CUDD_UNIQUE_SLOTS, CUDD_CACHE_SLOTS, 0);
    Cudd_MakeTreeNode(mManager, 0, function.getNumOfParameters(), MTR_DEFAULT);
    // Map the predefined 0/1 entries
    mCharacterizationMap[function.getEntryBlock().createZeroes()] = Zero();
    mCharacterizationMap[function.getEntryBlock().createOnes()] = One();    
    // Order the variables so the input Vars are pushed to the end; they ought to
    // be the most complex to resolve.    
    for (auto i = 0; i != function.getNumOfParameters(); ++i) {
        mCharacterizationMap[function.getParameter(i)] = NewVar(function.getParameter(i));
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

            for (auto promotion : mPromotions) {
                mCharacterizationMap[promotion] = NewVar(promotion);
            }
            mPromotions.clear();

            characterizeAndEliminateLogicallyEquivalentStatements(isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody(), subsitutionMap);

            for (auto promotion : mPromotions) {
                mCharacterizationMap[promotion] = NewVar(promotion);
            }
            mPromotions.clear();

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
            mPromotions.push_back(stmt);
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
            return NewVar(stmt);
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
    PabloBuilder builder(function.getEntryBlock());
    simplifyAST(builder);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplifyAST
 ** ------------------------------------------------------------------------------------------------------------- */
void BDDMinimizationPass::simplifyAST(PabloBuilder & block) {
    for (Statement * stmt : block) {
        switch (stmt->getClassTypeId()) {
            case PabloAST::ClassTypeId::If:
            case PabloAST::ClassTypeId::While: {
                    PabloBlock & nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                    PabloBuilder builder(nested, block);
                    simplifyAST(builder);
                    if (isa<If>(stmt)) {
                        for (Assign * var : cast<const If>(stmt)->getDefined()) {
                            block.record(var);
                        }
                    } else {
                        for (Next * var : cast<const While>(stmt)->getVariants()) {
                            block.record(var);
                        }
                    }
                }
                break;
            case PabloAST::ClassTypeId::ScanThru:
            case PabloAST::ClassTypeId::MatchStar:
                simplifyAST(stmt, dyn_cast<Statement>(stmt->getOperand(1)), block);
            case PabloAST::ClassTypeId::Advance:
            case PabloAST::ClassTypeId::Assign:
            case PabloAST::ClassTypeId::Next:
                simplifyAST(stmt, dyn_cast<Statement>(stmt->getOperand(0)), block);
            default: block.record(stmt);
        }        
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplifyAST
 ** ------------------------------------------------------------------------------------------------------------- */
void BDDMinimizationPass::simplifyAST(Statement * stmt, Statement * const expr, PabloBuilder & builder) {
    if (expr && expr->getParent() == stmt->getParent()) {
        DdNode * const bdd = mCharacterizationMap[expr];
        if (bdd) {
            builder.getPabloBlock()->setInsertPoint(stmt->getPrevNode());
            Cudd_Ref(bdd);
            PabloAST * replacement = simplifyAST(bdd, builder);
            Cudd_RecursiveDeref(mManager, bdd);
            if (replacement) {
                expr->replaceWith(replacement, true, true);
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplifyAST
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * BDDMinimizationPass::simplifyAST(DdNode * const f, PabloBuilder & builder) {
    DdNode * g = Cudd_FindEssential(mManager, f);
    if (g && Cudd_SupportSize(mManager, g) > 0) {
        if (g == f) { // every variable is essential
            return makeCoverAST(f, builder);
        }
        Cudd_Ref(g);
        PabloAST * c0 = makeCoverAST(g, builder);
        if (LLVM_UNLIKELY(c0 == nullptr)) {
            Cudd_RecursiveDeref(mManager, g);
            return nullptr;
        }
        DdNode * h = Cudd_Cofactor(mManager, f, g);
        Cudd_Ref(h);
        Cudd_RecursiveDeref(mManager, g);
        PabloAST * c1 = simplifyAST(h, builder);
        Cudd_RecursiveDeref(mManager, h);
        if (LLVM_UNLIKELY(c1 == nullptr)) {
            if (LLVM_LIKELY(isa<Statement>(c0))) {
                cast<Statement>(c0)->eraseFromParent(true);
            }
            return nullptr;
        }
        return builder.createAnd(c0, c1);
    }
    DdNode ** disjunct = nullptr;
    const auto disjuncts = Cudd_bddVarConjDecomp(mManager, f, &disjunct);
    if (LLVM_LIKELY(disjuncts == 2)) {
        Cudd_Ref(disjunct[0]);
        Cudd_Ref(disjunct[1]);
        PabloAST * d0 = simplifyAST(disjunct[0], builder);
        Cudd_RecursiveDeref(mManager, disjunct[0]);
        if (LLVM_UNLIKELY(d0 == nullptr)) {
            Cudd_RecursiveDeref(mManager, disjunct[1]);
            return nullptr;
        }
        PabloAST * d1 = simplifyAST(disjunct[1], builder);
        Cudd_RecursiveDeref(mManager, disjunct[1]);
        FREE(disjunct);
        if (LLVM_UNLIKELY(d1 == nullptr)) {
            if (LLVM_LIKELY(isa<Statement>(d0))) {
                cast<Statement>(d0)->eraseFromParent(true);
            }
            return nullptr;
        }
        return builder.createAnd(d0, d1);
    }
    FREE(disjunct);
    return makeCoverAST(f, builder);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeCoverAST
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * BDDMinimizationPass::makeCoverAST(DdNode * const f, PabloBuilder & builder) {

    std::queue<PabloAST *> SQ;

    circular_buffer<PabloAST *> CQE(mManager->size);
    circular_buffer<PabloAST *> CQO(mManager->size);

    circular_buffer<PabloAST *> DQE(mManager->size);
    circular_buffer<PabloAST *> DQO(mManager->size);

    int cube[mManager->size];

    DdNode * g = f;

    Cudd_Ref(g);

    while (g != Cudd_ReadLogicZero(mManager)) {
        int length;
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

        for (auto i = 0; i != mManager->size; ++i) {
            if ((i & 1) == 0) { // i is even
                if (cube[i] == 0) {
                    DQE.push_back(mVariables[i]);
                } else if (cube[i] == 1) {
                    CQE.push_back(mVariables[i]);
                }
            } else { // i is odd
                if (cube[i] == 0) {
                    DQO.push_back(mVariables[i]);
                } else if (cube[i] == 1) {
                    CQO.push_back(mVariables[i]);
                }
            }
        }

        PabloAST * dq = builder.createZeroes();
        if (!DQE.empty() || !DQO.empty()) {
            while (DQE.size() > 1) {
                PabloAST * v1 = DQE.front(); DQE.pop_front();
                PabloAST * v2 = DQE.front(); DQE.pop_front();
                DQE.push_back(builder.createOr(v1, v2));
            }
            while (DQO.size() > 1) {
                PabloAST * v1 = DQO.front(); DQO.pop_front();
                PabloAST * v2 = DQO.front(); DQO.pop_front();
                DQO.push_back(builder.createOr(v1, v2));
            }
            if (DQE.empty()) {
                dq = DQO.front();
            } else if (DQO.empty()) {
                dq = DQE.front();
            } else {
                dq = builder.createOr(DQE.front(), DQO.front());
            }
            DQE.clear();
            DQO.clear();
        }

        PabloAST * cq = builder.createOnes();
        if (!CQE.empty() || !CQO.empty()) {
            while (CQE.size() > 1) {
                PabloAST * v1 = CQE.front(); CQE.pop_front();
                PabloAST * v2 = CQE.front(); CQE.pop_front();
                CQE.push_back(builder.createOr(v1, v2));
            }
            while (CQO.size() > 1) {
                PabloAST * v1 = CQO.front(); CQO.pop_front();
                PabloAST * v2 = CQO.front(); CQO.pop_front();
                CQO.push_back(builder.createOr(v1, v2));
            }
            if (CQE.empty()) {
                dq = CQO.front();
            } else if (CQO.empty()) {
                dq = CQE.front();
            } else {
                dq = builder.createAnd(CQE.front(), CQO.front());
            }
            CQE.clear();
            CQO.clear();
        }
        SQ.push(builder.createAnd(cq, builder.createNot(dq)));
    }
    Cudd_RecursiveDeref(mManager, g);

    while (SQ.size() > 1) {
        PabloAST * v1 = SQ.front(); SQ.pop();
        PabloAST * v2 = SQ.front(); SQ.pop();
        SQ.push(builder.createOr(v1, v2));
    }

    return SQ.front();
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

inline DdNode * BDDMinimizationPass::NewVar(const PabloAST * expr) {
    DdNode * var = Cudd_bddIthVar(mManager, mVariables.size());
    mVariables.push_back(const_cast<PabloAST *>(expr));
    return var;
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

