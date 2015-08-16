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

using namespace llvm;
using namespace boost;
using namespace boost::container;

namespace pablo {

bool BDDMinimizationPass::optimize(PabloFunction & function, const bool full) {
    BDDMinimizationPass am;
    am.eliminateLogicallyEquivalentStatements(function);
    if (full) {
        am.simplifyAST(function);
    }
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

    SubsitutionMap baseMap;
    baseMap.insert(Zero(), function.getEntryBlock().createZeroes());
    baseMap.insert(One(), function.getEntryBlock().createOnes());

    Cudd_AutodynEnable(mManager, CUDD_REORDER_LAZY_SIFT);

    eliminateLogicallyEquivalentStatements(function.getEntryBlock(), baseMap);

    Cudd_Quit(mManager);
    mCharacterizationMap.clear();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 * @param vars the input vars for this program
 *
 * Scan through the program and iteratively compute the BDDs for each statement.
 ** ------------------------------------------------------------------------------------------------------------- */
void BDDMinimizationPass::eliminateLogicallyEquivalentStatements(PabloBlock & block, SubsitutionMap & parent) {
    SubsitutionMap subsitutionMap(&parent);
    Statement * stmt = block.front();

    while (stmt) {
        if (LLVM_UNLIKELY(isa<If>(stmt))) {
            eliminateLogicallyEquivalentStatements(cast<If>(stmt)->getBody(), subsitutionMap);
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            eliminateLogicallyEquivalentStatements(cast<While>(stmt)->getBody(), subsitutionMap);
        } else { // attempt to characterize this statement and replace it if
            DdNode * bdd = eliminateLogicallyEquivalentStatements(stmt);
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
inline DdNode * BDDMinimizationPass::eliminateLogicallyEquivalentStatements(const Statement * const stmt) {

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
    std::vector<Statement *> terminals;
    for (unsigned i = 0; i != function.getNumOfResults(); ++i) {
        terminals.push_back(function.getResult(i));
    }
    simplifyAST(builder, terminals);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isSimplifiable
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool isSimplifiable(const PabloAST * const expr, const PabloBlock * const pb) {
    switch (expr->getClassTypeId()) {
        case PabloAST::ClassTypeId::And:
        case PabloAST::ClassTypeId::Or:
        case PabloAST::ClassTypeId::Not:
        case PabloAST::ClassTypeId::Sel:
            return cast<Statement>(expr)->getParent() == pb;
        default:
            return false;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplifyAST
 ** ------------------------------------------------------------------------------------------------------------- */
void BDDMinimizationPass::simplifyAST(PabloBuilder & block, std::vector<Statement *> & terminals) {

    for (Statement * stmt : block) {
        if (isa<While>(stmt)) {
            PabloBuilder builder(cast<If>(stmt)->getBody(), block);
            std::vector<Statement *> terminals;
            for (Assign * var : cast<const If>(stmt)->getDefined()) {
                terminals.push_back(var);
            }
            simplifyAST(builder, terminals);
            for (Assign * var : cast<const If>(stmt)->getDefined()) {
                block.record(var);
            }
            continue;
        } else if (isa<While>(stmt)) {
            PabloBuilder builder(cast<While>(stmt)->getBody(), block);
            std::vector<Statement *> terminals;
            for (Next * var : cast<const While>(stmt)->getVariants()) {
                terminals.push_back(var);
            }
            simplifyAST(builder, terminals);
            for (Next * var : cast<const While>(stmt)->getVariants()) {
                block.record(var);
            }
            continue;
        }
        block.record(stmt);
    }

    std::queue<Statement *> Q;
    std::vector<PabloAST *> variables;

    while (!terminals.empty()) {

        for (Statement * term : terminals) {
            Q.push(term);
        }

        // find the variables for this set of terminals
        for (;;) {
            Statement * stmt = Q.front();
            Q.pop();
            mCharacterizationMap[stmt] = nullptr;
            for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                if (isSimplifiable(stmt->getOperand(i), stmt->getParent())) {
                    Q.push(cast<Statement>(stmt->getOperand(i)));
                } else {
                    variables.push_back(stmt->getOperand(i));
                }
            }
            if (Q.empty()) {
                break;
            }
        }

        std::sort(variables.begin(), variables.end());
        std::unique(variables.begin(), variables.end());


        mManager = Cudd_Init(variables.size(), 0, CUDD_UNIQUE_SLOTS, CUDD_CACHE_SLOTS, 0);
        Cudd_AutodynEnable(mManager, CUDD_REORDER_LAZY_SIFT);

        unsigned i = 0;
        for (PabloAST * var : variables) {
            mCharacterizationMap[var] = Cudd_bddIthVar(mManager, i++);
        }

        for (PabloAST * term : terminals) {
            characterizeTerminalBddTree(term);
        }
        Cudd_ReduceHeap(mManager, CUDD_REORDER_SIFT, 0);

        for (Statement * term : terminals) {
            DdNode * const f = mCharacterizationMap[term];
            Cudd_Ref(f);
            PabloAST * replacement = simplifyAST(f, variables, block);
            if (replacement) {
                term->replaceWith(replacement, false, true);
            }
            Cudd_RecursiveDeref(mManager, f);
        }

        Cudd_Quit(mManager);

        mCharacterizationMap.clear();

        // Now clear our terminals and test whether we can process another layer within this block
        terminals.clear();
        for (PabloAST * var : variables) {
            if (isa<Statement>(var) && cast<Statement>(var)->getParent() == block.getPabloBlock()) {
                terminals.push_back(cast<Statement>(var));
            }
        }
    }

}

///** ------------------------------------------------------------------------------------------------------------- *
// * @brief simplifyAST
// ** ------------------------------------------------------------------------------------------------------------- */
DdNode * BDDMinimizationPass::characterizeTerminalBddTree(PabloAST * expr) {

    const auto f = mCharacterizationMap.find(expr);
    if (f == mCharacterizationMap.end()) {
        return nullptr;
    } else if (f->second) {
        return f->second;
    }

    Statement * stmt = cast<Statement>(expr);
    std::array<DdNode *, 3> input;

    for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
        input[i] = characterizeTerminalBddTree(stmt->getOperand(i));
    }

    DdNode * bdd = nullptr;
    switch (expr->getClassTypeId()) {
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
        default:
            throw std::runtime_error("Unexpected operand given to BDDMinimizationPass::characterizeTerminalBddTree!");
    }
    Cudd_Ref(bdd);
    return bdd;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplifyAST
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * BDDMinimizationPass::simplifyAST(DdNode * const f, const std::vector<PabloAST *> & variables, PabloBuilder & builder) {
    assert (!noSatisfyingAssignment(f));
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
PabloAST * BDDMinimizationPass::makeCoverAST(DdNode * const f, const std::vector<PabloAST *> & variables, PabloBuilder & builder) {

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

