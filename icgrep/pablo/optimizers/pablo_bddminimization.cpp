#include "pablo_bddminimization.h"
#include <pablo/codegenstate.h>
#include <pablo/builder.hpp>
#include <pablo/printer_pablos.h>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/analysis/pabloverifier.hpp>
#include <stack>
#include <bdd.h>

using namespace llvm;
using namespace boost;
using namespace boost::container;

namespace pablo {

using TypeId = PabloAST::ClassTypeId;

// TODO: add in analysis to verify that the outputs of an If would be zero if the If cond is false?

// TODO: test whether an If node can be moved into another If body in the same scope?

bool BDDMinimizationPass::optimize(PabloFunction & function) {
    BDDMinimizationPass am;
    am.initialize(function);
    am.eliminateLogicallyEquivalentStatements(function);
    bdd_done();
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-bdd-minimization");
    #endif
    return Simplifier::optimize(function);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initialize
 *
 * Scan through the program to identify any advances and calls then initialize the BDD engine with
 * the proper variable ordering.
 ** ------------------------------------------------------------------------------------------------------------- */
void BDDMinimizationPass::initialize(PabloFunction & function) {

    std::stack<Statement *> scope;
    unsigned variableCount = function.getNumOfParameters(); // number of statements that cannot always be categorized without generating a new variable
    unsigned statementCount = 0;
    for (const Statement * stmt = function.getEntryBlock()->front(); ; ) {
        while ( stmt ) {
            if (LLVM_UNLIKELY(isa<If>(stmt) || isa<While>(stmt))) {
                // Set the next statement to be the first statement of the inner scope and push the
                // next statement of the current statement into the scope stack.
                const PabloBlock * const nested = isa<If>(stmt) ? cast<If>(stmt)->getBody() : cast<While>(stmt)->getBody();
                scope.push(stmt->getNextNode());
                stmt = nested->front();
                assert (stmt);
                continue;
            }
            ++statementCount;
            switch (stmt->getClassTypeId()) {
                case TypeId::Advance:
                case TypeId::Call:
                case TypeId::MatchStar:
                case TypeId::ScanThru:
                    ++variableCount;
                    break;
                default: break;
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
    bdd_init(1000000, 10000);
    bdd_setvarnum(variableCount + function.getNumOfParameters());
    bdd_setcacheratio(32);
    bdd_setmaxincrease(1000000);
    bdd_disable_reorder();

    // Map the predefined 0/1 entries
    mCharacterizationMap[PabloBlock::createZeroes()] = bdd_zero();
    mCharacterizationMap[PabloBlock::createOnes()] = bdd_one();
    // Order the variables so the input Vars are pushed to the end; they ought to
    // be the most complex to resolve.    
    for (mVariables = 0; mVariables != function.getNumOfParameters(); ++mVariables) {
        mCharacterizationMap[function.getParameter(mVariables)] = bdd_ithvar(mVariables);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief eliminateLogicallyEquivalentStatements
 ** ------------------------------------------------------------------------------------------------------------- */
void BDDMinimizationPass::eliminateLogicallyEquivalentStatements(PabloFunction & function) {
    SubsitutionMap baseMap;
    baseMap.insert(bdd_zero(), PabloBlock::createZeroes());
    baseMap.insert(bdd_one(), PabloBlock::createOnes());
    eliminateLogicallyEquivalentStatements(function.getEntryBlock(), baseMap);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief eliminateLogicallyEquivalentStatements
 ** ------------------------------------------------------------------------------------------------------------- */
void BDDMinimizationPass::eliminateLogicallyEquivalentStatements(PabloBlock * const block, SubsitutionMap & parent) {
    SubsitutionMap map(&parent);
    Statement * stmt = block->front();
    while (stmt) {
        if (LLVM_UNLIKELY(isa<If>(stmt))) {
            eliminateLogicallyEquivalentStatements(cast<If>(stmt)->getBody(), map);
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            eliminateLogicallyEquivalentStatements(cast<While>(stmt)->getBody(), map);
            for (Next * var : cast<const While>(stmt)->getVariants()) {
                if (!escapes(var)) {
                    bdd_delref(mCharacterizationMap[var]);
                    mCharacterizationMap.erase(var);
                }
            }
        } else { // attempt to characterize this statement and replace it if we've encountered an equivalent one

            /// TODO: I found evidence that some of the UCD functions have disjunctions of nested marker values
            /// in which one is a superset of the other. It's not safe to eliminate the subset marker unless the
            /// condition that lead us to compute the first marker is a superset of the condition that let us
            /// compute the subset value too. We can alter the superset condition to include the union of both
            /// but this may lead to taking an expensive branch more often. So, we'd need to decide whether the
            /// cost of each scope is close enough w.r.t. the probability both branches are taken.

            BDD bdd = bdd_zero();
            bool test = false;
            std::tie(bdd, test) = characterize(stmt);
            if (test) {
                PabloAST * replacement = map.get(bdd);
                if (LLVM_UNLIKELY(replacement != nullptr)) {
                    bdd_delref(bdd);
                    stmt = stmt->replaceWith(replacement, false, true);
                    continue;
                }
            } else if (LLVM_LIKELY(!bdd_constant(bdd))) {
                map.insert(bdd, stmt);
            }
            mCharacterizationMap.emplace(stmt, bdd);
        }
        stmt = stmt->getNextNode();
    }   
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief characterize
 ** ------------------------------------------------------------------------------------------------------------- */
inline std::pair<BDD, bool> BDDMinimizationPass::characterize(Statement * const stmt) {

    BDD bdd = bdd_zero();
    // Map our operands to the computed BDDs
    std::array<BDD, 3> input;
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
            msg << "BDDMinimizationPass: uncharacterized operand " << std::to_string(i) << " of ";
            PabloPrinter::print(stmt, msg);
            throw std::runtime_error(msg.str());
        }
        input[i] = f->second;
    }

    switch (stmt->getClassTypeId()) {
        case TypeId::Assign:
        case TypeId::Next:
            return std::make_pair(input[0], false);
        case TypeId::And:
            bdd = bdd_and(input[0], input[1]);
            break;
        case TypeId::Or:
            bdd = bdd_or(input[0], input[1]);
            break;
        case TypeId::Xor:
            bdd = bdd_xor(input[0], input[1]);
            break;
        case TypeId::Not:
            bdd = bdd_not(input[0]);
            break;
        case TypeId::Sel:
            bdd = bdd_ite(input[0], input[1], input[2]);
            break;
        case TypeId::MatchStar:
        case TypeId::ScanThru:
            if (LLVM_UNLIKELY(input[1] == bdd_zero())) {
                bdd = input[0];
            }
        case TypeId::Advance:
            if (LLVM_UNLIKELY(input[0] == bdd_zero())) {
                return std::make_pair(bdd_zero(), true);
            }
        case TypeId::Call:
            // TODO: we may have more than one output. Need to fix call class to allow for it.
            return std::make_pair(bdd_ithvar(mVariables++), false);
        default:
            throw std::runtime_error("Unexpected statement type " + stmt->getName()->to_string());
    }
    bdd_addref(bdd);
    if (LLVM_UNLIKELY(bdd_satone(bdd) == bdd_zero())) {
        bdd_delref(bdd);
        bdd = bdd_zero();
    }
    return std::make_pair(bdd, true);
}

} // end of namespace pablo

