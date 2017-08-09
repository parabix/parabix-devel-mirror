#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/pablo_kernel.h>
#include <pablo/codegenstate.h>
#include <pablo/expression_map.hpp>
#include <pablo/boolean.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <pablo/arithmetic.h>
#include <pablo/branch.h>
#include <pablo/ps_assign.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_var.h>
#include <boost/container/flat_set.hpp>
#ifndef NDEBUG
#include <pablo/analysis/pabloverifier.hpp>
#endif
#include <llvm/Support/raw_ostream.h>


using namespace boost;
using namespace boost::container;
using namespace llvm;

namespace pablo {

using TypeId = PabloAST::ClassTypeId;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief fold
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * triviallyFold(Statement * stmt, PabloBlock * const block) {
    if (isa<Not>(stmt)) {
        PabloAST * value = stmt->getOperand(0);
        if (LLVM_UNLIKELY(isa<Not>(value))) {
            return cast<Not>(value)->getOperand(0); // ¬¬A ⇔ A
        } else if (LLVM_UNLIKELY(isa<Zeroes>(value))) {
            return block->createOnes(stmt->getType()); // ¬0 ⇔ 1
        }  else if (LLVM_UNLIKELY(isa<Ones>(value))) {
            return block->createZeroes(stmt->getType()); // ¬1 ⇔ 0
        }
    } else if (isa<Advance>(stmt)) {
        if (LLVM_UNLIKELY(isa<Zeroes>(stmt->getOperand(0)))) {
            return block->createZeroes(stmt->getType());
        }
    } else if (isa<Add>(stmt) || isa<Subtract>(stmt)) {
       if (LLVM_UNLIKELY(isa<Integer>(stmt->getOperand(0)) && isa<Integer>(stmt->getOperand(1)))) {
           const Integer * const int0 = cast<Integer>(stmt->getOperand(0));
           const Integer * const int1 = cast<Integer>(stmt->getOperand(1));
           Integer::IntTy result = 0;
           if (isa<Add>(stmt)) {
               result = int0->value() + int1->value();
           } else {
               result = int0->value() - int1->value();
           }
           return block->getInteger(result);
       }
    } else {
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            if (LLVM_UNLIKELY(isa<Zeroes>(stmt->getOperand(i)))) {
                switch (stmt->getClassTypeId()) {
                    case TypeId::Sel:
                        block->setInsertPoint(stmt->getPrevNode());
                        switch (i) {
                            case 0: return stmt->getOperand(2);
                            case 1: return block->createAnd(block->createNot(stmt->getOperand(0)), stmt->getOperand(2));
                            case 2: return block->createAnd(stmt->getOperand(0), stmt->getOperand(1));
                        }
                    case TypeId::ScanThru:
                    case TypeId::MatchStar:
                        return stmt->getOperand(0);
                    default: break;
                }
            } else if (LLVM_UNLIKELY(isa<Ones>(stmt->getOperand(i)))) {
                switch (stmt->getClassTypeId()) {
                    case TypeId::Sel:
                        block->setInsertPoint(stmt->getPrevNode());
                        switch (i) {
                            case 0: return stmt->getOperand(1);
                            case 1: return block->createOr(stmt->getOperand(0), stmt->getOperand(2));
                            case 2: return block->createOr(block->createNot(stmt->getOperand(0)), stmt->getOperand(1));
                        }
                    case TypeId::ScanThru:
                        if (LLVM_UNLIKELY(i == 1)) {
                            return block->createZeroes(stmt->getType());
                        }
                        break;
                    case TypeId::MatchStar:
                        if (LLVM_UNLIKELY(i == 0)) {
                            return block->createOnes(stmt->getType());
                        }
                        break;
                    default: break;
                }
            }
        }        
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief VariableTable
 ** ------------------------------------------------------------------------------------------------------------- */
struct VariableTable {

    VariableTable(VariableTable * predecessor = nullptr)
    : mPredecessor(predecessor) {

    }

    PabloAST * get(PabloAST * const var) const {
        const auto f = mMap.find(var);
        if (f == mMap.end()) {
            return (mPredecessor) ? mPredecessor->get(var) : nullptr;
        }
        return f->second;
    }

    void put(PabloAST * const var, PabloAST * value) {
        const auto f = mMap.find(var);
        if (LLVM_LIKELY(f == mMap.end())) {
            mMap.emplace(var, value);
        } else {
            f->second = value;
        }
        assert (get(var) == value);
    }

    bool isNonZero(const PabloAST * const var) const {
        if (mNonZero.count(var) != 0) {
            return true;
        } else if (mPredecessor) {
            return mPredecessor->isNonZero(var);
        }
        return false;
    }

    void addNonZero(const PabloAST * const var) {
        mNonZero.insert(var);
    }

private:
    VariableTable * const mPredecessor;
    flat_map<PabloAST *, PabloAST *> mMap;
    flat_set<const PabloAST *> mNonZero;
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isTrivial
 *
 * If this inner block is composed of only Boolean logic and Assign statements and there are fewer than 3
 * statements, just add the statements in the inner block to the current block
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool isTrivial(const PabloBlock * const block) {
    unsigned computations = 0;
    for (const Statement * stmt : *block) {
        switch (stmt->getClassTypeId()) {
            case TypeId::And:
            case TypeId::Or:
            case TypeId::Xor:
                if (++computations > 3) {
                    return false;
                }
            case TypeId::Not:
            case TypeId::Assign:
                break;
            default:
                return false;
        }
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief flatten
 ** ------------------------------------------------------------------------------------------------------------- */
Statement * flatten(Branch * const br) {
    Statement * stmt = br;
    Statement * nested = br->getBody()->front();
    while (nested) {
        Statement * next = nested->removeFromParent();
        nested->insertAfter(stmt);
        stmt = nested;
        nested = next;
    }
    return br->eraseFromParent();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief redundancyElimination
 *
 * Note: Do not recursively delete statements in this function. The ExpressionTable could use deleted statements
 * as replacements. Let the DCE remove the unnecessary statements with the finalized Def-Use information.
 ** ------------------------------------------------------------------------------------------------------------- */
void redundancyElimination(PabloBlock * const block, ExpressionTable * const et, VariableTable * const vt) {
    VariableTable variables(vt);

    // When processing a While body, we cannot use its initial value from the outer
    // body since the Var will likely be assigned a different value in the current
    // body that should be used on the subsequent iteration of the loop.
    if (Branch * br = block->getBranch()) {
        assert ("block has a branch but the expression and variable tables were not supplied" && et && vt);
        variables.addNonZero(br->getCondition());
        for (Var * var : br->getEscaped()) {
            variables.put(var, var);
        }
    }

    ExpressionTable expressions(et);

    Statement * stmt = block->front();
    while (stmt) {

        if (LLVM_UNLIKELY(isa<Assign>(stmt))) {
            Assign * const assign = cast<Assign>(stmt);
            PabloAST * const var = assign->getVariable();
            PabloAST * value = assign->getValue();
            while (LLVM_UNLIKELY(isa<Var>(value))) {
                PabloAST * next = variables.get(cast<Var>(value));
                if (LLVM_LIKELY(next == nullptr || next == value)) {
                    break;
                }
                value = next;
                assign->setValue(value);
            }
            if (LLVM_UNLIKELY(variables.get(var) == value)) {
                stmt = stmt->eraseFromParent();
                continue;
            }
            variables.put(var, value);
        } else if (LLVM_UNLIKELY(isa<Branch>(stmt))) {

            Branch * const br = cast<Branch>(stmt);

            // Test whether we can ever take this branch
            PabloAST * cond = br->getCondition();
            if (isa<Var>(cond)) {
                PabloAST * const value = variables.get(cast<Var>(cond));
                if (value) {
                    cond = value;
                    // TODO: verify this works for a nested If node within a While body.
                    if (isa<If>(br)) {
                        br->setCondition(cond);
                    }
                }
            }

            if (LLVM_UNLIKELY(isa<Zeroes>(cond))) {
                stmt = stmt->eraseFromParent();
                continue;
            }

            if (LLVM_LIKELY(isa<If>(br))) {
                if (LLVM_UNLIKELY(variables.isNonZero(cond))) {
                    stmt = flatten(br);
                    continue;
                }
            }

            // Process the Branch body
            redundancyElimination(br->getBody(), &expressions, &variables);

            if (LLVM_LIKELY(isa<If>(br))) {
                // Check whether the cost of testing the condition and taking the branch with
                // 100% correct prediction rate exceeds the cost of the body itself
                if (LLVM_UNLIKELY(isTrivial(br->getBody()))) {
                    stmt = flatten(br);
                    continue;
                }
            }

        } else {

            // demote any uses of a Var whose value is in scope
            for (unsigned i = 0; i < stmt->getNumOperands(); ++i) {
                PabloAST * op = stmt->getOperand(i);
                if (LLVM_UNLIKELY(isa<Var>(op))) {
                    PabloAST * const value = variables.get(cast<Var>(op));
                    if (value && value != op) {
                        stmt->setOperand(i, value);
                    }
                }
            }

            PabloAST * const folded = triviallyFold(stmt, block);
            if (folded) {
                stmt = stmt->replaceWith(folded);
                continue;
            }

            // By recording which statements have already been seen, we can detect the redundant statements
            // as any having the same type and operands. If so, we can replace its users with the prior statement.
            // and erase this statement from the AST
            const auto f = expressions.findOrAdd(stmt);
            if (!f.second) {
                stmt = stmt->replaceWith(f.first);
                continue;
            }

            // Check whether this statement is trivially non-zero and if so, add it to our set of non-zero variables.
            // This will allow us to flatten an If scope if its branch is always taken.
            if (isa<Or>(stmt)) {
                for (unsigned i = 0; i < stmt->getNumOperands(); ++i) {
                    if (LLVM_UNLIKELY(variables.isNonZero(stmt->getOperand(i)))) {
                        variables.addNonZero(stmt);
                        break;
                    }
                }
            } else if (isa<Advance>(stmt)) {
                const Advance * const adv = cast<Advance>(stmt);
                if (LLVM_LIKELY(adv->getAmount() < 32)) {
                    if (LLVM_UNLIKELY(variables.isNonZero(adv->getExpression()))) {
                        variables.addNonZero(adv);
                    }
                }
            }
        }

        stmt = stmt->getNextNode();
    }

    // If this block has a branch statement leading into it, we can verify whether an escaped value
    // was updated within this block and update the preceeding block's variable state appropriately.

    if (Branch * const br = block->getBranch()) {

        // When removing identical escaped values, we have to consider that the identical Vars could
        // be assigned new differing values later in the outer body. Thus instead of replacing them
        // directly, we map future uses of the duplicate Var to the initial one. The DCE pass will
        // later mark any Assign statement as dead if the Var is never read.

        /// TODO: this doesn't properly optimize the loop control variable(s) yet.

        const auto escaped = br->getEscaped();
        const auto n = escaped.size();
        PabloAST * variable[n];
        PabloAST * incoming[n];
        PabloAST * outgoing[n];

        for (unsigned i = 0; i < escaped.size(); ++i) {
            PabloAST * var = escaped[i];
            incoming[i] = vt->get(var);
            outgoing[i] = variables.get(var);
            if (LLVM_UNLIKELY(incoming[i] == outgoing[i])) {
                var = incoming[i];
            } else {
                for (size_t j = 0; j != i; ++j) {
                    if ((outgoing[j] == outgoing[i]) && (incoming[j] == incoming[i])) {
                        var = variable[j];
                        break;
                    }
                }
            }
            variable[i] = var;
            vt->put(escaped[i], var);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deadCodeElimination
 ** ------------------------------------------------------------------------------------------------------------- */
void deadCodeElimination(PabloBlock * const block) {

    flat_map<PabloAST *, Assign *> unread;

    Statement * stmt = block->front();
    while (stmt) {
        if (unread.size() != 0) {
            for (unsigned i = 0; i < stmt->getNumOperands(); ++i) {
                PabloAST * const op = stmt->getOperand(i);
                if (LLVM_UNLIKELY(isa<Var>(op))) {
                    unread.erase(op);
                }
            }
        }
        if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
            Branch * const br = cast<Branch>(stmt);
            deadCodeElimination(br->getBody());
            if (LLVM_UNLIKELY(br->getEscaped().empty())) {
                stmt = stmt->eraseFromParent(true);
                continue;
            }
        } else if (LLVM_UNLIKELY(isa<Assign>(stmt))) {
            // An Assign statement is locally dead whenever its variable is not read
            // before being reassigned a value.
            PabloAST * var = cast<Assign>(stmt)->getVariable();
            auto f = unread.find(var);
            if (f != unread.end()) {
                auto prior = f->second;
                prior->eraseFromParent(true);
                f->second = cast<Assign>(stmt);
            } else {
                unread.emplace(var, cast<Assign>(stmt));
            }
        } else if (LLVM_UNLIKELY(stmt->getNumUses() == 0)) {
            stmt = stmt->eraseFromParent(true);
            continue;
        }
        stmt = stmt->getNextNode();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deadCodeElimination
 ** ------------------------------------------------------------------------------------------------------------- */
void deadCodeElimination(PabloKernel * kernel) {

    deadCodeElimination(kernel->getEntryBlock());

    for (unsigned i = 0; i < kernel->getNumOfVariables(); ++i) {
        Var * var = kernel->getVariable(i);
        bool unused = true;
        for (PabloAST * user : var->users()) {
            if (isa<Assign>(user)) {
                if (cast<Assign>(user)->getValue() == var) {
                    unused = false;
                    break;
                }
            } else {
                unused = false;
                break;
            }
        }
        if (LLVM_UNLIKELY(unused)) {
            for (PabloAST * user : var->users()) {
                cast<Assign>(user)->eraseFromParent(true);
            }
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief strengthReduction
 *
 * Find and replace any Pablo operations with a less expensive equivalent operation whenever possible.
 ** ------------------------------------------------------------------------------------------------------------- */
void strengthReduction(PabloBlock * const block) {

    Statement * stmt = block->front();
    while (stmt) {
        if (isa<Branch>(stmt)) {
            strengthReduction(cast<Branch>(stmt)->getBody());
        } else if (isa<Advance>(stmt)) {
            Advance * adv = cast<Advance>(stmt);
            if (LLVM_UNLIKELY(isa<Advance>(adv->getOperand(0)))) {
                // Replace an Advance(Advance(x, n), m) with an Advance(x,n + m)
                // Test whether this will generate a long advance and abort?
                Advance * op = cast<Advance>(stmt->getOperand(0));
                if (LLVM_UNLIKELY(op->getNumUses() == 1)) {
                    adv->setOperand(0, op->getOperand(0));
                    adv->setOperand(1, block->getInteger(adv->getAmount() + op->getAmount()));
                    op->eraseFromParent(false);
                }
            }
        } else if (LLVM_UNLIKELY(isa<ScanThru>(stmt))) {
            ScanThru * scanThru = cast<ScanThru>(stmt);
            if (LLVM_UNLIKELY(isa<Advance>(scanThru->getScanFrom()))) {
                // Replace a ScanThru(Advance(x,n),y) with an ScanThru(Advance(x, n - 1), Advance(x, n - 1) | y), where Advance(x, 0) = x
                Advance * adv = cast<Advance>(scanThru->getScanFrom());
                if (LLVM_UNLIKELY(adv->getNumUses() == 1)) {
                    PabloAST * stream = adv->getExpression();
                    block->setInsertPoint(stmt);
                    if (LLVM_UNLIKELY(adv->getAmount() != 1)) {
                        stream = block->createAdvance(stream, block->getInteger(adv->getAmount() - 1));
                    }
                    stmt = scanThru->replaceWith(block->createAdvanceThenScanThru(stream, scanThru->getScanThru()));
                    adv->eraseFromParent(false);
                    continue;
                }
            } else if (LLVM_UNLIKELY(isa<And>(scanThru->getScanFrom()))) {
                // Suppose B is an arbitrary bitstream and A = Advance(B, 1). ScanThru(B ∧ ¬A, B) will leave a marker on the position
                // following the end of any run of 1-bits in B. But this is equivalent to computing A ∧ ¬B since A will have exactly
                // one 1-bit past the end of any run of 1-bits in B.





            }
        } else if (LLVM_UNLIKELY(isa<ScanTo>(stmt))) {
            ScanTo * scanTo = cast<ScanTo>(stmt);
            if (LLVM_UNLIKELY(isa<Advance>(scanTo->getScanFrom()))) {
                // Replace a ScanTo(Advance(x,n),y) with an ScanTo(Advance(x, n - 1), Advance(x, n - 1) | y), where Advance(x, 0) = x
                Advance * adv = cast<Advance>(scanTo->getScanFrom());
                if (LLVM_UNLIKELY(adv->getNumUses() == 1)) {
                    PabloAST * stream = adv->getExpression();
                    block->setInsertPoint(stmt);
                    if (LLVM_UNLIKELY(adv->getAmount() != 1)) {
                        stream = block->createAdvance(stream, block->getInteger(adv->getAmount() - 1));
                    }
                    stmt = scanTo->replaceWith(block->createAdvanceThenScanTo(stream, scanTo->getScanTo()));
                    adv->eraseFromParent(false);
                    continue;
                }
            }
        }
        stmt = stmt->getNextNode();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool Simplifier::optimize(PabloKernel * kernel) {
    redundancyElimination(kernel->getEntryBlock(), nullptr, nullptr);
    strengthReduction(kernel->getEntryBlock());
    deadCodeElimination(kernel);
    #ifndef NDEBUG
    PabloVerifier::verify(kernel, "post-simplification");
    #endif
    return true;
}

}
