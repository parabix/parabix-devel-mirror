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
#include <pablo/pe_lookahead.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_var.h>
#ifndef NDEBUG
#include <pablo/analysis/pabloverifier.hpp>
#endif
#include <boost/container/flat_set.hpp>
#include <llvm/IR/Type.h>
#include <llvm/IR/DerivedTypes.h>  // for get getSequentialElementType
#include <llvm/Support/raw_ostream.h>

using namespace boost;
using namespace boost::container;
using namespace llvm;

namespace pablo {

using TypeId = PabloAST::ClassTypeId;

using ScopeMap = flat_map<PabloBlock *, unsigned>;

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

private:
    VariableTable * const mPredecessor;
    flat_map<PabloAST *, PabloAST *> mMap;
};

struct PassContainer {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief run
 ** ------------------------------------------------------------------------------------------------------------- */
void run(PabloKernel * const kernel) {
    redundancyElimination(kernel->getEntryScope(), nullptr, nullptr);
    strengthReduction(kernel->getEntryScope());
    deadCodeElimination(kernel->getEntryScope());
}

protected:

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief redundancyElimination
 *
 * Note: Do not recursively delete statements in this function. The ExpressionTable could use deleted statements
 * as replacements. Let the DCE remove the unnecessary statements with the finalized Def-Use information.
 ** ------------------------------------------------------------------------------------------------------------- */
void redundancyElimination(PabloBlock * const block, ExpressionTable * const et, VariableTable * const vt) {
    ExpressionTable expressions(et);
    VariableTable variables(vt);

    if (Branch * br = block->getBranch()) {
        assert ("block has a branch but the expression and variable tables were not supplied" && et && vt);
        for (Var * var : br->getEscaped()) {
            variables.put(var, var);
        }
    }

    mInScope.push_back(block);

    const auto baseNonZeroEntries = mNonZero.size();
    Statement * stmt = block->front();
    while (stmt) {
        if (LLVM_UNLIKELY(isa<Assign>(stmt))) {
            Assign * const assign = cast<Assign>(stmt);
            PabloAST * const var = assign->getVariable();
            PabloAST * value = assign->getValue();
            if (LLVM_UNLIKELY(var == value)) {
                stmt = stmt->eraseFromParent();
                continue;
            }
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
            PabloAST * cond = br->getCondition();
            if (isa<Var>(cond)) {
                PabloAST * const value = variables.get(cast<Var>(cond));
                if (value) {
                    cond = value;
                    if (isa<If>(br)) {
                        br->setCondition(cond);
                    }
                }
            }

            // Test whether we can ever take this branch
            if (LLVM_UNLIKELY(isa<Zeroes>(cond))) {
                stmt = stmt->eraseFromParent();
                continue;
            }

            // If we're guaranteed to take this branch, flatten it.
            if (LLVM_LIKELY(isa<If>(br)) && LLVM_UNLIKELY(isNonZero(cond))) {
                stmt = flatten(br);
                continue;
            }

            // Mark the cond as non-zero prior to processing the inner scope.
            mNonZero.push_back(cond);
            // Process the Branch body
            redundancyElimination(br->getBody(), &expressions, &variables);
            assert (mNonZero.back() == cond);
            mNonZero.pop_back();

            if (LLVM_LIKELY(isa<If>(br))) {
                // Check whether the cost of testing the condition and taking the branch with
                // 100% correct prediction rate exceeds the cost of the body itself
                if (LLVM_UNLIKELY(isTrivial(br->getBody()))) {
                    stmt = flatten(br);
                    continue;
                }
            }

        } else {

            // demote any uses of any Var whose value is in scope
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
                Statement * const prior = stmt->getPrevNode();
                stmt->replaceWith(folded);
                // Since folding a statement may result in inserting new
                // sub-statements or transform the statement in such a
                // manner that we must re-evaluate it, check the statement
                // after the prior to ensure that we traverse the AST in
                // sequential order.
                stmt = prior ? prior->getNextNode() : block->front();
                continue;
            }

            // By recording which statements have already been seen, we can detect the redundant statements
            // as any having the same type and operands. If so, we can replace its users with the prior statement.
            // and erase this statement from the AST
            PabloAST * replacement = nullptr;
            bool added = false;
            std::tie(replacement, added) = expressions.findOrAdd(stmt);
            if (!added) {
                stmt = stmt->replaceWith(replacement);
                continue;
            }

            // Attempt to extend our set of trivially non-zero statements.
            if (isa<Or>(stmt)) {
                for (unsigned i = 0; i < stmt->getNumOperands(); ++i) {
                    if (LLVM_UNLIKELY(isNonZero(stmt->getOperand(i)))) {
                        mNonZero.push_back(stmt);
                        break;
                    }
                }
            } else if (isa<Advance>(stmt)) {
                const Advance * const adv = cast<Advance>(stmt);
                if (LLVM_LIKELY(adv->getAmount() < (adv->getType()->getPrimitiveSizeInBits() / 2))) {
                    if (LLVM_UNLIKELY(isNonZero(adv->getExpression()))) {
                        mNonZero.push_back(adv);
                    }
                }
            }
        }

        stmt = stmt->getNextNode();
    }

    // Erase any local non-zero entries that were discovered while processing this scope
    mNonZero.erase(mNonZero.begin() + baseNonZeroEntries, mNonZero.end());

    assert (mInScope.back() == block);
    mInScope.pop_back();

    // If this block has a branch statement leading into it, we can verify whether an escaped value
    // was updated within this block and update the preceeding block's variable state appropriately.

    Branch * const br = block->getBranch();
    if (LLVM_LIKELY(br != nullptr)) {

        // When removing identical escaped values, we have to consider that the identical Vars could
        // be assigned new differing values later in the outer body. Thus instead of replacing them
        // directly, we map future uses of the duplicate Var to the initial one. The DCE pass will
        // later mark any Assign statement as dead if the Var is never read.

        const auto escaped = br->getEscaped();
        const auto n = escaped.size();

        SmallVector<PabloAST *, 16> variable(n);
        SmallVector<PabloAST *, 16> incoming(n);
        SmallVector<PabloAST *, 16> outgoing(n);
        for (unsigned i = 0; i < n; ++i) {
            variable[i] = escaped[i];
            incoming[i] = vt->get(variable[i]);
            outgoing[i] = variables.get(variable[i]);
            if (LLVM_UNLIKELY(incoming[i] == outgoing[i])) {
                variable[i] = incoming[i];
            } else {
                for (unsigned j = 0; j < i; ++j) {
                    if (LLVM_UNLIKELY((outgoing[j] == outgoing[i]) && (incoming[j] == incoming[i]))) {
                        variable[i] = variable[j];
                        break;
                    }
                }
            }
            vt->put(escaped[i], variable[i]);
        }

    }

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief fold
 ** ------------------------------------------------------------------------------------------------------------- */
static PabloAST * triviallyFold(Statement * stmt, PabloBlock * const block) {
    if (isa<Not>(stmt)) {
        PabloAST * value = cast<Not>(stmt)->getExpr();
        if (LLVM_UNLIKELY(isa<Not>(value))) {
            return cast<Not>(value)->getExpr(); // ¬¬A ⇔ A
        } else if (LLVM_UNLIKELY(isa<Zeroes>(value))) {
            return block->createOnes(stmt->getType()); // ¬0 ⇔ 1
        }  else if (LLVM_UNLIKELY(isa<Ones>(value))) {
            return block->createZeroes(stmt->getType()); // ¬1 ⇔ 0
        }
    } else if (isa<And>(stmt) || isa<Or>(stmt)) {
        PabloAST * op[2];
        op[0] = stmt->getOperand(0);
        op[1] = stmt->getOperand(1);
        for (unsigned i = 0; i < 2; ++i) {
            if (const Not * const n = dyn_cast<Not>(op[i])) {
                if (LLVM_UNLIKELY(n->getExpr() == op[1 - i])) {
                    if (isa<And>(stmt)) {
                        return block->createZeroes(stmt->getType());
                    } else {
                        return block->createOnes(stmt->getType());
                    }
                }
            } else if (LLVM_UNLIKELY(isa<Zeroes>(op[i]) || isa<Ones>(op[i]))) {
                if (isa<And>(stmt) ^ isa<Zeroes>(op[i])) {
                    return op[1 - i];
                } else {
                    return op[i];
                }
            }
        }
        if (LLVM_UNLIKELY(op[0] == op[1])) {
            return op[0];
        } else {
            if (op[1] < op[0]) {
                stmt->setOperand(0, op[1]);
                stmt->setOperand(1, op[0]);
            }
            return nullptr;
        }
    } else if (isa<Xor>(stmt)) {

        PabloAST * op[2];
        op[0] = stmt->getOperand(0);
        op[1] = stmt->getOperand(1);
        if (LLVM_UNLIKELY(op[0] == op[1])) {
            return block->createZeroes(stmt->getType());
        }
        bool negated = false;
        PabloAST * expr = nullptr;
        bool unchanged = true;
        for (unsigned i = 0; i < 2; ++i) {
            if (isa<Not>(op[i])) {
                negated ^= true;
                op[i] = cast<Not>(op[i])->getExpr();
                unchanged = false;
            } else if (LLVM_UNLIKELY(isa<Zeroes>(op[i]) || isa<Ones>(op[i]))) {
                negated ^= isa<Ones>(op[i]);
                expr = op[1 - i];
                unchanged = false;
            }
        }

        if (LLVM_LIKELY(unchanged)) {
            return nullptr;
        }

        block->setInsertPoint(stmt);

        if (LLVM_LIKELY(expr == nullptr)) {
            if (LLVM_UNLIKELY(op[0] == op[1])) {
                if (LLVM_LIKELY(negated)) {
                    return block->createOnes(stmt->getType());
                } else {
                    return block->createZeroes(stmt->getType());
                }
            }
            if (op[1] < op[0]) {
                std::swap(op[0], op[1]);
            }
            expr = block->createXor(op[0], op[1]);
        }

        if (LLVM_UNLIKELY(negated)) {
            Not * const negated = block->createNot(expr);
            PabloAST * const folded = triviallyFold(negated, block);
            expr = folded ? folded : negated;
        }
        return expr;
    } else if (isa<Advance>(stmt)) {
        Advance * const adv = cast<Advance>(stmt);
        if (LLVM_UNLIKELY(isa<Zeroes>(adv->getExpression()) || adv->getAmount() == 0)) {
            return adv->getExpression();
        }
    } else if (isa<ScanThru>(stmt)) {
        ScanThru * const st = cast<ScanThru>(stmt);
        if (LLVM_UNLIKELY(isa<Zeroes>(st->getScanFrom()) || isa<Zeroes>(st->getScanThru()))) {
            return st->getScanFrom();
        } else if (LLVM_UNLIKELY(isa<Ones>(st->getScanThru()))) {
            block->setInsertPoint(stmt->getPrevNode());
            return block->createZeroes(stmt->getType());
        } else if (LLVM_UNLIKELY(isa<ScanThru>(st->getScanFrom()))) {
            ScanThru * const nested = cast<ScanThru>(st->getScanFrom());
            if (LLVM_UNLIKELY(st->getScanThru() == nested->getScanThru())) {
                return nested;
            }
        }
    } else if (isa<MatchStar>(stmt)) {
        MatchStar * const mstar = cast<MatchStar>(stmt);
        if (LLVM_UNLIKELY(isa<Zeroes>(mstar->getMarker()) || isa<Zeroes>(mstar->getCharClass()))) {
            return mstar->getMarker();
        } else if (LLVM_UNLIKELY(isa<Ones>(mstar->getMarker()))) {
            block->setInsertPoint(stmt->getPrevNode());
            return block->createOnes(stmt->getType());
        }
    } else if (isa<Lookahead>(stmt)) {
        Lookahead * const la = cast<Lookahead>(stmt);
        if (LLVM_UNLIKELY(isa<Zeroes>(la->getExpression()) || la->getAmount() == 0)) {
            return la->getExpression();
        }
    } else if (LLVM_UNLIKELY(isa<Sel>(stmt))) {
        Sel * const sel = cast<Sel>(stmt);
        if (LLVM_UNLIKELY(isa<Zeroes>(sel->getCondition()))) {
            return sel->getFalseExpr();
        }
        if (LLVM_UNLIKELY(isa<Ones>(sel->getCondition()))) {
            return sel->getTrueExpr();
        }
        if (LLVM_UNLIKELY(isa<Zeroes>(sel->getTrueExpr()))) {
            block->setInsertPoint(stmt->getPrevNode());
            PabloAST * const negCond = triviallyFold(block->createNot(sel->getCondition()), block);
            return triviallyFold(block->createAnd(sel->getFalseExpr(), negCond), block);
        }
        if (LLVM_UNLIKELY(isa<Ones>(sel->getTrueExpr()))) {
            block->setInsertPoint(stmt->getPrevNode());
            return triviallyFold(block->createOr(sel->getCondition(), sel->getFalseExpr()), block);
        }
        if (LLVM_UNLIKELY(isa<Zeroes>(sel->getFalseExpr()))) {
            block->setInsertPoint(stmt->getPrevNode());
            return triviallyFold(block->createAnd(sel->getCondition(), sel->getTrueExpr()), block);
        }
        if (LLVM_UNLIKELY(isa<Ones>(sel->getFalseExpr()))) {
            block->setInsertPoint(stmt->getPrevNode());
            PabloAST * const negCond = triviallyFold(block->createNot(sel->getCondition()), block);
            return triviallyFold(block->createOr(sel->getTrueExpr(), negCond), block);
        }
    } else if (LLVM_UNLIKELY(isa<Add>(stmt) || isa<Subtract>(stmt))) {
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
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isTrivial
 *
 * If this inner block is composed of only Boolean logic and Assign statements and there are fewer than 3
 * statements, just add the statements in the inner block to the current block
 ** ------------------------------------------------------------------------------------------------------------- */
static bool isTrivial(const PabloBlock * const block) {
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
static Statement * flatten(Branch * const br) {
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
 * @brief isNonZero
 ** ------------------------------------------------------------------------------------------------------------- */
bool isNonZero(const PabloAST * const expr) const {
    return isa<Ones>(expr) || std::find(mNonZero.begin(), mNonZero.end(), expr) != mNonZero.end();
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
            ScanThru * const outer = cast<ScanThru>(stmt);
            if (LLVM_UNLIKELY(isa<Advance>(outer->getScanFrom()))) {
                // Replace ScanThru(Advance(x,n),y) with ScanThru(Advance(x, n - 1), Advance(x, n - 1) | y), where Advance(x, 0) = x
                Advance * const inner = cast<Advance>(outer->getScanFrom());
                if (LLVM_UNLIKELY(inner->getNumUses() == 1)) {
                    PabloAST * stream = inner->getExpression();
                    block->setInsertPoint(stmt);
                    if (LLVM_UNLIKELY(inner->getAmount() != 1)) {
                        stream = block->createAdvance(stream, block->getInteger(inner->getAmount() - 1));
                    }
                    stmt = outer->replaceWith(block->createAdvanceThenScanThru(stream, outer->getScanThru()));
                    inner->eraseFromParent(false);
                    continue;
                }
//            } else if (LLVM_UNLIKELY(isa<ScanThru>(outer->getScanFrom()))) {
//                // Replace ScanThru(ScanThru(x, y), z) with ScanThru(x, y | z)
//                // TODO: this transformation is valid if and only if there can be no instance of ...yzy... in the (y | z) stream
//                // but that degree of reasoning is too complex to perform linearly here
//                ScanThru * const inner = cast<ScanThru>(outer->getScanFrom());
//                block->setInsertPoint(stmt);
//                ScanThru * const scanThru = block->createScanThru(inner->getScanFrom(), block->createOr(inner->getScanThru(), outer->getScanThru()));
//                stmt->replaceWith(scanThru);
//                stmt = scanThru;
//                continue;
            } else if (LLVM_UNLIKELY(isa<And>(outer->getScanFrom()))) {
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
 * @brief deadCodeElimination
 ** ------------------------------------------------------------------------------------------------------------- */
void deadCodeElimination(PabloBlock * const block) {

    flat_set<PabloAST *> written;

    for (Statement * stmt = block->back(), * prior; stmt; stmt = prior) {
        prior = stmt->getPrevNode();
        if (LLVM_UNLIKELY(stmt->getNumUses() == 0)) {
            if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
                written.clear();
                deadCodeElimination(cast<Branch>(stmt)->getBody());
            } else if (LLVM_UNLIKELY(isa<Assign>(stmt))) {
                // An Assign statement is locally dead whenever its variable is not read
                // before being reassigned a value.
                PabloAST * var = cast<Assign>(stmt)->getVariable();
                if (LLVM_UNLIKELY(!written.insert(var).second)) {
                    stmt->eraseFromParent();
                }
            } else {
                stmt->eraseFromParent();
            }
        }
    }
}

std::vector<const PabloAST *>       mNonZero;
std::vector<const PabloBlock *>     mInScope;

};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool Simplifier::optimize(PabloKernel * kernel) {
    PassContainer pc;
    pc.run(kernel);
    #ifndef NDEBUG
    PabloVerifier::verify(kernel, "post-simplification");
    #endif
    return true;
}

}
