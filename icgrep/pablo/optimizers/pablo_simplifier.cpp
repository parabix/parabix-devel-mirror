#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/codegenstate.h>
#include <pablo/expression_map.hpp>
#include <pablo/function.h>
#include <pablo/printer_pablos.h>
#include <pablo/analysis/pabloverifier.hpp>
#include <boost/container/flat_set.hpp>

#include <pablo/printer_pablos.h>
#include <iostream>

namespace pablo {

template <typename Type>
using SmallSet = boost::container::flat_set<Type>;

using TypeId = PabloAST::ClassTypeId;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief fold
 *
 * Note: if folding alters this variadic without making any other modification to the AST, it will return null as
 * if no change was made.
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * Simplifier::fold(Variadic * var, PabloBlock * const block) {

    assert (var);

    bool negated = false;
    if (LLVM_UNLIKELY(isa<Xor>(var))) {
        for (unsigned i = 0; i != var->getNumOperands(); ++i) {
            if (isa<Not>(var->getOperand(i))) {
                // (A ⊕ ¬B) = (A ⊕ (B ⊕ 1)) = ¬(A ⊕ B)
                var->setOperand(i, cast<Not>(var->getOperand(i))->getOperand(0));
                negated = !negated;
            }
        }
    }

    // Ensure all operands of a reassociatiable function are consistently ordered.
    std::sort(var->begin(), var->end());

    // Apply the idempotence law to any And and Or statement and the identity law to any Xor
    for (int i = var->getNumOperands() - 1; i > 0; --i) {
        if (LLVM_UNLIKELY(equals(var->getOperand(i), var->getOperand(i - 1)))) {
            var->removeOperand(i);
            if (LLVM_UNLIKELY(isa<Xor>(var))) {
                var->removeOperand(--i);
            }
        }
    }

    // Apply the annihilator and identity laws
    for (unsigned i = 0; i != var->getNumOperands(); ) {
        if (LLVM_UNLIKELY(isa<Zeroes>(var->getOperand(i)))) {
            if (LLVM_UNLIKELY(isa<And>(var))) {
                return PabloBlock::createZeroes();
            }
            var->removeOperand(i);
            continue;
        } else if (LLVM_UNLIKELY(isa<Ones>(var->getOperand(i)))) {
            if (LLVM_UNLIKELY(isa<Or>(var))) {
                return PabloBlock::createOnes();
            } else if (LLVM_UNLIKELY(isa<Xor>(var))) {
                negated = !negated;
            }
            var->removeOperand(i);
            continue;
        }
        ++i;
    }

    PabloAST * replacement = nullptr;

    if (LLVM_LIKELY(isa<And>(var) || isa<Or>(var))) {
        // Apply an implicit distribution + identity law whenever possible
        //    (P ∧ Q) ∨ (P ∧ ¬Q) = P ∨ (Q ∧ ¬Q) ⇔ (P ∨ Q) ∧ (P ∨ ¬Q) = P ∧ (Q ∨ ¬Q) ⇔ P
        const TypeId typeId = isa<And>(var) ? TypeId::Or : TypeId::And;
        for (unsigned i = 1; i < var->getNumOperands(); ++i) {
            if (var->getOperand(i)->getClassTypeId() == typeId) {
                Variadic * const Vi = cast<Variadic>(var->getOperand(i));
                assert (std::is_sorted(Vi->begin(), Vi->end()));
                for (unsigned j = 0; j < i; ++j) {
                    assert (var->getOperand(i) == Vi);
                    if (var->getOperand(j)->getClassTypeId() == typeId) {
                        Variadic * const Vj = cast<Variadic>(var->getOperand(j));
                        assert (std::is_sorted(Vj->begin(), Vj->end()));
                        if (Vi->getNumOperands() == Vj->getNumOperands()) {
                            // If vi and vj differ by precisely one operand, say di and dj, and di ⇔ ¬dj, we can apply this rule.
                            unsigned vi = 0, vj = 0;
                            const unsigned operands = Vi->getNumOperands();
                            unsigned di = operands - 1, dj = operands - 1;
                            bool differsByOne = true;
                            while (vi < operands && vj < operands) {
                                if (Vi->getOperand(vi) < Vj->getOperand(vj)) {
                                    if (LLVM_UNLIKELY(di != (operands - 1))) { // <- we want the branch predictor to fail only once
                                        differsByOne = false;
                                        break;
                                    }
                                    di = vi++;
                                } else if (Vj->getOperand(vj) < Vi->getOperand(vi)) {
                                    if (LLVM_UNLIKELY(dj != (operands - 1))) {
                                        differsByOne = false;
                                        break;
                                    }
                                    dj = vj++;
                                } else {
                                    ++vi;
                                    ++vj;
                                }
                            }
                            if (LLVM_UNLIKELY(differsByOne)) {
                                assert (di < operands && dj < operands);
                                assert ("Found an equivalent set of operations that was not deduced earlier!" && (!equals(Vi, Vj)));
                                // test if di ⇔ ¬dj
                                bool apply = false;
                                if (isa<Not>(Vi->getOperand(di))) {
                                    apply = cast<Not>(Vi->getOperand(di))->getOperand(0) == Vj->getOperand(dj);
                                } else if (isa<Not>(Vj->getOperand(dj))) {
                                    apply = cast<Not>(Vj->getOperand(dj))->getOperand(0) == Vi->getOperand(di);
                                }
                                if (LLVM_UNLIKELY(apply)) {
                                    // Although we can apply this transformation, we have a potential problem. If P is not a "literal", we
                                    // cannot optimize var without creating a new And/Or statement. However, the redundancy elimination
                                    // pass will miss this new statement unless we mark "var" as its own replacement. We'll end up checking
                                    // "var" again but termination is still guaranteed once none of the new statements can be replaced by
                                    // prior statements in the AST.
                                    PabloAST * expr = nullptr;
                                    if (operands == 2) {
                                        expr = Vi->getOperand(1 ^ di);
                                        if (LLVM_LIKELY(var->getNumOperands() == 2)) {
                                            return expr;
                                        }
                                    } else { // if (operands > 2) {
                                        assert (operands > 2);
                                        block->setInsertPoint(var->getPrevNode());
                                        if (typeId == TypeId::And) {
                                            expr = block->createAnd(operands - 1);
                                        } else { // if (typeId == TypeId::Or) {
                                            expr = block->createOr(operands - 1);
                                        }
                                        for (unsigned k = 0; k != di; ++k) {
                                            cast<Variadic>(expr)->addOperand(Vi->getOperand(k));
                                        }
                                        for (unsigned k = di + 1; k < operands; ++k) {
                                            cast<Variadic>(expr)->addOperand(Vi->getOperand(k));
                                        }
                                        replacement = var;
                                    }
                                    var->removeOperand(i);
                                    var->removeOperand(j);
                                    bool unique = true;
                                    for (unsigned k = 0; k != var->getNumOperands(); ++k) {
                                        if (LLVM_UNLIKELY(equals(var->getOperand(k), expr))) {
                                            unique = false;
                                            break;
                                        }
                                    }
                                    if (LLVM_LIKELY(unique)) {
                                        var->addOperand(expr);
                                    }
                                    i -= 2;
                                    break; // out of for j = 0 to i - 1
                                }
                            }
                        }
                    }
                }
            }
        }

        // Apply the absorption law whenever possible
        //   P ∨ (P ∧ Q) ⇔ P ∧ (P ∨ Q) ⇔ P
        for (unsigned i = 1; i < var->getNumOperands(); ++i) {
            PabloAST * const op = var->getOperand(i);
            if (op->getClassTypeId() == typeId) {
                Variadic * const vi = cast<Variadic>(op);
                assert (std::is_sorted(vi->begin(), vi->end()));
                for (unsigned j = 0; j < i; ++j) {
                    assert (var->getOperand(i) == vi);
                    if (var->getOperand(j)->getClassTypeId() == typeId) {
                        Variadic * const vj = cast<Variadic>(var->getOperand(j));
                        assert (std::is_sorted(vj->begin(), vj->end()));
                        if (vi->getNumOperands() < vj->getNumOperands()) {
                            if (LLVM_UNLIKELY(std::includes(vi->begin(), vi->end(), vj->begin(), vj->end()))) {
                                var->removeOperand(i--);
                                break;
                            }
                        } else { // if (vi->getNumOperands() >= vj->getNumOperands()) {
                            if (LLVM_UNLIKELY(std::includes(vj->begin(), vj->end(), vi->begin(), vi->end()))) {
                                var->removeOperand(j--);
                                --i;
                            }
                        }
                    }
                }
            } else { // treat the operand as a literal
                for (unsigned j = 0; j < var->getNumOperands(); ) {
                    if (var->getOperand(j)->getClassTypeId() == typeId) {
                        Variadic * const vj = cast<Variadic>(var->getOperand(j));
                        assert (std::is_sorted(vj->begin(), vj->end()));
                        if (LLVM_UNLIKELY(std::binary_search(vj->begin(), vj->end(), op))) {
                            var->removeOperand(j);
                            continue;
                        }
                    }
                    ++j;
                }
            }
        }

        // Apply the complementation law whenever possible.
        for (unsigned i = 0; i < var->getNumOperands(); ++i) {
            if (isa<Not>(var->getOperand(i))) {
                const PabloAST * const negated = cast<Not>(var->getOperand(i))->getOperand(0);
                for (unsigned j = 0; j != var->getNumOperands(); ++j) {
                    if (LLVM_UNLIKELY(var->getOperand(j) == negated)) {
                        if (isa<And>(var)) { // (A ∧ ¬A) ∧ B ⇔ 0 for any B
                            return PabloBlock::createZeroes();
                        } else { // if (isa<Or>(var)) { // (A ∨ ¬A) ∨ B ⇔ 1 for any B
                            return PabloBlock::createOnes();
                        }
                    }
                }
            }
        }

    }

    if (LLVM_UNLIKELY(var->getNumOperands() < 2)) {
        if (LLVM_UNLIKELY(var->getNumOperands() == 0)) {
            return PabloBlock::createZeroes();
        }
        replacement = var->getOperand(0);
    }
    if (LLVM_UNLIKELY(negated)) {
        assert (isa<Xor>(var));
        block->setInsertPoint(var);
        replacement = block->createNot(replacement ? replacement : var);
    }
    return replacement;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief fold
 ** ------------------------------------------------------------------------------------------------------------- */
inline PabloAST * Simplifier::fold(Statement * stmt, PabloBlock * const block) {
    if (isa<Variadic>(stmt)) {
        return fold(cast<Variadic>(stmt), block);
    } else if (isa<Not>(stmt)) {
        PabloAST * value = stmt->getOperand(0);
        if (LLVM_UNLIKELY(isa<Not>(value))) {
            return cast<Not>(value)->getOperand(0); // ¬¬A ⇔ A
        } else if (LLVM_UNLIKELY(isa<Zeroes>(value))) {
            return PabloBlock::createOnes(); // ¬0 ⇔ 1
        }  else if (LLVM_UNLIKELY(isa<Ones>(value))) {
            return PabloBlock::createZeroes(); // ¬1 ⇔ 0
        }
    } else if (isa<Advance>(stmt)) {
        if (LLVM_UNLIKELY(isa<Zeroes>(stmt->getOperand(0)))) {
            return PabloBlock::createZeroes();
        }
    } else {
        for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
            if (LLVM_UNLIKELY(isa<Zeroes>(stmt->getOperand(i)))) {
                switch (stmt->getClassTypeId()) {
                    case PabloAST::ClassTypeId::Sel:
                        block->setInsertPoint(stmt->getPrevNode());
                        switch (i) {
                            case 0: return stmt->getOperand(2);
                            case 1: return block->createAnd(block->createNot(stmt->getOperand(0)), stmt->getOperand(2));
                            case 2: return block->createAnd(stmt->getOperand(0), stmt->getOperand(1));
                        }
                    case PabloAST::ClassTypeId::ScanThru:
                    case PabloAST::ClassTypeId::MatchStar:
                        return stmt->getOperand(0);
                    default: break;
                }
            } else if (LLVM_UNLIKELY(isa<Ones>(stmt->getOperand(i)))) {
                switch (stmt->getClassTypeId()) {
                    case PabloAST::ClassTypeId::Sel:
                        block->setInsertPoint(stmt->getPrevNode());
                        switch (i) {
                            case 0: return stmt->getOperand(1);
                            case 1: return block->createOr(stmt->getOperand(0), stmt->getOperand(2));
                            case 2: return block->createOr(block->createNot(stmt->getOperand(0)), stmt->getOperand(1));
                        }
                    case PabloAST::ClassTypeId::ScanThru:
                        if (LLVM_UNLIKELY(i == 1)) {
                            return PabloBlock::createZeroes();
                        }
                        break;
                    case PabloAST::ClassTypeId::MatchStar:
                        if (LLVM_UNLIKELY(i == 0)) {
                            return PabloBlock::createOnes();
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
 * @brief isSuperfluous
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool Simplifier::isSuperfluous(const Assign * const assign) {
    for (const PabloAST * user : assign->users()) {
        if (LLVM_UNLIKELY(isa<PabloFunction>(user) || isa<Next>(user))) {
            return false;
        } else if (isa<If>(user)) {
            if (LLVM_UNLIKELY(cast<If>(user)->getCondition() == assign)) {
                continue;
            } else if (isa<Assign>(assign->getExpression())) {
                continue;
            }
            return false;
        }
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief demoteDefinedVar
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool demoteDefinedVar(const If * ifNode, const Assign * def) {
    // If this value isn't used outside of this scope then there is no reason to allow it to escape it.
    if (!escapes(def)) {
        return true;
    }
    // Similarly if the defined variable is equivalent to the condition, an equivalent value is already available.
    if (ifNode->getCondition() == def->getExpression()) {
        return true;
    }
    // Finally, if the assignment is a constant, it's already known.
    if (isa<Zeroes>(def->getExpression()) || isa<Ones>(def->getExpression()))  {
        return true;
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief replaceReachableUsersOfWith
 ** ------------------------------------------------------------------------------------------------------------- */
inline void replaceReachableUsersOfWith(Statement * stmt, PabloAST * expr) {
    const PabloBlock * const root = stmt->getParent();
    SmallSet<const PabloBlock *> forbidden;
    for (PabloAST * use : stmt->users()) {
        if (LLVM_UNLIKELY(isa<Next>(use))) {
            const PabloBlock * parent = cast<Next>(use)->getParent();
            if (parent != root) {
                forbidden.insert(parent);
            }
        }
    }
    for (PabloAST * use : stmt->users()) {
        if (Statement * user = dyn_cast<Statement>(use)) {
            const PabloBlock * parent = user->getParent();
            while (parent && forbidden.count(parent) == 0) {
                if (LLVM_UNLIKELY(parent == root)) {
                    user->replaceUsesOfWith(stmt, expr);
                    break;
                }
                parent = parent->getParent();
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief discardNestedIfBlock
 *
 * If this inner block is composed of only Boolean logic and Assign statements and there are fewer than 3
 * statements, just add the statements in the inner block to the current block->
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool discardNestedIfBlock(const PabloBlock * const block) {
    unsigned computations = 0;
    for (const Statement * stmt : *block) {
        switch (stmt->getClassTypeId()) {
            case PabloAST::ClassTypeId::And:
            case PabloAST::ClassTypeId::Or:
            case PabloAST::ClassTypeId::Xor:
                if (++computations > 3) {
                    return false;
                }
            case PabloAST::ClassTypeId::Not:
            case PabloAST::ClassTypeId::Assign:
                break;
            default:
                return false;
        }
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief removeIdenticalEscapedValues
 ** ------------------------------------------------------------------------------------------------------------- */
template <class ValueList, class ValueType = typename ValueList::value_type>
inline void removeIdenticalEscapedValues(ValueList & list) {
    std::vector<ValueType> identicalValues;
    for (auto i = list.begin(); i != list.end(); ++i) {
        for (auto j = i + 1; j != list.end(); ++j) {
            if (LLVM_UNLIKELY(equals(*i, *j))) {
                identicalValues.push_back(*j);
            }
        }
        for (ValueType identicalValue : identicalValues) {
            identicalValue->replaceWith(*i);
        }
        identicalValues.clear();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief redundancyElimination
 *
 * Note: Do not recursively delete statements in this function. The ExpressionTable could use deleted statements
 * as replacements. Let the DCE remove the unnecessary statements with the finalized Def-Use information.
 ** ------------------------------------------------------------------------------------------------------------- */
void Simplifier::redundancyElimination(PabloBlock * const block, ExpressionTable * predecessor) {
    ExpressionTable encountered(predecessor);
    Statement * stmt = block->front();

    while (stmt) {

        if (Assign * assign = dyn_cast<Assign>(stmt)) {
            // If we have an Assign whose users do not contain an If or Next node, we can replace its users with
            // the Assign's expression directly.
            if (isSuperfluous(assign)) {
                stmt = assign->replaceWith(assign->getExpression());
                continue;
            }
            // Force the uses of an Assign node that can reach the original expression to use the expression instead.
            replaceReachableUsersOfWith(assign, assign->getExpression());
        } else if (Next * next = dyn_cast<Next>(stmt)) {
            replaceReachableUsersOfWith(next, next->getExpr());
        } else if (If * ifNode = dyn_cast<If>(stmt)) {
            // Test whether we can ever take this branch
            if (LLVM_UNLIKELY(isa<Zeroes>(cast<If>(stmt)->getCondition()))) {
                stmt = stmt->eraseFromParent();
                continue;
            }
            // Test whether all of the defined variables are necessary
            If::DefinedVars & defs = ifNode->getDefined();
            for (auto def = defs.begin(); def != defs.end(); ) {
                if (LLVM_UNLIKELY(demoteDefinedVar(ifNode, *def))) {
                    (*def)->replaceWith((*def)->getExpression());
                    def = ifNode->removeDefined(*def);
                    continue;
                }
                ++def;
            }

            // Process the If body
            redundancyElimination(cast<If>(stmt)->getBody(), &encountered);

            // If we ended up removing all of the defined variables, delete the If node.
            if (LLVM_UNLIKELY(defs.empty())) {
                stmt = stmt->eraseFromParent();
                continue;
            }

            // Otherwise check if we any Assign reports the same value as another. If so, replace all uses of the
            // second with the first. This will simplify future analysis.
            removeIdenticalEscapedValues(ifNode->getDefined());

            // Finally after we've eliminated everything we can from the If body, check whether testing the If
            // condition will meet or exceed the cost of executing the body.
            if (LLVM_UNLIKELY(discardNestedIfBlock(ifNode->getBody()))) {
                Statement * nested = ifNode->getBody()->front();
                while (nested) {
                    Statement * next = nested->removeFromParent();
                    if (isa<Assign>(nested)) {
                        ifNode->removeDefined(cast<Assign>(nested));
                    }
                    nested->insertAfter(stmt);
                    stmt = nested;
                    nested = next;
                }
                stmt = ifNode->eraseFromParent();
                continue;
            }

        } else if (While * whileNode = dyn_cast<While>(stmt)) {
            // Test whether we can ever take this branch
            const PabloAST * initial = cast<While>(stmt)->getCondition();
            if (LLVM_LIKELY(isa<Next>(initial))) {
                initial = cast<Next>(initial)->getInitial();
            }
            if (LLVM_UNLIKELY(isa<Zeroes>(initial))) {
                stmt = stmt->eraseFromParent();
                continue;
            }
            redundancyElimination(whileNode->getBody(), &encountered);
            removeIdenticalEscapedValues(whileNode->getVariants());
            // If the condition's Next state is Zero, we can eliminate the loop after copying the internal
            // statements into the body.
        } else {
            Statement * const prior = stmt->getPrevNode();
            PabloAST * const folded = fold(stmt, block);
            if (folded) {
                // If we determine we can fold this statement, go back to the original prior node of this statement.
                // New statements may have been inserted after it.
                stmt->replaceWith(folded, true);
                stmt = LLVM_LIKELY(prior != nullptr) ? prior->getNextNode() : block->front();
                continue;
            }
            // When we're creating the Pablo program, it's possible to have multiple instances of an "identical"
            // statement. By recording which statements have already been seen, we can detect the redundant statements
            // as any having the same type and operands. If so, we can replace its users with the prior statement.
            // and erase this statement from the AST
            const auto f = encountered.findOrAdd(stmt);
            if (!f.second) {
                stmt = stmt->replaceWith(f.first, true);
                continue;
            }

        }
        stmt = stmt->getNextNode();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief unused
 ** ------------------------------------------------------------------------------------------------------------- */
inline static bool unused(const Statement * const stmt) {
    if (LLVM_UNLIKELY(stmt->getNumUses() == 0)) {
        // TODO: prototypes ought to state whether they have side effects.
        if (LLVM_UNLIKELY(isa<Call>(stmt) && cast<Call>(stmt)->getPrototype()->getNumOfResults() == 0)) {
            return false;
        }
        return true;
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deadCodeElimination
 ** ------------------------------------------------------------------------------------------------------------- */
void Simplifier::deadCodeElimination(PabloBlock * const block) {
    Statement * stmt = block->front();
    while (stmt) {
        if (LLVM_UNLIKELY(isa<If>(stmt))) {
            deadCodeElimination(cast<If>(stmt)->getBody());
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            deadCodeElimination(cast<While>(stmt)->getBody());
        } else if (LLVM_UNLIKELY(unused(stmt))){
            stmt = stmt->eraseFromParent(true);
            continue;
        }
        stmt = stmt->getNextNode();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief strengthReduction
 *
 * Find and replace any Pablo operations with a less expensive equivalent operation whenever possible.
 ** ------------------------------------------------------------------------------------------------------------- */
void Simplifier::strengthReduction(PabloBlock * const block) {
    Statement * stmt = block->front();
    while (stmt) {
        if (isa<If>(stmt)) {
            strengthReduction(cast<If>(stmt)->getBody());
        } else if (isa<While>(stmt)) {
            strengthReduction(cast<While>(stmt)->getBody());
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
            if (LLVM_UNLIKELY(isa<Advance>(scanThru->getOperand(0)))) {
                // Replace a ScanThru(Advance(x,n),y) with an ScanThru(Advance(x, n - 1), Advance(x, n - 1) | y), where Advance(x, 0) = x
                Advance * op = cast<Advance>(stmt->getOperand(0));
                if (LLVM_UNLIKELY(op->getNumUses() == 1)) {
                    block->setInsertPoint(scanThru->getPrevNode());
                    PabloAST * expr = block->createAdvance(op->getOperand(0), op->getAmount() - 1);
                    scanThru->setOperand(0, expr);
                    scanThru->setOperand(1, block->createOr(scanThru->getOperand(1), expr));
                    op->eraseFromParent(false);
                }
            } else if (isa<And>(scanThru->getOperand(0))) {
                // Suppose B is an arbitrary bitstream and A = Advance(B, 1). ScanThru(B ∧ ¬A, B) will leave a marker on the position
                // following the end of any run of 1-bits in B. But this is equivalent to computing A ∧ ¬B since A will have exactly
                // one 1-bit past the end of any run of 1-bits in B.





            }




        }
        stmt = stmt->getNextNode();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool Simplifier::optimize(PabloFunction & function) {
    redundancyElimination(function.getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-eliminate-redundant-code");
    #endif
    strengthReduction(function.getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-strength-reduction");
    #endif
    deadCodeElimination(function.getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-dead-code-elimination");
    #endif
    return true;
}

}
