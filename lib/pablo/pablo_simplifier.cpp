#include <pablo/pablo_simplifier.hpp>

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
#include <toolchain/pablo_toolchain.h>
#ifndef NDEBUG
#include <pablo/pabloverifier.hpp>
#endif
#include <boost/container/flat_set.hpp>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/Support/raw_ostream.h>
#include <util/slab_allocator.h>
#include <pablo/printer_pablos.h>

using namespace boost;
using namespace boost::container;
using namespace llvm;

namespace pablo {

using TypeId = PabloAST::ClassTypeId;
using EscapedVars = Branch::EscapedVars;
using Allocator = SlabAllocator<uint8_t>;
using KeySet = SmallVector<const Var *, 64>;

// #define EXPERIMENTAL

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief VariableTable
 ** ------------------------------------------------------------------------------------------------------------- */
struct VariableTable {

    using MapValueType = flat_map<const Var *, PabloAST *, std::less<const Var *>>::value_type;
    using MapAllocator = ProxyAllocator<MapValueType>;
    using Map = flat_map<const Var *, PabloAST *, std::less<const Var *>, MapAllocator>;

    VariableTable(Allocator & allocator) noexcept
    : mOuter(nullptr)
    , mMap(MapAllocator{allocator}) {

    }

    VariableTable(VariableTable & outer) noexcept
    : mOuter(&outer)
    , mMap(MapAllocator{outer.mMap.get_allocator()}) {

    }

    PabloAST * get(const Var * const var) const noexcept {
        const auto f = mMap.find(var);
        if (f == mMap.end()) {
            return (mOuter) ? mOuter->get(var) : nullptr;
        }
        return f->second;
    }

    void put(const Var * const var, PabloAST * value) noexcept {
        const auto f = mMap.find(var);
        if (LLVM_LIKELY(f == mMap.end())) {
            mMap.emplace(var, value);
        } else {
            f->second = value;
        }
    }

    KeySet keySet() const {
        KeySet keySet;
        const Map & M = mMap;
        unsigned i = 0;
        for (const auto & e : M) {
            keySet[i++] = e.first;
        }
        if (mOuter) {
            KeySet tmp1;
            KeySet tmp2;
            const VariableTable * current = mOuter;
            for (;;) {
                const Map & M = current->mMap;
                tmp1.resize(M.size());
                unsigned i = 0;
                for (const auto & e : M) {
                    tmp1[i++] = e.first;
                }
                tmp2.reserve(tmp1.size() + keySet.size());
                std::set_union(keySet.begin(), keySet.end(), tmp1.begin(), tmp1.end(), std::back_inserter(tmp2));
                keySet.swap(tmp2);
                current = current->mOuter;
                if (current == nullptr) {
                    break;
                }
                tmp1.clear();
                tmp2.clear();
            }
        }
        return keySet;
    }

    void clear() {
        mMap.clear();
    }

private:
    const VariableTable * const mOuter;
    Map                         mMap;
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief LiveVarSet
 ** ------------------------------------------------------------------------------------------------------------- */
struct LiveVarSet : public flat_set<const Var *> {

    LiveVarSet() noexcept
    : mOuter(nullptr) {

    }

    LiveVarSet(const LiveVarSet & outer) noexcept
    : mOuter(&outer) {

    }

    bool contains(const Var * const var) const noexcept {
        const auto f = find(var);
        if (f == end()) {
            return (mOuter) ? mOuter->contains(var) : false;
        }
        return true;
    }

    bool remove(const Var * const var) noexcept {
        const auto f = find(var);
        if (LLVM_LIKELY(f != end())) {
            erase(f);
            return true;
        }
        return false;
    }

    void put(const Var * const var) noexcept {
        insert(var);
    }

private:
    const LiveVarSet * const mOuter;
};


struct PassContainer {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
PassContainer()
: UseTernaryOptimizations(CompileOptionIsSet(EnableTernaryOpt)) {

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief run
 ** ------------------------------------------------------------------------------------------------------------- */
void run(PabloKernel * const kernel) {
    for (;;) {
        redundancyElimination(kernel);
        if (LLVM_LIKELY(deadCodeElimination(kernel))) {
            break;
        }
    }
    strengthReduction(kernel->getEntryScope());
}

protected:

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief redundancyElimination
 *
 * Note: Do not recursively delete statements in this function. The ExpressionTable could use deleted statements
 * as replacements. Let the DCE remove the unnecessary statements with the finalized Def-Use information.
 ** ------------------------------------------------------------------------------------------------------------- */
void redundancyElimination(PabloKernel * const kernel) {
    ExpressionTable expressions;
    VariableTable variables(expressions.get_allocator());
    const auto n = kernel->getNumOfInputs();
    for (unsigned i = 0; i < n; ++i) {
        Var * const input = kernel->getInput(i);
        variables.put(input, input);
    }
    PabloBlock * const entryScope = kernel->getEntryScope();
    mSideEffecting.clear();
    assert (mNonZero.empty());
    redundancyElimination(entryScope, expressions, variables);
    assert (mNonZero.empty());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief redundancyElimination
 ** ------------------------------------------------------------------------------------------------------------- */
bool redundancyElimination(PabloBlock * const currentScope, ExpressionTable & expressions, VariableTable & variables) {
    bool hasSideEffects = false;
    const auto baseNonZeroEntries = mNonZero.size();
    Statement * stmt = currentScope->front();
    while (stmt) {

        if (LLVM_UNLIKELY(isa<Assign>(stmt))) {

            if (isRedundantAssign(cast<Assign>(stmt), variables)) {
                stmt = stmt->eraseFromParent();
                continue;
            }

        } else if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
            Statement * const next = evaluateBranch(cast<Branch>(stmt), expressions, variables);
            if (LLVM_UNLIKELY(next != nullptr)) {
                assert (next != stmt);
                assert (next->getParent() == currentScope);
                stmt = next;
                continue;
            }
        } else {

            for (unsigned i = 0; i < stmt->getNumOperands(); ++i) {
                PabloAST * const op = stmt->getOperand(i);
                // demote any uses of any Var whose value is in scope
                if (LLVM_UNLIKELY(isa<Var>(op))) {
                    PabloAST * const var = variables.get(cast<Var>(op));
                    if (LLVM_UNLIKELY(var != nullptr && var != op)) {
                        stmt->setOperand(i, var);
                    }
                }
            }

            PabloAST * const folded = triviallyFold(stmt, currentScope);
            if (folded) {
                assert (folded != stmt);
                Statement * const prior = stmt->getPrevNode();
                stmt->replaceWith(folded);
                // Since folding a statement may result in inserting new
                // sub-statements or transform the statement in such a
                // manner that we must re-evaluate it, check the statement
                // after the prior to ensure that we traverse the AST in
                // sequential order.
                stmt = prior ? prior->getNextNode() : currentScope->front();
                continue;
            }

            // By recording which statements have already been seen, we can detect the redundant statements
            // as any having the same type and operands. If so, we can replace its users with the prior statement.
            // and erase this statement from the AST
            PabloAST * replacement = nullptr;
            bool added = false;
            std::tie(replacement, added) = expressions.findOrAdd(stmt);
            if (!added) {
                assert (replacement != stmt);
                stmt = stmt->replaceWith(replacement);
                continue;
            }

            if (LLVM_UNLIKELY(stmt->isSideEffecting())) {
                hasSideEffects = true;
                mSideEffecting.insert(stmt);
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
    return hasSideEffects;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief evaluateBranch
 ** ------------------------------------------------------------------------------------------------------------- */
Statement * evaluateBranch(Branch * const br, ExpressionTable & expressions, VariableTable & variables) {

    // Construct the nested variable table
    VariableTable nestedVariables(variables);
    EscapedVars escaped = br->getEscaped();
    std::sort(escaped.begin(), escaped.end());

    if (LLVM_UNLIKELY(isa<While>(br))) {
        for (Var * const var : escaped) {
            nestedVariables.put(var, var);
        }
    }

    PabloAST * cond = br->getCondition();
    auto subsituteCondValue = [&]() {
        if (isa<Var>(cond)) {
            PabloAST * const value = nestedVariables.get(cast<Var>(cond));
            if (value && cond != value) {
                cond = value;
                br->setCondition(cond);
            }
        }
    };

    subsituteCondValue();

    // Test whether we can ever take this branch
    if (LLVM_UNLIKELY(isa<Zeroes>(cond))) {
        return br->eraseFromParent();
    }

    // If we're guaranteed to take this branch, flatten it.
    if (LLVM_UNLIKELY(isNonZero(cond))) {
        if (LLVM_LIKELY(isa<If>(br))) {
            return flatten(br);
        } else {
            report_fatal_error("While condition is guaranteed to be non-zero");
        }
    }

    // Process the Branch body
    ExpressionTable nestedExpressions(&expressions);

    bool hasSideEffects = false;

    auto processBranchBody = [&]() {
        mNonZero.push_back(cond); // mark the cond as non-zero prior to processing the inner scope.
        hasSideEffects = redundancyElimination(br->getBody(), nestedExpressions, nestedVariables);
        assert (mNonZero.back() == cond);
        mNonZero.pop_back();
    };

    processBranchBody();

    // If this block has a branch statement leading into it, we can verify whether an escaped value
    // was updated within this block and update the preceeding block's variable state appropriately.
    if (phiEscapedVars(br, escaped, nestedVariables, variables)) {
        // If we modified the escaped vars, reevaluate the branch.
        nestedExpressions.clear();
        subsituteCondValue();
        processBranchBody();
    }

    // Check whether the cost of testing the condition and taking the branch with
    // 100% correct prediction rate exceeds the cost of the body itself
    if (LLVM_LIKELY(isa<If>(br)) && LLVM_UNLIKELY(isTrivial(br->getBody()))) {
        return flatten(br);
    }
    if (LLVM_UNLIKELY(hasSideEffects)) {
        mSideEffecting.insert(br);
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief phiEscapedVars
 ** ------------------------------------------------------------------------------------------------------------- */
static bool phiEscapedVars(Branch * const br, const EscapedVars & escaped, VariableTable & inner, VariableTable & outer) {

    const auto n = escaped.size();

    SmallVector<PabloAST *, 16> incoming(n);
    SmallVector<PabloAST *, 16> outgoing(n);
    SmallVector<PabloAST *, 16> initial(n);
    SmallVector<PabloAST *, 16> final(n);
    bool modified = false;

    for (unsigned i = 0; i < n; ++i) {
        Var * const var = escaped[i];
        incoming[i] = outer.get(var);
        outgoing[i] = inner.get(var);
        initial[i] = var;
        final[i] = var;
    }

    // CASE 1:

    // Detect when Var a is assigned the same value within the branch as its original value
    // and replace a with the value.

    //             a = x                     a = x
    //             branch a:        =>       branch x:
    //               a = x                      ...
    //             y = a op z                y = *x* op z

    for (unsigned i = 0; i < n; ++i) {
        if (LLVM_UNLIKELY(incoming[i] == outgoing[i])) {
            initial[i] = incoming[i];
            final[i] = incoming[i];
            modified = true;
        }
    }

    // CASE 2:

    // Detect when two Vars, a and b, are assigned identical values and replace future uses
    // of b with a -- assuming a is not reassigned a different variable outside of the scope.
    // Dead code elimination will remove the redundant assignments to b.

    //             a = x                     a = x
    //             b = x                     ...
    //             branch b:                 branch a:
    //                a = y         =>          a = y
    //                b = y                     ...

    for (unsigned i = 0; i < n; ++i) {
        for (unsigned j = 0; j < i; ++j) {
            if (LLVM_UNLIKELY((outgoing[i] == outgoing[j]) && (incoming[i] == incoming[j]))) {
                initial[i] = escaped[j];
                final[i] = outgoing[j];
                modified = true;
                break;
            }
        }
    }

    #ifdef EXPERIMENTAL
    // CASE 3:

    // TODO: there is one final simplification to vars that is within the scope of this pass
    // but is non-trivial to reason out here. If we have:

    //             a = x                            a = x
    //             branch a:                        branch a:
    //                b = y                            ...
    //                ...                              b = a
    //                ...                              a = y
    //                branch b:                        branch *a*:
    //                   ...                              ...
    //                   f1 = op a, e1       =>            f1 = op *b*, e1
    //                   f2 = op b, e2                     f2 = op *a*, e2
    //                   ...                              ...
    //                   a = r                            *b* = r
    //                   b = z                            *a* = z
    //                ...                              b = a
    //                a = b                            ...

    // We could reassign then "swap" a and b within branch c2. This accommodates the fact
    // that "a" may be used -- or potentially reassigned --- within the c2 branch whilst
    // ensuring uses of "a" refer to the original value of "a" (presuming no reassignments
    // of "a" occur within branch c2).

    // The problem here is we have not yet seen the final assignment of "a" to "b" and we
    // must be certain that "b" is not reassigned some other value before it occurs and
    // that there are no uses of "a" dominated by branch b "a" prior to the assignment.

    const auto reassignments = identifyReassignments(br, escaped, outer);

    if (LLVM_UNLIKELY(!reassignments.empty())) {
        PabloBlock * const scope = br->getParent();
        PabloBlock * const nested = br->getBody();

        Statement * before = br->getPrevNode();
        Statement * after = br->getPrevNode();

        nested->setInsertPoint(nested->back());
        for (Assign * const reassign : reassignments) {
            Var * const b = cast<Var>(reassign->getValue());
            const auto f = std::lower_bound(escaped.begin(), escaped.end(), b);
            assert (f != escaped.end() && *f == b);
            const auto i = std::distance(escaped.begin(), f);

            scope->setInsertPoint(before);

            Var * const a = reassign->getVariable();
            scope->createAssign(b, a);
            before = scope->createAssign(a, initial[i]);


            nested->createAssign(a, final[i]);


            scope->setInsertPoint(after);
            after = scope->createAssign(b, a);

            inner.put(b, a);
            inner.put(a, b);

            initial[i] = a;
            final[i] = a;

            modified = true;
        }
    }
    #endif

    for (unsigned i = 0; i < n; ++i) {
        outer.put(escaped[i], final[i]);
    }

    if (modified) {
        inner.clear();
        for (unsigned i = 0; i < n; ++i) {
            inner.put(escaped[i], initial[i]);
        }
    }

    return modified;
}

#ifdef EXPERIMENTAL

using Reassignments = SmallVector<Assign *, 4>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyReassignments
 ** ------------------------------------------------------------------------------------------------------------- */
static Reassignments identifyReassignments(Branch * const br, const EscapedVars & escaped, const VariableTable & outer) {
    Reassignments reassignments;
    for (const Var * var : getNonEscapedLiveVars(escaped, outer)) {
        for (const PabloAST * const user : var->users()) {
            if (isa<Assign>(user)) {
                // is the assignment to a Var in the escaped list?
                if (LLVM_UNLIKELY(isEscapedVar(cast<Assign>(user)->getValue(), escaped))) {
                    // We may find several such assignments, potentially to the same Var since DCE may not have been
                    // run yet on this program. However, we are only interested in the first such assignment that
                    // post-dominates the branch and whether it happens to be an escaped Var. So once we determine
                    // it is plausible that we have one, find the first post-dominating assignment to "var". If there
                    // are no other uses of "var" prior to it, add it to our reassignment list.
                    Assign * const assign = getSafePostDominatingAssignment(var, br);
                    // Then add it to our list of reassignments if it happens to be one.
                    if (LLVM_LIKELY(assign && (assign == user || isEscapedVar(assign->getValue(), escaped)))) {
                        reassignments.emplace_back(assign);
                    }
                    break;
                }
            }
        }
    }
    return reassignments;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isEscapedVar
 ** ------------------------------------------------------------------------------------------------------------- */
static bool isEscapedVar(const PabloAST * const expr, const EscapedVars & escaped) {
    return isa<Var>(expr) && std::binary_search(escaped.begin(), escaped.end(), cast<Var>(expr));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNonEscapedLiveVars
 ** ------------------------------------------------------------------------------------------------------------- */
static KeySet getNonEscapedLiveVars(const EscapedVars & escaped, const VariableTable & outer) {
    const auto full = outer.keySet();
    KeySet potential;
    assert (std::is_sorted(escaped.begin(), escaped.end());
    std::set_difference(full.begin(), full.end(), escaped.begin(), escaped.end(), std::back_inserter(potential));
    return potential;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getSafePostDominatingAssignment
 ** ------------------------------------------------------------------------------------------------------------- */
static Assign * getSafePostDominatingAssignment(const Var * const var, Statement * prior) {
    assert (prior);
    for (;;) {
        Statement * const stmt = prior->getNextNode();
        if (LLVM_UNLIKELY(stmt == nullptr)) {
            prior = prior->getParent()->getBranch();
            if (LLVM_UNLIKELY(prior == nullptr)) {
                return nullptr;
            }
        } else {
            if (LLVM_UNLIKELY(isa<Assign>(stmt))) {
                if (cast<Assign>(stmt)->getVariable() == var) {
                    return cast<Assign>(stmt);
                }
                if (cast<Assign>(stmt)->getValue() == var) {
                    return nullptr;
                }
            } else if (LLVM_UNLIKELY(isa<Branch>(stmt))) {



            } else {
                const auto n = stmt->getNumOfOperands();
                for (unsigned i = 0; i != n; ++i) {
                    if (LLVM_UNLIKELY(var == stmt->getOperand(i))) {
                        return nullptr;
                    }
                }
            }
            prior = stmt;
        }
    }
}

#endif

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isRedundantAssign
 ** ------------------------------------------------------------------------------------------------------------- */
static bool isRedundantAssign(Assign * const assign, VariableTable & variables) {
    Var * const var = assign->getVariable();
    PabloAST * const prior = variables.get(var);
    PabloAST * value = assign->getValue();
    while (LLVM_UNLIKELY(isa<Var>(value))) {
        if (LLVM_UNLIKELY(var == value || prior == value)) {
            return true;
        }
        PabloAST * const next = variables.get(cast<Var>(value));
        if (LLVM_LIKELY(next == nullptr || next == value)) {
            break;
        }
        value = next;
        assign->setValue(value);
    }
    variables.put(var, value);
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief fold
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * triviallyFold(Statement * stmt, PabloBlock * const currentScope) {
    if (isa<Not>(stmt)) {
        return triviallyFoldNot(stmt, currentScope);
    } else if (isa<And>(stmt)) {
        return triviallyFoldAnd(stmt, currentScope);
    } else if (isa<Or>(stmt)) {
        return triviallyFoldOr(stmt, currentScope);
    } else if (isa<Xor>(stmt)) {
        return triviallyFoldXor(stmt, currentScope);
    } else if (isa<Advance>(stmt)) {
        return triviallyFoldAdvance(stmt, currentScope);
    } else if (isa<ScanThru>(stmt)) {
        return triviallyFoldScanThru(stmt, currentScope);
    } else if (isa<MatchStar>(stmt)) {
        return triviallyFoldMatchStar(stmt, currentScope);
    } else if (isa<Lookahead>(stmt)) {
        return triviallyFoldLookAhead(stmt, currentScope);
    } else if (LLVM_UNLIKELY(isa<Sel>(stmt))) {
        return triviallyFoldSel(stmt, currentScope);
    } else if (LLVM_UNLIKELY(isa<Add>(stmt) || isa<Subtract>(stmt))) {
        return triviallyFoldAddOrSub(stmt, currentScope);
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief fold operation Not
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * triviallyFoldNot(Statement * stmt, PabloBlock * const block) {
    assert(isa<Not>(stmt));
    PabloAST * value = cast<Not>(stmt)->getExpr();
    if (LLVM_UNLIKELY(isa<Not>(value))) {
        return cast<Not>(value)->getExpr(); // ¬¬A ⇔ A
    } else if (LLVM_UNLIKELY(isa<Zeroes>(value))) {
        return block->createOnes(stmt->getType()); // ¬0 ⇔ 1
    } else if (LLVM_UNLIKELY(isa<Ones>(value))) {
        return block->createZeroes(stmt->getType()); // ¬1 ⇔ 0
    } else if (UseTernaryOptimizations) {
        if (auto ternary = dyn_cast<Ternary>(value)) { // ¬Ternary(m, a, b, c) ⇔ Ternary(¬m, a, b, c)
            block->setInsertPoint(stmt);
            const uint8_t mask = ternary->getMask()->value();
            return block->createTernary(block->getInteger(~mask), ternary->getA(), ternary->getB(), ternary->getC());
        }
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initial fold of binary operations And and Or
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * triviallyFoldBinaryAndOr(Statement * stmt, PabloBlock * const block) {
    assert(isa<And>(stmt) || isa<Or>(stmt));
    std::array<PabloAST *, 2> op;
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
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief fold operation And
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * triviallyFoldAnd(Statement * stmt, PabloBlock * const block) {
    assert(isa<And>(stmt));
    PabloAST * const folded = triviallyFoldBinaryAndOr(stmt, block);
    if (folded) {
        return folded;
    }
    if (UseTernaryOptimizations) {
        std::array<PabloAST *, 3> op;
        if (triviallyFoldBasicBinOp(stmt, stmt->getOperand(0), stmt->getOperand(1), block, op)) {
            PabloAST * basic_op = isBasicBinaryOp(stmt->getOperand(0)) ? stmt->getOperand(0) : stmt->getOperand(1);
            if (isa<And>(basic_op)) {
                return block->createAnd3(op[2], op[0], op[1]);
            } else if (isa<Or>(basic_op)) {
                return block->createAndOr(op[2], op[0], op[1]);
            } else if (isa<Xor>(basic_op)) {
                return block->createAndXor(op[2], op[0], op[1]);
            }
        }
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief fold operation Or
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * triviallyFoldOr(Statement * stmt, PabloBlock * const block) {
    assert(isa<Or>(stmt));
    PabloAST * const folded = triviallyFoldBinaryAndOr(stmt, block);
    if (folded) {
        return folded;
    }
    if (UseTernaryOptimizations) {
        std::array<PabloAST *, 3> op;
        if (triviallyFoldBasicBinOp(stmt, stmt->getOperand(0), stmt->getOperand(1), block, op)) {
            PabloAST * basic_op = isBasicBinaryOp(stmt->getOperand(0)) ? stmt->getOperand(0) : stmt->getOperand(1);
            if (isa<And>(basic_op)) {
                return block->createOrAnd(op[2], op[0], op[1]);
            } else if (isa<Or>(basic_op)) {
                return block->createOr3(op[2], op[0], op[1]);
            } else if (isa<Xor>(basic_op)) {
                return block->createOrXor(op[2], op[0], op[1]);
            }
        }
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief fold operation Xor
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * triviallyFoldXor(Statement * stmt, PabloBlock * const block) {
    assert(isa<Xor>(stmt));
    std::array<PabloAST *, 2> op;
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

    if (UseTernaryOptimizations && isa<PabloAST>(stmt) && equals(expr, cast<PabloAST>(stmt))) {
        std::array<PabloAST *, 3> op;
        if (triviallyFoldBasicBinOp(stmt, stmt->getOperand(0), stmt->getOperand(1), block, op)) {
            PabloAST * basic_op = isBasicBinaryOp(stmt->getOperand(0)) ? stmt->getOperand(0) : stmt->getOperand(1);
            if (isa<And>(basic_op)) {
                expr = block->createXorAnd(op[2], op[0], op[1]);
            } else if (isa<Or>(basic_op)) {
                expr = block->createXorOr(op[2], op[0], op[1]);
            } else if (isa<Xor>(basic_op)) {
                expr = block->createXor3(op[2], op[0], op[1]);
            }
        }
    }

    return expr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief fold operation Advance
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * triviallyFoldAdvance(Statement * stmt, PabloBlock * const block) {
    assert(isa<Advance>(stmt));
    Advance * const adv = cast<Advance>(stmt);
    if (LLVM_UNLIKELY(isa<Zeroes>(adv->getExpression()) || adv->getAmount() == 0)) {
        return adv->getExpression();
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief fold operation ScanThru
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * triviallyFoldScanThru(Statement * stmt, PabloBlock * const block) {
    assert(isa<ScanThru>(stmt));
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
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief fold operation MatchStar
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * triviallyFoldMatchStar(Statement * stmt, PabloBlock * const block) {
    assert(isa<MatchStar>(stmt));
    MatchStar * const mstar = cast<MatchStar>(stmt);
    if (LLVM_UNLIKELY(isa<Zeroes>(mstar->getMarker()) || isa<Zeroes>(mstar->getCharClass()))) {
        return mstar->getMarker();
    } else if (LLVM_UNLIKELY(isa<Ones>(mstar->getMarker()))) {
        block->setInsertPoint(stmt->getPrevNode());
        return block->createOnes(stmt->getType());
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief fold operation LookAhead
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * triviallyFoldLookAhead(Statement * stmt, PabloBlock * const block) {
    assert(isa<Lookahead>(stmt));
    Lookahead * const la = cast<Lookahead>(stmt);
    if (LLVM_UNLIKELY(isa<Zeroes>(la->getExpression()) || la->getAmount() == 0)) {
        return la->getExpression();
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief fold operation Sel
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * triviallyFoldSel(Statement * stmt, PabloBlock * const block) {
    assert(isa<Sel>(stmt));
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
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief fold operations Add or Subtraction
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * triviallyFoldAddOrSub(Statement * stmt, PabloBlock * const block) {
    assert(isa<Add>(stmt) || isa<Subtract>(stmt));
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
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief helper function to fold basic operation operands
 ** ------------------------------------------------------------------------------------------------------------- */
bool triviallyFoldBasicBinOp(Statement * stmt, PabloAST * expr1, PabloAST * expr2, PabloBlock * const block, std::array<PabloAST *, 3> & op) {
    if (isBasicBinaryOp(expr1)) {
        op[0] = cast<Statement>(expr1)->getOperand(0);
        op[1] = cast<Statement>(expr1)->getOperand(1);
        op[2] = expr2;
    } else if (isBasicBinaryOp(expr2)) {
        op[0] = cast<Statement>(expr2)->getOperand(0);
        op[1] = cast<Statement>(expr2)->getOperand(1);
        op[2] = expr1;
    } else {
        return false;
    }
    block->setInsertPoint(stmt);
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief check if op is And, Or or Xor
 ** ------------------------------------------------------------------------------------------------------------- */
bool isBasicBinaryOp(PabloAST * stmt) {
    return isa<Or>(stmt) || isa<Xor>(stmt) || isa<And>(stmt);
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
    Statement * const first = br->getBody()->front();
    Statement * nested = first;
    while (nested) {
        Statement * next = nested->removeFromParent();
        nested->insertAfter(stmt);
        stmt = nested;
        nested = next;
    }
    Statement * const result = br->eraseFromParent();
    assert (first == nullptr || result == first);
    return result;
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
 * @brief isOutputExtract
 ** ------------------------------------------------------------------------------------------------------------- */
static bool isOutputExtract(const Var * var, const LiveVarSet & set) {
    while (isa<Extract>(var)) {
        var = cast<Extract>(var)->getArray();
        if (set.contains(var)) {
            return true;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isSideEffecting
 ** ------------------------------------------------------------------------------------------------------------- */
bool isSideEffecting(const Statement * const stmt) const {
    const auto f = mSideEffecting.find(stmt);
    return f != mSideEffecting.end();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deadCodeElimination
 ** ------------------------------------------------------------------------------------------------------------- */
bool deadCodeElimination(PabloKernel * const kernel) {
    LiveVarSet variables;
    const auto n = kernel->getNumOfOutputs();
    for (unsigned i = 0; i < n; ++i) {
        variables.put(kernel->getOutput(i));
    }
    // add in any Extract nodes of one of our output Vars
    const auto m = kernel->getNumOfVariables();
    for (unsigned i = 0; i < m; ++i) {
        Var * const var = kernel->getVariable(i);
        if (LLVM_UNLIKELY(isOutputExtract(var, variables))) {
            variables.put(var);
        }
    }
    return !deadCodeElimination(kernel->getEntryScope(), variables, false);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deadCodeElimination
 ** ------------------------------------------------------------------------------------------------------------- */
bool deadCodeElimination(PabloBlock * const block, LiveVarSet & liveSet, const bool inspect) {
    bool flattened = false;
    for (Statement * stmt = block->back(), * prior; stmt; stmt = prior) {
        prior = stmt->getPrevNode();
        if (LLVM_UNLIKELY(stmt->getNumUses() == 0)) {
            const auto sideEffecting = isSideEffecting(stmt);
            if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
                Branch * const br = cast<Branch>(stmt);
                auto escaped = br->getEscaped();
                LiveVarSet nested(liveSet);
                // Collect all of the live vars in the branch's escaped var list
                for (const Var * var : escaped) {
                    if (LLVM_LIKELY(liveSet.contains(var))) {
                        nested.put(var);
                    }
                }

                // If none of the escaped vars from this branch are read, delete it.
                if (LLVM_LIKELY(!nested.empty() || sideEffecting)) {
                    if (LLVM_UNLIKELY(isa<While>(br))) {
                        // The condition of a while loop may not have had any uses outside
                        // of its reassignment for the next loop iteration test.
                        PabloAST * const cond = br->getCondition();
                        liveSet.put(cast<Var>(cond));
                        nested.put(cast<Var>(cond));
                        // Since while loops may have internal assignments that are required
                        // purely to support the production of a "live" var, we scan through
                        // the branch body once to collect all of the vars whose incoming
                        // value is read within the loop ...
                        LiveVarSet internal(nested);
                        deadCodeElimination(br->getBody(), internal, true);
                        // ... and add those to our set of live vars in the nested branch so
                        // that we do not erroneously delete them if we observe an assignment
                        nested.insert(internal.begin(), internal.end());
                    }
                    flattened |= deadCodeElimination(br->getBody(), nested, inspect);
                    // Although unlikely, it is possible that after DCE, a branch that did not
                    // appear to be trivial during the redundancy elimination (RE) phase is now
                    // trivial. However, this means we ought to re-run the RE pass to correctly
                    // replace any uses of the escaped Vars with their (likely) concrete value
                    // and propagate the impact of that change to the remaining program.
                    // While the impact is likely to be local to this scope, it is possible
                    // that it directly affects a Var that escapes this scope, which means we
                    // should not limit re-running RE on this scope alone. Given the rarity
                    // of finding a trivial scope, it's easier just to finish the DCE pass
                    // then repeat the whole process until a "flattening fixpoint" is found.
                    if (LLVM_LIKELY(!inspect && isa<If>(br)) && LLVM_UNLIKELY(isTrivial(br->getBody()))) {
                        prior = br->getBody()->back();
                        flatten(br);
                        flattened = true;
                        assert (prior->getParent() == block);
                    } else {
                        liveSet.insert(nested.begin(), nested.end());
                        continue;
                    }
                }
            } else if (LLVM_UNLIKELY(isa<Assign>(stmt))) {
                Var * const var = cast<Assign>(stmt)->getVariable();
                // Since we are reading the AST in reverse, an assignment to a var marks the
                // "end" of a local var's life.
                if (LLVM_LIKELY(liveSet.remove(var))) {
                    PabloAST * const value = cast<Assign>(stmt)->getValue();
                    if (LLVM_UNLIKELY(isa<Var>(value))) {
                        liveSet.put(cast<Var>(value));
                    }
                    continue;
                }
            }
            if (LLVM_LIKELY(!inspect && !sideEffecting)) {
                stmt->eraseFromParent();
            }
        } else {
            assert (!isa<Branch>(stmt) && !isa<Assign>(stmt));
            const auto n = stmt->getNumOperands();
            for (unsigned i = 0; i < n; ++i) {
                PabloAST * const op = stmt->getOperand(i);
                if (LLVM_UNLIKELY(isa<Var>(op))) {
                    liveSet.put(cast<Var>(op));
                }
            }
        }
    }
    return flattened;
}

private:

const bool                          UseTernaryOptimizations;
SmallVector<const PabloAST *, 64>   mNonZero;
flat_set<const Statement *>         mSideEffecting;

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
