#include "codemotionpass.h"
#include <pablo/pablo_kernel.h>
#include <pablo/codegenstate.h>
#include <pablo/branch.h>
#include <pablo/ps_assign.h>
#include <pablo/pe_var.h>
#include <boost/container/flat_set.hpp>
#ifndef NDEBUG
#include <pablo/analysis/pabloverifier.hpp>
#endif

#include <llvm/Support/raw_ostream.h>
#include <pablo/printer_pablos.h>

using namespace llvm;

namespace pablo {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief ScopeSet
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename T>
struct SetQueue : public std::vector<T> {
    inline void insert(T const item) {
        const auto i = std::lower_bound(std::vector<T>::begin(), std::vector<T>::end(), item);
        if (i == std::vector<T>::end() || *i != item) {
            std::vector<T>::insert(i, item);
        }
    }
    inline bool count(T const item) const {
        const auto i = std::lower_bound(std::vector<T>::begin(), std::vector<T>::end(), item);
        return (i != std::vector<T>::end() && *i == item);
    }
};

using ScopeSet = SetQueue<PabloBlock *>;

using UserSet = SetQueue<Statement *>;

using LoopVariants = boost::container::flat_set<const PabloAST *>;

struct CodeMotionPassContainer {

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief depthOf
     ** ------------------------------------------------------------------------------------------------------------- */
    static int depthOf(PabloBlock * scope) {
        int depth = 0;
        while (scope) {
            ++depth;
            scope = scope->getPredecessor();
        }
        return depth;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief findLCA
     ** ------------------------------------------------------------------------------------------------------------- */
    PabloBlock * getLCA(PabloBlock * scope1, PabloBlock * scope2) {
        int depth1 = depthOf(scope1);
        int depth2 = depthOf(scope2);
        // If one of these scopes is nested deeper than the other, scan upwards through
        // the scope tree until both scopes are at the same depth.
        while (depth1 > depth2) {
            scope1 = scope1->getPredecessor();
            --depth1;
        }
        while (depth1 < depth2) {
            scope2 = scope2->getPredecessor();
            --depth2;
        }
        // Then iteratively step backwards until we find a matching set of scopes; this
        // must be the LCA of our original scopes.
        while (scope1 != scope2) {
            assert (scope1 && scope2);
            scope1 = scope1->getPredecessor();
            scope2 = scope2->getPredecessor();
        }
        return scope1;
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief getScopesOfAllUsers
     ** ------------------------------------------------------------------------------------------------------------- */
    void getScopesOfAllUsers(PabloAST * expr) {
        for (PabloAST * use : expr->users()) {
            if (LLVM_LIKELY(isa<Statement>(use))) {
                mScopes.insert(cast<Statement>(use)->getParent());
            } else if (LLVM_UNLIKELY(isa<PabloKernel>(use))) {
                mScopes.insert(cast<PabloKernel>(use)->getEntryScope());
            }
        }
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief getInScopeDominatorsOfAllUsers
     ** ------------------------------------------------------------------------------------------------------------- */
    void getInScopeDominatorsOfAllUsers(PabloAST * expr, PabloBlock * const block) {
        for (PabloAST * use : expr->users()) {
            if (LLVM_LIKELY(isa<Statement>(use))) {
                Statement * user = cast<Statement>(use);
                PabloBlock * parent = user->getParent();
                while (parent != block) {
                    assert (parent);
                    user = parent->getBranch();
                    parent = parent->getPredecessor();
                }
                mUsers.insert(user);
            }
        }
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief sinkIfAcceptableTarget
     *
     * Scan through this statement's users to see whether they're all in a nested scope. If not, check whether the
     * statement can be moved past a branch statement within the same scope.
     ** ------------------------------------------------------------------------------------------------------------- */
    void sinkIfAcceptableTarget(Statement * const stmt, PabloBlock * const block) {
        assert (mScopes.empty() && mUsers.empty());
        if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
            for (Var * def : cast<Branch>(stmt)->getEscaped()) {
                getScopesOfAllUsers(def);
            }
        } else {
            PabloAST * expr = stmt;
            if (isa<Assign>(stmt)) {
                expr = cast<Assign>(stmt)->getVariable();
            }
            getScopesOfAllUsers(expr);
        }
        if (LLVM_UNLIKELY(mScopes.empty())) {
            return;
        }
        while (mScopes.size() > 1) {
            PabloBlock * scope1 = mScopes.back(); mScopes.pop_back();
            PabloBlock * scope2 = mScopes.back(); mScopes.pop_back();
            mScopes.insert(getLCA(scope1, scope2));
        }
        PabloBlock * const scope = mScopes.back(); mScopes.clear();
        if (LLVM_LIKELY(scope == block)) {
            assert (mUsers.empty());
            if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
                for (Var * def : cast<Branch>(stmt)->getEscaped()) {
                    getInScopeDominatorsOfAllUsers(def, block);
                }
            } else {

                PabloAST * expr = stmt;
                if (isa<Assign>(stmt)) {
                    expr = cast<Assign>(stmt)->getVariable();
                }

                getInScopeDominatorsOfAllUsers(expr, block);
            }
            Branch * branch = nullptr;
            Statement * temp = stmt;
            for (;;) {
                temp = temp->getNextNode();
                if (temp == nullptr || mUsers.count(temp)) {
                    if (branch) {
                        // we can move the statement past a branch within its current scope
                        stmt->insertAfter(branch);
                    }
                    break;
                }
                if (isa<Branch>(temp)) {
                    branch = cast<Branch>(temp);
                }
            }
            mUsers.clear();
        } else { // test whether the LCA scope is nested within this scope.
            PabloBlock * temp = scope;
            for (;;) {
                temp = temp->getPredecessor();
                if (temp == nullptr) {
                    break;
                } else if (temp == block) {
                    // we can move the statement into a nested scope
                    stmt->insertBefore(scope->front());
                    break;
                }
            }
        }
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief hoistLoopInvariants
     ** ------------------------------------------------------------------------------------------------------------- */
    void hoistLoopInvariants(While * const loop) {

        assert ("Loop variants were not cleared correctly." && mVariants.empty());

        // Assume every escaped Var is tainted but not necessarily a variant. Then iterate
        // through the loop body and mark any statement that uses a tainted expression as
        // tainted itself. If a Var is assigned a tainted value, mark the Var as a variant.

        for (const Var * var : loop->getEscaped()) {
            mTainted.insert(var);
        }

        for (const Statement * stmt : *loop->getBody()) {
            if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
                // Trust that any escaped values of an inner branch is a variant.
                for (const Var * var : cast<Branch>(stmt)->getEscaped()) {
                    mVariants.insert(var);
                }
            } else if (LLVM_UNLIKELY(isa<Assign>(stmt))) {
                const Assign * const a = cast<Assign>(stmt);
                if (LLVM_LIKELY(mTainted.count(a->getValue()))) {
                    mVariants.insert(a->getVariable());
               }
            } else {
                for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                    const PabloAST * const op = stmt->getOperand(i);
                    if (mTainted.count(op)) {
                        mTainted.insert(stmt);
                        break;
                    }
                }
            }
        }

        assert ("A loop without any variants is an infinite loop" && !mVariants.empty());

        mVariants.reserve(mTainted.size());
        mTainted.clear();

        // Iterate over the loop again but this time move any invariants out of the loop.
        // An invariant is any statement whose operands are all non-variants.

        Statement * outerNode = loop->getPrevNode();
        Statement * stmt = loop->getBody()->front();
        while (stmt) {
            if (isa<Branch>(stmt)) {
                stmt = stmt->getNextNode();
            } else {
                bool invariant = true;
                for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                    const PabloAST * const op = stmt->getOperand(i);
                    if (mVariants.count(op)) {
                        invariant = false;
                        break;
                    }
                }
                Statement * const next = stmt->getNextNode();
                if (LLVM_UNLIKELY(invariant)) {
                    stmt->insertAfter(outerNode);
                    outerNode = stmt;
                } else {
                    mVariants.insert(stmt);
                }
                stmt = next;
            }
        }

        mVariants.clear();
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief doCodeMovement
     ** ------------------------------------------------------------------------------------------------------------- */
    void doCodeMovement(PabloBlock * const block) {

        std::vector<Branch *> branches;

        Statement * stmt = block->back(); // note: reverse AST traversal
        while (stmt) {
            Statement * const prevNode = stmt->getPrevNode();
            sinkIfAcceptableTarget(stmt, block);
            if (isa<Branch>(stmt)) {
                branches.push_back(cast<Branch>(stmt));
            }
            stmt = prevNode;
        }

        while (!branches.empty()) {
            Branch * const br = branches.back();
            branches.pop_back();
            doCodeMovement(br->getBody());
            if (isa<While>(br)) {
                hoistLoopInvariants(cast<While>(br));
            }
        }

    }

private:
    ScopeSet        mScopes;
    UserSet         mUsers;
    LoopVariants    mVariants;
    LoopVariants    mTainted;
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool CodeMotionPass::optimize(PabloKernel * kernel) {
    CodeMotionPassContainer C;
    C.doCodeMovement(kernel->getEntryScope());
    #ifndef NDEBUG
    PabloVerifier::verify(kernel, "post-code-motion");
    #endif
    return true;
}


}
