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
    inline bool contains(T const item) const {
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
                mScopes.insert(cast<PabloKernel>(use)->getEntryBlock());
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
            getScopesOfAllUsers(isa<Assign>(stmt) ? cast<Assign>(stmt)->getVariable() : stmt);
        }
        if (LLVM_UNLIKELY(mScopes.empty())) {
            assert (!isa<Assign>(stmt));
            // should not occur unless we have a branch with no escaped vars or a statement
            // that has no users. In either event, the statement itself should be removed.
            stmt->eraseFromParent(true);
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
                getInScopeDominatorsOfAllUsers(isa<Assign>(stmt) ? cast<Assign>(stmt)->getVariable() : stmt, block);
            }
            Branch * branch = nullptr;
            Statement * temp = stmt;
            for (;;) {
                temp = temp->getNextNode();
                if (temp == nullptr || mUsers.contains(temp)) {
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
     * @brief doCodeSinking
     ** ------------------------------------------------------------------------------------------------------------- */
    void doCodeSinking(PabloBlock * const block) {
        Statement * stmt = block->back(); // note: reverse AST traversal
        while (stmt) {
            Statement * const prevNode = stmt->getPrevNode();
            sinkIfAcceptableTarget(stmt, block);
            stmt = prevNode;
        }
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief hoistLoopInvariants
     ** ------------------------------------------------------------------------------------------------------------- */
    void hoistLoopInvariants(Branch * const loop) {
        assert (mLoopVariants.empty());
        for (Var * variant : loop->getEscaped()) {
            mLoopVariants.insert(variant);
        }
        Statement * outerNode = loop->getPrevNode();
        Statement * stmt = loop->getBody()->front();
        while (stmt) {
            if (isa<Branch>(stmt)) {
                for (Var * var : cast<Branch>(stmt)->getEscaped()) {
                    mLoopVariants.insert(var);
                }
                stmt = stmt->getNextNode();
            } else {
                bool invariant = true;
                for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                    if (mLoopVariants.count(stmt->getOperand(i)) != 0) {
                        invariant = false;
                        break;
                    }
                }
                if (LLVM_UNLIKELY(invariant)) {
                    Statement * next = stmt->getNextNode();
                    stmt->insertAfter(outerNode);
                    outerNode = stmt;
                    stmt = next;
                } else {
                    mLoopVariants.insert(stmt);
                    stmt = stmt->getNextNode();
                }
            }
        }
        mLoopVariants.clear();
    }

    /** ------------------------------------------------------------------------------------------------------------- *
     * @brief doCodeMovement
     ** ------------------------------------------------------------------------------------------------------------- */
    void doCodeMovement(PabloBlock * const block) {
        doCodeSinking(block);
        for (Statement * stmt : *block) {
            if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
                doCodeMovement(cast<Branch>(stmt)->getBody());
                if (isa<While>(stmt)) {
                    // TODO: if we analyzed the probability of this loop being executed once, twice, or many times, we could
                    // determine whether hoisting will helpful or harmful to the expected run time.
                    hoistLoopInvariants(cast<While>(stmt));
                }
            }
        }
    }

private:
    ScopeSet        mScopes;
    UserSet         mUsers;
    LoopVariants    mLoopVariants;
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool CodeMotionPass::optimize(PabloKernel * kernel) {
    CodeMotionPassContainer C;
    C.doCodeMovement(kernel->getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(kernel, "post-code-motion");
    #endif
    return true;
}


}
