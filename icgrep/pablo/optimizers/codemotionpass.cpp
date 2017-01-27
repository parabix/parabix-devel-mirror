#include "codemotionpass.h"
#include <pablo/pablo_kernel.h>
#include <pablo/codegenstate.h>
#include <pablo/branch.h>
#include <pablo/ps_assign.h>
#include <pablo/pe_var.h>
#include <boost/container/flat_set.hpp>
#include <vector>
#ifndef NDEBUG
#include <pablo/analysis/pabloverifier.hpp>
#endif

using namespace boost;
using namespace boost::container;
using namespace llvm;

namespace pablo {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool CodeMotionPass::optimize(PabloKernel * kernel) {
    CodeMotionPass::movement(kernel->getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(kernel, "post-code-motion");
    #endif
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief movement
 ** ------------------------------------------------------------------------------------------------------------- */
void CodeMotionPass::movement(PabloBlock * const block) {
    sink(block);
    for (Statement * stmt : *block) {
        if (isa<If>(stmt)) {
            movement(cast<If>(stmt)->getBody());
        } else if (isa<While>(stmt)) {
            movement(cast<While>(stmt)->getBody());
            // TODO: if we analyzed the probability of this loop being executed once, twice, or many times, we could
            // determine whether hoisting will helpful or harmful to the expected run time.
            hoistLoopInvariants(cast<While>(stmt));
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief depthOf
 ** ------------------------------------------------------------------------------------------------------------- */
inline static int depthOf(PabloBlock * scope) {
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
inline PabloBlock * getLCA(PabloBlock * scope1, PabloBlock * scope2) {
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getScopesOfAllUsers
 ** ------------------------------------------------------------------------------------------------------------- */
inline void getScopesOfAllUsers(PabloAST * expr, ScopeSet & scopes) {
    for (PabloAST * use : expr->users()) {
        if (LLVM_LIKELY(isa<Statement>(use))) {
            scopes.insert(cast<Statement>(use)->getParent());
        } else if (LLVM_UNLIKELY(isa<PabloKernel>(use))) {
            scopes.insert(cast<PabloKernel>(use)->getEntryBlock());
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInScopeDominatorsOfAllUsers
 ** ------------------------------------------------------------------------------------------------------------- */
inline void getInScopeDominatorsOfAllUsers(PabloAST * expr, UserSet & users, PabloBlock * const block) {
    for (PabloAST * use : expr->users()) {
        if (LLVM_LIKELY(isa<Statement>(use))) {
            Statement * user = cast<Statement>(use);
            PabloBlock * parent = user->getParent();
            while (parent != block) {
                assert (parent);
                user = parent->getBranch();
                parent = parent->getPredecessor();
            }
            users.insert(user);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief sinkIfAcceptableTarget
 *
 * Scan through this statement's users to see whether they're all in a nested scope. If not, check whether the
 * statement can be moved past a branch statement within the same scope.
 ** ------------------------------------------------------------------------------------------------------------- */
inline void sinkIfAcceptableTarget(Statement * const stmt, PabloBlock * const block, ScopeSet & scopes, UserSet & users) {
    assert (scopes.empty());
    if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
        for (Var * def : cast<Branch>(stmt)->getEscaped()) {
            getScopesOfAllUsers(def, scopes);
        }
    } else {
        getScopesOfAllUsers(isa<Assign>(stmt) ? cast<Assign>(stmt)->getVariable() : stmt, scopes);
    }    
    if (LLVM_UNLIKELY(scopes.empty())) {
        assert (!isa<Assign>(stmt));
        // should not occur unless we have a branch with no escaped vars or a statement
        // that has no users. In either event, the statement itself should be removed.
        stmt->eraseFromParent(true);
        return;
    }
    while (scopes.size() > 1) {
        PabloBlock * scope1 = scopes.back(); scopes.pop_back();
        PabloBlock * scope2 = scopes.back(); scopes.pop_back();
        scopes.insert(getLCA(scope1, scope2));
    }
    PabloBlock * const scope = scopes.back(); scopes.clear();
    if (LLVM_LIKELY(scope == block)) {
        assert (users.empty());
        if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
            for (Var * def : cast<Branch>(stmt)->getEscaped()) {
                getInScopeDominatorsOfAllUsers(def, users, block);
            }
        } else {
            getInScopeDominatorsOfAllUsers(isa<Assign>(stmt) ? cast<Assign>(stmt)->getVariable() : stmt, users, block);
        }
        Branch * branch = nullptr;
        Statement * temp = stmt;
        for (;;) {
            temp = temp->getNextNode();
            if (temp == nullptr || users.count(temp)) {
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
        users.clear();
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
 * @brief sink
 ** ------------------------------------------------------------------------------------------------------------- */
inline void CodeMotionPass::sink(PabloBlock * const block) {
    ScopeSet scopes;
    UserSet users;
    Statement * stmt = block->back(); // note: reverse AST traversal
    while (stmt) {
        Statement * prevNode = stmt->getPrevNode();
        sinkIfAcceptableTarget(stmt, block, scopes, users);
        stmt = prevNode;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hoistLoopInvariants
 ** ------------------------------------------------------------------------------------------------------------- */
void CodeMotionPass::hoistLoopInvariants(While * loop) {
    flat_set<const PabloAST *> loopVariants;
    for (Var * variant : loop->getEscaped()) {
        loopVariants.insert(variant);
    }
    Statement * outerNode = loop->getPrevNode();
    Statement * stmt = loop->getBody()->front();
    while (stmt) {
        if (isa<Branch>(stmt)) {
            for (Var * var : cast<Branch>(stmt)->getEscaped()) {
                loopVariants.insert(var);
            }
        } else {
            bool invariant = true;
            for (unsigned i = 0; i != stmt->getNumOperands(); ++i) {
                if (loopVariants.count(stmt->getOperand(i)) != 0) {
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
                loopVariants.insert(stmt);
                stmt = stmt->getNextNode();
            }
        }
    }
}

}
