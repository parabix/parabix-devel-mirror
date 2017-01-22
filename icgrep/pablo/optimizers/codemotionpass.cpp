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
inline PabloBlock * findLCA(PabloBlock * scope1, PabloBlock * scope2) {
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
struct ScopeSet : public std::vector<PabloBlock *> {
    inline void insert(PabloBlock * block) {
        const auto i = std::lower_bound(begin(), end(), block);
        if (i == end() || *i != block) {
            std::vector<PabloBlock *>::insert(i, block);
        }
    }
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief findScopeUsages
 ** ------------------------------------------------------------------------------------------------------------- */
inline void findUsageScopes(PabloAST * expr, ScopeSet & scopes) {
    for (PabloAST * use : expr->users()) {
        scopes.insert(cast<Statement>(use)->getParent());
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isAcceptableTarget
 ** ------------------------------------------------------------------------------------------------------------- */
inline PabloBlock * isAcceptableTarget(Statement * stmt, ScopeSet & scopes, PabloBlock * const block) {
    // Scan through this statement's users to see if they're all in a nested scope. If so,
    // find the least common ancestor of the scope blocks. If it is not the current scope,
    // then we can sink the instruction.
    assert (scopes.empty());
    if (isa<Assign>(stmt)) {
        return nullptr;
    } else if (isa<Branch>(stmt)) {
        for (Var * def : cast<Branch>(stmt)->getEscaped()) {
            findUsageScopes(def, scopes);
        }
    } else {
        findUsageScopes(stmt, scopes);
    }
    if (LLVM_UNLIKELY(scopes.empty())) {
        return nullptr;
    }
    while (scopes.size() > 1) {
        PabloBlock * scope1 = scopes.back(); scopes.pop_back();
        PabloBlock * scope2 = scopes.back(); scopes.pop_back();
        scopes.insert(findLCA(scope1, scope2));
    }
    // If the LCA scope is nested within the block, return the LCA scope.
    // Otherwise return nullptr.
    PabloBlock * const scope = scopes.back(); scopes.clear();
    if (scope == block) {
        return nullptr;
    }
    PabloBlock * temp = scope;
    for (;;) {
        temp = temp->getPredecessor();
        if (temp == nullptr) {
            return nullptr;
        } else if (temp == block) {
            return scope;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief sink
 ** ------------------------------------------------------------------------------------------------------------- */
inline void CodeMotionPass::sink(PabloBlock * const block) {
    ScopeSet scopes;
    Statement * stmt = block->back(); // note: reverse AST traversal
    while (stmt) {
        Statement * prevNode = stmt->getPrevNode();
        if (PabloBlock * scope = isAcceptableTarget(stmt, scopes, block)) {
            stmt->insertBefore(scope->front());
        }
        stmt = prevNode;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hoistWhileLoopInvariants
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
