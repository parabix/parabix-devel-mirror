#include "codemotionpass.h"
#include <pablo/pablo_kernel.h>
#include <pablo/codegenstate.h>
#include <pablo/analysis/pabloverifier.hpp>
#include <boost/container/flat_set.hpp>
// #include <boost/circular_buffer.hpp>

using namespace boost;
using namespace boost::container;

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
 * @brief depthTo
 ** ------------------------------------------------------------------------------------------------------------- */
inline static unsigned depthTo(const PabloBlock * scope, const PabloBlock * const root) {
    unsigned depth = 0;
    while (scope != root) {
        ++depth;
        assert (scope);
        scope = scope->getPredecessor();
    }
    return depth;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief findScopeUsages
 ** ------------------------------------------------------------------------------------------------------------- */
template <class ScopeSet>
inline bool findScopeUsages(PabloAST * expr, ScopeSet & scopeSet, const PabloBlock * const block, const PabloBlock * const blocker) {
    for (PabloAST * use : expr->users()) {
        assert (isa<Statement>(use));
        PabloBlock * const parent = cast<Statement>(use)->getParent();
        if (LLVM_LIKELY(parent == block)) {
            return false;
        }
        if (parent != blocker) {
            scopeSet.insert(parent);
        }
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isAcceptableTarget
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool CodeMotionPass::isAcceptableTarget(Statement * stmt, ScopeSet & scopeSet, const PabloBlock * const block) {
    // Scan through this statement's users to see if they're all in a nested scope. If so,
    // find the least common ancestor of the scope blocks. If it is not the current scope,
    // then we can sink the instruction.
    assert (scopeSet.empty());
    if (isa<Branch>(stmt)) {
        for (Var * def : cast<Branch>(stmt)->getEscaped()) {
            if (!findScopeUsages(def, scopeSet, block, cast<Branch>(stmt)->getBody())) {
                return false;
            }
        }
    } else if (!isa<Assign>(stmt)) {
        return findScopeUsages(stmt, scopeSet, block, nullptr);
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief sink
 ** ------------------------------------------------------------------------------------------------------------- */
inline void CodeMotionPass::sink(PabloBlock * const block) {
    ScopeSet scopes;
    Statement * stmt = block->back(); // note: reverse AST traversal
    while (stmt) {
        Statement * prevNode = stmt->getPrevNode();
        if (isAcceptableTarget(stmt, scopes, block)) {
            assert (scopes.size() > 0);
            while (scopes.size() > 1) {
                // Find the LCA of both scopes then add the LCA back to the list of scopes.
                PabloBlock * scope1 = scopes.back(); scopes.pop_back();
                unsigned depth1 = depthTo(scope1, block);

                PabloBlock * scope2 = scopes.back(); scopes.pop_back();
                unsigned depth2 = depthTo(scope2, block);

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
                assert (scope1);
                // But if the LCA is the current block, we can't sink the statement.
                if (scope1 == block) {
                    goto abort;
                }
                scopes.push_back(scope1);
            }
            assert (scopes.size() == 1);
            assert (isa<Branch>(stmt) ? (cast<Branch>(stmt)->getBody() != scopes.front()) : true);
            stmt->insertBefore(scopes.front()->front());
        }
abort:  scopes.clear();
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
