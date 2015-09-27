#include "pablo_codesinking.hpp"
#include <pablo/function.h>
#include <pablo/analysis/pabloverifier.hpp>

namespace pablo {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool CodeSinking::optimize(PabloFunction & function) {
    CodeSinking lcf;
    lcf.sink(function.getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-sinking");
    #endif
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isSafeToMove
 ** ------------------------------------------------------------------------------------------------------------- */
inline static bool isSafeToMove(Statement * stmt) {
    return !isa<Assign>(stmt) && !isa<Next>(stmt);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateDepthToCurrentBlock
 ** ------------------------------------------------------------------------------------------------------------- */
inline static unsigned calculateDepthToCurrentBlock(const PabloBlock * scope, const PabloBlock & root) {
    unsigned depth = 0;
    while (scope != &root) {
        ++depth;
        assert (scope);
        scope = scope->getParent();
    }
    return depth;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief sink
 ** ------------------------------------------------------------------------------------------------------------- */
void CodeSinking::sink(PabloBlock & block) {

    Statement * stmt = block.back(); // note: reverse AST traversal
    while (stmt) {
        Statement * next = stmt->getPrevNode();
        if (isa<If>(stmt)) {
            sink(cast<If>(stmt)->getBody());
        } else if (isa<While>(stmt)) {
            sink(cast<While>(stmt)->getBody());
        } else if (isSafeToMove(stmt)) {

            // Scan through this statement's users to see if they're all in a nested scope. If so,
            // find the least comon ancestor of the scope blocks. If it is not the current scope,
            // then we can sink the instruction.

            // (Note: the current scope is added to the list of processed ones AFTER we've traversed it.)

            ScopeSet scopes;
            bool canSinkInstruction = false;
            for (const PabloAST * use : stmt->users()) {
                if (const Statement * user = dyn_cast<Statement>(use)) {
                    if (mProcessed.count(user->getParent())) {
                        canSinkInstruction = true;
                        scopes.insert(user->getParent());
                        continue;
                    }
                    canSinkInstruction = false;
                    break;
                }
            }
            if (canSinkInstruction) {

                while (scopes.size() > 1) {
                    // Find the LCA of both scopes then add the LCA back to the list of scopes.
                    PabloBlock * scope1 = scopes.back();
                    scopes.pop_back();
                    unsigned depth1 = calculateDepthToCurrentBlock(scope1, block);

                    PabloBlock * scope2 = scopes.back();
                    scopes.pop_back();
                    unsigned depth2 = calculateDepthToCurrentBlock(scope2, block);

                    // If one of these scopes is nested deeper than the other, scan upwards through
                    // the scope tree until both scopes are at the same depth.
                    while (depth1 > depth2) {
                        scope1 = scope1->getParent();
                        --depth1;
                    }
                    while (depth1 < depth2) {
                        scope2 = scope2->getParent();
                        --depth2;
                    }

                    // Then iteratively step backwards until we find a matching set of scopes; this
                    // must be the LCA of our original scopes.
                    while (scope1 != scope2) {
                        scope1 = scope1->getParent();
                        scope2 = scope2->getParent();
                    }
                    assert (scope1 && scope2);
                    // But if the LCA is the current block, we can't sink the statement.
                    if (scope1 == &block) {
                        canSinkInstruction = false;
                        break;
                    }
                    scopes.push_back(scope1);
                }

                if (canSinkInstruction) {
                    assert (scopes.size() == 1);
                    stmt->insertBefore(scopes.front()->front());
                }
            }
        }
        stmt = next;
    }
    mProcessed.insert(&block);
}

}
