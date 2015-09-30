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
 * @brief findScopeUsages
 ** ------------------------------------------------------------------------------------------------------------- */
template <class ScopeSet>
inline bool findScopeUsages(Statement * stmt, ScopeSet & scopeSet, const PabloBlock & block) {
    for (PabloAST * use : stmt->users()) {
        assert (isa<Statement>(use));
        PabloBlock * const parent = cast<Statement>(use)->getParent();
        if (LLVM_LIKELY(parent == &block)) {
            return false;
        }
        scopeSet.insert(parent);
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief findScopeUsages
 ** ------------------------------------------------------------------------------------------------------------- */
template <class ScopeSet>
inline bool findScopeUsages(Statement * stmt, ScopeSet & scopeSet, const PabloBlock & block, const PabloBlock & ignored) {
    for (PabloAST * use : stmt->users()) {
        assert (isa<Statement>(use));
        PabloBlock * const parent = cast<Statement>(use)->getParent();
        if (LLVM_LIKELY(parent == &block)) {
            return false;
        }
        if (parent != &ignored) {
            scopeSet.insert(parent);
        }
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief sink
 ** ------------------------------------------------------------------------------------------------------------- */
void CodeSinking::sink(PabloBlock & block) {

    ScopeSet scopes;
    Statement * stmt = block.back(); // note: reverse AST traversal
    while (stmt) {
        Statement * prevNode = stmt->getPrevNode();

        bool sinkable = true;
        // Scan through this statement's users to see if they're all in a nested scope. If so,
        // find the least common ancestor of the scope blocks. If it is not the current scope,
        // then we can sink the instruction.
        if (isa<If>(stmt)) {
            PabloBlock & nested = cast<If>(stmt)->getBody();
            sink(nested);
            for (Assign * def : cast<const If>(stmt)->getDefined()) {
                if (!findScopeUsages(def, scopes, block, nested)) {
                    sinkable = false;
                    break;
                }
            }
        } else if (isa<While>(stmt)) {
            PabloBlock & nested = cast<While>(stmt)->getBody();
            sink(nested);
            for (Next * var : cast<const While>(stmt)->getVariants()) {
                if (escapes(var) && !findScopeUsages(var, scopes, block, nested)) {
                    sinkable = false;
                    break;
                }
            }
        } else {
            sinkable = isSafeToMove(stmt) ? findScopeUsages(stmt, scopes, block) : false;
        }

        if (sinkable) {
            assert (scopes.size() > 0);
            while (scopes.size() > 1) {
                // Find the LCA of both scopes then add the LCA back to the list of scopes.
                PabloBlock * scope1 = scopes.back(); scopes.pop_back();
                unsigned depth1 = calculateDepthToCurrentBlock(scope1, block);

                PabloBlock * scope2 = scopes.back(); scopes.pop_back();
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
                    sinkable = false;
                    break;
                }
                scopes.push_back(scope1);
            }
            if (sinkable) {
                assert (scopes.size() == 1);
                assert (isa<If>(stmt) ? &(cast<If>(stmt)->getBody()) != scopes.front() : true);
                assert (isa<While>(stmt) ? &(cast<While>(stmt)->getBody()) != scopes.front() : true);
                stmt->insertBefore(scopes.front()->front());
            }
        }
        scopes.clear();
        stmt = prevNode;
    }
}

}
