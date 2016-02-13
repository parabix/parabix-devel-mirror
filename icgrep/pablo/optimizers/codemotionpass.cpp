#include "codemotionpass.h"
#include <pablo/codegenstate.h>
#include <pablo/analysis/pabloverifier.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/circular_buffer.hpp>

using namespace boost;
using namespace boost::container;

namespace pablo {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief optimize
 ** ------------------------------------------------------------------------------------------------------------- */
bool CodeMotionPass::optimize(PabloFunction & function) {
    CodeMotionPass::movement(function.getEntryBlock());
    #ifndef NDEBUG
    PabloVerifier::verify(function, "post-code-motion");
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
    reschedule(block);
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
inline static unsigned depthTo(const PabloBlock * scope, const PabloBlock * const root) {
    unsigned depth = 0;
    while (scope != root) {
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
inline bool findScopeUsages(Statement * stmt, ScopeSet & scopeSet, const PabloBlock * const block, const PabloBlock * const blocker) {
    for (PabloAST * use : stmt->users()) {
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
    if (isa<If>(stmt)) {
        for (Assign * def : cast<If>(stmt)->getDefined()) {
            if (!findScopeUsages(def, scopeSet, block, cast<If>(stmt)->getBody())) {
                return false;
            }
        }
    } else if (isa<While>(stmt)) {
        for (Next * var : cast<While>(stmt)->getVariants()) {
            if (escapes(var) && !findScopeUsages(var, scopeSet, block, cast<While>(stmt)->getBody())) {
                return false;
            }
        }
    } else if (isSafeToMove(stmt)) {
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
                    assert (scope1 && scope2);
                    scope1 = scope1->getParent();
                    scope2 = scope2->getParent();
                }
                assert (scope1);
                // But if the LCA is the current block, we can't sink the statement.
                if (scope1 == block) {
                    goto abort;
                }
                scopes.push_back(scope1);
            }
            assert (scopes.size() == 1);
            assert (isa<If>(stmt) ? (cast<If>(stmt)->getBody() != scopes.front()) : true);
            assert (isa<While>(stmt) ? (cast<While>(stmt)->getBody() != scopes.front()) : true);
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
    for (Next * variant : loop->getVariants()) {
        loopVariants.insert(variant);
        loopVariants.insert(variant->getInitial());
    }
    Statement * outerNode = loop->getPrevNode();
    Statement * stmt = loop->getBody()->front();
    while (stmt) {
        if (isa<If>(stmt)) {
            for (Assign * def : cast<If>(stmt)->getDefined()) {
                loopVariants.insert(def);
            }
        } else if (isa<While>(stmt)) {
            for (Next * var : cast<While>(stmt)->getVariants()) {
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief reschedule
 ** ------------------------------------------------------------------------------------------------------------- */
void CodeMotionPass::reschedule(PabloBlock * const block) {

//    using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, Statement *>;
//    using Vertex = Graph::vertex_descriptor;
//    using Map = flat_map<Statement *, Vertex>;

//    const unsigned size = std::distance(block->begin(), block->end());

//    Graph G(size);
//    Map M;

//    M.reserve(size);

//    unsigned i = 0;
//    for (Statement * stmt : *block) {
//        G[i] = stmt;
//        M.emplace(stmt, i);
//        ++i;
//    }

//    i = 0;
//    for (Statement * stmt : *block) {
//        for (PabloAST * user : stmt->users()) {
//            if (isa<Statement>(user)) {
//                Statement * use = cast<Statement>(user);
//                PabloBlock * parent = use->getParent();
//                while (parent) {
//                    if (parent == block) {
//                        break;
//                    }
//                    use = parent->getBranch();
//                    parent = parent->getParent();
//                }
//                auto f = M.find(use);
//                assert (f != M.end());
//                add_edge(i, f->second, G);
//            }
//        }
//        ++i;
//    }

//    circular_buffer<Vertex> ordering(size);
//    std::vector<unsigned> cumulativeDependencies;
//    topological_sort(G, std::back_inserter(ordering));
    


}

}
