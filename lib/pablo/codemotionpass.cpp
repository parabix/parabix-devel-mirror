#include <pablo/codemotionpass.h>

#include <pablo/pablo_kernel.h>
#include <pablo/codegenstate.h>
#include <pablo/branch.h>
#include <pablo/ps_assign.h>
#include <pablo/pe_var.h>
#include <boost/container/flat_set.hpp>
#ifndef NDEBUG
#include <pablo/pabloverifier.hpp>
#endif

using namespace llvm;

namespace pablo {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief ScopeSet
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename T, unsigned n>
struct SetQueue : public SmallVector<T, n> {
    using Base = SmallVector<T, n>;
    inline void insert(T const item) {
        const auto i = std::lower_bound(Base::begin(), Base::end(), item);
        if (i == Base::end() || *i != item) {
            Base::insert(i, item);
        }
    }
    inline bool count(T const item) const {
        const auto i = std::lower_bound(Base::begin(), Base::end(), item);
        return (i != Base::end() && *i == item);
    }
};

using ScopeSet = SetQueue<PabloBlock *, 32>;

using UserSet = SetQueue<Statement *, 256>;

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

        PabloBlock * const nestedBody = loop->getBody();
        for (const Statement * stmt : *nestedBody) {
            if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
                // Trust that any escaped values of an inner branch is a variant.
                for (const Var * var : cast<Branch>(stmt)->getEscaped()) {
                    mTainted.insert(var);
                }
            } else if (LLVM_UNLIKELY(isa<Assign>(stmt))) {
                const Assign * const a = cast<Assign>(stmt);
                if (LLVM_LIKELY(mTainted.count(a->getValue()))) {
                    const Var * const var = a->getVariable();
                    mVariants.insert(var);
                    mTainted.insert(var);
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

        mVariants.reserve(mTainted.size());
        mTainted.clear();

        // Iterate over the loop again but this time move any invariants out of the loop.
        // An invariant is any statement whose operands are also invariant.

        Statement * ip = loop->getPrevNode();
        Statement * stmt = nestedBody->front();
        while (stmt) {
            Statement * const next = stmt->getNextNode();
            if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
                stmt = next;
            } else {
                bool invariant = true;
                const auto n = stmt->getNumOperands();
                for (unsigned i = 0; i != n; ++i) {
                    const PabloAST * const op = stmt->getOperand(i);
                    if (mVariants.count(op)) {
                        invariant = false;
                        break;
                    }
                }
                if (LLVM_UNLIKELY(invariant)) {
                    stmt->insertAfter(ip);
                    ip = stmt;
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

        SmallVector<Branch *, 8> branches;

        Statement * stmt = block->back(); // note: reverse AST traversal
        while (stmt) {
            Statement * const prevNode = stmt->getPrevNode();
            sinkIfAcceptableTarget(stmt, block);
            if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
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
