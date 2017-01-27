#include "branch.h"
#include <pablo/codegenstate.h>
#include <pablo/pe_var.h>
#include <pablo/ps_assign.h>
#include <pablo/pablo_kernel.h>

#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace pablo {

Branch::Branch(const ClassTypeId typeId, PabloAST * condition, PabloBlock * body, Allocator &allocator)
: Statement(typeId, nullptr, {condition}, nullptr, allocator)
, mBody(body) {

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief escapes
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool escapes(const Var * const var, const Branch * const br) {
    bool inside = false;
    bool outside = false;

    for (const PabloAST * user : var->users()) {

        if (isa<Assign>(user)) {

            const PabloBlock * const scope = cast<Assign>(user)->getParent();
            // Is this Var assigned a value within the body of this branch?
            for (const PabloBlock * test = scope; test; test = test->getPredecessor()) {
                if (test == br->getBody()) {
                    if (outside) {
                        return true;
                    }
                    inside = true;
                    goto outer_loop;
                }
            }

            // Is there an assignment to this Var that dominates this branch?
            const Branch * check = br;
            for (const PabloBlock * test = br->getParent(); test; ) {
                if (test == scope) {
                    // verify this assignment actually dominates the branch
                    const Statement * temp1 = cast<Assign>(user);
                    const Statement * temp2 = check;
                    while (temp1 && temp2) {
                        if (temp1 == check) {
                            break;
                        } else if (temp2 == cast<Assign>(user)) {
                            temp1 = nullptr;
                            break;
                        }
                        temp1 = temp1->getNextNode();
                        temp2 = temp2->getNextNode();
                    }
                    if (temp1 != nullptr) {
                        if (inside) {
                            return true;
                        }
                        outside = true;
                    }
                    break;
                }
                check = test->getBranch();
                if (LLVM_UNLIKELY(check == nullptr)) {
                    break;
                }
                test = check->getParent();
            }
        } else if (isa<PabloKernel>(user)) {
            if (inside) {
                return true;
            }
            outside = true;
        }
outer_loop: continue;
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getEscaped
 ** ------------------------------------------------------------------------------------------------------------- */
Branch::EscapedVars Branch::getEscaped() const {
    const auto f = getParent()->getParent();
    const auto n = f->getNumOfVariables();
    EscapedVars escaped;
    for (unsigned i = 0; i < n; ++i) {
        Var * const var = f->getVariable(i);
        if (escapes(var, this)) {
            escaped.push_back(var);
        }
    }
    return escaped;
}

PabloBlock * Branch::setBody(PabloBlock * const body) {
    body->setBranch(this);
    PabloBlock * const priorBody = mBody;
    mBody = body;
    priorBody->setBranch(nullptr);
    return priorBody;
}

}
