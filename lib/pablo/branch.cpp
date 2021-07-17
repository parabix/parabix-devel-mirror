#include <pablo/branch.h>
#include <pablo/codegenstate.h>
#include <pablo/pe_var.h>
#include <pablo/ps_assign.h>
#include <pablo/pablo_kernel.h>

using namespace llvm;

namespace pablo {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief escapes
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool escapes(const Var * const var, const Branch * const br) {
    bool inside = false;
    bool outside = false;

    for (const PabloAST * user : var->users()) {
        if (isa<Assign>(user)) {
            const PabloBlock * const scope = cast<Assign>(user)->getParent();
            if (!inside) {
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
            }

            // Is there an assignment to this Var that dominates this branch?
            if (!outside) {
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
            }
        } else if (LLVM_LIKELY(isa<PabloKernel>(user) && !outside)) {
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
    const auto kernel = getParent()->getParent();
    const auto n = kernel->getNumOfVariables();
    EscapedVars escaped;
    for (unsigned i = 0; i < n; ++i) {
        Var * const var = kernel->getVariable(i);
        if (escapes(var, this)) {
            escaped.push_back(var);
        }
    }
    return escaped;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setBody
 ** ------------------------------------------------------------------------------------------------------------- */
PabloBlock * Branch::setBody(PabloBlock * const body) {
    body->setBranch(this);
    PabloBlock * const priorBody = mBody;
    mBody = body;
    priorBody->setBranch(nullptr);
    return priorBody;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
Branch::Branch(const ClassTypeId typeId, PabloAST * condition, PabloBlock * body, Allocator &allocator)
: Statement(typeId, nullptr, {condition}, nullptr, allocator)
, mBody(body)
, mRegular(true)
{

}

}
