#include <pablo/branch.h>
#include <pablo/codegenstate.h>
#include <pablo/printer_pablos.h>

namespace pablo {

Branch::Branch(const ClassTypeId typeId, PabloAST * condition, PabloBlock * body)
: Statement(typeId, nullptr, {condition}, nullptr)
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
            // is this var assigned a value within the scope of the branch?
            const PabloBlock * const scope = cast<Assign>(user)->getParent();

            for (const PabloBlock * test = scope; test; test = test->getPredecessor()) {
                if (test == br->getBody()) {
                    if (outside) {
                        return true;
                    }
                    inside = true;
                    break;
                }
            }

            for (const PabloBlock * test = br->getParent(); test; test = test->getPredecessor()) {
                // technically we should check whether this user dominates the branch but if it
                // doesn't, then the program is illegal anyway and the error will be found.
                if (test == scope) {
                    if (inside) {
                        return true;
                    }
                    outside = true;
                    break;
                }
            }

        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getEscaped
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Var *> Branch::getEscaped() const {

    PabloFunction * const f = getParent()->getParent();
    const auto n = f->getNumOfVariables();

    std::vector<Var *> escaped;    
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
