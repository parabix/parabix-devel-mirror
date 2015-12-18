#include <pablo/ps_if.h>
#include <pablo/codegenstate.h>
#include <pablo/ps_assign.h>

namespace pablo {

If::If(PabloAST * expr, const std::initializer_list<Assign *> definedVars, PabloBlock * body)
: Statement(ClassTypeId::If, {expr}, nullptr)
, mBody(body)
, mDefined(definedVars.begin(), definedVars.end(), reinterpret_cast<DefinedAllocator &>(mAllocator)) {
    // Conceptually, having a defined var X is identical to having:
    //
    // Assign(X, 0)
    // If (...)
    //    Next(Assign(X), ...)
    //
    // Since the implied 'Next' node is a user of the Assign node, and the Assign node is
    // embedded into the If, the defined var is a user of the If node. However, since the
    // Assign's value is also dependant on the 'Next' value, the If node is also a user
    // of it.
    mBody->setBranch(this);
    mBody->setParent(getParent());
    for (Assign * def : mDefined) {
        def->addUser(this);
        this->addUser(def);
    }
}

If::If(PabloAST * expr, const std::vector<Assign *> & definedVars, PabloBlock * body)
: Statement(ClassTypeId::If, {expr}, nullptr)
, mBody(body)
, mDefined(definedVars.begin(), definedVars.end(), reinterpret_cast<DefinedAllocator &>(mAllocator)) {
    mBody->setBranch(this);
    mBody->setParent(getParent());
    for (Assign * def : mDefined) {
        def->addUser(this);
        this->addUser(def);
    }
}

void If::addDefined(Assign * def) {
    if (LLVM_LIKELY(std::find(mDefined.begin(), mDefined.end(), def) == mDefined.end())) {
        mDefined.push_back(def);
        def->addUser(this);
        this->addUser(def);
    }
}

If::DefinedVars::iterator If::removeDefined(Assign * def) {
    auto f = std::find(mDefined.begin(), mDefined.end(), def);
    if (LLVM_LIKELY(f != mDefined.end())) {
        def->removeUser(this);
        this->removeUser(def);
        return mDefined.erase(f);
    }
    return mDefined.end();
}

PabloBlock * If::setBody(PabloBlock * body) {
    body->setParent(mBody->getParent());
    std::swap(mBody, body);
    body->setParent(nullptr);
    return body;
}

}
