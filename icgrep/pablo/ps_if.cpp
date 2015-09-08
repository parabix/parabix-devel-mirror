#include <pablo/ps_if.h>
#include <pablo/codegenstate.h>
#include <pablo/ps_assign.h>

namespace pablo {

If::If(PabloAST * expr, const std::initializer_list<Assign *> definedVars, PabloBlock & body)
: Statement(ClassTypeId::If, {expr}, nullptr)
, mBody(body)
, mDefined(definedVars.begin(), definedVars.end(), reinterpret_cast<DefinedAllocator &>(mVectorAllocator))
{
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

    for (PabloAST * def : mDefined) {
        def->addUser(this);
        addUser(def);
    }
}

If::If(PabloAST * expr, const std::vector<Assign *> & definedVars, PabloBlock & body)
: Statement(ClassTypeId::If, {expr}, nullptr)
, mBody(body)
, mDefined(definedVars.begin(), definedVars.end(), reinterpret_cast<DefinedAllocator &>(mVectorAllocator))
{
    for (PabloAST * def : mDefined) {
        def->addUser(this);
        addUser(def);
    }
}

void If::addDefined(Assign * def) {
    if (LLVM_LIKELY(std::find(mDefined.begin(), mDefined.end(), def) != mDefined.end())) {
        const auto size = mDefined.size();
        mDefined.push_back(def);
        assert (mDefined.size() == size + 1);
        assert (mDefined.back() == def);
        def->addUser(this);
        addUser(def);
    }
}

}
