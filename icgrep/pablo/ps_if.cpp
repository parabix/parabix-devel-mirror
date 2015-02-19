#include <pablo/ps_if.h>
#include <pablo/codegenstate.h>
#include <pablo/ps_assign.h>

namespace pablo {

If::If(PabloAST * expr, std::initializer_list<Assign *> && definedVars, PabloBlock & body, PabloBlock * parent)
: Statement(ClassTypeId::If, {expr}, nullptr, parent)
, mBody(body)
, mDefined(definedVars.begin(), definedVars.end(), mVectorAllocator)
, mCarryCount(0)
, mAdvanceCount(0)
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

    for (PabloAST * assign : mDefined) {
        assign->addUser(this);
        addUser(assign);
    }
}

If::If(PabloAST * expr, const std::vector<Assign *> & definedVars, PabloBlock & body, PabloBlock * parent)
: Statement(ClassTypeId::If, {expr}, nullptr, parent)
, mBody(body)
, mDefined(definedVars.begin(), definedVars.end(), mVectorAllocator)
, mCarryCount(0)
, mAdvanceCount(0)
{
    for (PabloAST * assign : mDefined) {
        assign->addUser(this);
        addUser(assign);
    }
}


}
