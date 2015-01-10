#include <pablo/ps_if.h>
#include <pablo/codegenstate.h>

namespace pablo {

If::If(PabloAST * expr, DefinedVars && definedVars, PabloBlock & body, PabloBlock * parent)
: Statement(ClassTypeId::If, {expr}, nullptr, parent)
, mBody(body)
, mDefined(definedVars.begin(), definedVars.end())
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
    // embedded into the If, the defined var is a user of the If node.

    for (Assign * x : mDefined) {
        addUser(x);
    }
}

}
