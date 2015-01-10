#include <pablo/ps_if.h>
#include <pablo/codegenstate.h>

namespace pablo {

If::If(PabloAST * expr, DefinedVars && definedVars, PabloBlock & body, PabloBlock * parent)
: Statement(ClassTypeId::If, {expr}, nullptr, parent)
, mBody(body)
, mDefined(std::move(definedVars))
, mCarryCount(0)
, mAdvanceCount(0)
{

}

}
