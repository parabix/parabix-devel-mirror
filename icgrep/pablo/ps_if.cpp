#include <pablo/ps_if.h>
#include <pablo/codegenstate.h>

namespace pablo {

If::If(PabloAST * expr, DefinedVars && definedVars, PabloBlock & body, PabloBlock * parent)
: Statement(ClassTypeId::If, parent)
, mExpr(expr)
, mBody(body)
, mDefined(std::move(definedVars))
, mCarryCount(0)
, mAdvanceCount(0)
{
    expr->addUser(this);
    for (Statement * s : body) {
        addUser(s);
    }
}

}
