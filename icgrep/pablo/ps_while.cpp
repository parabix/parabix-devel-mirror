#include <pablo/ps_while.h>
#include <pablo/codegenstate.h>

namespace pablo {

While::While(PabloAST * expr, PabloBlock & body, PabloBlock * parent)
: Statement(ClassTypeId::While, nullptr, parent)
, mExpr(expr)
, mBody(body)
, mCarryCount(0)
, mAdvanceCount(0)
{
    expr->addUser(this);
    for (Statement * s : body) {
        addUser(s);
    }
}

}
