#include <pablo/ps_while.h>
#include <pablo/codegenstate.h>

namespace pablo {

While::While(PabloAST * expr, PabloBlock & body, PabloBlock * parent)
: Statement(ClassTypeId::While, {expr}, nullptr, parent)
, mBody(body)
, mCarryCount(0)
, mAdvanceCount(0)
{

}

}
