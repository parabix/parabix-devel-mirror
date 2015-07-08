#include <pablo/ps_while.h>
#include <pablo/codegenstate.h>

namespace pablo {

While::While(PabloAST * expr, const std::initializer_list<Next *> nextVars, PabloBlock & body)
: Statement(ClassTypeId::While, {expr}, nullptr)
, mBody(body)
, mNext(nextVars.begin(), nextVars.end(), reinterpret_cast<NextAllocator &>(mVectorAllocator))
{

}

While::While(PabloAST * expr, const std::vector<Next *> & nextVars, PabloBlock & body)
: Statement(ClassTypeId::While, {expr}, nullptr)
, mBody(body)
, mNext(nextVars.begin(), nextVars.end(), reinterpret_cast<NextAllocator &>(mVectorAllocator))
{

}

}
