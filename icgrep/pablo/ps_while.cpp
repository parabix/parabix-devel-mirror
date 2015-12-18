#include <pablo/ps_while.h>
#include <pablo/codegenstate.h>

namespace pablo {

While::While(PabloAST * expr, const std::initializer_list<Next *> nextVars, PabloBlock * body)
: Statement(ClassTypeId::While, {expr}, nullptr)
, mBody(body)
, mVariant(nextVars.begin(), nextVars.end(), reinterpret_cast<NextAllocator &>(mAllocator)) {
    mBody->setBranch(this);
    mBody->setParent(getParent());
    for (Next * variant : nextVars) {
        variant->addUser(this);
        this->addUser(variant);
    }
}

While::While(PabloAST * expr, const std::vector<Next *> & nextVars, PabloBlock * body)
: Statement(ClassTypeId::While, {expr}, nullptr)
, mBody(body)
, mVariant(nextVars.begin(), nextVars.end(), reinterpret_cast<NextAllocator &>(mAllocator)) {
    mBody->setBranch(this);
    mBody->setParent(getParent());
    for (Next * variant : nextVars) {
        variant->addUser(this);
        this->addUser(variant);
    }
}

PabloBlock * While::setBody(PabloBlock * body) {
    body->setParent(mBody->getParent());
    std::swap(mBody, body);
    body->setParent(nullptr);
    return body;
}

}
