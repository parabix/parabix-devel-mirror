#include <pablo/ps_while.h>
#include <pablo/codegenstate.h>

namespace pablo {

While::VariantAllocator While::mVariantAllocator;

While::While(PabloAST * expr, const std::initializer_list<Next *> nextVars, PabloBlock * body)
: Statement(ClassTypeId::While, nullptr, {expr}, nullptr)
, mBody(body)
, mVariant(nextVars.begin(), nextVars.end(), mVariantAllocator) {
    mBody->setBranch(this);
    mBody->setPredecessor (getParent());
    for (Next * variant : nextVars) {
        variant->addUser(this);
        this->addUser(variant);
    }
}

While::While(PabloAST * expr, const std::vector<Next *> & nextVars, PabloBlock * body)
: Statement(ClassTypeId::While, nullptr, {expr}, nullptr)
, mBody(body)
, mVariant(nextVars.begin(), nextVars.end(), mVariantAllocator) {
    mBody->setBranch(this);
    mBody->setPredecessor (getParent());
    for (Next * variant : nextVars) {
        variant->addUser(this);
        this->addUser(variant);
    }
}

PabloBlock * While::setBody(PabloBlock * body) {
    body->setBranch(this);
    body->setPredecessor (mBody->getPredecessor ());
    std::swap(mBody, body);
    body->setPredecessor (nullptr);
    return body;
}

}
