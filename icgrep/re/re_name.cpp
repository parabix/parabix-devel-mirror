#include "re_name.h"

Name::Name()
: mName()
, mNegated(false)
, mType(Name::FixedLength) {
    mName = "";
    mNegated = false;
    mType = Name::FixedLength;
}

Name::Name(std::string name)
: mName(name)
, mNegated(false)
, mType(Name::FixedLength) {

}

Name::Name(const Name * name)
: mName(name->getName())
, mNegated(name->isNegated())
, mType(name->getType()) {

}

Name::~Name(){

}

void Name::setName(std::string name) {
    mName = name;
}

std::string Name::getName() const {
    return mName;
}

bool Name::isNegated() const {
    return mNegated;
}

void Name::setNegated(const bool is_negated) {
    mNegated = is_negated;
}

void Name::setType(const Type type) {
    mType = type;
}

Name::Type Name::getType() const {
    return mType;
}


