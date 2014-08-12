#include "re_name.h"

Name::Name()
{
    mName = "";
    mNegated = false;
    mType = Name::FixedLength;
}

Name::Name(std::string name)
{
    mName = name;
    mNegated = false;
    mType = Name::FixedLength;
}

Name::~Name(){}

void Name::setName(std::string name)
{
    mName = name;
}

std::string Name::getName()
{
    return mName;
}

bool Name::isNegated()
{
    return mNegated;
}

void Name::setNegated(bool is_negated)
{
    mNegated = is_negated;
}

void Name::setType(Name::Type type)
{
    mType = type;
}

Name::Type Name::getType()
{
    return mType;
}


