#include "re_name.h"

Name::Name()
{
    mName = "";
    mType = Name::FixedLength;
}

Name::Name(std::string name)
{
    mName = name;
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

void Name::setType(Name::Type type)
{
    mType = type;
}

Name::Type Name::getType()
{
    return mType;
}


