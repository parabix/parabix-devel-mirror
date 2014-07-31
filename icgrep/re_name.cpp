#include "re_name.h"

Name::Name()
{
    mName = "";
}

Name::Name(std::string name)
{
    mName = name;
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
