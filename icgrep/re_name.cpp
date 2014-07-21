#include "re_name.h"

Name::Name(std::string name)
{
    mName = name;
}

Name::~Name(){}

std::string Name::getName()
{
    return mName;
}
