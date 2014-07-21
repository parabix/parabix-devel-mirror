/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "parsesuccess.h"

ParseSuccess::ParseSuccess(RE* re)
{
    mRE = re;
}

ParseSuccess::~ParseSuccess(){}

RE* ParseSuccess::getRE()
{
    return mRE;
}


