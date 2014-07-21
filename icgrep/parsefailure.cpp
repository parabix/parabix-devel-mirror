/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "parsefailure.h"

ParseFailure::ParseFailure(std::string msg)
{
    mErrorMsg = msg;
}

ParseFailure::~ParseFailure(){}

std::string ParseFailure::getErrorMsg()
{
    return mErrorMsg;
}
