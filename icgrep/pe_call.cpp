/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pe_call.h"

Call::Call(std::string callee)
{
    mCallee = callee;
}

Call::~Call(){}

std::string Call::getCallee()
{
    return mCallee;
}

