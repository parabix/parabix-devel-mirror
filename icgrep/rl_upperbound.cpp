/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "rl_upperbound.h"

UpperBound::UpperBound(int ub)
{
    mUB = ub;
}

UpperBound::~UpperBound(){}

int UpperBound::getUB()
{
    return mUB;
}

void UpperBound::setUB(int ub)
{
    mUB = ub;
}

