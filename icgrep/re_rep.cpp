/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_rep.h"

Rep::Rep(RE* re, int lb, int ub)
{
    mRE = re;
    mLB = lb;
    mUB = ub;
}

Rep::~Rep()
{
    delete mRE;
}

RE* Rep::getRE()
{
    return mRE;
}

int Rep::getLB()
{
    return mLB;
}

void Rep::setLB(int lb)
{
    mLB = lb;
}

int Rep::getUB()
{
    return mUB;
}

void Rep::setUB(int ub)
{
    mUB = ub;
}


