/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pe_all.h"

All::All(int num)
{
    mNum = num;
}

All::~All(){}

int All::getNum()
{
    return mNum;
}

void All::setNum(int num)
{
    mNum = num;
}
