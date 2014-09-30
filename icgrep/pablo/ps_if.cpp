/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "ps_if.h"

If::If(PabloE* expr, std::list<PabloS*> psl)
{
    mExpr = expr;
    mPSList = psl;
}

If::~If()
{
    delete mExpr;
    while (!mPSList.empty()) delete mPSList.front(), mPSList.pop_front();
}

PabloE* If::getExpr()
{
    return mExpr;
}

std::list<PabloS*> If::getPSList()
{
    return mPSList;
}

