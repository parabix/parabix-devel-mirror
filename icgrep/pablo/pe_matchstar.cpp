/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pe_matchstar.h"

MatchStar::MatchStar(PabloE* expr1, PabloE* expr2)
{
    mExpr1 = expr1;
    mExpr2 = expr2;
}

MatchStar::~MatchStar()
{
    delete mExpr1;
    delete mExpr2;
}

PabloE* MatchStar::getExpr1()
{
    return mExpr1;
}

PabloE* MatchStar::getExpr2()
{
    return mExpr2;
}


