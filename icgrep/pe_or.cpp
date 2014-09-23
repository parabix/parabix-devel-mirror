/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pe_or.h"

Or::Or(PabloE* expr1, PabloE* expr2)
: mExpr1(expr1)
, mExpr2(expr2)
{

}

Or::~Or()
{
    delete mExpr1;
    delete mExpr2;
}

PabloE* Or::getExpr1() const
{
    return mExpr1;
}

PabloE* Or::getExpr2() const
{
    return mExpr2;
}

