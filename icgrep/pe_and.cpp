/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pe_and.h"

And::And(PabloE *expr1, PabloE *expr2)
{
    mExpr1 = expr1;
    mExpr2 = expr2;
}

And::~And()
{
    delete mExpr1;
    delete mExpr2;
}

PabloE* And::getExpr1()
{
    return mExpr1;
}

PabloE* And::getExpr2()
{
    return mExpr2;
}


