/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pe_xor.h"

Xor::Xor(PabloE *expr1, PabloE *expr2)
: mExpr1(expr1)
, mExpr2(expr2)
{

}

Xor::~Xor()
{
    delete mExpr1;
    delete mExpr2;
}

PabloE* Xor:: getExpr1() const
{
    return mExpr1;
}

PabloE* Xor:: getExpr2() const
{
    return mExpr2;
}


