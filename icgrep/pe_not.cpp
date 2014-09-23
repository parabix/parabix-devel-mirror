/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pe_not.h"

Not::Not(PabloE* expr)
{
    mExpr = expr;
}

Not::~Not()
{
    delete mExpr;
}

PabloE* Not::getExpr() const
{
    return mExpr;
}


