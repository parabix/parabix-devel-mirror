/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "ps_assign.h"

Assign::Assign(std::string m, PabloE* expr)
{
    mM = m;
    mExpr = expr;
}

Assign::~Assign()
{
    delete mExpr;
}

std::string Assign::getM()
{
    return mM;
}

PabloE* Assign::getExpr()
{
    return mExpr;
}


