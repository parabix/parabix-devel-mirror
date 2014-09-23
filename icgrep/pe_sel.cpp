/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pe_sel.h"

Sel::Sel(PabloE* if_expr, PabloE* t_expr, PabloE* f_expr)
: mIf_expr(if_expr)
, mT_expr(t_expr)
, mF_expr(f_expr)
{

}

Sel::~Sel()
{
    delete mIf_expr;
    delete mT_expr;
    delete mF_expr;
}

PabloE* Sel::getIf_expr() const
{
    return mIf_expr;
}

PabloE* Sel::getT_expr() const
{
    return mT_expr;
}

PabloE* Sel::getF_expr() const
{
    return mF_expr;
}

