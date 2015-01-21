/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pe_sel.h>
#include <pablo/codegenstate.h>
#include <pablo/pe_ones.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_not.h>


namespace pablo {

Sel::Sel(PabloAST* if_expr, PabloAST* t_expr, PabloAST* f_expr, PabloBlock * parent)
: Statement(ClassTypeId::Sel, {if_expr, t_expr, f_expr}, parent->makeName("sel"), parent)
{

}

}
