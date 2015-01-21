/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pe_or.h>
#include <pablo/codegenstate.h>
#include <pablo/pe_ones.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_not.h>
#include <pablo/pe_and.h>

namespace pablo {

Or::Or(PabloAST * expr1, PabloAST * expr2, PabloBlock * parent)
: Statement(ClassTypeId::Or, {expr1, expr2}, parent->makeName("or"), parent)
{

}

}
