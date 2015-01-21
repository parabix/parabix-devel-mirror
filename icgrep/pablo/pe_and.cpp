/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pe_and.h>
#include <pablo/codegenstate.h>
#include <pablo/pe_not.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>

namespace pablo {

And::And(PabloAST * expr1, PabloAST * expr2, PabloBlock * parent)
: Statement(ClassTypeId::And, {expr1, expr2}, parent->makeName("and"), parent)
{

}

}
