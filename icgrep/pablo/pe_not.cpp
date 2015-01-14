/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pe_not.h>
#include <pablo/codegenstate.h>
#include <pablo/pe_ones.h>
#include <pablo/pe_zeroes.h>

namespace pablo {

Not::Not(PabloAST * expr, PabloBlock * parent)
: Statement(ClassTypeId::Not, {expr}, parent->makeName("not"), parent)
{

}

}
