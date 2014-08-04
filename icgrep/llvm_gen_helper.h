/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef LLVM_GENERATOR_HELPER_H
#define LLVM_GENERATOR_HELPER_H

//Regular Expressions
#include "re_re.h"
#include "re_cc.h"
#include "re_start.h"
#include "re_end.h"
#include "re_seq.h"

//Pablo Expressions
#include "pe_pabloe.h"
#include "pe_advance.h"
#include "pe_and.h"
#include "pe_charclass.h"
#include "pe_not.h"
#include "pe_or.h"
#include "pe_matchstar.h"
#include "pe_scanthru.h"

//Pablo Statements
#include "ps_pablos.h"
#include "ps_assign.h"
#include "ps_if.h"
#include "ps_while.h"


class LLVM_Generator_Helper
{
public:
    static int CarryCount_PabloStatements(std::list<PabloS*> stmts);
private:
    static int CarryCount_PabloS(PabloS* stmt);
    static int CarryCount_PabloE(PabloE* expr);
    LLVM_Generator_Helper();
};

#endif // LLVM_GENERATOR_HELPER_H
