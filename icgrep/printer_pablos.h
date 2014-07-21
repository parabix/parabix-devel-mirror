/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef SHOW_H
#define SHOW_H

//Regular Expressions
#include "re_re.h"
#include "re_cc.h"
#include "re_start.h"
#include "re_end.h"
#include "re_seq.h"

//Pablo Expressions
#include "pe_pabloe.h"
#include "pe_sel.h"
#include "pe_advance.h"
#include "pe_all.h"
#include "pe_and.h"
#include "pe_charclass.h"
#include "pe_not.h"
#include "pe_or.h"
#include "pe_var.h"
#include "pe_xor.h"
#include "pe_matchstar.h"

//Pablo Statements
#include "ps_pablos.h"
#include "ps_assign.h"
#include "ps_if.h"

#include "pbix_compiler.h"

//Code Generation
#include "symbol_generator.h"

#include <iostream>
#include <string>
#include <sstream>
#include <list>

#define INT2STRING(i) static_cast<std::ostringstream*>(&(std::ostringstream() << i))->str()

class StatementPrinter
{
public:
    static std::string PrintStmts(CodeGenState cg_state);
    static std::string Print_CC_PabloStmts(std::list<PabloS*> stmts);
    static std::string Print_PB_PabloStmts(std::list<PabloS*> stmts, std::string strOut);
    static std::string ShowPabloE(PabloE* expr);
    static std::string ShowPabloS(PabloS* stmt);
private:
    StatementPrinter();
};

#endif // SHOW_H
