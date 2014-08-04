/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef COMPILER_H
#define COMPILER_H

//Regular Expressions
#include "re_re.h"
#include "re_name.h"
#include "re_start.h"
#include "re_end.h"
#include "re_seq.h"
#include "re_alt.h"
#include "re_rep.h"

//Pablo Expressions
#include "pe_pabloe.h"
#include "pe_sel.h"
#include "pe_advance.h"
#include "pe_all.h"
#include "pe_and.h"
#include "pe_charclass.h"
#include "pe_call.h"
#include "pe_matchstar.h"
#include "pe_scanthru.h"
#include "pe_not.h"
#include "pe_or.h"
#include "pe_var.h"
#include "pe_xor.h"

//Pablo Statements
#include "ps_pablos.h"
#include "ps_assign.h"
#include "ps_if.h"
#include "ps_while.h"

//Code Generation
#include "symbol_generator.h"

#include <iostream>
#include <string>
#include <sstream>
#include <list>
#include <vector>
#include <map>


struct CodeGenState{
    std::list<PabloS*> stmtsl;
    std::string newsym;
};

class Pbix_Compiler
{
public:
    Pbix_Compiler(std::map<std::string, std::string> name_map);
    CodeGenState compile(RE *re);
    CodeGenState compile_subexpressions(const std::map<std::string, RE*>& re_map);
private:
    CodeGenState re2pablo_helper(RE *re, CodeGenState cg_state);
    CodeGenState Seq_helper(std::list<RE*>* lst, std::list<RE*>::const_iterator it, CodeGenState cg_state);
    CodeGenState Alt_helper(std::list<RE*>* lst, std::list<RE*>::const_iterator it, CodeGenState cg_state);

    SymbolGenerator symgen;
    std::map<std::string, std::string> m_name_map;
};

#endif // COMPILER_H
