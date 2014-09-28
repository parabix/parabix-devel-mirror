/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef COMPILER_H
#define COMPILER_H

//Regular Expressions
#include "re/re_re.h"
//Pablo Statements
#include "ps_pablos.h"
//Code Generation
#include "symbol_generator.h"

#include <string>
#include <list>
#include <map>


struct CodeGenState{
    std::list<PabloS*> stmtsl;
    std::string newsym;
};

class Pbix_Compiler
{
    typedef re::RE                  RE;
    typedef re::Vector              Vector;
    typedef Vector::const_iterator  const_iterator;

public:
    Pbix_Compiler(std::map<std::string, std::string> name_map);
    CodeGenState compile(RE *re);
    CodeGenState compile_subexpressions(const std::map<std::string, RE*>& re_map);
private:
    CodeGenState re2pablo_helper(RE *re, CodeGenState cg_state);
    CodeGenState Seq_helper(Vector * lst, const_iterator it, CodeGenState cg_state);
    CodeGenState Alt_helper(Vector * lst, const_iterator it, CodeGenState cg_state);
    CodeGenState UnboundedRep_helper(RE* repeated, int lb, CodeGenState cg_state);
    CodeGenState BoundedRep_helper(RE* repeated, int lb, int ub, CodeGenState cg_state);

    static bool hasUnicode(const RE *re);

    SymbolGenerator symgen;
    std::map<std::string, std::string> m_name_map;
};

#endif // COMPILER_H
