/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_TO_PABLO_COMPILER_H
#define RE_TO_PABLO_COMPILER_H

//Regular Expressions
#include "re_re.h"
//Pablo Statements
#include "../ps_pablos.h"
//Code Generation
#include "symbol_generator.h"

#include <string>
#include <list>
#include <map>

namespace re {
class RE_Compiler {
public:
    RE_Compiler(std::map<std::string, std::string> name_map);
    CodeGenState compile(RE *re);
    CodeGenState compile_subexpressions(const std::map<std::string, RE*>& re_map);
private:
    void compile(RE * re, CodeGenState & cg_state);
    void compile(Alt * alt, CodeGenState & cg_state);
    void compile(Seq * seq, CodeGenState & cg_state);
    void compile(Rep * rep, CodeGenState & cg_state);
    void compileUnboundedRep(RE * repeated, int lb, CodeGenState  & cg_state);
    void compileBoundedRep(RE * repeated, int lb, int ub, CodeGenState &cg_state);
    void compile(Name * name, CodeGenState & cg_state);



    static bool hasUnicode(const RE *re);

    SymbolGenerator symgen;
    std::map<std::string, std::string> m_name_map;
};

}

#endif // COMPILER_H
