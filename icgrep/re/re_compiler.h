/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_TO_PABLO_COMPILER_H
#define RE_TO_PABLO_COMPILER_H

#include <pablo/codegenstate.h>
#include <re/re_re.h>
#include <re/symbol_generator.h>

#include <string>
#include <list>
#include <map>

namespace re {
class RE_Compiler {
public:
    RE_Compiler(std::map<std::string, std::string> name_map);
    pablo::CodeGenState compile(RE *re);
    pablo::CodeGenState compile_subexpressions(const std::map<std::string, RE*>& re_map);
private:
    void compile(RE * re, pablo::CodeGenState & cg_state);
    void compile(Alt * alt, pablo::CodeGenState & cg_state);
    void compile(Seq * seq, pablo::CodeGenState & cg_state);
    void compile(Rep * rep, pablo::CodeGenState & cg_state);
    void compileUnboundedRep(RE * repeated, int lb, pablo::CodeGenState  & cg_state);
    void compileBoundedRep(RE * repeated, int lb, int ub, pablo::CodeGenState &cg_state);
    void compile(Name * name, pablo::CodeGenState & cg_state);

    static bool hasUnicode(const RE *re);

    SymbolGenerator symgen;
    std::map<std::string, std::string> m_name_map;
};

}

#endif // COMPILER_H
