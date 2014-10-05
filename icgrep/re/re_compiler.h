/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_TO_PABLO_COMPILER_H
#define RE_TO_PABLO_COMPILER_H

#include <pablo/codegenstate.h>
#include <re/re_re.h>

#include <string>
#include <list>
#include <map>

namespace re {
class RE_Compiler {
public:

    RE_Compiler(pablo::CodeGenState & baseCG, std::map<std::string, std::string> name_map);

    void compile(RE * re);

private:
    pablo::Assign * process(RE * re, pablo::Assign * target, pablo::CodeGenState & cg_state);
    pablo::Assign * process(Alt * alt, pablo::Assign * target, pablo::CodeGenState & cg_state);
    pablo::Assign * process(Seq * seq, pablo::Assign * target, pablo::CodeGenState & cg_state);
    pablo::Assign * process(Rep * rep, pablo::Assign * target, pablo::CodeGenState & cg_state);
    pablo::Assign * processUnboundedRep(RE * repeated, int lb, pablo::Assign * target, pablo::CodeGenState & cg_state);
    pablo::Assign * processBoundedRep(RE * repeated, int lb, int ub, pablo::Assign * target, pablo::CodeGenState & cg_state);
    pablo::Assign * process(Name * name, pablo::Assign * target, pablo::CodeGenState & cg_state);

    static bool hasUnicode(const RE *re);

    pablo::CodeGenState &                      mBaseCG;
    std::map<std::string, std::string>  m_name_map;
};

}

#endif // COMPILER_H
