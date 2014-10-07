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

    RE_Compiler(pablo::PabloBlock & baseCG, std::map<std::string, std::string> name_map);

    inline void compile(RE * re) {
        compile(re, mCG);
    }

private:

    void compile(RE * re, pablo::PabloBlock & cg);

    pablo::PabloE * process(RE * re, pablo::PabloE * target, pablo::PabloBlock & cg_state);
    pablo::PabloE * process(Alt * alt, pablo::PabloE * target, pablo::PabloBlock & cg_state);
    pablo::PabloE * process(Seq * seq, pablo::PabloE * target, pablo::PabloBlock & cg_state);
    pablo::PabloE * process(Rep * rep, pablo::PabloE * target, pablo::PabloBlock & cg_state);
    pablo::PabloE * processUnboundedRep(RE * repeated, int lb, pablo::PabloE * target, pablo::PabloBlock & cg_state);
    pablo::PabloE * processBoundedRep(RE * repeated, int lb, int ub, pablo::PabloE * target, pablo::PabloBlock & cg_state);
    pablo::PabloE * process(Name * name, pablo::PabloE * target, pablo::PabloBlock & cg_state);

    static bool hasUnicode(const RE *re);

    pablo::PabloBlock &                      mCG;
    std::map<std::string, std::string>  m_name_map;
};

}

#endif // COMPILER_H
