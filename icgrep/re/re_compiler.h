/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_TO_PABLO_COMPILER_H
#define RE_TO_PABLO_COMPILER_H

#include <re/re_re.h>

#include <string>
#include <list>
#include <map>

namespace pablo {

class PabloBlock;
class PabloAST;
class Assign;
class Var;

}

namespace re {
class RE_Compiler {
public:

    RE_Compiler(pablo::PabloBlock & baseCG, std::map<std::string, std::string> name_map);

    inline void compile(RE * re) {
        compile(re, mCG);
    }

private:

    void compile(RE * re, pablo::PabloBlock & cg);

    pablo::Assign * process(RE * re, pablo::Assign *target, pablo::PabloBlock & cg_state);
    pablo::Assign * process(Name * name, pablo::Assign * target, pablo::PabloBlock & cg_state);
    pablo::Assign * process(Seq * seq, pablo::Assign * target, pablo::PabloBlock & cg_state);
    pablo::Assign * process(Alt * alt, pablo::Assign * target, pablo::PabloBlock & cg_state);
    pablo::Assign * process(Rep * rep, pablo::Assign *target, pablo::PabloBlock & cg_state);
    pablo::Assign * processUnboundedRep(RE * repeated, int lb, pablo::Assign * target, pablo::PabloBlock & cg_state);
    pablo::Assign * processBoundedRep(RE * repeated, int lb, int ub, pablo::Assign * target, pablo::PabloBlock & cg_state);


    static bool hasUnicode(const RE *re);

    pablo::PabloBlock &                             mCG;
    pablo::Var *                                    mLineFeed;
    pablo::PabloAST *                               mInitial;
    pablo::PabloAST *                               mNonFinal;
    std::map<std::string, std::string>              m_name_map;
};

}

#endif // COMPILER_H
