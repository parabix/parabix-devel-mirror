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

namespace cc {
class CC_NameMap;
}

namespace pablo {
class PabloBlock;
class PabloAST;
class Assign;
class Var;
}

namespace re {
class RE_Compiler {
public:

    RE_Compiler(pablo::PabloBlock & baseCG, const cc::CC_NameMap & nameMap);

    inline void compile(RE * re) {
        compile(re, mCG);
    }

private:

    void compile(RE * re, pablo::PabloBlock & cg);

    pablo::PabloAST * character_class_strm(Name * name, pablo::PabloBlock & pb);
    pablo::Assign * process(RE * re, pablo::Assign *marker, pablo::PabloBlock & pb);
    pablo::Assign * process(Name * name, pablo::Assign * marker, pablo::PabloBlock & pb);
    pablo::Assign * process(Seq * seq, pablo::Assign * marker, pablo::PabloBlock & pb);
    pablo::Assign * process(Alt * alt, pablo::Assign * marker, pablo::PabloBlock & pb);
    pablo::Assign * process(Rep * rep, pablo::Assign *marker, pablo::PabloBlock & pb);
    pablo::Assign * process(Diff * diff, pablo::Assign * marker, pablo::PabloBlock & cg);
    pablo::Assign * process(Intersect * x, pablo::Assign * marker, pablo::PabloBlock & cg);
    pablo::Assign * consecutive(pablo::Assign * repeated,  int repeated_lgth, int repeat_count, pablo::PabloBlock & pb);
    static bool isFixedLength(RE * regexp);
    pablo::Assign * processLowerBound(RE * repeated,  int lb, pablo::Assign * marker, pablo::PabloBlock & pb);
    pablo::Assign * processUnboundedRep(RE * repeated, pablo::Assign * marker, pablo::PabloBlock & pb);
    pablo::Assign * processBoundedRep(RE * repeated, int ub, pablo::Assign * marker, pablo::PabloBlock & pb);

    pablo::PabloBlock &                             mCG;
    const cc::CC_NameMap &                          mNameMap;
    pablo::Var *                                    mLineFeed;
    pablo::PabloAST *                               mInitial;
    pablo::PabloAST *                               mNonFinal;    
};

}

#endif // COMPILER_H
