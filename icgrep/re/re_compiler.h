/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_TO_PABLO_COMPILER_H
#define RE_TO_PABLO_COMPILER_H

#include <re/re_re.h>
#include <cc/cc_compiler.h>

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

/*  Marker streams represent the results of matching steps.
    Two types of marker streams are used internally.
    FinalByte markers are used for character classes and
    other strings by a one bit at their final position.
    PostPosition markers are used to mark matches with
    a 1 bit immediately after a match.   PostPosition markers
    are generally required whenever a regular expression element
    can match the empty string (e.g., * and ? repeated items).
*/
    
namespace re {

enum MarkerPosition {FinalByte, PostPosition};

struct MarkerType { 
    MarkerPosition pos;
    pablo::Assign * stream;
};

inline bool isPostPositionMarker(MarkerType m) {
    return m.pos == PostPosition;
}

inline bool isFinalPositionMarker(MarkerType m) {
    return m.pos == FinalByte;
}

MarkerType makePostPositionMarker(std::string marker_name, pablo::PabloAST * s, pablo::PabloBlock & pb);

MarkerType makeFinalPositionMarker(std::string marker_name, pablo::PabloAST * s, pablo::PabloBlock & pb);

pablo::Assign * markerStream(MarkerType m, pablo::PabloBlock & pb);

pablo::Var * markerVar(MarkerType m, pablo::PabloBlock & pb);

pablo::Var * postPositionVar(MarkerType m, pablo::PabloBlock & pb);

class RE_Compiler {
public:

    RE_Compiler(pablo::PabloBlock & baseCG, const cc::CC_NameMap & nameMap);
    void initializeRequiredStreams(cc::CC_Compiler & ccc);
    void finalizeMatchResult(MarkerType match_result);
    MarkerType compile(RE * re) {
        return compile(re, mCG);
    }

private:

    MarkerType compile(RE * re, pablo::PabloBlock & cg);

    pablo::PabloAST * character_class_strm(Name * name, pablo::PabloBlock & pb);
    pablo::PabloAST * nextUnicodePosition(MarkerType m, pablo::PabloBlock & pb);
    MarkerType process(RE * re, MarkerType marker, pablo::PabloBlock & pb);
    MarkerType process(Name * name, MarkerType marker, pablo::PabloBlock & pb);
    MarkerType process(Seq * seq, MarkerType marker, pablo::PabloBlock & pb);
    MarkerType process(Alt * alt, MarkerType marker, pablo::PabloBlock & pb);
    MarkerType process(Rep * rep, MarkerType marker, pablo::PabloBlock & pb);
    MarkerType process(Diff * diff, MarkerType marker, pablo::PabloBlock & cg);
    MarkerType process(Intersect * x, MarkerType marker, pablo::PabloBlock & cg);
    pablo::Assign * consecutive(pablo::Assign * repeated,  int repeated_lgth, int repeat_count, pablo::PabloBlock & pb);
    static bool isFixedLength(RE * regexp);
    MarkerType processLowerBound(RE * repeated,  int lb, MarkerType marker, pablo::PabloBlock & pb);
    MarkerType processUnboundedRep(RE * repeated, MarkerType marker, pablo::PabloBlock & pb);
    MarkerType processBoundedRep(RE * repeated, int ub, MarkerType marker, pablo::PabloBlock & pb);

    pablo::PabloBlock &                             mCG;
    const cc::CC_NameMap &                          mNameMap;
    pablo::Var *                                    mLineFeed;
    pablo::PabloAST *                               mCRLF;
    pablo::PabloAST *                               mUnicodeLineBreak;
    pablo::PabloAST *                               mInitial;
    pablo::PabloAST *                               mNonFinal;    
};

}

#endif // COMPILER_H
