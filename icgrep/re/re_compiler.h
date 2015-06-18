/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_TO_PABLO_COMPILER_H
#define RE_TO_PABLO_COMPILER_H

#include <re/re_re.h>
#include <re/re_seq.h>
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

/*   Marker streams represent the results of matching steps.
     Three types of marker streams are used internally.
     FinalMatchByte markers are used for character classes and
     other strings identified by a one bit at their final position.
     InitialPostPositionByte markers are used to mark matches with
     a 1 bit immediately after a match.   InitialPostPositionByte markers
     are generally required whenever a regular expression element
     can match the empty string (e.g., * and ? repeated items).
     FinalPostPositionByte markers are used for single code unit
     lookahead assertions.  
*/

namespace re {

enum MarkerPosition {FinalMatchByte, InitialPostPositionByte, FinalPostPositionByte};

struct MarkerType { 
    MarkerPosition pos;
    pablo::PabloAST * stream;
};

inline MarkerPosition markerPos(MarkerType m) {return m.pos;}

inline pablo::PabloAST * markerVar(MarkerType m) {return m.stream;}
    
inline MarkerType makeMarker(MarkerPosition newpos, pablo::PabloAST * strm) {return {newpos, strm};}


class RE_Compiler {
public:

    RE_Compiler(pablo::PabloBlock & baseCG);
    void initializeRequiredStreams(cc::CC_Compiler & ccc);
    void finalizeMatchResult(MarkerType match_result);
    MarkerType compile(RE * re) {
        return compile(re, mPB);
    }

private:

    MarkerType compile(RE * re, pablo::PabloBlock & cg);
    MarkerType AdvanceMarker(MarkerType m, MarkerPosition newpos, pablo::PabloBlock & pb);
    
    void AlignMarkers(MarkerType & m1, MarkerType & m2, pablo::PabloBlock & pb);
    
    pablo::PabloAST * getNamedCharacterClassStream(Name * name);
    pablo::PabloAST * nextUnicodePosition(MarkerType m, pablo::PabloBlock & pb);
    MarkerType process(RE * re, MarkerType marker, pablo::PabloBlock & pb);
    MarkerType process(Name * name, MarkerType marker, pablo::PabloBlock & pb);
    MarkerType process(Seq * seq, MarkerType marker, pablo::PabloBlock & pb);
    MarkerType processSeqTail(Seq::iterator current, Seq::iterator end, int matchLenSoFar, MarkerType marker, pablo::PabloBlock & pb);
    MarkerType process(Alt * alt, MarkerType marker, pablo::PabloBlock & pb);
    MarkerType process(Assertion * a, MarkerType marker, pablo::PabloBlock & pb);
    MarkerType process(Rep * rep, MarkerType marker, pablo::PabloBlock & pb);
    MarkerType process(Diff * diff, MarkerType marker, pablo::PabloBlock & cg);
    MarkerType process(Intersect * x, MarkerType marker, pablo::PabloBlock & cg);
    pablo::PabloAST *consecutive1(pablo::PabloAST *repeated,  int repeated_lgth, int repeat_count, pablo::PabloBlock & pb);
    pablo::PabloAST * reachable(pablo::PabloAST * repeated,  int repeated_lgth, int repeat_count, pablo::PabloBlock & pb);
    static bool isFixedLength(RE * regexp);
    MarkerType processLowerBound(RE * repeated,  int lb, MarkerType marker, pablo::PabloBlock & pb);
    MarkerType processUnboundedRep(RE * repeated, MarkerType marker, pablo::PabloBlock & pb);
    MarkerType processBoundedRep(RE * repeated, int ub, MarkerType marker, pablo::PabloBlock & pb);

    pablo::PabloBlock &                             mPB;
    pablo::Assign *                                 mLineFeed;
    pablo::PabloAST *                               mCRLF;
    pablo::PabloAST *                               mUnicodeLineBreak;
    pablo::PabloAST *                               mInitial;
    pablo::Assign *                                 mNonFinal;
    pablo::PabloAST *                               mFinal;
    pablo::PabloAST *                               mWhileTest;
    int                                             mStarDepth;
};

}

#endif // COMPILER_H
