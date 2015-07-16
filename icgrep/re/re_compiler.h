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
#include <pablo/builder.hpp>
#include <UCD/ucd_compiler.hpp>
#include <string>
#include <list>
#include <map>

namespace cc {
class CC_NameMap;
}

namespace pablo {
class PabloFunction;
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

bool UsePregeneratedUnicode();

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

    RE_Compiler(pablo::PabloFunction & function, cc::CC_Compiler & ccCompiler);
    void initializeRequiredStreams();
    void finalizeMatchResult(MarkerType match_result);
    MarkerType compile(RE * re) {
        return compile(re, mPB);
    }

private:

    MarkerType compile(RE * re, pablo::PabloBuilder & cg);
    MarkerType AdvanceMarker(MarkerType m, MarkerPosition newpos, pablo::PabloBuilder & pb);
    
    void AlignMarkers(MarkerType & m1, MarkerType & m2, pablo::PabloBuilder & pb);
    
    pablo::PabloAST * getNamedCharacterClassStream(Name * name, pablo::PabloBuilder & pb);
    pablo::PabloAST * nextUnicodePosition(MarkerType m, pablo::PabloBuilder & pb);
    MarkerType process(RE * re, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType process(Name * name, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType process(Seq * seq, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType processSeqTail(Seq::iterator current, Seq::iterator end, int matchLenSoFar, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType process(Alt * alt, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType process(Assertion * a, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType process(Rep * rep, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType process(Diff * diff, MarkerType marker, pablo::PabloBuilder & cg);
    MarkerType process(Intersect * x, MarkerType marker, pablo::PabloBuilder & cg);
    pablo::PabloAST *consecutive1(pablo::PabloAST *repeated,  int repeated_lgth, int repeat_count, pablo::PabloBuilder & pb);
    pablo::PabloAST * reachable(pablo::PabloAST * repeated,  int repeated_lgth, int repeat_count, pablo::PabloBuilder & pb);
    static bool isFixedLength(RE * regexp);
    MarkerType processLowerBound(RE * repeated,  int lb, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType processUnboundedRep(RE * repeated, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType processBoundedRep(RE * repeated, int ub, MarkerType marker, pablo::PabloBuilder & pb);

private:

    cc::CC_Compiler &                               mCCCompiler;
    pablo::Assign *                                 mLineFeed;
    pablo::PabloAST *                               mCRLF;
    pablo::PabloAST *                               mUnicodeLineBreak;
    pablo::PabloAST *                               mInitial;
    pablo::Assign *                                 mNonFinal;
    pablo::PabloAST *                               mFinal;
    pablo::PabloAST *                               mWhileTest;
    int                                             mStarDepth;
    std::vector<pablo::Next *>                      mLoopVariants; // <- rethink name
    pablo::PabloBuilder                             mPB;
    UCD::UCDCompiler                                mUCDCompiler;
    pablo::PabloFunction &                          mFunction;
};

}

#endif // COMPILER_H
