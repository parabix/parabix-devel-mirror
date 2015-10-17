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

namespace pablo {
class PabloFunction;
}

namespace UCD {
class UnicodeSet;
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

    RE_Compiler(pablo::PabloFunction & function, cc::CC_Compiler & ccCompiler);
    void initializeRequiredStreams();
    void compileUnicodeNames(RE *& re);
    void finalizeMatchResult(MarkerType match_result);
    MarkerType compile(RE * re) {
        return compile(re, mPB);
    }

private:

    MarkerType compile(RE * re, pablo::PabloBuilder & cg);

    MarkerType process(RE * re, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileName(Name * name, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileSeq(Seq * seq, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileSeqTail(Seq::iterator current, Seq::iterator end, int matchLenSoFar, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileAlt(Alt * alt, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileAssertion(Assertion * a, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileRep(Rep * rep, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileDiff(Diff * diff, MarkerType marker, pablo::PabloBuilder & cg);
    MarkerType compileIntersect(Intersect * x, MarkerType marker, pablo::PabloBuilder & cg);
    pablo::PabloAST * consecutive_matches(pablo::PabloAST * repeated,  int length, int repeat_count, pablo::PabloBuilder & pb);
    pablo::PabloAST * reachable(pablo::PabloAST * repeated,  int length, int repeat_count, pablo::PabloBuilder & pb);
    static bool isFixedLength(RE * regexp);
    MarkerType processLowerBound(RE * repeated,  int lb, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType processUnboundedRep(RE * repeated, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType processBoundedRep(RE * repeated, int ub, MarkerType marker, pablo::PabloBuilder & pb);
    RE * resolveUnicodeProperties(RE * re);

    Name * generateGraphemeClusterBoundaryRule();
    MarkerType compileName(Name * name, pablo::PabloBuilder & pb);
    MarkerType compileAny(const MarkerType m, pablo::PabloBuilder & pb);
    MarkerType compileStart(const MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileEnd(const MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileGraphemeBoundary(GraphemeBoundary *gb, MarkerType marker, pablo::PabloBuilder & pb);

    MarkerType AdvanceMarker(MarkerType marker, const MarkerPosition newpos, pablo::PabloBuilder & pb);
    void AlignMarkers(MarkerType & m1, MarkerType & m2, pablo::PabloBuilder & pb);

private:

    cc::CC_Compiler &                               mCCCompiler;
    pablo::Assign *                                 mLineFeed;
    pablo::PabloAST *                               mCRLF;
    pablo::PabloAST *                               mUnicodeLineBreak;
    pablo::PabloAST *                               mAny;
    pablo::PabloAST *                               mGraphemeBoundaryRule;
    pablo::PabloAST *                               mInitial;
    pablo::Assign *                                 mNonFinal;    
    pablo::PabloAST *                               mFinal;
    pablo::PabloAST *                               mWhileTest;
    int                                             mStarDepth;
    std::vector<pablo::Next *>                      mLoopVariants; // <- rethink name
    pablo::PabloBuilder                             mPB;
    std::unordered_map<Name *, MarkerType>          mCompiledName;
    pablo::PabloFunction &                          mFunction;
};

}

#endif // COMPILER_H
