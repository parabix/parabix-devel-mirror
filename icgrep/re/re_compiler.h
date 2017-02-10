/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_TO_PABLO_COMPILER_H
#define RE_TO_PABLO_COMPILER_H

#include <re/re_seq.h>  // for Seq
#include <boost/container/flat_map.hpp>
#include <vector>       // for vector<>::iterator
namespace cc { class CC_Compiler; }
namespace pablo { class PabloAST; }
namespace pablo { class PabloBuilder; }
namespace pablo { class PabloKernel; }
namespace re { class Alt; }
namespace re { class Assertion; }
namespace re { class Diff; }
namespace re { class Intersect; }
namespace re { class Name; }
namespace re { class RE; }
namespace re { class Rep; }

//namespace UCD {
//class UnicodeSet;
//}

/*   Marker streams represent the results of matching steps.
     Three types of marker streams are used internally.
     FinalMatchUnit markers are used for character classes and
     other strings identified by a one bit at their final position.
     InitialPostPositionUnit markers are used to mark matches with
     a 1 bit immediately after a match.   InitialPostPositionUnit markers
     are generally required whenever a regular expression element
     can match the empty string (e.g., * and ? repeated items).
     FinalPostPositionUnit markers are used for single code unit
     lookahead assertions.  
*/

namespace re {

enum MarkerPosition {FinalMatchUnit, FinalPostPositionUnit};

struct MarkerType { 
    MarkerPosition pos;
    pablo::PabloAST * stream;
    MarkerType & operator =(const MarkerType &) = default;
};

inline MarkerPosition markerPos(const MarkerType & m) {return m.pos; }

inline pablo::PabloAST * markerVar(const MarkerType & m) {return m.stream; }

inline MarkerType makeMarker(MarkerPosition newpos, pablo::PabloAST * strm) {return {newpos, strm};}


class RE_Compiler {
public:

    RE_Compiler(pablo::PabloKernel * kernel, cc::CC_Compiler & ccCompiler, bool CountOnly = false);
    void initializeRequiredStreams(const unsigned encodingBits);
    void compileUnicodeNames(RE *& re);
    void finalizeMatchResult(MarkerType match_result, bool InvertMatches = false);
    MarkerType compile(RE * re) {
        return compile(re, mPB);
    }

    static LLVM_ATTRIBUTE_NORETURN void UnsupportedRE(std::string errmsg);
    
private:

    struct NameMap {
        NameMap(NameMap * parent = nullptr) : mParent(parent), mMap() {}
        bool get(const Name * name, MarkerType & marker) const {
            auto f = mMap.find(name);
            if (f == mMap.end()) {
                return mParent ? mParent->get(name, marker) : false;
            } else {
                marker = f->second;
                return true;
            }
        }
        void add(const Name * const name, MarkerType marker) {
            mMap.emplace(name, std::move(marker));
        }
        NameMap * getParent() const { return mParent; }
    private:
        NameMap * const mParent;
        boost::container::flat_map<const Name *, MarkerType> mMap;
    };

    void initializeRequiredStreams_utf8();
    void initializeRequiredStreams_utf16();
    MarkerType compile(RE * re, pablo::PabloBuilder & cg);

    MarkerType process(RE * re, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileName(Name * name, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileSeq(Seq * seq, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileSeqTail(Seq::iterator current, const Seq::iterator end, int matchLenSoFar, MarkerType marker, pablo::PabloBuilder & pb);
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

    MarkerType compileName(Name * name, pablo::PabloBuilder & pb);
    MarkerType compileAny(const MarkerType m, pablo::PabloBuilder & pb);
    MarkerType compileStart(const MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileEnd(const MarkerType marker, pablo::PabloBuilder & pb);

    MarkerType AdvanceMarker(MarkerType marker, const MarkerPosition newpos, pablo::PabloBuilder & pb);
    void AlignMarkers(MarkerType & m1, MarkerType & m2, pablo::PabloBuilder & pb);

private:

    pablo::PabloKernel * const                      mKernel;
    bool                                            mCountOnly;
    cc::CC_Compiler &                               mCCCompiler;
    pablo::PabloAST *                               mLineBreak;
    pablo::PabloAST *                               mCRLF;
    pablo::PabloAST *                               mAny;
    pablo::PabloAST *                               mGraphemeBoundaryRule;
    pablo::PabloAST *                               mInitial;
    pablo::PabloAST *                               mNonFinal;
    pablo::PabloAST *                               mFinal;
    pablo::PabloAST *                               mWhileTest;
    int                                             mStarDepth;
    pablo::PabloBuilder &                           mPB;
    NameMap *                                       mCompiledName;
    NameMap                                         mBaseMap;
};

}

#endif // COMPILER_H
