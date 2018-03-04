/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_TO_PABLO_COMPILER_H
#define RE_TO_PABLO_COMPILER_H

#include <re/re_seq.h>  // for Seq
#include <boost/container/flat_map.hpp>
#include <pablo/builder.hpp>
#include <vector>       // for vector<>::iterator
namespace cc { class CC_Compiler; class Alphabet;}
namespace pablo { class PabloAST; }
namespace pablo { class PabloBlock; }
namespace pablo { class Var; }
namespace re { class Alt; }
namespace re { class Assertion; }
namespace re { class Diff; }
namespace re { class Intersect; }
namespace re { class Name; }
namespace re { class RE; }
namespace re { class Rep; }
namespace re { class CC; }

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

class RE_Compiler {
public:

    enum MarkerPosition {FinalMatchUnit, InitialPostPositionUnit, FinalPostPositionUnit};

    struct MarkerType {
        MarkerPosition pos;
        pablo::PabloAST * stream;
        MarkerType & operator =(const MarkerType &) = default;
    };

    RE_Compiler(pablo::PabloBlock * scope, cc::CC_Compiler & ccCompiler);
    
    //
    // The CCs (character classes) within a regular expression are generally
    // expressed using a single alphabet.   But multiple alphabets may be
    // used under some circumstances.   For example, regular expressions for
    // Unicode may use both the Unicode alphabet for full Unicode characters
    // as well as the Byte alphabet for the individual code units of UTF-8.
    // In other cases, a multiplexed alphabet may be used for a certain
    // subexpression, for example, if the subexpression involves a local
    // language or a capture-backreference combination.
    //
    // Alphabets are added as needed using the addAlphabet method, giving both
    // the alphabet value and the set of parallel bit streams that comprise
    // a basis for the coded alphabet values.

    void addAlphabet(cc::Alphabet * a, std::vector<pablo::PabloAST* > basis_set);
    
    void addPrecompiled(std::string precompiledName, pablo::PabloAST * precompiledStream);

    pablo::PabloAST * compile(RE * re, pablo::PabloAST * const initialCursors = nullptr);

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


    MarkerType compile(RE * re, pablo::PabloBuilder & pb);
    MarkerType compile(RE * re, pablo::PabloAST * const cursors, pablo::PabloBuilder & pb);

    MarkerType process(RE * re, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileName(Name * name, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileCC(CC * cc, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileSeq(Seq * seq, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileSeqTail(Seq::const_iterator current, const Seq::const_iterator end, int matchLenSoFar, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileAlt(Alt * alt, MarkerType base, pablo::PabloBuilder & pb);
    MarkerType compileAssertion(Assertion * a, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileRep(Rep * rep, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileDiff(Diff * diff, MarkerType marker, pablo::PabloBuilder & cg);
    MarkerType compileIntersect(Intersect * x, MarkerType marker, pablo::PabloBuilder & cg);
    pablo::PabloAST * consecutive_matches(pablo::PabloAST * repeated_j, int j, int repeat_count, pablo::PabloAST * indexStream, pablo::PabloBuilder & pb);
    pablo::PabloAST * reachable(pablo::PabloAST * repeated, int length, int repeat_count, pablo::PabloAST * indexStream, pablo::PabloBuilder & pb);
    static bool isFixedLength(RE * regexp);
    MarkerType processLowerBound(RE * repeated,  int lb, MarkerType marker, int ifGroupSize, pablo::PabloBuilder & pb);
    MarkerType processUnboundedRep(RE * repeated, MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType processBoundedRep(RE * repeated, int ub, MarkerType marker, int ifGroupSize,  pablo::PabloBuilder & pb);

    MarkerType compileName(Name * name, pablo::PabloBuilder & pb);
    MarkerType compileAny(const MarkerType m, pablo::PabloBuilder & pb);
    MarkerType compileStart(MarkerType marker, pablo::PabloBuilder & pb);
    MarkerType compileEnd(MarkerType marker, pablo::PabloBuilder & pb);

    MarkerType AdvanceMarker(MarkerType marker, const MarkerPosition newpos, pablo::PabloBuilder & pb);
    void AlignMarkers(MarkerType & m1, MarkerType & m2, pablo::PabloBuilder & pb);

    static inline MarkerPosition markerPos(const MarkerType & m) {return m.pos; }
    static inline pablo::PabloAST * markerVar(const MarkerType & m) {return m.stream; }
    static inline MarkerType makeMarker(MarkerPosition newpos, pablo::PabloAST * strm) {return {newpos, strm};}

private:

    pablo::PabloBlock * const                       mEntryScope;
    std::vector<cc::Alphabet *>                     mAlphabets;
    std::vector<std::unique_ptr<cc::CC_Compiler>>   mAlphabetCompilers;

    cc::CC_Compiler &                               mCCCompiler;
    pablo::PabloAST *                               mLineBreak;
    pablo::PabloAST *                               mNonFinal;
    pablo::PabloAST *                               mFinal;
    pablo::PabloAST *                               mWhileTest;
    int                                             mStarDepth;
    NameMap *                                       mCompiledName;
    NameMap                                         mBaseMap;
    std::map<std::string, pablo::PabloAST *>        mExternalNameMap;

};

}

#endif // COMPILER_H
