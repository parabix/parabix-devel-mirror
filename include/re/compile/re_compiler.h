/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_TO_PABLO_COMPILER_H
#define RE_TO_PABLO_COMPILER_H

#include <vector>                       // for vector<>::iterator
#include <boost/container/flat_map.hpp>
#include <pablo/builder.hpp>
#include <re/adt/adt.h>              // for Seq
#include <re/alphabet/alphabet.h>
#include <re/transforms/to_utf8.h>

namespace cc { class CC_Compiler; class Alphabet;}
namespace pablo { class PabloAST; }
namespace pablo { class PabloBlock; }
namespace pablo { class Var; }

namespace re {

class RE_Compiler {
    public:

/*   The regular expression compiler works in terms of two fundamental bit stream
     concepts: index streams and marker streams.

     Index streams mark positions corresponding to whole matching units.
     For example, if the matching units are UTF-8 sequences, then the index
     stream identifies the last byte of each UTF-8 sequence denoting a single
     Unicode codepoint.   If the matching units are UTF-16 sequences then the
     index stream will have one bits on every UTF-16 code unit that denotes
     a full Unicode character itself, as well as on the final code unit of
     a surrogate pair.

     Index streams may be defined with respect to a particular regular expression.
     For example, an index stream with respect to an regular expression involved
     in a bounded repetition may mark the last code unit of substrings matching
     a so-called characteristic subexpression of the bounded repetition.
     As a characteristic subexpression is a subexpression that must be matched
     once and only once for the repetition, the index stream may be used to
     count the number of repetitions.

     If the unit of matching is defined in terms of code units (e.g., UTF-8 bytes
     or UTF-16 double bytes) then the correspoinding index stream is the stream
     of all one bits.

     Marker streams represent the results of matching steps.
     Markers have an offset with respect to compiled regular expressions.
     Offset 0 means that each marker bit is placed at a position corresponding
     to the last code unit of a matched substring.

     Offsets are expressed with respect to an index stream.   An offset N > 0
     means that the marker is placed N positions further along the index stream
     than the last code unit of the matched substring.

     An offset of 1 with an index stream of all ones means that marker stream
     is placed on the code unit immediately after the last code unit matched.  The offset
     1 naturally arises whenever a regular expression can match the empty string.

     Non-zero offsets also arise in matching lookahead assertions.  For example,
     Unicode boundary assertions such as word boundaries or grapheme cluster
     boundaries typically involving lookahead at one or two Unicode characters.
     If the underlying streams are based on UTF-8 code units, then the index
     stream for such an offset is the the stream marking the final bytes of
     UTF-8 code unit sequences.  */

    class Marker {
    public:
        Marker(pablo::PabloAST * strm, unsigned offset = 0) : mOffset(offset), mStream(strm) {}
        Marker & operator =(const Marker &) = default;
        unsigned offset() {return mOffset;}
        pablo::PabloAST * stream() {return mStream;}
    private:
        unsigned mOffset;
        pablo::PabloAST * mStream;
    };

    RE_Compiler(pablo::PabloBlock * scope,
                const cc::Alphabet * codeUnitAlphabet = &cc::UTF8);

    void addIndexingAlphabet(EncodingTransformer * indexingTransformer, pablo::PabloAST * idxStream);
    
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

    void addAlphabet(const cc::Alphabet * a, std::vector<pablo::PabloAST* > basis_set);

    void addAlphabet(const std::shared_ptr<cc::Alphabet> & a, std::vector<pablo::PabloAST* > basis_set) {
        addAlphabet(a.get(), basis_set);
    }
    
    void addPrecompiled(std::string precompiledName, Marker precompiled);

    Marker compileRE(RE * re);
    
    Marker compileRE(RE * re, Marker initialMarkers);
    
    static LLVM_ATTRIBUTE_NORETURN void UnsupportedRE(std::string errmsg);
    
private:

    struct NameMap {
        NameMap(NameMap * parent = nullptr) : mParent(parent), mMap() {}
        bool get(const Name * name, Marker & marker) const {
            auto f = mMap.find(name);
            if (f == mMap.end()) {
                return mParent ? mParent->get(name, marker) : false;
            } else {
                marker = f->second;
                return true;
            }
        }
        void add(const Name * const name, Marker marker) {
            mMap.emplace(name, std::move(marker));
        }
        NameMap * getParent() const { return mParent; }
    private:
        NameMap * const mParent;
        boost::container::flat_map<const Name *, Marker> mMap;
    };


    Marker compile(RE * re, pablo::PabloBuilder & pb);
    Marker compile(RE * re, Marker initialMarkers, pablo::PabloBuilder & pb);

    Marker process(RE * re, Marker marker, pablo::PabloBuilder & pb);
    Marker compileName(Name * name, Marker marker, pablo::PabloBuilder & pb);
    Marker compileCC(CC * cc, Marker marker, pablo::PabloBuilder & pb);
    Marker compileSeq(Seq * seq, Marker marker, pablo::PabloBuilder & pb);
    Marker compileSeqTail(Seq::const_iterator current, const Seq::const_iterator end, int matchLenSoFar, Marker marker, pablo::PabloBuilder & pb);
    Marker compileAlt(Alt * alt, Marker base, pablo::PabloBuilder & pb);
    Marker compileAssertion(Assertion * a, Marker marker, pablo::PabloBuilder & pb);
    Marker compileRep(int LB, int UB, RE * repeated, Marker marker, pablo::PabloBuilder & pb);
    Marker compileDiff(Diff * diff, Marker marker, pablo::PabloBuilder & cg);
    Marker compileIntersect(Intersect * x, Marker marker, pablo::PabloBuilder & cg);
    pablo::PabloAST * consecutive_matches(pablo::PabloAST * repeated_j, int j, int repeat_count, const int match_length, pablo::PabloAST * indexStream, pablo::PabloBuilder & pb);
    pablo::PabloAST * reachable(pablo::PabloAST * repeated, int length, int repeat_count, pablo::PabloAST * indexStream, pablo::PabloBuilder & pb);
    static bool isFixedLength(RE * regexp);
    Marker expandLowerBound(RE * repeated,  int lb, Marker marker, int ifGroupSize, pablo::PabloBuilder & pb);
    Marker processUnboundedRep(RE * repeated, Marker marker, pablo::PabloBuilder & pb);
    Marker expandUpperBound(RE * repeated, int ub, Marker marker, int ifGroupSize,  pablo::PabloBuilder & pb);

    Marker compileName(Name * name, pablo::PabloBuilder & pb);
    Marker compileStart(Marker marker, pablo::PabloBuilder & pb);
    Marker compileEnd(Marker marker, pablo::PabloBuilder & pb);

    Marker AdvanceMarker(Marker marker, const unsigned newpos, pablo::PabloBuilder & pb);
    void AlignMarkers(Marker & m1, Marker & m2, pablo::PabloBuilder & pb);
    
private:

    pablo::PabloBlock * const                       mEntryScope;
    const cc::Alphabet *                            mCodeUnitAlphabet;
    EncodingTransformer *                           mIndexingTransformer;
    pablo::PabloAST *                               mIndexStream;
    std::vector<const cc::Alphabet *>               mAlphabets;
    std::vector<std::vector<pablo::PabloAST *>>     mBasisSets;
    std::vector<std::unique_ptr<cc::CC_Compiler>>   mAlphabetCompilers;
    pablo::PabloAST *                               mWhileTest;
    int                                             mStarDepth;
    NameMap *                                       mCompiledName;
    NameMap                                         mBaseMap;
    std::map<std::string, pablo::PabloAST *>        mExternalNameMap;
};

}

#endif // COMPILER_H
