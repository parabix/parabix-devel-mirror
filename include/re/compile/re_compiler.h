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

class RE_Block_Compiler;

class RE_Compiler {
    friend class RE_Block_Compiler;
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

    //
    // The regular expression compiler may include one or more externally
    // defined Names.   Each name has a length, expressed in terms of the
    // matching units defined by the index stream, and may also have a
    // nonzero offset.   Note that external names that correspond to Unicode
    // boundaries or other zero-width assertions will have a length of 0.
    //
    class ExternalStream {
    public:
        ExternalStream(Marker m, unsigned lgth = 1) :
            mMarker(m), mLength(lgth) {}
        ExternalStream & operator = (const ExternalStream &) = default;
        unsigned length() {return mLength;}
        Marker & marker() {return mMarker;}
    private:
        Marker mMarker;
        unsigned mLength;
    };

    void addPrecompiled(std::string externalName, ExternalStream s);

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
    
    Marker compileRE(RE * re);
    
    Marker compileRE(RE * re, Marker initialMarkers);
        
    static LLVM_ATTRIBUTE_NORETURN void UnsupportedRE(std::string errmsg);

private:
    using ExternalNameMap = std::map<std::string, ExternalStream>;
    pablo::PabloBlock * const                       mEntryScope;
    const cc::Alphabet *                            mCodeUnitAlphabet;
    EncodingTransformer *                           mIndexingTransformer;
    pablo::PabloAST *                               mIndexStream;
    std::vector<const cc::Alphabet *>               mAlphabets;
    std::vector<std::vector<pablo::PabloAST *>>     mBasisSets;
    std::vector<std::unique_ptr<cc::CC_Compiler>>   mAlphabetCompilers;
    pablo::PabloAST *                               mWhileTest;
    int                                             mStarDepth;
    ExternalNameMap                                 mExternalNameMap;
};

}

#endif // COMPILER_H
