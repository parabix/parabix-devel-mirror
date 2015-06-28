#ifndef UNICODE_SET_H
#define UNICODE_SET_H
#include <stdint.h>
#include <vector>
#include <re/re_cc.h>
#include <boost/iterator/iterator_facade.hpp>

//
// unicode_set.h - representing and manipulating sets of Unicode
// characters, based on data from UCD - the Unicode Character Database
//
// Robert D. Cameron
// September 18, 2014
//
// Licensed under Open Software License 3.0.
//
// Unicode Sparse Bitset Representation 
//
// The Unicode Sparse Bitset representation is based on 
// (a) Dividing the Unicode codepoint space into groups of 2^k codepoints called quads.
// (b) Specifying the quads using a run-length encoding, in which each run
//     is Empty (quads contain no members), Mixed (quads contain some members and
//     some nonmembers) or Full (all codepoints in each quad are members of the set). 
// (c) Explicitly listing all the quads of Mixed type.
//

//
// The internal datatype for quads - bitsets of 2^k codepoints.
// Default: 64 codepoints (k=6).
//

namespace llvm {
class raw_ostream;
}

typedef uint32_t bitquad_t;

// The representation for runs
enum run_type_t : uint16_t {Empty, Mixed, Full};

struct RunStructure {
  RunStructure(run_type_t r, uint16_t lgth) : mType(r), mRunLength(lgth) {}
  run_type_t mType;
  uint16_t mRunLength;
};

class UnicodeSet {
public:

    using codepoint_t = re::codepoint_t;
    using interval_t = re::interval_t;
    using RunVector = std::vector<RunStructure>;
    using QuadVector = std::vector<bitquad_t>;

    class iterator : public boost::iterator_facade<iterator, interval_t, boost::forward_traversal_tag, interval_t> {
        friend class UnicodeSet;
        friend class boost::iterator_core_access;
    protected:
        iterator(RunVector::const_iterator runIterator, QuadVector::const_iterator quadIterator)
        : mRunIterator(runIterator), mQuadIterator(quadIterator)
        , mQuadOffset(0), mQuadPosition(0), mBaseCodePoint(0), mMinCodePoint(0), mMaxCodePoint(0)
        {

        }

        void advance(const unsigned n);

        re::interval_t dereference() const {
            return std::make_pair(mMinCodePoint, mMaxCodePoint);
        }

        inline void increment() {
            advance(1);
        }

        inline bool equal(iterator const & other) const {
            return (mRunIterator == other.mRunIterator) && (mQuadIterator == other.mQuadIterator);
        }
    private:
        RunVector::const_iterator           mRunIterator;
        const RunVector::const_iterator     mRunEnd;
        QuadVector::const_iterator          mQuadIterator;


        bitquad_t                   mQuadOffset;
        unsigned                    mQuadPosition;
        unsigned                    mBaseCodePoint;
        re::codepoint_t             mMinCodePoint;
        re::codepoint_t             mMaxCodePoint;
    };

    inline iterator begin() const {
        // note: pre-increment is intentional to move the iterator onto the first non-Empty interval.
        return ++iterator(mRuns.cbegin(), mQuads.cbegin());
    }

    inline iterator end() const {
        return iterator(mRuns.cend(), mQuads.cend());
    }

    class quad_iterator : public boost::iterator_facade<quad_iterator, std::pair<RunStructure, bitquad_t>, boost::random_access_traversal_tag> {
        friend class UnicodeSet;
        friend class boost::iterator_core_access;
    public:
        quad_iterator(RunVector::const_iterator runIterator, QuadVector::const_iterator quadIterator)
            : mRunIterator(runIterator), mQuadIterator(quadIterator), mOffset(0) {}

        void advance(unsigned n);

        inline const std::pair<RunStructure, bitquad_t> dereference() const {
            return std::make_pair(getRun(), getQuad());
        }

        inline void increment() {
            advance(1);
        }

        inline RunStructure getRun() const {
            const auto & t = *mRunIterator;
            return RunStructure(t.mType, t.mRunLength - mOffset);
        }

        inline bitquad_t getQuad() const {
            return *mQuadIterator;
        }

        inline bool equal(const quad_iterator & other) const {
            return (mRunIterator == other.mRunIterator) && (mQuadIterator == other.mQuadIterator);
        }

    private:
        RunVector::const_iterator   mRunIterator;
        QuadVector::const_iterator  mQuadIterator;
        unsigned                    mOffset;
    };

    inline quad_iterator quad_begin() const {
        return quad_iterator(mRuns.cbegin(), mQuads.cbegin());
    }

    inline quad_iterator quad_end() const {
        return quad_iterator(mRuns.cend(), mQuads.cend());
    }

    bool contains(const codepoint_t codepoint) const;

    void dump(llvm::raw_ostream & out) const;

    UnicodeSet complement() const;
    UnicodeSet operator & (const UnicodeSet & other) const;
    UnicodeSet operator + (const UnicodeSet & other) const;
    UnicodeSet operator - (const UnicodeSet & other) const;
    UnicodeSet operator ^ (const UnicodeSet & other) const;

    UnicodeSet();
    UnicodeSet(const codepoint_t codepoint);
    UnicodeSet(const codepoint_t lo_codepoint, const codepoint_t hi_codepoint);
    UnicodeSet(std::initializer_list<RunStructure> r, std::initializer_list<bitquad_t> q) : mRuns(r), mQuads(q) {}
    UnicodeSet(std::vector<RunStructure> && r, std::vector<bitquad_t> && q) : mRuns(r), mQuads(q) {}

private:

    std::vector<RunStructure>   mRuns;
    std::vector<bitquad_t>      mQuads;
};

inline UnicodeSet uset_complement(const UnicodeSet & s) {
    return s.complement();
}

inline UnicodeSet uset_intersection(const UnicodeSet & s1, const UnicodeSet & s2) {
    return s1 & s2;
}

inline UnicodeSet uset_union(const UnicodeSet & s1, const UnicodeSet & s2) {
    return s1 + s2;
}

inline UnicodeSet uset_difference(const UnicodeSet & s1, const UnicodeSet & s2) {
    return s1 - s2;
}

inline UnicodeSet uset_symmetric_difference(const UnicodeSet & s1, const UnicodeSet & s2) {
    return s1 ^ s2;
}

#endif

