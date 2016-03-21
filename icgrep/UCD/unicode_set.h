#ifndef UNICODE_SET_H
#define UNICODE_SET_H
#include <stdint.h>
#include <vector>
#include <boost/iterator/iterator_facade.hpp>
#include <util/slab_allocator.h>

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

namespace UCD {

enum run_type_t : uint16_t {Empty, Mixed, Full};

class UnicodeSet {
public:

    using bitquad_t = uint32_t;
    using length_t = uint16_t;
    using run_t = std::pair<run_type_t, length_t>;
    using quad_iterator_return_t = std::pair<run_t, bitquad_t>;

    using codepoint_t = unsigned;
    using interval_t = std::pair<codepoint_t, codepoint_t>;

    using Allocator = SlabAllocator<uint32_t>;
    using RunAllocator = Allocator::rebind<run_t>::other;
    using QuadAllocator = Allocator::rebind<bitquad_t>::other;

    using RunVector = std::vector<run_t, RunAllocator>;
    using QuadVector = std::vector<bitquad_t, QuadAllocator>;
    using RunIterator = RunVector::const_iterator;
    using QuadIterator = QuadVector::const_iterator;

    using size_type = RunVector::size_type;

    class iterator : public boost::iterator_facade<iterator, interval_t, boost::forward_traversal_tag, interval_t> {
        friend class UnicodeSet;
        friend class boost::iterator_core_access;
    protected:

        iterator(const RunVector::const_iterator runIterator, const QuadVector::const_iterator quadIterator, const codepoint_t baseCodePoint)
        : mRunIterator(runIterator), mQuadIterator(quadIterator)
        , mMixedRunIndex(0), mQuadOffset(0), mBaseCodePoint(baseCodePoint), mMinCodePoint(baseCodePoint), mMaxCodePoint(baseCodePoint) {

        }

        void advance(const unsigned n);

        inline interval_t dereference() const {
            return std::make_pair(mMinCodePoint, mMaxCodePoint);
        }

        inline void increment() {
            advance(1);
        }

        inline bool equal(const iterator & other) const {
            return (mMinCodePoint == other.mMinCodePoint);
        }
    private:
        RunIterator         mRunIterator;
        QuadIterator        mQuadIterator;
        unsigned            mMixedRunIndex;
        bitquad_t           mQuadOffset;
        codepoint_t         mBaseCodePoint;
        codepoint_t         mMinCodePoint;
        codepoint_t         mMaxCodePoint;
    };

    inline iterator begin() const {
        // note: preincrement forces the iterator to advance onto and capture the first interval.
        return ++iterator(mRuns.cbegin(), mQuads.cbegin(), 0);
    }

    inline iterator end() const {
        return iterator(mRuns.cend(), mQuads.cend(), 0x110000);
    }

    bool contains(const codepoint_t codepoint) const;

    bool intersects(const codepoint_t lo, const codepoint_t hi) const;

    void insert(const codepoint_t cp);

    void insert_range(const codepoint_t lo, const codepoint_t hi);

    bool empty() const;

    size_type size() const;

    interval_t front() const;

    interval_t back() const;

    void dump(llvm::raw_ostream & out) const;

    UnicodeSet operator~() const;
    UnicodeSet operator&(const UnicodeSet & other) const;
    UnicodeSet operator+(const UnicodeSet & other) const;
    UnicodeSet operator-(const UnicodeSet & other) const;
    UnicodeSet operator^(const UnicodeSet & other) const;

    inline UnicodeSet & operator=(const UnicodeSet & other) = default;
    inline UnicodeSet & operator=(UnicodeSet && other) = default;
    bool operator==(const UnicodeSet & other) const;
    bool operator<(const UnicodeSet & other) const;

    UnicodeSet();
    UnicodeSet(const codepoint_t codepoint);
    UnicodeSet(const codepoint_t lo, const codepoint_t hi);
    UnicodeSet(const UnicodeSet & other);
    UnicodeSet(std::initializer_list<run_t> r, std::initializer_list<bitquad_t> q);
    UnicodeSet(std::initializer_list<interval_t>::iterator begin, std::initializer_list<interval_t>::iterator end);
    UnicodeSet(const std::vector<interval_t>::iterator begin, const std::vector<interval_t>::iterator end);

    inline void swap(UnicodeSet & other);
    inline void swap(UnicodeSet && other);

protected:

    UnicodeSet(std::vector<run_t> && r, std::vector<bitquad_t> && q);

    class quad_iterator : public boost::iterator_facade<quad_iterator, quad_iterator_return_t, boost::random_access_traversal_tag, quad_iterator_return_t> {
        friend class UnicodeSet;
        friend class boost::iterator_core_access;
    public:
        quad_iterator(RunIterator runIterator, QuadIterator quadIterator)
            : mRunIterator(runIterator), mQuadIterator(quadIterator), mOffset(0) {}

        void advance(unsigned n);

        inline quad_iterator_return_t dereference() const {
            return std::make_pair(std::make_pair(type(), length()), quad());
        }

        inline void increment() {
            advance(1);
        }

        inline run_type_t type() const {
            return mRunIterator->first;
        }

        inline length_t length() const {
            return mRunIterator->second - mOffset;
        }

        inline bitquad_t quad() const {
            return *mQuadIterator;
        }

        inline bool equal(const quad_iterator & other) const {
            return (mRunIterator == other.mRunIterator) && (mQuadIterator == other.mQuadIterator);
        }

    private:
        RunIterator     mRunIterator;
        QuadIterator    mQuadIterator;
        unsigned        mOffset;
    };

    inline quad_iterator quad_begin() const {
        return quad_iterator(mRuns.cbegin(), mQuads.cbegin());
    }

    inline quad_iterator quad_end() const {
        return quad_iterator(mRuns.cend(), mQuads.cend());
    }

private:

    RunVector           mRuns;
    QuadVector          mQuads;
    static Allocator    mAllocator;
};

enum : UnicodeSet::codepoint_t { UNICODE_MAX = 0x10FFFF };

inline void UnicodeSet::swap(UnicodeSet & other) {
    mRuns.swap(other.mRuns); mQuads.swap(other.mQuads);
}

inline void UnicodeSet::swap(UnicodeSet && other) {
    mRuns.swap(other.mRuns); mQuads.swap(other.mQuads);
}

}

#endif

