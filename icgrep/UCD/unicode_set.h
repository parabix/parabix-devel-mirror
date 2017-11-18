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

typedef unsigned codepoint_t;
enum : codepoint_t { UNICODE_MAX = 0x10FFFF };

enum run_type_t : uint16_t {Empty, Mixed, Full};

class UnicodeSet {
public:

    using bitquad_t = uint32_t;
    using length_t = uint16_t;
    using run_t = std::pair<run_type_t, length_t>;
    using quad_iterator_return_t = std::pair<run_t, bitquad_t>;

    using interval_t = std::pair<codepoint_t, codepoint_t>;

    using RunVector = std::vector<run_t, ProxyAllocator<run_t>>;
    using QuadVector = std::vector<bitquad_t, ProxyAllocator<bitquad_t>>;
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
        return iterator(mRuns.cend(), mQuads.cend(), UNICODE_MAX+1);
    }

    bool empty() const; // The set has no members
    
    bool full() const;  // The set has the full set of possible Unicode codepoints.
    
    codepoint_t at(const size_type k) const; // return the k-th codepoint (or throw an error if it doesn't exist)

    bool contains(const codepoint_t codepoint) const;

    bool intersects(const codepoint_t lo, const codepoint_t hi) const;
    
    bool intersects(const UnicodeSet & other) const;
    
    void insert(const codepoint_t cp);

    void insert_range(const codepoint_t lo, const codepoint_t hi);

    size_type size() const; // number of intervals in this set

    size_type count() const; // number of codepoints in this set

    interval_t front() const;

    interval_t back() const;

    void print(llvm::raw_ostream & out) const;

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

    UnicodeSet(run_type_t emptyOrFull = Empty);
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
        quad_iterator(RunIterator runIterator, RunIterator runEnd, QuadIterator quadIterator, QuadIterator quadEnd, const run_type_t type, const length_t remaining)
        : mRunIterator(runIterator)
        , mRunEnd(runEnd)
        , mQuadIterator(quadIterator)
        #ifndef NDEBUG
        , mQuadEnd(quadEnd)
        #endif
        , mType(type)
        , mRemaining(remaining) {}

        void advance(unsigned n);

        inline quad_iterator_return_t dereference() const {
            return std::make_pair(std::make_pair(type(), length()), quad());
        }

        inline void increment() {
            advance(1);
        }

        inline run_type_t type() const {
            return mType;
        }

        inline length_t length() const {
            return mRemaining;
        }

        inline bitquad_t quad() const {
            assert (mQuadIterator != mQuadEnd);
            return *mQuadIterator;
        }

        inline bool equal(const quad_iterator & other) const {
            const auto r = (mRunIterator == other.mRunIterator) && (mRemaining == other.mRemaining);
            assert (!r || (mQuadIterator == other.mQuadIterator));
            return r;
        }

    private:
        RunIterator         mRunIterator;    
        const RunIterator   mRunEnd;
        QuadIterator        mQuadIterator;        
        #ifndef NDEBUG
        const QuadIterator  mQuadEnd;
        #endif
        run_type_t          mType;
        length_t            mRemaining;
    };

    inline quad_iterator quad_begin() const {
        return quad_iterator(mRuns.cbegin(), mRuns.cend(), mQuads.cbegin(), mQuads.cend(), std::get<0>(*mRuns.cbegin()), std::get<1>(*mRuns.cbegin()));
    }

    inline quad_iterator quad_end() const {
        return quad_iterator(mRuns.cend(), mRuns.cend(), mQuads.cend(), mQuads.cend(), Empty, 0);
    }

private:

    RunVector               mRuns;
    QuadVector              mQuads;
    static SlabAllocator<>  mAllocator;
};


inline void UnicodeSet::swap(UnicodeSet & other) {
    mRuns.swap(other.mRuns); mQuads.swap(other.mQuads);
}

inline void UnicodeSet::swap(UnicodeSet && other) {
    mRuns.swap(other.mRuns); mQuads.swap(other.mQuads);
}

}

#endif

