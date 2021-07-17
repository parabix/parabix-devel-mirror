#ifndef UNICODE_SET_H
#define UNICODE_SET_H

#include <stdint.h>
#include <vector>
#include <boost/iterator/iterator_facade.hpp>
#include <unicode/core/UCD_Config.h>
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

namespace llvm { class raw_ostream; }
namespace re { class RE; }

namespace UCD {

enum run_type_t : uint16_t {Empty, Mixed, Full};

class UnicodeSet {
    friend class re::RE;
    template<typename RunVector, typename QuadVector> friend void assign(UnicodeSet *, const RunVector &, const QuadVector &) noexcept;
public:

    using bitquad_t = uint32_t;
    using length_t = uint16_t;
    using size_type = size_t;

    using run_t = std::pair<run_type_t, length_t>;
    using quad_iterator_return_t = std::pair<run_t, bitquad_t>;

    class iterator : public boost::iterator_facade<iterator, interval_t, boost::forward_traversal_tag, interval_t> {
        friend class UnicodeSet;
        friend class boost::iterator_core_access;
    protected:

        iterator(const run_t * const runIterator, const bitquad_t * const quadIterator, const codepoint_t baseCodePoint)
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
        const run_t *       mRunIterator;
        const bitquad_t *   mQuadIterator;
        unsigned            mMixedRunIndex;
        bitquad_t           mQuadOffset;
        codepoint_t         mBaseCodePoint;
        codepoint_t         mMinCodePoint;
        codepoint_t         mMaxCodePoint;
    };

    inline iterator begin() const {
        // note: preincrement forces the iterator to advance onto and capture the first interval.
        return ++iterator(mRuns, mQuads, 0);
    }

    inline iterator end() const {
        return iterator(mRuns, mQuads, UNICODE_MAX+1);
    }

    bool empty() const { // The set has no members
        return (mRunLength == 1) && mRuns->first == Empty;
    }

    bool full() const {  // The set has the full set of possible Unicode codepoints.
        return (mRunLength == 1) && mRuns->first == Full;
    }
    
    codepoint_t at(const size_type k) const; // return the k-th codepoint (or throw an error if it doesn't exist)

    bool contains(const codepoint_t codepoint) const noexcept;

    bool intersects(const codepoint_t lo, const codepoint_t hi) const noexcept;
    
    bool intersects(const UnicodeSet & other) const noexcept;

    bool subset(const UnicodeSet & other) const noexcept;
    
    void insert(const codepoint_t cp);

    void insert(const UnicodeSet & other) noexcept;

    void invert() noexcept;

    void insert_range(const codepoint_t lo, const codepoint_t hi);

    size_type size() const noexcept; // number of intervals in this set

    size_type count() const noexcept; // number of codepoints in this set

    interval_t front() const noexcept;

    interval_t back() const noexcept;

    void print(llvm::raw_ostream & out) const noexcept;

    void dump(llvm::raw_ostream & out) const noexcept;

    UnicodeSet operator~() const noexcept;
    UnicodeSet operator&(const UnicodeSet & other) const noexcept;
    UnicodeSet operator+(const UnicodeSet & other) const noexcept;
    UnicodeSet operator-(const UnicodeSet & other) const noexcept;
    UnicodeSet operator^(const UnicodeSet & other) const noexcept;
    
    // The subset of a UnicodeSet consisting of the isolated codepoints only, i.e.,
    // those codepoints cp such that neither cp-1 nor cp+1 is a member of the set.
    UnicodeSet isolates () const noexcept;

    UnicodeSet & operator=(const UnicodeSet & other) noexcept;
    UnicodeSet & operator=(const UnicodeSet && other) noexcept;
    bool operator==(const UnicodeSet & other) const noexcept;
    bool operator<(const UnicodeSet & other) const noexcept;

    UnicodeSet() noexcept;
    UnicodeSet(const codepoint_t codepoint) noexcept;
    UnicodeSet(const codepoint_t lo, const codepoint_t hi) noexcept;
    UnicodeSet(const UnicodeSet & other) noexcept;
    UnicodeSet(const UnicodeSet && other) noexcept;

    UnicodeSet(const std::vector<interval_t>::iterator begin, const std::vector<interval_t>::iterator end) noexcept;
    UnicodeSet(std::initializer_list<interval_t>::iterator begin, std::initializer_list<interval_t>::iterator end) noexcept;
    UnicodeSet(run_t * const runs, const uint32_t runLength, const uint32_t runCapacity, bitquad_t * const quads, const uint32_t quadLength, const uint32_t quadCapacity) noexcept;

    UnicodeSet(std::initializer_list<run_t> r, std::initializer_list<bitquad_t> q) noexcept;

    inline static void Reset() {
        GlobalAllocator.Reset();
    }

protected:

    class quad_iterator : public boost::iterator_facade<quad_iterator, quad_iterator_return_t, boost::random_access_traversal_tag, quad_iterator_return_t> {
        friend class UnicodeSet;
        friend class boost::iterator_core_access;
    public:
        explicit quad_iterator(const run_t * const runIterator, const run_t * const runEnd, const bitquad_t * const quadIterator, const bitquad_t * const quadEnd, const run_type_t type, const length_t remaining)
        : mRunIterator(runIterator)
        , mRunEnd(runEnd)
        , mQuadIterator(quadIterator)
        #ifndef NDEBUG
        , mQuadEnd(quadEnd)
        #endif
        , mType(type)
        , mRemaining(remaining) {
            assert (type == Empty || type == Mixed || type == Full);
            assert (remaining > 0 || type == Empty);
            assert (remaining <= ((UNICODE_MAX+1) / (sizeof(bitquad_t) * 8)));
        }

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
        const run_t *           mRunIterator;
        const run_t * const     mRunEnd;
        const bitquad_t *       mQuadIterator;
        #ifndef NDEBUG
        const bitquad_t * const mQuadEnd;
        #endif
        run_type_t              mType;
        length_t                mRemaining;
    };

    inline quad_iterator quad_begin() const {
        return quad_iterator(mRuns, mRuns + mRunLength, mQuads, mQuads + mQuadLength, std::get<0>(*mRuns), std::get<1>(*mRuns));
    }

    inline quad_iterator quad_end() const {       
        return quad_iterator(mRuns + mRunLength, mRuns + mRunLength, mQuads + mQuadLength, mQuads + mQuadLength, Empty, 0);
    }

private:

    run_t *                 mRuns;
    bitquad_t *             mQuads;

    uint32_t                mRunLength;
    uint32_t                mQuadLength;

    uint32_t                mRunCapacity;
    uint32_t                mQuadCapacity;

    static SlabAllocator<>  GlobalAllocator;
};

}

#endif

