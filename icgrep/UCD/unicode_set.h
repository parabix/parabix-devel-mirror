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

typedef uint32_t bitquad_t;

// The representation for runs
enum run_type_t : uint16_t {Empty, Mixed, Full};

struct RunStructure {
  RunStructure(run_type_t r, uint16_t lgth) : mType(r), mRunLength(lgth) {}
  run_type_t mType;
  uint16_t mRunLength;
};

class UnicodeSet;

class UnicodeSet {
public:

    class quad_iterator : public boost::iterator_facade<quad_iterator, const std::pair<RunStructure, bitquad_t>, boost::forward_traversal_tag> {
        friend class UnicodeSet;
        friend class boost::iterator_core_access;
    protected:
        quad_iterator(const UnicodeSet & set, unsigned runIndex) : mUnicodeSet(set), mRunIndex(runIndex), mOffset(0), mQuadIndex(0) {}

        void advance(unsigned n);

        const std::pair<RunStructure, bitquad_t> dereference() const;

        inline void increment() {
            advance(1);
        }

        inline bool equal(quad_iterator const& other) const {
            assert (&(mUnicodeSet) == &(other.mUnicodeSet));
            return (mRunIndex == other.mRunIndex) && (mQuadIndex == other.mQuadIndex) && (mOffset == other.mOffset);
        }
    private:
        const UnicodeSet &          mUnicodeSet;
        unsigned                    mRunIndex;
        unsigned                    mOffset;
        unsigned                    mQuadIndex;
    };

    class iterator : public boost::iterator_facade<iterator, re::interval_t, boost::forward_traversal_tag, re::interval_t> {
        friend class UnicodeSet;
        friend class boost::iterator_core_access;
    protected:
        iterator(const UnicodeSet & set, unsigned runIndex, unsigned quadIndex)
        : mUnicodeSet(set), mRunIndex(runIndex), mQuadIndex(quadIndex), mQuadOffset(0)
        , mQuadRunIndex(0), mBaseCodePoint(0), mLeft(0), mRight(0)
        {

        }

        void advance(unsigned n);

        re::interval_t dereference() const {
            return std::make_pair(mLeft, mRight);
        }

        inline void increment() {
            advance(1);
        }

        inline bool equal(iterator const & other) const {
            assert (&(mUnicodeSet) == &(other.mUnicodeSet));
            return (mRunIndex == other.mRunIndex) && (mQuadIndex == other.mQuadIndex) &&
                   (mQuadOffset == other.mQuadOffset) && (mQuadRunIndex == other.mQuadRunIndex);
        }
    private:
        const UnicodeSet &      mUnicodeSet;
        unsigned                mRunIndex;
        unsigned                mQuadIndex;
        bitquad_t               mQuadOffset;
        unsigned                mQuadRunIndex;
        unsigned                mBaseCodePoint;
        re::codepoint_t         mLeft;
        re::codepoint_t         mRight;
    };

    inline quad_iterator quad_begin() const {
        return quad_iterator(*this, 0);
    }

    inline quad_iterator quad_end() const {
        return quad_iterator(*this, runs.size());
    }

    inline iterator begin() const {
        return iterator(*this, 0,0);
    }

    inline iterator end() const {
        return iterator(*this, runs.size(), quads.size());
    }

//
//  The internal fields for a UnicodeSet.
    std::vector<RunStructure>   runs;
    std::vector<bitquad_t>      quads;
    unsigned quad_count;
    
//  
//  Internal helper functions
    void append_run(run_type_t run_type, int run_length);
    void append_quad(bitquad_t q);
//
//  Nullary constructor for incremental building.
    UnicodeSet() : quad_count(0) {}
//
//  Ternary constructor for constant construction using precomputed data.
    UnicodeSet(std::initializer_list<RunStructure> r, std::initializer_list<bitquad_t> q, unsigned c) : runs(r), quads(q), quad_count(c) {}
};

void Dump_uset(UnicodeSet s);
UnicodeSet empty_uset();
UnicodeSet singleton_uset(int codepoint);
UnicodeSet range_uset(int lo_codepoint, int hi_codepoint);
UnicodeSet uset_complement (const UnicodeSet &s);
UnicodeSet uset_union(const UnicodeSet & s1, const UnicodeSet & s2);
UnicodeSet uset_intersection(const UnicodeSet &s1, const UnicodeSet &s2);
UnicodeSet uset_difference(const UnicodeSet &s1, const UnicodeSet &s2);
UnicodeSet uset_symmetric_difference(const UnicodeSet & s1, const UnicodeSet & s2);
bool uset_member(const UnicodeSet & s, int codepoint);

#endif

