#ifndef UNICODE_SET_H
#define UNICODE_SET_H
#include <stdint.h>
#include <vector>
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

    class iterator : public boost::iterator_facade<iterator, const UnicodeSet &, boost::forward_traversal_tag, std::pair<RunStructure, bitquad_t>> {
        friend class UnicodeSet;
    public:
        iterator(const UnicodeSet & set, unsigned runIndex) : mUnicodeSet(set), mRunIndex(runIndex), mOffset(0), mQuadIndex(0) {}
    protected:
        friend class boost::iterator_core_access;
        void advance(unsigned n);
        const std::pair<RunStructure, bitquad_t> dereference() const;
        inline void increment() {
            advance(1);
        }
        inline bool equal(iterator const& other) const {
            return (mRunIndex == other.mRunIndex) && (&(mUnicodeSet) == &(other.mUnicodeSet)) && (mQuadIndex == other.mQuadIndex) && (mOffset == other.mOffset);
        }
    private:
        const UnicodeSet &          mUnicodeSet;
        unsigned                    mRunIndex;
        unsigned                    mOffset;
        unsigned                    mQuadIndex;
    };

    inline iterator begin() const {
        return iterator(*this, 0);
    }

    inline iterator end() const {
        return iterator(*this, runs.size());
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

