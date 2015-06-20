#ifndef UNICODE_SET_H
#define UNICODE_SET_H
#include <stdint.h>
#include <vector>
#include <ostream>

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

const size_t quad_bits = 8 * sizeof(bitquad_t);
const size_t mod_quad_bit_mask = quad_bits - 1;
const size_t UnicodeQuadCount = 0x110000 / quad_bits;
const bitquad_t FullQuadMask = -1;

// The representation for runs
enum run_type_t : uint16_t {Empty, Mixed, Full};

struct RunStructure {
  RunStructure(run_type_t r, uint16_t lgth) : run_type(r), run_length(lgth) {}
  run_type_t run_type;
  uint16_t run_length;
};

class UnicodeSet {
friend class Uset_Iterator;
public:
//
//  The internal fields for a UnicodeSet.
    std::vector<RunStructure> runs;
    std::vector<bitquad_t> quads;
    int quad_count;
    
//  
//  Internal helper functions
    void append_run(run_type_t run_type, int run_length);
    void append_quad(bitquad_t q);
//
//  Nullary constructor for incremental building.
    UnicodeSet() : runs(std::vector<RunStructure>()), quads(std::vector<bitquad_t>()), quad_count(0) {}
//
//  Ternary constructor for constant construction using precomputed data.
    UnicodeSet(std::initializer_list<RunStructure> r, std::initializer_list<bitquad_t> q, int c) : runs(r), quads(q), quad_count(c) {}
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

class Uset_Iterator {
public:
    Uset_Iterator(const UnicodeSet & s) : uSet(s), run_no(0), offset(0), quad_no(0) {}
    bool at_end();
    RunStructure current_run();
    bitquad_t get_quad();
    void advance(int n);
private:
    const UnicodeSet & uSet;
    int run_no;
    int offset;
    int quad_no;
};

#endif

