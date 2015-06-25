//
// unicode_set.cpp - representing and manipulating sets of Unicode
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

#include "unicode_set.h"
#include "assert.h"
#include <string>
#include <iostream>
#include <include/simd-lib/builtins.hpp>

const size_t QUAD_BITS = (8 * sizeof(bitquad_t));
const size_t MOD_QUAD_BIT_MASK = QUAD_BITS - 1;
const size_t UNICODE_QUAD_COUNT = 0x110000 / QUAD_BITS;
const bitquad_t FULL_QUAD_MASK = -1;

inline const RunStructure & get_run(UnicodeSet::quad_iterator i) {
    return std::get<0>(*i);
}

inline bitquad_t get_quad(UnicodeSet::quad_iterator i) {
    return std::get<1>(*i);
}

const std::pair<RunStructure, bitquad_t> UnicodeSet::quad_iterator::dereference() const {
    const RunStructure & t = mUnicodeSet.runs[mRunIndex];
    RunStructure s(t.mType, t.mRunLength - mOffset);
    const bitquad_t q = ((t.mType == Empty) ? 0 : (t.mType == Full) ? FULL_QUAD_MASK : mUnicodeSet.quads[mQuadIndex]);
    return std::make_pair(s, q);
}

void UnicodeSet::quad_iterator::advance(unsigned n) {
    while (n > 0) {
        const RunStructure & t = mUnicodeSet.runs[mRunIndex];
        int remain = t.mRunLength - mOffset;
        if (remain > n) {
            mOffset += n;
            if (t.mType == Mixed) {
                mQuadIndex += n;
            }
            break;
        }
        else if (remain == n) {
            ++mRunIndex;
            mOffset = 0;
            if (t.mType == Mixed) {
                mQuadIndex += n;
            }
            break;
        }
        else {
            ++mRunIndex;
            mOffset = 0;
            if (t.mType == Mixed) {
                mQuadIndex += remain;
            }
            n -= remain;
        }
    }
}

void UnicodeSet::append_run(run_type_t run_type, int run_length) {
    if (run_length == 0) {
        return;
    }
    else if (runs.size() == 0) {
        runs.emplace_back(run_type, run_length);
    }
    else {
        RunStructure last_run = runs[runs.size()-1];
        if (last_run.mType == run_type) {
            runs.back().mRunLength += run_length;
        }
        else {
            runs.emplace_back(run_type, run_length);
        }
    }
    quad_count += run_length;
}

void UnicodeSet::append_quad(bitquad_t q) {
    if (q == 0) {
        append_run(Empty, 1);
    }
    else if (q == FULL_QUAD_MASK) {
        append_run(Full, 1);
    }
    else {
        quads.push_back(q);
        append_run(Mixed, 1);
    }
}

void Dump_uset(const UnicodeSet & s) {
    for (auto it = s.quad_begin(); it != s.quad_end(); ++it) {
        RunStructure this_run = get_run(it);
        if (this_run.mType == Empty) {
            std::cout << "Empty(" << this_run.mRunLength << ")\n";
            it += this_run.mRunLength;
        }
        else if (this_run.mType == Full) {
            std::cout << "Full(" << this_run.mRunLength << ")\n";
            it += this_run.mRunLength;
        }
        else {
            for (int i = 0; i != this_run.mRunLength; i++) {
                std::cout << "Mixed(" << std::hex << get_quad(it) << std::dec << ")\n";
                ++it;
            }
        }
    }
}

UnicodeSet empty_uset() {
    UnicodeSet iset;
    iset.runs.emplace_back(Empty, UNICODE_QUAD_COUNT);
    iset.quad_count = UNICODE_QUAD_COUNT;
    return iset;
}

// singleton set constructor
UnicodeSet singleton_uset(int codepoint) {
    UnicodeSet iset;
    int quad_no = codepoint / QUAD_BITS;
    bitquad_t quad_val = 1 << (codepoint & MOD_QUAD_BIT_MASK);
    if (quad_no > 0) iset.append_run(Empty, quad_no);
    iset.append_run(Mixed, 1);
    iset.quads.push_back(quad_val);
    if (quad_no < UNICODE_QUAD_COUNT - 1) iset.append_run(Empty, UNICODE_QUAD_COUNT - (quad_no + 1));
    iset.quad_count =  UNICODE_QUAD_COUNT;
    return iset;
}

// range set constructor
UnicodeSet range_uset(int lo_codepoint, int hi_codepoint) {
    UnicodeSet iset;
    int lo_quad_no = lo_codepoint / QUAD_BITS;
    int hi_quad_no = hi_codepoint / QUAD_BITS;
    int lo_offset = lo_codepoint & MOD_QUAD_BIT_MASK;
    int hi_offset = hi_codepoint & MOD_QUAD_BIT_MASK;
    if (lo_quad_no > 0) iset.append_run(Empty, lo_quad_no);
    if (lo_quad_no == hi_quad_no) {
        bitquad_t quad = (FULL_QUAD_MASK << lo_offset) & (FULL_QUAD_MASK >> (QUAD_BITS - 1 - hi_offset));
        iset.append_quad(quad);
    }
    else {
        iset.append_quad((FULL_QUAD_MASK << lo_offset) & FULL_QUAD_MASK);
        iset.append_run(Full, hi_quad_no - (lo_quad_no + 1));
        iset.append_quad((FULL_QUAD_MASK >> (QUAD_BITS - 1 - hi_offset)) & FULL_QUAD_MASK);
    }
    if (hi_quad_no < UNICODE_QUAD_COUNT - 1) iset.append_run(Empty, UNICODE_QUAD_COUNT - (hi_quad_no + 1));
    return iset;
}

UnicodeSet uset_complement (const UnicodeSet & s) {
    assert(s.quad_count == UNICODE_QUAD_COUNT);
    UnicodeSet iset;
    for (auto itr = s.quad_begin(); itr != s.quad_end(); ) {
        auto run = get_run(itr);
        if (run.mType == Empty) {
            iset.append_run(Full, run.mRunLength);
            itr += run.mRunLength;
        }
        else if (run.mType == Full) {
            iset.append_run(Empty, run.mRunLength);
            itr += run.mRunLength;
        }
        else {
            for (unsigned i = 0; i != run.mRunLength; i++) {
                iset.append_quad(FULL_QUAD_MASK ^ get_quad(itr++));
            }
        }
    }
    return iset;
}

UnicodeSet uset_intersection (const UnicodeSet & s1, const UnicodeSet & s2) {
    assert(s1.quad_count == UNICODE_QUAD_COUNT);
    assert(s2.quad_count == UNICODE_QUAD_COUNT);
    UnicodeSet iset;
    for (auto i1 = s1.quad_begin(), i2 = s2.quad_begin(); i1 != s1.quad_end(); ) {
        auto run1 = get_run(i1);
        auto run2 = get_run(i2);
        unsigned n = std::min(run1.mRunLength, run2.mRunLength);
        if ((run1.mType == Empty) || (run2.mType == Empty)) {
            iset.append_run(Empty, n);
            i1 += n;
            i2 += n;
        }
        else if ((run1.mType == Full) && (run2.mType == Full)) {
            iset.append_run(Full, n);
            i1 += n;
            i2 += n;
        }
        else if (run1.mType == Full) {
            for (unsigned i = 0; i != n; ++i, ++i2) {
                iset.append_quad(get_quad(i2));
            }
            i1 += n;
        }
        else if (run2.mType == Full) {
            for (unsigned i = 0; i != n; ++i, ++i1) {
                iset.append_quad(get_quad(i1));
            }
            i2 += n;
        }
        else {
            for (unsigned i = 0; i < n; ++i, ++i1, ++i2) {
                iset.append_quad(get_quad(i1) & get_quad(i2));
            }
        }
    }
    return iset;
}

UnicodeSet uset_union (const UnicodeSet & s1, const UnicodeSet & s2) {
    assert(s1.quad_count == UNICODE_QUAD_COUNT);
    assert(s2.quad_count == UNICODE_QUAD_COUNT);
    UnicodeSet iset;
    for (auto i1 = s1.quad_begin(), i2 = s2.quad_begin(); i1 != s1.quad_end(); ) {
        auto run1 = get_run(i1);
        auto run2 = get_run(i2);
        unsigned n = std::min(run1.mRunLength, run2.mRunLength);
        if ((run1.mType == Empty) && (run2.mType == Empty)) {
            iset.append_run(Empty, n);
            i1 += n;
            i2 += n;
        }
        else if ((run1.mType == Full) || (run2.mType == Full)) {
            iset.append_run(Full, n);
            i1 += n;
            i2 += n;
        }
        else if (run1.mType == Empty) {
            for (unsigned i = 0; i < n; ++i, ++i2) {
                iset.append_quad(get_quad(i2));
            }
            i1 += n;
        }
        else if (run2.mType == Empty) {
            for (unsigned i = 0; i < n; ++i, ++i1) {
                iset.append_quad(get_quad(i1));
            }
            i2 += n;
        }
        else {
            for (unsigned i = 0; i != n; ++i, ++i1, ++i2) {
                iset.append_quad(get_quad(i1) | get_quad(i2));
            }
        }
    }
    return iset;
}

UnicodeSet uset_difference (const UnicodeSet & s1, const UnicodeSet & s2) {
    assert(s1.quad_count == UNICODE_QUAD_COUNT);
    assert(s2.quad_count == UNICODE_QUAD_COUNT);
    UnicodeSet iset;
    for (auto i1 = s1.quad_begin(), i2 = s2.quad_begin(); i1 != s1.quad_end(); ) {
        auto run1 = get_run(i1);
        auto run2 = get_run(i2);
        unsigned n = std::min(run1.mRunLength, run2.mRunLength);
        if ((run1.mType == Empty) || (run2.mType == Full)) {
            iset.append_run(Empty, n);
            i1 += n;
            i2 += n;
        }
        else if ((run1.mType == Full) && (run2.mType == Empty)) {
            iset.append_run(Full, n);
            i1 += n;
            i2 += n;
        }
        else if (run1.mType == Full) {
            for (unsigned i = 0; i != n; ++i, ++i2) {
                iset.append_quad(FULL_QUAD_MASK ^ get_quad(i2));
            }
            i1 += n;
        }
        else if (run2.mType == Empty) {
            for (unsigned i = 0; i != n; ++i, ++i1) {
                iset.append_quad(get_quad(i1));
            }
            i2 += n;
        }
        else {
            for (unsigned i = 0; i != n; ++i, ++i1, ++i2) {
                iset.append_quad(get_quad(i1) & ~get_quad(i2));
            }
        }
    }
    return iset;
}

UnicodeSet uset_symmetric_difference (const UnicodeSet & s1, const UnicodeSet & s2) {
    assert(s1.quad_count == UNICODE_QUAD_COUNT);
    assert(s2.quad_count == UNICODE_QUAD_COUNT);
    UnicodeSet iset;
    for (auto i1 = s1.quad_begin(), i2 = s2.quad_begin(); i1 != s1.quad_end(); ) {
        auto run1 = get_run(i1);
        auto run2 = get_run(i2);
        unsigned n = std::min(run1.mRunLength, run2.mRunLength);
        if (((run1.mType == Empty) && (run2.mType == Full)) || ((run1.mType == Full) && (run2.mType == Empty))) {
            iset.append_run(Full, n);
            i1 += n;
            i2 += n;
        }
        else if (((run1.mType == Full) && (run2.mType == Full)) || ((run1.mType == Empty) && (run2.mType == Empty))) {
            iset.append_run(Empty, n);
            i1 += n;
            i2 += n;
        }
        else if (run1.mType == Empty) {
            for (int i = 0; i < n; ++i, ++i2) {
                iset.append_quad(get_quad(i2));
            }
            i1 += n;
        }
        else if (run2.mType == Empty) {
            for (int i = 0; i < n; ++i, ++i1) {
                iset.append_quad(get_quad(i1));
            }
            i2 += n;
        }
        else if (run1.mType == Full) {
            for (int i = 0; i < n; ++i, ++i2) {
                iset.append_quad(FULL_QUAD_MASK ^ get_quad(i2));
            }
            i1 += n;
        }
        else if (run2.mType == Empty) {
            for (unsigned i = 0; i < n; ++i, ++i1) {
                iset.append_quad(FULL_QUAD_MASK ^ get_quad(i1));
            }
            i2 += n;
        }
        else {
            for (unsigned i = 0; i != n; ++i, ++i1, ++i2) {
                iset.append_quad(get_quad(i1) ^ get_quad(i2));
            }
        }
    }
    return iset;
}

bool uset_member(const UnicodeSet & s, int codepoint){
    int quad_no = codepoint / QUAD_BITS;
    bitquad_t quad_val = 1 << (codepoint & MOD_QUAD_BIT_MASK);
    return (get_quad(s.quad_begin() + quad_no) & quad_val) != 0;
}

void UnicodeSet::iterator::advance(unsigned n) {

    while (n) {

        const RunStructure & t = mUnicodeSet.runs[mRunIndex];

        if (t.mType == Full) {
            mRight = mBaseCodePoint + t.mRunLength * QUAD_BITS;
            --n;
        }

        if (t.mType != Mixed) {
            ++mRunIndex;
            mBaseCodePoint += t.mRunLength * QUAD_BITS;
            mQuadOffset = 0;
            mQuadRunIndex = 0;
            continue;
        }

        while (mQuadRunIndex < t.mRunLength) {

            const bitquad_t q = mUnicodeSet.quads[mQuadIndex];
            const bitquad_t m = q &(MOD_QUAD_BIT_MASK >> mQuadOffset);

            // Nothing left in this quad to add; skip to the next one.
            if (m == 0) {
                mBaseCodePoint += QUAD_BITS;
                mLeft = mBaseCodePoint;
                ++mQuadIndex;
                if (++mQuadRunIndex == t.mRunLength) {
                    ++mRunIndex;
                }
                continue;
            }

            mQuadOffset = scan_forward_zeroes(m);
            mLeft = mBaseCodePoint + mQuadOffset;
            break;
        }


        while (mQuadRunIndex < t.mRunLength) {

            // Although the initial position was in this quad, the final position isn't
            // unless this is the last quad of this mixed run and the subsequent quad is
            // Empty.

            const bitquad_t q = mUnicodeSet.quads[mQuadIndex];
            const bitquad_t m = ~q & (MOD_QUAD_BIT_MASK >> mQuadOffset);
            // Nothing left in this quad to add; skip to the next one.
            if (m == 0) {
                mBaseCodePoint += QUAD_BITS;
                mRight = mBaseCodePoint;
                ++mQuadIndex;
                if (++mQuadRunIndex == t.mRunLength) {
                    ++mRunIndex;
                }
                continue;
            }

            mQuadOffset = scan_forward_zeroes(m);
            mRight = mBaseCodePoint + mQuadOffset;
            --n;
            break;
        }
    }
}


