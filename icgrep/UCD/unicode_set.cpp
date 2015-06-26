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
#include <llvm/Support/raw_ostream.h>
#include <include/simd-lib/builtins.hpp>
#include <iostream>

const size_t QUAD_BITS = (8 * sizeof(bitquad_t));
const size_t MOD_QUAD_BIT_MASK = QUAD_BITS - 1;
const size_t UNICODE_QUAD_COUNT = 0x110000 / QUAD_BITS;
const bitquad_t FULL_QUAD_MASK = -1;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief append_run
 ** ------------------------------------------------------------------------------------------------------------- */
inline void UnicodeSet::append_run(const run_type_t type, const unsigned length) {
    if (length == 0) {
        return;
    }
    else if (runs.size() == 0) {
        runs.emplace_back(type, length);
    }
    else {
        RunStructure last_run = runs[runs.size()-1];
        if (last_run.mType == type) {
            runs.back().mRunLength += length;
        }
        else {
            runs.emplace_back(type, length);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief append_quad
 ** ------------------------------------------------------------------------------------------------------------- */
inline void UnicodeSet::append_quad(const bitquad_t quad) {
    if (quad == 0) {
        append_run(Empty, 1);
    }
    else if (quad == FULL_QUAD_MASK) {
        append_run(Full, 1);
    }
    else {
        quads.push_back(quad);
        append_run(Mixed, 1);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief dump
 ** ------------------------------------------------------------------------------------------------------------- */
void UnicodeSet::dump(llvm::raw_ostream & out) const {
    auto quad_itr = quads.cbegin();
    for (const RunStructure & run : runs) {
        if (run.mType == Empty) {
            out << "Empty(" << run.mRunLength << ")\n";
        }
        else if (run.mType == Empty) {
            out << "Full(" << run.mRunLength << ")\n";
        }
        else {
            for (unsigned i = 0; i != run.mRunLength; ++i, ++quad_itr) {
                assert (quad_itr != quads.cend());
                out << "Mixed("; out.write_hex(*quad_itr) << ")\n";
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief complement
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::complement() const {
    UnicodeSet set;
    auto quad_itr = quads.cbegin();
    for (const RunStructure & run : runs) {
        if (run.mType == Empty) {
            set.append_run(Full, run.mRunLength);
        }
        else if (run.mType == Empty) {
            set.append_run(Empty, run.mRunLength);
        }
        else {
            for (unsigned i = 0; i != run.mRunLength; ++i, ++quad_itr) {
                assert (quad_itr != quads.cend());
                set.append_quad(FULL_QUAD_MASK ^ *quad_itr);
            }
        }
    }
    return set;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief intersection
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator&(const UnicodeSet & other) const {
    UnicodeSet iset;
    const auto e1 = quad_end();
    const auto e2 = other.quad_end();
    for (auto i1 = quad_begin(), i2 = other.quad_begin(); i1 != e1 && i2 != e2; ) {
        const auto run1 = i1.getRun();
        const auto run2 = i2.getRun();
        const auto n = std::min(run1.mRunLength, run2.mRunLength);
        if (run1.mType == run2.mType && run1.mType != Mixed) {
            iset.append_run(run1.mType, n);
            i1 += n;
            i2 += n;
        }
        else if (run1.mType == Full) {
            for (unsigned i = 0; i != n; ++i, ++i2) {
                iset.append_quad(i2.getQuad());
            }
            i1 += n;
        }
        else if (run2.mType == Full) {
            for (unsigned i = 0; i != n; ++i, ++i1) {
                iset.append_quad(i1.getQuad());
            }
            i2 += n;
        }
        else {
            for (unsigned i = 0; i < n; ++i, ++i1, ++i2) {
                iset.append_quad(i1.getQuad() & i2.getQuad());
            }
        }
    }
    return iset;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief union
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator+(const UnicodeSet & other) const {
    UnicodeSet iset;
    const auto e1 = quad_end();
    const auto e2 = other.quad_end();
    for (auto i1 = quad_begin(), i2 = other.quad_begin(); i1 != e1 && i2 != e2; ) {
        const auto run1 = i1.getRun();
        const auto run2 = i2.getRun();
        const auto n = std::min(run1.mRunLength, run2.mRunLength);
        if (run1.mType == run2.mType && run1.mType != Mixed) {
            iset.append_run(run1.mType, n);
            i1 += n;
            i2 += n;
        }
        else if (run1.mType == Empty) {
            for (unsigned i = 0; i != n; ++i, ++i2) {
                iset.append_quad(i2.getQuad());
            }
            i1 += n;
        }
        else if (run2.mType == Empty) {
            for (unsigned i = 0; i != n; ++i, ++i1) {
                iset.append_quad(i1.getQuad());
            }
            i2 += n;
        }
        else {
            for (unsigned i = 0; i < n; ++i, ++i1, ++i2) {
                iset.append_quad(i1.getQuad() | i2.getQuad());
            }
        }
    }
    return iset;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief difference
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator-(const UnicodeSet & other) const {
    UnicodeSet iset;
    const auto e1 = quad_end();
    const auto e2 = other.quad_end();
    for (auto i1 = quad_begin(), i2 = other.quad_begin(); i1 != e1 && i2 != e2; ) {
        const auto run1 = i1.getRun();
        const auto run2 = i2.getRun();
        unsigned n = std::min(run1.mRunLength, run2.mRunLength);
        if ((run1.mType == Empty) || (run2.mType == Full) || (run1.mType == Full && run2.mType == Empty)) {
            iset.append_run(run1.mType, n);
            i1 += n;
            i2 += n;
        }
        else if (run1.mType == Full) {
            for (unsigned i = 0; i != n; ++i, ++i2) {
                iset.append_quad(FULL_QUAD_MASK ^ i2.getQuad());
            }
            i1 += n;
        }
        else if (run2.mType == Empty) {
            for (unsigned i = 0; i != n; ++i, ++i1) {
                iset.append_quad(i1.getQuad());
            }
            i2 += n;
        }
        else {
            for (unsigned i = 0; i != n; ++i, ++i1, ++i2) {
                iset.append_quad(i1.getQuad() &~ i2.getQuad());
            }
        }
    }
    return iset;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief symmetric difference
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator^(const UnicodeSet & other) const {
    UnicodeSet iset;
    const auto e1 = quad_end();
    const auto e2 = other.quad_end();
    for (auto i1 = quad_begin(), i2 = other.quad_begin(); i1 != e1 && i2 != e2; ) {
        const auto run1 = i1.getRun();
        const auto run2 = i2.getRun();
        unsigned n = std::min(run1.mRunLength, run2.mRunLength);
        if (run1.mType != Mixed && run2.mType != Mixed) {
            iset.append_run(run1.mType == run2.mType ? Empty : Full, n);
            i1 += n;
            i2 += n;
        }
        else if (run1.mType == Empty) {
            for (int i = 0; i < n; ++i, ++i2) {
                iset.append_quad(i2.getQuad());
            }
            i1 += n;
        }
        else if (run2.mType == Empty) {
            for (int i = 0; i < n; ++i, ++i1) {
                iset.append_quad(i1.getQuad());
            }
            i2 += n;
        }
        else if (run1.mType == Full) {
            for (int i = 0; i < n; ++i, ++i2) {
                iset.append_quad(FULL_QUAD_MASK ^ i2.getQuad());
            }
            i1 += n;
        }
        else if (run2.mType == Empty) {
            for (unsigned i = 0; i < n; ++i, ++i1) {
                iset.append_quad(FULL_QUAD_MASK ^ i1.getQuad());
            }
            i2 += n;
        }
        else {
            for (unsigned i = 0; i != n; ++i, ++i1, ++i2) {
                iset.append_quad(i1.getQuad() ^ i2.getQuad());
            }
        }
    }
    return iset;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief contains
 * @param codepoint
 *
 * Return whether this UnicodeSet contains the specified code point
 ** ------------------------------------------------------------------------------------------------------------- */
bool UnicodeSet::contains(const codepoint_t codepoint) const {

    auto n = codepoint / QUAD_BITS;
    unsigned runIndex = 0;
    unsigned quadIndex = 0;

    for (;;) {
        const RunStructure & t = runs[runIndex];
        if (t.mRunLength >= n) {
            if (t.mType == Mixed) {
                return (quads[quadIndex + n - 1] & (static_cast<bitquad_t>(1) << (codepoint & MOD_QUAD_BIT_MASK))) != 0;
            }
            return (t.mType == Full);
        }
        if (t.mType == Mixed) {
            quadIndex += n;
        }
        ++runIndex;
        n -= t.mRunLength;
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief UnicodeSet::quad_iterator::advance
 ** ------------------------------------------------------------------------------------------------------------- */
void UnicodeSet::quad_iterator::advance(unsigned n) {
    while (n > 0) {
        const RunStructure & t = mUnicodeSet.runs[mRunIndex];
        const unsigned remain = t.mRunLength - mOffset;
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief UnicodeSet::iterator::advance
 ** ------------------------------------------------------------------------------------------------------------- */
void UnicodeSet::iterator::advance(unsigned n) {

    std::cerr << "advance(" << n << ")\n";

    mMinCodePoint = mBaseCodePoint;

    for  ( ;n; ++mRunIterator) {

        const RunStructure & t = *mRunIterator;

        std::cerr << "Type:";
        switch (t.mType) {
            case Empty: std::cerr << "Empty"; break;
            case Full: std::cerr << "Full"; break;
            case Mixed: std::cerr << "Mixed"; break;
        }
        std::cerr << " Length:" << t.mRunLength;
        std::cerr << " BaseCodePoint:" << mBaseCodePoint;


        std::cerr << std::endl;

        if (t.mType != Mixed) {
            mMaxCodePoint = mBaseCodePoint + t.mRunLength * QUAD_BITS;
            mBaseCodePoint = mMaxCodePoint;
            mQuadOffset = 0;
            mQuadPosition = 0;
            if (t.mType == Full) {
                --n;
            }
            continue;
        }

        while (mQuadPosition != t.mRunLength) {

            const bitquad_t q = *mQuadIterator;

            const bitquad_t m = q & ((-1) >> mQuadOffset);

            std::cerr << "  q:" << std::hex << q << std::endl;
            std::cerr << " +m:" << std::hex << m << std::dec << "   (" << mQuadOffset << ")" << std::endl;

            // Nothing left in this quad to add; skip to the next one.
            if (m == 0) {
                mBaseCodePoint += QUAD_BITS;
                mMinCodePoint = mBaseCodePoint;
                ++mQuadIterator;
                continue;
            }

            mQuadOffset = scan_forward_zeroes(m);
            mMinCodePoint = mBaseCodePoint + mQuadOffset;



            break;
        }

        while (mQuadPosition != t.mRunLength) {

            // Although the initial position was in this quad, the final position isn't
            // unless this is the last quad of this mixed run and the subsequent quad is
            // Empty.

            const bitquad_t q = *mQuadIterator;
            const bitquad_t m = ~q & ((-1) >> mQuadOffset);

            std::cerr << "  q:" << std::hex << q << std::endl;
            std::cerr << " -m:" << std::hex << m << std::dec << "   (" << mQuadOffset << ")" << std::endl;

            // Nothing left in this quad to add; skip to the next one.
            if (m == 0) {
                mBaseCodePoint += QUAD_BITS;
                mMaxCodePoint = mBaseCodePoint;
                ++mQuadIterator;
                continue;
            }

            mQuadOffset = scan_forward_zeroes(m);
            mMaxCodePoint = mBaseCodePoint + mQuadOffset;
            --n;
            break;
        }
    }
}

UnicodeSet::UnicodeSet()
: runs({{{Empty, UNICODE_QUAD_COUNT}}})
{

}

// singleton set constructor
UnicodeSet::UnicodeSet(const codepoint_t codepoint) {
    codepoint_t quad_no = codepoint / QUAD_BITS;
    if (quad_no > 0) {
        append_run(Empty, quad_no);
    }
    append_run(Mixed, 1);
    quads.push_back(static_cast<bitquad_t>(1) << (codepoint & MOD_QUAD_BIT_MASK));
    if (quad_no < UNICODE_QUAD_COUNT - 1) {
        append_run(Empty, UNICODE_QUAD_COUNT - (quad_no + 1));
    }
}

// range set constructor
UnicodeSet::UnicodeSet(const codepoint_t lo_codepoint, const codepoint_t hi_codepoint) {
    codepoint_t lo_quad_no = lo_codepoint / QUAD_BITS;
    codepoint_t hi_quad_no = hi_codepoint / QUAD_BITS;
    codepoint_t lo_offset = lo_codepoint & MOD_QUAD_BIT_MASK;
    codepoint_t hi_offset = hi_codepoint & MOD_QUAD_BIT_MASK;
    if (lo_quad_no > 0) {
        append_run(Empty, lo_quad_no);
    }
    if (lo_quad_no == hi_quad_no) {
        bitquad_t quad = (FULL_QUAD_MASK << lo_offset) & (FULL_QUAD_MASK >> (QUAD_BITS - 1 - hi_offset));
        append_quad(quad);
    }
    else {
        append_quad((FULL_QUAD_MASK << lo_offset) & FULL_QUAD_MASK);
        append_run(Full, hi_quad_no - (lo_quad_no + 1));
        append_quad((FULL_QUAD_MASK >> (QUAD_BITS - 1 - hi_offset)) & FULL_QUAD_MASK);
    }
    if (hi_quad_no < UNICODE_QUAD_COUNT - 1) {
        append_run(Empty, UNICODE_QUAD_COUNT - (hi_quad_no + 1));
    }
}

