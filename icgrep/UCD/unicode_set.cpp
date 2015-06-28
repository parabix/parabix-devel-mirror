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
#include <llvm/Support/Format.h>
#include <include/simd-lib/builtins.hpp>
#include <iostream>
#include <iomanip>

const size_t QUAD_BITS = (8 * sizeof(bitquad_t));
const size_t MOD_QUAD_BIT_MASK = QUAD_BITS - 1;
const size_t UNICODE_QUAD_COUNT = 0x110000 / QUAD_BITS;
const bitquad_t FULL_QUAD_MASK = -1;

std::string run_type_name(const run_type_t type) {
    if (type == Empty) {
        return "Empty";
    }
    if (type == Full) {
        return "Full";
    }
    if (type == Mixed) {
        return "Mixed";
    }
    return "???";
}

using RunVector = UnicodeSet::RunVector;
using QuadVector = UnicodeSet::QuadVector;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief append_run
 ** ------------------------------------------------------------------------------------------------------------- */
inline void append_run(const run_type_t type, const unsigned length, RunVector & runs) {
    if (LLVM_UNLIKELY(length == 0)) {
        return;
    }
    else if (!runs.empty() && runs.back().mType == type) {
        runs.back().mRunLength += length;
        return;
    }
    runs.emplace_back(type, length);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief append_quad
 ** ------------------------------------------------------------------------------------------------------------- */
inline void append_quad(const bitquad_t quad, QuadVector & quads, RunVector & runs) {
    run_type_t type = Empty;
    if (LLVM_UNLIKELY(quad == 0)) {
        type = Empty;
    }
    else if (LLVM_UNLIKELY(quad == FULL_QUAD_MASK)) {
        type = Full;
    }
    else {
        quads.emplace_back(quad);
        type = Mixed;
    }
    append_run(type, 1, runs);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief dump
 ** ------------------------------------------------------------------------------------------------------------- */
void UnicodeSet::dump(llvm::raw_ostream & out) const {
    auto qi = mQuads.cbegin();
    for (const RunStructure & run : mRuns) {
        if (run.mType == Empty) {
            out << "Empty(" << run.mRunLength << ")\n";
        }
        else if (run.mType == Full) {
            out << "Full(" << run.mRunLength << ")\n";
        }
        else {
            for (const auto qi_end = qi + run.mRunLength; qi != qi_end; ++qi) {
                assert (qi != mQuads.cend());
                out << "Mixed(" << llvm::format("%08x", *qi) << ")\n";
            }
        }
    }
    out.flush();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief complement
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::complement() const {
    RunVector runs;
    QuadVector quads;
    runs.reserve(mRuns.size());
    quads.reserve(mQuads.size());
    auto qi = quads.cbegin();
    for (const RunStructure & run : mRuns) {
        if (run.mType == Empty) {
            append_run(Full, run.mRunLength, runs);
        }
        else if (run.mType == Full) {
            append_run(Empty, run.mRunLength, runs);
        }
        else {
            for (const auto qi_end = qi + run.mRunLength; qi != qi_end; ++qi) {
                assert (qi != quads.cend());
                append_quad(FULL_QUAD_MASK ^ *qi, quads, runs);
            }
        }
    }
    return UnicodeSet(std::move(runs), std::move(quads));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief intersection
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator&(const UnicodeSet & other) const {
    RunVector runs;
    QuadVector quads;
    const auto e1 = quad_end();
    const auto e2 = other.quad_end();
    for (auto i1 = quad_begin(), i2 = other.quad_begin(); i1 != e1 && i2 != e2; ) {
        const auto run1 = i1.getRun();
        const auto run2 = i2.getRun();
        const auto n = std::min(run1.mRunLength, run2.mRunLength);
        if (run1.mType == run2.mType && run1.mType != Mixed) {
            append_run(run1.mType, n, runs);
            i1 += n;
            i2 += n;
        }
        else if (run1.mType == Full) {
            for (unsigned i = 0; i != n; ++i, ++i2) {
                append_quad(i2.getQuad(), quads, runs);
            }
            i1 += n;
        }
        else if (run2.mType == Full) {
            for (unsigned i = 0; i != n; ++i, ++i1) {
                append_quad(i1.getQuad(), quads, runs);
            }
            i2 += n;
        }
        else {
            for (unsigned i = 0; i != n; ++i, ++i1, ++i2) {
                append_quad(i1.getQuad() & i2.getQuad(), quads, runs);
            }
        }
    }
    return UnicodeSet(std::move(runs), std::move(quads));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief union
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator+(const UnicodeSet & other) const {
    RunVector runs;
    QuadVector quads;
    const auto e1 = quad_end();
    const auto e2 = other.quad_end();
    for (auto i1 = quad_begin(), i2 = other.quad_begin(); i1 != e1 && i2 != e2; ) {
        const auto run1 = i1.getRun();
        const auto run2 = i2.getRun();

        const auto n = std::min(run1.mRunLength, run2.mRunLength);
        if ((run1.mType == Empty) && (run2.mType == Empty)) {
            append_run(Empty, n, runs);
            i1 += n;
            i2 += n;
        }
        else if ((run1.mType == Full) || (run2.mType == Full)) {
            append_run(Full, n, runs);
            i1 += n;
            i2 += n;
        }
        else if (run1.mType == Empty) {
            for (unsigned i = 0; i != n; ++i, ++i2) {
                append_quad(i2.getQuad(), quads, runs);
            }
            i1 += n;
        }
        else if (run2.mType == Empty) {
            for (unsigned i = 0; i != n; ++i, ++i1) {
                append_quad(i1.getQuad(), quads, runs);
            }
            i2 += n;
        }
        else {
            for (unsigned i = 0; i < n; ++i, ++i1, ++i2) {
                append_quad(i1.getQuad() | i2.getQuad(), quads, runs);
            }
        }
    }
    return UnicodeSet(std::move(runs), std::move(quads));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief difference
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator-(const UnicodeSet & other) const {
    RunVector runs;
    QuadVector quads;
    const auto e1 = quad_end();
    const auto e2 = other.quad_end();
    for (auto i1 = quad_begin(), i2 = other.quad_begin(); i1 != e1 && i2 != e2; ) {
        const auto run1 = i1.getRun();
        const auto run2 = i2.getRun();
        unsigned n = std::min(run1.mRunLength, run2.mRunLength);
        if ((run1.mType == Empty) || (run2.mType == Full) || (run1.mType == Full && run2.mType == Empty)) {
            append_run(run1.mType, n, runs);
            i1 += n;
            i2 += n;
        }
        else if (run1.mType == Full) {
            for (unsigned i = 0; i != n; ++i, ++i2) {
                append_quad(FULL_QUAD_MASK ^ i2.getQuad(), quads, runs);
            }
            i1 += n;
        }
        else if (run2.mType == Empty) {
            for (unsigned i = 0; i != n; ++i, ++i1) {
                append_quad(i1.getQuad(), quads, runs);
            }
            i2 += n;
        }
        else {
            for (unsigned i = 0; i != n; ++i, ++i1, ++i2) {
                append_quad(i1.getQuad() &~ i2.getQuad(), quads, runs);
            }
        }
    }
    return UnicodeSet(std::move(runs), std::move(quads));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief symmetric difference
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator^(const UnicodeSet & other) const {
    RunVector runs;
    QuadVector quads;
    const auto e1 = quad_end();
    const auto e2 = other.quad_end();
    for (auto i1 = quad_begin(), i2 = other.quad_begin(); i1 != e1 && i2 != e2; ) {
        const auto run1 = i1.getRun();
        const auto run2 = i2.getRun();
        unsigned n = std::min(run1.mRunLength, run2.mRunLength);
        if (run1.mType != Mixed && run2.mType != Mixed) {
            append_run(run1.mType == run2.mType ? Empty : Full, n, runs);
            i1 += n;
            i2 += n;
        }
        else if (run1.mType == Empty) {
            for (unsigned i = 0; i < n; ++i, ++i2) {
                append_quad(i2.getQuad(), quads, runs);
            }
            i1 += n;
        }
        else if (run2.mType == Empty) {
            for (unsigned i = 0; i < n; ++i, ++i1) {
                append_quad(i1.getQuad(), quads, runs);
            }
            i2 += n;
        }
        else if (run1.mType == Full) {
            for (unsigned i = 0; i < n; ++i, ++i2) {
                append_quad(FULL_QUAD_MASK ^ i2.getQuad(), quads, runs);
            }
            i1 += n;
        }
        else if (run2.mType == Empty) {
            for (unsigned i = 0; i < n; ++i, ++i1) {
                append_quad(FULL_QUAD_MASK ^ i1.getQuad(), quads, runs);
            }
            i2 += n;
        }
        else {
            for (unsigned i = 0; i != n; ++i, ++i1, ++i2) {
                append_quad(i1.getQuad() ^ i2.getQuad(), quads, runs);
            }
        }
    }
    return UnicodeSet(std::move(runs), std::move(quads));
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
        const RunStructure & t = mRuns[runIndex];
        if (t.mRunLength >= n) {
            if (t.mType == Mixed) {
                return (mQuads[quadIndex + n - 1] & (static_cast<bitquad_t>(1) << (codepoint & MOD_QUAD_BIT_MASK))) != 0;
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
        const unsigned remain = mRunIterator->mRunLength - mOffset;
        if (remain > n) {
            if (mRunIterator->mType == Mixed) {
                mQuadIterator += n;
            }
            mOffset += n;
            break;
        }
        if (mRunIterator->mType == Mixed) {
            mQuadIterator += remain;
        }
        ++mRunIterator;
        mOffset = 0;
        n -= remain;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief UnicodeSet::iterator::advance
 ** ------------------------------------------------------------------------------------------------------------- */
void UnicodeSet::iterator::advance(const unsigned n) {

    assert (n == 1);

    // Find the start of our interval
    for ( ; mBaseCodePoint <= re::CC::UNICODE_MAX; ++mRunIterator) {
        // Find the first non-empty block
        const RunStructure & run = *mRunIterator;
        if (run.mType != Mixed) {
            mMinCodePoint = mBaseCodePoint;
            mBaseCodePoint += run.mRunLength * QUAD_BITS;
            mQuadOffset = 0;
            mQuadPosition = 0;
            if (run.mType == Full) {
                break;
            }
        }
        else { // if (left.mType == Mixed)
            while (mQuadPosition != run.mRunLength) {
                const bitquad_t q = *mQuadIterator;
                const bitquad_t M = (FULL_QUAD_MASK << mQuadOffset);
                const bitquad_t m = q & M;
                // Nothing left in this quad to add; skip to the next one.
                if (m == 0) {
                    mBaseCodePoint += QUAD_BITS;
                    ++mQuadPosition;
                    ++mQuadIterator;
                    continue;
                }
                mQuadOffset = scan_forward_zeroes(m);
                mMinCodePoint = mBaseCodePoint + mQuadOffset;
                break;
            }
            // If we found nothing in the quad, restart the loop.
            if (mQuadPosition != run.mRunLength) {
                break;
            }
        }
    }

    // Find the end of our interval
    for ( ; mBaseCodePoint <= re::CC::UNICODE_MAX; ++mRunIterator) {
        const RunStructure & run = *mRunIterator;
        // If the next run is empty, we already know the max code point.
        if (run.mType == Empty) {
            mMaxCodePoint = mBaseCodePoint;
            break;
        }
        // If the next run is Full, increment the base code point.
        else if (run.mType == Full) {
            mBaseCodePoint += run.mRunLength * QUAD_BITS;
            mMaxCodePoint = mBaseCodePoint;
            mQuadOffset = 0;
            mQuadPosition = 0;
            continue;
        }
        else { // if (left.mType == Mixed)
            while (mQuadPosition != run.mRunLength) {

                const bitquad_t q = *mQuadIterator;
                const bitquad_t M = (FULL_QUAD_MASK << mQuadOffset);
                const bitquad_t m = ~q & M;

                // Nothing left in this quad to add; skip to the next one.
                if (m == 0) {
                    mBaseCodePoint += QUAD_BITS;
                    mMaxCodePoint = mBaseCodePoint;
                    ++mQuadPosition;
                    ++mQuadIterator;
                    continue;
                }

                mQuadOffset = scan_forward_zeroes(m);
                mMaxCodePoint = mBaseCodePoint + mQuadOffset - 1;
                break;
            }
            // If we found nothing in the quad, restart the loop.
            if (mQuadPosition != run.mRunLength) {
                break;
            }
        }
    }


}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Empty Set Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet() : mRuns({{{Empty, UNICODE_QUAD_COUNT}}}) { }

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Singleton Set Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet(const codepoint_t codepoint) {
    const codepoint_t quad_no = codepoint / QUAD_BITS;
    append_run(Empty, quad_no, mRuns);
    append_quad(static_cast<bitquad_t>(1) << (codepoint & MOD_QUAD_BIT_MASK), mQuads, mRuns);
    append_run(Empty, UNICODE_QUAD_COUNT - (quad_no + 1), mRuns);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Range Set Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet(const codepoint_t lo_codepoint, const codepoint_t hi_codepoint) {
    const codepoint_t lo_quad_no = lo_codepoint / QUAD_BITS;
    const codepoint_t hi_quad_no = hi_codepoint / QUAD_BITS;
    const codepoint_t lo_offset = lo_codepoint & MOD_QUAD_BIT_MASK;
    const codepoint_t hi_offset = hi_codepoint & MOD_QUAD_BIT_MASK;
    const bitquad_t lo_quad = (FULL_QUAD_MASK << lo_offset);
    const bitquad_t hi_quad = (FULL_QUAD_MASK >> (QUAD_BITS - 1 - hi_offset));
    append_run(Empty, lo_quad_no, mRuns);
    if (lo_quad_no == hi_quad_no) {        
        append_quad(lo_quad & hi_quad, mQuads, mRuns);
    }
    else {
        append_quad(lo_quad, mQuads, mRuns);
        append_run(Full, hi_quad_no - (lo_quad_no + 1), mRuns);
        append_quad(hi_quad, mQuads, mRuns);
    }
    append_run(Empty, UNICODE_QUAD_COUNT - (hi_quad_no + 1), mRuns);
}

