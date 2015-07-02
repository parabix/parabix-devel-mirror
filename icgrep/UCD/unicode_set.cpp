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

using namespace re;

namespace UCD {

using bitquad_t = UnicodeSet::bitquad_t;
using run_t = UnicodeSet::run_t;
using RunVector = UnicodeSet::RunVector;
using QuadVector = UnicodeSet::QuadVector;

const size_t QUAD_BITS = (8 * sizeof(bitquad_t));
const size_t MOD_QUAD_BIT_MASK = QUAD_BITS - 1;
const size_t UNICODE_QUAD_COUNT = (CC::UNICODE_MAX + 1) / QUAD_BITS;
const bitquad_t FULL_QUAD_MASK = -1;

inline run_type_t typeOf(const run_t & run) {
    return std::get<0>(run);
}

inline UnicodeSet::length_t lengthOf(const run_t & run) {
    return std::get<1>(run);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief append_run
 ** ------------------------------------------------------------------------------------------------------------- */
inline void append_run(const run_type_t type, const unsigned length, RunVector & runs) {
    if (LLVM_UNLIKELY(length == 0)) {
        return;
    }
    else if (!runs.empty() && typeOf(runs.back()) == type) {
        std::get<1>(runs.back()) += length;
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
 * @brief runLengthSumsUpToUnicodeQuadCount
 *
 * Sanity check for each function that constructs a new UnicodeSet
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool runLengthSumsUpToUnicodeQuadCount(const RunVector & runs) {
    unsigned sum = 0;
    for (auto & run : runs) {
        sum += lengthOf(run);
    }
    return sum == UNICODE_QUAD_COUNT;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief dump
 ** ------------------------------------------------------------------------------------------------------------- */
void UnicodeSet::dump(llvm::raw_ostream & out) const {
    auto qi = mQuads.cbegin();
    for (const run_t & run : mRuns) {
        if (typeOf(run) == Empty) {
            out << "Empty(" << lengthOf(run) << ")\n";
        }
        else if (typeOf(run) == Full) {
            out << "Full(" << lengthOf(run) << ")\n";
        }
        else {
            for (const auto qi_end = qi + lengthOf(run); qi != qi_end; ++qi) {
                assert (qi != mQuads.cend());
                out << "Mixed(" << llvm::format("%08x", *qi) << ")\n";
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief complement
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator~() const {
    RunVector runs;
    QuadVector quads;
    runs.reserve(mRuns.size());
    quads.reserve(mQuads.size());
    auto qi = quads.cbegin();
    for (const run_t & run : mRuns) {
        if (typeOf(run) == Empty) {
            append_run(Full, lengthOf(run), runs);
        }
        else if (typeOf(run) == Full) {
            append_run(Empty, lengthOf(run), runs);
        }
        else {
            for (const auto qi_end = qi + lengthOf(run); qi != qi_end; ++qi) {
                assert (qi != quads.cend());
                append_quad(FULL_QUAD_MASK ^ *qi, quads, runs);
            }
        }
    }
    assert (runLengthSumsUpToUnicodeQuadCount(runs));
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
        const auto n = std::min(i1.length(), i2.length());
        if (i1.type() == i2.type() && i1.type() != Mixed) {
            append_run(i1.type(), n, runs);
            i1 += n;
            i2 += n;
        }
        else if (i1.type() == Full) {
            for (unsigned i = 0; i != n; ++i, ++i2) {
                append_quad(i2.quad(), quads, runs);
            }
            i1 += n;
        }
        else if (i2.type() == Full) {
            for (unsigned i = 0; i != n; ++i, ++i1) {
                append_quad(i1.quad(), quads, runs);
            }
            i2 += n;
        }
        else {
            for (unsigned i = 0; i != n; ++i, ++i1, ++i2) {
                append_quad(i1.quad() & i2.quad(), quads, runs);
            }
        }
    }
    assert (runLengthSumsUpToUnicodeQuadCount(runs));
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
    auto i1 = quad_begin(), i2 = other.quad_begin();
    for (; i1 != e1 && i2 != e2; ) {
        const auto n = std::min(i1.length(), i2.length());
        if ((i1.type() == Empty) && (i2.type() == Empty)) {
            append_run(Empty, n, runs);
            i1 += n;
            i2 += n;
        }
        else if ((i1.type() == Full) || (i2.type() == Full)) {
            append_run(Full, n, runs);
            i1 += n;
            i2 += n;
        }
        else if (i1.type() == Empty) {
            for (unsigned i = 0; i != n; ++i, ++i2) {
                append_quad(i2.quad(), quads, runs);
            }
            i1 += n;
        }
        else if (i2.type() == Empty) {
            for (unsigned i = 0; i != n; ++i, ++i1) {
                append_quad(i1.quad(), quads, runs);
            }
            i2 += n;
        }
        else {
            for (unsigned i = 0; i < n; ++i, ++i1, ++i2) {
                append_quad(i1.quad() | i2.quad(), quads, runs);
            }
        }
    }
    assert (runLengthSumsUpToUnicodeQuadCount(runs));
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
        unsigned n = std::min(i1.length(), i2.length());
        if ((i1.type() == Empty) || (i2.type() == Full) || (i1.type() == Full && i2.type() == Empty)) {
            append_run(i1.type(), n, runs);
            i1 += n;
            i2 += n;
        }
        else if (i1.type() == Full) {
            for (unsigned i = 0; i != n; ++i, ++i2) {
                append_quad(FULL_QUAD_MASK ^ i2.quad(), quads, runs);
            }
            i1 += n;
        }
        else if (i2.type() == Empty) {
            for (unsigned i = 0; i != n; ++i, ++i1) {
                append_quad(i1.quad(), quads, runs);
            }
            i2 += n;
        }
        else {
            for (unsigned i = 0; i != n; ++i, ++i1, ++i2) {
                append_quad(i1.quad() &~ i2.quad(), quads, runs);
            }
        }
    }
    assert (runLengthSumsUpToUnicodeQuadCount(runs));
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
        unsigned n = std::min(i1.length(), i2.length());
        if (i1.type() != Mixed && i2.type() != Mixed) {
            append_run(i1.type() == i2.type() ? Empty : Full, n, runs);
            i1 += n;
            i2 += n;
        }
        else if (i1.type() == Empty) {
            for (unsigned i = 0; i < n; ++i, ++i2) {
                append_quad(i2.quad(), quads, runs);
            }
            i1 += n;
        }
        else if (i2.type() == Empty) {
            for (unsigned i = 0; i < n; ++i, ++i1) {
                append_quad(i1.quad(), quads, runs);
            }
            i2 += n;
        }
        else if (i1.type() == Full) {
            for (unsigned i = 0; i < n; ++i, ++i2) {
                append_quad(FULL_QUAD_MASK ^ i2.quad(), quads, runs);
            }
            i1 += n;
        }
        else if (i2.type() == Empty) {
            for (unsigned i = 0; i < n; ++i, ++i1) {
                append_quad(FULL_QUAD_MASK ^ i1.quad(), quads, runs);
            }
            i2 += n;
        }
        else {
            for (unsigned i = 0; i != n; ++i, ++i1, ++i2) {
                append_quad(i1.quad() ^ i2.quad(), quads, runs);
            }
        }
    }
    assert (runLengthSumsUpToUnicodeQuadCount(runs));
    return UnicodeSet(std::move(runs), std::move(quads));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief equality
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator==(const UnicodeSet & other) const {
    if (mRuns.size() != other.mRuns.size() || mQuads.size() != other.mQuads.size()) {
        return false;
    }
    for (auto i = mQuads.begin(), j = other.mQuads.begin(); i != mQuads.end(); ++i, ++j) {
        if (*i != *j) return false;
    }
    for (auto i = mRuns.begin(), j = other.mRuns.begin(); i != mRuns.end(); ++i, ++j) {
        if (*i != *j) return false;
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief contains
 * @param codepoint
 *
 * Return whether this UnicodeSet contains the specified code point
 ** ------------------------------------------------------------------------------------------------------------- */
bool UnicodeSet::contains(const codepoint_t codepoint) const {
    auto n = codepoint / QUAD_BITS;
    auto qi = mQuads.cbegin();
    for (const auto & r : mRuns) {
        if (lengthOf(r) >= n) {
            if (typeOf(r) == Mixed) {
                qi += n - 1;
                return (*qi & (static_cast<bitquad_t>(1) << (codepoint & MOD_QUAD_BIT_MASK))) != 0;
            }
            return (typeOf(r) == Full);
        }
        if (typeOf(r) == Mixed) {
            qi += lengthOf(r);
        }        
        n -= lengthOf(r);
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief intersects
 * @param lo_codepoint
 * @param hi_codepoint
 *
 * Return true if this UnicodeSet contains any code point(s) between lo_codepoint and hi_codepoint
 ** ------------------------------------------------------------------------------------------------------------- */
bool UnicodeSet::intersects(const codepoint_t lo_codepoint, const codepoint_t hi_codepoint) const {
    quad_iterator qi = quad_begin() + lo_codepoint / QUAD_BITS;
    unsigned n = (hi_codepoint - lo_codepoint) / QUAD_BITS;
    while (n) {
        if (qi.type() != Empty) {
            return true;
        }
        const auto l = std::min<unsigned>(qi.length(), n);
        qi += l;
        n -= l;
    }
    // check the remaining portion of this quad
    unsigned r = (hi_codepoint - lo_codepoint) % QUAD_BITS;
    if (r == 0 || (++qi).type() == Empty) {
        return false;
    }
    if (qi.type() == Full) {
        return true;
    }
    return (qi.quad() & (FULL_QUAD_MASK << r)) != 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief UnicodeSet::quad_iterator::advance
 ** ------------------------------------------------------------------------------------------------------------- */
void UnicodeSet::quad_iterator::advance(unsigned n) {
    while (n > 0) {
        const unsigned remain = lengthOf(*mRunIterator) - mOffset;
        if (remain > n) {
            if (typeOf(*mRunIterator) == Mixed) {
                mQuadIterator += n;
            }
            mOffset += n;
            break;
        }
        if (typeOf(*mRunIterator) == Mixed) {
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

    if (LLVM_UNLIKELY(mMinCodePoint >= 0x110000)) {
        throw std::runtime_error("UnicodeSet iterator exceeded maximum code point.");
    }

    bool found = false;
    // Find the start of our interval
    while ( mBaseCodePoint < 0x110000 ) {
        // Find the first non-empty block
        if (typeOf(*mRunIterator) != Mixed) {            
            // If we found a full run, this must be the start of our interval.
            const auto baseCodePoint = mBaseCodePoint;
            const auto type = typeOf(*mRunIterator);
            mBaseCodePoint += lengthOf(*mRunIterator++) * QUAD_BITS;
            if (type == Full) {
                mMinCodePoint = baseCodePoint;
                found = true;
                break;
            }
        }
        else { // if (typeOf(t) == Mixed)
            while (mMixedRunIndex != lengthOf(*mRunIterator)) {
                const bitquad_t m = (*mQuadIterator) & (FULL_QUAD_MASK << mQuadOffset);
                // If we found a marker in m, it marks the beginning of our current interval.
                // Find it and break out of the loop.
                if (m) {
                    mQuadOffset = scan_forward_zeroes(m);
                    mMinCodePoint = mBaseCodePoint + mQuadOffset;
                    found = true;
                    break;
                }
                mBaseCodePoint += QUAD_BITS;
                ++mQuadIterator;
                ++mMixedRunIndex;
                mQuadOffset = 0;
            }
            if (found) break;
            ++mRunIterator;
            mQuadOffset = 0;
            mMixedRunIndex = 0;
        }
    }

    if (!found) {
        assert (mBaseCodePoint == 0x110000);
        mMinCodePoint = 0x110000;
        return;
    }

    // at this stage, the max code point is the previous max code point (initially 0)
    assert (mMaxCodePoint <= mMinCodePoint);
    found = false;
    // Find the end of our interval
    while ( mBaseCodePoint < 0x110000 ) {

        // Find the first non-Full block
        if (typeOf(*mRunIterator) != Mixed) {
            // If this run is Empty, the max code point is the last computed base code point - 1.
            const auto baseCodePoint = mBaseCodePoint;
            const auto type = typeOf(*mRunIterator);
            mBaseCodePoint += lengthOf(*mRunIterator++) * QUAD_BITS;
            if (type == Empty) {
                mMaxCodePoint = baseCodePoint - 1;
                found = true;
                break;
            }
        }
        else { // if (typeOf(t) == Mixed)
            while (mMixedRunIndex != lengthOf(*mRunIterator)) {
                const bitquad_t m = ((~(*mQuadIterator)) & FULL_QUAD_MASK) & (FULL_QUAD_MASK << mQuadOffset);

                // If we found a marker in m, it marks the end of our current interval.
                // Find it and break out of the loop.
                if (m) {
                    mQuadOffset = scan_forward_zeroes(m);
                    mMaxCodePoint = mBaseCodePoint + mQuadOffset - 1;
                    found = true;
                    break;
                }
                mBaseCodePoint += QUAD_BITS;
                ++mQuadIterator;
                ++mMixedRunIndex;
                mQuadOffset = 0;
            }
            if (found) break;
            ++mRunIterator;
            mQuadOffset = 0;
            mMixedRunIndex = 0;
        }
    }
    // if the very last block is a mixed block and we go past it, the last code point of the range is 0x10FFFF
    if (!found) {
        assert (mBaseCodePoint == 0x110000);
        mMaxCodePoint = 0x10FFFF;
    }

    assert (mMinCodePoint <= mMaxCodePoint);
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
    if (quad_no > 0) {
        append_run(Empty, quad_no, mRuns);
    }
    append_quad(static_cast<bitquad_t>(1) << (codepoint & MOD_QUAD_BIT_MASK), mQuads, mRuns);
    if (quad_no < UNICODE_QUAD_COUNT - 1) {
        append_run(Empty, UNICODE_QUAD_COUNT - (quad_no + 1), mRuns);
    }
    assert (runLengthSumsUpToUnicodeQuadCount(mRuns));
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
    if (hi_quad_no < UNICODE_QUAD_COUNT - 1) {
        append_run(Empty, UNICODE_QUAD_COUNT - (hi_quad_no + 1), mRuns);
    }
    assert (runLengthSumsUpToUnicodeQuadCount(mRuns));
}

}
