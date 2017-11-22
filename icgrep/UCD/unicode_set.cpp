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

namespace UCD {

using bitquad_t = UnicodeSet::bitquad_t;
using run_t = UnicodeSet::run_t;
using interval_t = UnicodeSet::interval_t;

//
// Select the correct built-in scan function, dependent on whatever
// bitquad_t resolves to, when scan_forward_zeroes<bitquad_t> is called.
template <typename T> int scan_forward_zeroes(const T x) noexcept;
template <> inline int scan_forward_zeroes<unsigned int>(const unsigned int x) noexcept { return __builtin_ctz(x); }
template <> inline int scan_forward_zeroes<unsigned long>(const unsigned long x) noexcept { return __builtin_ctzl(x); }
template <> inline int scan_forward_zeroes<unsigned long long>(const unsigned long long x) noexcept { return __builtin_ctzll(x); }

template <typename T> unsigned popcount(const T x) noexcept;
template <> inline unsigned popcount<unsigned int>(const unsigned int x) noexcept { return __builtin_popcount(x); }
template <> inline unsigned popcount<unsigned long>(const unsigned long x) noexcept { return __builtin_popcountl(x); }
template <> inline unsigned popcount<unsigned long long>(const unsigned long long x) noexcept { return __builtin_popcountll(x); }


SlabAllocator<> UnicodeSet::GlobalAllocator;

const uint64_t QUAD_BITS = (8 * sizeof(bitquad_t));
const uint64_t MOD_QUAD_BIT_MASK = QUAD_BITS - 1;
const uint64_t UNICODE_QUAD_COUNT = (UNICODE_MAX + 1) / QUAD_BITS;
const bitquad_t FULL_QUAD_MASK = -1;

inline run_type_t typeOf(const run_t & run) {
    return run.first;
}

inline UnicodeSet::length_t lengthOf(const run_t & run) {
    return run.second;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief append_run
 ** ------------------------------------------------------------------------------------------------------------- */
template <class RunVector>
inline void append_run(const run_type_t type, const unsigned length, RunVector & runs) {
    if (LLVM_UNLIKELY(length == 0)) {
        return;
    } else if (!runs.empty() && typeOf(runs.back()) == type) {
        std::get<1>(runs.back()) += length;
        return;
    }
    runs.emplace_back(type, length);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief append_quad
 ** ------------------------------------------------------------------------------------------------------------- */
template <class QuadVector, class RunVector>
inline void append_quad(const bitquad_t quad, QuadVector & quads, RunVector & runs) {
    run_type_t type = Empty;
    if (LLVM_UNLIKELY(quad == 0)) {
        type = Empty;
    } else if (LLVM_UNLIKELY(quad == FULL_QUAD_MASK)) {
        type = Full;
    } else {
        quads.emplace_back(quad);
        type = Mixed;
    }
    append_run(type, 1, runs);
}

#ifndef NDEBUG
/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verify
 *
 * Sanity check for each function that constructs or modifies a UnicodeSet
 ** ------------------------------------------------------------------------------------------------------------- */
template <class RunVector, class QuadVector>
inline bool verify(const RunVector & runs, const QuadVector & quads) {
    unsigned sum = 0;
    unsigned mixedQuads = 0;
    for (auto run : runs) {
        const auto type = typeOf(run);
        if (LLVM_UNLIKELY(type != Empty && type != Mixed && type != Full)) {
            throw std::runtime_error("illegal run type " + std::to_string(type) + " found");
        }
        const auto l = lengthOf(run);
        if (LLVM_UNLIKELY(l == 0)) {
            throw std::runtime_error("zero-length quad found");
        }
        if (type == Mixed) {
            mixedQuads += l;
        }
        sum += l;
    }
    if (LLVM_UNLIKELY(sum != UNICODE_QUAD_COUNT)) {
        throw std::runtime_error("found " + std::to_string(sum) + " quads but expected " + std::to_string(UNICODE_QUAD_COUNT));
    }
    if (LLVM_UNLIKELY(mixedQuads != quads.size())) {
        throw std::runtime_error("found " + std::to_string(quads.size()) + " mixed quad but expected " + std::to_string(mixedQuads));
    }
    for (const auto quad : quads) {
        if (LLVM_UNLIKELY(quad == 0)) {
            throw std::runtime_error("Empty quad found in Mixed quad array!");
        } else if (LLVM_UNLIKELY(quad == FULL_QUAD_MASK)) {
            throw std::runtime_error("Full quad found in Mixed quad array!");
        }
    }
    return true;
}
#endif

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief empty
 ** ------------------------------------------------------------------------------------------------------------- */
bool UnicodeSet::empty() const {
    return (mRuns.size() == 1) && typeOf(mRuns.front()) == Empty;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief full
 ** ------------------------------------------------------------------------------------------------------------- */
bool UnicodeSet::full() const {
    return (mRuns.size() == 1) && typeOf(mRuns.front()) == Full;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief at
 ** ------------------------------------------------------------------------------------------------------------- */
codepoint_t UnicodeSet::at(const size_type k) const {
    auto qi = mQuads.cbegin();
    size_type base = 0;
    size_type remaining = k;
    for (const run_t & r : mRuns) {
        assert ((base % QUAD_BITS) == 0);
        if (typeOf(r) == Empty) {
            base += lengthOf(r) * QUAD_BITS;
        } else if (typeOf(r) == Full) {
            const auto m = lengthOf(r) * QUAD_BITS;
            if (LLVM_UNLIKELY(remaining < m)) {
                return base + remaining;
            }
            base += m;
            remaining -= m;
        } else { // if (typeOf(r) == Mixed) {
            for (auto l = lengthOf(r); l; --l, ++qi) {
                auto q = *qi;
                assert (q);
                const auto c = popcount<bitquad_t>(q);
                if (LLVM_UNLIKELY(remaining < c)) {
                    // iterate through the remaining bits to find the offset
                    for (;;) {
                        assert (q);
                        const bitquad_t k = scan_forward_zeroes<bitquad_t>(q);
                        if (remaining == 0) {
                            return base + k;
                        }
                        q ^= static_cast<bitquad_t>(1) << k;
                        --remaining;
                    }
                }
                base += QUAD_BITS;
                remaining -= c;
            }
        }
    }
    throw std::runtime_error("cannot retrieve codepoint " + std::to_string(k) + " from a "
                             " set containing " + std::to_string(count()) + " codepoints");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief size
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::size_type UnicodeSet::size() const {
    return std::distance(begin(), end());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief count
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::size_type UnicodeSet::count() const {
    size_type count = 0;
    for (const run_t & r : mRuns) {
        if (typeOf(r) == Full) {
            count += lengthOf(r);
        }
    }
    count *= QUAD_BITS;
    for (const bitquad_t q : mQuads) {
        count += popcount<bitquad_t>(q);
    }
    return count;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief front
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::interval_t UnicodeSet::front() const {
    return *begin();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief back
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::interval_t UnicodeSet::back() const {
    auto back = begin();
    for (auto i = back; i != end(); back = i++);
    return *back;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief print
 ** ------------------------------------------------------------------------------------------------------------- */
void UnicodeSet::print(llvm::raw_ostream & out) const {
    if (LLVM_UNLIKELY(empty())) {
        out << "()";
    } else {
        char joiner = '(';
        for (auto r : *this) {
            out << joiner << std::get<0>(r);
            if (std::get<0>(r) != std::get<1>(r)) {
                out << '-' << std::get<1>(r);
            }
            joiner = ',';
        }
        out << ')';

    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief dump
 ** ------------------------------------------------------------------------------------------------------------- */
void UnicodeSet::dump(llvm::raw_ostream & out) const {
    auto qi = mQuads.cbegin();
    for (const run_t & run : mRuns) {
        if (typeOf(run) == Empty) {
            out << "Empty(" << lengthOf(run) << ")\n";
        } else if (typeOf(run) == Full) {
            out << "Full(" << lengthOf(run) << ")\n";
        } else {
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
    std::vector<run_t> runs(mRuns.size());
    auto ri = runs.begin();
    for (const auto run : mRuns) {
        run_type_t type = Empty;
        if (typeOf(run) == Empty) {            
            type = Full;
        } else if (typeOf(run) == Full) {
            type = Empty;
        } else {
            type = Mixed;
        }
        *ri++ = { type, lengthOf(run) };
    }
    std::vector<bitquad_t> quads(mQuads.size());
    auto qi = quads.begin();
    for (const auto quad : mQuads) {
        *qi++ = ~quad;
    }
    return UnicodeSet(std::move(runs), std::move(quads), mRuns.get_allocator());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief intersection
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator&(const UnicodeSet & other) const {

    std::vector<run_t> runs;
    std::vector<bitquad_t> quads;    
    
    auto i1 = quad_begin(), i2 = other.quad_begin();
    
    for (;;) {
        assert ("neither run can be zero length unless both are of zero length" && ((i1.length() != 0) ^ (i2.length() == 0)));
        const auto n = std::min(i1.length(), i2.length());
        if (LLVM_UNLIKELY(n == 0)) {
            break;
        }
        if ((i1.type() == Full) && (i2.type() == Full)) {
            append_run(Full, n, runs);
            i1 += n;
            i2 += n;
        } else if ((i1.type() == Empty) || (i2.type() == Empty)) {
            append_run(Empty, n, runs);
            i1 += n;
            i2 += n;
        } else if (i1.type() == Full) {
            for (unsigned i = 0; i != n; ++i, ++i2) {
                append_quad(i2.quad(), quads, runs);
            }
            i1 += n;
        } else if (i2.type() == Full) {
            for (unsigned i = 0; i != n; ++i, ++i1) {
                append_quad(i1.quad(), quads, runs);
            }
            i2 += n;
        } else { // both Mixed
            assert (i1.type() == Mixed && i2.type() == Mixed);
            for (unsigned i = 0; i != n; ++i, ++i1, ++i2) {
                append_quad(i1.quad() & i2.quad(), quads, runs);
            }
        }
    }
    assert (i1 == quad_end() && i2 == other.quad_end());
    return UnicodeSet(std::move(runs), std::move(quads), mRuns.get_allocator());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief union
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator+(const UnicodeSet & other) const {
    std::vector<run_t> runs;
    std::vector<bitquad_t> quads;
    auto i1 = quad_begin(), i2 = other.quad_begin();
    for (;;) {
        assert ("neither run can be zero length unless both are of zero length" && ((i1.length() != 0) ^ (i2.length() == 0)));
        const auto n = std::min(i1.length(), i2.length());
        if (LLVM_UNLIKELY(n == 0)) {
            break;
        }
        if ((i1.type() == Empty) && (i2.type() == Empty)) {
            append_run(Empty, n, runs);
            i1 += n;
            i2 += n;
        } else if ((i1.type() == Full) || (i2.type() == Full)) {
            append_run(Full, n, runs);
            i1 += n;
            i2 += n;
        } else if (i1.type() == Empty) {
            assert (i2.type() == Mixed);
            for (unsigned i = 0; i != n; ++i, ++i2) {
                append_quad(i2.quad(), quads, runs);
            }
            i1 += n;
        } else if (i2.type() == Empty) {
            assert (i1.type() == Mixed);
            for (unsigned i = 0; i != n; ++i, ++i1) {
                append_quad(i1.quad(), quads, runs);
            }
            i2 += n;
        } else { // both Mixed
            assert (i1.type() == Mixed && i2.type() == Mixed);
            for (unsigned i = 0; i < n; ++i, ++i1, ++i2) {
                append_quad(i1.quad() | i2.quad(), quads, runs);
            }
        }
    }
    assert (i1 == quad_end() && i2 == other.quad_end());
    return UnicodeSet(std::move(runs), std::move(quads), mRuns.get_allocator());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief difference
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator-(const UnicodeSet & other) const {
    std::vector<run_t> runs;
    std::vector<bitquad_t> quads;
    auto i1 = quad_begin(), i2 = other.quad_begin();
    for (;;) {
        assert ("neither run can be zero length unless both are of zero length" && ((i1.length() != 0) ^ (i2.length() == 0)));
        const auto n = std::min(i1.length(), i2.length());
        if (LLVM_UNLIKELY(n == 0)) {
            break;
        }
        if ((i1.type() == Empty) || (i2.type() == Full)) {           
            append_run(Empty, n, runs);
            i1 += n;
            i2 += n;
        } else if (i1.type() == Full) {
            if (i2.type() == Empty) {
                append_run(Full, n, runs);
                i2 += n;
            } else {
                for (unsigned i = 0; i != n; ++i, ++i2) {
                    append_quad(~i2.quad(), quads, runs);
                }                
            }
            i1 += n;
        } else if (i2.type() == Empty) {
            for (unsigned i = 0; i != n; ++i, ++i1) {                
                append_quad(i1.quad(), quads, runs);
            }
            i2 += n;
        } else { // both Mixed
            assert (i1.type() == Mixed && i2.type() == Mixed);
            for (unsigned i = 0; i != n; ++i, ++i1, ++i2) {                
                append_quad(i1.quad() &~ i2.quad(), quads, runs);
            }
        }
    }
    assert (i1 == quad_end() && i2 == other.quad_end());
    return UnicodeSet(std::move(runs), std::move(quads), mRuns.get_allocator());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief symmetric difference
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator^(const UnicodeSet & other) const {
    std::vector<run_t> runs;
    std::vector<bitquad_t> quads;
    auto i1 = quad_begin(), i2 = other.quad_begin();
    for (;;) {
        assert ("neither run can be zero length unless both are of zero length" && ((i1.length() != 0) ^ (i2.length() == 0)));
        const auto n = std::min(i1.length(), i2.length());
        if (LLVM_UNLIKELY(n == 0)) {
            break;
        }
        if (i1.type() != Mixed && i2.type() != Mixed) {
            append_run(i1.type() == i2.type() ? Empty : Full, n, runs);
            i1 += n;
            i2 += n;
        } else if (i1.type() == Empty) {
            for (unsigned i = 0; i < n; ++i, ++i2) {
                append_quad(i2.quad(), quads, runs);
            }
            i1 += n;
        } else if (i2.type() == Empty) {
            for (unsigned i = 0; i < n; ++i, ++i1) {
                append_quad(i1.quad(), quads, runs);
            }
            i2 += n;
        } else if (i1.type() == Full) {
            for (unsigned i = 0; i < n; ++i, ++i2) {
                append_quad(~i2.quad(), quads, runs);
            }
            i1 += n;
        } else if (i2.type() == Full) {
            for (unsigned i = 0; i < n; ++i, ++i1) {
                append_quad(~i1.quad(), quads, runs);
            }
            i2 += n;
        } else { // both Mixed
            assert (i1.type() == Mixed && i2.type() == Mixed);
            for (unsigned i = 0; i != n; ++i, ++i1, ++i2) {
                append_quad(i1.quad() ^ i2.quad(), quads, runs);
            }
        }
    }
    assert (i1 == quad_end() && i2 == other.quad_end());
    return UnicodeSet(std::move(runs), std::move(quads), mRuns.get_allocator());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief equality
 ** ------------------------------------------------------------------------------------------------------------- */
bool UnicodeSet::operator==(const UnicodeSet & other) const {
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
 * @brief less operator
 ** ------------------------------------------------------------------------------------------------------------- */
bool UnicodeSet::operator<(const UnicodeSet & other) const {
    if (LLVM_LIKELY(mRuns.size() != other.mRuns.size())) {
        return mRuns.size() < other.mRuns.size();
    } else if (LLVM_LIKELY(mQuads.size() != other.mQuads.size())) {
        return (mQuads.size() < other.mQuads.size());
    } else { // equal run and quad lengths; test their individual values
        for (auto ri = mRuns.cbegin(), end = mRuns.cend(), rj = other.mRuns.cbegin(); ri != end; ++ri, ++rj) {
            if (*ri < *rj) {
                return true;
            } else if (*ri > *rj) {
                return false;
            }
        }
        for (auto qi = mQuads.cbegin(), end = mQuads.cend(), qj = other.mQuads.cbegin(); qi != end; ++qi, ++qj) {
            if (*qi < *qj) {
                return true;
            } else if (*qi > *qj) {
                return false;
            }
        }
        return false;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief insert
 ** ------------------------------------------------------------------------------------------------------------- */
void UnicodeSet::insert(const codepoint_t cp) {

    if (LLVM_UNLIKELY(cp > UNICODE_MAX)) {
        throw std::runtime_error(std::to_string(cp) + " exceeds maximum code point.");
    }

    codepoint_t offset = cp / QUAD_BITS;
    const bitquad_t value = static_cast<bitquad_t>(1) << (cp & MOD_QUAD_BIT_MASK);
    auto run = mRuns.begin();
    auto quad = mQuads.begin();
    length_t l = 0;
    run_type_t t = Empty;
    for (;;) {
        std::tie(t, l) = *run;
        if (offset < l) {
            break;
        }
        if (t == Mixed) {
            quad += l;
        }
        offset -= l;
        ++run;
    }
    if (LLVM_LIKELY(t == Mixed)) {
        quad += offset;
        *quad |= value;
        if (LLVM_LIKELY(*quad != FULL_QUAD_MASK)) {
            assert (contains(cp));
            return;
        }
        mQuads.erase(quad);
    } else if (t == Empty) {
        mQuads.insert(quad, value);
    } else { // if (t == Full) {
        assert (contains(cp));
        return;
    }
    const unsigned remaining = l - (offset + 1);
    if (offset == 0) {
        *run = std::make_pair(t == Empty ? Mixed : Full, 1);
        if (remaining != 0) {
            mRuns.insert(++run, std::make_pair(t, remaining));
        }
    } else {
        run->second = offset;
        if (remaining == 0) {
            mRuns.insert(++run, std::make_pair(t == Empty ? Mixed : Full, 1));
        } else {
            mRuns.insert(++run, {std::make_pair(t == Empty ? Mixed : Full, 1), std::make_pair(t, remaining)});
        }
    }

    assert (verify(mRuns, mQuads));
    assert (contains(cp));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief insert_range
 ** ------------------------------------------------------------------------------------------------------------- */
void UnicodeSet::insert_range(const codepoint_t lo, const codepoint_t hi)  {

    if (LLVM_UNLIKELY(lo > hi)) {
        throw std::runtime_error('[' + std::to_string(lo) + ',' + std::to_string(hi) + "] is an illegal codepoint range!");
    } else if (LLVM_UNLIKELY(hi > UNICODE_MAX)) {
        throw std::runtime_error(std::to_string(hi) + " exceeds maximum code point.");
    }

    // Create a temporary run and quad set for the given range
    std::vector<run_t> runs;
    std::vector<bitquad_t> quads;

    codepoint_t lo_index = lo / QUAD_BITS;
    codepoint_t hi_index = hi / QUAD_BITS;

    auto ri = mRuns.cbegin();
    auto qi = mQuads.cbegin();

    codepoint_t length = 0;
    run_type_t type = Empty;

    // Advance past any runs prior to the lo_index
    for (;;) {
        assert (ri != mRuns.cend());
        std::tie(type, length) = *ri;       
        if (lo_index < length) {
            break;
        }
        if (type == Mixed) {
            assert (std::distance(qi, mQuads.cend()) >= length);
            qi += length;
        }
        lo_index -= length;
        hi_index -= length;
        ++ri;
    }

    // Now record the runs and any quads prior to lo_index
    runs.assign(mRuns.cbegin(), ri++);
    if (lo_index) {
        runs.emplace_back(type, lo_index);
        if (type == Mixed) {
            assert (static_cast<codepoint_t>(std::distance(qi, mQuads.cend())) >= lo_index);
            qi += lo_index;
        }
        hi_index -= lo_index;
        length -= lo_index;
    }

    quads.assign(mQuads.cbegin(), qi);
    // We now have everything up to the first quad added to our temporary buffers; merge in the first new quad.
    bitquad_t lo_quad = (FULL_QUAD_MASK << (lo & MOD_QUAD_BIT_MASK));
    bitquad_t hi_quad = (FULL_QUAD_MASK >> (QUAD_BITS - 1 - (hi & MOD_QUAD_BIT_MASK)));

    // If our original quad is full, just append a Full run
    if (LLVM_UNLIKELY(type == Full)) {
        append_run(Full, 1, runs);
    } else { // Otherwise merge the new quad value with the original one
        if (hi_index == 0) {
            lo_quad &= hi_quad;
        }
        if (type == Mixed) {
            assert (std::distance(qi, mQuads.cend()) > 0);
            lo_quad |= *qi++;
        }
        append_quad(lo_quad, quads, runs);
    }
    --length;

    // Now check if we need to write out any Full blocks between the lo and hi code points; adjust our position
    // in the original quad to suit.
    if (hi_index) {
        // Add in any full runs between the lo and hi quads
        append_run(Full, hi_index - 1, runs);
        // Advance past original quads that were filled in
        while (ri != mRuns.cend()) {
            if (type == Mixed) {
                assert (std::distance(qi, mQuads.cend()) >= length);
                qi += length;
            }
            std::tie(type, length) = *ri++;
            if (hi_index < length) {
                break;
            }
            hi_index -= length;
            length = 0;
        }        
        // Write out the hi_quad value
        if (LLVM_UNLIKELY(type == Full)) {
            append_run(Full, 1, runs);
        } else {
            if (type == Mixed) {
                assert (static_cast<codepoint_t>(std::distance(qi, mQuads.cend())) > hi_index);
                qi += hi_index;
                hi_quad |= *qi++;
            }
            append_quad(hi_quad, quads, runs);
        }
    }

    // And append any remaining values from the original data
    assert (length >= hi_index);
    append_run(type, length - hi_index, runs);
    assert ("We wrote all the runs but still have remaining quads?" && (ri != mRuns.cend() || qi == mQuads.cend()));
    runs.insert(runs.end(), ri, mRuns.cend());
    quads.insert(quads.end(), qi, mQuads.cend());
    assert (verify(runs, quads));
    mRuns.assign(runs.cbegin(), runs.cend());
    mQuads.assign(quads.cbegin(), quads.cend());    
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief contains
 * @param codepoint
 *
 * Return whether this UnicodeSet contains the specified code point
 ** ------------------------------------------------------------------------------------------------------------- */
bool UnicodeSet::contains(const codepoint_t cp) const {
    codepoint_t offset = cp / QUAD_BITS;
    auto quad = mQuads.cbegin();
    for (const auto r : mRuns) {
        length_t l = 0;
        run_type_t t = Empty;
        std::tie(t, l) = r;
        if (offset < l) {
            if (t == Mixed) {
                quad += offset;
                return (*quad & static_cast<bitquad_t>(1) << (cp & MOD_QUAD_BIT_MASK)) != 0;
            }
            return (t == Full);
        }
        if (t == Mixed) {
            quad += l;
        }        
        offset -= l;
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief intersects
 * @param lo_codepoint
 * @param hi_codepoint
 *
 * Return true if this UnicodeSet contains any code point(s) between lo and hi (inclusive)
 ** ------------------------------------------------------------------------------------------------------------- */
bool UnicodeSet::intersects(const codepoint_t lo, const codepoint_t hi) const {
    for (auto range : *this) {
        if (range.second < lo) {
            continue;
        }
        if (range.first > hi) {
            break;
        }
        return true;
    }
    return false;
}
       
/** ------------------------------------------------------------------------------------------------------------- *
 * @brief intersects
 * @param other
 *
 * Return true if this UnicodeSet has a non-empty intersection with other
 ** ------------------------------------------------------------------------------------------------------------- */
bool UnicodeSet::intersects(const UnicodeSet & other) const {
    for (auto i1 = quad_begin(), i2 = other.quad_begin();; ) {
        auto n = std::min(i1.length(), i2.length());
        if (LLVM_UNLIKELY(n == 0)) {
            assert (i1 == quad_end() && i2 == other.quad_end());
            return false;
        }
        if (i1.type() == Empty || i2.type() == Empty) {
            i1 += n;
            i2 += n;
        } else if (i1.type() == Full || i2.type() == Full) {
            return true;
        } else { //both Mixed
            assert (i1.type() == Mixed && i2.type() == Mixed);
            for (; n; --n, ++i1, ++i2) {
                if ((i1.quad() & i2.quad()) != 0) return true;
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isSubsetOf
 * @param other
 *
 * Return true if this UnicodeSet is a subset of other
 ** ------------------------------------------------------------------------------------------------------------- */
bool UnicodeSet::subset(const UnicodeSet & other) const {
    for (auto i1 = quad_begin(), i2 = other.quad_begin();; ) {
        auto n = std::min(i1.length(), i2.length());
        if (LLVM_UNLIKELY(n == 0)) {
            assert (i1 == quad_end() && i2 == other.quad_end());
            return true;
        }
        if (i1.type() == Empty || i2.type() == Full) {
            i1 += n;
            i2 += n;
        } else if (i1.type() == Full || i2.type() == Empty) {
            return false;
        } else { //both Mixed
            assert (i1.type() == Mixed && i2.type() == Mixed);
            for (; n; --n, ++i1, ++i2) {
                if (i1.quad() &~ i2.quad()) return false;
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief UnicodeSet::quad_iterator::advance
 ** ------------------------------------------------------------------------------------------------------------- */
void UnicodeSet::quad_iterator::advance(unsigned n) {
    assert (mRemaining > 0);
    while (n > 0) {    
        if (mRemaining > n) {
            if (mType == Mixed) {
                assert (mQuadIterator <= (mQuadEnd - n));
                mQuadIterator += n;
            }
            mRemaining -= n;
            break;
        }
        if (mType == Mixed) {
            assert (mQuadIterator <= (mQuadEnd - mRemaining));
            mQuadIterator += mRemaining;
        }
        n -= mRemaining;
        ++mRunIterator;
        if (LLVM_UNLIKELY(mRunIterator == mRunEnd)) {
            mType = Empty;
            mRemaining = 0;
            break;
        }
        mType = typeOf(*mRunIterator);
        mRemaining = lengthOf(*mRunIterator);
    }
    assert ("remaining length cannot be 0 unless this is the final run" && ((mRunIterator != mRunEnd) || (mRemaining == 0)));
    assert ("cannot be the final quad unless this is the final run" && ((mRunIterator != mRunEnd) || (mQuadIterator == mQuadEnd)));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief UnicodeSet::iterator::advance
 ** ------------------------------------------------------------------------------------------------------------- */
void UnicodeSet::iterator::advance(const unsigned n) {

    assert (n == 1);

    if (LLVM_UNLIKELY(mMinCodePoint > UNICODE_MAX)) {
        throw std::runtime_error("UnicodeSet iterator exceeded maximum code point.");
    }

    bool found = false;
    // Find the start of our interval
    while ( mBaseCodePoint <= UNICODE_MAX ) {
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
        } else { // if (typeOf(t) == Mixed)
            while (mMixedRunIndex != lengthOf(*mRunIterator)) {
                const bitquad_t m = (*mQuadIterator) & (FULL_QUAD_MASK << mQuadOffset);
                // If we found a marker in m, it marks the beginning of our current interval.
                // Find it and break out of the loop.
                if (m) {
                    mQuadOffset = scan_forward_zeroes<bitquad_t>(m);
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
        assert (mBaseCodePoint == (UNICODE_MAX+1));
        mMinCodePoint = (UNICODE_MAX+1);
        return;
    }

    // at this stage, the max code point is the previous max code point (initially 0)
    assert (mMaxCodePoint <= mMinCodePoint);
    found = false;
    // Find the end of our interval
    while ( mBaseCodePoint <= UNICODE_MAX ) {

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
        } else { // if (typeOf(t) == Mixed)
            while (mMixedRunIndex != lengthOf(*mRunIterator)) {
                const bitquad_t m = ((~(*mQuadIterator)) & FULL_QUAD_MASK) & (FULL_QUAD_MASK << mQuadOffset);

                // If we found a marker in m, it marks the end of our current interval.
                // Find it and break out of the loop.
                if (m) {
                    mQuadOffset = scan_forward_zeroes<bitquad_t>(m);
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
    // if the very last block is a mixed block and we go past it, the last code point of the range is UNICODE_MAX
    if (!found) {
        assert (mBaseCodePoint == (UNICODE_MAX+1));
        mMaxCodePoint = UNICODE_MAX;
    }

    assert (mMinCodePoint <= mMaxCodePoint);
}
    
/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Empty/Full Set Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet(run_type_t emptyOrFull, ProxyAllocator<> allocator)
: mRuns(allocator)
, mQuads(allocator) {
    assert((emptyOrFull == Empty) || (emptyOrFull == Full));
    append_run(emptyOrFull, UNICODE_QUAD_COUNT, mRuns);
    assert (verify(mRuns, mQuads));
}
           
/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Singleton Set Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet(const codepoint_t codepoint, ProxyAllocator<> allocator)
: mRuns(allocator)
, mQuads(allocator) {
    const codepoint_t quad_no = codepoint / QUAD_BITS;
    append_run(Empty, quad_no, mRuns);
    append_quad(static_cast<bitquad_t>(1) << (codepoint & MOD_QUAD_BIT_MASK), mQuads, mRuns);
    append_run(Empty, UNICODE_QUAD_COUNT - (quad_no + 1), mRuns);
    assert (verify(mRuns, mQuads));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Range Set Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet(const codepoint_t lo, const codepoint_t hi, ProxyAllocator<> allocator)
: mRuns(allocator)
, mQuads(allocator)
{
    const codepoint_t lo_index = lo / QUAD_BITS;
    const codepoint_t hi_index = hi / QUAD_BITS;
    const codepoint_t lo_offset = lo & MOD_QUAD_BIT_MASK;
    const codepoint_t hi_offset = hi & MOD_QUAD_BIT_MASK;
    const bitquad_t lo_quad = (FULL_QUAD_MASK << lo_offset);
    const bitquad_t hi_quad = (FULL_QUAD_MASK >> (QUAD_BITS - 1 - hi_offset));
    append_run(Empty, lo_index, mRuns);
    bitquad_t mask = hi_quad;
    if (lo_index == hi_index) {
        mask &= lo_quad;
    } else {
        append_quad(lo_quad, mQuads, mRuns);
        append_run(Full, hi_index - (lo_index + 1), mRuns);
    }
    append_quad(mask, mQuads, mRuns);
    append_run(Empty, UNICODE_QUAD_COUNT - (hi_index + 1), mRuns);
    assert (verify(mRuns, mQuads));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Range List Set Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename itr>
void convertIntervalRangesToSparseSet(const itr begin, const itr end, UnicodeSet::RunVector & mRuns, UnicodeSet::QuadVector & mQuads) {

    std::vector<run_t> runs;
    std::vector<bitquad_t> quads;

    assert ("interval list must be totally ordered" && std::is_sorted(begin, end, [](const interval_t l, const interval_t r) {
        if (l.first > l.second) return false;
        if (l.second > UNICODE_MAX) return false;
        if (r.first > r.second) return false;
        if (r.second > UNICODE_MAX) return false;
        return l.second < r.first;
    }));
    
    codepoint_t prior_index = 0;
    bitquad_t mask = 0;
    for (auto interval = begin; interval != end; ) {
        codepoint_t lo, hi;
        std::tie(lo, hi) = *interval;
        const codepoint_t lo_index = (lo / QUAD_BITS);
        const codepoint_t hi_index = (hi / QUAD_BITS);
        const codepoint_t lo_offset = lo & MOD_QUAD_BIT_MASK;
        const codepoint_t hi_offset = hi & MOD_QUAD_BIT_MASK;
        const bitquad_t lo_quad = (FULL_QUAD_MASK << lo_offset);
        const bitquad_t hi_quad = (FULL_QUAD_MASK >> (QUAD_BITS - 1 - hi_offset));
        append_run(Empty, lo_index - prior_index, runs);
        if (lo_index == hi_index) {
            mask |= (lo_quad & hi_quad);
        } else {
            append_quad(mask | lo_quad, quads, runs);
            append_run(Full, hi_index - (lo_index + 1), runs);
            mask = hi_quad;
        }
        // if the next interval is in our current quad, continue to the next range
        prior_index = hi_index;
        if (LLVM_LIKELY(++interval != end)) {
            if (hi_index == (interval->first / QUAD_BITS)) {
                continue;
            }
        }
        append_quad(mask, quads, runs);
        mask = 0;
        ++prior_index;
    }
    assert (mask == 0);
    if (prior_index < UNICODE_QUAD_COUNT) {
        append_run(Empty, UNICODE_QUAD_COUNT - prior_index, runs);
    }    
    mRuns.assign(runs.begin(), runs.end());
    mQuads.assign(quads.begin(), quads.end());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Interval Range Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet(std::initializer_list<interval_t>::iterator begin, std::initializer_list<interval_t>::iterator end, ProxyAllocator<> allocator)
: mRuns(allocator)
, mQuads(allocator) {
    convertIntervalRangesToSparseSet(begin, end, mRuns, mQuads);
    assert (verify(mRuns, mQuads));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Interval Range Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet(const std::vector<interval_t>::iterator begin, const std::vector<interval_t>::iterator end, ProxyAllocator<> allocator)
: mRuns(allocator)
, mQuads(allocator) {
    convertIntervalRangesToSparseSet(begin, end, mRuns, mQuads);
    assert (verify(mRuns, mQuads));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Copy Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet(const UnicodeSet & other, ProxyAllocator<> allocator)
: mRuns(other.mRuns, allocator)
, mQuads(other.mQuads, allocator) {
    assert (verify(mRuns, mQuads));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Initializer Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet(std::initializer_list<run_t> r, std::initializer_list<bitquad_t> q, ProxyAllocator<> allocator)
: mRuns(r.begin(), r.end(), allocator)
, mQuads(q.begin(), q.end(), allocator) {
    assert (verify(mRuns, mQuads));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Internal Vector Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
inline UnicodeSet::UnicodeSet(std::vector<run_t> && r, std::vector<bitquad_t> && q, ProxyAllocator<> allocator)
: mRuns(r.begin(), r.end(), allocator)
, mQuads(q.begin(), q.end(), allocator) {
    assert (verify(mRuns, mQuads));
}

}

