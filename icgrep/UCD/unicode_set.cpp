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
#include <llvm/ADT/SmallVector.h>

namespace UCD {

using bitquad_t = UnicodeSet::bitquad_t;
using run_t = UnicodeSet::run_t;

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

const auto QUAD_BITS = (8 * sizeof(bitquad_t));
const auto QUAD_LIMIT = (QUAD_BITS - 1);
const auto UNICODE_QUAD_COUNT = (UNICODE_MAX + 1) / QUAD_BITS;
const bitquad_t FULL_QUAD_MASK = ~static_cast<bitquad_t>(0);

inline run_type_t typeOf(const run_t & run) {
    return run.first;
}

inline UnicodeSet::length_t lengthOf(const run_t & run) {
    return run.second;
}

template<typename T>
T * copyOf(const T * base, const uint32_t length, SlabAllocator<> & allocator) {
    T * const ptr = allocator.allocate<T>(length);
    std::memcpy(ptr, base, length * sizeof(T));
    return ptr;
}

using RunVector = llvm::SmallVector<run_t, 32>;
using QuadVector = llvm::SmallVector<bitquad_t, 32>;

template<typename RunVector, typename QuadVector>
void assign(UnicodeSet * const self, const RunVector & runs, const QuadVector & quads) noexcept {
    assert (verify(runs.data(), runs.size(), quads.data(), quads.size()));
    const unsigned n = runs.size();
    assert (n > 0 && n < UNICODE_QUAD_COUNT);
    if (self->mRunCapacity < n) {
        UnicodeSet::GlobalAllocator.deallocate<run_t>(self->mRuns, self->mRunCapacity);
        self->mRuns = UnicodeSet::GlobalAllocator.allocate<run_t>(n);
        self->mRunCapacity = n;
    }
    std::memcpy(self->mRuns, runs.data(), n * sizeof(run_t));
    self->mRunLength = n;
    const unsigned m = quads.size();
    assert (m < UNICODE_QUAD_COUNT);
    if (m > 0) {
        if (self->mQuadCapacity < m) {
            UnicodeSet::GlobalAllocator.deallocate<bitquad_t>(self->mQuads, self->mQuadCapacity);
            self->mQuads = UnicodeSet::GlobalAllocator.allocate<bitquad_t>(m);
            self->mQuadCapacity = m;
        }
        std::memcpy(self->mQuads, quads.data(), m * sizeof(bitquad_t));
    }
    self->mQuadLength = m;
}

using namespace llvm;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief append_run
 ** ------------------------------------------------------------------------------------------------------------- */
inline void append_run(const run_type_t type, const unsigned length, RunVector & runs) noexcept {
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
inline void append_quad(const bitquad_t quad, QuadVector & quads, RunVector & runs) noexcept {
    run_type_t type = Empty;
    if (LLVM_UNLIKELY(quad == 0)) {
        type = Empty;
    } else if (LLVM_UNLIKELY(~quad == 0)) {
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
inline bool verify(const run_t * const runs, const uint32_t runLength, const bitquad_t * const quads, const uint32_t quadLength) noexcept {
    unsigned sum = 0;
    unsigned mixedQuads = 0;
    for (unsigned i = 0; i < runLength; ++i) {
        const auto type = typeOf(runs[i]);
        if (LLVM_UNLIKELY(type != Empty && type != Mixed && type != Full)) {
            llvm::errs() << "illegal run type " << type << " found\n";
            return false;
        }
        const auto l = lengthOf(runs[i]);
        if (LLVM_UNLIKELY(l == 0)) {
            llvm::errs() << "zero-length quad found\n";
            return false;
        }
        if (type == Mixed) {
            mixedQuads += l;
        }
        sum += l;
    }
    if (LLVM_UNLIKELY(sum != UNICODE_QUAD_COUNT)) {
        llvm::errs() << "found " << sum << " quads but expected " << UNICODE_QUAD_COUNT << "\n";
        return false;
    }
    if (LLVM_UNLIKELY(mixedQuads != quadLength)) {
        llvm::errs() << "found " << quadLength << " mixed quad(s) but expected " << mixedQuads << "\n";
        return false;
    }
    for (unsigned i = 0; i < quadLength; ++i) {
        if (LLVM_UNLIKELY((quads[i] == 0) || (~quads[i] == 0))) {
            llvm::errs() << "Empty or Full quad found in Mixed quad array\n";
            return false;
        }
    }
    return true;
}
#endif

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief at
 ** ------------------------------------------------------------------------------------------------------------- */
codepoint_t UnicodeSet::at(const size_type k) const {

    size_type base = 0;
    size_type remaining = k;
    const bitquad_t * qi = mQuads;

    for (unsigned i = 0; i < mRunLength; ++i) {
        assert ((base % QUAD_BITS) == 0);
        const auto & r = mRuns[i];
        if (typeOf(r) == Empty) {
            base += lengthOf(r) * QUAD_BITS;
        } else if (typeOf(r) == Full) {
            const auto m = lengthOf(r) * QUAD_BITS;
            if (LLVM_UNLIKELY(remaining < m)) {
                return (base * QUAD_BITS) + remaining;
            }
            base += m;
            remaining -= m * QUAD_BITS;
        } else { // if (typeOf(r) == Mixed) {
            for (auto l = lengthOf(r); l; --l, ++qi) {
                auto q = *qi; assert (q);
                const auto c = popcount<bitquad_t>(q);
                if (LLVM_UNLIKELY(remaining < c)) {
                    // iterate through the remaining bits to find the offset
                    for (;;) {
                        assert (q);
                        const bitquad_t k = scan_forward_zeroes<bitquad_t>(q);
                        if (remaining == 0) {
                            return (base * QUAD_BITS) + k;
                        }
                        q ^= static_cast<bitquad_t>(1) << k;
                        --remaining;
                    }
                }
                ++base;
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
UnicodeSet::size_type UnicodeSet::size() const noexcept {
    return std::distance(begin(), end());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief count
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::size_type UnicodeSet::count() const noexcept {
    size_type count = 0;
    for (unsigned i = 0; i < mRunLength; ++i) {
        const auto & r = mRuns[i];
        if (typeOf(r) == Full) {
            assert (lengthOf(r) > 0);
            count += lengthOf(r);
        }
    }
    count *= QUAD_BITS;
    for (unsigned i = 0; i < mQuadLength; ++i) {
        assert (mQuads[i]);
        count += popcount<bitquad_t>(mQuads[i]);
    }
    return count;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief front
 ** ------------------------------------------------------------------------------------------------------------- */
interval_t UnicodeSet::front() const noexcept {
    return *begin();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief back
 ** ------------------------------------------------------------------------------------------------------------- */
interval_t UnicodeSet::back() const noexcept {
    auto back = begin();
    for (auto i = back; i != end(); back = i++);
    return *back;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief print
 ** ------------------------------------------------------------------------------------------------------------- */
void UnicodeSet::print(llvm::raw_ostream & out) const noexcept {
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
void UnicodeSet::dump(llvm::raw_ostream & out) const noexcept {
    auto qi = mQuads;
    for (unsigned i = 0; i < mRunLength; ++i) {
        const auto & run = mRuns[i];
        const auto l = lengthOf(run);
        if (typeOf(run) == Empty) {
            out << "Empty(" << l << ")\n";
        } else if (typeOf(run) == Full) {
            out << "Full(" << l << ")\n";
        } else {
            for (const auto qi_end = qi + l; qi != qi_end; ++qi) {
                assert (qi != (mQuads + mQuadLength));
                out << "Mixed(" << llvm::format("%08x", *qi) << ")\n";
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief complement
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator~() const noexcept {
    run_t * const runs = GlobalAllocator.allocate<run_t>(mRunLength);
    run_t * ri = runs;
    for (unsigned i = 0; i < mRunLength; ++i) {
        const auto & run = mRuns[i];
        run_type_t type = Mixed;
        if (typeOf(run) == Empty) {            
            type = Full;
        } else if (typeOf(run) == Full) {
            type = Empty;
        }
        *ri++ = { type, lengthOf(run) };
    }
    bitquad_t * quads = nullptr;
    if (mQuadLength > 0) {
        quads = GlobalAllocator.allocate<bitquad_t>(mQuadLength);
        bitquad_t * qi = quads;
        for (unsigned i = 0; i < mQuadLength; ++i) {
            *qi++ = ~mQuads[i];
        }
    }
    return UnicodeSet(runs, mRunLength, mRunLength, quads, mQuadLength, mQuadLength);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief intersection
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator&(const UnicodeSet & other) const noexcept {
    RunVector runs;
    QuadVector quads;
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
    const auto runsLength = runs.size();
    const auto runsCopy = copyOf<run_t>(runs.data(), runsLength, GlobalAllocator);
    const auto quadsLength = quads.size();
    const auto quadsCopy = quadsLength ? copyOf<bitquad_t>(quads.data(), quadsLength, GlobalAllocator) : nullptr;
    return UnicodeSet(runsCopy, runsLength, runsLength, quadsCopy, quadsLength, quadsLength);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief union
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator+(const UnicodeSet & other) const noexcept {
    RunVector runs;
    QuadVector quads;
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
    const auto runsLength = runs.size();
    const auto runsCopy = copyOf<run_t>(runs.data(), runsLength, GlobalAllocator);
    const auto quadsLength = quads.size();
    const auto quadsCopy = quadsLength ? copyOf<bitquad_t>(quads.data(), quadsLength, GlobalAllocator) : nullptr;
    return UnicodeSet(runsCopy, runsLength, runsLength, quadsCopy, quadsLength, quadsLength);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief difference
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator-(const UnicodeSet & other) const noexcept {
    RunVector runs;
    QuadVector quads;
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
    const auto runsLength = runs.size();
    const auto runsCopy = copyOf<run_t>(runs.data(), runsLength, GlobalAllocator);
    const auto quadsLength = quads.size();
    const auto quadsCopy = quadsLength ? copyOf<bitquad_t>(quads.data(), quadsLength, GlobalAllocator) : nullptr;
    return UnicodeSet(runsCopy, runsLength, runsLength, quadsCopy, quadsLength, quadsLength);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief symmetric difference
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet UnicodeSet::operator^(const UnicodeSet & other) const noexcept {
    RunVector runs;
    QuadVector quads;
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
    const auto runsLength = runs.size();
    const auto runsCopy = copyOf<run_t>(runs.data(), runsLength, GlobalAllocator);
    const auto quadsLength = quads.size();
    const auto quadsCopy = quadsLength ? copyOf<bitquad_t>(quads.data(), quadsLength, GlobalAllocator) : nullptr;
    return UnicodeSet(runsCopy, runsLength, runsLength, quadsCopy, quadsLength, quadsLength);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief equality
 ** ------------------------------------------------------------------------------------------------------------- */
bool UnicodeSet::operator==(const UnicodeSet & other) const noexcept {
    if (LLVM_LIKELY(mRunLength != other.mRunLength || mQuadLength != other.mQuadLength)) {
        return false;
    }
    for (unsigned i = 0; i < mQuadLength; ++i) {
        if (mQuads[i] != other.mQuads[i]) {
            return false;
        }
    }
    for (unsigned i = 0; i < mRunLength; ++i) {
        if (mRuns[i] != other.mRuns[i]) {
            return false;
        }
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief less operator
 ** ------------------------------------------------------------------------------------------------------------- */
bool UnicodeSet::operator<(const UnicodeSet & other) const noexcept {
    if (LLVM_LIKELY(mRunLength != other.mRunLength)) {
        return mRunLength < other.mRunLength;
    } else if (LLVM_LIKELY(mQuadLength != other.mQuadLength)) {
        return (mQuadLength < other.mQuadLength);
    } else { // equal run and quad lengths; test their individual values
        for (unsigned i = 0; i < mRunLength; ++i) {
            if (mRuns[i] < other.mRuns[i]) {
                return true;
            } else if (mRuns[i] > other.mRuns[i]) {
                return false;
            }
        }
        for (unsigned i = 0; i < mQuadLength; ++i) {
            if (mQuads[i] < other.mQuads[i]) {
                return true;
            } else if (mQuads[i] > other.mQuads[i]) {
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
    const bitquad_t value = static_cast<bitquad_t>(1) << (cp & QUAD_LIMIT);

    unsigned runIndex = 0;
    unsigned quadIndex = 0;

    length_t length = 0;
    run_type_t type = Empty;
    for (;;) {
        std::tie(type, length) = mRuns[runIndex];
        if (offset < length) {
            break;
        }
        if (type == Mixed) {
            quadIndex += length;
        }
        offset -= length;
        ++runIndex;
    }

    if (LLVM_LIKELY(type == Mixed)) {        
        quadIndex += offset;
        if (LLVM_UNLIKELY(mQuadCapacity == 0)) {
            bitquad_t * const quads = GlobalAllocator.allocate<bitquad_t>(mQuadLength - 1);
            std::memcpy(quads, mQuads, (quadIndex - 1) * sizeof(bitquad_t));
            quads[quadIndex] = mQuads[quadIndex] | value;
            const unsigned offset = (~quads[quadIndex] == 0) ? 1 : 0;
            mQuadCapacity = mQuadLength;
            mQuadLength -= offset;
            ++quadIndex;
            std::memcpy(quads + quadIndex, mQuads + quadIndex + offset, (mQuadLength - quadIndex) * sizeof(bitquad_t));
            mQuads = quads;
            if (LLVM_LIKELY(offset == 0)) {  // no change to runs needed
                assert (verify(mRuns, mRunLength, mQuads, mQuadLength));
                assert (contains(cp));
                return;
            }

        } else { // reuse the buffer
            mQuads[quadIndex] |= value;
            if (LLVM_LIKELY(~mQuads[quadIndex] != 0)) { // no change to runs needed
                assert (verify(mRuns, mRunLength, mQuads, mQuadLength));
                assert (contains(cp));
                return;
            }
            --mQuadLength;
            ++quadIndex;
            std::memmove(mQuads + quadIndex, mQuads + quadIndex + 1, (mQuadLength - quadIndex) * sizeof(bitquad_t));
        }

    } else if (type == Empty) {
        // insert a new quad
        const auto l = mQuadLength + 1;
        if (l > mQuadCapacity) {
            bitquad_t * const quads = GlobalAllocator.allocate<bitquad_t>(l);
            std::memcpy(quads, mQuads, quadIndex * sizeof(bitquad_t));
            quads[quadIndex] = value;
            std::memcpy(quads + quadIndex + 1, mQuads + quadIndex, (mQuadLength - quadIndex) * sizeof(bitquad_t));
            GlobalAllocator.deallocate<bitquad_t>(mQuads, mQuadCapacity);
            mQuads = quads;
            mQuadCapacity = l;
        } else { // reuse the same buffer
            std::memmove(mQuads + quadIndex + 1, mQuads + quadIndex, (mQuadLength - quadIndex) * sizeof(bitquad_t));
            mQuads[quadIndex] = value;
        }
        mQuadLength = l;
    } else { // if (t == Full) {
        assert (contains(cp));
        return;
    }

    if (LLVM_UNLIKELY(length == 1 && mRunCapacity != 0)) { // are we overwriting a 1-length run?
        std::get<0>(mRuns[runIndex]) = (type == Empty) ? Mixed : Full;
    } else {
        const unsigned remaining = length - (offset + 1);
        const auto m = (offset && remaining) ? 2 : 1; // splitting the run in the middle?
        const auto l = mRunLength + m;
        const auto k = runIndex + ((offset != 0) ? 1 : 0);
        if (l > mRunCapacity) {
            run_t * const newRun = GlobalAllocator.allocate<run_t>(l);
            std::memcpy(newRun, mRuns, k * sizeof(run_t));
            std::memcpy(newRun + k + m, mRuns + k, (mRunLength - k) * sizeof(run_t));
            GlobalAllocator.deallocate<run_t>(mRuns, mRunCapacity);
            mRuns = newRun;
            mRunCapacity = l;
        } else { // reuse the same buffer
            std::memmove(mRuns + k + m, mRuns + k, (mRunLength - k) * sizeof(run_t));
        }
        mRuns[k] = std::make_pair(type == Empty ? Mixed : Full, 1);
        if (offset) {
            mRuns[k - 1] = std::make_pair(type, offset);
        }
        if (remaining) {
            mRuns[k + 1] = std::make_pair(type, remaining);
        }
        mRunLength = l;
    }
    assert (verify(mRuns, mRunLength, mQuads, mQuadLength));
    assert (contains(cp));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief insert
 ** ------------------------------------------------------------------------------------------------------------- */
void UnicodeSet::insert(const UnicodeSet & other) noexcept {
    RunVector runs;
    QuadVector quads;
    auto i1 = quad_begin(), i2 = other.quad_begin();
    bool changed = false;
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
            changed |= (i1.type() != Full);
            append_run(Full, n, runs);
            i1 += n;
            i2 += n;
        } else if (i1.type() == Empty) {
            assert (i2.type() == Mixed);
            changed = true;
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
                changed |= (i2.quad() &~ i1.quad());
                append_quad(i1.quad() | i2.quad(), quads, runs);
            }
        }
    }
    assert (i1 == quad_end() && i2 == other.quad_end());
    if (LLVM_LIKELY(changed)) {
        assign(this, runs, quads);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief invert
 ** ------------------------------------------------------------------------------------------------------------- */
void UnicodeSet::invert() noexcept {

    if (LLVM_UNLIKELY(mRunCapacity == 0)) {
        run_t * const runs = GlobalAllocator.allocate<run_t>(mRunLength);
        for (unsigned i = 0; i < mRunLength; ++i) {
            auto & run = mRuns[i];
            const auto l = lengthOf(run);
            if (typeOf(run) == Empty) {
                runs[i] = std::make_pair(Full, l);
            } else if (typeOf(run) == Full) {
                runs[i] = std::make_pair(Empty, l);
            } else {
                runs[i] = std::make_pair(Mixed, l);
            }
        }
        mRuns = runs;
        mRunCapacity = mRunLength;
    } else {
        for (unsigned i = 0; i < mRunLength; ++i) {
            auto & run = mRuns[i];
            if (typeOf(run) == Empty) {
                std::get<0>(run) = Full;
            } else if (typeOf(run) == Full) {
                std::get<0>(run) = Empty;
            }
        }
    }

    if (mQuadLength) {
        if (mQuadCapacity == 0) {
            bitquad_t * const quads = GlobalAllocator.allocate<bitquad_t>(mQuadLength);
            for (unsigned i = 0; i != mQuadLength; ++i) {
                quads[i] = ~mQuads[i];
            }
            mQuads = quads;
            mQuadCapacity = mQuadLength;
        } else {
            for (unsigned i = 0; i != mQuadLength; ++i) {
                mQuads[i] = ~mQuads[i];
            }
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief insert_range
 ** ------------------------------------------------------------------------------------------------------------- */
void UnicodeSet::insert_range(const codepoint_t lo, const codepoint_t hi) {

    if (LLVM_UNLIKELY(lo > hi)) {
        throw std::runtime_error('[' + std::to_string(lo) + ',' + std::to_string(hi) + "] is an illegal codepoint range!");
    } else if (LLVM_UNLIKELY(hi > UNICODE_MAX)) {
        throw std::runtime_error(std::to_string(hi) + " exceeds maximum code point.");
    }

    // Create a temporary run and quad set for the given range
    RunVector runs;
    QuadVector quads;

    codepoint_t lo_index = lo / QUAD_BITS;
    codepoint_t hi_index = hi / QUAD_BITS;

    const run_t * ri = mRuns;
    const run_t * const ri_end = mRuns + mRunLength;

    const bitquad_t * qi = mQuads;
    const bitquad_t * const qi_end = mQuads + mQuadLength;

    codepoint_t length = 0;
    run_type_t type = Empty;

    // Advance past any runs prior to the lo_index
    for (;;) {
        assert (ri < ri_end);
        std::tie(type, length) = *ri;
        if (lo_index < length) {
            break;
        }
        if (type == Mixed) {
            assert ((qi + length) <= qi_end);
            qi += length;
        }
        lo_index -= length;
        hi_index -= length;
        ++ri;
    }

    // Now record the runs and any quads prior to lo_index   
    runs.append<const run_t *>(mRuns, ri++);
    if (lo_index) {
        runs.emplace_back(type, lo_index);
        if (type == Mixed) {
            assert ((qi + lo_index) <= qi_end);
            qi += lo_index;
        }
        hi_index -= lo_index;
        assert (length >= lo_index);
        length -= lo_index;
    }

    quads.append<const bitquad_t *>(mQuads, qi);
    // We now have everything up to the first quad added to our temporary buffers; merge in the first new quad.
    bitquad_t lo_quad = (FULL_QUAD_MASK << (lo & QUAD_LIMIT));
    bitquad_t hi_quad = (FULL_QUAD_MASK >> (QUAD_LIMIT - (hi & QUAD_LIMIT)));

    // If our original quad is full, just append a Full run
    if (LLVM_UNLIKELY(type == Full)) {
        append_run(Full, 1, runs);
    } else { // Otherwise merge the new quad value with the original one
        if (hi_index == 0) {
            lo_quad &= hi_quad;
        }        
        if (type == Mixed) {
            assert (qi < qi_end);
            lo_quad |= *qi++;
        }
        append_quad(lo_quad, quads, runs);
    }

    assert (length > 0);
    --length;

    // Now check if we need to write out any Full blocks between the lo and hi code points; adjust our position
    // in the original quad to suit.
    if (hi_index) {
        // Add in any full runs between the lo and hi quads
        append_run(Full, hi_index - 1, runs);
        // Advance past original quads that were filled in
        while (length < hi_index) {
            if (type == Mixed) {
                assert ((qi + length) <= qi_end);
                qi += length;
            }
            hi_index -= length;
            assert (ri < ri_end);
            std::tie(type, length) = *ri++;
        }
        length -= hi_index;
        // Write out the hi_quad value
        if (LLVM_UNLIKELY(type == Full)) {
            append_run(Full, 1, runs);
        } else {
            if (type == Mixed) {
                assert ((qi + hi_index) < qi_end);
                qi += hi_index;
                hi_quad |= *qi++;
            }
            append_quad(hi_quad, quads, runs);
        }
    }
    assert ("We wrote all the runs but still have remaining quads?" && (ri != ri_end || qi == qi_end));
    append_run(type, length, runs);

    // append any remaining values from the original data
    runs.append<const run_t *>(ri, ri_end);
    quads.append<const bitquad_t *>(qi, qi_end);
    assign(this, runs, quads);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief contains
 * @param codepoint
 *
 * Return whether this UnicodeSet contains the specified code point
 ** ------------------------------------------------------------------------------------------------------------- */
bool UnicodeSet::contains(const codepoint_t cp) const noexcept {
    codepoint_t offset = cp / QUAD_BITS;
    for (unsigned runIndex = 0, quadIndex = 0; runIndex < mRunLength; ++runIndex) {
        length_t length = 0;
        run_type_t type = Empty;
        std::tie(type, length) = mRuns[runIndex];
        if (offset < length) {
            if (type == Mixed) {
                return (mQuads[quadIndex + offset] & (static_cast<bitquad_t>(1) << (cp & QUAD_LIMIT))) != 0;
            }
            return (type == Full);
        }
        if (type == Mixed) {
            quadIndex += length;
        }        
        offset -= length;
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief intersects
 * @param lo_codepoint
 * @param hi_codepoint
 *
 * Return true if this UnicodeSet contains any code point between lo and hi (inclusive)
 ** ------------------------------------------------------------------------------------------------------------- */
bool UnicodeSet::intersects(const codepoint_t lo, const codepoint_t hi) const noexcept {
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
bool UnicodeSet::intersects(const UnicodeSet & other) const noexcept {
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
bool UnicodeSet::subset(const UnicodeSet & other) const noexcept {
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
                const bitquad_t m = ((~*mQuadIterator)) & (FULL_QUAD_MASK << mQuadOffset);
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
UnicodeSet::UnicodeSet() noexcept
: mRuns(GlobalAllocator.allocate<run_t>(1))
, mQuads(nullptr)
, mRunLength(1)
, mQuadLength(0)
, mRunCapacity(1)
, mQuadCapacity(0) {
    mRuns[0] = std::make_pair(Empty, UNICODE_QUAD_COUNT);
    assert (verify(mRuns, mRunLength, mQuads, mQuadLength));
}
           
/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Singleton Set Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet(const codepoint_t codepoint) noexcept
: mRuns(GlobalAllocator.allocate<run_t>(3))
, mQuads(GlobalAllocator.allocate<bitquad_t>(1))
, mRunLength(0)
, mQuadLength(1)
, mRunCapacity(3)
, mQuadCapacity(1) {
    assert (codepoint <= UNICODE_MAX);
    const codepoint_t offset = codepoint / QUAD_BITS;
    const codepoint_t remaining = UNICODE_QUAD_COUNT - (offset + 1);    
    unsigned l = 0;
    if (offset) {
        mRuns[l++] = std::make_pair(Empty, offset);
    }
    mRuns[l++] = std::make_pair(Mixed, 1);
    if (remaining) {
        mRuns[l++] = std::make_pair(Empty, remaining);
    }
    mRunLength = l;
    mQuads[0] = static_cast<bitquad_t>(1) << (codepoint & QUAD_LIMIT);
    assert (verify(mRuns, mRunLength, mQuads, mQuadLength));
    assert (contains(codepoint));
    assert (codepoint == 0 || !contains(codepoint - 1));
    assert (codepoint == UNICODE_MAX || !contains(codepoint + 1));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Range Set Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet(const codepoint_t lo, const codepoint_t hi) noexcept
: mRuns(GlobalAllocator.allocate<run_t>(5))
, mQuads(GlobalAllocator.allocate<bitquad_t>(2))
, mRunLength(0)
, mQuadLength(0)
, mRunCapacity(5)
, mQuadCapacity(2) {

    const codepoint_t lo_index = lo / QUAD_BITS;
    const codepoint_t hi_index = hi / QUAD_BITS;
    const codepoint_t lo_offset = lo & QUAD_LIMIT;
    const codepoint_t hi_offset = hi & QUAD_LIMIT;

    assert (lo <= hi);
    assert (hi <= UNICODE_MAX);

    // Assuming that Empty and Full quads can be of 0 length and Mixed are
    // length 1, we have the following cases:

    // LO_INDEX == HI_INDEX && LO_OFFSET == 0 && HI_OFFSET == QUAD_BITS

    //             Empty           Full            Empty
    //      |                 |############|                 |

    // LO_INDEX == HI_INDEX && (LO_OFFSET != 0 || HI_OFFSET != QUAD_BITS)

    //             Empty          Mixed             Empty
    //      |                 |  #######   |                 |

    // LO_INDEX != HI_INDEX && LO_OFFSET != 0 && HI_OFFSET == QUAD_BITS

    //          Empty    Mixed     Full            Empty
    //      |           |  ###|############|                 |

    // LO_INDEX != HI_INDEX && LO_OFFSET == 0 && HI_OFFSET != QUAD_BITS

    //             Empty           Full     Mixed    Empty
    //      |                 |############|###  |           |

    // LO_INDEX != HI_INDEX && LO_OFFSET != 0 && HI_OFFSET != QUAD_BITS

    //          Empty    Mixed     Full     Mixed    Empty
    //      |           |  ###|############|###  |           |

    if (LLVM_LIKELY(lo_index != 0)) {
        mRuns[0] = std::make_pair(Empty, lo_index);
        mRunLength = 1;
    }
    if (lo_index == hi_index) {
        if ((lo_offset == 0) && (hi_offset == QUAD_LIMIT)) {
            mRuns[mRunLength++] = std::make_pair(Full, 1);
        } else {
            mRuns[mRunLength++] = std::make_pair(Mixed, 1);
            const bitquad_t lo_quad = (FULL_QUAD_MASK << lo_offset);
            const bitquad_t hi_quad = (FULL_QUAD_MASK >> (QUAD_LIMIT - hi_offset));
            mQuads[0] = lo_quad & hi_quad;
            mQuadLength = 1;
        }

    } else {
        const auto lm = ((lo_offset != 0) ? 1U : 0U);
        const auto rm = ((hi_offset != QUAD_LIMIT) ? 1U : 0U);
        const auto d = (hi_index - lo_index) - (lm + rm) + 1;
        if (lo_offset != 0) {
            mRuns[mRunLength++] = std::make_pair(Mixed, 1);
            mQuads[0] = (FULL_QUAD_MASK << lo_offset);
            mQuadLength = 1;
        }
        if (d != 0) {
            mRuns[mRunLength++] = std::make_pair(Full, d);
        }
        if (hi_offset != QUAD_LIMIT) {
            mRuns[mRunLength++] = std::make_pair(Mixed, 1);
            const bitquad_t hi_quad = (FULL_QUAD_MASK >> (QUAD_LIMIT - hi_offset));
            mQuads[mQuadLength++] = hi_quad;
        }
    }
    const auto remaining = UNICODE_QUAD_COUNT - (hi_index + 1);
    if (LLVM_LIKELY(remaining != 0)) {
        mRuns[mRunLength++] = std::make_pair(Empty, remaining);
    }
    assert (mRunLength <= mRunCapacity);
    assert (mQuadLength <= mQuadCapacity);
    assert (verify(mRuns, mRunLength, mQuads, mQuadLength));
    assert (contains(lo) && contains(hi));
    assert (lo == 0 || !contains(lo - 1));
    assert (hi == UNICODE_MAX || !contains(hi + 1));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Range List Set Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename iterator>
inline void convertIntervalsToUnicodeSet(const iterator begin, const iterator end, RunVector & runs, QuadVector & quads) noexcept {

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
        const codepoint_t lo_offset = lo & QUAD_LIMIT;
        const codepoint_t hi_offset = hi & QUAD_LIMIT;
        const bitquad_t lo_quad = (FULL_QUAD_MASK << lo_offset);
        const bitquad_t hi_quad = (FULL_QUAD_MASK >> (QUAD_LIMIT - hi_offset));
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
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Interval Range Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet(std::initializer_list<interval_t>::iterator begin, std::initializer_list<interval_t>::iterator end) noexcept
: mRuns(nullptr)
, mQuads(nullptr)
, mRunLength(0)
, mQuadLength(0)
, mRunCapacity(0)
, mQuadCapacity(0) {
    RunVector runs;
    QuadVector quads;

    convertIntervalsToUnicodeSet(begin, end, runs, quads);

    const auto n = runs.size();
    assert (n > 0 && n < UNICODE_QUAD_COUNT);
    mRuns = GlobalAllocator.allocate<run_t>(n);
    mRunCapacity = n;
    mRunLength = n;
    std::memcpy(mRuns, runs.data(), n * sizeof(run_t));

    const auto m = quads.size();
    assert (m < UNICODE_QUAD_COUNT);
    if (m) {
        mQuads = GlobalAllocator.allocate<bitquad_t>(m);
        mQuadCapacity = m;
        mQuadLength = m;
        std::memcpy(mQuads, quads.data(), m * sizeof(bitquad_t));
    }

    assert (verify(mRuns, mRunLength, mQuads, mQuadLength));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Interval Range Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet(const std::vector<interval_t>::iterator begin, const std::vector<interval_t>::iterator end) noexcept
: mRuns(nullptr)
, mQuads(nullptr)
, mRunLength(0)
, mQuadLength(0)
, mRunCapacity(0)
, mQuadCapacity(0) {
    RunVector runs;
    QuadVector quads;

    convertIntervalsToUnicodeSet(begin, end, runs, quads);

    const auto n = runs.size();
    assert (n > 0 && n < UNICODE_QUAD_COUNT);
    mRuns = GlobalAllocator.allocate<run_t>(n);
    mRunCapacity = n;
    mRunLength = n;
    std::memcpy(mRuns, runs.data(), n * sizeof(run_t));

    const auto m = quads.size();
    assert (m < UNICODE_QUAD_COUNT);
    if (m) {
        mQuads = GlobalAllocator.allocate<bitquad_t>(m);
        mQuadCapacity = m;
        mQuadLength = m;
        std::memcpy(mQuads, quads.data(), m * sizeof(bitquad_t));
    }

    assert (verify(mRuns, mRunLength, mQuads, mQuadLength));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Move Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet(const UnicodeSet && other) noexcept
: mRuns(other.mRuns)
, mQuads(other.mQuads)
, mRunLength(other.mRunLength)
, mQuadLength(other.mQuadLength)
, mRunCapacity(other.mRunLength)
, mQuadCapacity(other.mQuadLength) {
    assert (verify(mRuns, other.mRunLength, mQuads, other.mQuadLength));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Move Assignment Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet & UnicodeSet::operator=(const UnicodeSet && other) noexcept {
    if (mRunCapacity) {
        GlobalAllocator.deallocate<run_t>(mRuns, mRunCapacity);
    }
    if (mQuadCapacity) {
        GlobalAllocator.deallocate<bitquad_t>(mQuads, mQuadCapacity);
    }
    mRuns = other.mRuns;
    mQuads = other.mQuads;
    mRunLength = other.mRunLength;
    mQuadLength = other.mQuadLength;
    mRunCapacity = other.mRunCapacity;
    mQuadCapacity = other.mQuadCapacity;
    assert (verify(mRuns, mRunLength, mQuads, mQuadLength));
    return *this;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Copy Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet(const UnicodeSet & other) noexcept
: mRuns(nullptr)
, mQuads(nullptr)
, mRunLength(0)
, mQuadLength(0)
, mRunCapacity(0)
, mQuadCapacity(0) {
    // lazily ensure reallocation on modification if and only if the source cannot modify it
    if (other.mRunCapacity == 0) {
        mRuns = other.mRuns;
        mRunCapacity = 0;
    } else {
        mRuns = copyOf<run_t>(other.mRuns, other.mRunLength, GlobalAllocator);
        mRunCapacity = other.mRunLength;
    }
    mRunLength = other.mRunLength;
    if (other.mQuadCapacity == 0) {
        mQuads = other.mQuads;
        mQuadCapacity = 0;
    } else {
        mQuads = copyOf<bitquad_t>(other.mQuads, other.mQuadCapacity, GlobalAllocator);
        mQuadCapacity = other.mQuadLength;
    }
    mQuadLength = other.mQuadLength;
    assert (verify(mRuns, other.mRunLength, mQuads, other.mQuadLength));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Copy Assignment Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet & UnicodeSet::operator=(const UnicodeSet & other) noexcept {
    if (mRunCapacity) {
        GlobalAllocator.deallocate<run_t>(mRuns, mRunCapacity);
    }
    if (mQuadCapacity) {
        GlobalAllocator.deallocate<bitquad_t>(mQuads, mQuadCapacity);
    }
    // lazily ensure reallocation on modification if and only if the source cannot modify it
    if (other.mRunCapacity == 0) {
        mRuns = other.mRuns;
        mRunCapacity = 0;
    } else {
        mRuns = copyOf<run_t>(other.mRuns, other.mRunLength, GlobalAllocator);
        mRunCapacity = other.mRunLength;
    }
    mRunLength = other.mRunLength;
    if (other.mQuadCapacity == 0) {
        mQuads = other.mQuads;
        mQuadCapacity = 0;
    } else {
        mQuads = copyOf<bitquad_t>(other.mQuads, other.mQuadCapacity, GlobalAllocator);
        mQuadCapacity = other.mQuadLength;
    }
    mQuadLength = other.mQuadLength;
    assert (verify(mRuns, mRunLength, mQuads, mQuadLength));
    return *this;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Internal / Predefined Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet(run_t * const runs, const uint32_t runLength, const uint32_t runCapacity,
                       bitquad_t * const quads, const uint32_t quadLength, const uint32_t quadCapacity) noexcept
: mRuns(runs)
, mQuads(quads)
, mRunLength(runLength)
, mQuadLength(quadLength)
, mRunCapacity(runCapacity)
, mQuadCapacity(quadCapacity) {
    assert (verify(mRuns, mRunLength, mQuads, mQuadLength));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief Deprecated Constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UnicodeSet::UnicodeSet(std::initializer_list<run_t> r, std::initializer_list<bitquad_t> q) noexcept
: mRuns(GlobalAllocator.allocate<run_t>(r.size()))
, mQuads(q.size() == 0 ? nullptr : GlobalAllocator.allocate<bitquad_t>(q.size()))
, mRunLength(r.size())
, mQuadLength(q.size())
, mRunCapacity(r.size())
, mQuadCapacity(q.size()) {
    std::copy(r.begin(), r.end(), mRuns);
    std::copy(q.begin(), q.end(), mQuads);
    assert (verify(mRuns, mRunLength, mQuads, mQuadLength));
}

}
