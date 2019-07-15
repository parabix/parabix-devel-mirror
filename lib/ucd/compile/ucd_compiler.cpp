#include <ucd/compile/ucd_compiler.hpp>

#include <array>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_ostream.h>
#include <pablo/pe_var.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <pablo/printer_pablos.h>
#include <re/alphabet/alphabet.h>
#include <re/cc/cc_compiler.h>
#include <re/adt/re_name.h>
#include <re/adt/re_cc.h>
#include <ucd/core/unicode_set.h>
#include <ucd/compile/utf8_encoder.h>
#include <ucd/compile/utf16_encoder.h>

using namespace cc;
using namespace re;
using namespace pablo;
using namespace llvm;
using namespace boost::container;

namespace UCD {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief UTF_16 UTF_8
 ** ------------------------------------------------------------------------------------------------------------- */
inline codepoint_t encodingByte(const codepoint_t cp, const unsigned n, bool UTF_16) {
    return UTF_16 ? UTF16_Encoder::encodingByte(cp, n) : UTF8_Encoder::encodingByte(cp, n);
}

inline unsigned length(const codepoint_t cp, bool UTF_16) {
    return UTF_16 ? UTF16_Encoder::length(cp) : UTF8_Encoder::length(cp);
}

inline codepoint_t maxCodePoint(const unsigned length, bool UTF_16) {
    return UTF_16 ?  UTF16_Encoder::maxCodePoint(length) : UTF8_Encoder::maxCodePoint(length);
}

inline bool isLowCodePointAfterByte(const codepoint_t cp, const unsigned n, bool UTF_16) {
    return UTF_16 ? UTF16_Encoder::isLowCodePointAfterByte(cp, n) : UTF8_Encoder::isLowCodePointAfterByte(cp, n);
}
inline bool isHighCodePointAfterByte(const codepoint_t cp, const unsigned n, bool UTF_16) {
    return UTF_16 ? UTF16_Encoder::isHighCodePointAfterByte(cp, n) : UTF8_Encoder::isHighCodePointAfterByte(cp, n);
}
inline codepoint_t minCodePointWithCommonBytes(const re::codepoint_t cp, const unsigned n, bool UTF_16) {
    return UTF_16 ? UTF16_Encoder::minCodePointWithCommonBytes(cp, n) : UTF8_Encoder::minCodePointWithCommonBytes(cp, n);
}
inline codepoint_t maxCodePointWithCommonBytes(const re::codepoint_t cp, const unsigned n, bool UTF_16) {
    return UTF_16 ? UTF16_Encoder::maxCodePointWithCommonBytes(cp, n) : UTF8_Encoder::maxCodePointWithCommonBytes(cp, n);
}

const UCDCompiler::RangeList UCDCompiler::defaultIfHierachy = {
    // Non-ASCII
    {0x80, 0x10FFFF},
    // Two-byte sequences
    {0x80, 0x7FF},
    {0x100, 0x3FF},
    // 0100..017F; Latin Extended-A
    // 0180..024F; Latin Extended-B
    // 0250..02AF; IPA Extensions
    // 02B0..02FF; Spacing Modifier Letters
    {0x100, 0x2FF}, {0x100, 0x24F}, {0x100, 0x17F}, {0x180, 0x24F}, {0x250, 0x2AF}, {0x2B0, 0x2FF},
    // 0300..036F; Combining Diacritical Marks
    // 0370..03FF; Greek and Coptic
    {0x300, 0x36F}, {0x370, 0x3FF},
    // 0400..04FF; Cyrillic
    // 0500..052F; Cyrillic Supplement
    // 0530..058F; Armenian
    // 0590..05FF; Hebrew
    // 0600..06FF; Arabic
    {0x400, 0x5FF}, {0x400, 0x4FF}, {0x500, 0x058F}, {0x500, 0x52F}, {0x530, 0x58F}, {0x590, 0x5FF}, {0x600, 0x6FF},
    // 0700..074F; Syriac
    // 0750..077F; Arabic Supplement
    // 0780..07BF; Thaana
    // 07C0..07FF; NKo
    {0x700, 0x77F}, {0x700, 0x74F}, {0x750, 0x77F}, {0x780, 0x7FF}, {0x780, 0x7BF}, {0x7C0, 0x7FF},
    // Three-byte sequences
    {0x800, 0xFFFF}, {0x800, 0x4DFF}, {0x800, 0x1FFF}, {0x800, 0x0FFF},
    // 0800..083F; Samaritan
    // 0840..085F; Mandaic
    // 08A0..08FF; Arabic Extended-A
    // 0900..097F; Devanagari
    // 0980..09FF; Bengali
    // 0A00..0A7F; Gurmukhi
    // 0A80..0AFF; Gujarati
    // 0B00..0B7F; Oriya
    // 0B80..0BFF; Tamil
    // 0C00..0C7F; Telugu
    // 0C80..0CFF; Kannada
    // 0D00..0D7F; Malayalam
    // 0D80..0DFF; Sinhala
    // 0E00..0E7F; Thai
    // 0E80..0EFF; Lao
    // 0F00..0FFF; Tibetan
    {0x1000, 0x1FFF},
    // 1000..109F; Myanmar
    // 10A0..10FF; Georgian
    // 1100..11FF; Hangul Jamo
    // 1200..137F; Ethiopic
    // 1380..139F; Ethiopic Supplement
    // 13A0..13FF; Cherokee
    // 1400..167F; Unified Canadian Aboriginal Syllabics
    // 1680..169F; Ogham
    // 16A0..16FF; Runic
    // 1700..171F; Tagalog
    // 1720..173F; Hanunoo
    // 1740..175F; Buhid
    // 1760..177F; Tagbanwa
    // 1780..17FF; Khmer
    // 1800..18AF; Mongolian
    // 18B0..18FF; Unified Canadian Aboriginal Syllabics Extended
    // 1900..194F; Limbu
    // 1950..197F; Tai Le
    // 1980..19DF; New Tai Lue
    // 19E0..19FF; Khmer Symbols
    // 1A00..1A1F; Buginese
    // 1A20..1AAF; Tai Tham
    // 1AB0..1AFF; Combining Diacritical Marks Extended
    // 1B00..1B7F; Balinese
    // 1B80..1BBF; Sundanese
    // 1BC0..1BFF; Batak
    // 1C00..1C4F; Lepcha
    // 1C50..1C7F; Ol Chiki
    // 1CC0..1CCF; Sundanese Supplement
    // 1CD0..1CFF; Vedic Extensions
    // 1D00..1D7F; Phonetic Extensions
    // 1D80..1DBF; Phonetic Extensions Supplement
    // 1DC0..1DFF; Combining Diacritical Marks Supplement
    // 1E00..1EFF; Latin Extended Additional
    // 1F00..1FFF; Greek Extended
    {0x2000, 0x4DFF}, {0x2000, 0x2FFF},
    {0x3000, 0x4DFF},
    {0x4E00, 0x9FFF},
    // 4E00..9FFF; CJK Unified Ideographs
    {0xA000, 0xFFFF},

    {0x10000, 0x10FFFF}};

const UCDCompiler::RangeList UCDCompiler::noIfHierachy = {{0x80, 0x10FFFF}};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateRange
 ** ------------------------------------------------------------------------------------------------------------- */
void UCDCompiler::generateRange(const RangeList & ifRanges, PabloBuilder & entry) {
    // Pregenerate the suffix var outside of the if ranges. The DCE pass will either eliminate it if it's not used or the
    // code sinking pass will move appropriately into an inner if block.
    CC *  suffix = makeByte(0x80, 0xBF);
    assert (!suffix->empty());
    mSuffixVar = mCodeUnitCompiler.compileCC(suffix, entry);
    generateRange(ifRanges, 0, UNICODE_MAX, entry);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateRange
 * @param ifRangeList
 ** ------------------------------------------------------------------------------------------------------------- */
void UCDCompiler::generateRange(const RangeList & ifRanges, const codepoint_t lo, const codepoint_t hi, PabloBuilder & builder) {

    // Codepoints in unenclosed ranges will be computed unconditionally.
    // Generate them first so that computed subexpressions may be shared
    // with calculations within the if hierarchy.
    const auto enclosed = rangeIntersect(ifRanges, lo, hi);
    for (const auto rg : rangeGaps(enclosed, lo, hi)) {
        generateSubRanges(lo_codepoint(rg), hi_codepoint(rg), builder);
    }

    for (auto & f : mTargetValue) {
        const auto v = mTarget.find(f.first);
        assert (v != mTarget.end());
        PabloAST * value = f.second;
        if (!isa<Zeroes>(value)) {
            if (LLVM_LIKELY(builder.getParent() != nullptr)) {
                value = builder.createOr(v->second, value);
            }
            builder.createAssign(v->second, value);
            f.second = builder.createZeroes();
        }
    }

    const auto outer = outerRanges(enclosed);
    const auto inner = innerRanges(enclosed);

    Values nonIntersectingTargets;

    for (const auto range : outer) {

        // Split our current target list into two sets: the intersecting and non-intersecting ones. Any non-
        // intersecting set will be removed from the current map to eliminate the possibility of it being
        // considered until after we leave the current range. The intersecting sets are also stored to ensure
        // that we know what the original target value was going into this range block so tha we can OR the
        // inner value with the outer value.

        for (auto f = mTargetValue.begin(); f != mTargetValue.end(); ) {
            if (f->first->intersects(range.first, range.second)) {
                ++f;
            } else {
                nonIntersectingTargets.push_back(*f);
                f = mTargetValue.erase(f);
            }
        }
        if (mTargetValue.size() > 0) {
            auto inner_block = builder.createScope();
            builder.createIf(ifTestCompiler(range.first, range.second, builder), inner_block);
            generateRange(inner, range.first, range.second, inner_block);
        }
        for (auto t : nonIntersectingTargets) {
            mTargetValue.insert(t);
        }
        nonIntersectingTargets.clear();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateSubRanges
 ** ------------------------------------------------------------------------------------------------------------- */
void UCDCompiler::generateSubRanges(const codepoint_t lo, const codepoint_t hi, PabloBuilder & builder) {
    for (auto & t : mTargetValue) {
        const auto range = rangeIntersect(*t.first, lo, hi);
        PabloAST * target = t.second;
        // Divide by UTF-8 length, separating out E0, ED, F0 and F4 ranges
        const std::array<interval_t, 9> ranges =
            {{{0, 0x7F}, {0x80, 0x7FF}, {0x800, 0xFFF}, {0x1000, 0xD7FF}, {0xD800, 0xDFFF},
             {0xE000, 0xFFFF}, {0x10000, 0x3FFFF}, {0x40000, 0xFFFFF}, {0x100000, 0x10FFFF}}};
        for (auto r : ranges) {
            const auto subrange = rangeIntersect(range, lo_codepoint(r), hi_codepoint(r));
            target = sequenceGenerator(std::move(subrange), 1, builder, target, nullptr);
        }
        t.second = target;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief sequenceGenerator
 * @param ifRangeList
 *
 *
 * Generate remaining code to match UTF-8 code sequences within the codepoint set cpset, assuming that the code
 * matching the sequences up to byte number byte_no have been generated.
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * UCDCompiler::sequenceGenerator(const RangeList && ranges, const unsigned byte_no, PabloBuilder & builder, PabloAST * target, PabloAST * prefix) {
    bool isUTF_16 = false;

    if (LLVM_LIKELY(ranges.size() > 0)) {

        codepoint_t lo, hi;
        std::tie(lo, hi) = ranges[0];

        const auto min = length(lo_codepoint(ranges.front()), isUTF_16);
        const auto max = length(hi_codepoint(ranges.back()), isUTF_16);

        if (min != max) {
            const auto mid = maxCodePoint(min, isUTF_16);
            target = sequenceGenerator(std::move(rangeIntersect(ranges, lo, mid)), byte_no, builder, target, prefix);
            target = sequenceGenerator(std::move(rangeIntersect(ranges, mid + 1, hi)), byte_no, builder, target, prefix);
        } else if (min == byte_no) {
            // We have a single byte remaining to match for all code points in this CC.
            // Use the byte class compiler to generate matches for these codepoints.
            PabloAST * var = mCodeUnitCompiler.compileCC(makeCC(byteDefinitions(ranges, byte_no, isUTF_16), &Byte), builder);
            target = mCodeUnitCompiler.createUCDSequence(byte_no, target, var, makePrefix(lo, byte_no, builder, prefix), builder);
        } else {
            for (auto rg : ranges) {
                codepoint_t lo, hi;
                std::tie(lo, hi) = rg;
                const auto lo_byte = encodingByte(lo, byte_no, isUTF_16);
                const auto hi_byte = encodingByte(hi, byte_no, isUTF_16);
                if (lo_byte != hi_byte) {
                    unsigned num = isUTF_16 ? 10 : 6;
                    if (!isLowCodePointAfterByte(lo, byte_no, isUTF_16)) {
                        const codepoint_t mid = lo | ((1 << (num * (min - byte_no))) - 1);
                        target = sequenceGenerator(lo, mid, byte_no, builder, target, prefix);
                        target = sequenceGenerator(mid + 1, hi, byte_no, builder, target, prefix);
                    } else if (!isHighCodePointAfterByte(hi, byte_no, isUTF_16)) {
                        const codepoint_t mid = hi & ~((1 << (num * (min - byte_no))) - 1);
                        target = sequenceGenerator(lo, mid - 1, byte_no, builder, target, prefix);
                        target = sequenceGenerator(mid, hi, byte_no, builder, target, prefix);
                    } else { // we have a prefix group of type (a)
                        PabloAST * var = mCodeUnitCompiler.compileCC(makeByte(lo_byte, hi_byte), builder);
                        target = mCodeUnitCompiler.createUCDSequence(byte_no, length(lo, isUTF_16), target, var, prefix, mSuffixVar, builder);
                    }
                } else { // lbyte == hbyte
                    PabloAST * var = mCodeUnitCompiler.compileCC(makeByte(lo_byte, hi_byte), builder);
                    if (byte_no > 1) {
                        var = builder.createAnd(builder.createAdvance(prefix ? prefix : var, 1), var);
                    }
                    if (byte_no < length(lo, isUTF_16)) {
                        target = sequenceGenerator(lo, hi, byte_no + 1, builder, target, var);
                    }
                }
            }
        }
    }
    return target;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief sequenceGenerator
 ** ------------------------------------------------------------------------------------------------------------- */
inline PabloAST * UCDCompiler::sequenceGenerator(const codepoint_t lo, const codepoint_t hi, const unsigned byte_no, PabloBuilder & builder, PabloAST * target, PabloAST * prefix) {
    return sequenceGenerator({{ lo, hi }}, byte_no, builder, target, prefix);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief ifTestCompiler
 ** ------------------------------------------------------------------------------------------------------------- */
inline PabloAST * UCDCompiler::ifTestCompiler(const codepoint_t lo, const codepoint_t hi, PabloBuilder & builder) {
    return ifTestCompiler(lo, hi, 1, builder, builder.createOnes());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief ifTestCompiler
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * UCDCompiler::ifTestCompiler(const codepoint_t lo, const codepoint_t hi, const unsigned byte_no, PabloBuilder & builder, PabloAST * target) {

	bool isUTF_16 = false;
    codepoint_t lo_byte = encodingByte(lo, byte_no, isUTF_16);
    codepoint_t hi_byte = encodingByte(hi, byte_no, isUTF_16);
    const bool at_lo_boundary = (lo == 0 || encodingByte(lo - 1, byte_no, isUTF_16) != lo_byte);
    const bool at_hi_boundary = (hi == 0x10FFFF || encodingByte(hi + 1, byte_no, isUTF_16) != hi_byte);

    if (at_lo_boundary && at_hi_boundary) {
        if (!isUTF_16) {
            if (lo_byte != hi_byte) {
            if (lo == 0x80) lo_byte = 0xC0;
            if (hi == 0x10FFFF) hi_byte = 0xFF;
            }
        }
        PabloAST * cc = mCodeUnitCompiler.compileCC(makeByte(lo_byte, hi_byte), builder);
        target = builder.createAnd(cc, target);
    } else if (lo_byte == hi_byte) {
        PabloAST * cc = mCodeUnitCompiler.compileCC(makeByte(lo_byte, hi_byte), builder);
        target = builder.createAnd(cc, target);
        target = builder.createAdvance(target, 1);
        target = ifTestCompiler(lo, hi, byte_no + 1, builder, target);
    } else if (!at_hi_boundary) {
        const auto mid = minCodePointWithCommonBytes(hi, byte_no, isUTF_16);
        PabloAST * e1 = ifTestCompiler(lo, mid - 1, byte_no, builder, target);
        PabloAST * e2 = ifTestCompiler(mid, hi, byte_no, builder, target);
        target = builder.createOr(e1, e2);
    } else {
        const auto mid = maxCodePointWithCommonBytes(lo, byte_no, isUTF_16);
        PabloAST * e1 = ifTestCompiler(lo, mid, byte_no, builder, target);
        PabloAST * e2 = ifTestCompiler(mid + 1, hi, byte_no, builder, target);
        target = builder.createOr(e1, e2);
    }
    return target;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief definePrecedingPrefix
 * @param ifRangeList
 *
 *
 * Ensure the sequence of preceding bytes is defined, up to, but not including the given byte_no
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * UCDCompiler::makePrefix(const codepoint_t cp, const unsigned byte_no, PabloBuilder & builder, PabloAST * prefix) {
    assert (byte_no >= 1 && byte_no <= 4);
    assert (byte_no == 1 || prefix != nullptr);
    bool isUTF_16 = false;
    for (unsigned i = 1; i != byte_no; ++i) {
        const CC * const cc = makeByte(encodingByte(cp, i, isUTF_16));
        PabloAST * var = mCodeUnitCompiler.compileCC(cc, builder);
        if (i > 1) {
            var = builder.createAnd(var, builder.createAdvance(prefix, 1));
        }
        prefix = var;
    }
    return prefix;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief definePrecedingPrefix
 * @param ifRangeList
 *
 *
 * Ensure the sequence of preceding bytes is defined, up to, but not including the given byte_no
 ** ------------------------------------------------------------------------------------------------------------- */
UCDCompiler::RangeList UCDCompiler::byteDefinitions(const RangeList & list, const unsigned byte_no, bool isUTF_16) {
    RangeList result;
    result.reserve(list.size());
    for (const auto & i : list) {
        result.emplace_back(encodingByte(lo_codepoint(i), byte_no, isUTF_16), encodingByte(hi_codepoint(i), byte_no, isUTF_16));
    }
    return result;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief rangeIntersect
 * @param list
 * @param lo
 * @param hi
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename RangeListOrUnicodeSet>
UCDCompiler::RangeList UCDCompiler::rangeIntersect(const RangeListOrUnicodeSet & list, const codepoint_t lo, const codepoint_t hi) {
    RangeList result;
    for (const auto i : list) {
        if ((lo_codepoint(i) <= hi) && (hi_codepoint(i) >= lo)) {
            result.emplace_back(std::max(lo, lo_codepoint(i)), std::min(hi, hi_codepoint(i)));
        }
    }
    return result;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief rangeGaps
 * @param cc
 * @param lo
 * @param hi
 ** ------------------------------------------------------------------------------------------------------------- */
UCDCompiler::RangeList UCDCompiler::rangeGaps(const RangeList & list, const codepoint_t lo, const codepoint_t hi) {
    RangeList gaps;
    if (LLVM_LIKELY(lo < hi)) {
        if (LLVM_UNLIKELY(list.empty())) {
            gaps.emplace_back(lo, hi);
        } else {
            codepoint_t cp = lo;
            for (const auto & i : list) {
                if (hi_codepoint(i) < cp) {
                    continue;
                } else if (lo_codepoint(i) > cp) {
                    gaps.emplace_back(cp, lo_codepoint(i) - 1);
                } else if (hi_codepoint(i) >= hi) {
                    continue;
                }
                cp = hi_codepoint(i) + 1;
            }
        }
    }
    return gaps;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief outerRanges
 * @param list
 ** ------------------------------------------------------------------------------------------------------------- */
UCDCompiler::RangeList UCDCompiler::outerRanges(const RangeList & list) {
    RangeList ranges;
    if (LLVM_LIKELY(list.size() > 0)) {
        auto i = list.cbegin();
        for (auto j = i + 1; j != list.cend(); ++j) {
            if (hi_codepoint(*j) > hi_codepoint(*i)) {
                ranges.emplace_back(lo_codepoint(*i), hi_codepoint(*i));
                i = j;
            }
        }
        if (LLVM_LIKELY(i != list.end())) {
            ranges.emplace_back(lo_codepoint(*i), hi_codepoint(*i));
        }
    }
    return ranges;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief innerRanges
 ** ------------------------------------------------------------------------------------------------------------- */
UCDCompiler::RangeList UCDCompiler::innerRanges(const RangeList & list) {
    RangeList ranges;
    if (LLVM_LIKELY(list.size() > 0)) {
        for (auto i = list.cbegin(), j = i + 1; j != list.cend(); ++j) {
            if (hi_codepoint(*j) <= hi_codepoint(*i)) {
                ranges.emplace_back(lo_codepoint(*j), hi_codepoint(*j));
            } else {
                i = j;
            }
        }
    }
    return ranges;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateWithDefaultIfHierarchy
 ** ------------------------------------------------------------------------------------------------------------- */
void UCDCompiler::generateWithDefaultIfHierarchy(NameMap & names, PabloBuilder & entry) {
    makeTargets(entry, names);
    generateRange(defaultIfHierachy, entry);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateWithoutIfHierarchy
 ** ------------------------------------------------------------------------------------------------------------- */
void UCDCompiler::generateWithoutIfHierarchy(NameMap & names, PabloBuilder & entry) {
    makeTargets(entry, names);
    generateRange(noIfHierachy, entry);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addTargets
 ** ------------------------------------------------------------------------------------------------------------- */
inline void UCDCompiler::makeTargets(PabloBuilder & entry, NameMap & names) {

    mTargetValue.clear();
    mTarget.clear();

    struct Comparator {
        inline bool operator() (const CC * lh, const CC * rh) const{
            return *lh < *rh;
        }
    } ec;

    flat_map<CC *, Var *, Comparator> CCs(ec);

    Zeroes * const zeroes = entry.createZeroes();

    for (auto & t : names) {
        Name * const name = t.first;
        CC * const cc = dyn_cast<CC>(name->getDefinition());
        if (cc && (cc->getAlphabet() == &cc::Unicode)) {
            const auto f = CCs.find(cc);
            // This check may not be needed. Memoization ought to detect duplicate classes earlier.
            if (LLVM_LIKELY(f == CCs.end())) {
                PabloAST * const value = t.second ? t.second : zeroes;
                mTargetValue.emplace(cc, value);
                Var * const var = entry.createVar(name->getName(), value);
                mTarget.emplace(cc, var);
                CCs.emplace(cc, var);
                t.second = var;
            } else {
                t.second = f->second;
            }
        } else {
            report_fatal_error(name->getName() + " is not defined by a Unicode CC!");
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UCDCompiler::UCDCompiler(cc::CC_Compiler & ccCompiler)
: mCodeUnitCompiler(ccCompiler)
, mSuffixVar(nullptr) { }

}
