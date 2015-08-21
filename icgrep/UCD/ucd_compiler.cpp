#include "ucd_compiler.hpp"
#include <cc/cc_compiler.h>
#include <UCD/unicode_set.h>
#include <utf8_encoder.h>

using namespace cc;
using namespace re;
using namespace pablo;

namespace UCD {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateWithDefaultIfHierarchy
 * @param set the unicode set to generate
 * @param the entry block to the function we're filling
 * @return the output stream with a 1-bit in any position of a character in the unicode set
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * UCDCompiler::generateWithIfHierarchy(const RangeList & ifRanges, const UnicodeSet & set, PabloBuilder & entry) {
    // Pregenerate the suffix var outside of the if ranges. The DCE pass will either eliminate it if it's not used or the
    // code sinking pass will move appropriately into an inner if block.
    mSuffixVar = mCharacterClassCompiler.compileCC(makeCC(0x80, 0xBF), entry);
    return generateWithIfHierarchy(ifRanges, set, 0, CC::UNICODE_MAX, entry);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateCharClassDefsInIfHierarchy
 * @param ifRangeList
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * UCDCompiler::generateWithIfHierarchy(const RangeList & ifRanges, const UnicodeSet & set, const codepoint_t lo, const codepoint_t hi, PabloBuilder & builder) {

    PabloAST * target = builder.createZeroes();
    // Codepoints in unenclosed ranges will be computed unconditionally.
    // Generate them first so that computed subexpressions may be shared
    // with calculations within the if hierarchy.

    const auto enclosed = rangeIntersect(ifRanges, lo, hi);

    for (const auto rg : rangeGaps(enclosed, lo, hi)) {
        target = generateSubRanges(set, lo_codepoint(rg), hi_codepoint(rg), builder, target);
    }

    const auto outer = outerRanges(enclosed);
    const auto inner = innerRanges(enclosed);
    for (const auto range : outer) {
        codepoint_t lo, hi;
        std::tie(lo, hi) = range;
        if (set.intersects(lo, hi)) {
            PabloBuilder inner_block = PabloBuilder::Create(builder);
            PabloAST * inner_target = generateWithIfHierarchy(inner, set, lo, hi, inner_block);
            // If this range is empty, just skip creating the if block
            if (LLVM_UNLIKELY(isa<Zeroes>(inner_target))) {
                continue;
            }
            Assign * matches = inner_block.createAssign("m", inner_target);
            builder.createIf(ifTestCompiler(lo, hi, builder), {matches}, inner_block);
            target = builder.createOr(target, matches);
        }
    }

    return target;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateCharClassSubDefs
 * @param ifRangeList
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * UCDCompiler::generateSubRanges(const UnicodeSet & set, const codepoint_t lo, const codepoint_t hi, PabloBuilder & builder, PabloAST * target) {
    const auto range = rangeIntersect(set, lo, hi);
    // Divide by UTF-8 length, separating out E0, ED, F0 and F4 ranges
    const std::array<interval_t, 9> ranges =
        {{{0, 0x7F}, {0x80, 0x7FF}, {0x800, 0xFFF}, {0x1000, 0xD7FF}, {0xD800, 0xDFFF},
         {0xE000, 0xFFFF}, {0x10000, 0x3FFFF}, {0x40000, 0xFFFFF}, {0x100000, 0x10FFFF}}};
    for (auto r : ranges) {
        const auto subrange = rangeIntersect(range, lo_codepoint(r), hi_codepoint(r));
        target = sequenceGenerator(std::move(subrange), 1, builder, target, nullptr);
    }
    return target;
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

    if (LLVM_LIKELY(!ranges.empty())) {

        codepoint_t lo, hi;
        std::tie(lo, hi) = ranges[0];

        const auto min = UTF8_Encoder::length(lo_codepoint(ranges.front()));
        const auto max = UTF8_Encoder::length(hi_codepoint(ranges.back()));

        if (min != max) {
            const auto mid = UTF8_Encoder::maxCodePoint(min);
            target = sequenceGenerator(std::move(rangeIntersect(ranges, lo, mid)), byte_no, builder, target, prefix);
            target = sequenceGenerator(std::move(rangeIntersect(ranges, mid + 1, hi)), byte_no, builder, target, prefix);
        }
        else if (min == byte_no) {
            // We have a single byte remaining to match for all code points in this CC.
            // Use the byte class compiler to generate matches for these codepoints.
            const auto bytes = byteDefinitions(ranges, byte_no);
            PabloAST * var = mCharacterClassCompiler.compileCC(makeCC(bytes), builder);
            if (byte_no > 1) {
                var = builder.createAnd(var, builder.createAdvance(makePrefix(lo, byte_no, builder, prefix), 1));
            }
            target = builder.createOr(target, var);
        }
        else {
            for (auto rg : ranges) {
                codepoint_t lo, hi;
                std::tie(lo, hi) = rg;
                const auto lo_byte = UTF8_Encoder::encodingByte(lo, byte_no);
                const auto hi_byte = UTF8_Encoder::encodingByte(hi, byte_no);
                if (lo_byte != hi_byte) {
                    if (!UTF8_Encoder::isLowCodePointAfterByte(lo, byte_no)) {
                        const codepoint_t mid = lo | ((1 << (6 * (min - byte_no))) - 1);
                        target = sequenceGenerator(lo, mid, byte_no, builder, target, prefix);
                        target = sequenceGenerator(mid + 1, hi, byte_no, builder, target, prefix);
                    }
                    else if (!UTF8_Encoder::isHighCodePointAfterByte(hi, byte_no)) {
                        const codepoint_t mid = hi & ~((1 << (6 * (min - byte_no))) - 1);
                        target = sequenceGenerator(lo, mid - 1, byte_no, builder, target, prefix);
                        target = sequenceGenerator(mid, hi, byte_no, builder, target, prefix);
                    }
                    else { // we have a prefix group of type (a)
                        PabloAST * var = mCharacterClassCompiler.compileCC(makeCC(lo_byte, hi_byte), builder);
                        if (byte_no > 1) {
                            var = builder.createAnd(builder.createAdvance(prefix, 1), var);
                        }
                        for (unsigned i = byte_no; i != UTF8_Encoder::length(lo); ++i) {
                            var = builder.createAnd(mSuffixVar, builder.createAdvance(var, 1));
                        }
                        target = builder.createOr(target, var);
                    }
                }
                else { // lbyte == hbyte
                    PabloAST * var = mCharacterClassCompiler.compileCC(makeCC(lo_byte, hi_byte), builder);
                    if (byte_no > 1) {
                        var = builder.createAnd(builder.createAdvance(prefix ? prefix : var, 1), var);
                    }
                    if (byte_no < UTF8_Encoder::length(lo)) {
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

    codepoint_t lo_byte = UTF8_Encoder::encodingByte(lo, byte_no);
    codepoint_t hi_byte = UTF8_Encoder::encodingByte(hi, byte_no);
    const bool at_lo_boundary = (lo == 0 || UTF8_Encoder::encodingByte(lo - 1, byte_no) != lo_byte);
    const bool at_hi_boundary = (hi == 0x10FFFF || UTF8_Encoder::encodingByte(hi + 1, byte_no) != hi_byte);

    if (at_lo_boundary && at_hi_boundary) {
        if (lo_byte != hi_byte) {
            if (lo == 0x80) lo_byte = 0xC0;
            if (hi == 0x10FFFF) hi_byte = 0xFF;
        }
        PabloAST * cc = mCharacterClassCompiler.compileCC(makeCC(lo_byte, hi_byte), builder);
        target = builder.createAnd(cc, target);
    }
    else if (lo_byte == hi_byte) {
        PabloAST * cc = mCharacterClassCompiler.compileCC(makeCC(lo_byte, hi_byte), builder);
        target = builder.createAnd(cc, target);
        target = builder.createAdvance(target, 1);
        target = ifTestCompiler(lo, hi, byte_no + 1, builder, target);
    }
    else if (!at_hi_boundary) {
        const auto mid = UTF8_Encoder::minCodePointWithCommonBytes(hi, byte_no);
        PabloAST * e1 = ifTestCompiler(lo, mid - 1, byte_no, builder, target);
        PabloAST * e2 = ifTestCompiler(mid, hi, byte_no, builder, target);
        target = builder.createOr(e1, e2);
    }
    else {
        const auto mid = UTF8_Encoder::maxCodePointWithCommonBytes(lo, byte_no);
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
    for (unsigned i = 1; i != byte_no; ++i) {
        const CC * const cc = makeCC(UTF8_Encoder::encodingByte(cp, i));
        PabloAST * var = mCharacterClassCompiler.compileCC(cc, builder);
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
UCDCompiler::RangeList UCDCompiler::byteDefinitions(const RangeList & list, const unsigned byte_no) {
    RangeList result;
    result.reserve(list.size());
    for (const auto & i : list) {
        result.emplace_back(UTF8_Encoder::encodingByte(lo_codepoint(i), byte_no), UTF8_Encoder::encodingByte(hi_codepoint(i), byte_no));
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
        }
        else {
            codepoint_t cp = lo;
            for (const auto & i : list) {
                if (hi_codepoint(i) < cp) {
                    continue;
                }
                else if (lo_codepoint(i) > cp) {
                    gaps.emplace_back(cp, lo_codepoint(i) - 1);
                }
                else if (hi_codepoint(i) >= hi) {
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
 * @param list
 ** ------------------------------------------------------------------------------------------------------------- */
UCDCompiler::RangeList UCDCompiler::innerRanges(const RangeList & list) {
    RangeList ranges;
    if (LLVM_LIKELY(list.size() > 0)) {
        for (auto i = list.cbegin(), j = i + 1; j != list.cend(); ++j) {
            if (hi_codepoint(*j) <= hi_codepoint(*i)) {
                ranges.emplace_back(lo_codepoint(*j), hi_codepoint(*j));
            }
            else {
                i = j;
            }
        }
    }
    return ranges;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateWithDefaultIfHierarchy
 * @param set the unicode set to generate
 * @param the entry block to the function we're filling
 * @return the output stream with a 1-bit in any position of a character in the unicode set
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * UCDCompiler::generateWithDefaultIfHierarchy(const UnicodeSet & set, PabloBuilder & entry) {

    const RangeList defaultIfHierachy = {
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

    return generateWithIfHierarchy(defaultIfHierachy, set, entry);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateWithoutIfHierarchy
 * @param set the unicode set to generate
 * @param the entry block to the function we're filling
 * @return the output stream with a 1-bit in any position of a character in the unicode set
 ** ------------------------------------------------------------------------------------------------------------- */
PabloAST * UCDCompiler::generateWithoutIfHierarchy(const UnicodeSet & set, PabloBuilder & entry) {

    const RangeList defaultIfHierachy = {{0x10000, 0x10FFFF}};

    return generateWithIfHierarchy(defaultIfHierachy, set, entry);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
UCDCompiler::UCDCompiler(cc::CC_Compiler & ccCompiler)
: mCharacterClassCompiler(ccCompiler)
, mSuffixVar(nullptr) { }

}
