/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/transforms/to_utf8.h>
#include <re/transforms/re_transformer.h>
#include <re/adt/adt.h>
#include <unicode/core/unicode_set.h>
#include <unicode/utf/UTF.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace re {

static RE * rangeCodeUnits(codepoint_t lo, codepoint_t hi, unsigned index, const unsigned lgth){
    const codepoint_t hunit = UTF<8>::nthCodeUnit(hi, index);
    const codepoint_t lunit = UTF<8>::nthCodeUnit(lo, index);
    if (index == lgth) {
        return makeCC(lunit, hunit, &cc::UTF8);
    }
    else if (hunit == lunit) {
        return makeSeq({makeCC(hunit, &cc::UTF8), rangeCodeUnits(lo, hi, index + 1, lgth)});
    }
    else {
        const unsigned suffix_mask = (static_cast<unsigned>(1) << ((lgth - index) * 6)) - 1;
        if ((hi & suffix_mask) != suffix_mask) {
            const unsigned hi_floor = (~suffix_mask) & hi;
            return makeAlt({rangeCodeUnits(hi_floor, hi, index, lgth), rangeCodeUnits(lo, hi_floor - 1, index, lgth)});
        }
        else if ((lo & suffix_mask) != 0) {
            const unsigned low_ceil = lo | suffix_mask;
            return makeAlt({rangeCodeUnits(low_ceil + 1, hi, index, lgth), rangeCodeUnits(lo, low_ceil, index, lgth)});
        }
        else {
            return makeSeq({makeCC(lunit, hunit, &cc::UTF8), rangeCodeUnits(lo, hi, index + 1, lgth)});
        }
    }
}

static RE * rangeToUTF8(codepoint_t lo, codepoint_t hi) {
    const auto min_lgth = UTF<8>::encoded_length(lo);
    const auto max_lgth = UTF<8>::encoded_length(hi);
    if (min_lgth < max_lgth) {
        const auto m = UTF<8>::max_codepoint_of_length(min_lgth);
        return makeAlt({rangeToUTF8(lo, m), rangeToUTF8(m + 1, hi)});
    }
    else {
        return rangeCodeUnits(lo, hi, 1, max_lgth);
    }
}

RE * UTF8_Transformer::transformCC(CC * cc) {
    if (cc->getAlphabet() != &cc::Unicode) return cc;
    std::vector<RE *> alt;
    for (const interval_t & i : *cc) {
        alt.push_back(rangeToUTF8(lo_codepoint(i), hi_codepoint(i)));
    }
    return makeAlt(alt.begin(), alt.end());
}

UTF8_Transformer::UTF8_Transformer(NameTransformationMode m) :
    EncodingTransformer("ToUTF8", &cc::Unicode, &cc::UTF8, m) {}

RE * toUTF8(RE * r, bool convertName) {
    const auto mode = convertName ? NameTransformationMode::TransformDefinition : NameTransformationMode::None;
    return UTF8_Transformer(mode).transformRE(r);
}

static RE *rangeCodeUnitsU16(codepoint_t lo, codepoint_t hi, unsigned index, const unsigned lgth) {
    const codepoint_t hunit = UTF<16>::nthCodeUnit(hi, index);  
    const codepoint_t lunit = UTF<16>::nthCodeUnit(lo, index);  
    if (index == lgth) {
        if(hunit<lunit )
            return makeCC(hunit, lunit, &cc::UTF16); //surrogate pair code points treated separately 
        return makeCC(lunit, hunit, &cc::UTF16);
    } else if (hunit == lunit) {
        return makeSeq({makeCC(hunit, &cc::UTF16), rangeCodeUnitsU16(lo, hi, index + 1, lgth)});
    } else {
        const unsigned suffix_mask = UTF<16>::suffixDataBits(index, lgth);
        if ((hi & suffix_mask) != suffix_mask) {
            const unsigned hi_floor = (~suffix_mask) & hi;
            return makeAlt({rangeCodeUnitsU16(hi_floor, hi, index, lgth), rangeCodeUnitsU16(lo, hi_floor - 1, index, lgth)});
        } else if ((lo & suffix_mask) != 0) {
            const unsigned low_ceil = lo | suffix_mask;
            return makeAlt({rangeCodeUnitsU16(low_ceil + 1, hi, index, lgth), rangeCodeUnitsU16(lo, low_ceil, index, lgth)});
        } else {
            return makeSeq({makeCC(lunit, hunit, &cc::UTF16), rangeCodeUnitsU16(lo, hi, index + 1, lgth)});
        }
    }
}

static RE *rangeToUTF16(codepoint_t lo, codepoint_t hi) {
    const auto min_lgth16 = UTF<16>::encoded_length(lo);
    const auto max_lgth16 = UTF<16>::encoded_length(hi);
    if (min_lgth16 < max_lgth16) {
        const auto m = UTF<16>::max_codepoint_of_length(min_lgth16);
        return makeAlt({rangeToUTF16(lo, m), rangeToUTF16(m + 1, hi)});
    } else {
        return rangeCodeUnitsU16(lo, hi, 1, max_lgth16); 
    }
}

RE *UTF16_Transformer::transformCC(CC *cc) {
    if (cc->getAlphabet() != &cc::Unicode) return cc;
    std::vector<RE *> alt;
    for (const interval_t &i : *cc) {
        alt.push_back(rangeToUTF16(lo_codepoint(i), hi_codepoint(i)));
    }
    return makeAlt(alt.begin(), alt.end());
}

UTF16_Transformer::UTF16_Transformer(NameTransformationMode m)
    : EncodingTransformer("ToUTF16", &cc::Unicode, &cc::UTF16, m) {}

RE * toUTF16(RE *r, bool convertName) {
    const auto mode = convertName ? NameTransformationMode::TransformDefinition
                                  : NameTransformationMode::None;
    return UTF16_Transformer(mode).transformRE(r);
}
}
