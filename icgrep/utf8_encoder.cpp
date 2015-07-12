/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <utf8_encoder.h>
#include <re/re_name.h>
#include <re/re_seq.h>
#include <re/re_alt.h>
#include <re/re_rep.h>
#include <cc/cc_namemap.hpp>
#include <assert.h>
#include <algorithm>
#include <stdexcept>

using namespace re;

namespace cc {

RE * UTF8_Encoder::toUTF8(CC_NameMap & nameMap, RE * ast) {
    for (Name * name : nameMap) {
        if (const CC * cc = dyn_cast_or_null<CC>(name->getDefinition())) {
            if (cc->size() == 1) {
                RE * def = rangeToUTF8(cc->front());
                name->setDefinition(def);
            }
            else if (cc->size() > 1) {
                std::vector<RE *> alt;
                for (const interval_t & i : *cc) {
                    alt.push_back(rangeToUTF8(i));
                }
                RE * def = makeAlt(alt.begin(), alt.end());
                name->setDefinition(def);
            }
        }
    }
    nameMap.clear();
    return nameMap.process(ast, ByteClass);
}

RE * UTF8_Encoder::rangeToUTF8(const interval_t & item) {
    const auto min = length(lo_codepoint(item));
    const auto max = length(hi_codepoint(item));
    if (min < max) {
        const auto m = maxCodePoint(min);
        return makeAlt({rangeToUTF8(std::make_pair(lo_codepoint(item), m)), rangeToUTF8(std::make_pair(m + 1, hi_codepoint(item)))});
    }
    else {
        return rangeToUTF8(lo_codepoint(item), hi_codepoint(item), 1, max);
    }
}

RE * UTF8_Encoder::rangeToUTF8(const codepoint_t lo, const codepoint_t hi, const unsigned index, const unsigned max)
{
    const codepoint_t hbyte = encodingByte(hi, index);
    const codepoint_t lbyte = encodingByte(lo, index);
    if (index == max) {
        return makeByteRange(lbyte, hbyte);
    }
    else if (hbyte == lbyte) {
        return makeSeq({makeByteClass(hbyte), rangeToUTF8(lo, hi, index + 1, max)});
    }
    else {
        const unsigned suffix_mask = (static_cast<unsigned>(1) << ((max - index) * 6)) - 1;
        if ((hi & suffix_mask) != suffix_mask) {
            const unsigned hi_floor = (~suffix_mask) & hi;
            return makeAlt({rangeToUTF8(hi_floor, hi, index, max), rangeToUTF8(lo, hi_floor - 1, index, max)});
        }
        else if ((lo & suffix_mask) != 0) {
            const unsigned low_ceil = lo | suffix_mask;
            return makeAlt({rangeToUTF8(low_ceil + 1, hi, index, max), rangeToUTF8(lo, low_ceil, index, max)});
        }
        else {
            return makeSeq({makeByteRange(lbyte, hbyte), rangeToUTF8(lo, hi, index + 1, max)});
        }
    }
}

bool UTF8_Encoder::isPrefix(const codepoint_t cp) {
    return (cp >= 0xC2) && (cp <= 0xF4);
}

codepoint_t UTF8_Encoder::encodingByte(const codepoint_t cp, const unsigned n) {
    codepoint_t retVal = 0;
    const unsigned len = length(cp);
    if (n == 1) {
        switch (len) {
            case 1: retVal = cp; break;
            case 2: retVal = 0xC0 | (cp >> 6); break;
            case 3: retVal = 0xE0 | (cp >> 12); break;
            case 4: retVal = 0xF0 | (cp >> 18); break;
        }
    }
    else {
        retVal = 0x80 | ((cp >> (6 * (len - n))) & 0x3F);
    }
    return retVal;
}

unsigned UTF8_Encoder::length(const codepoint_t cp) {
    if (cp <= 0x7F) {
        return 1;
    }
    else if (cp <= 0x7FF) {
        return 2;
    }
    else if (cp <= 0xFFFF) {
        return 3;
    }
    else {
        return 4;
    }
}

codepoint_t UTF8_Encoder::maxCodePoint(const unsigned length) {
    if (length == 1) {
        return 0x7F;
    }
    else if (length == 2) {
        return 0x7FF;
    }
    else if (length == 3) {
        return 0xFFFF;
    }
    else if (length == 4) {
        return 0x10FFFF;
    }
    throw std::runtime_error("Unexpected UTF8 Length: " + std::to_string(length));
}

bool UTF8_Encoder::isLowCodePointAfterByte(const codepoint_t cp, const unsigned n) {
    const auto l = length(cp);
    for (auto i = n; i != l; ++i) {
        if (encodingByte(cp, i + 1) != 0x80) {
            return false;
        }
    }
    return true;
}

bool UTF8_Encoder::isHighCodePointAfterByte(const codepoint_t cp, const unsigned n) {
    const auto l = length(cp);
    for (auto i = n; i != l; ++i) {
        if (encodingByte(cp, i + 1) != 0xBF) {
            return false;
        }
    }
    return true;
}

codepoint_t UTF8_Encoder::minCodePointWithCommonBytes(const re::codepoint_t cp, const unsigned n) {
    const auto len = length(cp);
    const auto mask = (static_cast<codepoint_t>(1) << (len - n) * 6) - 1;
    const auto lo_cp = cp &~ mask;
    return (lo_cp == 0) ? mask + 1 : lo_cp;
}

codepoint_t UTF8_Encoder::maxCodePointWithCommonBytes(const re::codepoint_t cp, const unsigned n) {
    const auto len = length(cp);
    const auto mask = (static_cast<codepoint_t>(1) << (len - n) * 6) - 1;
    return cp | mask;
}

inline CC * UTF8_Encoder::makeByteRange(const codepoint_t lo, const codepoint_t hi) {
    return makeCC(lo, hi);
}

inline CC * UTF8_Encoder::makeByteClass(const codepoint_t cp) {
    return makeCC(cp, cp);
}

}
