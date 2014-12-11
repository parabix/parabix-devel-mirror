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
        if (const CC * cc = dyn_cast_or_null<CC>(name->getCC())) {
            if (cc->size() == 1) {
                name->setCC(rangeToUTF8(cc->front()));
            }
            else if (cc->size() > 1) {
                std::vector<RE *> alt;
                for (const CharSetItem & item : *cc) {
                    alt.push_back(rangeToUTF8(item));
                }
                name->setCC(makeAlt(alt.begin(), alt.end()));
            }
        }
    }
    ast = nameMap.process(ast);
    return ast;
}

RE * UTF8_Encoder::rangeToUTF8(const CharSetItem & item) {
    const auto min = lenUTF8(item.lo_codepoint);
    const auto max = lenUTF8(item.hi_codepoint);
    if (min < max) {
        const auto m = maxCodePoint(min);
        return makeAlt({rangeToUTF8(CharSetItem(item.lo_codepoint, m)), rangeToUTF8(CharSetItem(m + 1, item.hi_codepoint))});
    }
    else {
        return rangeToUTF8(item.lo_codepoint, item.hi_codepoint, 1, max);
    }
}

RE * UTF8_Encoder::rangeToUTF8(const CodePointType lo, const CodePointType hi, const unsigned index, const unsigned max)
{
    const CodePointType hbyte = u8byte(hi, index);
    const CodePointType lbyte = u8byte(lo, index);
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

inline bool UTF8_Encoder::isUTF8Prefix(const unsigned cp) {
    return (cp >= 0xC2) && (cp <= 0xF4);
}

inline CodePointType UTF8_Encoder::u8byte(const CodePointType codepoint, const unsigned n)
{
    CodePointType retVal = 0;

    const unsigned len = lenUTF8(codepoint);

    if (n == 1) {
        switch (len) {
            case 1: retVal = codepoint; break;
            case 2: retVal = 0xC0 | (codepoint >> 6); break;
            case 3: retVal = 0xE0 | (codepoint >> 12); break;
            case 4: retVal = 0xF0 | (codepoint >> 18); break;
        }
    }
    else {
        retVal = 0x80 | ((codepoint >> (6 * (len - n))) & 0x3F);
    }

    return retVal;
}

inline unsigned UTF8_Encoder::lenUTF8(const unsigned cp) {
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

inline unsigned UTF8_Encoder::maxCodePoint(const unsigned length) {
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

inline CC * UTF8_Encoder::makeByteRange(const CodePointType lo, const CodePointType hi) {
    return makeCC(lo, hi);
}

inline CC * UTF8_Encoder::makeByteClass(const CodePointType cp) {
    return makeCC(cp, cp);
}

}
