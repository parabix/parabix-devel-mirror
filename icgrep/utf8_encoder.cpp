/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <utf8_encoder.h>
#include <assert.h>
#include <algorithm>
#include <stdexcept>

using namespace re;

namespace cc {

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

}
