/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <utf16_encoder.h>
#include <assert.h>
#include <algorithm>
#include <stdexcept>

using namespace re;

namespace cc {

bool UTF16_Encoder::isHi_Surrogate(const codepoint_t cp) {
    return (cp >= 0xD800) && (cp <= 0xDBFF);
}

bool UTF16_Encoder::isLo_Surrogate(const codepoint_t cp) {
    return (cp >= 0xDC00) && (cp <= 0xDFFF);
}

codepoint_t UTF16_Encoder::encodingByte(const codepoint_t cp, const unsigned n) {
    codepoint_t retVal = 0;
    const unsigned len = length(cp);
    if (len == 1) {
	retVal = cp;
    }
    else {
	codepoint_t code = cp - 0x010000;
	if (n == 1) {
		retVal = (code >> 10) | 0xD800;
	}
	if (n == 2) {
		retVal = (code & 0x3FF) | 0xDC00;
	}
    }
    return retVal;
}

unsigned UTF16_Encoder::length(const codepoint_t cp) {
    if (cp <= 0xFFFF) {
	return 1;
    }
    else {
	return 2;
    }
}

codepoint_t UTF16_Encoder::maxCodePoint(const unsigned length) {
    if (length == 1) {
	return 0xFFFF;
    }
    else if (length == 2) {
	return 0x10FFFF;
    } 
    throw std::runtime_error("Unexpected UTF16 Length: " + std::to_string(length));
}

bool UTF16_Encoder::isLowCodePointAfterByte(const codepoint_t cp, const unsigned n) {
    const auto l = length(cp);
    for (auto i = n; i != l; ++i) {
        if (encodingByte(cp, i + 1) != 0xDC00) {
            return false;
        }
    }
    return true;
}

bool UTF16_Encoder::isHighCodePointAfterByte(const codepoint_t cp, const unsigned n) {
    const auto l = length(cp);
    for (auto i = n; i != l; ++i) {
        if (encodingByte(cp, i + 1) != 0xDFFF) {
            return false;
        }
    }
    return true;
}

codepoint_t UTF16_Encoder::minCodePointWithCommonBytes(const re::codepoint_t cp, const unsigned n) {
    const auto len = length(cp);
    const auto mask = (static_cast<codepoint_t>(1) << (len - n) * 10) - 1;
    const auto lo_cp = cp &~ mask;
    return (lo_cp == 0) ? mask + 1 : lo_cp;
}

codepoint_t UTF16_Encoder::maxCodePointWithCommonBytes(const re::codepoint_t cp, const unsigned n) {
    const auto len = length(cp);
    const auto mask = (static_cast<codepoint_t>(1) << (len - n) * 10) - 1;
    return cp | mask;
}

}
