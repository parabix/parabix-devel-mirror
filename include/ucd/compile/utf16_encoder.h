/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef UTF16_ENCODER_H
#define UTF16_ENCODER_H

#include <ucd/core/unicode_set.h>

namespace cc {

struct UTF16_Encoder {
    static bool isHi_Surrogate(const UCD::codepoint_t cp);
    static bool isLo_Surrogate(const UCD::codepoint_t cp);
    static unsigned length(const UCD::codepoint_t cp);
    static UCD::codepoint_t maxCodePoint(const unsigned length);
    static UCD::codepoint_t encodingByte(const UCD::codepoint_t cp, const unsigned n);
    static bool isLowCodePointAfterByte(const UCD::codepoint_t cp, const unsigned n);
    static bool isHighCodePointAfterByte(const UCD::codepoint_t cp, const unsigned n);
    static UCD::codepoint_t minCodePointWithCommonBytes(const UCD::codepoint_t cp, const unsigned n);
    static UCD::codepoint_t maxCodePointWithCommonBytes(const UCD::codepoint_t cp, const unsigned n);
};

}

#endif // UTF16_ENCODER_H
