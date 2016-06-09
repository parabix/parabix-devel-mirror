/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef UTF16_ENCODER_H
#define UTF16_ENCODER_H

#include <re/re_cc.h>

namespace cc {

struct UTF16_Encoder {
    static bool isHi_Surrogate(const re::codepoint_t cp);
    static bool isLo_Surrogate(const re::codepoint_t cp);
    static unsigned length(const re::codepoint_t cp);
    static re::codepoint_t maxCodePoint(const unsigned length);
    static re::codepoint_t encodingByte(const re::codepoint_t cp, const unsigned n);
    static bool isLowCodePointAfterByte(const re::codepoint_t cp, const unsigned n);
    static bool isHighCodePointAfterByte(const re::codepoint_t cp, const unsigned n);
    static re::codepoint_t minCodePointWithCommonBytes(const re::codepoint_t cp, const unsigned n);
    static re::codepoint_t maxCodePointWithCommonBytes(const re::codepoint_t cp, const unsigned n);
};

}

#endif // UTF16_ENCODER_H
