/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef UTF8_ENCODER_H
#define UTF8_ENCODER_H

#include <re/re_cc.h>

namespace cc {

struct UTF8_Encoder {
    static bool isPrefix(const re::codepoint_t cp);
    static unsigned length(const re::codepoint_t cp);
    static re::codepoint_t maxCodePoint(const unsigned length);
    static re::codepoint_t encodingByte(const re::codepoint_t cp, const unsigned n);
    static bool isLowCodePointAfterByte(const re::codepoint_t cp, const unsigned n);
    static bool isHighCodePointAfterByte(const re::codepoint_t cp, const unsigned n);
    static re::codepoint_t minCodePointWithCommonBytes(const re::codepoint_t cp, const unsigned n);
    static re::codepoint_t maxCodePointWithCommonBytes(const re::codepoint_t cp, const unsigned n);
};

}

#endif // UTF8_ENCODER_H
