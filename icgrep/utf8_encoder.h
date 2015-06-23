/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef UTF8_ENCODER_H
#define UTF8_ENCODER_H

//Regular Expressions
#include <re/re_cc.h>
#include <cc/cc_namemap.hpp>

namespace cc {

class CC_NameMap;

class UTF8_Encoder {
public:
    static re::RE * toUTF8(CC_NameMap & nameMap, re::RE * ast);

    static bool isUTF8Prefix(const re::codepoint_t cp);
    static unsigned lenUTF8(const re::codepoint_t cp);
    static re::codepoint_t maxCodePoint(const unsigned length);
    static re::codepoint_t u8byte(const re::codepoint_t cp, const unsigned n);
    static bool isLowCodePointAfterByte(const re::codepoint_t cp, const unsigned index);
    static bool isHighCodePointAfterByte(const re::codepoint_t cp, const unsigned index);
private:
    static re::RE * rangeToUTF8(const re::interval_t & item);
    static re::RE * rangeToUTF8(const re::codepoint_t lo, const re::codepoint_t hi, const unsigned index, const unsigned max);
    static re::CC * makeByteClass(const re::codepoint_t cp);
    static re::CC * makeByteRange(const re::codepoint_t lo, const re::codepoint_t hi);
};

}

#endif // UTF8_ENCODER_H
