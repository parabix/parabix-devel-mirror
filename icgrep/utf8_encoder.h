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

class UTF8_Encoder
{
public:
    static re::RE * toUTF8(CC_NameMap & nameMap, re::RE * ast);
private:
    static re::RE * rangeToUTF8(const re::CharSetItem & item);
    static re::RE * rangeToUTF8(const re::codepoint_t lo, const re::codepoint_t hi, const unsigned index, const unsigned max);
    static re::CC * makeByteClass(const re::codepoint_t cp);
    static re::CC * makeByteRange(const re::codepoint_t lo, const re::codepoint_t hi);
    static bool isUTF8Prefix(const unsigned cp);
    static unsigned lenUTF8(const unsigned cp);
    static unsigned maxCodePoint(const unsigned length);
    static re::codepoint_t u8byte(const re::codepoint_t codepoint, const unsigned n);
};

}

#endif // UTF8_ENCODER_H
