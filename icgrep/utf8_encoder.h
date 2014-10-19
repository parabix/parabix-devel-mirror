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
    static re::RE * rangeToUTF8(const re::CodePointType lo, const re::CodePointType hi, const unsigned index, const unsigned max);
    static re::CC * makeByteClass(const re::CodePointType cp);
    static re::CC * makeByteRange(const re::CodePointType lo, const re::CodePointType hi);
    static bool isUTF8Prefix(const unsigned cp);
    static unsigned lenUTF8(const unsigned cp);
    static unsigned maxCodePoint(const unsigned length);
    static re::CodePointType u8byte(const re::CodePointType codepoint, const unsigned n);
};

}

#endif // UTF8_ENCODER_H
