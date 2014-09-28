/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef UTF8_ENCODER_H
#define UTF8_ENCODER_H

//Regular Expressions
#include "re/re_re.h"
#include "re/re_cc.h"


class UTF8_Encoder
{
public:
    static re::RE* toUTF8(re::RE * re);
private:
    static re::RE* rangeToUTF8(const re::CharSetItem &item);
    static re::RE* rangeToUTF8_helper(int lo, int hi, int n, int hlen);
    static re::CC* makeByteClass(int byteval);
    static re::CC* makeByteRange(int lo, int hi);

    static bool u8Prefix(int cp);
    static int u8len(int cp);
    static int max_of_u8len(int lgth);
    static int u8byte(int codepoint, int n);
};

#endif // UTF8_ENCODER_H
