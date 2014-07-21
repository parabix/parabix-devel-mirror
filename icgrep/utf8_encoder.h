/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef UTF8_ENCODER_H
#define UTF8_ENCODER_H

//Regular Expressions
#include "re_re.h"
#include "re_cc.h"
#include "re_start.h"
#include "re_end.h"
#include "re_seq.h"
#include "re_alt.h"
#include "re_rep.h"

#include "rl_replimit.h"
#include "rl_unbounded.h"
#include "rl_upperbound.h"

class UTF8_Encoder
{
public:
    UTF8_Encoder();
    RE* toUTF8(RE* re);
private:
    RE* rangeToUTF8(CharSetItem item);
    RE* rangeToUTF8_helper(int lo, int hi, int n, int hlen);
    CC* makeByteClass(int byteval);
    CC* makeByteRange(int lo, int hi);

    int u8len(int cp);
    int max_of_u8len(int lgth);
    int u8byte(int codepoint, int n);
};

#endif // UTF8_ENCODER_H
