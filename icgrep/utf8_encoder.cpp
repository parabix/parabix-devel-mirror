/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "utf8_encoder.h"

#include "re/re_name.h"
#include "re/re_start.h"
#include "re/re_end.h"
#include "re/re_seq.h"
#include "re/re_alt.h"
#include "re/re_rep.h"
#include "re/re_simplifier.h"

#include <assert.h>
#include <algorithm>
#include <stdexcept>

using namespace re;

RE * UTF8_Encoder::toUTF8(RE* re) {
    if (Alt * alt = dyn_cast<Alt>(re)) {
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            *i = toUTF8(*i);
        }
    }
    else if (Seq * seq = dyn_cast<Seq>(re)) {
        //If this is a previously encoded Unicode byte sequence.
        if (seq->getType() == Seq::Type::Byte) {
            throw std::runtime_error("Unexpected UTF Byte Sequence given to UTF8 Encoder.");
        }
        for (auto i = seq->begin(); i != seq->end(); ++i) {
            *i = toUTF8(*i);
        }
    }
    else if (CC * cc = dyn_cast<CC>(re)) {
        if (cc->size() == 1) {
            re = rangeToUTF8(cc->front());
        }
        else if (cc->size() > 1) {
            std::vector<RE *> alt;
            for (const CharSetItem & item : *cc) {
                alt.push_back(rangeToUTF8(item));
            }
            re = makeAlt(alt.begin(), alt.end());
        }
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        rep->setRE(toUTF8(rep->getRE()));
    }
    return re;
}

RE * UTF8_Encoder::rangeToUTF8(const CharSetItem & item) {
    int u8len_lo = lenUTF8(item.lo_codepoint);
    int u8len_hi = lenUTF8(item.hi_codepoint);
    if (u8len_lo < u8len_hi) {
        int m = maxUTF8Len(u8len_lo);
        return makeAlt({rangeToUTF8(CharSetItem(item.lo_codepoint, m)), rangeToUTF8(CharSetItem(m + 1, item.hi_codepoint))});
    }
    else {
        return rangeToUTF8_helper(item.lo_codepoint, item.hi_codepoint, 1, u8len_hi);
    }
}

RE* UTF8_Encoder::rangeToUTF8_helper(int lo, int hi, int n, int hlen)
{
    int hbyte = u8byte(hi, n);
    int lbyte = u8byte(lo, n);

    if (n == hlen)
    {
        return makeByteRange(lbyte, hbyte);
    }
    else if (hbyte == lbyte)
    {
        Seq* seq = makeSeq(isUTF8Prefix(hbyte) ? Seq::Type::Byte : Seq::Type::Normal);
        seq->push_back(makeByteClass(hbyte));
        seq->push_back(rangeToUTF8_helper(lo, hi, n+1, hlen));
        return seq;
    }
    else
    {
        int suffix_mask = (1 << ((hlen - n) * 6)) - 1;

        if ((hi & suffix_mask) != suffix_mask)
        {
            int hi_floor = (~suffix_mask) & hi;
            return makeAlt({rangeToUTF8_helper(hi_floor, hi, n, hlen), rangeToUTF8_helper(lo, hi_floor - 1, n, hlen)});
        }
        else if ((lo & suffix_mask) != 0)
        {
            int low_ceil = lo | suffix_mask;

            Alt* alt = makeAlt();
            alt->push_back(rangeToUTF8_helper(low_ceil + 1, hi, n, hlen));
            alt->push_back(rangeToUTF8_helper(lo, low_ceil, n, hlen));
            return alt;
        }
        else
        {
            Seq* seq = makeSeq();
            seq->setType((isUTF8Prefix(hbyte) ? Seq::Type::Byte : Seq::Type::Normal));
            seq->push_back(makeByteRange(lbyte, hbyte));
            seq->push_back(rangeToUTF8_helper(lo, hi, n + 1, hlen));
            return seq;
        }
    }
}

inline bool UTF8_Encoder::isUTF8Prefix(const int cp) {
    return ((cp >= 0xC2) && (cp <= 0xF4));
}

CC* UTF8_Encoder::makeByteRange(int lo, int hi)
{
    return makeCC(lo, hi);
}

CC* UTF8_Encoder::makeByteClass(int byteval)
{
    return makeCC(byteval, byteval);
}

inline int UTF8_Encoder::u8byte(int codepoint, int n)
{
    int retVal = 0;

    int len = lenUTF8(codepoint);

    if (n == 1)
    {
        if (len == 1)
        {
            retVal = codepoint;
        }
        else if (len == 2)
        {
            retVal = 0xC0 | (codepoint >> 6);
        }
        else if (len == 3)
        {
            retVal = 0xE0 | (codepoint >> 12);
        }
        else
        {
            retVal = 0xF0 | (codepoint >> 18);
        }
    }
    else
    {
        retVal = 0x80 | ((codepoint >> (6 * (len - n))) & 0x3F);
    }

    return retVal;
}

inline int UTF8_Encoder::lenUTF8(const int cp)
{
    if (cp <= 0x7F)
    {
        return 1;
    }
    else if (cp <= 0x7FF)
    {
        return 2;
    }
    else if (cp <= 0xFFFF)
    {
        return 3;
    }
    else
    {
        return 4;
    }
}

inline int UTF8_Encoder::maxUTF8Len(int lgth)
{
    if (lgth == 1)
    {
        return 0x7F;
    }
    else if (lgth == 2)
    {
        return 0x7FF;
    }
    else if (lgth == 3)
    {
        return 0xFFFF;
    }
    else if (lgth == 4)
    {
        return 0x10FFFF;
    }
    else
    {
        return -1;
    }
}

