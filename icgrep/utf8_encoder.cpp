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
#include <stdexcept>

RE* UTF8_Encoder::toUTF8(RE* re) {

    RE* retVal = nullptr;
    if (Alt* re_alt = dynamic_cast<Alt*>(re)) {
        Alt * new_alt = new Alt();
        for (RE * re : *re_alt) {
            new_alt->push_back(toUTF8(re));
        }
        retVal = new_alt;
    }
    else if (Seq * re_seq = dynamic_cast<Seq*>(re)) {
        //If this is a previously encoded Unicode byte sequence.
        if (re_seq->getType() == Seq::Byte) {
            throw std::runtime_error("Unexpected UTF Byte Sequence given to UTF8 Encoder.");
        }
        Seq * new_seq = new Seq(Seq::Normal);
        for (RE * re : *re_seq) {
            new_seq->push_back(toUTF8(re));
        }
        retVal = new_seq;
    }
    else if (Rep* re_rep = dynamic_cast<Rep*>(re))
    {
        retVal = new Rep(toUTF8(re_rep->getRE()), re_rep->getLB(), re_rep->getUB());
    }
    else if (CC* re_cc = dynamic_cast<CC*>(re))
    {  
        if (re_cc->size() == 1)
        {
            retVal = rangeToUTF8(re_cc->front());
        }
        else if (re_cc->size() > 1) {
            RE::Vector re_list;
            for (const CharSetItem & item : *re_cc) {
                re_list.push_back(rangeToUTF8(item));
            }
            retVal = RE_Simplifier::makeAlt(re_list);
        }
    }
    else if (Name* re_name = dynamic_cast<Name*>(re)) {
        retVal = new Name(re_name);
    }
    else if (dynamic_cast<Start*>(re)) {
        retVal = new Start();
    }
    else if (dynamic_cast<End*>(re)) {
        retVal = new End();
    }

    return retVal;
}

RE * UTF8_Encoder::rangeToUTF8(const CharSetItem & item) {
    int u8len_lo = u8len(item.lo_codepoint);
    int u8len_hi = u8len(item.hi_codepoint);
    if (u8len_lo < u8len_hi)
    {
        int m = max_of_u8len(u8len_lo);
        Alt* alt = new Alt();

        CharSetItem lo_item;
        lo_item.lo_codepoint = item.lo_codepoint;
        lo_item.hi_codepoint = m;
        alt->push_back(rangeToUTF8(lo_item));
        CharSetItem hi_item;
        hi_item.lo_codepoint = m + 1;
        hi_item.hi_codepoint = item.hi_codepoint;
        alt->push_back(rangeToUTF8(hi_item));

        return alt;
    }
    else
    {
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
        Seq* seq = new Seq();
        seq->setType((u8Prefix(hbyte) ? Seq::Byte : Seq::Normal));
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

            Alt* alt = new Alt();
            alt->push_back(rangeToUTF8_helper(hi_floor, hi, n, hlen));
            alt->push_back(rangeToUTF8_helper(lo, hi_floor - 1, n, hlen));
            return alt;
        }
        else if ((lo & suffix_mask) != 0)
        {
            int low_ceil = lo | suffix_mask;

            Alt* alt = new Alt();
            alt->push_back(rangeToUTF8_helper(low_ceil + 1, hi, n, hlen));
            alt->push_back(rangeToUTF8_helper(lo, low_ceil, n, hlen));
            return alt;
        }
        else
        {
            Seq* seq = new Seq();
            seq->setType((u8Prefix(hbyte) ? Seq::Byte : Seq::Normal));
            seq->push_back(makeByteRange(lbyte, hbyte));
            seq->push_back(rangeToUTF8_helper(lo, hi, n + 1, hlen));
            return seq;
        }
    }
}

bool UTF8_Encoder::u8Prefix(int cp)
{
    return ((cp >= 0xC2) && (cp <= 0xF4));
}

CC* UTF8_Encoder::makeByteRange(int lo, int hi)
{
    return new CC(lo, hi);
}

CC* UTF8_Encoder::makeByteClass(int byteval)
{
    return new CC(byteval, byteval);
}

int UTF8_Encoder::u8byte(int codepoint, int n)
{
    int retVal = 0;

    int len = u8len(codepoint);

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

int UTF8_Encoder::u8len(int cp)
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

int UTF8_Encoder::max_of_u8len(int lgth)
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

