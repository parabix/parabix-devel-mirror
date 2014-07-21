/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "utf8_encoder.h"


RE* UTF8_Encoder::toUTF8(RE* re)
{
    RE* retVal = 0;

    if (Alt* re_alt = dynamic_cast<Alt*>(re))
    {
        std::list<RE*> re_list;
        std::list<RE*>::reverse_iterator rit = re_alt->GetREList()->rbegin();

        for (rit = re_alt->GetREList()->rbegin(); rit != re_alt->GetREList()->rend(); ++rit)
        {
            re_list.push_back(toUTF8(*rit));
        }

        retVal = new Alt(&re_list);
    }
    else if (Seq* re_seq = dynamic_cast<Seq*>(re))
    {
        std::list<RE*> re_list;
        std::list<RE*>::iterator it;

        for (it = re_seq->GetREList()->begin(); it != re_seq->GetREList()->end(); ++it)
        {
            re_list.push_front(toUTF8(*it));
        }

        retVal = new Seq(&re_list);
    }
    else if (Rep* re_rep = dynamic_cast<Rep*>(re))
    {
        retVal = new Rep(toUTF8(re_rep->getRE()), re_rep->getLB(), re_rep->getUB());
    }
    else if (CC* re_cc = dynamic_cast<CC*>(re))
    {  
        if (re_cc->getItems().size() == 1)
        {
            retVal = rangeToUTF8(re_cc->getItems().front());
        }
        else if (re_cc->getItems().size() > 1)
        {           
            std::list<RE*> re_list;
            for (int i = 0; i < re_cc->getItems().size(); i++)
            {
                re_list.push_back(rangeToUTF8(re_cc->getItems().at(i)));
            }
            retVal = RE_Simplifier::mkAlt(&re_list);
            //retVal = new Alt(&re_list);
        }
    }
    else if (Start* re_start = dynamic_cast<Start*>(re))
    {
        retVal = new Start();
    }
    else if (End* re_end = dynamic_cast<End*>(re))
    {
        retVal = new End();
    }

    return retVal;
}

RE* UTF8_Encoder::rangeToUTF8(CharSetItem item)
{
    int u8len_lo = u8len(item.lo_codepoint);
    int u8len_hi = u8len(item.hi_codepoint);

    if (u8len_lo < u8len_hi)
    {
        int m = max_of_u8len(u8len_lo);
        Alt* alt = new Alt();

        CharSetItem lo_item;
        lo_item.lo_codepoint = item.lo_codepoint;
        lo_item.hi_codepoint = m;
        alt->AddREListItem(rangeToUTF8(lo_item));
        CharSetItem hi_item;
        hi_item.lo_codepoint = m + 1;
        hi_item.hi_codepoint = item.hi_codepoint;
        alt->AddREListItem(rangeToUTF8(hi_item));

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
        seq->AddREListItem(makeByteClass(hbyte));
        seq->AddREListItem(rangeToUTF8_helper(lo, hi, n+1, hlen));
        return seq;
    }
    else
    {
        int suffix_mask = (1 << ((hlen - n) * 6)) - 1;

        if ((hi & suffix_mask) != suffix_mask)
        {
            int hi_floor = (~suffix_mask) & hi;

            Alt* alt = new Alt();
            alt->AddREListItem(rangeToUTF8_helper(hi_floor, hi, n, hlen));
            alt->AddREListItem(rangeToUTF8_helper(lo, hi_floor - 1, n, hlen));
            return alt;
        }
        else if ((lo & suffix_mask) != 0)
        {
            int low_ceil = lo | suffix_mask;

            Alt* alt = new Alt();
            alt->AddREListItem(rangeToUTF8_helper(low_ceil + 1, hi, n, hlen));
            alt->AddREListItem(rangeToUTF8_helper(lo, low_ceil, n, hlen));
            return alt;
        }
        else
        {
            Seq* seq = new Seq();
            seq->AddREListItem(makeByteRange(lbyte, hbyte));
            seq->AddREListItem(rangeToUTF8_helper(lo, hi, n + 1, hlen));
            return seq;
        }
    }
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

