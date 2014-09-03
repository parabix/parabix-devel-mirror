/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_cc.h"

int CC::msCSIidx = 0;

CC::CC()
{
    gensym_name();
}

CC::CC(int codepoint)
{
    gensym_name();
    insert1(codepoint);
}

CC::CC(int lo_codepoint, int hi_codepoint)
{
    gensym_name();
    insert_range(lo_codepoint, hi_codepoint);
}

CC::CC(CC *cc1, CC *cc2)
{
    gensym_name();
    mSparseCharSet = cc2->getItems();
    joinCharSets(cc1->getItems());
}

CC::~CC(){}

std::vector<CharSetItem> CC::getItems()
{
    return mSparseCharSet;
}

std::string CC::getName()
{
    std::string name = "CC";

    std::vector<CharSetItem>::iterator it;
    for (it = mSparseCharSet.begin(); it != mSparseCharSet.end(); ++it)
    {
        name += "_" + std::to_string(it->lo_codepoint);
        name += "." + std::to_string(it->hi_codepoint);
    }

    return name;
}

std::string CC::getId()
{
    return mId;
}

bool CC::is_member(int codepoint)
{
    return is_member_helper(codepoint, mSparseCharSet.size() - 1);
}

bool CC::is_member_helper(int codepoint, int idx)
{
    if (idx == -1)
    {
        return false;
    }
    else
    {
        CharSetItem item = mSparseCharSet.at(idx);

        if (codepoint < item.lo_codepoint)
        {
            return false;
        }
        else if (codepoint > item.hi_codepoint)
        {
            idx--;
            return is_member_helper(codepoint, idx);
        }
        else
        {
            return true;
        }
    }
}

void CC::joinCharSets(std::vector<CharSetItem> items1)
{
    joinCharSets_helper(items1, items1.size() - 1);
}

void CC::joinCharSets_helper(std::vector<CharSetItem> items1, int idx)
{
    if (idx > -1)
    {
        CharSetItem item = items1.at(idx);
        insert_range(item.lo_codepoint, item.hi_codepoint);
        idx--;
        joinCharSets_helper(items1, idx);
    }
}

void CC::insert1(int codepoint)
{
    insert_range(codepoint, codepoint);
}

void CC::insert_range(int lo_codepoint, int hi_codepoint)
{
    insert_range_helper(lo_codepoint, hi_codepoint, mSparseCharSet.size() - 1);
}

void CC::insert_range_helper(int lo_codepoint, int hi_codepoint, int idx)
{
    if (idx == -1)
    {
        CharSetItem new_item;
        new_item.lo_codepoint = lo_codepoint;
        new_item.hi_codepoint = hi_codepoint;
        std::vector<CharSetItem>::iterator it;
        it = mSparseCharSet.begin();
        mSparseCharSet.insert(it, new_item);
    }
    else
    {
        CharSetItem item = mSparseCharSet.at(idx);

        if (hi_codepoint < item.lo_codepoint - 1)
        {
            CharSetItem new_item;
            new_item.lo_codepoint = lo_codepoint;
            new_item.hi_codepoint = hi_codepoint;
            std::vector<CharSetItem>::iterator it;
            it = mSparseCharSet.begin();
            mSparseCharSet.insert(it + idx + 1, new_item);
        }
        else if (lo_codepoint > item.hi_codepoint + 1)
        {
            idx--;
            insert_range_helper(lo_codepoint, hi_codepoint, idx);
        }
        else
        {
            int overlap_lo = item.lo_codepoint;
            int overlap_hi = item.hi_codepoint;
            std::vector<CharSetItem>::iterator it;
            it = mSparseCharSet.begin();
            mSparseCharSet.erase(it + idx);
            idx--;
            insert_range_helper(std::min(overlap_lo, lo_codepoint), std::max(overlap_hi, hi_codepoint), idx);
        }
    }
}

void CC::negate_class()
{
    negate_class_helper(mSparseCharSet.size() - 1, 0);
}

void CC::negate_class_helper(int idx, int b)
{
    if (idx == -1)
    {
        if (b <= mUnicodeMax)
        {
            CharSetItem new_item;

            new_item.lo_codepoint = b;
            new_item.hi_codepoint = mUnicodeMax;
            std::vector<CharSetItem>::iterator it;
            it = mSparseCharSet.begin();
            mSparseCharSet.insert(it, new_item);
        }
    }
    else
    {
        CharSetItem item = mSparseCharSet.at(idx);

        if (b < item.lo_codepoint)
        {
            CharSetItem new_item;

            new_item.lo_codepoint = b;
            new_item.hi_codepoint = item.lo_codepoint - 1;
            std::vector<CharSetItem>::iterator it;
            it = mSparseCharSet.begin();
            mSparseCharSet.erase(it + idx);
            mSparseCharSet.insert(it + idx, new_item);
            idx--;
            negate_class_helper(idx, item.hi_codepoint + 1);
        }
        else
        {
            std::vector<CharSetItem>::iterator it;
            it = mSparseCharSet.begin();
            mSparseCharSet.erase(it + idx);
            idx--;
            negate_class_helper(idx, item.hi_codepoint + 1);
        }
    }
}

void CC::remove1(int codepoint)
{
    remove_range(codepoint, codepoint);
}

void CC::remove_range(int lo_codepoint, int hi_codepoint)
{
    remove_range_helper(lo_codepoint, hi_codepoint, mSparseCharSet.size() - 1);
}

void CC::remove_range_helper(int lo_codepoint, int hi_codepoint, int idx)
{
    if (idx != -1)
    {
        CharSetItem item = mSparseCharSet.at(idx);

        if (hi_codepoint < item.lo_codepoint - 1)
        {
            return;
        }
        else if (lo_codepoint > item.hi_codepoint + 1)
        {
            idx--;
            remove_range_helper(lo_codepoint, hi_codepoint, idx);
        }
        else if ((lo_codepoint <= item.lo_codepoint) && (hi_codepoint >= item.hi_codepoint))
        {
            std::vector<CharSetItem>::iterator it;
            it = mSparseCharSet.begin();
            mSparseCharSet.erase(it + idx);
            idx--;
            remove_range_helper(lo_codepoint, hi_codepoint, idx);
        }
        else if (lo_codepoint <= item.lo_codepoint)
        {
            CharSetItem new_item;
            new_item.lo_codepoint = hi_codepoint + 1;
            new_item.hi_codepoint = item.hi_codepoint;
            std::vector<CharSetItem>::iterator it;
            it = mSparseCharSet.begin();
            mSparseCharSet.erase(it + idx);
            mSparseCharSet.insert(it + idx, new_item);
        }
        else if (hi_codepoint >= item.hi_codepoint)
        {
            CharSetItem new_item;
            new_item.lo_codepoint = item.lo_codepoint;
            new_item.hi_codepoint = lo_codepoint - 1;
            std::vector<CharSetItem>::iterator it;
            it = mSparseCharSet.begin();
            mSparseCharSet.erase(it + idx);
            mSparseCharSet.insert(it + idx, new_item);
            idx--;
            remove_range_helper(lo_codepoint, hi_codepoint, idx);
        }
        else
        {
            CharSetItem new_item1;
            new_item1.lo_codepoint = hi_codepoint + 1;
            new_item1.hi_codepoint = item.hi_codepoint;
            CharSetItem new_item2;
            new_item2.lo_codepoint = item.lo_codepoint;
            new_item2.hi_codepoint = lo_codepoint - 1;
            std::vector<CharSetItem>::iterator it;
            it = mSparseCharSet.begin();
            mSparseCharSet.erase(it + idx);
            mSparseCharSet.insert(it + idx, new_item1);
            mSparseCharSet.insert(it + idx, new_item2);
        }
    }
}

void CC::gensym_name()
{
    mId = "lex.CC" + std::to_string(msCSIidx);
    msCSIidx++;
}

