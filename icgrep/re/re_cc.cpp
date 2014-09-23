/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_cc.h"
#include <assert.h>
#include <atomic>

CC::CC() {

}

CC::CC(const CodePointType codepoint) {
    insert(codepoint);
}

CC::CC(const CodePointType lo_codepoint, const CodePointType hi_codepoint) {
    insert_range(lo_codepoint, hi_codepoint);
}

CC::CC(const CC * cc1, const CC * cc2) {
    mSparseCharSet.assign(cc1->cbegin(), cc1->cend());
    join(cc2->mSparseCharSet);
}

CC::~CC() {

}

std::string CC::getName() const {
    std::string name = "CC";
    for (const CharSetItem & i : mSparseCharSet) {
        name += "_" + std::to_string(i.lo_codepoint);
        name += "." + std::to_string(i.hi_codepoint);
    }
    return name;
}

void CC::join(const CharSetVector & other) {
    for (const CharSetItem & i : other) {
        insert_range(i.lo_codepoint, i.hi_codepoint);
    }
}

void CC::insert_range(const CodePointType lo_codepoint, const CodePointType hi_codepoint) {
    CharSetItem item(lo_codepoint, hi_codepoint);
    for (auto i = mSparseCharSet.begin(); i != mSparseCharSet.end(); ) {
        CharSetItem & range = *i;
        if (item.hi_codepoint < range.lo_codepoint - 1) {
            mSparseCharSet.insert(i, item);
            return;
        }
        else if (item.lo_codepoint > range.hi_codepoint + 1) {
            ++i;
        }
        else {
            // ranges overlap; expand the range to include the prior one and
            // remove the old one from the list
            item.lo_codepoint = std::min(range.lo_codepoint, item.lo_codepoint);
            item.hi_codepoint = std::max(range.hi_codepoint, item.hi_codepoint);
            i = mSparseCharSet.erase(i);
        }
    }
    mSparseCharSet.push_back(item);
}

void CC::negate() {
    CharSetVector negated;
    negated.reserve(mSparseCharSet.size() + 1);
    CodePointType lo_codepoint = 0;
    for (const CharSetItem & item : mSparseCharSet) {
        negated.push_back(std::move(CharSetItem(lo_codepoint, item.lo_codepoint - 1)));
        lo_codepoint = item.hi_codepoint + 1;
    }
    if (lo_codepoint <= UNICODE_MAX) {
        negated.push_back(std::move(CharSetItem(lo_codepoint, UNICODE_MAX)));
    }
    mSparseCharSet.assign(negated.begin(), negated.end());
}

void CC::remove_range(const CodePointType lo_codepoint, const CodePointType hi_codepoint) {
    for (auto i = mSparseCharSet.begin(); i != mSparseCharSet.end(); ) {
        CharSetItem & range = *i;
        if (lo_codepoint > range.hi_codepoint + 1) {
            ++i;
        }
        else if (hi_codepoint < range.lo_codepoint - 1) {
            break;
        }
        else if (lo_codepoint <= range.lo_codepoint && hi_codepoint >= range.hi_codepoint) {
            i = mSparseCharSet.erase(i);
        }
        else if (lo_codepoint <= range.lo_codepoint) {
            range.lo_codepoint = hi_codepoint + 1;
            break;
        }
        else if (hi_codepoint >= range.hi_codepoint) {
            range.hi_codepoint = lo_codepoint - 1;
            ++i;
        }
        else {
            CharSetItem item(hi_codepoint + 1, range.hi_codepoint);
            range.hi_codepoint = lo_codepoint - 1;
            mSparseCharSet.insert(++i, std::move(item));
            break;
        }
    }
}
