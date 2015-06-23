/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_cc.h"
#include <llvm/Support/Compiler.h>
#include <UCD/CaseFolding_txt.h>
#include <sstream>

namespace re {
CC::IntervalAllocator CC::mCharSetAllocator;

CC::CC(const CC * cc1, const CC * cc2)
: RE(ClassTypeId::CC)
, mSparseCharSet(cc1->cbegin(), cc1->cend(), mCharSetAllocator) {
    for (const interval_t & i : cc2->mSparseCharSet) {
        insert_range(lo_codepoint(i), hi_codepoint(i));
    }
}

CC::CC(const CC & cc)
: RE(ClassTypeId::CC)
, mSparseCharSet(cc.cbegin(), cc.cend(), mCharSetAllocator) {

}

std::string CC::canonicalName(const CC_type type) const {
    std::stringstream name;
    name << std::hex;
    if ((type == ByteClass) && (max_codepoint() >= 0x80)) {
        name << "BC";
    }
    else {
        name << "CC";
    }
    char separator = '_';
    for (const interval_t & i : mSparseCharSet) {
        name << separator;
        if (lo_codepoint(i) == hi_codepoint(i)) {
            name << lo_codepoint(i);
        }
        else {
            name << lo_codepoint(i) << '_' << hi_codepoint(i);
        }
        separator = ',';
    }
    return name.str();
}

void CC::insert_range(const codepoint_t lo, const codepoint_t hi) {
    for (auto i = mSparseCharSet.begin(); i != mSparseCharSet.end(); ) {
        if (hi < lo_codepoint(i) - 1) {
            mSparseCharSet.emplace(i, lo, hi);
            return;
        }
        else if (lo > hi_codepoint(i) + 1) {
            ++i;
        }
        else {
            // ranges overlap; expand the range to include the prior one and
            // remove the old one from the list
            lo_codepoint(i) = std::min(lo_codepoint(i), lo);
            hi_codepoint(i) = std::max(hi_codepoint(i), hi);
            return;
        }
    }
    mSparseCharSet.emplace_back(lo, hi);
}

void CC::remove_range(const codepoint_t lo, const codepoint_t hi) {
    for (auto i = mSparseCharSet.begin(); i != mSparseCharSet.end(); ) {
        if (lo > hi_codepoint(i) + 1) {
            ++i;
        }
        else if (hi < lo_codepoint(i) - 1) {
            break;
        }
        else if (lo <= lo_codepoint(i) && hi >= hi_codepoint(i)) {
            i = mSparseCharSet.erase(i);
        }
        else if (lo <= lo_codepoint(i)) {
            lo_codepoint(i) = hi + 1;
            break;
        }
        else if (hi >= hi_codepoint(i)) {
            hi_codepoint(i) = lo - 1;
            ++i;
        }
        else {         
            mSparseCharSet.emplace(++i, hi + 1, hi_codepoint(i));
            hi_codepoint(i) = lo - 1;
            break;
        }
    }
}

CC * subtractCC(const CC * a, const CC * b) {
    CC * diff = makeCC();
    auto i = a->cbegin();
    const auto i_end = a->cend();
    auto j = b->cbegin();
    const auto j_end = b->cend();
    while (i != i_end && j != j_end) {
        if (hi_codepoint(j) < lo_codepoint(i)) {
            ++j;
        }
        else { // test whether the intervals overlap
            if (lo_codepoint(i) < lo_codepoint(j)) {
                diff->insert_range(lo_codepoint(i), std::min(lo_codepoint(j) - 1, hi_codepoint(i)));
            }
            if (hi_codepoint(i) > hi_codepoint(j)) {
                diff->insert_range(std::max(hi_codepoint(j) + 1, lo_codepoint(i)), hi_codepoint(i));
            }
            ++i;
        }
    }
    for (; i != i_end; ++i) {
        diff->insert_range(lo_codepoint(i), hi_codepoint(i));
    }
    return diff;
}
    
CC * intersectCC(const CC * a, const CC * b) {
    CC * isect = makeCC();
    auto ai = a->cbegin();
    const auto ai_end = a->cend();
    auto bi = b->cbegin();
    const auto bi_end = b->cend();
    while (ai != ai_end && bi != bi_end) {
        if (hi_codepoint(ai) < lo_codepoint(bi)) {
            ++ai;
        }
        else if (hi_codepoint(bi) < lo_codepoint(ai)) {
            ++bi;
        }
        else {
            isect->insert_range(std::max(lo_codepoint(ai), lo_codepoint(bi)), std::min(hi_codepoint(ai), hi_codepoint(bi)));
            if (hi_codepoint(ai) < hi_codepoint(bi)) {
                ++ai;
            }
            else {
                ++bi;
            }
        }
    }
    return isect;
}
    
CC * caseInsensitize(const CC * cc) {
    CC * cci = makeCC();
    for (const interval_t & i : *cc) {
        caseInsensitiveInsertRange(cci, lo_codepoint(i), hi_codepoint(i));
    }
    return cci;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief rangeIntersect
 * @param cc
 * @param lo
 * @param hi
 ** ------------------------------------------------------------------------------------------------------------- */
CC * rangeIntersect(const CC * cc, const codepoint_t lo, const codepoint_t hi) {
    assert ("cc cannot be null" && cc);
    CC * intersect = makeCC();
    for (const auto & i : *cc) {
        if ((lo_codepoint(i) <= hi) && (hi_codepoint(i) >= lo)) {
            intersect->insert_range(std::max(lo, lo_codepoint(i)), std::min(hi, hi_codepoint(i)));
        }
    }
    return intersect;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief rangeGaps
 * @param cc
 * @param lo
 * @param hi
 ** ------------------------------------------------------------------------------------------------------------- */
CC * rangeGaps(const CC * cc, const codepoint_t lo, const codepoint_t hi) {
    assert ("cc cannot be null" && cc);
    CC * gaps = makeCC();
    codepoint_t cp = lo;
    if (cp < hi) {
        auto i = cc->cbegin(), end = cc->cend();
        for (; i != end && cp < hi; ++i) {
            if (hi_codepoint(i) < cp) {
                continue;
            }
            else if (lo_codepoint(i) > cp) {
                gaps->insert_range(cp, lo_codepoint(i) - 1);
            }
            cp = hi_codepoint(i) + 1;
        }
        if (cp < hi) {
            gaps->insert_range(cp, hi);
        }
    }
    return gaps;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief outerRanges
 * @param cc
 ** ------------------------------------------------------------------------------------------------------------- */
CC * outerRanges(const CC * cc) {
    assert ("cc cannot be null" && cc);
    CC * ranges = makeCC();
    auto i = cc->cbegin();
    const auto end = cc->cend();
    for (auto j = i; ++j != end; ) {
        if (hi_codepoint(j) > hi_codepoint(i)) {
            ranges->insert_range(lo_codepoint(i), hi_codepoint(i));
            i = j;
        }
    }
    return ranges;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief innerRanges
 * @param cc
 ** ------------------------------------------------------------------------------------------------------------- */
CC * innerRanges(const CC * cc) {
    assert ("cc cannot be null" && cc);
    CC * ranges = makeCC();
    auto i = cc->cbegin();
    const auto end = cc->cend();
    for (auto j = i; ++j != end; ) {
        if (hi_codepoint(j) <= hi_codepoint(i)) {
            ranges->insert_range(lo_codepoint(j), hi_codepoint(j));
        }
        else {
            i = j;
        }
    }
    return ranges;
}
    
}
