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
CC::CharSetAllocator CC::mCharSetAllocator;

CC::CC(const CC * cc1, const CC * cc2)
: RE(ClassTypeId::CC)
, mSparseCharSet(cc1->cbegin(), cc1->cend(), mCharSetAllocator) {
    for (const CharSetItem & i : cc2->mSparseCharSet) {
        insert_range(i.lo_codepoint, i.hi_codepoint);
    }
}

CC::CC(const CC & cc)
: RE(ClassTypeId::CC)
, mSparseCharSet(cc.cbegin(), cc.cend(), mCharSetAllocator) {

}

std::string CC::canonicalName(const CC_type type) const {
    std::stringstream name;
    name << std::hex;
    if ((type == ByteClass) && (mSparseCharSet.back().hi_codepoint >= 0x80)) {
      name << "BC_";
    }
    else {
        name << "CC_";
    }
    char separator = '_';
    for (const CharSetItem & i : mSparseCharSet) {
        name << separator;
        if (i.lo_codepoint == i.hi_codepoint) {
            name << i.lo_codepoint;
        }
        else {
            name << i.lo_codepoint << '_' << i.hi_codepoint;
        }
    }
    return name.str();
}

CodePointType CC::max_codepoint() {
    return mSparseCharSet.size() == 0 ? 0 : mSparseCharSet.back().hi_codepoint;
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
            range.lo_codepoint = std::min(range.lo_codepoint, item.lo_codepoint);
            range.hi_codepoint = std::max(range.hi_codepoint, item.hi_codepoint);
            return;
        }
    }
    mSparseCharSet.push_back(item);
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

CC * subtractCC(const CC * cc1, const CC * cc2) {
    CC * diff = makeCC();
    for (const CharSetItem & i : cc1->mSparseCharSet) {
        diff->insert_range(i.lo_codepoint, i.hi_codepoint);
    }
    for (const CharSetItem & i : cc2->mSparseCharSet) {
        diff->remove_range(i.lo_codepoint, i.hi_codepoint);
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
        const CharSetItem & ra = *ai;
        const CharSetItem & rb = *bi;
        if (ra.hi_codepoint < rb.lo_codepoint) {
            ++ai;
            continue;
        }
        else if (rb.hi_codepoint < ra.lo_codepoint) {
            ++bi;
            continue;
        }
        isect->insert_range(std::max(ra.lo_codepoint, rb.lo_codepoint), std::min(ra.hi_codepoint, rb.hi_codepoint));
        if (ra.hi_codepoint < rb.hi_codepoint) ++ai; 
        else ++bi;
    }
    return isect;
}
    
CC * caseInsensitize(const CC * cc) {
    CC * cci = makeCC();
    for (auto i = cc->cbegin(); i != cc->cend(); i++) {
        caseInsensitiveInsertRange(cci, i->lo_codepoint, i->hi_codepoint);
    }
    return cci;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief rangeIntersect
 * @param cc
 * @param lo
 * @param hi
 ** ------------------------------------------------------------------------------------------------------------- */
CC * rangeIntersect(const CC * cc, const CodePointType lo, const CodePointType hi) {
    assert ("cc cannot be null" && cc);
    CC * intersect = makeCC();
    for (const auto & p : *cc) {
        if ((p.lo_codepoint <= hi) && (p.hi_codepoint >= lo)) {
            intersect->insert_range(std::max(lo, p.lo_codepoint), std::min(hi, p.hi_codepoint));
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
CC * rangeGaps(const CC * cc, const CodePointType lo, const CodePointType hi) {
    assert ("cc cannot be null" && cc);
    CC * gaps = makeCC();
    CodePointType cp = lo;
    if (cp < hi) {
        auto i = cc->cbegin(), end = cc->cend();
        for (; i != end && cp < hi; ++i) {
            if (i->hi_codepoint < cp) {
                continue;
            }
            else if (i->lo_codepoint > cp) {
                gaps->insert_range(cp, i->lo_codepoint - 1);
            }
            cp = i->hi_codepoint + 1;
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
        if (j->hi_codepoint > i->hi_codepoint) {
            ranges->insert_range(i->lo_codepoint, i->hi_codepoint);
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
        if (j->hi_codepoint <= i->hi_codepoint) {
            ranges->insert_range(j->lo_codepoint, j->hi_codepoint);
        }
        else {
            i = j;
        }
    }
    return ranges;
}
    
}
