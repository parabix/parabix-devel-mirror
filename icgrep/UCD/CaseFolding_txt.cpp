/*
 *  Copyright (c) 2014 International Characters, Inc.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters, Inc.
 *
 */

#include "CaseFolding_txt.h"
#include <algorithm>

using namespace re;

int findFoldEntry(const codepoint_t cp) {
    int lo = 0;
    int hi = foldTableSize;
    while (hi - lo > 1) {
        int mid = (lo + hi)/2;
        if (cp < foldTable[mid].range_lo) {
            hi = mid;
        }
        else {
            lo = mid;
        }
    }
    return lo;
}

void caseInsensitiveInsertRange(CC * cc, const codepoint_t lo, const codepoint_t hi) {
    cc->insert_range(lo, hi);
    // Find the first foldTable entry overlapping the (lo, hi) range.
    int e = findFoldEntry(lo);
    // There may be multiple consecutive entries overlapping the range.
    // Keep processing until we are done.
    while (foldTable[e].range_lo <= hi) {
        const FoldEntry & fe = foldTable[e];
        const FoldEntry & fnext = foldTable[e + 1];
        // Constrain (lo, hi) to this entry only.
        codepoint_t lo1 = std::max(lo, fe.range_lo);
        codepoint_t hi1 = std::min(hi, fnext.range_lo - 1);
        if (fe.fold_offset > 0 && fe.range_lo + fe.fold_offset < fnext.range_lo) {
            //
            // There are more than fold_offset values in the range, meaning that
            // we have an extended range with alternating subranges of positive
            // and negative offsets.
            // First find the negative offset subrange.
            codepoint_t subrange_lo = lo1 - ((lo1 - fe.range_lo) % (2 * fe.fold_offset));
            codepoint_t negative_subrange_lo = subrange_lo + fe.fold_offset;
            codepoint_t negative_subrange_hi = subrange_lo + 2 * fe.fold_offset - 1;
            if ((lo1 <= negative_subrange_hi) && (hi1 >= negative_subrange_lo)) {
                // negative offsets apply
                cc->insert_range(std::max(negative_subrange_lo,lo1) - fe.fold_offset, std::min(negative_subrange_hi, hi1) - fe.fold_offset);
            }
            // Now the positive offset subrange.
            codepoint_t positive_subrange_lo = hi1 - ((hi1 - fe.range_lo) % (2 * fe.fold_offset));
            codepoint_t positive_subrange_hi = positive_subrange_lo + fe.fold_offset - 1;
            if ((lo1 <= positive_subrange_hi) && (hi1 >= positive_subrange_lo)) {
                cc->insert_range(std::max(positive_subrange_lo, lo1) + fe.fold_offset, std::min(positive_subrange_hi, hi1) + fe.fold_offset);
            }
        }
        else if (fe.fold_offset != 0) {
            // We have either a positive or negative offset, and all offsets for
            // this entry have the same sign.
            cc->insert_range(lo1 + fe.fold_offset, hi1 + fe.fold_offset);
        }
        // Now pick up any individual fold entries.
        for (unsigned i = 0; i < fe.fold_pairs.size(); i++) {
            if (fe.fold_pairs[i].first < lo) continue;  // Only possible for first fold_entry.
            if (fe.fold_pairs[i].first > hi) break;     // Only possible for last fold_entry.
            cc->insert(fe.fold_pairs[i].second);
        }
        // Move on to the next fold_entry.
        e++;
    }
}


