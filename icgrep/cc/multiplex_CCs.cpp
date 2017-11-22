/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <map>
#include <UCD/unicode_set.h>
#include <re/re_cc.h>
#include "boost/dynamic_bitset.hpp"

//
// Breakpoints of a set of character classes (CCs): each codepoint c such that
// there is some CC in CCs such that either (a) c is in the CC and c-1 is not, or
// (b) c-1 is in the CC and c is not.  Boundary cases: if codepoint 0 is in
// some CC, then 0 is a breakpoint (codepoint -1 is not in any CC).  If codepoint
// 0x10FFFF is in some CC then 0x110000 is a breakpoint.
//
// The breakpoints may be determined by iterating through the interval
// representation of each CC.   For each interval (lo, hi), lo and hi+1 
// are breakpoints. 
//
// For each breakpoint, a bitset is computed identifying the source CCs whose
// status changes at the breakpoint.
//

std::map<UCD::codepoint_t, boost::dynamic_bitset<>> computeBreakpoints(std::vector<UCD::UnicodeSet> CCs) {
    std::map<UCD::codepoint_t, boost::dynamic_bitset<>> breakpoints;
    for (unsigned i = 0; i < CCs.size(); i++) {
        for (const UCD::UnicodeSet::interval_t & range : CCs[i]) {
            auto lo = re::lo_codepoint(range);
            auto hi = re::hi_codepoint(range);
            auto f = breakpoints.find(lo);
            if (f == breakpoints.end()) {
                breakpoints.emplace(lo, boost::dynamic_bitset<>(CCs.size()));
            }
            breakpoints[lo].set(i);
            f = breakpoints.find(hi + 1);
            if (f == breakpoints.end()) {
                breakpoints.emplace(hi+1, boost::dynamic_bitset<>(CCs.size()));
            }
            breakpoints[hi+1].set(i);
        }
    }
    return breakpoints;
}


void doMultiplexCCs(const std::vector<UCD::UnicodeSet> & CCs,
                    std::vector<std::vector<unsigned>> & exclusiveSetIDs,
                    std::vector<UCD::UnicodeSet> & multiplexedCCs) {
    
    std::map<UCD::codepoint_t, boost::dynamic_bitset<>> breakpoints = computeBreakpoints(CCs);
    // Initialize the exclusiveSetIDs to have one empty vector per source CC.
    exclusiveSetIDs.clear();
    exclusiveSetIDs.resize(CCs.size());
    //
    // Exclusive set determination.   
    //
    // Set up a map from the set of source CCs for each exclusive set to the exclusive set index.

    std::map<boost::dynamic_bitset<>, unsigned> CC_set_to_exclusive_set_map;

    // Entry 0 is for the characters not in any of the CCs.
    CC_set_to_exclusive_set_map.emplace(boost::dynamic_bitset<>(CCs.size()), 0);

    unsigned current_exclusive_set_idx = 0;
    unsigned multiplexed_bit_count = 0;
    boost::dynamic_bitset<> current_set(CCs.size());
    
    unsigned range_lo = 0;
    unsigned next_set_index = 1;
    for (auto & bkpt_entry : breakpoints) {
        if (current_exclusive_set_idx > 0) {  // We have a range entry to close for a pending exclusive set.
            unsigned range_hi = bkpt_entry.first - 1;
            for (unsigned bit = 0; bit < multiplexed_bit_count; bit++) {
                if (((current_exclusive_set_idx >> bit) & 1) == 1) {
                    multiplexedCCs[bit].insert_range(range_lo, range_hi);
                }
            }
        }
        // Start a new range.
        range_lo = bkpt_entry.first;
        if (range_lo > UCD::UNICODE_MAX) continue; // Nothing to do for bkpt 0x110000
        current_set ^= bkpt_entry.second;
        auto idx_iter = CC_set_to_exclusive_set_map.find(current_set);
        if (idx_iter == CC_set_to_exclusive_set_map.end()) {
            // New exclusive class; assign the next sequential integer.
            //current_exclusive_set_idx = exclusiveSetIDs.size();
            current_exclusive_set_idx = next_set_index;
            next_set_index++;
            CC_set_to_exclusive_set_map.emplace(current_set, current_exclusive_set_idx);
            
            for (unsigned CC1 = current_set.find_first(); CC1 < CCs.size(); CC1 = current_set.find_next(CC1)) {
                exclusiveSetIDs[CC1].push_back(current_exclusive_set_idx);
            }
            if ((current_exclusive_set_idx & (current_exclusive_set_idx - 1)) == 0) {
                multiplexed_bit_count++;
                multiplexedCCs.push_back(UCD::UnicodeSet());
            }
        }
        else {
            current_exclusive_set_idx = idx_iter->second;
        }
    }
}
