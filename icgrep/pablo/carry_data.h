/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CARRY_DATA_H
#define CARRY_DATA_H
#include <include/simd-lib/bitblock.hpp>
#include <stdexcept>
/* 
 * Carry Data system.
 * 
 * Each PabloBlock (Main, If, While) has a contiguous data area for carry information.
 * The data area may be at a fixed or variable base offset from the base of the 
 * main function carry data area.
 * The data area for each block consists of contiguous space for the local carries and 
 * advances of the block plus the areas of any ifs/whiles nested within the block.

*/
unsigned const LongAdvanceBase = 64;

static unsigned power2ceil (unsigned v) {
    unsigned ceil = 1;
    while (ceil < v) ceil *= 2;
    return ceil;
}

namespace pablo {

class PabloBlock;

class PabloBlockCarryData {
public:
    PabloBlockCarryData(): framePosition(0), 
                           ifDepth(0), whileDepth (0), maxNestingDepth(0),
                           longAdvance({0, 0, 0}),
                           shortAdvance({0, 0, 0}),
                           advance1({0, 0}),
                           addWithCarry({0, 0}),
                           nested({0, 0, 0}),
                           summary({0, 0}),
                           totalCarryDataBits(0)
                           {}
        
    unsigned enumerate(PabloBlock & p);
    
    unsigned getBlockCarryDataIndex()  const {
        return framePosition/BLOCK_SIZE;
    }
    
    unsigned getIfDepth()  const {
        return ifDepth;
    }
    
    unsigned getWhileDepth()  const {
        return whileDepth;
    }
        
    unsigned longAdvanceCarryDataOffset(unsigned advanceIndex)  const {
        return longAdvance.frameOffsetinBits / BLOCK_SIZE + advanceIndex;
    }
    
    unsigned longAdvanceEntries(unsigned shift_amount) const {
        return (shift_amount + BLOCK_SIZE - 1)/BLOCK_SIZE;
    }
    
    unsigned longAdvanceBufferSize(unsigned shift_amount)  const {
        return power2ceil(longAdvanceEntries(shift_amount));
    }
    
    bool blockHasLongAdvances() const { return longAdvance.entries > 0;}
    
    unsigned shortAdvanceCarryDataOffset(unsigned advanceIndex)  const {
        return shortAdvance.frameOffsetinBits / BLOCK_SIZE + advanceIndex;
    }
    
    unsigned unitAdvanceCarryDataOffset(unsigned advanceIndex)  const {
        return advance1.frameOffsetinBits / BLOCK_SIZE + advanceIndex;
    }
    
    unsigned carryOpCarryDataOffset(unsigned idx)  const {
        return addWithCarry.frameOffsetinBits / BLOCK_SIZE + idx;
    }
    
    unsigned summaryCarryDataIndex()  const {
        return summary.frameOffsetinBits / BLOCK_SIZE;
    }
    
    unsigned getLocalCarryDataSize () { return nested.frameOffsetinBits / BLOCK_SIZE; }

    unsigned getTotalCarryDataSize () { return totalCarryDataBits / BLOCK_SIZE; }
   
    bool blockHasCarries() const { return totalCarryDataBits > 0;}
    
    bool explicitSummaryRequired() const { return totalCarryDataBits > BLOCK_SIZE;}
    
    bool summaryNeededInParentBlock() const {return (ifDepth > 0) && blockHasCarries();}
    
private:
    
    unsigned framePosition;
    
    unsigned ifDepth;
    unsigned whileDepth;
    unsigned maxNestingDepth;    
    
    struct {unsigned frameOffsetinBits; unsigned entries; unsigned allocatedBitBlocks;} longAdvance;
    struct {unsigned frameOffsetinBits; unsigned entries; unsigned allocatedBits;} shortAdvance;
    struct {unsigned frameOffsetinBits; unsigned entries;} advance1;
    struct {unsigned frameOffsetinBits; unsigned entries;} addWithCarry;
    struct {unsigned frameOffsetinBits; unsigned entries; unsigned allocatedBits;} nested;
    struct {unsigned frameOffsetinBits; unsigned allocatedBits;} summary;

    unsigned totalCarryDataBits;
    
};


}


#endif // CARRY_DATA_H
