/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CARRY_DATA_H
#define CARRY_DATA_H
#include <include/simd-lib/bitblock.hpp>
#include <pablo/codegenstate.h>
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
    PabloBlockCarryData(): blockCarryDataIndex(0),
                           ifDepth(0), whileDepth (0),
                           localCarries(0), unitAdvances(0),
                           shortAdvances(0), shortAdvanceTotal(0),
                           longAdvances(0), longAdvanceTotalBlocks(0),
                           nestedBlockCount(0), nestedCarryDataSize(0),
                           totalCarryDataSize(0), 
                           carryOffset(0), unitAdvanceOffset(0),
                           shortAdvanceOffset(0), longAdvanceOffset(0)
                           {}
    
    
    void setBlockCarryDataIndex (unsigned idx) {blockCarryDataIndex = idx;}
    
    void setIfDepth (unsigned depth) {ifDepth = depth;}
    
    void setWhileDepth (unsigned depth) {whileDepth = depth;}
    
    unsigned enumerate(PabloBlock & p);
    
    unsigned getBlockCarryDataIndex()  const {
        return blockCarryDataIndex;
    }
    
    unsigned getIfDepth()  const {
        return ifDepth;
    }
    
    unsigned getWhileDepth()  const {
        return whileDepth;
    }
    
    unsigned longAdvanceCarryDataOffset(unsigned advanceIndex)  const {
        return blockCarryDataIndex + longAdvanceOffset + advanceIndex;
    }
    
    unsigned longAdvanceEntries(unsigned shift_amount) const {
        return (shift_amount + BLOCK_SIZE - 1)/BLOCK_SIZE;
    }
    
    unsigned longAdvanceBufferSize(unsigned shift_amount)  const {
        return power2ceil(longAdvanceEntries(shift_amount));
    }
    
    unsigned shortAdvanceCarryDataOffset(unsigned advanceIndex)  const {
        return blockCarryDataIndex + shortAdvanceOffset + advanceIndex;
    }
    
    unsigned unitAdvanceCarryDataOffset(unsigned advanceIndex)  const {
        return blockCarryDataIndex + unitAdvanceOffset + advanceIndex;
    }
    
    unsigned carryOpCarryDataOffset(unsigned idx)  const {
        return blockCarryDataIndex + carryOffset + idx;
    }
    
    bool blockHasCarries() const { return totalCarryDataSize > 0;}
    
    bool explicitSummaryRequired() const { return totalCarryDataSize > 1;}
    
    bool summaryNeededInParentBlock() const {return (ifDepth > 0) && blockHasCarries();}
    
    unsigned summaryCarryDataIndex()  const {
        return blockCarryDataIndex + totalCarryDataSize - 1;
    }
    
    unsigned getTotalCarryDataSize()  const {
        return totalCarryDataSize;
    }
    
private:
    unsigned blockCarryDataIndex;
    unsigned ifDepth;
    unsigned whileDepth;
    unsigned localCarries;
    unsigned unitAdvances;
    unsigned shortAdvances;
    unsigned shortAdvanceTotal;
    unsigned longAdvances;
    unsigned longAdvanceTotalBlocks;
    unsigned nestedBlockCount;
    unsigned nestedCarryDataSize;
    unsigned totalCarryDataSize;
    unsigned carryOffset;
    unsigned unitAdvanceOffset;
    unsigned shortAdvanceOffset;
    unsigned longAdvanceOffset;
    
};


}


#endif // CARRY_DATA_H
