/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CARRY_DATA_H
#define CARRY_DATA_H
#include <include/simd-lib/bitblock.hpp>
#include <stdexcept>
#include <iostream>
#include <ostream>
#include <llvm/Support/raw_os_ostream.h>
#include <llvm/IR/Module.h>

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

static unsigned alignCeiling(unsigned toAlign, unsigned alignment) {
    return ((toAlign - 1) | (alignment - 1)) + 1;
}

static unsigned fullOrPartialBlocks(unsigned bits, unsigned block_size) {
    return alignCeiling(bits, block_size) / block_size;
}

namespace pablo {

class PabloBlock;

class PabloBlockCarryData {
public:
    PabloBlockCarryData(PabloBlock * b, unsigned PACK_SIZE, unsigned ITEMS_PER_PACK):
                           mPACK_SIZE(PACK_SIZE),
                           mITEMS_PER_PACK(ITEMS_PER_PACK),
                           mPOSITIONS_PER_BLOCK(mITEMS_PER_PACK * (BLOCK_SIZE/mPACK_SIZE)),
                           theScope(b), framePosition(0),
                           ifDepth(0), whileDepth (0), maxNestingDepth(0),
                           longAdvance({0, 0, 0}),
                           shortAdvance({0, 0, 0}),
                           advance1({0, 0}),
                           addWithCarry({0, 0}),
                           nested({0, 0, 0}),
                           summary({0, 0}),
                           scopeCarryDataSize(0)
    {enumerateLocal();}
        
    friend class CarryManager;
    
    inline unsigned roomInFinalPack(unsigned allocatedBits) {
        return mITEMS_PER_PACK - (allocatedBits % mITEMS_PER_PACK);
    }
    
    void enumerateLocal();
    void dumpCarryData(llvm::raw_ostream & strm);
    
    unsigned getFrameIndex()  const {
        return framePosition;
    }
    
    void setFramePosition(unsigned p) {
        framePosition = p;
    }
    
    unsigned getIfDepth()  const {
        return ifDepth;
    }
    
    void setIfDepth(unsigned d) {
        ifDepth = d;
    }
    
    unsigned getWhileDepth()  const {
        return whileDepth;
    }
        
    void setWhileDepth(unsigned d) {
        whileDepth = d;
    }
    
    unsigned longAdvanceEntries(unsigned shift_amount) const {
        return fullOrPartialBlocks(shift_amount, BLOCK_SIZE);
    }
    
    unsigned longAdvanceBufferSize(unsigned shift_amount)  const {
        return power2ceil(longAdvanceEntries(shift_amount));
    }
    
    bool blockHasLongAdvances() const { return longAdvance.entries > 0;}
    
    unsigned getLocalCarryPackIndex () { 
        return shortAdvance.frameOffset / mITEMS_PER_PACK;
    }

    unsigned getLocalCarryPackCount () {
        return fullOrPartialBlocks(nested.frameOffset, mITEMS_PER_PACK) - shortAdvance.frameOffset / mITEMS_PER_PACK;
    }
    
    unsigned getScopeCarryPackCount () { 
        return fullOrPartialBlocks(scopeCarryDataSize, mITEMS_PER_PACK);
    }
    
    bool blockHasCarries() const { return scopeCarryDataSize > 0;}
    
    bool explicitSummaryRequired() const { 
        return (ifDepth > 0) && (scopeCarryDataSize > mITEMS_PER_PACK);
    }
    
protected:
    
    unsigned mPACK_SIZE, mITEMS_PER_PACK, mPOSITIONS_PER_BLOCK;
    
    PabloBlock * theScope;
    
    unsigned framePosition;
    
    unsigned ifDepth;
    unsigned whileDepth;
    unsigned maxNestingDepth;    
    
    struct {unsigned frameOffset; unsigned entries; unsigned allocatedBitBlocks;} longAdvance;
    struct {unsigned frameOffset; unsigned entries; unsigned allocatedBits;} shortAdvance;
    struct {unsigned frameOffset; unsigned entries;} advance1;
    struct {unsigned frameOffset; unsigned entries;} addWithCarry;
    struct {unsigned frameOffset; unsigned entries; unsigned allocatedBits;} nested;
    struct {unsigned frameOffset; unsigned allocatedBits;} summary;

    unsigned scopeCarryDataSize;
    
    llvm::Value * ifEntryPack;
    
};


}


#endif // CARRY_DATA_H
