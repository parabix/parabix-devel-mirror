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

//#define PACKING

#ifdef PACKING
const unsigned PACK_SIZE = 64;
#else
const unsigned PACK_SIZE = BLOCK_SIZE;
#endif


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

#ifdef PACKING
static void EnsurePackHasSpace(unsigned & packedTotalBits, unsigned addedBits) {
    unsigned bitsInCurrentPack = packedTotalBits % PACK_SIZE;
    if ((bitsInCurrentPack > 0) && (bitsInCurrentPack + addedBits > PACK_SIZE)) {
        packedTotalBits = alignCeiling(packedTotalBits, PACK_SIZE);
    }
}
#endif


namespace pablo {

class PabloBlock;

class PabloBlockCarryData {
public:
    PabloBlockCarryData(PabloBlock * b):
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
#ifdef PACKING
        return shortAdvance.frameOffset / PACK_SIZE;
#else
        return shortAdvance.frameOffset;
#endif
    }

    unsigned getLocalCarryPackCount () { 
#ifdef PACKING
        return fullOrPartialBlocks(nested.frameOffset, PACK_SIZE) - shortAdvance.frameOffset / PACK_SIZE;
#else
        return nested.frameOffset - shortAdvance.frameOffset;
#endif
    }
    
    unsigned getScopeCarryPackCount () { 
#ifdef PACKING
        return fullOrPartialBlocks(scopeCarryDataSize, PACK_SIZE);
#else
        return scopeCarryDataSize;
#endif
    }
    
    bool blockHasCarries() const { return scopeCarryDataSize > 0;}
    
    bool explicitSummaryRequired() const { 
#ifdef PACKING
        return (ifDepth > 0) && (scopeCarryDataSize > PACK_SIZE);
#else
        return (ifDepth > 0) && (scopeCarryDataSize > 1);
#endif
    }
    
protected:
    
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
