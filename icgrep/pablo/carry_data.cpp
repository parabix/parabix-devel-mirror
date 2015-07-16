/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */


#include <include/simd-lib/bitblock.hpp>
#include <pablo/pablo_compiler.h>
#include <pablo/codegenstate.h>
#include <pablo/carry_data.h>
#include <iostream>

namespace pablo {
  
    
const unsigned bitsPerPack = 64;


void AlignUpwards(unsigned & toAlign, unsigned alignment) {
    if ((toAlign & (alignment - 1)) != 0) {
        toAlign = (toAlign + alignment) & (alignment - 1);
    }
}

void EnsurePackHasSpace(unsigned & packedTotalBits, unsigned addedBits) {
    unsigned bitsInCurrentPack = packedTotalBits % bitsPerPack;
    if (bitsInCurrentPack + addedBits > bitsPerPack) {
        AlignUpwards(packedTotalBits, bitsPerPack);
    }
}
    
    
unsigned PabloBlockCarryData::enumerate(PabloBlock & blk) {
    for (Statement * stmt : blk) {
        if (Advance * adv = dyn_cast<Advance>(stmt)) {
            unsigned shift_amount = adv->getAdvanceAmount();
            if (shift_amount == 1) {
                adv->setLocalAdvanceIndex(advance1.entries);
                advance1.entries++;                
            }
            else if (shift_amount < LongAdvanceBase) {
#ifdef PACKING
                EnsurePackHasSpace(shortAdvance.allocatedBits, shift_amount);
                adv->setLocalAdvanceIndex(shortAdvance.allocatedBits);
#else
                adv->setLocalAdvanceIndex(shortAdvance.entries);
#endif
                shortAdvance.entries++;
                shortAdvance.allocatedBits += shift_amount;
            }
            else {
                adv->setLocalAdvanceIndex(longAdvance.allocatedBitBlocks);
                longAdvance.entries++;
                longAdvance.allocatedBitBlocks += longAdvanceBufferSize(shift_amount);
            }
        }
        else if (MatchStar * m = dyn_cast<MatchStar>(stmt)) {
            m->setLocalCarryIndex(addWithCarry.entries);
            ++addWithCarry.entries;
        }
        else if (ScanThru * s = dyn_cast<ScanThru>(stmt)) {
            s->setLocalCarryIndex(addWithCarry.entries);
            ++addWithCarry.entries;
        }
    }
    longAdvance.frameOffsetinBits = 0;
#ifdef PACKING
    shortAdvance.frameOffsetinBits = longAdvance.frameOffsetinBits + longAdvance.allocatedBitBlocks * BLOCK_SIZE;
    addWithCarry.frameOffsetinBits = shortAdvance.frameOffsetinBits + shortAdvance.allocatedBits;
    EnsurePackHasSpace(addWithCarry.frameOffsetinBits, addWithCarry.entries);
    advance1.frameOffsetinBits = addWithCarry.frameOffsetinBits + addWithCarry.entries;
    EnsurePackHasSpace(advance1.frameOffsetinBits, advance1.entries);
    nested.frameOffsetinBits = advance1.frameOffsetinBits + advance1.entries;
#else
    addWithCarry.frameOffsetinBits = longAdvance.frameOffsetinBits + longAdvance.allocatedBitBlocks * BLOCK_SIZE;
    advance1.frameOffsetinBits = addWithCarry.frameOffsetinBits + addWithCarry.entries * BLOCK_SIZE;
    shortAdvance.frameOffsetinBits = advance1.frameOffsetinBits + advance1.entries * BLOCK_SIZE;
    nested.frameOffsetinBits = shortAdvance.frameOffsetinBits + shortAdvance.entries * BLOCK_SIZE;
#endif
    unsigned nestedframePosition = nested.frameOffsetinBits;
    
    for (Statement * stmt : blk) {
        if (If * ifStatement = dyn_cast<If>(stmt)) {
            PabloBlockCarryData & nestedBlockData = ifStatement->getBody().carryData;
            nestedBlockData.ifDepth = ifDepth + 1;
            nestedBlockData.whileDepth = whileDepth;
            const unsigned ifCarryDataBits = nestedBlockData.enumerate(ifStatement->getBody());
#ifdef PACKING
            EnsurePackHasSpace(nestedframePosition, ifCarryDataBits);
#endif
            nestedBlockData.framePosition = nestedframePosition;
            nestedframePosition += ifCarryDataBits;
            if (maxNestingDepth <= nestedBlockData.maxNestingDepth) maxNestingDepth = nestedBlockData.maxNestingDepth + 1;
            nested.entries++;
        }
        else if (While * whileStatement = dyn_cast<While>(stmt)) {
            PabloBlockCarryData & nestedBlockData = whileStatement->getBody().carryData;
            nestedBlockData.ifDepth = ifDepth;
            nestedBlockData.whileDepth = whileDepth + 1;
            unsigned whileCarryDataBits = nestedBlockData.enumerate(whileStatement->getBody());
            //if (whileStatement->isMultiCarry()) whileCarryDataBits *= whileStatement->getMaxIterations();
#ifdef PACKING
            EnsurePackHasSpace(nestedframePosition, whileCarryDataBits);
#endif
            nestedBlockData.framePosition = nestedframePosition;
            nestedframePosition += whileCarryDataBits;
            if (maxNestingDepth <= nestedBlockData.maxNestingDepth) maxNestingDepth = nestedBlockData.maxNestingDepth + 1;
            nested.entries++;
        }
    }
    
    
    scopeCarryDataBits = nestedframePosition;
    
    if (explicitSummaryRequired()) {
        // Need extra space for the summary variable, always the last
        // entry within an if block.
        AlignUpwards(scopeCarryDataBits, BLOCK_SIZE);
        summary.frameOffsetinBits = scopeCarryDataBits;
        summary.allocatedBits = BLOCK_SIZE;
        scopeCarryDataBits += BLOCK_SIZE;
    }
    else {
        summary.frameOffsetinBits = 0;
        summary.allocatedBits = scopeCarryDataBits;
    }
    return scopeCarryDataBits;
}

}
