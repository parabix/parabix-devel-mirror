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
  
unsigned PabloBlockCarryData::enumerate(PabloBlock & blk) {
    for (Statement * stmt : blk) {
        if (Advance * adv = dyn_cast<Advance>(stmt)) {
            unsigned shift_amount = adv->getAdvanceAmount();
            if (shift_amount == 1) {
                adv->setLocalAdvanceIndex(unitAdvances);
                unitAdvances++;                
            }
            else if (shift_amount < LongAdvanceBase) {
                //EnsurePackHasSpace(shortAdvanceTotal, shift_amount);
                //adv->setLocalAdvanceIndex(shortAdvanceTotal);
                adv->setLocalAdvanceIndex(shortAdvances);
                shortAdvances++;
                shortAdvanceTotal += shift_amount;
            }
            else {
                adv->setLocalAdvanceIndex(longAdvanceTotalBlocks);
                longAdvances++;
                longAdvanceTotalBlocks += longAdvanceBufferSize(shift_amount);
            }
        }
        else if (MatchStar * m = dyn_cast<MatchStar>(stmt)) {
            m->setLocalCarryIndex(localCarries);
            ++localCarries;
        }
        else if (ScanThru * s = dyn_cast<ScanThru>(stmt)) {
            s->setLocalCarryIndex(localCarries);
            ++localCarries;
        }
    }
    unsigned localCarryDataIndex = localCarries + unitAdvances + shortAdvances + longAdvanceTotalBlocks;
    localCarryDataSize = localCarryDataIndex;
    /*
    totalCarryDataSize = longAdvanceTotalBlocks * BLOCK_SIZE; 
    totalCarryDataSize += shortAdvanceTotal;
    EnsurePackHasSpace(totalCarryDataSize, localCarries);
    totalCarryDataSize += localCarries;
    EnsurePackHasSpace(totalCarryDataSize, unitAdvances);
    totalCarryDataSize += unitAdvances;
     */
    
    for (Statement * stmt : blk) {
        if (If * ifStatement = dyn_cast<If>(stmt)) {
            PabloBlockCarryData & nestedBlockData = ifStatement->getBody().carryData;
            nestedBlockData.setIfDepth(ifDepth + 1);
            nestedBlockData.setBlockCarryDataIndex(blockCarryDataIndex + localCarryDataIndex);
            const unsigned ifCarryDataSize = nestedBlockData.enumerate(ifStatement->getBody());
            nestedBlockCount++;
            //EnsurePackHasSpace(totalCarryDataSize, ifCarryDataSize);
            nestedCarryDataSize += ifCarryDataSize;
            localCarryDataIndex += ifCarryDataSize;            
        }
        else if (While * whileStatement = dyn_cast<While>(stmt)) {
            PabloBlockCarryData & nestedBlockData = whileStatement->getBody().carryData;
            nestedBlockData.setWhileDepth(whileDepth + 1);
            nestedBlockData.setBlockCarryDataIndex(blockCarryDataIndex + localCarryDataIndex);
            unsigned whileCarryDataSize = nestedBlockData.enumerate(whileStatement->getBody());
            //if (whileStatement->isMultiCarry()) whileCarryDataSize *= whileStatement->getMaxIterations();
            nestedBlockCount++;
            //EnsurePackHasSpace(totalCarryDataSize, whileCarryDataSize);
            nestedCarryDataSize += whileCarryDataSize;
            localCarryDataIndex += whileCarryDataSize;
        }
    }
    totalCarryDataSize = localCarryDataIndex;
    if ((ifDepth > 0) && (totalCarryDataSize > 1)) {
        // Need extra space for the summary variable, always the last
        // entry within an if block.
        totalCarryDataSize += 1;
    }
    longAdvanceOffset = 0;
    /*
    if (totalCarryDataSize > CarryPackSize) {
        // Need extra space for the summary variable, always the first
        // entry within an if block.
        totalCarryDataSize += BLOCK_SIZE;
        longAdvanceOffset = BLOCK_SIZE;
    }
     */
    carryOffset = longAdvanceOffset + longAdvanceTotalBlocks;
    unitAdvanceOffset = carryOffset + localCarries;
    shortAdvanceOffset = unitAdvanceOffset + unitAdvances;

//     std::cerr << "blockCarryDataIndex = " << blockCarryDataIndex << " nestedBlockCount = " << nestedBlockCount << std::endl;
//     std::cerr << "longAdvanceOffset = " << longAdvanceOffset << " carryOffset = " << carryOffset << std::endl;
//     std::cerr << "unitAdvanceOffset = " << unitAdvanceOffset << " shortAdvanceOffset = " << shortAdvanceOffset << std::endl;
//     std::cerr << "ifDepth = " << ifDepth << " whileDepth = " << whileDepth << std::endl;
//     std::cerr << "totalCarryDataSize = " << totalCarryDataSize << std::endl;
    return totalCarryDataSize;
}

}
