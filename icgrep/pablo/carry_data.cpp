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

   

void PabloBlockCarryData::enumerateLocal() {
    for (Statement * stmt : *theScope) {
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
    shortAdvance.frameOffsetinBits = longAdvance.frameOffsetinBits + longAdvance.allocatedBitBlocks * BLOCK_SIZE;
    addWithCarry.frameOffsetinBits = shortAdvance.frameOffsetinBits + shortAdvance.entries * BLOCK_SIZE;
    advance1.frameOffsetinBits = addWithCarry.frameOffsetinBits + addWithCarry.entries * BLOCK_SIZE;
    nested.frameOffsetinBits = advance1.frameOffsetinBits + advance1.entries * BLOCK_SIZE;
#endif
}
        
void PabloBlockCarryData::dumpCarryData(llvm::raw_ostream & strm) {
    unsigned totalDepth = ifDepth + whileDepth;
    for (int i = 0; i < totalDepth; i++) strm << "  ";
    strm << "scope index = " << theScope->getScopeIndex();
    strm << " framePosition: " << framePosition << ", ifDepth: " << ifDepth << ", whileDepth:" << whileDepth << ", maxNestingDepth: " << maxNestingDepth << "\n";
    for (int i = 0; i < totalDepth; i++) strm << "  ";
    strm << "longAdvance: offset = " << longAdvance.frameOffsetinBits << ", entries = " << longAdvance.entries << "\n";
    for (int i = 0; i < totalDepth; i++) strm << "  ";
    strm << "shortAdvance: offset = " << shortAdvance.frameOffsetinBits << ", entries = " << shortAdvance.entries << "\n";
    for (int i = 0; i < totalDepth; i++) strm << "  ";
    strm << "advance1: offset = " << advance1.frameOffsetinBits << ", entries = " << advance1.entries << "\n";
    for (int i = 0; i < totalDepth; i++) strm << "  ";
    strm << "addWithCarry: offset = " << addWithCarry.frameOffsetinBits << ", entries = " << addWithCarry.entries << "\n";
    for (int i = 0; i < totalDepth; i++) strm << "  ";
    strm << "nested: offset = " << nested.frameOffsetinBits << ", allocatedBits = " << nested.allocatedBits << "\n";
    for (int i = 0; i < totalDepth; i++) strm << "  ";
    strm << "summary: offset = " << summary.frameOffsetinBits << ", allocatedBits = " << summary.allocatedBits << "\n";
    for (int i = 0; i < totalDepth; i++) strm << "  ";
    strm << "scopeCarryDataBits = " << scopeCarryDataBits  << "\n";
    strm.flush();
    
}

}
