/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/pablo_compiler.h>
#include <pablo/codegenstate.h>
#include <pablo/carry_data.h>
#include <iostream>

namespace pablo {

void CarryData::enumerateLocal() {
    for (Statement * stmt : *theScope) {
        if (Advance * adv = dyn_cast<Advance>(stmt)) {
            unsigned shift_amount = adv->getAmount();
            if (shift_amount == 1) {
                adv->setLocalIndex(unitAdvance.entries);
                unitAdvance.entries++;                
            }
            else if (shift_amount < LongAdvanceBase) {
                // short Advance
                if (mItemsPerPack >= LongAdvanceBase) {
                    // Packing is possible.   We will use the allocated bit position as
                    // the index.
                    if (roomInFinalPack(shortAdvance.allocatedBits) < shift_amount) {
                        // Start a new pack.
                        shortAdvance.allocatedBits = alignCeiling(shortAdvance.allocatedBits, mPackSize);
                    }
                    adv->setLocalIndex(shortAdvance.allocatedBits);
                }
                else {
                    adv->setLocalIndex(shortAdvance.entries);
                }
                shortAdvance.entries++;
                shortAdvance.allocatedBits += shift_amount;
            }
            else {
                adv->setLocalIndex(longAdvance.allocatedBitBlocks);
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
    longAdvance.frameOffset = 0;
    shortAdvance.frameOffset = longAdvance.frameOffset + longAdvance.allocatedBitBlocks * mPositionsPerBlock;
    if (mItemsPerPack == mPackSize) {
        addWithCarry.frameOffset = shortAdvance.frameOffset + shortAdvance.allocatedBits;
        if (roomInFinalPack(addWithCarry.frameOffset) < addWithCarry.entries) {
            addWithCarry.frameOffset = alignCeiling(addWithCarry.frameOffset, mPackSize);
        }
        unitAdvance.frameOffset = addWithCarry.frameOffset + addWithCarry.entries;
        if (roomInFinalPack(unitAdvance.frameOffset) < unitAdvance.entries) {
            unitAdvance.frameOffset = alignCeiling(unitAdvance.frameOffset, mPackSize);
        }
    }
    else {
        addWithCarry.frameOffset = shortAdvance.frameOffset + shortAdvance.entries;
        unitAdvance.frameOffset = addWithCarry.frameOffset + addWithCarry.entries;

    }
    nested.frameOffset = unitAdvance.frameOffset + unitAdvance.entries;
}
        
void CarryData::dumpCarryData(llvm::raw_ostream & strm) {
    unsigned totalDepth = ifDepth + whileDepth;
    for (unsigned i = 0; i < totalDepth; i++) strm << "  ";
    strm << "scope index = " << theScope->getScopeIndex();
    strm << " framePosition: " << framePosition << ", ifDepth: " << ifDepth << ", whileDepth:" << whileDepth << ", maxNestingDepth: " << maxNestingDepth << "\n";
    for (unsigned i = 0; i < totalDepth; i++) strm << "  ";
    strm << "longAdvance: offset = " << longAdvance.frameOffset << ", entries = " << longAdvance.entries << "\n";
    for (unsigned i = 0; i < totalDepth; i++) strm << "  ";
    strm << "shortAdvance: offset = " << shortAdvance.frameOffset << ", entries = " << shortAdvance.entries << "\n";
    for (unsigned i = 0; i < totalDepth; i++) strm << "  ";
    strm << "advance1: offset = " << unitAdvance.frameOffset << ", entries = " << unitAdvance.entries << "\n";
    for (unsigned i = 0; i < totalDepth; i++) strm << "  ";
    strm << "addWithCarry: offset = " << addWithCarry.frameOffset << ", entries = " << addWithCarry.entries << "\n";
    for (unsigned i = 0; i < totalDepth; i++) strm << "  ";
    strm << "nested: offset = " << nested.frameOffset << ", allocatedBits = " << nested.allocatedBits << "\n";
    for (unsigned i = 0; i < totalDepth; i++) strm << "  ";
    strm << "summary: offset = " << summary.frameOffset << "\n";
    for (unsigned i = 0; i < totalDepth; i++) strm << "  ";
    strm << "scopeCarryDataSize = " << scopeCarryDataSize  << "\n";
    strm.flush();

}

}
