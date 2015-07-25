/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */


#include <include/simd-lib/bitblock.hpp>
#include <stdexcept>
#include <pablo/carry_data.h>
#include <pablo/codegenstate.h>
#include <pablo/carry_manager.h>
#include <pablo/pabloAST.h>
#include <iostream>

namespace pablo {
  
    unsigned doScopeCount(PabloBlock * pb) {
        unsigned count = 1;
        
        for (Statement * stmt : *pb) {
            if (If * ifStatement = dyn_cast<If>(stmt)) {
                count += doScopeCount(&ifStatement->getBody());
            }
            else if (While * whileStatement = dyn_cast<While>(stmt)) {
                count += doScopeCount(&whileStatement->getBody());
            }
        }
        return count;
       
    }

unsigned CarryManager::initialize(PabloBlock * pb, Value * carryPtr) {
    mPabloRoot = pb;
#ifdef PACKING
    mCarryPackType = mBuilder->getIntNTy(PACK_SIZE);
    mCarryPackBasePtr = mBuilder->CreateBitCast(carryPtr, Type::getInt64PtrTy(mBuilder->getContext()));
    mCarryBitBlockPtr = carryPtr;
    mZeroInitializer = mBuilder->getInt64(0);
    mOneInitializer = mBuilder->getInt64(-1);
#else
#define mCarryPackType mBitBlockType
    mCarryPackBasePtr = carryPtr;
#define mCarryBitBlockPtr mCarryPackBasePtr
#endif
    unsigned scopeCount = doScopeCount(pb);
    mCarryInfoVector.resize(scopeCount);
    
    unsigned totalCarryDataBits = enumerate(pb, 0, 0);
    
    mTotalCarryDataBitBlocks = (totalCarryDataBits + BLOCK_SIZE - 1)/BLOCK_SIZE; 
    // Carry Data area will have one extra bit block to store the block number.
    mBlockNoPtr = mBuilder->CreateBitCast(mBuilder->CreateGEP(carryPtr, mBuilder->getInt64(mTotalCarryDataBitBlocks)), Type::getInt64PtrTy(mBuilder->getContext()));
    mBlockNo = mBuilder->CreateLoad(mBlockNoPtr);
    mCarryPackPtr.resize(mTotalCarryDataBitBlocks);
    mCarryInPack.resize(mTotalCarryDataBitBlocks);
    mCarryOutPack.resize(mTotalCarryDataBitBlocks);
    for (auto i = 0; i < mTotalCarryDataBitBlocks; i++) mCarryInPack[i]=nullptr;
    
    /*  Set the current scope to PabloRoot */
    mCurrentScope = mPabloRoot;
    mCurrentFrameIndex = 0;
    mCarryInfo = mCarryInfoVector[0];

    return mTotalCarryDataBitBlocks + 1; // One extra element for the block no.
}
    
void CarryManager::generateBlockNoIncrement() {
    mBuilder->CreateStore(mBuilder->CreateAdd(mBlockNo, mBuilder->getInt64(1)), mBlockNoPtr);
}

Value * CarryManager::getBlockNoPtr() {
    return mBlockNoPtr;
}


unsigned CarryManager::enumerate(PabloBlock * blk, unsigned ifDepth, unsigned whileDepth) {
    llvm::raw_os_ostream cerr(std::cerr);
    unsigned idx = blk->getScopeIndex();
    PabloBlockCarryData * cd = new PabloBlockCarryData(blk);
    mCarryInfoVector[idx] = cd;

    cd->setIfDepth(ifDepth);
    cd->setWhileDepth(whileDepth);
    unsigned nestedOffset = cd->nested.frameOffsetinBits;
  
    for (Statement * stmt : *blk) {
        if (If * ifStatement = dyn_cast<If>(stmt)) {
            const unsigned ifCarryDataBits = enumerate(&ifStatement->getBody(), ifDepth+1, whileDepth);
            PabloBlockCarryData * nestedBlockData = mCarryInfoVector[ifStatement->getBody().getScopeIndex()];
#ifdef PACKING
            EnsurePackHasSpace(nestedOffset, ifCarryDataBits);
#endif
            nestedBlockData->setFramePosition(nestedOffset);

            nestedOffset += ifCarryDataBits;
            if (cd->maxNestingDepth <= nestedBlockData->maxNestingDepth) cd->maxNestingDepth = nestedBlockData->maxNestingDepth + 1;
            cd->nested.entries++;
#ifndef NDEBUG
            nestedBlockData->dumpCarryData(cerr);
#endif
        }
        else if (While * whileStatement = dyn_cast<While>(stmt)) {
            const unsigned whileCarryDataBits = enumerate(&whileStatement->getBody(), ifDepth, whileDepth+1);
            PabloBlockCarryData * nestedBlockData = mCarryInfoVector[whileStatement->getBody().getScopeIndex()];
            //if (whileStatement->isMultiCarry()) whileCarryDataBits *= whileStatement->getMaxIterations();
#ifdef PACKING
            EnsurePackHasSpace(nestedOffset, whileCarryDataBits);
#endif
            nestedBlockData->setFramePosition(nestedOffset);
            nestedOffset += whileCarryDataBits;
            if (cd->maxNestingDepth <= nestedBlockData->maxNestingDepth) cd->maxNestingDepth = nestedBlockData->maxNestingDepth + 1;
            cd->nested.entries++;
#ifndef NDEBUG
            nestedBlockData->dumpCarryData(cerr);
#endif
        }
    }
    
    cd->scopeCarryDataBits = nestedOffset;
    
    if (cd->explicitSummaryRequired()) {
        // Need extra space for the summary variable, always the last
        // entry within an if block.
        cd->scopeCarryDataBits = alignCeiling(cd->scopeCarryDataBits, PACK_SIZE);
        cd->summary.frameOffsetinBits = cd->scopeCarryDataBits;
        cd->summary.allocatedBits = PACK_SIZE;
        cd->scopeCarryDataBits += PACK_SIZE;
    }
    else {
        cd->summary.frameOffsetinBits = 0;
        cd->summary.allocatedBits = cd->scopeCarryDataBits;
    }
    return cd->scopeCarryDataBits;
}


/* Entering and leaving blocks. */

void CarryManager::enterScope(PabloBlock * blk) {
    
    mCurrentScope = blk;
    mCarryInfo = mCarryInfoVector[blk->getScopeIndex()];
    mCurrentFrameIndex += mCarryInfo->getFrameIndex();
    //std::cerr << "enterScope:  mCurrentFrameIndex = " << mCurrentFrameIndex << std::endl;
}

void CarryManager::leaveScope() {
    mCurrentFrameIndex -= mCarryInfo->getFrameIndex();
    mCurrentScope = mCurrentScope->getParent();
    mCarryInfo = mCarryInfoVector[mCurrentScope->getScopeIndex()];
    //std::cerr << "leaveScope:  mCurrentFrameIndex = " << mCurrentFrameIndex << std::endl;
}


/* Helper routines */

unsigned CarryManager::absPosition(unsigned frameOffsetinBits, unsigned relPos) {
#ifdef PACKING
    return mCurrentFrameIndex + frameOffsetinBits + relPos;
#else
    return mCurrentFrameIndex + frameOffsetinBits/BLOCK_SIZE + relPos;
#endif
}


unsigned CarryManager::carryOpPosition(unsigned localIndex) {
    return absPosition(mCarryInfo->addWithCarry.frameOffsetinBits, localIndex);
}

unsigned CarryManager::advance1Position(unsigned localIndex) {
    return absPosition(mCarryInfo->advance1.frameOffsetinBits, localIndex);
}

unsigned CarryManager::shortAdvancePosition(unsigned localIndex) {
    return absPosition(mCarryInfo->shortAdvance.frameOffsetinBits, localIndex);
}

unsigned CarryManager::longAdvanceBitBlockPosition(unsigned localIndex) {
#ifdef PACKING
    return (mCurrentFrameIndex + mCarryInfo->longAdvance.frameOffsetinBits) / BLOCK_SIZE + localIndex;
#else
    return mCurrentFrameIndex + (mCarryInfo->longAdvance.frameOffsetinBits / BLOCK_SIZE) + localIndex;
#endif
}
    
unsigned CarryManager::localBasePack() {
#ifdef PACKING
    return (mCurrentFrameIndex + mCarryInfo->shortAdvance.frameOffsetinBits) / PACK_SIZE;
#else
    return mCurrentFrameIndex + (mCarryInfo->shortAdvance.frameOffsetinBits / PACK_SIZE);
#endif
}
    
unsigned CarryManager::scopeBasePack() {
#ifdef PACKING
    return mCurrentFrameIndex / PACK_SIZE;
#else
    return mCurrentFrameIndex;
#endif
}
    


unsigned CarryManager::summaryPosition() {
    return absPosition(mCarryInfo->summary.frameOffsetinBits, 0);
}

unsigned CarryManager::summaryBits() {
    return mCarryInfo->summary.allocatedBits;
}



Value * CarryManager::getCarryPack(unsigned packIndex) {
    if (mCarryInPack[packIndex] == nullptr) {
        Value * packPtr = mBuilder->CreateGEP(mCarryPackBasePtr, mBuilder->getInt64(packIndex));
        mCarryPackPtr[packIndex] = packPtr;
        mCarryInPack[packIndex] = mBuilder->CreateAlignedLoad(packPtr, PACK_SIZE/8);
    }
    return mCarryInPack[packIndex];
}

void CarryManager::storeCarryPack(unsigned packIndex) {
    mBuilder->CreateAlignedStore(mCarryOutPack[packIndex], mCarryPackPtr[packIndex], PACK_SIZE/8);
}

Value * CarryManager::getCarryRange(unsigned carryBit_lo, unsigned carryRangeSize) {

    unsigned packIndex = carryBit_lo / PACK_SIZE;
    unsigned carryOffset = carryBit_lo % PACK_SIZE;
    unsigned hiOffset = carryBit_lo + carryRangeSize - 1;
    
    Value * carryItem = getCarryPack(packIndex);
    if (carryRangeSize < PACK_SIZE) {
       carryItem = mBuilder->CreateAnd(carryItem, mBuilder->getInt64((1 << hiOffset) - 1));
    }
    if (carryOffset > 0) {
       carryItem = mBuilder->CreateLShr(carryItem, mBuilder->getInt64(carryOffset));
    }
    return carryItem;
}
    
Value * CarryManager::getCarryBit(unsigned carryBitPos) {
    return getCarryRange(carryBitPos, 1);
}
    
void CarryManager::setCarryBits(unsigned carryBit_lo, Value * bits) {
    
    unsigned packIndex = carryBit_lo / PACK_SIZE;
    unsigned carryOffset = carryBit_lo % PACK_SIZE;
    if (carryOffset > 0) {
        bits = mBuilder->CreateLShr(bits, mBuilder->getInt64(carryOffset));
    }
    if (mCarryOutPack[packIndex] == nullptr) {
        mCarryOutPack[packIndex] = bits;
    }
    else {
        mCarryOutPack[packIndex] = mBuilder->CreateOr(mCarryOutPack[packIndex], bits);
    }
}
    
    
/* Methods for getting and setting individual carry values. */
    
Value * CarryManager::getCarryOpCarryIn(int localIndex) {
    unsigned posn = carryOpPosition(localIndex);
#ifdef PACKING
    return getCarryBit(posn);
#else
    return getCarryPack(posn);
#endif
}
    
    
void CarryManager::setCarryOpCarryOut(unsigned localIndex, Value * carry_out) {
    unsigned posn = carryOpPosition(localIndex);
#ifdef PACKING
    setCarryBits(posn, carry_out);
#else
    mCarryOutPack[posn] = carry_out;
    if (mCarryInfo->getWhileDepth() == 0) {
        storeCarryPack(posn);
    }
#endif
}

    
Value * CarryManager::advanceCarryInCarryOut(int localIndex, int shift_amount, Value * strm) {
    if (shift_amount == 1) {
        return unitAdvanceCarryInCarryOut(localIndex, strm);
    }
    else if (shift_amount < LongAdvanceBase) {
        return shortAdvanceCarryInCarryOut(localIndex, shift_amount, strm);
    }
    else {
        return longAdvanceCarryInCarryOut(localIndex, shift_amount, strm);
    }
}

Value * CarryManager::unitAdvanceCarryInCarryOut(int localIndex, Value * strm) {
    unsigned posn = advance1Position(localIndex);
#ifdef PACKING
    unsigned offset = posn % PACK_SIZE;
    unsigned rshift = PACK_SIZE - offset - 1;
    Value * field = iBuilder->mvmd_extract(PACK_SIZE, strm, BLOCK_SIZE/PACK_SIZE - 1);
    if (rshift != 0) {
        field = mBuilder->CreateLShr(field, mBuilder->getInt64(rshift));
    }
    if (offset != 0) {
        field = mBuilder->CreateAnd(field, mBuilder->getInt64(1<<offset));
    }
    setCarryBits(posn - offset, field);
    Value* carry_longint = mBuilder->CreateZExt(getCarryBit(posn), mBuilder->getIntNTy(BLOCK_SIZE));
    Value* strm_longint = mBuilder->CreateBitCast(strm, mBuilder->getIntNTy(BLOCK_SIZE));
    Value* adv_longint = mBuilder->CreateOr(mBuilder->CreateShl(strm_longint, 1), carry_longint);
    Value* result_value = mBuilder->CreateBitCast(adv_longint, mBitBlockType);
    return result_value;
#else
    mCarryOutPack[posn] = strm; 
    Value * carry_in = getCarryPack(posn);
    if (mCarryInfo->getWhileDepth() == 0) {
        storeCarryPack(posn);
    }
    Value* result_value;
    
#if (BLOCK_SIZE == 128) && !defined(USE_LONG_INTEGER_SHIFT)
    Value * ahead64 = iBuilder->mvmd_dslli(64, carry_in, strm, 1);
    result_value = mBuilder->CreateOr(iBuilder->simd_srli(64, ahead64, 63), iBuilder->simd_slli(64, strm, 1));
#else
    Value* advanceq_longint = mBuilder->CreateBitCast(carry_in, mBuilder->getIntNTy(BLOCK_SIZE));
    Value* strm_longint = mBuilder->CreateBitCast(strm, mBuilder->getIntNTy(BLOCK_SIZE));
    Value* adv_longint = mBuilder->CreateOr(mBuilder->CreateShl(strm_longint, 1), mBuilder->CreateLShr(advanceq_longint, BLOCK_SIZE - 1), "advance");
    result_value = mBuilder->CreateBitCast(adv_longint, mBitBlockType);
    
#endif
    return result_value;
#endif
}

Value * CarryManager::shortAdvanceCarryInCarryOut(int localIndex, int shift_amount, Value * strm) {
    unsigned posn = shortAdvancePosition(localIndex);
#ifdef PACKING
    unsigned offset = posn % PACK_SIZE;
    unsigned rshift = PACK_SIZE - offset - shift_amount;
    Value * field = iBuilder->mvmd_extract(PACK_SIZE, strm, BLOCK_SIZE/PACK_SIZE - 1);
    if (rshift != 0) {
        field = mBuilder->CreateLShr(field, mBuilder->getInt64(rshift));
    }
    if (offset != 0) {
        field = mBuilder->CreateAnd(field, mBuilder->getInt64(((1<<shift_amount) - 1) << offset));
    }    
    setCarryBits(posn - offset, field);
    Value* carry_longint = mBuilder->CreateZExt(getCarryRange(posn, shift_amount), mBuilder->getIntNTy(BLOCK_SIZE));
    Value* strm_longint = mBuilder->CreateBitCast(strm, mBuilder->getIntNTy(BLOCK_SIZE));
    Value* adv_longint = mBuilder->CreateOr(mBuilder->CreateShl(strm_longint, 1), carry_longint);
    Value* result_value = mBuilder->CreateBitCast(adv_longint, mBitBlockType);
    return result_value;
#else
    mCarryOutPack[posn] = strm; 
    Value * carry_in = getCarryPack(posn);
    if (mCarryInfo->getWhileDepth() == 0) {
        storeCarryPack(posn);
    }
    Value* advanceq_longint = mBuilder->CreateBitCast(carry_in, mBuilder->getIntNTy(BLOCK_SIZE));
    Value* strm_longint = mBuilder->CreateBitCast(strm, mBuilder->getIntNTy(BLOCK_SIZE));
    Value* adv_longint = mBuilder->CreateOr(mBuilder->CreateShl(strm_longint, shift_amount), mBuilder->CreateLShr(advanceq_longint, BLOCK_SIZE - shift_amount), "advance");
    return mBuilder->CreateBitCast(adv_longint, mBitBlockType);
#endif
}
    

/*  currently defined in carry_data.h 
 
 static unsigned power2ceil (unsigned v) {
 unsigned ceil = 1;
 while (ceil < v) ceil *= 2;
 return ceil;
 }
 
 unsigned longAdvanceEntries(unsigned shift_amount) const {
 return (shift_amount + BLOCK_SIZE - 1)/BLOCK_SIZE;
 }
 
 unsigned longAdvanceBufferSize(unsigned shift_amount)  const {
 return power2ceil(longAdvanceEntries(shift_amount));
 }
 */

    
Value * CarryManager::longAdvanceCarryInCarryOut(int localIndex, int shift_amount, Value * carry_out) {
    unsigned carryDataIndex = longAdvanceBitBlockPosition(localIndex);
    Value * advBaseIndex = mBuilder->getInt64(carryDataIndex);
    if (shift_amount <= BLOCK_SIZE) {
        // special case using a single buffer entry and the carry_out value.
        Value * advanceDataPtr = mBuilder->CreateGEP(mCarryBitBlockPtr, advBaseIndex);
        Value * carry_block0 = mBuilder->CreateAlignedLoad(advanceDataPtr, BLOCK_SIZE/8);
        mBuilder->CreateAlignedStore(carry_out, advanceDataPtr, BLOCK_SIZE/8);
        /* Very special case - no combine */
        if (shift_amount == BLOCK_SIZE) return carry_block0;
        Value* block0_shr = mBuilder->CreateLShr(mBuilder->CreateBitCast(carry_block0, mBuilder->getIntNTy(BLOCK_SIZE)), BLOCK_SIZE - shift_amount);
        Value* block1_shl = mBuilder->CreateShl(mBuilder->CreateBitCast(carry_out, mBuilder->getIntNTy(BLOCK_SIZE)), shift_amount);
        return mBuilder->CreateBitCast(mBuilder->CreateOr(block1_shl, block0_shr), mBitBlockType);
    }
    // We need a buffer of at least two elements for storing the advance data.
    const unsigned block_shift = shift_amount % BLOCK_SIZE;
    const unsigned advanceEntries = mCarryInfo->longAdvanceEntries(shift_amount);
    const unsigned bufsize = mCarryInfo->longAdvanceBufferSize(shift_amount);
    Value * indexMask = mBuilder->getInt64(bufsize - 1);  // A mask to implement circular buffer indexing
    Value * loadIndex0 = mBuilder->CreateAdd(mBuilder->CreateAnd(mBuilder->CreateSub(mBlockNo, mBuilder->getInt64(advanceEntries)), indexMask), advBaseIndex);
    Value * storeIndex = mBuilder->CreateAdd(mBuilder->CreateAnd(mBlockNo, indexMask), advBaseIndex);
    Value * carry_block0 = mBuilder->CreateAlignedLoad(mBuilder->CreateGEP(mCarryBitBlockPtr, loadIndex0), BLOCK_SIZE/8);
    // If the long advance is an exact multiple of BLOCK_SIZE, we simply return the oldest 
    // block in the long advance carry data area.  
    if (block_shift == 0) {
        mBuilder->CreateAlignedStore(carry_out, mBuilder->CreateGEP(mCarryBitBlockPtr, storeIndex), BLOCK_SIZE/8);
        return carry_block0;
    }
    // Otherwise we need to combine data from the two oldest blocks.
    Value * loadIndex1 = mBuilder->CreateAdd(mBuilder->CreateAnd(mBuilder->CreateSub(mBlockNo, mBuilder->getInt64(advanceEntries-1)), indexMask), advBaseIndex);
    Value * carry_block1 = mBuilder->CreateAlignedLoad(mBuilder->CreateGEP(mCarryBitBlockPtr, loadIndex1), BLOCK_SIZE/8);
    Value* block0_shr = mBuilder->CreateLShr(mBuilder->CreateBitCast(carry_block0, mBuilder->getIntNTy(BLOCK_SIZE)), BLOCK_SIZE - block_shift);
    Value* block1_shl = mBuilder->CreateShl(mBuilder->CreateBitCast(carry_block1, mBuilder->getIntNTy(BLOCK_SIZE)), block_shift);
    mBuilder->CreateAlignedStore(carry_out, mBuilder->CreateGEP(mCarryBitBlockPtr, storeIndex), BLOCK_SIZE/8);
    return mBuilder->CreateBitCast(mBuilder->CreateOr(block1_shl, block0_shr), mBitBlockType);
}
    

/* Methods for getting and setting carry summary values */
   
bool CarryManager::blockHasCarries(){
    return mCarryInfo->blockHasCarries();
} 


Value * CarryManager::getCarrySummaryExpr() {
    unsigned summary_posn = summaryPosition();
#ifdef PACKING
    return getCarryRange(summary_posn, summaryBits());
#else
    return getCarryPack(summary_posn);
#endif
}

void CarryManager::addSummaryPhiIfNeeded(BasicBlock * ifEntryBlock, BasicBlock * ifBodyFinalBlock) {
    if ((mCarryInfo->getIfDepth() <= 1) || !mCarryInfo->blockHasCarries()){
        // For ifDepth == 1, the parent does not need a summary as it is not itself within an if.
        // Therefore, it doesn't need access to this block's summary in building its own.
        return;
    }
    const unsigned carrySummaryIndex = summaryPosition();
    PHINode * summary_phi = mBuilder->CreatePHI(mCarryPackType, 2, "summary");
    summary_phi->addIncoming(mZeroInitializer, ifEntryBlock);
    summary_phi->addIncoming(mCarryOutPack[carrySummaryIndex], ifBodyFinalBlock);
    mCarryOutPack[carrySummaryIndex] = summary_phi;
}

void CarryManager::generateCarryOutSummaryCodeIfNeeded() {
    
    if (!mCarryInfo->explicitSummaryRequired()) {
        // An explicit summary may not be required, if there is a single carry
        // operation within the block, or the carries are packed and all carry
        // bits fit within a single pack.
        return;
    }
    
    const unsigned carrySummaryIndex = summaryPosition();
    
    Value * carry_summary = mZeroInitializer;
    if (mCarryInfo->blockHasLongAdvances()) { // Force if entry
        carry_summary = mOneInitializer;
    }
    else {
        auto localCarryIndex = localBasePack();
        auto localCarryPacks = mCarryInfo->getLocalCarryPackCount();
        if (localCarryPacks > 0) {
            carry_summary = mCarryOutPack[localCarryIndex];
            for (auto i = 1; i < localCarryPacks; i++) {
                //carry_summary = mBuilder->CreateOr(carry_summary, mPabloBlock->mCarryOutPack[i]);            
                carry_summary = mBuilder->CreateOr(carry_summary, mCarryOutPack[localCarryIndex+i]);
            }
        }
        for (Statement * stmt : *mCurrentScope) {
            if (If * innerIf = dyn_cast<If>(stmt)) {
                PabloBlock * inner_blk = & innerIf->getBody();
                enterScope(inner_blk);
                if (blockHasCarries()) {
                  carry_summary = mBuilder->CreateOr(carry_summary, mCarryOutPack[summaryPosition()]);
                }
                leaveScope();
            }
            else if (While * innerWhile = dyn_cast<While>(stmt)) {
                PabloBlock * inner_blk = & innerWhile->getBody();
                enterScope(inner_blk);
                if (blockHasCarries()) {
                    carry_summary = mBuilder->CreateOr(carry_summary, mCarryOutPack[summaryPosition()]);
                }
                leaveScope();
            }
        }
    }
    // Calculation of the carry out summary is complete.   Store it and make it
    // available in case it must included by parent blocks.
    mCarryOutPack[carrySummaryIndex] = carry_summary;
    storeCarryPack(carrySummaryIndex);
}



void CarryManager::ensureCarriesLoadedRecursive() {
    const unsigned scopeCarryPacks = mCarryInfo->getScopeCarryPackCount();
    const unsigned currentScopeBase = scopeBasePack();
    if (mCarryInfo->getWhileDepth() == 1) {
        for (auto i = currentScopeBase; i < currentScopeBase + scopeCarryPacks; ++i) {
            getCarryPack(i);
        }
    }
}


void CarryManager::initializeCarryDataPhisAtWhileEntry(BasicBlock * whileEntryBlock) {
    const unsigned scopeCarryPacks = mCarryInfo->getScopeCarryPackCount();
    mCarryOutAccumPhis.resize(scopeCarryPacks);
#ifdef SET_WHILE_CARRY_IN_TO_ZERO_AFTER_FIRST_ITERATION
    const unsigned currentScopeBase = scopeBasePack();
    mCarryInPhis.resize(scopeCarryPacks);
#endif
    for (unsigned index = 0; index < scopeCarryPacks; ++index) {
#ifdef SET_WHILE_CARRY_IN_TO_ZERO_AFTER_FIRST_ITERATION
        PHINode * phi_in = mBuilder->CreatePHI(mCarryPackType, 2);
        phi_in->addIncoming(mCarryInPack[currentScopeBase+index], whileEntryBlock);
        mCarryInPhis[index] = phi_in;
#endif
        PHINode * phi_out = mBuilder->CreatePHI(mCarryPackType, 2);
        phi_out->addIncoming(mZeroInitializer, whileEntryBlock);
        mCarryOutAccumPhis[index] = phi_out;
    }
}


void CarryManager::extendCarryDataPhisAtWhileBodyFinalBlock(BasicBlock * whileBodyFinalBlock) {
    const unsigned scopeCarryPacks = mCarryInfo->getScopeCarryPackCount();
    const unsigned currentScopeBase = scopeBasePack();
    for (unsigned index = 0; index < scopeCarryPacks; ++index) {
#ifdef SET_WHILE_CARRY_IN_TO_ZERO_AFTER_FIRST_ITERATION
        mCarryInPhis[index]->addIncoming(mZeroInitializer, whileBodyFinalBlock);
#endif
        PHINode * phi = mCarryOutAccumPhis[index];
        Value * carryOut = mBuilder->CreateOr(phi, mCarryOutPack[currentScopeBase+index]);
        phi->addIncoming(carryOut, whileBodyFinalBlock);
        mCarryOutPack[currentScopeBase+index] = carryOut;
    }
}

void CarryManager::ensureCarriesStoredRecursive() {
    const unsigned scopeCarryPacks = mCarryInfo->getScopeCarryPackCount();
    const unsigned currentScopeBase = scopeBasePack();
    if (mCarryInfo->getWhileDepth() == 1) {
        for (auto i = currentScopeBase; i < currentScopeBase + scopeCarryPacks; ++i) {
            storeCarryPack(i);
        }
    }
}

}

