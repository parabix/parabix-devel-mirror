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
    mCarryDataPtr = carryPtr;
    unsigned scopeCount = doScopeCount(pb);
    mCarryInfoVector.resize(scopeCount);
    
    unsigned totalCarryDataBits = enumerate(pb, 0, 0);
    
    mTotalCarryDataBitBlocks = (totalCarryDataBits + BLOCK_SIZE - 1)/BLOCK_SIZE; 
    // Carry Data area will have one extra bit block to store the block number.
    mBlockNoPtr = mBuilder->CreateBitCast(mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(mTotalCarryDataBitBlocks)), Type::getInt64PtrTy(mBuilder->getContext()));
    mBlockNo = mBuilder->CreateLoad(mBlockNoPtr);
    mCarryPackPtr.resize(mTotalCarryDataBitBlocks);
    mCarryInPack.resize(mTotalCarryDataBitBlocks);
    mCarryInPhis.resize(mTotalCarryDataBitBlocks);
    mCarryOutAccumPhis.resize(mTotalCarryDataBitBlocks);
    mCarryOutVector.resize(mTotalCarryDataBitBlocks);
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
#ifndef NDEBUG
    cd->dumpCarryData(cerr);
#endif
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


Value * CarryManager::getCarryPack(unsigned packIndex) {
    if (mCarryInPack[packIndex] == nullptr) {
        Value * packPtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(packIndex));
        mCarryPackPtr[packIndex] = packPtr;
        mCarryInPack[packIndex] = mBuilder->CreateAlignedLoad(packPtr, PACK_SIZE/8);
    }
    return mCarryInPack[packIndex];
}

void CarryManager::CarryPackStore(unsigned packIndex) {
    mBuilder->CreateAlignedStore(mCarryOutVector[packIndex], mCarryPackPtr[packIndex], PACK_SIZE/8);
}

Value * CarryManager::genCarryInRange(unsigned carryBit_lo, unsigned carryRangeSize) {

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
    
 
Value * CarryManager::genCarryInBit(unsigned carryBitPos) {
    return genCarryInRange(carryBitPos, 1);
}
    /* Methods for getting and setting individual carry values. */
    
//#define LOAD_STORE_ON_BLOCK_ENTRY_EXIT    
Value * CarryManager::getCarryOpCarryIn(int localIndex) {
    unsigned cd_index = mCurrentFrameIndex + mCarryInfo->carryOpCarryDataOffset(localIndex);
    return getCarryPack(cd_index);
}

void CarryManager::setCarryOpCarryOut(unsigned localIndex, Value * carry_out) {
    unsigned cd_index = mCurrentFrameIndex + mCarryInfo->carryOpCarryDataOffset(localIndex);
    mCarryOutVector[cd_index] = carry_out;
#ifndef LOAD_STORE_ON_BLOCK_ENTRY_EXIT
    if (mCarryInfo->getWhileDepth() == 0) {
        CarryPackStore(cd_index);
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
   
    unsigned carryDataIndex = mCurrentFrameIndex + mCarryInfo->unitAdvanceCarryDataOffset(localIndex);
    mCarryOutVector[carryDataIndex] = strm; 
    Value * carry_in = getCarryPack(carryDataIndex);
#ifndef LOAD_STORE_ON_BLOCK_ENTRY_EXIT
    if (mCarryInfo->getWhileDepth() == 0) {
        CarryPackStore(carryDataIndex);
    }
#endif
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
}

Value * CarryManager::shortAdvanceCarryInCarryOut(int localIndex, int shift_amount, Value * strm) {
    unsigned carryDataIndex = mCurrentFrameIndex + mCarryInfo->shortAdvanceCarryDataOffset(localIndex);
    mCarryOutVector[carryDataIndex] = strm; 
    Value * carry_in = getCarryPack(carryDataIndex);
#ifndef LOAD_STORE_ON_BLOCK_ENTRY_EXIT
    if (mCarryInfo->getWhileDepth() == 0) {
        CarryPackStore(carryDataIndex);
    }
#endif
    Value* advanceq_longint = mBuilder->CreateBitCast(carry_in, mBuilder->getIntNTy(BLOCK_SIZE));
    Value* strm_longint = mBuilder->CreateBitCast(strm, mBuilder->getIntNTy(BLOCK_SIZE));
    Value* adv_longint = mBuilder->CreateOr(mBuilder->CreateShl(strm_longint, shift_amount), mBuilder->CreateLShr(advanceq_longint, BLOCK_SIZE - shift_amount), "advance");
    return mBuilder->CreateBitCast(adv_longint, mBitBlockType);
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
    unsigned carryDataIndex = mCurrentFrameIndex + mCarryInfo->longAdvanceCarryDataOffset(localIndex);
    Value * advBaseIndex = mBuilder->getInt64(carryDataIndex);
    if (shift_amount <= BLOCK_SIZE) {
        // special case using a single buffer entry and the carry_out value.
        Value * advanceDataPtr = mBuilder->CreateGEP(mCarryDataPtr, advBaseIndex);
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
    Value * carry_block0 = mBuilder->CreateAlignedLoad(mBuilder->CreateGEP(mCarryDataPtr, loadIndex0), BLOCK_SIZE/8);
    // If the long advance is an exact multiple of BLOCK_SIZE, we simply return the oldest 
    // block in the long advance carry data area.  
    if (block_shift == 0) {
        mBuilder->CreateAlignedStore(carry_out, mBuilder->CreateGEP(mCarryDataPtr, storeIndex), BLOCK_SIZE/8);
        return carry_block0;
    }
    // Otherwise we need to combine data from the two oldest blocks.
    Value * loadIndex1 = mBuilder->CreateAdd(mBuilder->CreateAnd(mBuilder->CreateSub(mBlockNo, mBuilder->getInt64(advanceEntries-1)), indexMask), advBaseIndex);
    Value * carry_block1 = mBuilder->CreateAlignedLoad(mBuilder->CreateGEP(mCarryDataPtr, loadIndex1), BLOCK_SIZE/8);
    Value* block0_shr = mBuilder->CreateLShr(mBuilder->CreateBitCast(carry_block0, mBuilder->getIntNTy(BLOCK_SIZE)), BLOCK_SIZE - block_shift);
    Value* block1_shl = mBuilder->CreateShl(mBuilder->CreateBitCast(carry_block1, mBuilder->getIntNTy(BLOCK_SIZE)), block_shift);
    mBuilder->CreateAlignedStore(carry_out, mBuilder->CreateGEP(mCarryDataPtr, storeIndex), BLOCK_SIZE/8);
    return mBuilder->CreateBitCast(mBuilder->CreateOr(block1_shl, block0_shr), mBitBlockType);
}
    

/* Methods for getting and setting carry summary values */
   
bool CarryManager::blockHasCarries(){
    return mCarryInfo->blockHasCarries();
} 


Value * CarryManager::getCarrySummaryExpr() {
    unsigned summary_idx = mCurrentFrameIndex + mCarryInfo->summaryCarryDataIndex();
    return getCarryPack(summary_idx);
}

void CarryManager::addSummaryPhiIfNeeded(BasicBlock * ifEntryBlock, BasicBlock * ifBodyFinalBlock) {
    if ((mCarryInfo->getIfDepth() <= 1) || !mCarryInfo->blockHasCarries()){
        // For ifDepth == 1, the parent does not need a summary as it is not itself within an if.
        // Therefore, it doesn't need access to this block's summary in building its own.
        return;
    }
    const unsigned carrySummaryIndex = mCurrentFrameIndex + mCarryInfo->summaryCarryDataIndex();
    PHINode * summary_phi = mBuilder->CreatePHI(mBitBlockType, 2, "summary");
    summary_phi->addIncoming(mZeroInitializer, ifEntryBlock);
    summary_phi->addIncoming(mCarryOutVector[carrySummaryIndex], ifBodyFinalBlock);
    mCarryOutVector[carrySummaryIndex] = summary_phi;
}

void CarryManager::generateCarryOutSummaryCodeIfNeeded() {
    
    if (!mCarryInfo->explicitSummaryRequired()) {
        // An explicit summary may not be required, if there is a single carry
        // operation within the block, or the carries are packed and all carry
        // bits fit within a single pack.
        return;
    }
    
    const unsigned carrySummaryIndex = mCurrentFrameIndex + mCarryInfo->summaryCarryDataIndex();
    
    Value * carry_summary = mZeroInitializer;
    
    if (mCarryInfo->blockHasLongAdvances()) { // Force if entry
        carry_summary = mOneInitializer;
    }
    else {
        auto localCarryIndex = mCurrentFrameIndex + mCarryInfo->getLocalCarryPackIndex();
        auto localCarryPacks = mCarryInfo->getLocalCarryDataSize();
        if (localCarryPacks > 0) {
            carry_summary = mCarryOutVector[localCarryIndex];
            for (auto i = 1; i < localCarryPacks; i++) {
                //carry_summary = mBuilder->CreateOr(carry_summary, mPabloBlock->mCarryOutPack[i]);            
                carry_summary = mBuilder->CreateOr(carry_summary, mCarryOutVector[localCarryIndex+i]);
            }
        }
        for (Statement * stmt : *mCurrentScope) {
            if (If * innerIf = dyn_cast<If>(stmt)) {
                PabloBlock * inner_blk = & innerIf->getBody();
                enterScope(inner_blk);
                if (blockHasCarries()) {
                  carry_summary = mBuilder->CreateOr(carry_summary, mCarryOutVector[mCurrentFrameIndex + mCarryInfo->summaryCarryDataIndex()]);
                }
                leaveScope();
            }
            else if (While * innerWhile = dyn_cast<While>(stmt)) {
                PabloBlock * inner_blk = & innerWhile->getBody();
                enterScope(inner_blk);
                if (blockHasCarries()) {
                    carry_summary = mBuilder->CreateOr(carry_summary, mCarryOutVector[mCurrentFrameIndex + mCarryInfo->summaryCarryDataIndex()]);
                }
                leaveScope();
            }
        }
    }
    // Calculation of the carry out summary is complete.   Store it and make it
    // available in case it must included by parent blocks.
    mCarryOutVector[carrySummaryIndex] = carry_summary;
    CarryPackStore(carrySummaryIndex);
}


void CarryManager::ensureCarriesLoadedLocal() {
#ifdef LOAD_STORE_ON_BLOCK_ENTRY_EXIT
    if ((mCarryInfo->getScopeCarryDataSize() == 0 ) || (mCarryInfo->getWhileDepth() > 0)) return;
    if ((mCarryInfo->getIfDepth() == 0) || mCarryInfo->explicitSummaryRequired()) {
        auto localCarryIndex = mCurrentFrameIndex + mCarryInfo->getLocalCarryPackIndex();
        auto localCarryPacks = mCarryInfo->getLocalCarryDataSize();
        //std::cerr << "ensureCarriesLoadedLocal: localCarryIndex =  " << localCarryIndex << "localCarryPacks =  " << localCarryPacks << std::endl;
        for (auto i = localCarryIndex; i < localCarryIndex + localCarryPacks; i++) {        
            mCarryInPack[i] = mBuilder->CreateAlignedLoad(mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(i)), BLOCK_SIZE/8, false);
        }
    }
#endif
}

void CarryManager::ensureCarriesStoredLocal() {
#ifdef LOAD_STORE_ON_BLOCK_ENTRY_EXIT
    if ((mCarryInfo->getScopeCarryDataSize() == 0 ) || (mCarryInfo->getWhileDepth() > 0)) return;
    auto localCarryIndex = mCurrentFrameIndex + mCarryInfo->getLocalCarryPackIndex();
    auto localCarryPacks = mCarryInfo->getLocalCarryDataSize();
    //std::cerr << "ensureCarriesStoredLocal: localCarryIndex =  " << localCarryIndex << "localCarryPacks =  " << localCarryPacks << std::endl;
    for (auto i = localCarryIndex; i < localCarryIndex + localCarryPacks; i++) {        
        Value * storePtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(i));
        mBuilder->CreateAlignedStore(mCarryOutVector[i], storePtr, BLOCK_SIZE/8, false);
    }
#endif
}



void CarryManager::ensureCarriesLoadedRecursive() {
    const unsigned scopeCarryDataSize = mCarryInfo->getScopeCarryDataSize();
    if (mCarryInfo->getWhileDepth() == 1) {
        for (auto i = mCurrentFrameIndex; i < mCurrentFrameIndex + scopeCarryDataSize; ++i) {
            getCarryPack(i);
        }
    }
}


void CarryManager::initializeCarryDataPhisAtWhileEntry(BasicBlock * whileEntryBlock) {
    const unsigned scopeCarryDataSize = mCarryInfo->getScopeCarryDataSize();
    for (unsigned index = mCurrentFrameIndex; index < mCurrentFrameIndex + scopeCarryDataSize; ++index) {
#ifdef SET_WHILE_CARRY_IN_TO_ZERO_AFTER_FIRST_ITERATION
        PHINode * phi_in = mBuilder->CreatePHI(mBitBlockType, 2);
        phi_in->addIncoming(mCarryInPack[index], whileEntryBlock);
        mCarryInPhis[index] = phi_in;
#endif
        PHINode * phi_out = mBuilder->CreatePHI(mBitBlockType, 2);
        phi_out->addIncoming(mZeroInitializer, whileEntryBlock);
        mCarryOutAccumPhis[index] = phi_out;
    }
}


void CarryManager::extendCarryDataPhisAtWhileBodyFinalBlock(BasicBlock * whileBodyFinalBlock) {
    const unsigned scopeCarryDataSize = mCarryInfo->getScopeCarryDataSize();
    for (unsigned index = mCurrentFrameIndex; index < mCurrentFrameIndex + scopeCarryDataSize; ++index) {
#ifdef SET_WHILE_CARRY_IN_TO_ZERO_AFTER_FIRST_ITERATION
        mCarryInPhis[index]->addIncoming(mZeroInitializer, whileBodyFinalBlock);
#endif
        PHINode * phi = mCarryOutAccumPhis[index];
        assert (phi);
        assert (mCarryOutVector[index]);
        Value * carryOut = mBuilder->CreateOr(phi, mCarryOutVector[index]);
        phi->addIncoming(carryOut, whileBodyFinalBlock);
        mCarryOutVector[index] = carryOut;
    }
}

void CarryManager::ensureCarriesStoredRecursive() {
    const unsigned scopeCarryDataSize = mCarryInfo->getScopeCarryDataSize();
    if (mCarryInfo->getWhileDepth() == 1) {
        for (auto i = mCurrentFrameIndex; i < mCurrentFrameIndex + scopeCarryDataSize; ++i) {
            CarryPackStore(i);
        }
    }
}

}

