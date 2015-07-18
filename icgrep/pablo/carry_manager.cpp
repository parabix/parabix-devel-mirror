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
  

unsigned CarryManager::initialize(PabloBlock * pb, Value * carryPtr) {
  
    mPabloRoot = pb;
    mCurrentScope = pb;
    mCurrentScopeIndex = 0;
    
    mCarryDataPtr = carryPtr;
    mCarryInfo = &(pb->carryData);
    unsigned totalCarryDataBits = mCarryInfo->enumerate(*pb);
    mTotalCarryDataBitBlocks = (totalCarryDataBits + BLOCK_SIZE - 1)/BLOCK_SIZE + 1; // One extra element for the block no.
    mBlockNoPtr = mBuilder->CreateBitCast(mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(mTotalCarryDataBitBlocks - 1)), Type::getInt64PtrTy(mBuilder->getContext()));
    mBlockNo = mBuilder->CreateLoad(mBlockNoPtr);
    mCarryInVector.resize(mTotalCarryDataBitBlocks);
    mCarryInPhis.resize(mTotalCarryDataBitBlocks);
    mCarryOutAccumPhis.resize(mTotalCarryDataBitBlocks);
    mCarryOutVector.resize(mTotalCarryDataBitBlocks);
    
    return mTotalCarryDataBitBlocks;
}
    
void CarryManager::generateBlockNoIncrement() {
    mBuilder->CreateStore(mBuilder->CreateAdd(mBlockNo, mBuilder->getInt64(1)), mBlockNoPtr);
}

Value * CarryManager::getBlockNoPtr() {
    return mBlockNoPtr;
}

/* Entering and leaving blocks. */

void CarryManager::enterScope(PabloBlock * blk) {
    
    mCurrentScope = blk;
    mCarryInfo = & (blk->carryData);
    mCurrentScopeIndex += mCarryInfo->getBlockCarryDataIndex();
    //std::cerr << "enterScope:  mCurrentScopeIndex = " << mCurrentScopeIndex << std::endl;
}

void CarryManager::leaveScope() {
    mCurrentScopeIndex -= mCarryInfo->getBlockCarryDataIndex();
    mCurrentScope = mCurrentScope->getParent();
    mCarryInfo = & (mCurrentScope->carryData);
    //std::cerr << "leaveScope:  mCurrentScopeIndex = " << mCurrentScopeIndex << std::endl;
}

    /* Methods for getting and setting individual carry values. */
    
//#define LOAD_STORE_ON_BLOCK_ENTRY_EXIT    
Value * CarryManager::getCarryOpCarryIn(int localIndex) {
    unsigned cd_index = mCurrentScopeIndex + mCarryInfo->carryOpCarryDataOffset(localIndex);
#ifndef LOAD_STORE_ON_BLOCK_ENTRY_EXIT
    if (mCarryInfo->getWhileDepth() == 0) {
       Value * packPtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(cd_index));
       mCarryInVector[cd_index] = mBuilder->CreateAlignedLoad(packPtr, BLOCK_SIZE/8);
    }
#endif
    return mCarryInVector[cd_index];
}

void CarryManager::setCarryOpCarryOut(unsigned localIndex, Value * carry_out) {
    unsigned cd_index = mCurrentScopeIndex + mCarryInfo->carryOpCarryDataOffset(localIndex);
    mCarryOutVector[cd_index] = carry_out;
#ifndef LOAD_STORE_ON_BLOCK_ENTRY_EXIT
    if (mCarryInfo->getWhileDepth() == 0) {
       Value * packPtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(cd_index));
       mBuilder->CreateAlignedStore(carry_out, packPtr, BLOCK_SIZE/8);
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
    unsigned carryDataIndex = mCurrentScopeIndex + mCarryInfo->unitAdvanceCarryDataOffset(localIndex);
    mCarryOutVector[carryDataIndex] = strm; 
#ifndef LOAD_STORE_ON_BLOCK_ENTRY_EXIT
    if (mCarryInfo->getWhileDepth() == 0) {
        Value * packPtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(carryDataIndex));
        mCarryInVector[carryDataIndex] = mBuilder->CreateAlignedLoad(packPtr, BLOCK_SIZE/8);
        mBuilder->CreateAlignedStore(strm, packPtr, BLOCK_SIZE/8);
    }
#endif
    Value * carry_in = mCarryInVector[carryDataIndex];
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
    unsigned carryDataIndex = mCurrentScopeIndex + mCarryInfo->shortAdvanceCarryDataOffset(localIndex);
    mCarryOutVector[carryDataIndex] = strm; 
#ifndef LOAD_STORE_ON_BLOCK_ENTRY_EXIT
    if (mCarryInfo->getWhileDepth() == 0) {
        Value * packPtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(carryDataIndex));
        mCarryInVector[carryDataIndex] = mBuilder->CreateAlignedLoad(packPtr, BLOCK_SIZE/8);
        mBuilder->CreateAlignedStore(strm, packPtr, BLOCK_SIZE/8);
    }
#endif
    Value * carry_in = mCarryInVector[carryDataIndex];
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
    unsigned carryDataIndex = mCurrentScopeIndex + mCarryInfo->longAdvanceCarryDataOffset(localIndex);
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
    unsigned summary_idx = mCurrentScopeIndex + mCarryInfo->summaryCarryDataIndex();
    Value * packPtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(summary_idx));
    Value * summary_expr = mBuilder->CreateAlignedLoad(packPtr, BLOCK_SIZE/8);
    // If the scopeCarryDataSize is 1, then the carry summary is also the pack expr.
    mCarryInVector[summary_idx] = summary_expr;
    return summary_expr;
}

void CarryManager::addSummaryPhiIfNeeded(BasicBlock * ifEntryBlock, BasicBlock * ifBodyFinalBlock) {
    if ((mCarryInfo->getIfDepth() <= 1) || !mCarryInfo->blockHasCarries()){
        // For ifDepth == 1, the parent does not need a summary as it is not itself within an if.
        // Therefore, it doesn't need access to this block's summary in building its own.
        return;
    }
    const unsigned carrySummaryIndex = mCurrentScopeIndex + mCarryInfo->summaryCarryDataIndex();
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
    
    const unsigned carrySummaryIndex = mCurrentScopeIndex + mCarryInfo->summaryCarryDataIndex();
    
    Value * carry_summary = mZeroInitializer;
    
    if (mCarryInfo->blockHasLongAdvances()) { // Force if entry
        carry_summary = mOneInitializer;
    }
    else {
        auto localCarryIndex = mCurrentScopeIndex + mCarryInfo->getLocalCarryPackIndex();
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
                  carry_summary = mBuilder->CreateOr(carry_summary, mCarryOutVector[mCurrentScopeIndex + mCarryInfo->summaryCarryDataIndex()]);
                }
                leaveScope();
            }
            else if (While * innerWhile = dyn_cast<While>(stmt)) {
                PabloBlock * inner_blk = & innerWhile->getBody();
                enterScope(inner_blk);
                if (blockHasCarries()) {
                    carry_summary = mBuilder->CreateOr(carry_summary, mCarryOutVector[mCurrentScopeIndex + mCarryInfo->summaryCarryDataIndex()]);
                }
                leaveScope();
            }
        }
    }
    // Calculation of the carry out summary is complete.   Store it and make it
    // available in case it must included by parent blocks.
    mCarryOutVector[carrySummaryIndex] = carry_summary;
    Value * packPtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(carrySummaryIndex));
    mBuilder->CreateAlignedStore(carry_summary, packPtr, BLOCK_SIZE/8);
}


void CarryManager::ensureCarriesLoadedLocal() {
#ifdef LOAD_STORE_ON_BLOCK_ENTRY_EXIT
    if ((mCarryInfo->getScopeCarryDataSize() == 0 ) || (mCarryInfo->getWhileDepth() > 0)) return;
    if ((mCarryInfo->getIfDepth() == 0) || mCarryInfo->explicitSummaryRequired()) {
        auto localCarryIndex = mCurrentScopeIndex + mCarryInfo->getLocalCarryPackIndex();
        auto localCarryPacks = mCarryInfo->getLocalCarryDataSize();
        //std::cerr << "ensureCarriesLoadedLocal: localCarryIndex =  " << localCarryIndex << "localCarryPacks =  " << localCarryPacks << std::endl;
        for (auto i = localCarryIndex; i < localCarryIndex + localCarryPacks; i++) {        
            mCarryInVector[i] = mBuilder->CreateAlignedLoad(mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(i)), BLOCK_SIZE/8, false);
        }
    }
#endif
}

void CarryManager::ensureCarriesStoredLocal() {
#ifdef LOAD_STORE_ON_BLOCK_ENTRY_EXIT
    if ((mCarryInfo->getScopeCarryDataSize() == 0 ) || (mCarryInfo->getWhileDepth() > 0)) return;
    auto localCarryIndex = mCurrentScopeIndex + mCarryInfo->getLocalCarryPackIndex();
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
        for (auto i = mCurrentScopeIndex; i < mCurrentScopeIndex + scopeCarryDataSize; ++i) {
            mCarryInVector[i] = mBuilder->CreateAlignedLoad(mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(i)), BLOCK_SIZE/8, false);
        }
    }
}


void CarryManager::initializeCarryDataPhisAtWhileEntry(BasicBlock * whileEntryBlock) {
    const unsigned scopeCarryDataSize = mCarryInfo->getScopeCarryDataSize();
    for (unsigned index = mCurrentScopeIndex; index < mCurrentScopeIndex + scopeCarryDataSize; ++index) {
#ifdef SET_WHILE_CARRY_IN_TO_ZERO_AFTER_FIRST_ITERATION
        PHINode * phi_in = mBuilder->CreatePHI(mBitBlockType, 2);
        phi_in->addIncoming(mCarryInVector[index], whileEntryBlock);
        mCarryInPhis[index] = phi_in;
#endif
        PHINode * phi_out = mBuilder->CreatePHI(mBitBlockType, 2);
        phi_out->addIncoming(mZeroInitializer, whileEntryBlock);
        mCarryOutAccumPhis[index] = phi_out;
    }
}


void CarryManager::extendCarryDataPhisAtWhileBodyFinalBlock(BasicBlock * whileBodyFinalBlock) {
    const unsigned scopeCarryDataSize = mCarryInfo->getScopeCarryDataSize();
    for (unsigned index = mCurrentScopeIndex; index < mCurrentScopeIndex + scopeCarryDataSize; ++index) {
#ifdef SET_WHILE_CARRY_IN_TO_ZERO_AFTER_FIRST_ITERATION
        mCarryInPhis[index]->addIncoming(mZeroInitializer, whileBodyFinalBlock);
#endif
        PHINode * phi = mCarryOutAccumPhis[index];
        Value * carryOut = mBuilder->CreateOr(phi, mCarryOutVector[index]);
        phi->addIncoming(carryOut, whileBodyFinalBlock);
        mCarryOutVector[index] = carryOut;
    }
}

void CarryManager::ensureCarriesStoredRecursive() {
    const unsigned scopeCarryDataSize = mCarryInfo->getScopeCarryDataSize();
    if (mCarryInfo->getWhileDepth() == 1) {
        for (auto i = mCurrentScopeIndex; i < mCurrentScopeIndex + scopeCarryDataSize; ++i) {
            Value * storePtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(i));
            mBuilder->CreateAlignedStore(mCarryOutVector[i], storePtr, BLOCK_SIZE/8, false);
        }
    }
}

}

