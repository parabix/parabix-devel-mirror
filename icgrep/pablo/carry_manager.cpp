/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */


#include <include/simd-lib/bitblock.hpp>
#include <stdexcept>
#include <carry_data.h>
#include <codegenstate.h>
#include <carry_manager.h>
#include <pabloAST.h>
#include <iostream>

namespace pablo {
  

unsigned CarryManager::initialize(PabloBlock * pb, Value * carryPtr) {
  
    mPabloRoot = pb;
    mCarryDataPtr = carryPtr;
    
    PabloBlockCarryData & cd = pb->carryData;
    mTotalCarryDataSize = cd.enumerate(*pb) + 1;   // One extra element for the block no.
    mBlockNoPtr = mBuilder->CreateBitCast(mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(mTotalCarryDataSize - 1)), Type::getInt64PtrTy(mBuilder->getContext()));
    mBlockNo = mBuilder->CreateLoad(mBlockNoPtr);
    mCarryInVector.resize(mTotalCarryDataSize);
    mCarryInPhis.resize(mTotalCarryDataSize);
    mCarryOutAccumPhis.resize(mTotalCarryDataSize);
    mCarryOutVector.resize(mTotalCarryDataSize);
    
    return mTotalCarryDataSize;
}

void CarryManager::generateBlockNoIncrement() {
    mBuilder->CreateStore(mBuilder->CreateAdd(mBlockNo, mBuilder->getInt64(1)), mBlockNoPtr);
}

Value * CarryManager::getBlockNoPtr() {
    return mBlockNoPtr;
}


    /* Methods for getting and setting individual carry values. */
    
Value * CarryManager::getCarryOpCarryIn(PabloBlock * blk, int localIndex) {
    PabloBlockCarryData & cd = blk->carryData;
    if (cd.getWhileDepth() == 0) {
       Value * packPtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(cd.carryOpCarryDataOffset(localIndex)));
       mCarryInVector[cd.carryOpCarryDataOffset(localIndex)] = mBuilder->CreateAlignedLoad(packPtr, BLOCK_SIZE/8);
    }
    return mCarryInVector[cd.carryOpCarryDataOffset(localIndex)];
}

void CarryManager::setCarryOpCarryOut(PabloBlock * blk, unsigned idx, Value * carry_out) {
    PabloBlockCarryData & cd = blk->carryData;
    mCarryOutVector[cd.carryOpCarryDataOffset(idx)] = carry_out;
    if (cd.getWhileDepth() == 0) {
       Value * packPtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(cd.carryOpCarryDataOffset(idx)));
       mBuilder->CreateAlignedStore(carry_out, packPtr, BLOCK_SIZE/8);
    }
}

    
Value * CarryManager::advanceCarryInCarryOut(PabloBlock * blk, int localIndex, int shift_amount, Value * strm) {
    if (shift_amount == 1) {
        return unitAdvanceCarryInCarryOut(blk, localIndex, strm);
    }
    else if (shift_amount < LongAdvanceBase) {
        return shortAdvanceCarryInCarryOut(blk, localIndex, shift_amount, strm);
    }
    else {
        return longAdvanceCarryInCarryOut(blk, localIndex, shift_amount, strm);
    }
}

Value * CarryManager::unitAdvanceCarryInCarryOut(PabloBlock * blk, int localIndex, Value * strm) {
    PabloBlockCarryData & cd = blk->carryData;
    unsigned carryDataIndex = cd.unitAdvanceCarryDataOffset(localIndex);
    mCarryOutVector[carryDataIndex] = strm; 
    if (cd.getWhileDepth() == 0) {
        Value * packPtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(carryDataIndex));
        mCarryInVector[carryDataIndex] = mBuilder->CreateAlignedLoad(packPtr, BLOCK_SIZE/8);
        mBuilder->CreateAlignedStore(strm, packPtr, BLOCK_SIZE/8);
        
    }
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

Value * CarryManager::shortAdvanceCarryInCarryOut(PabloBlock * blk, int localIndex, int shift_amount, Value * strm) {
    PabloBlockCarryData & cd = blk->carryData;
    unsigned carryDataIndex = cd.shortAdvanceCarryDataOffset(localIndex);
    mCarryOutVector[carryDataIndex] = strm; 
    if (cd.getWhileDepth() == 0) {
        Value * packPtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(carryDataIndex));
        mCarryInVector[carryDataIndex] = mBuilder->CreateAlignedLoad(packPtr, BLOCK_SIZE/8);
        mBuilder->CreateAlignedStore(strm, packPtr, BLOCK_SIZE/8);
        
    }
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

    
Value * CarryManager::longAdvanceCarryInCarryOut(PabloBlock * blk, int localIndex, int shift_amount, Value * carry_out) {
    PabloBlockCarryData & cd = blk->carryData;
    Value * advBaseIndex = mBuilder->getInt64(cd.longAdvanceCarryDataOffset(localIndex));
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
    const unsigned advanceEntries = cd.longAdvanceEntries(shift_amount);
    const unsigned bufsize = cd.longAdvanceBufferSize(shift_amount);
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
   
bool CarryManager::blockHasCarries(PabloBlock & blk){
    PabloBlockCarryData & cd = blk.carryData;
    return cd.getTotalCarryDataSize() > 0;
} 

Value * CarryManager::getCarrySummaryExpr(PabloBlock & blk) {
    PabloBlockCarryData & cd = blk.carryData;
    Value * packPtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(cd.summaryCarryDataIndex()));
    Value * summary_expr = mBuilder->CreateAlignedLoad(packPtr, BLOCK_SIZE/8);
    // If the totalCarryDataSize is 1, then the carry summary is also the pack expr.
    if (cd.getTotalCarryDataSize() == 1) {
        mCarryInVector[cd.summaryCarryDataIndex()] = summary_expr;
    }
    return summary_expr;
}

bool CarryManager::summaryNeededInParentBlock(PabloBlock & blk){
    PabloBlockCarryData & cd = blk.carryData;
    return (cd.getIfDepth() > 0) && (cd.getTotalCarryDataSize() > 0);
} 

void CarryManager::addSummaryPhi(PabloBlock & blk, BasicBlock * ifEntryBlock, BasicBlock * ifBodyFinalBlock) {
    const PabloBlockCarryData & cd = blk.carryData;
    const unsigned carrySummaryIndex = cd.summaryCarryDataIndex();
    PHINode * summary_phi = mBuilder->CreatePHI(mBitBlockType, 2, "summary");
    summary_phi->addIncoming(mZeroInitializer, ifEntryBlock);
    summary_phi->addIncoming(mCarryOutVector[carrySummaryIndex], ifBodyFinalBlock);
    mCarryOutVector[carrySummaryIndex] = summary_phi;
}

void CarryManager::generateCarryOutSummaryCode(PabloBlock & blk) {
    
    const PabloBlockCarryData & cd = blk.carryData;
    const unsigned baseCarryDataIdx = cd.getBlockCarryDataIndex();
    const unsigned carrySummaryIndex = cd.summaryCarryDataIndex();
    
    if (cd.getTotalCarryDataSize() == 1) {
        // If totalCarryDataSize == 1, then we have one pack which serves as
        // the summary.   It should already be stored.   
        return;
    }
    
    Value * carry_summary = mZeroInitializer;
    
    if (cd.blockHasLongAdvances() > 0) { // Force if entry
        carry_summary = mOneInitializer;
    }
    else {
        auto localCarryPacks = cd.getLocalCarryDataSize();
        if (localCarryPacks > 0) {
            carry_summary = mCarryOutVector[baseCarryDataIdx];
            for (auto i = 1; i < localCarryPacks; i++) {
                //carry_summary = mBuilder->CreateOr(carry_summary, mPabloBlock->mCarryOutPack[i]);            
                carry_summary = mBuilder->CreateOr(carry_summary, mCarryOutVector[baseCarryDataIdx+i]);
            }
        }
        for (Statement * stmt : blk) {
            if (If * innerIf = dyn_cast<If>(stmt)) {
                PabloBlock & inner_blk = innerIf->getBody();
                if (inner_blk.carryData.blockHasCarries()) {
                  //carry_summary = mBuilder->CreateOr(carry_summary, inner_blk.mCarryOutSummary);
                  carry_summary = mBuilder->CreateOr(carry_summary, mCarryOutVector[inner_blk.carryData.summaryCarryDataIndex()]);
                }
            }
            else if (While * innerWhile = dyn_cast<While>(stmt)) {
                PabloBlock & inner_blk = innerWhile->getBody();
                if (inner_blk.carryData.blockHasCarries()) 
                  //carry_summary = mBuilder->CreateOr(carry_summary, inner_blk.mCarryOutSummary);
                  carry_summary = mBuilder->CreateOr(carry_summary, mCarryOutVector[inner_blk.carryData.summaryCarryDataIndex()]);
            }
        }
    }
    // Calculation of the carry out summary is complete.   Store it and make it
    // available in case it must included by parent blocks.
    mCarryOutVector[carrySummaryIndex] = carry_summary;
    Value * packPtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(carrySummaryIndex));
    mBuilder->CreateAlignedStore(carry_summary, packPtr, BLOCK_SIZE/8);
}

void CarryManager::ensureCarriesLoadedLocal(PabloBlock & blk) {
#if 0
    const PabloBlockCarryData & cd = blk.carryData;
    const unsigned baseCarryDataIdx = cd.getBlockCarryDataIndex();
    const unsigned localCarryDataSize = cd.getLocalCarryDataSize();
    const unsigned totalCarryDataSize = cd.getTotalCarryDataSize();
    if (totalCarryDataSize == 0) return;
    if ((cd.getIfDepth() > 0) && (totalCarryDataSize == 1)) return;
    if (cd.getWhileDepth() > 0) return;
    for (auto i = baseCarryDataIdx; i < baseCarryDataIdx + localCarryDataSize; ++i) {
        mCarryInVector[i] = mBuilder->CreateAlignedLoad(mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(i)), BLOCK_SIZE/8, false);
    }
#endif
}

void CarryManager::ensureCarriesStoredLocal(PabloBlock & blk) {
#if 0
    const PabloBlockCarryData & cd = blk.carryData;
    const unsigned baseCarryDataIdx = cd.getBlockCarryDataIndex();
    const unsigned localCarryDataSize = cd.getLocalCarryDataSize();
    const unsigned totalCarryDataSize = cd.getTotalCarryDataSize();
    const unsigned carrySummaryIndex = cd.summaryCarryDataIndex();
    if (totalCarryDataSize == 0) return;
    if (cd.getWhileDepth() > 0) return;
    for (auto i = baseCarryDataIdx; i < baseCarryDataIdx + localCarryDataSize; ++i) {
        Value * storePtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(i));
        mBuilder->CreateAlignedStore(mCarryOutVector[i], storePtr, BLOCK_SIZE/8, false);
    }
    if (totalCarryDataSize > 1) {
        Value * summaryPtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(carrySummaryIndex));
        mBuilder->CreateAlignedStore(mCarryOutVector[carrySummaryIndex], summaryPtr, BLOCK_SIZE/8, false);
    }
#endif
}


void CarryManager::ensureCarriesLoadedRecursive(PabloBlock & whileBlk) {
    const PabloBlockCarryData & cd = whileBlk.carryData;
    const unsigned baseCarryDataIdx = cd.getBlockCarryDataIndex();
    const unsigned totalCarryDataSize = cd.getTotalCarryDataSize();
    if (cd.getWhileDepth() == 1) {
        for (auto i = baseCarryDataIdx; i < baseCarryDataIdx + totalCarryDataSize; ++i) {
            mCarryInVector[i] = mBuilder->CreateAlignedLoad(mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(i)), BLOCK_SIZE/8, false);
        }
    }
}


void CarryManager::initializeCarryDataPhisAtWhileEntry(PabloBlock & whileBlk, BasicBlock * whileEntryBlock) {
    const PabloBlockCarryData & cd = whileBlk.carryData;
    const unsigned baseCarryDataIdx = cd.getBlockCarryDataIndex();
    const unsigned totalCarryDataSize = cd.getTotalCarryDataSize();
    for (unsigned index = baseCarryDataIdx; index < baseCarryDataIdx + totalCarryDataSize; ++index) {
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


void CarryManager::extendCarryDataPhisAtWhileBodyFinalBlock(PabloBlock & whileBlk, BasicBlock * whileBodyFinalBlock) {
    const PabloBlockCarryData & cd = whileBlk.carryData;
    const unsigned baseCarryDataIdx = cd.getBlockCarryDataIndex();
    const unsigned totalCarryDataSize = cd.getTotalCarryDataSize();
    for (unsigned index = baseCarryDataIdx; index < baseCarryDataIdx + totalCarryDataSize; ++index) {
#ifdef SET_WHILE_CARRY_IN_TO_ZERO_AFTER_FIRST_ITERATION
        mCarryInPhis[index]->addIncoming(mZeroInitializer, whileBodyFinalBlock);
#endif
        PHINode * phi = mCarryOutAccumPhis[index];
        Value * carryOut = mBuilder->CreateOr(phi, mCarryOutVector[index]);
        phi->addIncoming(carryOut, whileBodyFinalBlock);
        mCarryOutVector[index] = carryOut;
    }
}

void CarryManager::ensureCarriesStoredRecursive(PabloBlock & whileBlk) {
    const PabloBlockCarryData & cd = whileBlk.carryData;
    const unsigned baseCarryDataIdx = cd.getBlockCarryDataIndex();
    const unsigned totalCarryDataSize = cd.getTotalCarryDataSize();
    if (cd.getWhileDepth() == 1) {
        for (auto i = baseCarryDataIdx; i < baseCarryDataIdx + totalCarryDataSize; ++i) {
            Value * storePtr = mBuilder->CreateGEP(mCarryDataPtr, mBuilder->getInt64(i));
            mBuilder->CreateAlignedStore(mCarryOutVector[i], storePtr, BLOCK_SIZE/8, false);
        }
    }
}

}

