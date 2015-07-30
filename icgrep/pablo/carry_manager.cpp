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
    
    unsigned totalCarryDataSize = enumerate(pb, 0, 0);
#ifdef PACKING
    mTotalCarryDataBitBlocks = (totalCarryDataSize + BLOCK_SIZE - 1)/BLOCK_SIZE;
#else
    mTotalCarryDataBitBlocks = totalCarryDataSize;
#endif
    // Carry Data area will have one extra bit block to store the block number.
    mBlockNoPtr = mBuilder->CreateBitCast(mBuilder->CreateGEP(carryPtr, mBuilder->getInt64(mTotalCarryDataBitBlocks)), Type::getInt64PtrTy(mBuilder->getContext()));
    mBlockNo = mBuilder->CreateLoad(mBlockNoPtr);
#ifdef PACKING
    unsigned totalPackCount = (totalCarryDataSize + PACK_SIZE - 1)/PACK_SIZE; 
    mCarryPackPtr.resize(totalPackCount);
    mCarryInPack.resize(totalPackCount);
    mCarryOutPack.resize(totalPackCount);
    for (auto i = 0; i < totalPackCount; i++) mCarryInPack[i]=nullptr;
#else
    mCarryPackPtr.resize(mTotalCarryDataBitBlocks);
    mCarryInPack.resize(mTotalCarryDataBitBlocks);
    mCarryOutPack.resize(mTotalCarryDataBitBlocks);
    for (auto i = 0; i < mTotalCarryDataBitBlocks; i++) mCarryInPack[i]=nullptr;
#endif    
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
    unsigned nestedOffset = cd->nested.frameOffset;
  
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
    
    cd->scopeCarryDataSize = nestedOffset;
    
    if (cd->explicitSummaryRequired()) {
        // Need extra space for the summary variable, always the last
        // entry within an if block.
#ifdef PACKING
        cd->scopeCarryDataSize = alignCeiling(cd->scopeCarryDataSize, PACK_SIZE);
        cd->summary.frameOffset = cd->scopeCarryDataSize;
        cd->scopeCarryDataSize += PACK_SIZE;
#else
        cd->summary.frameOffset = cd->scopeCarryDataSize;
        cd->scopeCarryDataSize++;
#endif
    }
    else {
        cd->summary.frameOffset = 0;
    }
    return cd->scopeCarryDataSize;
}


/* Entering and leaving blocks. */

void CarryManager::enterScope(PabloBlock * blk) {
    
    mCurrentScope = blk;
    mCarryInfo = mCarryInfoVector[blk->getScopeIndex()];
    mCurrentFrameIndex += mCarryInfo->getFrameIndex();
    //std::cerr << "enterScope:  blk->getScopeIndex() = " << blk->getScopeIndex() << ", mCurrentFrameIndex = " << mCurrentFrameIndex << std::endl;
}

void CarryManager::leaveScope() {
#ifdef PACKING
    if ((mCurrentFrameIndex % PACK_SIZE) == 0) {
        // Write out all local packs.
        auto localCarryIndex = localBasePack();
        auto localCarryPacks = mCarryInfo->getLocalCarryPackCount();
        for (auto i = localCarryIndex; i < localCarryIndex + localCarryPacks; i++) {
            storeCarryPack(i);
        }
    }
#endif
    mCurrentFrameIndex -= mCarryInfo->getFrameIndex();
    if (mCurrentScope != mPabloRoot) {
        mCurrentScope = mCurrentScope->getParent();
        mCarryInfo = mCarryInfoVector[mCurrentScope->getScopeIndex()];
    }
    //std::cerr << "leaveScope:  mCurrentFrameIndex = " << mCurrentFrameIndex << std::endl;
}


/* Helper routines */

unsigned CarryManager::absPosition(unsigned frameOffset, unsigned relPos) {
    return mCurrentFrameIndex + frameOffset + relPos;
}


unsigned CarryManager::carryOpPosition(unsigned localIndex) {
    //std::cerr << "carryOpPosition: addWithCarry.frameOffset = " << mCarryInfo->addWithCarry.frameOffset << ", localIndex = " <<localIndex << std::endl;
    return absPosition(mCarryInfo->addWithCarry.frameOffset, localIndex);
}

unsigned CarryManager::advance1Position(unsigned localIndex) {
    //std::cerr << "unsigned CarryManager::advance1Position: advance1.frameOffset = " << mCarryInfo->advance1.frameOffset << ", localIndex = " <<localIndex << std::endl;
    return absPosition(mCarryInfo->advance1.frameOffset, localIndex);
}

unsigned CarryManager::shortAdvancePosition(unsigned localIndex) {
    return absPosition(mCarryInfo->shortAdvance.frameOffset, localIndex);
}

unsigned CarryManager::longAdvanceBitBlockPosition(unsigned localIndex) {
#ifdef PACKING
    return (mCurrentFrameIndex + mCarryInfo->longAdvance.frameOffset) / BLOCK_SIZE + localIndex;
#else
    return mCurrentFrameIndex + mCarryInfo->longAdvance.frameOffset + localIndex;
#endif
}
    
unsigned CarryManager::localBasePack() {
#ifdef PACKING
    return (mCurrentFrameIndex + mCarryInfo->shortAdvance.frameOffset) / PACK_SIZE;
#else
    return mCurrentFrameIndex + mCarryInfo->shortAdvance.frameOffset;
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
    return absPosition(mCarryInfo->summary.frameOffset, 0);
}

unsigned CarryManager::summaryBits() {
#ifdef PACKING
    if (mCarryInfo->scopeCarryDataSize > PACK_SIZE) return PACK_SIZE;
    else return mCarryInfo->scopeCarryDataSize;
#else
    if (mCarryInfo->scopeCarryDataSize > 1) return PACK_SIZE;
    else return mCarryInfo->scopeCarryDataSize;
#endif
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
    //std::cerr << "storeCarryPack , pack = " << packIndex << std::endl;
    mBuilder->CreateAlignedStore(mCarryOutPack[packIndex], mCarryPackPtr[packIndex], PACK_SIZE/8);
}

/* maskSelectBitRange selects the bits of a pack from lo_bit through
 lo_bit + bitCount - 1, setting all other bits to zero.  */

Value * CarryManager::maskSelectBitRange(Value * pack, unsigned lo_bit, unsigned bitCount) {
    if (bitCount == PACK_SIZE) {
        assert(lo_bit == 0);
        return pack;
    }
    unsigned mask = (1 << bitCount) - 1;
    return mBuilder->CreateAnd(pack, mBuilder->getInt64(mask << lo_bit));
}

Value * CarryManager::getCarryInBits(unsigned carryBitPos, unsigned carryBitCount) {
    unsigned packIndex = carryBitPos / PACK_SIZE;
    unsigned packOffset = carryBitPos % PACK_SIZE;
    Value * selected = maskSelectBitRange(getCarryPack(packIndex), packOffset, carryBitCount);
    if (packOffset == 0) return selected;
    return mBuilder->CreateLShr(selected, packOffset);
}

void CarryManager::extractAndSaveCarryOutBits(Value * bitblock, unsigned carryBit_pos, unsigned carryBitCount) {
    
    unsigned packIndex = carryBit_pos / PACK_SIZE;
    unsigned packOffset = carryBit_pos % PACK_SIZE;
    unsigned rshift = PACK_SIZE - packOffset - carryBitCount;
    Value * field = iBuilder->mvmd_extract(PACK_SIZE, bitblock, BLOCK_SIZE/PACK_SIZE - 1);
    //Value * field = maskSelectBitRange(field, PACK_SIZE - carryBitCount, carryBitCount);
    if (rshift != 0) {
        field = mBuilder->CreateLShr(field, mBuilder->getInt64(rshift));
    }
    if (packOffset != 0) {
        field = mBuilder->CreateAnd(field, mBuilder->getInt64(((1<<carryBitCount) - 1) << packOffset));
    }
    if (mCarryOutPack[packIndex] == nullptr) {
        mCarryOutPack[packIndex] = field;
    }
    else {
        mCarryOutPack[packIndex] = mBuilder->CreateOr(mCarryOutPack[packIndex], field);
    }
}

Value * CarryManager::pack2bitblock(Value * pack) {
    return mBuilder->CreateBitCast(mBuilder->CreateZExt(pack, mBuilder->getIntNTy(BLOCK_SIZE)), mBitBlockType);
}
    
    /*  NOTE: In the following the mCarryOutPack is an accumulator.
    It must be created at the appropriate outer level.  */
void CarryManager::setCarryBits(unsigned carryBit_lo, unsigned carryRangeSize, Value * bits) {
    
    unsigned packIndex = carryBit_lo / PACK_SIZE;
    unsigned carryOffset = carryBit_lo % PACK_SIZE;
    if (carryOffset > 0) {
        bits = mBuilder->CreateShl(bits, mBuilder->getInt64(carryOffset));
    }
    if (mCarryOutPack[packIndex] == nullptr) {
        mCarryOutPack[packIndex] = bits;
        //std::cerr << "setCarryBits/initial , pack = " << packIndex << ", offset = " << carryOffset << ", count = " << carryRangeSize << std::endl;
    }
    else {
        //std::cerr << "setCarryBits/combine , pack = " << packIndex << ", offset = " << carryOffset << ", count = " << carryRangeSize << std::endl;
        mCarryOutPack[packIndex] = mBuilder->CreateOr(mCarryOutPack[packIndex], bits);
    }
}
    
    
/* Methods for getting and setting individual carry values. */
    
Value * CarryManager::getCarryOpCarryIn(int localIndex) {
    unsigned posn = carryOpPosition(localIndex);
#ifdef PACKING
    return pack2bitblock(getCarryInBits(posn, 1));
#else
    return getCarryPack(posn);
#endif
}

    
void CarryManager::setCarryOpCarryOut(unsigned localIndex, Value * carry_out_strm) {
    unsigned posn = carryOpPosition(localIndex);
#ifdef PACKING
    Value * field = iBuilder->mvmd_extract(PACK_SIZE, carry_out_strm, 0);
    //setCarryBits(posn, mBuilder->CreateLShr(field, mBuilder->getInt64(PACK_SIZE - 1)));
    setCarryBits(posn, 1, field);
#else
    mCarryOutPack[posn] = carry_out_strm;
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
    extractAndSaveCarryOutBits(strm, posn, 1);
    Value* carry_longint = mBuilder->CreateZExt(getCarryInBits(posn, 1), mBuilder->getIntNTy(BLOCK_SIZE));
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
    extractAndSaveCarryOutBits(strm, posn, shift_amount);
    Value* carry_longint = mBuilder->CreateZExt(getCarryInBits(posn, shift_amount), mBuilder->getIntNTy(BLOCK_SIZE));
    Value* strm_longint = mBuilder->CreateBitCast(strm, mBuilder->getIntNTy(BLOCK_SIZE));
    Value* adv_longint = mBuilder->CreateOr(mBuilder->CreateShl(strm_longint, shift_amount), carry_longint);
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
    Value * pack = getCarryPack(summary_posn/PACK_SIZE);
    Value * summary_bits = maskSelectBitRange(pack, summary_posn % PACK_SIZE, summaryBits());
    return mBuilder->CreateBitCast(mBuilder->CreateZExt(summary_bits, mBuilder->getIntNTy(BLOCK_SIZE)), mBitBlockType);
#else
    return getCarryPack(summary_posn);
#endif
}

void CarryManager::initializeCarryDataAtIfEntry() {
    if (mCarryOutPack[scopeBasePack()] == nullptr) {
        mCarryInfo->ifEntryPack = mZeroInitializer;
    }
    else {
        mCarryInfo->ifEntryPack = mCarryOutPack[scopeBasePack()];
    }
}
    
void CarryManager::buildCarryDataPhisAfterIfBody(BasicBlock * ifEntryBlock, BasicBlock * ifBodyFinalBlock) {
    unsigned const ifScopeCarrySize = mCarryInfo->scopeCarryDataSize;
    if (ifScopeCarrySize == 0) {
        // No carry data, therefore no phi nodes.
        return;
    }
#ifdef PACKING
    if (ifScopeCarrySize <= PACK_SIZE) {
        unsigned const ifPackIndex = scopeBasePack();
        PHINode * ifPack_phi = mBuilder->CreatePHI(mCarryPackType, 2, "ifPack");
        ifPack_phi->addIncoming(mCarryInfo->ifEntryPack, ifEntryBlock);
        ifPack_phi->addIncoming(mCarryOutPack[ifPackIndex], ifBodyFinalBlock);
        mCarryOutPack[ifPackIndex] = ifPack_phi;
        return;
    }
#endif
    if (mCarryInfo->getIfDepth() > 1) {
        const unsigned summaryPackIndex = summaryPosition();
        PHINode * summary_phi = mBuilder->CreatePHI(mCarryPackType, 2, "summary");
        summary_phi->addIncoming(mZeroInitializer, ifEntryBlock);
        summary_phi->addIncoming(mCarryOutPack[summaryPackIndex], ifBodyFinalBlock);
        mCarryOutPack[summaryPackIndex] = summary_phi;
    }
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

