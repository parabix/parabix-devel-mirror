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
#include <llvm/Support/CommandLine.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/CallingConv.h>
#include <llvm/IR/Function.h>


static cl::opt<CarryManagerStrategy> Strategy(cl::desc("Choose carry management strategy:"),
                                              cl::values(
                                                         clEnumVal(BitBlockStrategy, "Unpacked, each carry in a separate bitblock."),
                                                         clEnumVal(SequentialFullyPackedStrategy, "Sequential packing, up to 64 carries per pack."),
                                                         clEnumValEnd));


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

void CarryManager::initialize(Module * m, PabloBlock * pb) {
    mPabloRoot = pb;
    unsigned scopeCount = doScopeCount(pb);
    mCarryInfoVector.resize(scopeCount);
    if (Strategy == SequentialFullyPackedStrategy) {
        mPACK_SIZE = 64;
        mITEMS_PER_PACK = 64;
        mCarryPackType = mBuilder->getIntNTy(mPACK_SIZE);
        mZeroInitializer = mBuilder->getInt64(0);
        mOneInitializer = mBuilder->getInt64(-1);
    }
    else {
        mPACK_SIZE = BLOCK_SIZE;
        mITEMS_PER_PACK = 1;
        mCarryPackType = mBitBlockType;
    }
    unsigned totalCarryDataSize = enumerate(pb, 0, 0);
    
    unsigned totalPackCount = (totalCarryDataSize + mITEMS_PER_PACK - 1)/mITEMS_PER_PACK;

    mCarryPackPtr.resize(totalPackCount);
    mCarryInPack.resize(totalPackCount);
    mCarryOutPack.resize(totalPackCount);
    for (auto i = 0; i < totalPackCount; i++) mCarryInPack[i]=nullptr;

    if (Strategy == SequentialFullyPackedStrategy) {
        mTotalCarryDataBitBlocks = (totalCarryDataSize + BLOCK_SIZE - 1)/BLOCK_SIZE;       
    }
    else {
        mTotalCarryDataBitBlocks = totalCarryDataSize;
    }
    
    ArrayType* cdArrayTy = ArrayType::get(mBitBlockType, mTotalCarryDataBitBlocks);
    GlobalVariable* cdArray = new GlobalVariable(*m, cdArrayTy, /*isConstant=*/false, GlobalValue::CommonLinkage, /*Initializer=*/0, "process_block_carry_data");
    cdArray->setAlignment(BLOCK_SIZE/8);
    ConstantAggregateZero* cdInitData = ConstantAggregateZero::get(cdArrayTy);
    cdArray->setInitializer(cdInitData);
    
    mCarryPackBasePtr = mBuilder->CreateBitCast(cdArray, PointerType::get(mCarryPackType, 0));
    mCarryBitBlockPtr = mBuilder->CreateBitCast(cdArray, PointerType::get(mBitBlockType, 0));
    
    // Popcount data is stored after all the carry data.
    if (mPabloCountCount > 0) {
        ArrayType* pcArrayTy = ArrayType::get(mBuilder->getIntNTy(64), mPabloCountCount);
        GlobalVariable* pcArray = new GlobalVariable(*m, pcArrayTy, /*isConstant=*/false, GlobalValue::CommonLinkage, 0, "popcount_data");
        cdArray->setAlignment(BLOCK_SIZE/8);
        ConstantAggregateZero* pcInitData = ConstantAggregateZero::get(pcArrayTy);
        pcArray->setInitializer(pcInitData);
        mPopcountBasePtr = mBuilder->CreateBitCast(pcArray, Type::getInt64PtrTy(mBuilder->getContext()));
    }
    // Carry Data area will have one extra bit block to store the block number.
    GlobalVariable* blkNo = new GlobalVariable(*m, mBuilder->getIntNTy(64), /*isConstant=*/false, GlobalValue::CommonLinkage, 0, "blockNo");
    blkNo->setAlignment(8);
    blkNo->setInitializer(mBuilder->getInt64(0));
    mBlockNoPtr = blkNo;
    mBlockNo = mBuilder->CreateLoad(mBlockNoPtr);
    /*  Set the current scope to PabloRoot */
    mCurrentScope = mPabloRoot;
    mCurrentFrameIndex = 0;
    mCarryInfo = mCarryInfoVector[0];
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
    PabloBlockCarryData * cd = new PabloBlockCarryData(blk, mPACK_SIZE, mITEMS_PER_PACK);
    mCarryInfoVector[idx] = cd;

    cd->setIfDepth(ifDepth);
    cd->setWhileDepth(whileDepth);
    unsigned nestedOffset = cd->nested.frameOffset;
  
    for (Statement * stmt : *blk) {
        if (Count * c = dyn_cast<Count>(stmt)) {
            c->setGlobalCountIndex(mPabloCountCount);
            mPabloCountCount++;
        }
        else if (If * ifStatement = dyn_cast<If>(stmt)) {
            const unsigned ifCarryDataBits = enumerate(&ifStatement->getBody(), ifDepth+1, whileDepth);
            PabloBlockCarryData * nestedBlockData = mCarryInfoVector[ifStatement->getBody().getScopeIndex()];
            if (mITEMS_PER_PACK == mPACK_SIZE) {  // PACKING
                if (cd->roomInFinalPack(nestedOffset) < ifCarryDataBits) {
                    nestedOffset = alignCeiling(nestedOffset, mPACK_SIZE);
                }
            }
            nestedBlockData->setFramePosition(nestedOffset);

            nestedOffset += ifCarryDataBits;
            if (cd->maxNestingDepth <= nestedBlockData->maxNestingDepth) cd->maxNestingDepth = nestedBlockData->maxNestingDepth + 1;
            cd->nested.entries++;
#ifdef CARRY_DEBUG
            nestedBlockData->dumpCarryData(cerr);
#endif
        }
        else if (While * whileStatement = dyn_cast<While>(stmt)) {
            const unsigned whileCarryDataBits = enumerate(&whileStatement->getBody(), ifDepth, whileDepth+1);
            PabloBlockCarryData * nestedBlockData = mCarryInfoVector[whileStatement->getBody().getScopeIndex()];
            //if (whileStatement->isMultiCarry()) whileCarryDataBits *= whileStatement->getMaxIterations();
            if (mITEMS_PER_PACK == mPACK_SIZE) {  // PACKING
                if (cd->roomInFinalPack(nestedOffset) < whileCarryDataBits) {
                    nestedOffset = alignCeiling(nestedOffset, mPACK_SIZE);
                }
            }
            nestedBlockData->setFramePosition(nestedOffset);
            nestedOffset += whileCarryDataBits;
            if (cd->maxNestingDepth <= nestedBlockData->maxNestingDepth) cd->maxNestingDepth = nestedBlockData->maxNestingDepth + 1;
            cd->nested.entries++;
#ifdef CARRY_DEBUG
            nestedBlockData->dumpCarryData(cerr);
#endif
        }
    }
    
    cd->scopeCarryDataSize = nestedOffset;
    
    if (cd->explicitSummaryRequired()) {
        // Need extra space for the summary variable, always the last
        // entry within an if block.
        if (mITEMS_PER_PACK == mPACK_SIZE) {  // PACKING
            cd->scopeCarryDataSize = alignCeiling(cd->scopeCarryDataSize, mPACK_SIZE);
        }
        cd->summary.frameOffset = cd->scopeCarryDataSize;
        cd->scopeCarryDataSize += mITEMS_PER_PACK;  //  computed summary is a full pack.
    }
    else {
        cd->summary.frameOffset = 0;
    }
#ifdef CARRY_DEBUG
    if (cd->ifDepth == 0) cd->dumpCarryData(cerr);
#endif
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
    return (mCurrentFrameIndex + mCarryInfo->longAdvance.frameOffset) / mITEMS_PER_PACK + localIndex;
}
    
unsigned CarryManager::localBasePack() {
    return (mCurrentFrameIndex + mCarryInfo->shortAdvance.frameOffset) / mITEMS_PER_PACK;
}
    
unsigned CarryManager::scopeBasePack() {
    return mCurrentFrameIndex / mITEMS_PER_PACK;
}
    


unsigned CarryManager::summaryPosition() {
    return absPosition(mCarryInfo->summary.frameOffset, 0);
}


unsigned CarryManager::summaryPackIndex() {
    return summaryPosition()/mITEMS_PER_PACK;
}

unsigned CarryManager::summaryBits() {
    if (mCarryInfo->scopeCarryDataSize > mITEMS_PER_PACK) return mPACK_SIZE;
    else return mCarryInfo->scopeCarryDataSize;
}



Value * CarryManager::getCarryPack(unsigned packIndex) {
    if (mCarryInPack[packIndex] == nullptr) {
        Value * packPtr = mBuilder->CreateGEP(mCarryPackBasePtr, mBuilder->getInt64(packIndex));
        // Save the computed pointer - so that it can be used in storeCarryPack.
        mCarryPackPtr[packIndex] = packPtr;
        mCarryInPack[packIndex] = mBuilder->CreateAlignedLoad(packPtr, mPACK_SIZE/8);
    }
    return mCarryInPack[packIndex];
}

void CarryManager::storeCarryPack(unsigned packIndex) {
    mBuilder->CreateAlignedStore(mCarryOutPack[packIndex], mCarryPackPtr[packIndex], mPACK_SIZE/8);
}

    
/* maskSelectBitRange selects the bits of a pack from lo_bit through
   lo_bit + bitCount - 1, setting all other bits to zero.  */
    
Value * CarryManager::maskSelectBitRange(Value * pack, unsigned lo_bit, unsigned bitCount) {
    if (bitCount == mPACK_SIZE) {
        assert(lo_bit == 0);
        return pack;
    }
    uint64_t mask = ((((uint64_t) 1) << bitCount) - 1) << lo_bit;
    return mBuilder->CreateAnd(pack, mBuilder->getInt64(mask));
}
    
Value * CarryManager::getCarryInBits(unsigned carryBitPos, unsigned carryBitCount) {
    unsigned packIndex = carryBitPos / mPACK_SIZE;
    unsigned packOffset = carryBitPos % mPACK_SIZE;
    Value * selected = maskSelectBitRange(getCarryPack(packIndex), packOffset, carryBitCount);
    if (packOffset == 0) return selected;
    return mBuilder->CreateLShr(selected, packOffset);
}

void CarryManager::extractAndSaveCarryOutBits(Value * bitblock, unsigned carryBit_pos, unsigned carryBitCount) {
    unsigned packIndex = carryBit_pos / mPACK_SIZE;
    unsigned packOffset = carryBit_pos % mPACK_SIZE;
    unsigned rshift = mPACK_SIZE - packOffset - carryBitCount;
    uint64_t mask = ((((uint64_t) 1) << carryBitCount) - 1)  << packOffset;
    //std::cerr << "extractAndSaveCarryOutBits: packIndex =" << packIndex << ", packOffset = " << packOffset << ", mask = " << mask << std::endl;
    Value * field = iBuilder->mvmd_extract(mPACK_SIZE, bitblock, BLOCK_SIZE/mPACK_SIZE - 1);
    //Value * field = maskSelectBitRange(field, PACK_SIZE - carryBitCount, carryBitCount);
    if (rshift != 0) {
        field = mBuilder->CreateLShr(field, mBuilder->getInt64(rshift));
    }
    if (packOffset != 0) {
        field = mBuilder->CreateAnd(field, mBuilder->getInt64(mask));
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
    
    
/* Methods for getting and setting individual carry values. */
    
Value * CarryManager::getCarryOpCarryIn(int localIndex) {
    unsigned posn = carryOpPosition(localIndex);
    if (mITEMS_PER_PACK > 1) {// #ifdef PACKING
        return pack2bitblock(getCarryInBits(posn, 1));
    }
    else {
        return getCarryPack(posn);
    }
}

    
void CarryManager::setCarryOpCarryOut(unsigned localIndex, Value * carry_out_strm) {
    unsigned posn = carryOpPosition(localIndex);
    if (mITEMS_PER_PACK > 1) {// #ifdef PACKING
        extractAndSaveCarryOutBits(carry_out_strm, posn, 1);
    }
    else {
        Value * carry_bit = mBuilder->CreateLShr(mBuilder->CreateBitCast(carry_out_strm, mBuilder->getIntNTy(BLOCK_SIZE)), 127);
        mCarryOutPack[posn] = mBuilder->CreateBitCast(carry_bit, mBitBlockType);
        if (mCarryInfo->getWhileDepth() == 0) {
            storeCarryPack(posn);
        }
    }
}

Value* CarryManager::genShiftLeft64(Value* e) {
    Value* i128_val = mBuilder->CreateBitCast(e, mBuilder->getIntNTy(BLOCK_SIZE));
    return mBuilder->CreateBitCast(mBuilder->CreateShl(i128_val, 64), mBitBlockType);
}

Value * CarryManager::addCarryInCarryOut(int localIndex, Value* e1, Value* e2) {
    Value * carryq_value = getCarryOpCarryIn(localIndex);
    #if (BLOCK_SIZE == 128)
    //calculate carry through logical ops
    Value* carrygen = mBuilder->CreateAnd(e1, e2, "carrygen");
    Value* carryprop = mBuilder->CreateOr(e1, e2, "carryprop");
    Value* digitsum = mBuilder->CreateAdd(e1, e2, "digitsum");
    Value* partial = mBuilder->CreateAdd(digitsum, carryq_value, "partial");
    Value* digitcarry = mBuilder->CreateOr(carrygen, mBuilder->CreateAnd(carryprop, mBuilder->CreateNot(partial)));
    Value* mid_carry_in = genShiftLeft64(mBuilder->CreateLShr(digitcarry, 63));
    Value* sum = mBuilder->CreateAdd(partial, mBuilder->CreateBitCast(mid_carry_in, mBitBlockType), "sum");
    Value* carry_out_strm = mBuilder->CreateOr(carrygen, mBuilder->CreateAnd(carryprop, mBuilder->CreateNot(sum)));

    #else
    //BLOCK_SIZE == 256, there is no other implementation
    static_assert(false, "Add with carry for 256-bit bitblock requires USE_UADD_OVERFLOW");
    #endif         
    setCarryOpCarryOut(localIndex, carry_out_strm);
    return sum;
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
    if (mITEMS_PER_PACK > 1) {// #ifdef PACKING
        extractAndSaveCarryOutBits(strm, posn, 1);
        Value* carry_longint = mBuilder->CreateZExt(getCarryInBits(posn, 1), mBuilder->getIntNTy(BLOCK_SIZE));
        Value* strm_longint = mBuilder->CreateBitCast(strm, mBuilder->getIntNTy(BLOCK_SIZE));
        Value* adv_longint = mBuilder->CreateOr(mBuilder->CreateShl(strm_longint, 1), carry_longint);
        Value* result_value = mBuilder->CreateBitCast(adv_longint, mBitBlockType);
        return result_value;
    }
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
}

Value * CarryManager::shortAdvanceCarryInCarryOut(int localIndex, int shift_amount, Value * strm) {
    unsigned posn = shortAdvancePosition(localIndex);
    if (mITEMS_PER_PACK > 1) {// #ifdef PACKING
        extractAndSaveCarryOutBits(strm, posn, shift_amount);
        //std::cerr << "shortAdvanceCarryInCarryOut: posn = " << posn << ", shift_amount = " << shift_amount << std::endl;
        Value* carry_longint = mBuilder->CreateZExt(getCarryInBits(posn, shift_amount), mBuilder->getIntNTy(BLOCK_SIZE));
        Value* strm_longint = mBuilder->CreateBitCast(strm, mBuilder->getIntNTy(BLOCK_SIZE));
        Value* adv_longint = mBuilder->CreateOr(mBuilder->CreateShl(strm_longint, shift_amount), carry_longint);
        Value* result_value = mBuilder->CreateBitCast(adv_longint, mBitBlockType);
        return result_value;
    }
    mCarryOutPack[posn] = strm;
    Value * carry_in = getCarryPack(posn);
    if (mCarryInfo->getWhileDepth() == 0) {
        storeCarryPack(posn);
    }
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
    unsigned summary_index = summaryPackIndex();
    if (mITEMS_PER_PACK > 1) {// #ifdef PACKING
        Value * pack = getCarryPack(summary_index);
        Value * summary_bits = maskSelectBitRange(pack, summaryPosition() % mPACK_SIZE, summaryBits());
        return mBuilder->CreateBitCast(mBuilder->CreateZExt(summary_bits, mBuilder->getIntNTy(BLOCK_SIZE)), mBitBlockType);
    }
    else {
        return getCarryPack(summary_index);
    }
}

void CarryManager::initializeCarryDataAtIfEntry() {
    if (blockHasCarries()) {
        if (mCarryOutPack[scopeBasePack()] == nullptr) {
            mCarryInfo->ifEntryPack = mZeroInitializer;
        }
        else {
            mCarryInfo->ifEntryPack = mCarryOutPack[scopeBasePack()];
        }
    }
}
    
void CarryManager::buildCarryDataPhisAfterIfBody(BasicBlock * ifEntryBlock, BasicBlock * ifBodyFinalBlock) {
    unsigned const ifScopeCarrySize = mCarryInfo->scopeCarryDataSize;
    if (ifScopeCarrySize == 0) {
        // No carry data, therefore no phi nodes.
        return;
    }
    if (mITEMS_PER_PACK > 1) {// #ifdef PACKING
        if (ifScopeCarrySize <= mPACK_SIZE) {
            unsigned const ifPackIndex = scopeBasePack();
            PHINode * ifPack_phi = mBuilder->CreatePHI(mCarryPackType, 2, "ifPack");
            ifPack_phi->addIncoming(mCarryInfo->ifEntryPack, ifEntryBlock);
            ifPack_phi->addIncoming(mCarryOutPack[ifPackIndex], ifBodyFinalBlock);
            mCarryOutPack[ifPackIndex] = ifPack_phi;
            return;
        }
    }
    if (mCarryInfo->getIfDepth() > 1) {
        const unsigned summaryIndex = summaryPackIndex();
        PHINode * summary_phi = mBuilder->CreatePHI(mCarryPackType, 2, "summary");
        summary_phi->addIncoming(mZeroInitializer, ifEntryBlock);
        summary_phi->addIncoming(mCarryOutPack[summaryIndex], ifBodyFinalBlock);
        mCarryOutPack[summaryIndex] = summary_phi;
    }
}
    
void CarryManager::addSummaryPhiIfNeeded(BasicBlock * ifEntryBlock, BasicBlock * ifBodyFinalBlock) {
    if ((mCarryInfo->getIfDepth() <= 1) || !mCarryInfo->blockHasCarries()){
        // For ifDepth == 1, the parent does not need a summary as it is not itself within an if.
        // Therefore, it doesn't need access to this block's summary in building its own.
        return;
    }
    const unsigned carrySummaryIndex = summaryPackIndex();
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
    
    const unsigned carrySummaryIndex = summaryPackIndex();
    
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
                carry_summary = mBuilder->CreateOr(carry_summary, mCarryOutPack[localCarryIndex+i]);
            }
        }
        for (Statement * stmt : *mCurrentScope) {
            if (If * innerIf = dyn_cast<If>(stmt)) {
                PabloBlock * inner_blk = & innerIf->getBody();
                enterScope(inner_blk);
                if (blockHasCarries()) {
                  carry_summary = mBuilder->CreateOr(carry_summary, mCarryOutPack[summaryPackIndex()]);
                }
                leaveScope();
            }
            else if (While * innerWhile = dyn_cast<While>(stmt)) {
                PabloBlock * inner_blk = & innerWhile->getBody();
                enterScope(inner_blk);
                if (blockHasCarries()) {
                    carry_summary = mBuilder->CreateOr(carry_summary, mCarryOutPack[summaryPackIndex()]);
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

/* Store all the full carry packs generated locally in this scope or the
   single full pack for this scope*/
void CarryManager::ensureCarriesStoredLocal() {
    if (mITEMS_PER_PACK > 1) {// #ifdef PACKING
        const unsigned scopeCarryPacks = mCarryInfo->getScopeCarryPackCount();
        if ((scopeCarryPacks > 0) && ((mCurrentFrameIndex % mPACK_SIZE) == 0)) {
            // We have carry data and we are not in the middle of a pack.
            // Write out all local packs.
            auto localCarryIndex = localBasePack();
            auto localCarryPacks = mCarryInfo->getLocalCarryPackCount();
            for (auto i = localCarryIndex; i < localCarryIndex + localCarryPacks; i++) {
                storeCarryPack(i);
            }
            if ((localCarryPacks == 0) && (scopeCarryPacks == 1) && (mCarryInfo->nested.entries > 1)) {
                storeCarryPack(localCarryIndex);
            }
        }
    }
}

Value * CarryManager::popCount(Value * to_count, unsigned globalIdx) {
    Value * countPtr = mBuilder->CreateGEP(mPopcountBasePtr, mBuilder->getInt64(globalIdx));
    Value * countSoFar = mBuilder->CreateAlignedLoad(countPtr, 8);
    Value * fieldCounts = iBuilder->simd_popcount(64, to_count);
    for (int i = 0; i < BLOCK_SIZE/64; i++) {
        countSoFar = mBuilder->CreateAdd(countSoFar, iBuilder->mvmd_extract(64, fieldCounts, i));
    }
    mBuilder->CreateAlignedStore(countSoFar, countPtr, 8);
    return mBuilder->CreateBitCast(mBuilder->CreateZExt(countSoFar, mBuilder->getIntNTy(BLOCK_SIZE)), mBitBlockType);
}

CarryManager::~CarryManager() {
    for (auto * cd : mCarryInfoVector) {
        delete cd;
    }
}

}

