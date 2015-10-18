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
    
void CarryManager::generateCarryDataInitializer(Module * m) {
    FunctionType * functionType = FunctionType::get(Type::getVoidTy(m->getContext()), std::vector<Type *>({}), false);
    SmallVector<AttributeSet, 1> Attrs;
    Attrs.push_back(AttributeSet::get(m->getContext(), ~0U, std::vector<Attribute::AttrKind>({ Attribute::NoUnwind, Attribute::UWTable })));
    AttributeSet AttrSet = AttributeSet::get(m->getContext(), Attrs);
    
    // Create the function that will be generated.
    Function * f = Function::Create(functionType, GlobalValue::ExternalLinkage, "process_block_initialize_carries", m);
    f->setCallingConv(CallingConv::C);
    f->setAttributes(AttrSet);
    llvm::IRBuilderBase::InsertPoint ip = mBuilder->saveIP();
    mBuilder->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry1", f,0));
    mBuilder->CreateMemSet(mCarryBitBlockPtr, mBuilder->getInt8(0), mTotalCarryDataBitBlocks * mBITBLOCK_WIDTH/8, 4);
    ReturnInst::Create(m->getContext(), mBuilder->GetInsertBlock());
    mBuilder->restoreIP(ip);
}
    
    

void CarryManager::initialize(Module * m, PabloBlock * pb) {
    mPabloRoot = pb;
    unsigned scopeCount = doScopeCount(pb);
    mCarryInfoVector.resize(scopeCount);
    if (Strategy == SequentialFullyPackedStrategy) {
        mPACK_SIZE = 64;
        mITEMS_PER_PACK = 64;
        mCarryPackType = mBuilder->getIntNTy(mPACK_SIZE);
        mPackBuilder = new IDISA::IDISA_Builder(mCarryPackType);
        mPackBuilder->initialize(m, mBuilder);
    }
    else {
        mPACK_SIZE = mBITBLOCK_WIDTH;
        mITEMS_PER_PACK = 1;
        mCarryPackType = mBitBlockType;
    }
    unsigned totalCarryDataSize = enumerate(pb, 0, 0);
    
    unsigned totalPackCount = (totalCarryDataSize + mITEMS_PER_PACK - 1)/mITEMS_PER_PACK;

    mCarryPackPtr.resize(totalPackCount);
    mCarryInPack.resize(totalPackCount);
    mCarryOutPack.resize(totalPackCount);
    for (unsigned i = 0; i < totalPackCount; i++) mCarryInPack[i]=nullptr;

    if (Strategy == SequentialFullyPackedStrategy) {
        mTotalCarryDataBitBlocks = (totalCarryDataSize + mBITBLOCK_WIDTH - 1)/mBITBLOCK_WIDTH;       
    }
    else {
        mTotalCarryDataBitBlocks = totalCarryDataSize;
    }
    
    ArrayType* cdArrayTy = ArrayType::get(mBitBlockType, mTotalCarryDataBitBlocks);
    GlobalVariable* cdArray = new GlobalVariable(*m, cdArrayTy, /*isConstant=*/false, GlobalValue::CommonLinkage, /*Initializer=*/0, "process_block_carry_data");
    cdArray->setAlignment(mBITBLOCK_WIDTH/8);
    ConstantAggregateZero* cdInitData = ConstantAggregateZero::get(cdArrayTy);
    cdArray->setInitializer(cdInitData);
    
    mCarryPackBasePtr = mBuilder->CreateBitCast(cdArray, PointerType::get(mCarryPackType, 0));
    mCarryBitBlockPtr = mBuilder->CreateBitCast(cdArray, PointerType::get(mBitBlockType, 0));
    
    generateCarryDataInitializer(m);
    
    // Popcount data is stored after all the carry data.
    if (mPabloCountCount > 0) {
        ArrayType* pcArrayTy = ArrayType::get(mBuilder->getIntNTy(64), mPabloCountCount);
        GlobalVariable* pcArray = new GlobalVariable(*m, pcArrayTy, /*isConstant=*/false, GlobalValue::CommonLinkage, 0, "popcount_data");
        cdArray->setAlignment(mBITBLOCK_WIDTH/8);
        ConstantAggregateZero* pcInitData = ConstantAggregateZero::get(pcArrayTy);
        pcArray->setInitializer(pcInitData);
        mPopcountBasePtr = mBuilder->CreateBitCast(pcArray, Type::getInt64PtrTy(mBuilder->getContext()));
    }
    // Carry Data area will have one extra bit block to store the block number.
    GlobalVariable* blkNo = new GlobalVariable(*m, mBuilder->getIntNTy(64), /*isConstant=*/false, GlobalValue::CommonLinkage, 0, "blockNo");
    blkNo->setAlignment(16);
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
    mBuilder->CreateAlignedStore(mPackBuilder->bitCast(mCarryOutPack[packIndex]), mCarryPackPtr[packIndex], mPACK_SIZE/8);
}

    
/* maskSelectBitRange selects the bits of a pack from lo_bit through
   lo_bit + bitCount - 1, setting all other bits to zero.  */
    
Value * CarryManager::maskSelectBitRange(Value * pack, unsigned lo_bit, unsigned bitCount) {
    if (bitCount == mPACK_SIZE) {
        assert(lo_bit == 0);
        return pack;
    }
    uint64_t mask = ((((uint64_t) 1) << bitCount) - 1) << lo_bit;
    return mPackBuilder->simd_and(pack, mBuilder->getInt64(mask));
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
    Value * field = iBuilder->mvmd_extract(mPACK_SIZE, bitblock, mBITBLOCK_WIDTH/mPACK_SIZE - 1);
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
        mCarryOutPack[packIndex] = mPackBuilder->simd_or(mCarryOutPack[packIndex], field);
    }
}

Value * CarryManager::pack2bitblock(Value * pack) {
    return iBuilder->bitCast(mBuilder->CreateZExt(pack, mBuilder->getIntNTy(mBITBLOCK_WIDTH)));
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
        Value * carry_bit = mBuilder->CreateLShr(mBuilder->CreateBitCast(carry_out_strm, mBuilder->getIntNTy(mBITBLOCK_WIDTH)), mBITBLOCK_WIDTH-1);
        mCarryOutPack[posn] = mBuilder->CreateBitCast(carry_bit, mBitBlockType);
        if (mCarryInfo->getWhileDepth() == 0) {
            storeCarryPack(posn);
        }
    }
}

Value * CarryManager::addCarryInCarryOut(int localIndex, Value* e1, Value* e2) {
    if (mBITBLOCK_WIDTH == 128) {
        Value * carryq_value = getCarryOpCarryIn(localIndex);
        //calculate carry through logical ops
        Value* carrygen = iBuilder->simd_and(e1, e2);
        Value* carryprop = iBuilder->simd_or(e1, e2);
        Value* digitsum = iBuilder->simd_add(64, e1, e2);
        Value* partial = iBuilder->simd_add(64, digitsum, carryq_value);
        Value* digitcarry = iBuilder->simd_or(carrygen, iBuilder->simd_and(carryprop, mBuilder->CreateNot(partial)));
        Value* mid_carry_in = iBuilder->simd_slli(128, mBuilder->CreateLShr(digitcarry, 63), 64);
        Value* sum = iBuilder->simd_add(64, partial, mBuilder->CreateBitCast(mid_carry_in, mBitBlockType));
        Value* carry_out_strm = iBuilder->simd_or(carrygen, iBuilder->simd_and(carryprop, mBuilder->CreateNot(sum)));
        setCarryOpCarryOut(localIndex, carry_out_strm);
        return sum;
    }
    else {
        Value * carryq_value = getCarryOpCarryIn(localIndex);
        Value* carrygen = iBuilder->simd_and(e1, e2);
        Value* carryprop = iBuilder->simd_or(e1, e2);
        Value * sum = iBuilder->simd_add(mBITBLOCK_WIDTH, iBuilder->simd_add(mBITBLOCK_WIDTH, e1, e2), carryq_value);
        Value* carry_out_strm = iBuilder->simd_or(carrygen, iBuilder->simd_and(carryprop, mBuilder->CreateNot(sum)));
        setCarryOpCarryOut(localIndex, carry_out_strm);
        return sum;
    }
}


Value * CarryManager::advanceCarryInCarryOut(int localIndex, unsigned shift_amount, Value * strm) {
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

#define DSSLI_FIELDWIDTH 64

Value * CarryManager::unitAdvanceCarryInCarryOut(int localIndex, Value * strm) {
    unsigned posn = advance1Position(localIndex);
    if (mITEMS_PER_PACK > 1) {// #ifdef PACKING
        extractAndSaveCarryOutBits(strm, posn, 1);
        Value* carry_longint = mBuilder->CreateZExt(getCarryInBits(posn, 1), mBuilder->getIntNTy(mBITBLOCK_WIDTH));
        Value* strm_longint = mBuilder->CreateBitCast(strm, mBuilder->getIntNTy(mBITBLOCK_WIDTH));
        Value* adv_longint = mBuilder->CreateOr(mBuilder->CreateShl(strm_longint, 1), carry_longint);
        Value* result_value = mBuilder->CreateBitCast(adv_longint, mBitBlockType);
        return result_value;
    }
    mCarryOutPack[posn] = strm;
    Value * carry_in = getCarryPack(posn);
    if (mCarryInfo->getWhileDepth() == 0) {
        storeCarryPack(posn);
    }
    Value * ahead = iBuilder->mvmd_dslli(DSSLI_FIELDWIDTH, strm, carry_in, iBuilder->getBitBlockWidth()/DSSLI_FIELDWIDTH -1);
    return iBuilder->simd_or(iBuilder->simd_srli(DSSLI_FIELDWIDTH, ahead, DSSLI_FIELDWIDTH-1), iBuilder->simd_slli(DSSLI_FIELDWIDTH, strm, 1));
}

Value * CarryManager::shortAdvanceCarryInCarryOut(int localIndex, unsigned shift_amount, Value * strm) {
    unsigned posn = shortAdvancePosition(localIndex);
    if (mITEMS_PER_PACK > 1) {// #ifdef PACKING
        extractAndSaveCarryOutBits(strm, posn, shift_amount);
        //std::cerr << "shortAdvanceCarryInCarryOut: posn = " << posn << ", shift_amount = " << shift_amount << std::endl;
        Value* carry_longint = mBuilder->CreateZExt(getCarryInBits(posn, shift_amount), mBuilder->getIntNTy(mBITBLOCK_WIDTH));
        Value* strm_longint = mBuilder->CreateBitCast(strm, mBuilder->getIntNTy(mBITBLOCK_WIDTH));
        Value* adv_longint = mBuilder->CreateOr(mBuilder->CreateShl(strm_longint, shift_amount), carry_longint);
        Value* result_value = mBuilder->CreateBitCast(adv_longint, mBitBlockType);
        return result_value;
    }
    mCarryOutPack[posn] = strm;
    Value * carry_in = getCarryPack(posn);
    if (mCarryInfo->getWhileDepth() == 0) {
        storeCarryPack(posn);
    }
    // Use a single whole-byte shift, if possible.
    if (shift_amount % 8 == 0) {
        return iBuilder->mvmd_dslli(8, strm, carry_in, iBuilder->getBitBlockWidth()/8 - shift_amount/8);
    }
    else if (shift_amount < DSSLI_FIELDWIDTH) {
        Value * ahead = iBuilder->mvmd_dslli(DSSLI_FIELDWIDTH, strm, carry_in, iBuilder->getBitBlockWidth()/DSSLI_FIELDWIDTH - 1);
        return iBuilder->simd_or(iBuilder->simd_srli(DSSLI_FIELDWIDTH, ahead, DSSLI_FIELDWIDTH-shift_amount), iBuilder->simd_slli(DSSLI_FIELDWIDTH, strm, shift_amount));
    }
    Value* advanceq_longint = mBuilder->CreateBitCast(carry_in, mBuilder->getIntNTy(mBITBLOCK_WIDTH));
    Value* strm_longint = mBuilder->CreateBitCast(strm, mBuilder->getIntNTy(mBITBLOCK_WIDTH));
    Value* adv_longint = mBuilder->CreateOr(mBuilder->CreateShl(strm_longint, shift_amount), mBuilder->CreateLShr(advanceq_longint, mBITBLOCK_WIDTH - shift_amount), "advance");
    return mBuilder->CreateBitCast(adv_longint, mBitBlockType);
}
    

/*  currently defined in carry_data.h 
 
 static unsigned power2ceil (unsigned v) {
 unsigned ceil = 1;
 while (ceil < v) ceil *= 2;
 return ceil;
 }
 
 unsigned longAdvanceEntries(unsigned shift_amount) const {
 return (shift_amount + mBITBLOCK_WIDTH - 1)/mBITBLOCK_WIDTH;
 }
 
 unsigned longAdvanceBufferSize(unsigned shift_amount)  const {
 return power2ceil(longAdvanceEntries(shift_amount));
 }
 */

    
Value * CarryManager::longAdvanceCarryInCarryOut(int localIndex, unsigned shift_amount, Value * carry_out) {
    unsigned carryDataIndex = longAdvanceBitBlockPosition(localIndex);
    Value * advBaseIndex = mBuilder->getInt64(carryDataIndex);
    if (shift_amount <= mBITBLOCK_WIDTH) {
        // special case using a single buffer entry and the carry_out value.
        Value * advanceDataPtr = mBuilder->CreateGEP(mCarryBitBlockPtr, advBaseIndex);
        Value * carry_block0 = mBuilder->CreateAlignedLoad(advanceDataPtr, mBITBLOCK_WIDTH/8);
        mBuilder->CreateAlignedStore(carry_out, advanceDataPtr, mBITBLOCK_WIDTH/8);
        /* Very special case - no combine */
        if (shift_amount == mBITBLOCK_WIDTH) return carry_block0;
        Value* block0_shr = mBuilder->CreateLShr(mBuilder->CreateBitCast(carry_block0, mBuilder->getIntNTy(mBITBLOCK_WIDTH)), mBITBLOCK_WIDTH - shift_amount);
        Value* block1_shl = mBuilder->CreateShl(mBuilder->CreateBitCast(carry_out, mBuilder->getIntNTy(mBITBLOCK_WIDTH)), shift_amount);
        return mBuilder->CreateBitCast(mBuilder->CreateOr(block1_shl, block0_shr), mBitBlockType);
    }
    // We need a buffer of at least two elements for storing the advance data.
    const unsigned block_shift = shift_amount % mBITBLOCK_WIDTH;
    const unsigned advanceEntries = mCarryInfo->longAdvanceEntries(shift_amount);
    const unsigned bufsize = mCarryInfo->longAdvanceBufferSize(shift_amount);
    Value * indexMask = mBuilder->getInt64(bufsize - 1);  // A mask to implement circular buffer indexing
    Value * loadIndex0 = mBuilder->CreateAdd(mBuilder->CreateAnd(mBuilder->CreateSub(mBlockNo, mBuilder->getInt64(advanceEntries)), indexMask), advBaseIndex);
    Value * storeIndex = mBuilder->CreateAdd(mBuilder->CreateAnd(mBlockNo, indexMask), advBaseIndex);
    Value * carry_block0 = mBuilder->CreateAlignedLoad(mBuilder->CreateGEP(mCarryBitBlockPtr, loadIndex0), mBITBLOCK_WIDTH/8);
    // If the long advance is an exact multiple of mBITBLOCK_WIDTH, we simply return the oldest 
    // block in the long advance carry data area.  
    if (block_shift == 0) {
        mBuilder->CreateAlignedStore(carry_out, mBuilder->CreateGEP(mCarryBitBlockPtr, storeIndex), mBITBLOCK_WIDTH/8);
        return carry_block0;
    }
    // Otherwise we need to combine data from the two oldest blocks.
    Value * loadIndex1 = mBuilder->CreateAdd(mBuilder->CreateAnd(mBuilder->CreateSub(mBlockNo, mBuilder->getInt64(advanceEntries-1)), indexMask), advBaseIndex);
    Value * carry_block1 = mBuilder->CreateAlignedLoad(mBuilder->CreateGEP(mCarryBitBlockPtr, loadIndex1), mBITBLOCK_WIDTH/8);
    Value* block0_shr = mBuilder->CreateLShr(mBuilder->CreateBitCast(carry_block0, mBuilder->getIntNTy(mBITBLOCK_WIDTH)), mBITBLOCK_WIDTH - block_shift);
    Value* block1_shl = mBuilder->CreateShl(mBuilder->CreateBitCast(carry_block1, mBuilder->getIntNTy(mBITBLOCK_WIDTH)), block_shift);
    mBuilder->CreateAlignedStore(carry_out, mBuilder->CreateGEP(mCarryBitBlockPtr, storeIndex), mBITBLOCK_WIDTH/8);
    return mBuilder->CreateBitCast(mBuilder->CreateOr(block1_shl, block0_shr), mBitBlockType);
}
    

/* Methods for getting and setting carry summary values */
   
bool CarryManager::blockHasCarries(){
    return mCarryInfo->blockHasCarries();
} 


Value * CarryManager::generateBitBlockOrSummaryTest(Value * bitblock) {
    Value * test_expr = bitblock;
    if (mCarryInfo->blockHasCarries()) {
        Value * summary_pack = getCarryPack(summaryPackIndex());
        if (mITEMS_PER_PACK > 1) {// #ifdef PACKING
            Value * summary_bits = maskSelectBitRange(summary_pack, summaryPosition() % mPACK_SIZE, summaryBits());
            test_expr = iBuilder->simd_or(test_expr, mBuilder->CreateZExt(summary_bits, mBuilder->getIntNTy(mBITBLOCK_WIDTH)));
        }
        else {
            test_expr = iBuilder->simd_or(test_expr, summary_pack);
        }
    }
    return iBuilder->bitblock_any(test_expr);
}

void CarryManager::initializeCarryDataAtIfEntry() {
    if (blockHasCarries()) {
        if (mCarryOutPack[scopeBasePack()] == nullptr) {
            mCarryInfo->ifEntryPack = mPackBuilder->allZeroes();
        }
        else {
            mCarryInfo->ifEntryPack = mCarryOutPack[scopeBasePack()];
        }
    }
}
    
void CarryManager::buildCarryDataPhisAfterIfBody(BasicBlock * ifEntryBlock, BasicBlock * ifBodyFinalBlock) {
    if (mCarryInfo->getWhileDepth() > 0) {
        // We need to phi out everything for the while carry accumulation process.
        const unsigned scopeCarryPacks = mCarryInfo->getScopeCarryPackCount();
        const unsigned currentScopeBase = scopeBasePack();
        for (unsigned index = currentScopeBase; index < currentScopeBase + scopeCarryPacks; ++index) {
            PHINode * phi_out = mBuilder->CreatePHI(mCarryPackType, 2);
            phi_out->addIncoming(mPackBuilder->allZeroes(),ifEntryBlock);
            phi_out->addIncoming(mPackBuilder->bitCast(mCarryOutPack[index]), ifBodyFinalBlock);
            mCarryOutPack[index] = phi_out;
        }
        return;
    }
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
            ifPack_phi->addIncoming(mPackBuilder->bitCast(mCarryOutPack[ifPackIndex]), ifBodyFinalBlock);
            mCarryOutPack[ifPackIndex] = ifPack_phi;
            return;
        }
    }
    if (mCarryInfo->getIfDepth() > 1) {
        // Our parent block is also an if.  It needs access to our summary to compute
        // its own summary.
        const unsigned summaryIndex = summaryPackIndex();
        PHINode * summary_phi = mBuilder->CreatePHI(mCarryPackType, 2, "summary");
        summary_phi->addIncoming(mPackBuilder->allZeroes(), ifEntryBlock);
        summary_phi->addIncoming(mPackBuilder->bitCast(mCarryOutPack[summaryIndex]), ifBodyFinalBlock);
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
    summary_phi->addIncoming(mPackBuilder->allZeroes(), ifEntryBlock);
    summary_phi->addIncoming(mPackBuilder->bitCast(mCarryOutPack[carrySummaryIndex]), ifBodyFinalBlock);
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
    
    Value * carry_summary = mPackBuilder->allZeroes();
    if (mCarryInfo->blockHasLongAdvances()) { // Force if entry
        carry_summary = mPackBuilder->allOnes();
    }
    else {
        unsigned localCarryIndex = localBasePack();
        unsigned localCarryPacks = mCarryInfo->getLocalCarryPackCount();
        if (localCarryPacks > 0) {
            carry_summary = mCarryOutPack[localCarryIndex];
            for (unsigned i = 1; i < localCarryPacks; i++) {
                carry_summary = mPackBuilder->simd_or(carry_summary, mCarryOutPack[localCarryIndex+i]);
            }
        }
        for (Statement * stmt : *mCurrentScope) {
            if (If * innerIf = dyn_cast<If>(stmt)) {
                PabloBlock * inner_blk = & innerIf->getBody();
                enterScope(inner_blk);
                if (blockHasCarries()) {
                  carry_summary = mPackBuilder->simd_or(carry_summary, mCarryOutPack[summaryPackIndex()]);
                }
                leaveScope();
            }
            else if (While * innerWhile = dyn_cast<While>(stmt)) {
                PabloBlock * inner_blk = & innerWhile->getBody();
                enterScope(inner_blk);
                if (blockHasCarries()) {
                    carry_summary = mPackBuilder->simd_or(carry_summary, mCarryOutPack[summaryPackIndex()]);
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
        phi_out->addIncoming(mPackBuilder->allZeroes(), whileEntryBlock);
        mCarryOutAccumPhis[index] = phi_out;
    }
}


void CarryManager::extendCarryDataPhisAtWhileBodyFinalBlock(BasicBlock * whileBodyFinalBlock) {
    const unsigned scopeCarryPacks = mCarryInfo->getScopeCarryPackCount();
    const unsigned currentScopeBase = scopeBasePack();
    for (unsigned index = 0; index < scopeCarryPacks; ++index) {
#ifdef SET_WHILE_CARRY_IN_TO_ZERO_AFTER_FIRST_ITERATION
        mCarryInPhis[index]->addIncoming(mPackBuilder->allZeroes(), whileBodyFinalBlock);
#endif
        PHINode * phi = mCarryOutAccumPhis[index];
        Value * carryOut = mPackBuilder->simd_or(phi, mCarryOutPack[currentScopeBase+index]);
        phi->addIncoming(mPackBuilder->bitCast(carryOut), whileBodyFinalBlock);
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
    for (int i = 0; i < mBITBLOCK_WIDTH/64; i++) {
        countSoFar = mBuilder->CreateAdd(countSoFar, iBuilder->mvmd_extract(64, fieldCounts, i));
    }
    mBuilder->CreateAlignedStore(countSoFar, countPtr, 8);
    return iBuilder->bitCast(mBuilder->CreateZExt(countSoFar, mBuilder->getIntNTy(mBITBLOCK_WIDTH)));
}

CarryManager::~CarryManager() {
    for (auto * cd : mCarryInfoVector) {
        delete cd;
    }
}

}

