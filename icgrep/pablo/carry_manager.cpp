/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <stdexcept>
#include <pablo/carry_data.h>
#include <pablo/codegenstate.h>
#include <pablo/carry_manager.h>
#include <pablo/pabloAST.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/CallingConv.h>
#include <llvm/IR/Function.h>

#define DSSLI_FIELDWIDTH 64

namespace pablo {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief doScopeCount
 ** ------------------------------------------------------------------------------------------------------------- */
static unsigned doScopeCount(const PabloBlock * const pb) {
    unsigned count = 1;
    for (const Statement * stmt : *pb) {
        if (LLVM_UNLIKELY(isa<If>(stmt))) {
            count += doScopeCount(cast<If>(stmt)->getBody());
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            count += doScopeCount(cast<While>(stmt)->getBody());
        }
    }
    return count;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initialize
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::initialize(PabloBlock * pb, KernelBuilder * kBuilder) {
    mRootScope = pb;
    mCarryInfoVector.resize(doScopeCount(pb));
    mCarryPackType = mBitBlockType;

    const unsigned totalCarryDataSize = enumerate(pb, 0, 0);

    mCarryPackPtr.resize(totalCarryDataSize, nullptr);
    mCarryInPack.resize(totalCarryDataSize, nullptr);
    mCarryOutPack.resize(totalCarryDataSize, nullptr);

    mTotalCarryDataBitBlocks = totalCarryDataSize;
    
    ArrayType* cdArrayTy = ArrayType::get(mBitBlockType, mTotalCarryDataBitBlocks);
    mCdArrayIdx = kBuilder->extendKernelInternalStateType(cdArrayTy);
    
    if (mPabloCountCount > 0) {
        ArrayType* pcArrayTy = ArrayType::get(iBuilder->getIntNTy(64), mPabloCountCount);
        mPcArrayIdx = kBuilder->extendKernelInternalStateType(pcArrayTy);
    }
  
    mCurrentScope = mRootScope;
    mCurrentFrameIndex = 0;
    mCarryInfo = mCarryInfoVector[0];
    mCarryOutPack[summaryPack()] = Constant::getNullValue(mCarryPackType);
}

void CarryManager::initialize_setPtrs(KernelBuilder * kBuilder) {

    Value * kernelStuctParam = kBuilder->getKernelStructParam();
    Value * cdArrayPtr = kBuilder->getKernelInternalStatePtr(kernelStuctParam, mCdArrayIdx);
  
    mCarryPackBasePtr = iBuilder->CreateBitCast(cdArrayPtr, PointerType::get(mCarryPackType, 0));
    mCarryBitBlockPtr = iBuilder->CreateBitCast(cdArrayPtr, PointerType::get(mBitBlockType, 0));   
    
    if (mPabloCountCount > 0) {
        Value * pcArrayPtr = kBuilder->getKernelInternalStatePtr(kernelStuctParam, mPcArrayIdx);
        mPopcountBasePtr = iBuilder->CreateBitCast(pcArrayPtr, Type::getInt64PtrTy(iBuilder->getContext()));
    }
  
    mBlockNo = iBuilder->CreateUDiv(kBuilder->getKernelInternalState(kernelStuctParam, mFilePosIdx), iBuilder->getInt64(mBitBlockWidth));
    mCurrentScope = mRootScope;
    mCurrentFrameIndex = 0;
    mCarryInfo = mCarryInfoVector[0];
    mCarryOutPack[summaryPack()] = Constant::getNullValue(mCarryPackType);
}

void CarryManager::set_BlockNo(KernelBuilder * kBuilder){
    Value * kernelStuctParam = kBuilder->getKernelStructParam();
    mBlockNo = iBuilder->CreateUDiv(kBuilder->getKernelInternalState(kernelStuctParam, mFilePosIdx), iBuilder->getInt64(mBitBlockWidth));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterScope(PabloBlock * const scope) {
    Value * summaryCarry = mCarryOutPack[summaryPack()];
    mCarrySummary.push_back(summaryCarry);
    mCurrentScope = scope;
    mCarryInfo = mCarryInfoVector[scope->getScopeIndex()];
    mCurrentFrameIndex += mCarryInfo->getFrameIndex();
    mCarryOutPack[summaryPack()] = Constant::getNullValue(mCarryPackType);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveScope() {
    Value * summaryCarry = mCarryOutPack[summaryPack()];
    assert (mCurrentScope != mRootScope);
    mCurrentFrameIndex -= mCarryInfo->getFrameIndex();
    mCurrentScope = mCurrentScope->getParent();
    mCarryInfo = mCarryInfoVector[mCurrentScope->getScopeIndex()];
    mCarryOutPack[summaryPack()] = summaryCarry;
    mCarrySummary.pop_back();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::addCarryInCarryOut(const unsigned localIndex, Value * const e1, Value * const e2) {
    Value * sum = nullptr;
    if (mBitBlockWidth == 128) {
        Value * carryq_value = getCarryIn(localIndex);
        //calculate carry through logical ops
        Value * carrygen = iBuilder->simd_and(e1, e2);
        Value * carryprop = iBuilder->simd_or(e1, e2);
        Value * digitsum = iBuilder->simd_add(64, e1, e2);
        Value * partial = iBuilder->simd_add(64, digitsum, carryq_value);
        Value * digitcarry = iBuilder->simd_or(carrygen, iBuilder->simd_and(carryprop, iBuilder->CreateNot(partial)));
        Value * mid_carry_in = iBuilder->simd_slli(128, iBuilder->CreateLShr(digitcarry, 63), 64);
        sum = iBuilder->simd_add(64, partial, iBuilder->CreateBitCast(mid_carry_in, mBitBlockType));
        Value * carry_out_strm = iBuilder->simd_or(carrygen, iBuilder->simd_and(carryprop, iBuilder->CreateNot(sum)));
        setCarryOut(localIndex, carry_out_strm);
    } else if (mBitBlockWidth == 256) {
        // using LONG_ADD
        Value * carryq_value = getCarryIn(localIndex);
        Value * carryin = iBuilder->mvmd_extract(32, carryq_value, 0);
        Value * carrygen = iBuilder->simd_and(e1, e2);
        Value * carryprop = iBuilder->simd_or(e1, e2);
        Value * digitsum = iBuilder->simd_add(64, e1, e2);
        Value * digitcarry = iBuilder->simd_or(carrygen, iBuilder->simd_and(carryprop, iBuilder->CreateNot(digitsum)));
        Value * carryMask = iBuilder->hsimd_signmask(64, digitcarry);
        Value * carryMask2 = iBuilder->CreateOr(iBuilder->CreateAdd(carryMask, carryMask), carryin);
        Value * bubble = iBuilder->simd_eq(64, digitsum, iBuilder->allOnes());
        Value * bubbleMask = iBuilder->hsimd_signmask(64, bubble);
        Value * incrementMask = iBuilder->CreateXor(iBuilder->CreateAdd(bubbleMask, carryMask2), bubbleMask);
        Value * increments = iBuilder->esimd_bitspread(64,incrementMask);
        sum = iBuilder->simd_add(64, digitsum, increments);
        Value * carry_out_strm = iBuilder->CreateZExt(iBuilder->CreateLShr(incrementMask, mBitBlockWidth / 64), iBuilder->getIntNTy(mBitBlockWidth));
        setCarryOut(localIndex, iBuilder->bitCast(carry_out_strm));
    }
    else {
        Value * carryq_value = getCarryIn(localIndex);
        Value * carrygen = iBuilder->simd_and(e1, e2);
        Value * carryprop = iBuilder->simd_or(e1, e2);
        sum = iBuilder->simd_add(mBitBlockWidth, iBuilder->simd_add(mBitBlockWidth, e1, e2), carryq_value);
        Value * carry_out_strm = iBuilder->simd_or(carrygen, iBuilder->simd_and(carryprop, iBuilder->CreateNot(sum)));
        setCarryOut(localIndex, carry_out_strm);
    }
    return sum;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief advanceCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::advanceCarryInCarryOut(const unsigned localIndex, const unsigned shiftAmount, Value * const value) {
    if (LLVM_LIKELY(shiftAmount == 1)) {
        return shortAdvanceCarryInCarryOut(unitAdvancePosition(localIndex), shiftAmount, value);
    } else if (shiftAmount < LongAdvanceBase) {
        return shortAdvanceCarryInCarryOut(shortAdvancePosition(localIndex), shiftAmount, value);
    } else {
        return longAdvanceCarryInCarryOut(longAdvancePosition(localIndex), shiftAmount, value);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief shortAdvanceCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::shortAdvanceCarryInCarryOut(const unsigned index, const unsigned shiftAmount, Value * const value) {
    Value * result = nullptr;
    Value * const carryIn = getCarryPack(index);
    mCarryOutPack[index] = value;
    if (mCarryInfo->getWhileDepth() == 0) {
        storeCarryOut(index);
    }
    if (LLVM_LIKELY(shiftAmount == 1)) {
        Value * ahead = iBuilder->mvmd_dslli(DSSLI_FIELDWIDTH, value, carryIn, iBuilder->getBitBlockWidth()/DSSLI_FIELDWIDTH -1);
        result = iBuilder->simd_or(iBuilder->simd_srli(DSSLI_FIELDWIDTH, ahead, DSSLI_FIELDWIDTH-1), iBuilder->simd_slli(DSSLI_FIELDWIDTH, value, 1));
    } else if (shiftAmount % 8 == 0) { // Use a single whole-byte shift, if possible.
        result = iBuilder->mvmd_dslli(8, value, carryIn, (iBuilder->getBitBlockWidth() / 8) - (shiftAmount / 8));
    } else if (shiftAmount < DSSLI_FIELDWIDTH) {
        Value * ahead = iBuilder->mvmd_dslli(DSSLI_FIELDWIDTH, value, carryIn, iBuilder->getBitBlockWidth()/DSSLI_FIELDWIDTH - 1);
        result = iBuilder->simd_or(iBuilder->simd_srli(DSSLI_FIELDWIDTH, ahead, DSSLI_FIELDWIDTH-shiftAmount), iBuilder->simd_slli(DSSLI_FIELDWIDTH, value, shiftAmount));
    } else {
        Value* advanceq_longint = iBuilder->CreateBitCast(carryIn, iBuilder->getIntNTy(mBitBlockWidth));
        Value* strm_longint = iBuilder->CreateBitCast(value, iBuilder->getIntNTy(mBitBlockWidth));
        Value* adv_longint = iBuilder->CreateOr(iBuilder->CreateShl(strm_longint, shiftAmount), iBuilder->CreateLShr(advanceq_longint, mBitBlockWidth - shiftAmount), "advance");
        result = iBuilder->CreateBitCast(adv_longint, mBitBlockType);
    }
    if (LLVM_LIKELY(hasSummary())) {
        addToSummary(value);
    }
    return result;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief longAdvanceCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::longAdvanceCarryInCarryOut(const unsigned index, const unsigned shiftAmount, Value * const value) {
    Value * advBaseIndex = iBuilder->getInt64(index);
    if (shiftAmount <= mBitBlockWidth) {
        // special case using a single buffer entry and the carry_out value.
        Value * advanceDataPtr = iBuilder->CreateGEP(mCarryBitBlockPtr, advBaseIndex);
        Value * carry_block0 = iBuilder->CreateAlignedLoad(advanceDataPtr, mBitBlockWidth/8);
        iBuilder->CreateAlignedStore(value, advanceDataPtr, mBitBlockWidth/8);
        /* Very special case - no combine */
        if (shiftAmount == mBitBlockWidth) {
            return carry_block0;
        }
        Value* block0_shr = iBuilder->CreateLShr(iBuilder->CreateBitCast(carry_block0, iBuilder->getIntNTy(mBitBlockWidth)), mBitBlockWidth - shiftAmount);
        Value* block1_shl = iBuilder->CreateShl(iBuilder->CreateBitCast(value, iBuilder->getIntNTy(mBitBlockWidth)), shiftAmount);
        return iBuilder->CreateBitCast(iBuilder->CreateOr(block1_shl, block0_shr), mBitBlockType);
    }
    // We need a buffer of at least two elements for storing the advance data.
    const unsigned block_shift = shiftAmount % mBitBlockWidth;
    const unsigned advanceEntries = mCarryInfo->longAdvanceEntries(shiftAmount);
    const unsigned bufsize = mCarryInfo->longAdvanceBufferSize(shiftAmount);
    Value * indexMask = iBuilder->getInt64(bufsize - 1);  // A mask to implement circular buffer indexing
    Value * loadIndex0 = iBuilder->CreateAdd(iBuilder->CreateAnd(iBuilder->CreateSub(mBlockNo, iBuilder->getInt64(advanceEntries)), indexMask), advBaseIndex);
    Value * storeIndex = iBuilder->CreateAdd(iBuilder->CreateAnd(mBlockNo, indexMask), advBaseIndex);
    Value * carry_block0 = iBuilder->CreateAlignedLoad(iBuilder->CreateGEP(mCarryBitBlockPtr, loadIndex0), mBitBlockWidth/8);
    // If the long advance is an exact multiple of mBITBLOCK_WIDTH, we simply return the oldest 
    // block in the long advance carry data area.  
    if (block_shift == 0) {
        iBuilder->CreateAlignedStore(value, iBuilder->CreateGEP(mCarryBitBlockPtr, storeIndex), mBitBlockWidth/8);
        return carry_block0;
    }
    // Otherwise we need to combine data from the two oldest blocks.
    Value * loadIndex1 = iBuilder->CreateAdd(iBuilder->CreateAnd(iBuilder->CreateSub(mBlockNo, iBuilder->getInt64(advanceEntries-1)), indexMask), advBaseIndex);
    Value * carry_block1 = iBuilder->CreateAlignedLoad(iBuilder->CreateGEP(mCarryBitBlockPtr, loadIndex1), mBitBlockWidth/8);
    Value* block0_shr = iBuilder->CreateLShr(iBuilder->CreateBitCast(carry_block0, iBuilder->getIntNTy(mBitBlockWidth)), mBitBlockWidth - block_shift);
    Value* block1_shl = iBuilder->CreateShl(iBuilder->CreateBitCast(carry_block1, iBuilder->getIntNTy(mBitBlockWidth)), block_shift);
    iBuilder->CreateAlignedStore(value, iBuilder->CreateGEP(mCarryBitBlockPtr, storeIndex), mBitBlockWidth/8);
    return iBuilder->CreateBitCast(iBuilder->CreateOr(block1_shl, block0_shr), mBitBlockType);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateSummaryTest
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::generateSummaryTest(Value * condition) {
    if (mCarryInfo->hasCarries()) {
        Value * summary_pack = getCarryPack(summaryPack());
        condition = iBuilder->simd_or(condition, summary_pack);
    }
    return iBuilder->bitblock_any(condition);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief storeCarryOutSummary
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::storeCarryOutSummary() {
    if (LLVM_LIKELY(mCarryInfo->explicitSummaryRequired())) {
        const unsigned carrySummaryIndex = summaryPack();
        if (LLVM_UNLIKELY(mCarryInfo->hasLongAdvances())) { // Force if entry
            mCarryOutPack[carrySummaryIndex] = Constant::getAllOnesValue(mCarryPackType);
        }
        storeCarryOut(carrySummaryIndex);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief popCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::popCount(Value * to_count, unsigned globalIdx) {
    Value * countPtr = iBuilder->CreateGEP(mPopcountBasePtr, iBuilder->getInt64(globalIdx));
    Value * countSoFar = iBuilder->CreateAlignedLoad(countPtr, 8);
    Value * fieldCounts = iBuilder->simd_popcount(64, to_count);
    for (int i = 0; i < mBitBlockWidth/64; i++) {
        countSoFar = iBuilder->CreateAdd(countSoFar, iBuilder->mvmd_extract(64, fieldCounts, i));
    }
    iBuilder->CreateAlignedStore(countSoFar, countPtr, 8);
    return iBuilder->bitCast(iBuilder->CreateZExt(countSoFar, iBuilder->getIntNTy(mBitBlockWidth)));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addOuterSummaryToNestedSummary
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::addOuterSummaryToNestedSummary() {
    if (LLVM_LIKELY(mCarrySummary.size() > 0)) {
        addToSummary(mCarrySummary.back());
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief buildCarryDataPhisAfterIfBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::buildCarryDataPhisAfterIfBody(BasicBlock * const entry, BasicBlock * const end) {
    if (mCarryInfo->getWhileDepth() > 0) {
        // We need to phi out everything for the while carry accumulation process.
        const unsigned scopeBaseOffset = scopeBasePack();
        const unsigned scopeCarryPacks = mCarryInfo->getScopeCarryPackCount();
        for (unsigned i = scopeBaseOffset; i < scopeBaseOffset + scopeCarryPacks; ++i) {
            Type * const type = mCarryOutPack[i]->getType();
            PHINode * phi = iBuilder->CreatePHI(type, 2);
            phi->addIncoming(Constant::getNullValue(type), entry);
            phi->addIncoming(mCarryOutPack[i], end);
            mCarryOutPack[i] = phi;
        }
    }
    if (LLVM_LIKELY(mCarrySummary.size() > 0)) {
        const unsigned summaryIndex = summaryPack();
        Value * carrySummary = mCarryOutPack[summaryIndex];
        if (mCarrySummary.back() != carrySummary) {
            Value * outerCarrySummary = mCarrySummary.back();
            Value * nestedCarrySummary = mCarryOutPack[summaryIndex];
            assert (outerCarrySummary->getType() == nestedCarrySummary->getType());
            PHINode * const phi = iBuilder->CreatePHI(outerCarrySummary->getType(), 2, "summary");
            phi->addIncoming(outerCarrySummary, entry);
            phi->addIncoming(nestedCarrySummary, end);
            mCarryOutPack[summaryIndex] = phi;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeWhileEntryCarryDataPhis
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::initializeWhileEntryCarryDataPhis(BasicBlock * const end) {
    const unsigned scopeCarryPacks = mCarryInfo->getScopeCarryPackCount();
    mCarryOutAccumPhis.resize(scopeCarryPacks);
    #ifdef SET_WHILE_CARRY_IN_TO_ZERO_AFTER_FIRST_ITERATION
    const unsigned currentScopeBase = scopeBasePack();
    mCarryInPhis.resize(scopeCarryPacks);
    #endif
    for (unsigned index = 0; index < scopeCarryPacks; ++index) {
        #ifdef SET_WHILE_CARRY_IN_TO_ZERO_AFTER_FIRST_ITERATION
        PHINode * phi_in = iBuilder->CreatePHI(mCarryPackType, 2);
        phi_in->addIncoming(mCarryInPack[currentScopeBase+index], whileEntryBlock);
        mCarryInPhis[index] = phi_in;
        #endif
        PHINode * phi_out = iBuilder->CreatePHI(mCarryPackType, 2);
        phi_out->addIncoming(Constant::getNullValue(mCarryPackType), end);
        mCarryOutAccumPhis[index] = phi_out;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalizeWhileBlockCarryDataPhis
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::finalizeWhileBlockCarryDataPhis(BasicBlock * const end) {
    const unsigned scopeCarryPacks = mCarryInfo->getScopeCarryPackCount();
    const unsigned currentScopeBase = scopeBasePack();
    for (unsigned index = 0; index < scopeCarryPacks; ++index) {
        #ifdef SET_WHILE_CARRY_IN_TO_ZERO_AFTER_FIRST_ITERATION
        mCarryInPhis[index]->addIncoming(Constant::getNullValue(mCarryPackType), whileBodyFinalBlock);
        #endif
        PHINode * phi = mCarryOutAccumPhis[index];
        Value * carryOut = iBuilder->CreateOr(phi, mCarryOutPack[currentScopeBase + index]);
        phi->addIncoming(carryOut, end);
        mCarryOutPack[currentScopeBase + index] = carryOut;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief ensureCarriesLoadedRecursive
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::ensureCarriesLoadedRecursive() {
    const unsigned scopeCarryPacks = mCarryInfo->getScopeCarryPackCount();
    const unsigned currentScopeBase = scopeBasePack();
    if (mCarryInfo->getWhileDepth() == 1) {
        for (auto i = currentScopeBase; i < currentScopeBase + scopeCarryPacks; ++i) {
            getCarryPack(i);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief ensureCarriesStoredRecursive
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::ensureCarriesStoredRecursive() {
    const unsigned scopeCarryPacks = mCarryInfo->getScopeCarryPackCount();
    const unsigned currentScopeBase = scopeBasePack();
    if (mCarryInfo->getWhileDepth() == 1) {
        for (auto i = currentScopeBase; i < currentScopeBase + scopeCarryPacks; ++i) {
            storeCarryOut(i);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getCarryIn
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::getCarryIn(const unsigned localIndex) {
    return getCarryPack(addPosition(localIndex));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::setCarryOut(const unsigned localIndex, Value * carryOut) {
    const unsigned index = addPosition(localIndex);
    if (mBitBlockWidth < 256) { // #ifndef USING_LONG_ADD
        Value * carry_bit = iBuilder->CreateLShr(iBuilder->CreateBitCast(carryOut, iBuilder->getIntNTy(mBitBlockWidth)), mBitBlockWidth-1);
        carryOut = iBuilder->CreateBitCast(carry_bit, mBitBlockType);
    }
    mCarryOutPack[index] = carryOut;
    if (LLVM_LIKELY(hasSummary())) {
        addToSummary(carryOut);
    }
    if (mCarryInfo->getWhileDepth() == 0) {
        storeCarryOut(index);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerate
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned CarryManager::enumerate(PabloBlock * blk, unsigned ifDepth, unsigned whileDepth) {
    unsigned idx = blk->getScopeIndex();
    CarryData * cd = new CarryData(blk, mBitBlockWidth, 1, mBitBlockWidth);
    mCarryInfoVector[idx] = cd;

    cd->setIfDepth(ifDepth);
    cd->setWhileDepth(whileDepth);
    unsigned nestedOffset = cd->nested.frameOffset;

    for (Statement * stmt : *blk) {
        if (Count * c = dyn_cast<Count>(stmt)) {
            c->setGlobalCountIndex(mPabloCountCount);
            mPabloCountCount++;
        } else if (If * ifStatement = dyn_cast<If>(stmt)) {
            const unsigned ifCarryDataBits = enumerate(ifStatement->getBody(), ifDepth + 1, whileDepth);
            CarryData * nestedBlockData = mCarryInfoVector[ifStatement->getBody()->getScopeIndex()];
            if (1 == mBitBlockWidth) {  // PACKING
                if (cd->roomInFinalPack(nestedOffset) < ifCarryDataBits) {
                    nestedOffset = alignCeiling(nestedOffset, mBitBlockWidth);
                }
            }
            nestedBlockData->setFramePosition(nestedOffset);
            nestedOffset += ifCarryDataBits;
            if (cd->maxNestingDepth <= nestedBlockData->maxNestingDepth) {
                cd->maxNestingDepth = nestedBlockData->maxNestingDepth + 1;
            }
            cd->nested.entries++;
        } else if (While * whileStatement = dyn_cast<While>(stmt)) {
            const unsigned whileCarryDataBits = enumerate(whileStatement->getBody(), ifDepth, whileDepth + 1);
            CarryData * const nestedBlockData = mCarryInfoVector[whileStatement->getBody()->getScopeIndex()];
            if (1 == mBitBlockWidth) {  // PACKING
                if (cd->roomInFinalPack(nestedOffset) < whileCarryDataBits) {
                    nestedOffset = alignCeiling(nestedOffset, mBitBlockWidth);
                }
            }
            nestedBlockData->setFramePosition(nestedOffset);
            nestedOffset += whileCarryDataBits;
            if (cd->maxNestingDepth <= nestedBlockData->maxNestingDepth) {
                cd->maxNestingDepth = nestedBlockData->maxNestingDepth + 1;
            }
            cd->nested.entries++;
        }
    }

    cd->scopeCarryDataSize = nestedOffset;

    if (cd->explicitSummaryRequired()) {
        // Need extra space for the summary variable, always the last
        // entry within an if block.
        if (1 == mBitBlockWidth) {  // PACKING
            cd->scopeCarryDataSize = alignCeiling(cd->scopeCarryDataSize, mBitBlockWidth);
        }
        cd->summary.frameOffset = cd->scopeCarryDataSize;
        cd->scopeCarryDataSize += 1;  //  computed summary is a full pack.
    } else {
        cd->summary.frameOffset = 0;
    }

    return cd->scopeCarryDataSize;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addToSummary
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * CarryManager::addToSummary(Value * const value) {
    const unsigned summaryIndex = summaryPack();
    Value * summary = mCarryOutPack[summaryIndex];
    assert (summary);
    assert (value);
    if (LLVM_UNLIKELY(isa<Constant>(summary))) {
        if (LLVM_LIKELY(cast<Constant>(summary)->isZeroValue())) {
            summary = value;
            goto return_result;
        } else if (cast<Constant>(summary)->isAllOnesValue()) {
            goto return_result;
        }
    }
    if (LLVM_UNLIKELY(isa<Constant>(value))) {
        if (LLVM_LIKELY(cast<Constant>(value)->isZeroValue())) {
            goto return_result;
        } else if (cast<Constant>(summary)->isAllOnesValue()) {
            summary = value;
            goto return_result;
        }
    }
    if (LLVM_LIKELY(summary != value)) {
        summary = iBuilder->CreateOr(summary, value, "summary");
    }
return_result:
    mCarryOutPack[summaryIndex] = summary;
    return summary;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getCarryPack
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::getCarryPack(const unsigned packIndex) {
    if (mCarryInPack[packIndex] == nullptr) {
        Value * const packPtr = iBuilder->CreateGEP(mCarryPackBasePtr, iBuilder->getInt64(packIndex));
        mCarryPackPtr[packIndex] = packPtr;
        mCarryInPack[packIndex] = iBuilder->CreateAlignedLoad(packPtr, mBitBlockWidth / 8);
    }
    return mCarryInPack[packIndex];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief storeCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::storeCarryOut(const unsigned packIndex) {
    assert (mCarryOutPack[packIndex]);
    assert (mCarryPackPtr[packIndex]);
    iBuilder->CreateAlignedStore(mCarryOutPack[packIndex], mCarryPackPtr[packIndex], mBitBlockWidth / 8);
}

/* Helper routines */

inline unsigned CarryManager::relativeFrameOffset(const unsigned frameOffset, const unsigned index) const {
    return mCurrentFrameIndex + frameOffset + index;
}

inline unsigned CarryManager::addPosition(const unsigned localIndex) const {
    return relativeFrameOffset(mCarryInfo->addWithCarry.frameOffset, localIndex);
}

inline unsigned CarryManager::unitAdvancePosition(const unsigned localIndex) const {
    return relativeFrameOffset(mCarryInfo->unitAdvance.frameOffset, localIndex);
}

inline unsigned CarryManager::shortAdvancePosition(const unsigned localIndex) const {
    return relativeFrameOffset(mCarryInfo->shortAdvance.frameOffset, localIndex);
}

inline unsigned CarryManager::longAdvancePosition(const unsigned localIndex) const {
    return (mCurrentFrameIndex + mCarryInfo->longAdvance.frameOffset) + localIndex;
}

inline unsigned CarryManager::localBasePack() const {
    return (mCurrentFrameIndex + mCarryInfo->shortAdvance.frameOffset);
}

inline unsigned CarryManager::scopeBasePack() const {
    return mCurrentFrameIndex;
}

inline unsigned CarryManager::summaryPack() const {
    return relativeFrameOffset(mCarryInfo->summary.frameOffset, 0);
}

inline bool CarryManager::hasSummary() const {
    return mCarryInfo->explicitSummaryRequired() && !(mCarryInfo->hasLongAdvances());
}

CarryManager::~CarryManager() {
    for (auto * cd : mCarryInfoVector) {
        delete cd;
    }
}

}

