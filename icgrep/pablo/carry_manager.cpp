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
#include <iostream>


namespace pablo {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeCarryData
 ** ------------------------------------------------------------------------------------------------------------- */
Type * CarryManager::initializeCarryData(PabloFunction * const function) {
    mRootScope = function->getEntryBlock();
    mCarryInfoVector.resize(mRootScope->enumerateScopes(0) + 1);
    mCarryPackType = mBitBlockType;
    const unsigned totalCarryDataSize = enumerate(mRootScope, 0, 0);

    mCarryPackPtr.resize(totalCarryDataSize + 1, nullptr);
    mCarryInPack.resize(totalCarryDataSize + 1, nullptr);
    mCarryOutPack.resize(totalCarryDataSize + 1, nullptr);

    mTotalCarryDataBitBlocks = totalCarryDataSize;
    ArrayType* cdArrayTy = ArrayType::get(mBitBlockType, mTotalCarryDataBitBlocks);
    return cdArrayTy;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeCodeGen
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::initializeCodeGen(PabloKernel * const kBuilder, Value * selfPtr) {
    mKernelBuilder = kBuilder;
    mSelf = selfPtr;
    
    Value * cdArrayPtr = iBuilder->CreateGEP(mSelf, {ConstantInt::get(iBuilder->getSizeTy(), 0), mKernelBuilder->getScalarIndex("carries")});
//#ifndef NDEBUG
//    iBuilder->CallPrintInt("cdArrayPtr", iBuilder->CreatePtrToInt(cdArrayPtr, iBuilder->getSizeTy()));
//#endif
    mCarryPackBasePtr = iBuilder->CreateBitCast(cdArrayPtr, PointerType::get(mCarryPackType, 0));
    mCarryBitBlockPtr = iBuilder->CreateBitCast(cdArrayPtr, PointerType::get(mBitBlockType, 0));
    mCurrentScope = mRootScope;
    mCurrentFrameIndex = 0;
    assert(mCarryInfoVector.size() > 0);
    mCarryInfo = mCarryInfoVector[0];
    assert(summaryPack() < mCarryOutPack.size());
    mCarryOutPack[summaryPack()] = Constant::getNullValue(mCarryPackType);
    assert (mCarrySummary.empty());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterScope(PabloBlock * const scope) {
    assert(summaryPack() < mCarryOutPack.size());
    Value * summaryCarry = mCarryOutPack[summaryPack()];
    mCarrySummary.push_back(summaryCarry);
    mCurrentScope = scope;
    mCarryInfo = mCarryInfoVector[scope->getScopeIndex()];
    mCurrentFrameIndex += mCarryInfo->getFrameIndex();
    assert(summaryPack() < mCarryOutPack.size());
    mCarryOutPack[summaryPack()] = Constant::getNullValue(mCarryPackType);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveScope() {
    assert(summaryPack() < mCarryOutPack.size());
    Value * summaryCarry = mCarryOutPack[summaryPack()];
    assert (mCurrentScope != mRootScope);
    mCurrentFrameIndex -= mCarryInfo->getFrameIndex();
    mCurrentScope = mCurrentScope->getPredecessor ();
    mCarryInfo = mCarryInfoVector[mCurrentScope->getScopeIndex()];
    assert(summaryPack() < mCarryOutPack.size());
    mCarryOutPack[summaryPack()] = summaryCarry;
    mCarrySummary.pop_back();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::addCarryInCarryOut(const unsigned localIndex, Value * const e1, Value * const e2) {
    std::pair<Value *, Value *> fullAdd = iBuilder->bitblock_add_with_carry(e1, e2, getCarryIn(localIndex));
    setCarryOut(localIndex, std::get<0>(fullAdd));
    return std::get<1>(fullAdd);
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
    Value * const carryIn = getCarryPack(index);
    assert (index < mCarryOutPack.size());
    std::pair<Value *, Value *> adv = iBuilder->bitblock_advance(value, carryIn, shiftAmount);
    mCarryOutPack[index] = std::get<0>(adv);
    if (mCarryInfo->getWhileDepth() == 0) {
        storeCarryOut(index);
    }
    if (LLVM_LIKELY(hasSummary())) {
        addToSummary(value);
    }
    return std::get<1>(adv);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief longAdvanceCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::longAdvanceCarryInCarryOut(const unsigned index, const unsigned shiftAmount, Value * const value) {
    Value * advBaseIndex = ConstantInt::get(iBuilder->getSizeTy(), index);
    if (shiftAmount <= mBitBlockWidth) {
        // special case using a single buffer entry and the carry_out value.
        Value * advanceDataPtr = iBuilder->CreateGEP(mCarryBitBlockPtr, advBaseIndex);
        Value * carry_block0 = iBuilder->CreateBlockAlignedLoad(advanceDataPtr);
        iBuilder->CreateBlockAlignedStore(value, advanceDataPtr);
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
    Value * indexMask = ConstantInt::get(iBuilder->getSizeTy(), bufsize - 1);  // A mask to implement circular buffer indexing
    Value * blockIndex = mKernelBuilder->getScalarField(mSelf, blockNoScalar);
    Value * loadIndex0 = iBuilder->CreateAdd(iBuilder->CreateAnd(iBuilder->CreateSub(blockIndex, ConstantInt::get(iBuilder->getSizeTy(), advanceEntries)), indexMask), advBaseIndex);
    Value * storeIndex = iBuilder->CreateAdd(iBuilder->CreateAnd(blockIndex, indexMask), advBaseIndex);
    Value * carry_block0 = iBuilder->CreateBlockAlignedLoad(iBuilder->CreateGEP(mCarryBitBlockPtr, loadIndex0));
    // If the long advance is an exact multiple of mBITBLOCK_WIDTH, we simply return the oldest 
    // block in the long advance carry data area.  
    if (block_shift == 0) {
        iBuilder->CreateBlockAlignedStore(value, iBuilder->CreateGEP(mCarryBitBlockPtr, storeIndex));
        return carry_block0;
    }
    // Otherwise we need to combine data from the two oldest blocks.
    Value * loadIndex1 = iBuilder->CreateAdd(iBuilder->CreateAnd(iBuilder->CreateSub(blockIndex, ConstantInt::get(iBuilder->getSizeTy(), advanceEntries-1)), indexMask), advBaseIndex);
    Value * carry_block1 = iBuilder->CreateBlockAlignedLoad(iBuilder->CreateGEP(mCarryBitBlockPtr, loadIndex1));
    Value* block0_shr = iBuilder->CreateLShr(iBuilder->CreateBitCast(carry_block0, iBuilder->getIntNTy(mBitBlockWidth)), mBitBlockWidth - block_shift);
    Value* block1_shl = iBuilder->CreateShl(iBuilder->CreateBitCast(carry_block1, iBuilder->getIntNTy(mBitBlockWidth)), block_shift);
    iBuilder->CreateBlockAlignedStore(value, iBuilder->CreateGEP(mCarryBitBlockPtr, storeIndex));
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
            assert (carrySummaryIndex < mCarryOutPack.size());
            mCarryOutPack[carrySummaryIndex] = Constant::getAllOnesValue(mCarryPackType);
        }
        storeCarryOut(carrySummaryIndex);
    }
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
            assert (i < mCarryOutPack.size());
            Type * const type = mCarryOutPack[i]->getType();
            PHINode * phi = iBuilder->CreatePHI(type, 2);
            phi->addIncoming(Constant::getNullValue(type), entry);
            phi->addIncoming(mCarryOutPack[i], end);
            mCarryOutPack[i] = phi;
        }
    }
    if (LLVM_LIKELY(mCarrySummary.size() > 0)) {
        const unsigned summaryIndex = summaryPack();
        assert (summaryIndex < mCarryOutPack.size());
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
        assert (index < mCarryOutAccumPhis.size());
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
        assert (index < mCarryOutAccumPhis.size());
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
    assert (index < mCarryOutPack.size());
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
        if (If * ifStatement = dyn_cast<If>(stmt)) {
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
void CarryManager::addToSummary(Value * const value) {
    const unsigned summaryIndex = summaryPack();
    assert (summaryIndex < mCarryInPack.size());
    Value * summary = mCarryOutPack[summaryIndex];
    assert (summary);
    assert (value);
    if (LLVM_UNLIKELY(summary == value)) return;  //Nothing to add.
    
    Type * summaryTy = summary->getType();
    Type * valueTy = value->getType();
    if (LLVM_UNLIKELY(isa<Constant>(value))) {
        if (LLVM_LIKELY(cast<Constant>(value)->isZeroValue())) return;
        if (cast<Constant>(value)->isAllOnesValue()) {
            mCarryOutPack[summaryIndex] = Constant::getAllOnesValue(summaryTy);
            return;
        }
    }
    Value * v = value;
    if (valueTy != summaryTy) {
        // valueTy must be an integer type.
        unsigned summaryWidth = summaryTy->isIntegerTy() ? summaryTy->getIntegerBitWidth() : cast<VectorType>(summaryTy)->getBitWidth();        
        if (valueTy->getIntegerBitWidth() != summaryWidth) {
            v = iBuilder->CreateZExt(v, iBuilder->getIntNTy(summaryWidth));
        }
        if (!(summaryTy->isIntegerTy())) {
            v = iBuilder->CreateBitCast(v, summaryTy);
        }
    }
    if (LLVM_UNLIKELY(isa<Constant>(summary))) {
        if (LLVM_LIKELY(cast<Constant>(summary)->isZeroValue())) {
            mCarryOutPack[summaryIndex] = v;
            return;
        } else if (cast<Constant>(summary)->isAllOnesValue()) {
            return;
        }
    }
    mCarryOutPack[summaryIndex] = iBuilder->CreateOr(summary, v, "summary");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getCarryPack
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::getCarryPack(const unsigned packIndex) {
    assert (packIndex < mCarryInPack.size());
    if (mCarryInPack[packIndex] == nullptr) {
        Value * const packPtr = iBuilder->CreateGEP(mCarryPackBasePtr, ConstantInt::get(iBuilder->getSizeTy(), packIndex));
        assert (packIndex < mCarryPackPtr.size());
        mCarryPackPtr[packIndex] = packPtr;
        mCarryInPack[packIndex] = iBuilder->CreateBlockAlignedLoad(packPtr);
    }
    return mCarryInPack[packIndex];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief storeCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::storeCarryOut(const unsigned packIndex) {
    assert (packIndex < mCarryOutPack.size());
    assert (mCarryOutPack[packIndex]);
    assert (packIndex < mCarryPackPtr.size());
    Value * const ptr = mCarryPackPtr[packIndex];
    assert (ptr);
    assert (cast<PointerType>(ptr->getType())->getElementType() == mBitBlockType);
    Value * const value = iBuilder->CreateBitCast(mCarryOutPack[packIndex], mBitBlockType);
    iBuilder->CreateBlockAlignedStore(value, ptr);
}

/* Helper routines */

inline unsigned CarryManager::relativeFrameOffset(const unsigned frameOffset, const unsigned index) const {
    return mCurrentFrameIndex + frameOffset + index;
}

inline unsigned CarryManager::addPosition(const unsigned localIndex) const {
    assert (mCarryInfo);
    return relativeFrameOffset(mCarryInfo->addWithCarry.frameOffset, localIndex);
}

inline unsigned CarryManager::unitAdvancePosition(const unsigned localIndex) const {
    assert (mCarryInfo);
    return relativeFrameOffset(mCarryInfo->unitAdvance.frameOffset, localIndex);
}

inline unsigned CarryManager::shortAdvancePosition(const unsigned localIndex) const {
    assert (mCarryInfo);
    return relativeFrameOffset(mCarryInfo->shortAdvance.frameOffset, localIndex);
}

inline unsigned CarryManager::longAdvancePosition(const unsigned localIndex) const {
    assert (mCarryInfo);
    return (mCurrentFrameIndex + mCarryInfo->longAdvance.frameOffset) + localIndex;
}

inline unsigned CarryManager::localBasePack() const {
    assert (mCarryInfo);
    return (mCurrentFrameIndex + mCarryInfo->shortAdvance.frameOffset);
}

inline unsigned CarryManager::scopeBasePack() const {
    return mCurrentFrameIndex;
}

inline unsigned CarryManager::summaryPack() const {
    assert (mCarryInfo);
    return relativeFrameOffset(mCarryInfo->summary.frameOffset, 0);
}

inline bool CarryManager::hasSummary() const {
    assert (mCarryInfo);
    return mCarryInfo->explicitSummaryRequired() && !(mCarryInfo->hasLongAdvances());
}

CarryManager::~CarryManager() {
    for (auto * cd : mCarryInfoVector) {
        delete cd;
    }
}

}

