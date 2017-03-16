/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "carry_manager.h"
#include <pablo/carry_data.h>
#include <pablo/codegenstate.h>
#include <llvm/IR/BasicBlock.h>
#include <IR_Gen/idisa_builder.h>
#include <llvm/IR/DerivedTypes.h>
#include <pablo/branch.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_var.h>
// #include <llvm/Support/raw_ostream.h>
using namespace llvm;

namespace pablo {

inline static unsigned floor_log2(const unsigned v) {
    assert ("log2(0) is undefined!" && v != 0);
    return 31 - __builtin_clz(v);
}

inline static unsigned nearest_pow2(const uint32_t v) {
    assert(v > 0 && v < (UINT32_MAX / 2));
    return (v < 2) ? 1 : (1 << (32 - __builtin_clz(v - 1)));
}

inline static unsigned ceil_udiv(const unsigned x, const unsigned y) {
    return (((x - 1) | (y - 1)) + 1) / y;
}

using TypeId = PabloAST::ClassTypeId;

inline static bool isNonAdvanceCarryGeneratingStatement(const Statement * const stmt) {
    switch (stmt->getClassTypeId()) {
        case TypeId::ScanThru:
        case TypeId::AdvanceThenScanThru:
        case TypeId::ScanTo:
        case TypeId::AdvanceThenScanTo:
        case TypeId::MatchStar:
            return true;
        default:
            return false;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeCarryData
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::initializeCarryData(PabloKernel * const kernel) {

    // Each scope constructs its own CarryData struct, which will be added to the final "carries" struct
    // that is added to the Kernel. The scope index will indicate which struct to access.

    // A CarryData struct either contains an array of CarryPackBlocks or an integer indicating the capacity of
    // the variable length CarryData struct and pointer. A variable length CarryData struct is required whenever
    // the streams accessed by a loop could vary between iterations. When resizing a CarryData struct for a
    // particular loop, the current loop struct and all nested structs need to be resized. This accommodates
    // the fact every pablo While loop must be executed at least once.

    // A nested loop may also contain a variable length CarryData struct

    // To determine whether we require a variable length CarryData struct, we test the escaped variables of
    // each loop branch to see whether they are used as the index parameter of a nested Extract statement.
    // Any scope that requires variable length CarryData, requires that all nested branches have a unique
    // set of carries for that iteration.

    mKernel = kernel;

    mCurrentScope = kernel->getEntryBlock();

    mCarryScopes = 0;

    mCarryMetadata.resize(getScopeCount(kernel->getEntryBlock()));

    Type * ty = analyse(kernel->getEntryBlock());

    ty->dump();

    mKernel->addScalar(ty, "carries");

    if (mHasLoop) {
        mKernel->addScalar(iBuilder->getInt32Ty(), "selector");
    }
    if (mHasLongAdvance) {
        mKernel->addScalar(iBuilder->getSizeTy(), "CarryBlockIndex");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeCodeGen
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::initializeCodeGen() {

    assert(!mCarryMetadata.empty());
    mCarryInfo = &mCarryMetadata[0];
    assert (!mCarryInfo->hasSummary());

    mCurrentFrame = mKernel->getScalarFieldPtr("carries");
    mCurrentFrameIndex = 0;
    mCarryScopes = 0;
    mCarryScopeIndex.push_back(0);

    assert (mCarryFrameStack.empty());

    assert (mCarryOutSummary.empty());
    mCarryOutSummary.push_back(Constant::getNullValue(mCarryPackType));

    if (mHasLoop) {
        mLoopSelector = mKernel->getScalarField("selector");
        mNextLoopSelector = iBuilder->CreateXor(mLoopSelector, ConstantInt::get(mLoopSelector->getType(), 1));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalizeCodeGen
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::finalizeCodeGen() {
    if (mHasLoop) {
        mKernel->setScalarField("selector", mNextLoopSelector);
    }
    if (mHasLongAdvance) {
        Value * idx = mKernel->getScalarField("CarryBlockIndex");
        idx = iBuilder->CreateAdd(idx, iBuilder->getSize(1));
        mKernel->setScalarField("CarryBlockIndex", idx);
    }
    assert (mCarryFrameStack.empty());

    assert (mCarryOutSummary.size() == 1);
    mCarryOutSummary.clear();

    assert (mCarryScopeIndex.size() == 1);
    mCarryScopeIndex.clear();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterLoopScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterLoopScope(const PabloBlock * const scope) {
    assert (scope);
    assert (mHasLoop);
    ++mLoopDepth;
    enterScope(scope);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterLoopBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterLoopBody(BasicBlock * const entryBlock) {
    if (mCarryInfo->hasSummary()) {
        PHINode * phiCarryOutSummary = iBuilder->CreatePHI(mCarryPackType, 2, "summary");
        assert (!mCarryOutSummary.empty());
        phiCarryOutSummary->addIncoming(mCarryOutSummary.back(), entryBlock);
        // Replace the incoming carry summary with the phi node and add the phi node to the stack
        // so that we can properly OR it into the outgoing summary value.
        mCarryOutSummary.back() = phiCarryOutSummary;
        mCarryOutSummary.push_back(phiCarryOutSummary);
    }
    if (LLVM_UNLIKELY(mCarryInfo->nonCarryCollapsingMode())) {

        assert (mCarryInfo->hasSummary());

        // Check whether we need to resize the carry state
        PHINode * index = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
        mLoopIndicies.push_back(index);
        index->addIncoming(iBuilder->getSize(0), entryBlock);

        Value * capacityPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
        Value * capacity = iBuilder->CreateLoad(capacityPtr, false, "capacity");
        Constant * const ONE = ConstantInt::get(capacity->getType(), 1);
        Value * arrayPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
        Value * array = iBuilder->CreateLoad(arrayPtr, false, "array");

        BasicBlock * const entry = iBuilder->GetInsertBlock();
        BasicBlock * const resizeCarryState = mKernel->CreateBasicBlock("ResizeCarryState");
        BasicBlock * const reallocExisting = mKernel->CreateBasicBlock("ReallocExisting");
        BasicBlock * const createNew = mKernel->CreateBasicBlock("CreateNew");
        BasicBlock * const resumeKernel = mKernel->CreateBasicBlock("ResumeKernel");

        iBuilder->CreateLikelyCondBr(iBuilder->CreateICmpULT(index, capacity), resumeKernel, resizeCarryState);

        // RESIZE CARRY BLOCK
        iBuilder->SetInsertPoint(resizeCarryState);
        const auto BlockWidth = mCarryPackType->getPrimitiveSizeInBits() / 8;
        const auto Log2BlockWidth = floor_log2(BlockWidth);
        Constant * const carryStateWidth = ConstantExpr::getIntegerCast(ConstantExpr::getSizeOf(array->getType()->getPointerElementType()), iBuilder->getSizeTy(), false);
        Value * summaryPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(2)});

        Value * const hasCarryState = iBuilder->CreateICmpNE(array, ConstantPointerNull::get(cast<PointerType>(array->getType())));

        iBuilder->CreateLikelyCondBr(hasCarryState, reallocExisting, createNew);

        // REALLOCATE EXISTING
        iBuilder->SetInsertPoint(reallocExisting);

        Value * const capacitySize = iBuilder->CreateMul(capacity, carryStateWidth);
        Value * const newCapacitySize = iBuilder->CreateShl(capacitySize, 1); // x 2


        Value * newArray = iBuilder->CreateAlignedMalloc(newCapacitySize, iBuilder->getCacheAlignment());
        iBuilder->CreateMemCpy(newArray, array, capacitySize, BlockWidth);
        iBuilder->CreateMemZero(iBuilder->CreateGEP(newArray, capacitySize), capacitySize, BlockWidth);
        iBuilder->CreateAlignedFree(array);
        newArray = iBuilder->CreatePointerCast(newArray, array->getType());
        iBuilder->CreateStore(newArray, arrayPtr);

        Value * const log2capacity = iBuilder->CreateAdd(iBuilder->CreateCeilLog2(capacity), ONE);
        Value * const summarySize = iBuilder->CreateShl(log2capacity, Log2BlockWidth + 1); // x 2(BlockWidth)
        Value * const newLog2Capacity = iBuilder->CreateAdd(log2capacity, ONE);
        Value * const newSummarySize = iBuilder->CreateShl(newLog2Capacity, Log2BlockWidth + 1); // x 2(BlockWidth)

        Value * const summary = iBuilder->CreateLoad(summaryPtr, false);
        Value * newSummary = iBuilder->CreateAlignedMalloc(newSummarySize, BlockWidth);
        iBuilder->CreateMemCpy(newSummary, summary, summarySize, BlockWidth);
        iBuilder->CreateMemZero(iBuilder->CreateGEP(newSummary, summarySize), iBuilder->getSize(2 * BlockWidth), BlockWidth);
        iBuilder->CreateAlignedFree(summary);

        Value * ptr1 = iBuilder->CreateGEP(newSummary, summarySize);
        ptr1 = iBuilder->CreatePointerCast(ptr1, mCarryPackType->getPointerTo());

        Value * ptr2 = iBuilder->CreateGEP(newSummary, iBuilder->CreateAdd(summarySize, iBuilder->getSize(BlockWidth)));
        ptr2 = iBuilder->CreatePointerCast(ptr2, mCarryPackType->getPointerTo());

        newSummary = iBuilder->CreatePointerCast(newSummary, mCarryPackType->getPointerTo());
        iBuilder->CreateStore(newSummary, summaryPtr);
        Value * const newCapacity = iBuilder->CreateShl(ONE, log2capacity);

        iBuilder->CreateStore(newCapacity, capacityPtr);

        iBuilder->CreateBr(resumeKernel);

        // CREATE NEW
        iBuilder->SetInsertPoint(createNew);

        Constant * const initialLog2Capacity = iBuilder->getInt64(4);
        Constant * const initialCapacity = ConstantExpr::getShl(ONE, initialLog2Capacity);
        Constant * const initialCapacitySize = ConstantExpr::getMul(initialCapacity, carryStateWidth);

        Value * initialArray = iBuilder->CreateAlignedMalloc(initialCapacitySize, iBuilder->getCacheAlignment());
        iBuilder->CreateMemZero(initialArray, initialCapacitySize, BlockWidth);
        initialArray = iBuilder->CreatePointerCast(initialArray, array->getType());
        iBuilder->CreateStore(initialArray, arrayPtr);

        Constant * initialSummarySize = ConstantExpr::getShl(ConstantExpr::getAdd(initialLog2Capacity, iBuilder->getInt64(1)), iBuilder->getInt64(Log2BlockWidth + 1));
        Value * initialSummary = iBuilder->CreateAlignedMalloc(initialSummarySize, BlockWidth);
        iBuilder->CreateMemZero(initialSummary, initialSummarySize, BlockWidth);
        initialSummary = iBuilder->CreatePointerCast(initialSummary, mCarryPackType->getPointerTo());
        iBuilder->CreateStore(initialSummary, summaryPtr);

        iBuilder->CreateStore(initialCapacity, capacityPtr);

        iBuilder->CreateBr(resumeKernel);

        // RESUME KERNEL
        iBuilder->SetInsertPoint(resumeKernel);
        // Load the appropriate carry stat block
        PHINode * phiArrayPtr = iBuilder->CreatePHI(array->getType(), 3);
        phiArrayPtr->addIncoming(array, entry);
        phiArrayPtr->addIncoming(initialArray, createNew);
        phiArrayPtr->addIncoming(newArray, reallocExisting);

        // NOTE: the 3 here is only to pass the assertion later. It refers to the number of elements in the carry data struct.
        mCarryFrameStack.emplace_back(mCurrentFrame, 3);
        mCurrentFrame = iBuilder->CreateGEP(phiArrayPtr, index);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveLoopBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveLoopBody(BasicBlock * /* exitBlock */) {

    if (LLVM_UNLIKELY(mCarryInfo->nonCarryCollapsingMode())) {

        assert (mCarryInfo->hasSummary());

        ConstantInt * const summaryIndex = iBuilder->getInt32(mCurrentFrame->getType()->getPointerElementType()->getStructNumElements() - 1);

        Value * carryInAccumulator = readCarryInSummary(summaryIndex);
        Value * carryOutAccumulator = mCarryOutSummary.back();

        if (mCarryInfo->hasExplicitSummary()) {
            writeCarryOutSummary(carryOutAccumulator, summaryIndex);
        }

        std::tie(mCurrentFrame, mCurrentFrameIndex) = mCarryFrameStack.back();
        mCarryFrameStack.pop_back();

        // In non-carry-collapsing mode, we cannot rely on the fact that performing a single iteration of this
        // loop will consume all of the incoming carries from the prior block. We need to subtract the carries
        // consumed by this iteration from our carry summary state. To do so in parallel, we use the the half-
        // subtractor circuit to do it in ceil log2 steps. Similarly, we compute our carry out summary state
        // (for the subsequent block to subtract) using a half-adder circuit.

        // NOTE: this requires that, for all loop iterations, i, and all block iterations, j, the carry in
        // summary, CI_i,j, matches the carry out summary of the prior block iteration, CO_i,j - 1.
        // Otherwise we may end up with an incorrect result or being trapped in an infinite loop.

        Value * capacityPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
        Value * capacity = iBuilder->CreateLoad(capacityPtr, false);
        Value * summaryPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(2)});
        Value * summary = iBuilder->CreateLoad(summaryPtr, false);

        Constant * const ONE = ConstantInt::get(capacity->getType(), 1);

        Value * loopSelector = iBuilder->CreateZExt(mLoopSelector, capacity->getType());

        BasicBlock * entry = iBuilder->GetInsertBlock();
        BasicBlock * update = mKernel->CreateBasicBlock("UpdateNonCarryCollapsingSummary");
        BasicBlock * resume = mKernel->CreateBasicBlock("ResumeAfterUpdatingNonCarryCollapsingSummary");

        iBuilder->CreateBr(update);

        iBuilder->SetInsertPoint(update);
        PHINode * i = iBuilder->CreatePHI(capacity->getType(), 2);
        i->addIncoming(ConstantInt::getNullValue(capacity->getType()), entry);
        PHINode * const borrow = iBuilder->CreatePHI(carryInAccumulator->getType(), 2);
        borrow->addIncoming(carryInAccumulator, entry);
        PHINode * const carry = iBuilder->CreatePHI(carryOutAccumulator->getType(), 2);
        carry->addIncoming(carryOutAccumulator, entry);
        // OR the updated carry in summary later for the summaryTest
        PHINode * const carryInSummary = iBuilder->CreatePHI(mCarryPackType, 2);
        carryInSummary->addIncoming(Constant::getNullValue(mCarryPackType), entry);

        // half subtractor
        Value * const carryInOffset = iBuilder->CreateOr(iBuilder->CreateShl(i, 1), loopSelector);
        Value * const carryInPtr = iBuilder->CreateGEP(summary, carryInOffset);
        Value * const carryIn = iBuilder->CreateBlockAlignedLoad(carryInPtr);
        Value * const carryInPrime = iBuilder->CreateXor(carryIn, borrow);
        Value * const finalCarryInSummary = iBuilder->CreateOr(carryInSummary, carryInPrime);
        iBuilder->CreateBlockAlignedStore(carryInPrime, carryInPtr);
        carryInSummary->addIncoming(finalCarryInSummary, update);
        Value * finalBorrow = iBuilder->CreateAnd(iBuilder->CreateNot(carryIn), borrow);
        borrow->addIncoming(finalBorrow, update);

        // half adder
        Value * const carryOutOffset = iBuilder->CreateXor(carryInOffset, ConstantInt::get(carryInOffset->getType(), 1));
        Value * const carryOutPtr = iBuilder->CreateGEP(summary, carryOutOffset);
        Value * const carryOut = iBuilder->CreateBlockAlignedLoad(carryOutPtr);
        Value * const carryOutPrime = iBuilder->CreateXor(carryOut, carry);
        iBuilder->CreateBlockAlignedStore(carryOutPrime, carryOutPtr);
        Value * finalCarry = iBuilder->CreateAnd(carryOut, carry);
        carry->addIncoming(finalCarry, update);

        // loop condition
        i->addIncoming(iBuilder->CreateAdd(i, ONE), update);
        iBuilder->CreateCondBr(iBuilder->CreateICmpNE(iBuilder->CreateShl(ONE, i), capacity), update, resume);

        iBuilder->SetInsertPoint(resume);

        assert (!mLoopIndicies.empty());
        PHINode * index = mLoopIndicies.back();
        index->addIncoming(iBuilder->CreateAdd(index, iBuilder->getSize(1)), resume);
        mLoopIndicies.pop_back();

        mNextSummaryTest = finalCarryInSummary;
    }
    if (mCarryInfo->hasSummary()) {
        const auto n = mCarryOutSummary.size(); assert (n > 1);
        Value * carryOut = mCarryOutSummary.back();
        mCarryOutSummary.pop_back();
        PHINode * phiCarryOut = cast<PHINode>(mCarryOutSummary.back());
        phiCarryOut->addIncoming(carryOut, iBuilder->GetInsertBlock());
        mCarryOutSummary.back() = carryOut;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveLoopScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveLoopScope(BasicBlock * const /* entryBlock */, BasicBlock * const /* exitBlock */) {
    assert (mLoopDepth > 0);
    --mLoopDepth;
    leaveScope();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterIfScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterIfScope(const PabloBlock * const scope) {
    ++mIfDepth;
    enterScope(scope);
    // We start with zero-initialized carry in order and later OR in the current carry summary so that upon
    // processing the subsequent block iteration, we branch into this If scope iff a carry was generated by
    // a statement within the If scope and not by a dominating statement in the outer scope.
    mCarryOutSummary.push_back(Constant::getNullValue(mCarryPackType));
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        assert (mCurrentFrameIndex == 0);
        mNextSummaryTest = readCarryInSummary(iBuilder->getInt32(0));
        if (mCarryInfo->hasExplicitSummary()) {
            mCurrentFrameIndex = 1;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateSummaryTest
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::generateSummaryTest(Value * condition) {
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        assert ("summary test was not generated" && mNextSummaryTest);
        condition = iBuilder->simd_or(condition, mNextSummaryTest);
        mNextSummaryTest = nullptr;
    }
    assert ("summary test was not consumed" && (mNextSummaryTest == nullptr));
    return condition;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterIfBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterIfBody(BasicBlock * const entryBlock) {
    assert (entryBlock);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveIfBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveIfBody(BasicBlock * const exitBlock) {
    assert (exitBlock);
    const auto n = mCarryOutSummary.size();
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        if (LLVM_LIKELY(mCarryInfo->hasExplicitSummary())) {
            writeCarryOutSummary(mCarryOutSummary[n - 1], iBuilder->getInt32(0));
        }
    }
    if (n > 2) {
        mCarryOutSummary[n - 1] = iBuilder->CreateOr(mCarryOutSummary[n - 1], mCarryOutSummary[n - 2], "summary");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveIfScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveIfScope(BasicBlock * const entryBlock, BasicBlock * const exitBlock) {
    assert (mIfDepth > 0);
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        const auto n = mCarryOutSummary.size(); assert (n > 0);
        if (n > 2) {
            // When leaving a nested If scope with a summary value, phi out the summary to ensure the
            // appropriate summary is stored in the outer scope.
            Value * nested = mCarryOutSummary[n - 1];
            Value * outer = mCarryOutSummary[n - 2];
            if (LLVM_LIKELY(nested != outer)) {
                assert (nested->getType() == outer->getType());
                PHINode * const phi = iBuilder->CreatePHI(nested->getType(), 2, "summary");
                phi->addIncoming(outer, entryBlock);
                phi->addIncoming(nested, exitBlock);
                mCarryOutSummary[n - 2] = phi;
            }
        }
    }
    --mIfDepth;
    leaveScope();
    mCarryOutSummary.pop_back();
}

/** ------------------------------------------------------------------------------------------------------------ *
 * @brief enterScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterScope(const PabloBlock * const scope) {
    assert (scope);
    // Store the state of the current frame and update the scope state
    mCarryFrameStack.emplace_back(mCurrentFrame, mCurrentFrameIndex + 1);
    mCurrentScope = scope;
    mCarryScopeIndex.push_back(++mCarryScopes);
    mCarryInfo = &mCarryMetadata[mCarryScopes];
    // Check whether we're still within our struct bounds; if this fails, either the Pablo program changed during
    // compilation or a memory corruption has occured.
    assert (mCurrentFrameIndex < mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    mCurrentFrame = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(mCurrentFrameIndex)});
    // Verify we're pointing to a carry frame struct
    assert(mCurrentFrame->getType()->getPointerElementType()->isStructTy());
    mCurrentFrameIndex = 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveScope() {

    // Did we use all of the packs in this carry struct?
    assert (mCurrentFrameIndex == mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    // Sanity test: are there remaining carry frames?
    assert (!mCarryFrameStack.empty());

    std::tie(mCurrentFrame, mCurrentFrameIndex) = mCarryFrameStack.back();

    assert(mCurrentFrame->getType()->getPointerElementType()->isStructTy());

    mCarryFrameStack.pop_back();
    mCarryScopeIndex.pop_back();
    assert (!mCarryScopeIndex.empty());
    mCurrentScope = mCurrentScope->getPredecessor();
    assert (mCurrentScope);
    mCarryInfo = &mCarryMetadata[mCarryScopeIndex.back()];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::addCarryInCarryOut(const Statement * const operation, Value * const e1, Value * const e2) {
    assert (operation && (isNonAdvanceCarryGeneratingStatement(operation)));
    Value * const carryIn = getNextCarryIn();
    Value * carryOut, * result;
    std::tie(carryOut, result) = iBuilder->bitblock_add_with_carry(e1, e2, carryIn);
    setNextCarryOut(carryOut);
    assert (result->getType() == mBitBlockType);
    return result;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief advanceCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::advanceCarryInCarryOut(const Advance * const advance, Value * const value) {
    const auto shiftAmount = advance->getAmount();
    if (LLVM_LIKELY(shiftAmount <= mBitBlockWidth)) {
        Value * const carryIn = getNextCarryIn();
        Value * carryOut, * result;
        if (LLVM_UNLIKELY(shiftAmount == mBitBlockWidth)) {
            result = carryIn;
            carryOut = value;
        } else {
            std::tie(carryOut, result) = iBuilder->bitblock_advance(value, carryIn, shiftAmount);
        }
        setNextCarryOut(carryOut);
        assert (result->getType() == mBitBlockType);
        return result;
    } else {
        return longAdvanceCarryInCarryOut(value, shiftAmount);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief longAdvanceCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::longAdvanceCarryInCarryOut(Value * const value, const unsigned shiftAmount) {

    assert (shiftAmount > mBitBlockWidth);
    assert (mHasLongAdvance);

    Type * const streamVectorTy = iBuilder->getIntNTy(mBitBlockWidth);
    Value * buffer = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(mCurrentFrameIndex++), iBuilder->getInt32(0)});

    const unsigned blockShift = shiftAmount % mBitBlockWidth;
    const unsigned entries = ceil_udiv(shiftAmount, mBitBlockWidth);

    if (LLVM_LIKELY(mCarryInfo->hasExplicitSummary())) {
        Value * const summaryPtr = iBuilder->CreateGEP(buffer, iBuilder->getInt32(0));
        assert (summaryPtr->getType()->getPointerElementType() == mBitBlockType);
        Value * carry = iBuilder->CreateZExtOrBitCast(iBuilder->bitblock_any(value), streamVectorTy);
        const auto limit = ceil_udiv(shiftAmount, std::pow(mBitBlockWidth, 2));
        assert (limit == summaryPtr->getType()->getPointerElementType()->getArrayNumElements());
        for (unsigned i = 0;;++i) {
            Value * ptr = iBuilder->CreateGEP(summaryPtr, iBuilder->getInt32(i));
            Value * prior = iBuilder->CreateBitCast(iBuilder->CreateBlockAlignedLoad(ptr), streamVectorTy);
            Value * stream = iBuilder->CreateOr(iBuilder->CreateShl(prior, 1), carry);
            if (LLVM_LIKELY(i == limit)) {
                stream = iBuilder->CreateAnd(stream, iBuilder->bitblock_mask_from(iBuilder->getInt32(entries % mBitBlockWidth)));
                addToCarryOutSummary(stream);
                iBuilder->CreateBlockAlignedStore(stream, ptr);                
                buffer = iBuilder->CreateGEP(buffer, iBuilder->getInt32(1));
                break;
            }
            addToCarryOutSummary(stream);
            iBuilder->CreateBlockAlignedStore(stream, ptr);
            carry = iBuilder->CreateLShr(prior, mBitBlockWidth - 1);
        }
    }
    assert (buffer->getType()->getPointerElementType() == mBitBlockType);

    // Create a mask to implement circular buffer indexing
    Value * indexMask = iBuilder->getSize(nearest_pow2(entries) - 1);    
    Value * blockIndex = mKernel->getScalarField("CarryBlockIndex");
    Value * carryIndex0 = iBuilder->CreateSub(blockIndex, iBuilder->getSize(entries));
    Value * loadIndex0 = iBuilder->CreateAnd(carryIndex0, indexMask);
    Value * storeIndex = iBuilder->CreateAnd(blockIndex, indexMask);
    Value * carryIn = iBuilder->CreateBlockAlignedLoad(iBuilder->CreateGEP(buffer, loadIndex0));
    assert (carryIn->getType() == mBitBlockType);

    // If the long advance is an exact multiple of BitBlockWidth, we simply return the oldest
    // block in the long advance carry data area.  
    if (blockShift == 0) {
        iBuilder->CreateBlockAlignedStore(value, iBuilder->CreateGEP(buffer, storeIndex));
        return carryIn;
    }
    // Otherwise we need to combine data from the two oldest blocks.
    Value * carryIndex1 = iBuilder->CreateSub(blockIndex, iBuilder->getSize(entries - 1));
    Value * loadIndex1 = iBuilder->CreateAnd(carryIndex1, indexMask);
    Value * carry_block1 = iBuilder->CreateBlockAlignedLoad(iBuilder->CreateGEP(buffer, loadIndex1));
    Value * block0_shr = iBuilder->CreateLShr(iBuilder->CreateBitCast(carryIn, streamVectorTy), mBitBlockWidth - blockShift);
    Value * block1_shl = iBuilder->CreateShl(iBuilder->CreateBitCast(carry_block1, streamVectorTy), blockShift);
    iBuilder->CreateBlockAlignedStore(value, iBuilder->CreateGEP(buffer, storeIndex));
    return iBuilder->CreateBitCast(iBuilder->CreateOr(block1_shl, block0_shr), mBitBlockType);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNextCarryIn
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::getNextCarryIn() {
    assert (mCurrentFrameIndex < mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    Value * carryInPtr = nullptr;
    if (mLoopDepth == 0) {
        carryInPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(mCurrentFrameIndex)});
    } else {
        carryInPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(mCurrentFrameIndex), mLoopSelector});
    }
    assert (carryInPtr->getType()->getPointerElementType() == mCarryPackType);
    Value * const carryIn = iBuilder->CreateBlockAlignedLoad(carryInPtr);
    if (mLoopDepth > 0) {
        iBuilder->CreateBlockAlignedStore(Constant::getNullValue(mCarryPackType), carryInPtr);
    }
    return carryIn;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief collapsingCarryMode
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool CarryManager::inCarryCollapsingMode() const {
    return (dyn_cast_or_null<While>(mCurrentScope->getBranch()) && !mCarryInfo->nonCarryCollapsingMode());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setNextCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::setNextCarryOut(Value * carryOut) {
    assert (mCurrentFrameIndex < mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    carryOut = iBuilder->CreateBitCast(carryOut, mCarryPackType);
    if (mCarryInfo->hasExplicitSummary()) {
        addToCarryOutSummary(carryOut);
    }
    Value * carryOutPtr = nullptr;
    if (mLoopDepth == 0) {
        carryOutPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(mCurrentFrameIndex)});
    } else {
        carryOutPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(mCurrentFrameIndex), mNextLoopSelector});
    }
    ++mCurrentFrameIndex;
    if (inCarryCollapsingMode()) {
        Value * accum = iBuilder->CreateBlockAlignedLoad(carryOutPtr);
        carryOut = iBuilder->CreateOr(carryOut, accum);
    }
    assert (carryOutPtr->getType()->getPointerElementType() == mCarryPackType);
    iBuilder->CreateBlockAlignedStore(carryOut, carryOutPtr);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readCarryInSummary
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::readCarryInSummary(ConstantInt * index) const {
    assert (mCarryInfo->hasSummary());
    unsigned count = 2;
    if (LLVM_UNLIKELY(mCarryInfo->hasBorrowedSummary())) {
        Type * frameTy = mCurrentFrame->getType()->getPointerElementType();
        count = 1;
        while (frameTy->isStructTy()) {
            ++count;
            frameTy = frameTy->getStructElementType(0);
        }
    }
    const unsigned length = (mLoopDepth == 0) ? count : (count + 1);
    Value * indicies[length];
    std::fill(indicies, indicies + count - 1, iBuilder->getInt32(0));
    indicies[count - 1] = index;
    if (mLoopDepth > 0) {
        indicies[count] = mLoopSelector;
    }
    ArrayRef<Value *> ar(indicies, length);
    Value * const ptr = iBuilder->CreateGEP(mCurrentFrame, ar);
    Value * const summary = iBuilder->CreateBlockAlignedLoad(ptr);
    if (mLoopDepth > 0) {
        iBuilder->CreateBlockAlignedStore(Constant::getNullValue(mCarryPackType), ptr);
    }
    return summary;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeCarryOutSummary
 ** ------------------------------------------------------------------------------------------------------------- */
inline void CarryManager::writeCarryOutSummary(Value * const summary, ConstantInt * index) const {
    Value * ptr = nullptr;
    assert (mCarryInfo->hasExplicitSummary());
    if (mLoopDepth > 0) {
        ptr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), index, mNextLoopSelector});
    } else {
        ptr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), index});
    }
    iBuilder->CreateBlockAlignedStore(summary, ptr);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addToCarryOutSummary
 ** ------------------------------------------------------------------------------------------------------------- */
inline void CarryManager::addToCarryOutSummary(Value * const value) {
    assert ("cannot add null summary value!" && value);
    assert ("summary stack is empty!" && !mCarryOutSummary.empty());
    mCarryOutSummary.back() = iBuilder->CreateOr(value, mCarryOutSummary.back());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerate
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned CarryManager::getScopeCount(PabloBlock * const scope, unsigned index) {
    for (Statement * stmt : *scope) {
        if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
            index = getScopeCount(cast<Branch>(stmt)->getBody(), index);
        }
    }
    return index + 1;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasIterationSpecificAssignment
 ** ------------------------------------------------------------------------------------------------------------- */
bool CarryManager::hasIterationSpecificAssignment(const PabloBlock * const scope) {
    if (const While * const br = dyn_cast_or_null<While>(scope->getBranch())) {
        for (const Var * var : br->getEscaped()) {
            for (const PabloAST * user : var->users()) {
                if (const Extract * e = dyn_cast<Extract>(user)) {
                    if (LLVM_UNLIKELY(e->getIndex() == var)) {
                        // If we assign this Var a value and read the value as the index parameter
                        // of a nested Extract statement, then we cannot collapse the carries.
                        const PabloBlock * parent = e->getParent();
                        for (;;) {
                            if (parent == scope) {
                                return true;
                            }
                            parent = parent->getPredecessor();
                            if (parent == nullptr) {
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief analyse
 ** ------------------------------------------------------------------------------------------------------------- */
StructType * CarryManager::analyse(PabloBlock * const scope, const unsigned ifDepth, const unsigned loopDepth, const bool isNestedWithinNonCarryCollapsingLoop) {

    assert (scope != mKernel->getEntryBlock() || mCarryScopes == 0);
    assert (mCarryScopes < mCarryMetadata.size());
    assert (mCarryPackType);

    const unsigned carryScopeIndex = mCarryScopes++;
    const bool nonCarryCollapsingMode = hasIterationSpecificAssignment(scope);
    Type * const carryPackType = (loopDepth == 0) ? mCarryPackType : ArrayType::get(mCarryPackType, 2);
    bool hasLongAdvances = false;
    std::vector<Type *> state;

    for (Statement * stmt : *scope) {
        if (LLVM_UNLIKELY(isa<Advance>(stmt))) {
            const auto amount = cast<Advance>(stmt)->getAmount();
            Type * type = carryPackType;
            if (LLVM_UNLIKELY(amount > mBitBlockWidth)) {
                const auto blocks = ceil_udiv(amount, mBitBlockWidth); assert (blocks > 1);
                type = ArrayType::get(mBitBlockType, nearest_pow2(blocks));
                if (LLVM_UNLIKELY(ifDepth > 0)) {
                    Type * carryType = ArrayType::get(mBitBlockType, ceil_udiv(amount, std::pow(mBitBlockWidth, 2)));
                    type = StructType::get(carryType, type, nullptr);
                    hasLongAdvances = true;                    
                }
                mHasLongAdvance = true;
            }
            state.push_back(type);
        } else if (LLVM_UNLIKELY(isNonAdvanceCarryGeneratingStatement(stmt))) {
            state.push_back(carryPackType);
        } else if (LLVM_UNLIKELY(isa<If>(stmt))) {
            state.push_back(analyse(cast<If>(stmt)->getBody(), ifDepth + 1, loopDepth, nonCarryCollapsingMode | isNestedWithinNonCarryCollapsingLoop));
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            mHasLoop = true;
            state.push_back(analyse(cast<While>(stmt)->getBody(), ifDepth, loopDepth + 1, nonCarryCollapsingMode | isNestedWithinNonCarryCollapsingLoop));
        }
    }
    // Build the carry state struct and add the summary pack if needed.
    CarryData & cd = mCarryMetadata[carryScopeIndex];
    StructType * carryState = nullptr;
    CarryData::SummaryType summaryType = CarryData::NoSummary;
    if (LLVM_UNLIKELY(state.empty())) {
        carryState = StructType::get(iBuilder->getContext());
    } else {
        //if (ifDepth > 0 || (nonCarryCollapsingMode | isNestedWithinNonCarryCollapsingLoop)) {
        if (dyn_cast_or_null<If>(scope->getBranch()) || nonCarryCollapsingMode || isNestedWithinNonCarryCollapsingLoop) {
            if (LLVM_LIKELY(state.size() > 1 || hasLongAdvances)) {
                summaryType = CarryData::ExplicitSummary;
                // NOTE: summaries are stored differently depending whether we're entering an If or While branch. With an If branch, they
                // preceed the carry state data and with a While loop they succeed it. This is to help cache prefectching performance.
                state.insert(isa<If>(scope->getBranch()) ? state.begin() : state.end(), carryPackType);
            } else {
                summaryType = CarryData::ImplicitSummary;
                if (state[0]->isStructTy()) {
                    summaryType = CarryData::BorrowedSummary;
                }
            }            
        }
        carryState = StructType::get(iBuilder->getContext(), state);
        // If we're in a loop and cannot use collapsing carry mode, convert the carry state struct into a capacity,
        // carry state pointer, and summary pointer struct.
        if (LLVM_UNLIKELY(nonCarryCollapsingMode)) {
            carryState = StructType::get(iBuilder->getSizeTy(), carryState->getPointerTo(), mCarryPackType->getPointerTo(), nullptr);
        }
        cd.setNonCollapsingCarryMode(nonCarryCollapsingMode);
    }
    cd.setSummaryType(summaryType);
    return carryState;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
CarryManager::CarryManager(IDISA::IDISA_Builder * idb) noexcept
: iBuilder(idb)
, mKernel(nullptr)
, mSelf(nullptr)
, mBitBlockType(idb->getBitBlockType())
, mBitBlockWidth(idb->getBitBlockWidth())
, mCurrentFrame(nullptr)
, mCurrentFrameIndex(0)
, mCurrentScope(nullptr)
, mCarryInfo(nullptr)
, mCarryPackType(mBitBlockType)
, mNextSummaryTest(nullptr)
, mIfDepth(0)
, mHasLongAdvance(false)
, mHasLoop(false)
, mLoopDepth(0)
, mLoopSelector(nullptr)
, mNextLoopSelector(nullptr)
, mCarryScopes(0) {

}

}
