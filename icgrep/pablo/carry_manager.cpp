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
#include <llvm/Support/raw_ostream.h>
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

    mKernel->addScalar(analyse(kernel->getEntryBlock()), "carries");

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


    assert (mCarryFrame.empty());

    assert (mCarryInSummary.empty());
    mCarryInSummary.push_back(Constant::getNullValue(mCarryPackType));

    assert (mCarryOutSummary.empty());
    mCarryOutSummary.push_back(Constant::getNullValue(mCarryPackType));

    if (mHasLoop) {
        mLoopSelector = mKernel->getScalarField("selector");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalizeCodeGen
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::finalizeCodeGen() {
    if (mHasLoop) {
        mKernel->setScalarField("selector", iBuilder->CreateXor(mLoopSelector, iBuilder->getInt32(1)));
    }
    if (mHasLongAdvance) {
        Value * idx = mKernel->getScalarField("CarryBlockIndex");
        idx = iBuilder->CreateAdd(idx, iBuilder->getSize(1));
        mKernel->setScalarField("CarryBlockIndex", idx);
    }
    assert (mCarryFrame.empty());

    assert (mCarryInSummary.size() == 1);
    mCarryInSummary.clear();

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
    ++mLoopDepth;
    enterScope(scope);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterLoopBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterLoopBody(BasicBlock * const entryBlock) {

    assert (mHasLoop);

    if (mCarryInfo->hasSummary()) {
        PHINode * phiCarryOutSummary = iBuilder->CreatePHI(mCarryPackType, 2, "summary");
        assert (!mCarryOutSummary.empty());
        phiCarryOutSummary->addIncoming(mCarryOutSummary.back(), entryBlock);
        // Replace the incoming carry summary with the phi node and add the phi node to the stack
        // so that we can properly OR it into the outgoing summary value.
        mCarryOutSummary.back() = phiCarryOutSummary;
        Value * carryOut = phiCarryOutSummary;
        // In non-carry-collapsing mode, the carry out summary of this block iteration *MUST* match the carry in of the
        // subsequent block iteration. Otherwise the subsequent block iteration may become trapped in an infinite loop.
        // To ensure this, we effectively "zero-initialize" the carry-out coming into this loop but OR in carry-out
        // of the outer scope for the phi value the end of the loop body. This avoids us needing to maintain a carry-in
        // summary for all outer scopes whenever only a nested scope requires this mode.
        if (LLVM_UNLIKELY(mCarryInfo->hasCountingSummary())) {
            carryOut = Constant::getNullValue(mCarryPackType);
        }
        mCarryOutSummary.push_back(carryOut);
    }

    if (LLVM_UNLIKELY(mCarryInfo->hasCountingSummary())) {

        // Check whether we need to resize the carry state
        PHINode * index = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
        mLoopIndicies.push_back(index);
        index->addIncoming(iBuilder->getSize(0), entryBlock);

        mCarryInSummary.push_back(Constant::getNullValue(mCarryPackType));

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

        // note: the 3 here is only to pass the assertion later. It refers to the number of elements in the carry data struct.
        mCarryFrame.emplace_back(mCurrentFrame, 3);
        mCurrentFrame = iBuilder->CreateGEP(phiArrayPtr, index);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveLoopBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveLoopBody(BasicBlock * /* exitBlock */) {

    if (LLVM_UNLIKELY(mCarryInfo->hasCountingSummary())) {

        std::tie(mCurrentFrame, mCurrentFrameIndex) = mCarryFrame.back();
        mCarryFrame.pop_back();
        assert (!mCarryInSummary.empty());
        Value * carryInAccumulator = mCarryInSummary.back();
        Value * carryOutAccumulator = mCarryOutSummary.back();

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
        Value * n = iBuilder->CreateAdd(i, ONE);
        i->addIncoming(n, update);
        iBuilder->CreateCondBr(iBuilder->CreateICmpNE(iBuilder->CreateShl(ONE, i), capacity), update, resume);

        iBuilder->SetInsertPoint(resume);

        IntegerType * ty = IntegerType::get(iBuilder->getContext(), iBuilder->getBitBlockWidth());
        iBuilder->CreateAssert(iBuilder->CreateICmpEQ(iBuilder->CreateBitCast(finalBorrow, ty), Constant::getNullValue(ty)), "borrow != 0");
        iBuilder->CreateAssert(iBuilder->CreateICmpEQ(iBuilder->CreateBitCast(finalCarry, ty), Constant::getNullValue(ty)), "carry != 0");

        assert (!mLoopIndicies.empty());
        PHINode * index = mLoopIndicies.back();
        index->addIncoming(iBuilder->CreateAdd(index, iBuilder->getSize(1)), resume);
        mLoopIndicies.pop_back();
        mCarryInSummary.back() = finalCarryInSummary;
    }
    if (mCarryInfo->hasSummary()) {
        const auto n = mCarryOutSummary.size(); assert (n > 1);
        Value * carryOut = mCarryOutSummary.back();
        mCarryOutSummary.pop_back();
        PHINode * phiCarryOut = cast<PHINode>(mCarryOutSummary.back());
        if (LLVM_UNLIKELY(mCarryInfo->hasCountingSummary())) {
            carryOut = iBuilder->CreateOr(phiCarryOut, carryOut);
        }
        phiCarryOut->addIncoming(carryOut, iBuilder->GetInsertBlock());
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
    mCarryOutSummary.push_back(Constant::getNullValue(mCarryPackType));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateSummaryTest
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::generateSummaryTest(Value * condition) {
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        Value * summary = nullptr;
        if (LLVM_UNLIKELY(mCarryInfo->hasCountingSummary())) {
            summary = mCarryInSummary.back();
            mCarryInSummary.pop_back();
        } else {
            // enter the (potentially nested) struct and extract the summary element (always element 0)
            unsigned count = 2;
            if (LLVM_UNLIKELY(mCarryInfo->hasBorrowedSummary())) {
                Type * frameTy = mCurrentFrame->getType()->getPointerElementType();
                count = 1;
                while (frameTy->isStructTy()) {
                    ++count;
                    frameTy = frameTy->getStructElementType(0);
                }
            }
            const bool useLoopSelector = mCarryInfo->hasImplicitSummary() && mLoopDepth > 0;
            const auto length = count + (useLoopSelector ? 1 : 0);
            Value * indicies[length];
            std::fill(indicies, indicies + count, iBuilder->getInt32(0));
            if (LLVM_UNLIKELY(useLoopSelector)) {
                indicies[count] = mLoopSelector;
            }
            ArrayRef<Value *> ar(indicies, length);
            Value * ptr = iBuilder->CreateGEP(mCurrentFrame, ar);
            // Sanity check: make sure we're accessing a summary value
            assert (ptr->getType()->getPointerElementType()->canLosslesslyBitCastTo(condition->getType()));
            summary = iBuilder->CreateBlockAlignedLoad(ptr);
        }
        condition = iBuilder->simd_or(condition, summary);
    }
    return condition;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterIfBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterIfBody(BasicBlock * const entryBlock) { assert (entryBlock);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveIfBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveIfBody(BasicBlock * const exitBlock) { assert (exitBlock);
    const auto n = mCarryOutSummary.size();
    if (LLVM_LIKELY(mCarryInfo->hasExplicitSummary())) {
        assert (!mCarryOutSummary.empty());
        Value * ptr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
        Value * const value = iBuilder->CreateBitCast(mCarryOutSummary.back(), mBitBlockType);
        iBuilder->CreateBlockAlignedStore(value, ptr);
    }
    if (n > 1) {
        mCarryOutSummary[n - 1] = iBuilder->CreateOr(mCarryOutSummary[n - 1], mCarryOutSummary[n - 2], "summary");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveIfScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveIfScope(BasicBlock * const entryBlock, BasicBlock * const exitBlock) {
    assert (mIfDepth > 0);
    if (mCarryInfo->hasSummary()) {
        const auto n = mCarryOutSummary.size(); assert (n > 0);
        if (n > 1) {
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
    mCarryFrame.emplace_back(mCurrentFrame, mCurrentFrameIndex + 1);
    mCurrentScope = scope;
    mCarryScopeIndex.push_back(++mCarryScopes);
    mCarryInfo = &mCarryMetadata[mCarryScopes];
    // Check whether we're still within our struct bounds; if this fails, either the Pablo program changed during
    // compilation or a memory corruption has occured.
    assert (mCurrentFrameIndex < mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    mCurrentFrame = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(mCurrentFrameIndex)});
    // Verify we're pointing to a carry frame struct
    assert(mCurrentFrame->getType()->getPointerElementType()->isStructTy());
    // We always use the 0-th slot for the summary value, even when it's implicit
    mCurrentFrameIndex = mCarryInfo->hasExplicitSummary() ? 1 : 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveScope() {

    // Did we use all of the packs in this carry struct?
    assert (mCurrentFrameIndex == mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    // Sanity test: are there remaining carry frames?
    assert (!mCarryFrame.empty());

    std::tie(mCurrentFrame, mCurrentFrameIndex) = mCarryFrame.back();

    assert(mCurrentFrame->getType()->getPointerElementType()->isStructTy());

    mCarryFrame.pop_back();
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
        return longAdvanceCarryInCarryOut(shiftAmount, value);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief longAdvanceCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::longAdvanceCarryInCarryOut(const unsigned shiftAmount, Value * value) {

    assert (shiftAmount > mBitBlockWidth);
    assert (mHasLongAdvance);

    Type * const streamVectorTy = iBuilder->getIntNTy(mBitBlockWidth);
    value = iBuilder->CreateBitCast(value, mBitBlockType);
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
                addToSummary(stream);
                iBuilder->CreateBlockAlignedStore(stream, ptr);                
                buffer = iBuilder->CreateGEP(buffer, iBuilder->getInt32(1));
                break;
            }
            addToSummary(stream);
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
    // in non-carry collapsing mode, we need to accumulate the carry in value in order to properly subtract it from the
    // carry in state in order to deduce whether we still have pending iterations even if the loop condition fails.
    if (LLVM_UNLIKELY(mCarryInfo->hasCountingSummary())) {
        mCarryInSummary.back() = iBuilder->CreateOr(mCarryInSummary.back(), carryIn);
    }
    // If the long advance is an exact multiple of mBitBlockWidth, we simply return the oldest
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
    Value * carryInPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(mCurrentFrameIndex++)});
    mCarryPackPtr = carryInPtr;
    if (mLoopDepth > 0) {
        carryInPtr = iBuilder->CreateGEP(carryInPtr, {iBuilder->getInt32(0), mLoopSelector});        
    }   
    assert (carryInPtr->getType()->getPointerElementType() == mCarryPackType);
    Value * const carryIn = iBuilder->CreateBlockAlignedLoad(carryInPtr);
    // in non-carry collapsing mode, we need to accumulate the carry in value in order to properly subtract it from the
    // carry in state in order to deduce whether we still have pending iterations even if the loop condition fails.
    if (LLVM_UNLIKELY(mCarryInfo->hasCountingSummary())) {
        mCarryInSummary.back() = iBuilder->CreateOr(mCarryInSummary.back(), carryIn);
    }
    if (mLoopDepth > 0) {
        iBuilder->CreateBlockAlignedStore(Constant::getNullValue(mCarryPackType), carryInPtr);
    }
    return carryIn;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setNextCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::setNextCarryOut(Value * carryOut) {
    if (mCarryInfo->hasExplicitSummary() || mCarryInfo->hasCountingSummary()) {
        addToSummary(carryOut);
    }
    Value * carryOutPtr = mCarryPackPtr;
    if (mLoopDepth > 0) {
        Value * selector = iBuilder->CreateXor(mLoopSelector, ConstantInt::get(mLoopSelector->getType(), 1));
        carryOutPtr = iBuilder->CreateGEP(mCarryPackPtr, {iBuilder->getInt32(0), selector});
    }
    carryOut = iBuilder->CreateBitCast(carryOut, mCarryPackType);
    if (inCollapsingCarryMode()) {
        Value * accum = iBuilder->CreateBlockAlignedLoad(carryOutPtr);
        carryOut = iBuilder->CreateOr(carryOut, accum);
    }
    assert (carryOutPtr->getType()->getPointerElementType() == mCarryPackType);
    iBuilder->CreateBlockAlignedStore(carryOut, carryOutPtr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addToSummary
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::addToSummary(Value * value) { assert (value);
    assert (!mCarryOutSummary.empty());
    Value * const summary = mCarryOutSummary.back(); assert (summary);
    if (LLVM_UNLIKELY(summary == value)) {
        return;  //Nothing to add.
    }
    value = iBuilder->CreateBitCast(value, mCarryPackType);
    if (LLVM_UNLIKELY(isa<Constant>(value))) {
        if (LLVM_UNLIKELY(cast<Constant>(value)->isZeroValue())) {
            return;
        } else if (LLVM_UNLIKELY(cast<Constant>(value)->isAllOnesValue())) {
            mCarryOutSummary.back() = value;
            return;
        }
    } else if (LLVM_UNLIKELY(isa<Constant>(summary))) {
        if (LLVM_UNLIKELY(cast<Constant>(summary)->isZeroValue())) {
            mCarryOutSummary.back() = value;
            return;
        } else if (LLVM_UNLIKELY(cast<Constant>(summary)->isAllOnesValue())) {
            return;
        }
    }    
    mCarryOutSummary.back() = iBuilder->CreateOr(summary, value);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief collapsingCarryMode
 ** ------------------------------------------------------------------------------------------------------------- */
bool CarryManager::inCollapsingCarryMode() const {
    return (mCurrentScope->getBranch() && isa<While>(mCurrentScope->getBranch()) && !mCarryInfo->hasCountingSummary());
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
    if (const Branch * const br = scope->getBranch()) {
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
StructType * CarryManager::analyse(PabloBlock * const scope, const unsigned ifDepth, const unsigned loopDepth) {

    assert (scope != mKernel->getEntryBlock() || mCarryScopes == 0);
    assert (mCarryScopes < mCarryMetadata.size());
    assert (mCarryPackType);

    CarryData & cd = mCarryMetadata[mCarryScopes++];

    std::vector<Type *> state;

    Type * const carryPackType = (loopDepth == 0) ? mCarryPackType : ArrayType::get(mCarryPackType, 2);

    bool hasLongAdvances = false;
    for (Statement * stmt : *scope) {
        if (LLVM_UNLIKELY(isa<Advance>(stmt))) {
            const auto amount = cast<Advance>(stmt)->getAmount();
            if (LLVM_LIKELY(amount <= mBitBlockWidth)) {
                state.push_back(carryPackType);
            } else {
                const auto blocks = ceil_udiv(amount, mBitBlockWidth); assert (blocks > 1);
                Type * type = ArrayType::get(mBitBlockType, nearest_pow2(blocks));
                if (LLVM_UNLIKELY(ifDepth > 0)) {
                    Type * carryType = ArrayType::get(mBitBlockType, ceil_udiv(amount, std::pow(mBitBlockWidth, 2)));
                    type = StructType::get(carryType, type, nullptr);
                    hasLongAdvances = true;                    
                }
                mHasLongAdvance = true;
                state.push_back(type);
            }
        } else if (LLVM_UNLIKELY(isNonAdvanceCarryGeneratingStatement(stmt))) {
            state.push_back(carryPackType);
        } else if (LLVM_UNLIKELY(isa<If>(stmt))) {
            state.push_back(analyse(cast<If>(stmt)->getBody(), ifDepth + 1, loopDepth));
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            mHasLoop = true;
            state.push_back(analyse(cast<While>(stmt)->getBody(), ifDepth, loopDepth + 1));
        }
    }

    StructType * carryState = nullptr;
    // Add the summary pack if needed.
    CarryData::SummaryType summaryType = CarryData::NoSummary;
    if (LLVM_UNLIKELY(state.empty())) {
        carryState = StructType::get(iBuilder->getContext());
    } else {
        const bool nonCarryCollapsingMode = loopDepth > 0 && hasIterationSpecificAssignment(scope);
        if (LLVM_UNLIKELY(nonCarryCollapsingMode)) {
            summaryType = CarryData::CountingSummary;
        } else if (ifDepth > 0) {
            // A non-collapsing loop requires a unique summary for each iteration. Thus whenever we have a non-collapsing While
            // within an If scope with an implicit summary, the If scope requires an explicit summary.
            if (isa<If>(scope->getBranch())) {
                if (LLVM_LIKELY(hasLongAdvances || state.size() > 1)) {
                    summaryType = CarryData::ExplicitSummary;
                    state.insert(state.begin(), mCarryPackType);
                } else {
                    summaryType = CarryData::ImplicitSummary;
                    if (state[0]->isStructTy()) {
                        summaryType = CarryData::BorrowedSummary;
                    }
                }
            }
        }
        carryState = StructType::get(iBuilder->getContext(), state);
        // If we're in a loop and cannot use collapsing carry mode, convert the carry state struct into a capacity,
        // carry state pointer, and summary pointer struct.
        if (LLVM_UNLIKELY(nonCarryCollapsingMode)) {
            carryState = StructType::get(iBuilder->getSizeTy(), carryState->getPointerTo(), mCarryPackType->getPointerTo(), nullptr);
        }
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
, mCarryPackPtr(nullptr)
, mIfDepth(0)
, mHasLongAdvance(false)
, mHasLoop(false)
, mLoopDepth(0)
, mLoopSelector(nullptr)
, mCarryScopes(0) {

}

}
