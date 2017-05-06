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

inline static unsigned ceil_log2(const unsigned v) {
    assert ("log2(0) is undefined!" && v != 0);
    return 32 - __builtin_clz(v - 1);
}

inline static unsigned floor_log2(const unsigned v) {
    assert ("log2(0) is undefined!" && v != 0);
    return 31 - __builtin_clz(v);
}

inline static unsigned nearest_pow2(const unsigned v) {
    assert(v > 0 && v < (UINT32_MAX / 2));
    return (v < 2) ? 1 : (1 << ceil_log2(v));
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

#define LONG_ADVANCE_BREAKPOINT 64

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeCarryData
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::initializeCarryData(IDISA::IDISA_Builder * const builder, PabloKernel * const kernel) {

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

    assert (mKernel == nullptr);
    mCurrentScope = kernel->getEntryBlock();
    mKernel = kernel;

    mCarryScopes = 0;

    mCarryMetadata.resize(getScopeCount(mCurrentScope));

    Type * const carryStateTy = analyse(builder, mCurrentScope);

    kernel->addScalar(carryStateTy, "carries");

    if (mHasLoop) {
        kernel->addScalar(builder->getInt32Ty(), "selector");
    }
    if (mHasLongAdvance) {
        kernel->addScalar(builder->getSizeTy(), "CarryBlockIndex");
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeCodeGen
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::initializeCodeGen(IDISA::IDISA_Builder * const builder) {

    assert(!mCarryMetadata.empty());
    mCarryInfo = &mCarryMetadata[0];
    assert (!mCarryInfo->hasSummary());

    mCurrentFrame = mKernel->getScalarFieldPtr("carries");
    mCurrentFrameIndex = 0;
    mCarryScopes = 0;
    mCarryScopeIndex.push_back(0);

    assert (mCarryFrameStack.empty());

    assert (mCarrySummaryStack.empty());

    Type * const carryTy = builder->getBitBlockType();

    mCarrySummaryStack.push_back(Constant::getNullValue(carryTy));

    if (mHasLoop) {        
        mLoopSelector = mKernel->getScalarField("selector");
        mNextLoopSelector = builder->CreateXor(mLoopSelector, ConstantInt::get(mLoopSelector->getType(), 1));
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalizeCodeGen
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::finalizeCodeGen(IDISA::IDISA_Builder * const builder) {
    if (mHasLoop) {
        mKernel->setScalarField("selector", mNextLoopSelector);
    }
    if (mHasLongAdvance) {
        Value * idx = mKernel->getScalarField("CarryBlockIndex");
        idx = builder->CreateAdd(idx, builder->getSize(1));
        mKernel->setScalarField("CarryBlockIndex", idx);
    }
    assert (mCarryFrameStack.empty());    
    assert ("base summary value was deleted!" && mCarrySummaryStack.size() == 1);
    assert ("base summary value was overwritten with non-zero value!" && isa<Constant>(mCarrySummaryStack[0]) && cast<Constant>(mCarrySummaryStack[0])->isNullValue());
    mCarrySummaryStack.clear();
    assert (mCarryScopeIndex.size() == 1);
    mCarryScopeIndex.clear();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterLoopScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterLoopScope(IDISA::IDISA_Builder * const builder, const PabloBlock * const scope) {
    assert (scope);
    assert (mHasLoop);
    ++mLoopDepth;
    enterScope(builder, scope);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterLoopBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterLoopBody(IDISA::IDISA_Builder * const builder, BasicBlock * const entryBlock) {
    if (mCarryInfo->hasSummary()) {
        Type * const carryTy = builder->getBitBlockType();
        PHINode * phiCarryOutSummary = builder->CreatePHI(carryTy, 2, "summary");
        assert (!mCarrySummaryStack.empty());
        phiCarryOutSummary->addIncoming(mCarrySummaryStack.back(), entryBlock);
        // Replace the incoming carry summary with the phi node and add the phi node to the stack  so that we can
        // properly OR it into the outgoing summary value.
        // NOTE: this may change the base summary value; when exiting to the base scope, replace this summary with
        // a null value to prevent subsequent nested scopes from inheriting the summary of this scope.
        mCarrySummaryStack.back() = phiCarryOutSummary;
        mCarrySummaryStack.push_back(phiCarryOutSummary);
    }
    if (LLVM_UNLIKELY(mCarryInfo->nonCarryCollapsingMode())) {

        assert (mCarryInfo->hasSummary());

        Type * const carryTy = builder->getBitBlockType();
        PointerType * const carryPtrTy = carryTy->getPointerTo();

        // Check whether we need to resize the carry state
        PHINode * index = builder->CreatePHI(builder->getSizeTy(), 2);
        mLoopIndicies.push_back(index);
        index->addIncoming(builder->getSize(0), entryBlock);

        Value * capacityPtr = builder->CreateGEP(mCurrentFrame, {builder->getInt32(0), builder->getInt32(0)});
        Value * capacity = builder->CreateLoad(capacityPtr, false, "capacity");
        Constant * const ONE = ConstantInt::get(capacity->getType(), 1);
        Value * arrayPtr = builder->CreateGEP(mCurrentFrame, {builder->getInt32(0), builder->getInt32(1)});
        Value * array = builder->CreateLoad(arrayPtr, false, "array");

        BasicBlock * const entry = builder->GetInsertBlock();
        BasicBlock * const resizeCarryState = mKernel->CreateBasicBlock("ResizeCarryState");
        BasicBlock * const reallocExisting = mKernel->CreateBasicBlock("ReallocExisting");
        BasicBlock * const createNew = mKernel->CreateBasicBlock("CreateNew");
        BasicBlock * const resumeKernel = mKernel->CreateBasicBlock("ResumeKernel");

        builder->CreateLikelyCondBr(builder->CreateICmpULT(index, capacity), resumeKernel, resizeCarryState);

        // RESIZE CARRY BLOCK
        builder->SetInsertPoint(resizeCarryState);
        const auto BlockWidth = carryTy->getPrimitiveSizeInBits() / 8;
        const auto Log2BlockWidth = floor_log2(BlockWidth);
        Constant * const carryStateWidth = ConstantExpr::getIntegerCast(ConstantExpr::getSizeOf(array->getType()->getPointerElementType()), builder->getSizeTy(), false);
        Value * summaryPtr = builder->CreateGEP(mCurrentFrame, {builder->getInt32(0), builder->getInt32(2)});

        Value * const hasCarryState = builder->CreateICmpNE(array, ConstantPointerNull::get(cast<PointerType>(array->getType())));

        builder->CreateLikelyCondBr(hasCarryState, reallocExisting, createNew);

        // REALLOCATE EXISTING
        builder->SetInsertPoint(reallocExisting);

        Value * const capacitySize = builder->CreateMul(capacity, carryStateWidth);
        Value * const newCapacitySize = builder->CreateShl(capacitySize, 1); // x 2


        Value * newArray = builder->CreateAlignedMalloc(newCapacitySize, builder->getCacheAlignment());
        builder->CreateMemCpy(newArray, array, capacitySize, BlockWidth);
        builder->CreateMemZero(builder->CreateGEP(newArray, capacitySize), capacitySize, BlockWidth);
        builder->CreateAlignedFree(array);
        newArray = builder->CreatePointerCast(newArray, array->getType());
        builder->CreateStore(newArray, arrayPtr);

        Value * const log2capacity = builder->CreateAdd(builder->CreateCeilLog2(capacity), ONE);
        Value * const summarySize = builder->CreateShl(log2capacity, Log2BlockWidth + 1); // x 2(BlockWidth)
        Value * const newLog2Capacity = builder->CreateAdd(log2capacity, ONE);
        Value * const newSummarySize = builder->CreateShl(newLog2Capacity, Log2BlockWidth + 1); // x 2(BlockWidth)

        Value * const summary = builder->CreateLoad(summaryPtr, false);
        Value * newSummary = builder->CreateAlignedMalloc(newSummarySize, BlockWidth);
        builder->CreateMemCpy(newSummary, summary, summarySize, BlockWidth);
        builder->CreateMemZero(builder->CreateGEP(newSummary, summarySize), builder->getSize(2 * BlockWidth), BlockWidth);
        builder->CreateAlignedFree(summary);

        Value * ptr1 = builder->CreateGEP(newSummary, summarySize);
        ptr1 = builder->CreatePointerCast(ptr1, carryPtrTy);

        Value * ptr2 = builder->CreateGEP(newSummary, builder->CreateAdd(summarySize, builder->getSize(BlockWidth)));
        ptr2 = builder->CreatePointerCast(ptr2, carryPtrTy);

        newSummary = builder->CreatePointerCast(newSummary, carryPtrTy);
        builder->CreateStore(newSummary, summaryPtr);
        Value * const newCapacity = builder->CreateShl(ONE, log2capacity);

        builder->CreateStore(newCapacity, capacityPtr);

        builder->CreateBr(resumeKernel);

        // CREATE NEW
        builder->SetInsertPoint(createNew);

        Constant * const initialLog2Capacity = builder->getInt64(4);
        Constant * const initialCapacity = ConstantExpr::getShl(ONE, initialLog2Capacity);
        Constant * const initialCapacitySize = ConstantExpr::getMul(initialCapacity, carryStateWidth);

        Value * initialArray = builder->CreateAlignedMalloc(initialCapacitySize, builder->getCacheAlignment());
        builder->CreateMemZero(initialArray, initialCapacitySize, BlockWidth);
        initialArray = builder->CreatePointerCast(initialArray, array->getType());
        builder->CreateStore(initialArray, arrayPtr);

        Constant * initialSummarySize = ConstantExpr::getShl(ConstantExpr::getAdd(initialLog2Capacity, builder->getInt64(1)), builder->getInt64(Log2BlockWidth + 1));
        Value * initialSummary = builder->CreateAlignedMalloc(initialSummarySize, BlockWidth);
        builder->CreateMemZero(initialSummary, initialSummarySize, BlockWidth);
        initialSummary = builder->CreatePointerCast(initialSummary, carryPtrTy);
        builder->CreateStore(initialSummary, summaryPtr);

        builder->CreateStore(initialCapacity, capacityPtr);

        builder->CreateBr(resumeKernel);

        // RESUME KERNEL
        builder->SetInsertPoint(resumeKernel);
        // Load the appropriate carry stat block
        PHINode * phiArrayPtr = builder->CreatePHI(array->getType(), 3);
        phiArrayPtr->addIncoming(array, entry);
        phiArrayPtr->addIncoming(initialArray, createNew);
        phiArrayPtr->addIncoming(newArray, reallocExisting);

        // NOTE: the 3 here is only to pass the assertion later. It refers to the number of elements in the carry data struct.
        mCarryFrameStack.emplace_back(mCurrentFrame, 3);
        mCurrentFrame = builder->CreateGEP(phiArrayPtr, index);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveLoopBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveLoopBody(IDISA::IDISA_Builder * const builder, BasicBlock * /* exitBlock */) {

    Type * const carryTy = builder->getBitBlockType();

    if (LLVM_UNLIKELY(mCarryInfo->nonCarryCollapsingMode())) {

        assert (mCarryInfo->hasSummary());

        ConstantInt * const summaryIndex = builder->getInt32(mCarryInfo->hasExplicitSummary() ? mCurrentFrameIndex : (mCurrentFrameIndex - 1));

        Value * const carryInAccumulator = readCarryInSummary(builder, summaryIndex);
        Value * const carryOutAccumulator = mCarrySummaryStack.back();

        if (mCarryInfo->hasExplicitSummary()) {
            writeCarryOutSummary(builder, carryOutAccumulator, summaryIndex);
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

        Value * capacityPtr = builder->CreateGEP(mCurrentFrame, {builder->getInt32(0), builder->getInt32(0)});
        Value * capacity = builder->CreateLoad(capacityPtr, false);
        Value * summaryPtr = builder->CreateGEP(mCurrentFrame, {builder->getInt32(0), builder->getInt32(2)});
        Value * summary = builder->CreateLoad(summaryPtr, false);

        Constant * const ONE = ConstantInt::get(capacity->getType(), 1);

        Value * loopSelector = builder->CreateZExt(mLoopSelector, capacity->getType());

        BasicBlock * entry = builder->GetInsertBlock();
        BasicBlock * update = mKernel->CreateBasicBlock("UpdateNonCarryCollapsingSummary");
        BasicBlock * resume = mKernel->CreateBasicBlock("ResumeAfterUpdatingNonCarryCollapsingSummary");

        builder->CreateBr(update);

        builder->SetInsertPoint(update);
        PHINode * i = builder->CreatePHI(capacity->getType(), 2);
        i->addIncoming(ConstantInt::getNullValue(capacity->getType()), entry);
        PHINode * const borrow = builder->CreatePHI(carryInAccumulator->getType(), 2);
        borrow->addIncoming(carryInAccumulator, entry);
        PHINode * const carry = builder->CreatePHI(carryOutAccumulator->getType(), 2);
        carry->addIncoming(carryOutAccumulator, entry);
        // OR the updated carry in summary later for the summaryTest
        PHINode * const carryInSummary = builder->CreatePHI(carryTy, 2);
        carryInSummary->addIncoming(Constant::getNullValue(carryTy), entry);

        // half subtractor
        Value * const carryInOffset = builder->CreateOr(builder->CreateShl(i, 1), loopSelector);
        Value * const carryInPtr = builder->CreateGEP(summary, carryInOffset);
        Value * const carryIn = builder->CreateBlockAlignedLoad(carryInPtr);
        Value * const nextCarryIn = builder->CreateXor(carryIn, borrow);
        Value * const nextSummary = builder->CreateOr(carryInSummary, nextCarryIn);
        builder->CreateBlockAlignedStore(nextCarryIn, carryInPtr);
        carryInSummary->addIncoming(nextSummary, update);
        Value * finalBorrow = builder->CreateAnd(builder->CreateNot(carryIn), borrow);
        borrow->addIncoming(finalBorrow, update);

        // half adder
        Value * const carryOutOffset = builder->CreateXor(carryInOffset, ConstantInt::get(carryInOffset->getType(), 1));
        Value * const carryOutPtr = builder->CreateGEP(summary, carryOutOffset);
        Value * const carryOut = builder->CreateBlockAlignedLoad(carryOutPtr);
        Value * const nextCarryOut = builder->CreateXor(carryOut, carry);
        builder->CreateBlockAlignedStore(nextCarryOut, carryOutPtr);
        Value * finalCarry = builder->CreateAnd(carryOut, carry);
        carry->addIncoming(finalCarry, update);

        // loop condition
        i->addIncoming(builder->CreateAdd(i, ONE), update);
        builder->CreateCondBr(builder->CreateICmpNE(builder->CreateShl(ONE, i), capacity), update, resume);

        builder->SetInsertPoint(resume);

        IntegerType * ty = IntegerType::get(builder->getContext(), carryTy->getPrimitiveSizeInBits());
        builder->CreateAssert(builder->CreateICmpEQ(builder->CreateBitCast(finalBorrow, ty), ConstantInt::getNullValue(ty)), "finalBorrow != 0");
        builder->CreateAssert(builder->CreateICmpEQ(builder->CreateBitCast(finalCarry, ty), ConstantInt::getNullValue(ty)), "finalCarry != 0");

        assert (!mLoopIndicies.empty());
        PHINode * index = mLoopIndicies.back();
        index->addIncoming(builder->CreateAdd(index, builder->getSize(1)), resume);
        mLoopIndicies.pop_back();

        mNextSummaryTest = nextSummary;
    }
    if (mCarryInfo->hasSummary()) {
        const auto n = mCarrySummaryStack.size(); assert (n > 1);
        Value * carryOut = mCarrySummaryStack.back();
        mCarrySummaryStack.pop_back();
        PHINode * phiCarryOut = cast<PHINode>(mCarrySummaryStack.back());
        phiCarryOut->addIncoming(carryOut, builder->GetInsertBlock());
        // If we're returning to the base scope, reset our accumulated summary value.
        if (n == 2) {
            carryOut = Constant::getNullValue(carryTy);
        }
        mCarrySummaryStack.back() = carryOut;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveLoopScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveLoopScope(IDISA::IDISA_Builder * const builder, BasicBlock * const /* entryBlock */, BasicBlock * const /* exitBlock */) {
    assert (mLoopDepth > 0);
    --mLoopDepth;
    leaveScope(builder);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterIfScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterIfScope(IDISA::IDISA_Builder * const builder, const PabloBlock * const scope) {
    ++mIfDepth;
    enterScope(builder, scope);
    // We zero-initialized the nested summary value and later OR in the current summary into the escaping summary
    // so that upon processing the subsequent block iteration, we branch into this If scope iff a carry out was
    // generated by a statement within this If scope and not by a dominating statement in the outer scope.
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        assert (mCurrentFrameIndex == 0);
        mNextSummaryTest = readCarryInSummary(builder, builder->getInt32(0));
        if (mCarryInfo->hasExplicitSummary()) {
            mCurrentFrameIndex = 1;
        }
    }
    Type * const carryTy = builder->getBitBlockType();
    mCarrySummaryStack.push_back(Constant::getNullValue(carryTy));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateSummaryTest
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::generateSummaryTest(IDISA::IDISA_Builder * const builder, Value * condition) {
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        assert ("summary test was not generated" && mNextSummaryTest);
        condition = builder->simd_or(condition, mNextSummaryTest);
        mNextSummaryTest = nullptr;
    }
    assert ("summary test was not consumed" && (mNextSummaryTest == nullptr));
    return condition;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterIfBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterIfBody(IDISA::IDISA_Builder * const /* builder */, BasicBlock * const entryBlock) {
    assert (entryBlock);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveIfBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveIfBody(IDISA::IDISA_Builder * const builder, BasicBlock * const exitBlock) {
    assert (exitBlock);
    const auto n = mCarrySummaryStack.size();
    if (LLVM_LIKELY(mCarryInfo->hasExplicitSummary())) {
        writeCarryOutSummary(builder, mCarrySummaryStack[n - 1], builder->getInt32(0));
    }
    if (n > 2) {
        mCarrySummaryStack[n - 1] = builder->CreateOr(mCarrySummaryStack[n - 1], mCarrySummaryStack[n - 2], "summary");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveIfScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveIfScope(IDISA::IDISA_Builder * const builder, BasicBlock * const entryBlock, BasicBlock * const exitBlock) {
    assert (mIfDepth > 0);
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        const auto n = mCarrySummaryStack.size(); assert (n > 0);
        if (n > 2) {
            // When leaving a nested If scope with a summary value, phi out the summary to ensure the
            // appropriate summary is stored in the outer scope.
            Value * nested = mCarrySummaryStack[n - 1];
            Value * outer = mCarrySummaryStack[n - 2];
            assert (nested->getType() == outer->getType());
            PHINode * const phi = builder->CreatePHI(nested->getType(), 2, "summary");
            phi->addIncoming(outer, entryBlock);
            phi->addIncoming(nested, exitBlock);
            mCarrySummaryStack[n - 2] = phi;
        }
    }
    --mIfDepth;
    leaveScope(builder);
    mCarrySummaryStack.pop_back();
}

/** ------------------------------------------------------------------------------------------------------------ *
 * @brief enterScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterScope(IDISA::IDISA_Builder * const builder, const PabloBlock * const scope) {
    assert (scope);
    // Store the state of the current frame and update the scope state
    mCarryFrameStack.emplace_back(mCurrentFrame, mCurrentFrameIndex + 1);
    mCurrentScope = scope;
    mCarryScopeIndex.push_back(++mCarryScopes);
    mCarryInfo = &mCarryMetadata[mCarryScopes];
    // Check whether we're still within our struct bounds; if this fails, either the Pablo program changed during
    // compilation or a memory corruption has occured.
    assert (mCurrentFrameIndex < mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    mCurrentFrame = builder->CreateGEP(mCurrentFrame, {builder->getInt32(0), builder->getInt32(mCurrentFrameIndex)});
    // Verify we're pointing to a carry frame struct
    assert(mCurrentFrame->getType()->getPointerElementType()->isStructTy());
    mCurrentFrameIndex = 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveScope(IDISA::IDISA_Builder * const /* builder */) {

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
Value * CarryManager::addCarryInCarryOut(IDISA::IDISA_Builder * const builder, const Statement * const operation, Value * const e1, Value * const e2) {
    assert (operation && (isNonAdvanceCarryGeneratingStatement(operation)));
    Value * const carryIn = getNextCarryIn(builder);
    Value * carryOut, * result;
    std::tie(carryOut, result) = builder->bitblock_add_with_carry(e1, e2, carryIn);
    setNextCarryOut(builder, carryOut);
    assert (result->getType() == builder->getBitBlockType());
    return result;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief advanceCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::advanceCarryInCarryOut(IDISA::IDISA_Builder * const builder, const Advance * const advance, Value * const value) {
    const auto shiftAmount = advance->getAmount();
    if (LLVM_LIKELY(shiftAmount < LONG_ADVANCE_BREAKPOINT)) {
        Value * const carryIn = getNextCarryIn(builder);
        Value * carryOut, * result;
        std::tie(carryOut, result) = builder->bitblock_advance(value, carryIn, shiftAmount);
        setNextCarryOut(builder, carryOut);
        assert (result->getType() == builder->getBitBlockType());
        return result;
    } else {
        return longAdvanceCarryInCarryOut(builder, value, shiftAmount);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief longAdvanceCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * CarryManager::longAdvanceCarryInCarryOut(IDISA::IDISA_Builder * const builder, Value * const value, const unsigned shiftAmount) {

    assert (mHasLongAdvance);
    assert (shiftAmount >= LONG_ADVANCE_BREAKPOINT);

    Type * const streamTy = builder->getIntNTy(builder->getBitBlockWidth());

    if (mIfDepth > 0) {
        if (shiftAmount > builder->getBitBlockWidth()) {
            const auto frameIndex = mCurrentFrameIndex++;
            Value * carry = builder->CreateZExt(builder->bitblock_any(value), streamTy);
            const unsigned summarySize = ceil_udiv(shiftAmount, builder->getBitBlockWidth() * builder->getBitBlockWidth());
            for (unsigned i = 0;;++i) {
                Value * const ptr = builder->CreateGEP(mCurrentFrame, {builder->getInt32(0), builder->getInt32(frameIndex), builder->getInt32(i)});
                Value * const prior = builder->CreateBitCast(builder->CreateBlockAlignedLoad(ptr), streamTy);
                Value * const stream = builder->CreateBitCast(builder->CreateOr(builder->CreateShl(prior, 1), carry), builder->getBitBlockType());
                if (LLVM_LIKELY(i == summarySize)) {
                    Value * const maskedStream = builder->CreateAnd(stream, builder->bitblock_mask_from(builder->getInt32(summarySize % builder->getBitBlockWidth())));
                    addToCarryOutSummary(builder, maskedStream);
                    builder->CreateBlockAlignedStore(maskedStream, ptr);
                    break;
                }
                addToCarryOutSummary(builder, stream);
                builder->CreateBlockAlignedStore(stream, ptr);
                carry = builder->CreateLShr(prior, builder->getBitBlockWidth() - 1);
            }
        } else if (LLVM_LIKELY(mCarryInfo->hasExplicitSummary())) {
            addToCarryOutSummary(builder, value);
        }
    }
    const auto frameIndex = mCurrentFrameIndex++;
    // special case using a single buffer entry and the carry_out value.
    if (LLVM_LIKELY((shiftAmount < builder->getBitBlockWidth()) && (mLoopDepth == 0))) {
        Value * const buffer = builder->CreateGEP(mCurrentFrame, {builder->getInt32(0), builder->getInt32(frameIndex), builder->getInt32(0)});
        assert (buffer->getType()->getPointerElementType() == builder->getBitBlockType());
        Value * carryIn = builder->CreateBlockAlignedLoad(buffer);
        builder->CreateBlockAlignedStore(value, buffer);
        /* Very special case - no combine */
        if (LLVM_UNLIKELY(shiftAmount == builder->getBitBlockWidth())) {
            return builder->CreateBitCast(carryIn, builder->getBitBlockType());
        }
        Value* block0_shr = builder->CreateLShr(builder->CreateBitCast(carryIn, streamTy), builder->getBitBlockWidth() - shiftAmount);
        Value* block1_shl = builder->CreateShl(builder->CreateBitCast(value, streamTy), shiftAmount);
        return builder->CreateBitCast(builder->CreateOr(block1_shl, block0_shr), builder->getBitBlockType());
    } else { //
        const unsigned blockShift = shiftAmount % builder->getBitBlockWidth();
        const unsigned blocks = ceil_udiv(shiftAmount, builder->getBitBlockWidth());
        // Create a mask to implement circular buffer indexing
        Value * indexMask = builder->getSize(nearest_pow2(blocks + ((mLoopDepth != 0) ? 1 : 0)) - 1);
        Value * blockIndex = mKernel->getScalarField("CarryBlockIndex");
        Value * carryIndex0 = builder->CreateSub(blockIndex, builder->getSize(blocks));
        Value * loadIndex0 = builder->CreateAnd(carryIndex0, indexMask);
        Value * const carryInPtr = builder->CreateGEP(mCurrentFrame, {builder->getInt32(0), builder->getInt32(frameIndex), loadIndex0});
        Value * carryIn = builder->CreateBlockAlignedLoad(carryInPtr);

        Value * storeIndex = builder->CreateAnd(blockIndex, indexMask);
        Value * const carryOutPtr = builder->CreateGEP(mCurrentFrame, {builder->getInt32(0), builder->getInt32(frameIndex), storeIndex});
        assert (carryIn->getType() == builder->getBitBlockType());

        // If the long advance is an exact multiple of BitBlockWidth, we simply return the oldest
        // block in the long advance carry data area.
        if (LLVM_UNLIKELY(blockShift == 0)) {
            builder->CreateBlockAlignedStore(value, carryOutPtr);
            return carryIn;
        } else { // Otherwise we need to combine data from the two oldest blocks.
            Value * carryIndex1 = builder->CreateSub(blockIndex, builder->getSize(blocks - 1));
            Value * loadIndex1 = builder->CreateAnd(carryIndex1, indexMask);
            Value * const carryInPtr2 = builder->CreateGEP(mCurrentFrame, {builder->getInt32(0), builder->getInt32(frameIndex), loadIndex1});
            Value * carry_block1 = builder->CreateBlockAlignedLoad(carryInPtr2);
            Value * block0_shr = builder->CreateLShr(builder->CreateBitCast(carryIn, streamTy), builder->getBitBlockWidth() - blockShift);
            Value * block1_shl = builder->CreateShl(builder->CreateBitCast(carry_block1, streamTy), blockShift);
            builder->CreateBlockAlignedStore(value, carryOutPtr);
            return builder->CreateBitCast(builder->CreateOr(block1_shl, block0_shr), builder->getBitBlockType());
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNextCarryIn
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::getNextCarryIn(IDISA::IDISA_Builder * const builder) {
    assert (mCurrentFrameIndex < mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    if (mLoopDepth == 0) {
        mCarryPackPtr = builder->CreateGEP(mCurrentFrame, {builder->getInt32(0), builder->getInt32(mCurrentFrameIndex)});
    } else {
        mCarryPackPtr = builder->CreateGEP(mCurrentFrame, {builder->getInt32(0), builder->getInt32(mCurrentFrameIndex), mLoopSelector});
    }
    Type * const carryTy = builder->getBitBlockType();
    assert (mCarryPackPtr->getType()->getPointerElementType() == carryTy);
    Value * const carryIn = builder->CreateBlockAlignedLoad(mCarryPackPtr);
    if (mLoopDepth > 0) {
        builder->CreateBlockAlignedStore(Constant::getNullValue(carryTy), mCarryPackPtr);
    }
    return carryIn;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setNextCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::setNextCarryOut(IDISA::IDISA_Builder * const builder, Value * carryOut) {
    Type * const carryTy = builder->getBitBlockType();
    assert (mCurrentFrameIndex < mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    carryOut = builder->CreateBitCast(carryOut, carryTy);
    if (mCarryInfo->hasSummary()) {
        addToCarryOutSummary(builder, carryOut);
    }
    if (mLoopDepth != 0) {
        mCarryPackPtr = builder->CreateGEP(mCurrentFrame, {builder->getInt32(0), builder->getInt32(mCurrentFrameIndex), mNextLoopSelector});
        if (LLVM_LIKELY(!mCarryInfo->nonCarryCollapsingMode())) {
            Value * accum = builder->CreateBlockAlignedLoad(mCarryPackPtr);
            carryOut = builder->CreateOr(carryOut, accum);
        }
    }
    ++mCurrentFrameIndex;
    assert (mCarryPackPtr->getType()->getPointerElementType() == carryTy);
    builder->CreateBlockAlignedStore(carryOut, mCarryPackPtr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readCarryInSummary
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::readCarryInSummary(IDISA::IDISA_Builder * const builder, ConstantInt * index) const {
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
    std::fill(indicies, indicies + count - 1, builder->getInt32(0));
    indicies[count - 1] = index;
    if (mLoopDepth != 0) {
        indicies[count] = mLoopSelector;
    }

    ArrayRef<Value *> ar(indicies, length);
    Value * const ptr = builder->CreateGEP(mCurrentFrame, ar);
    Value * const summary = builder->CreateBlockAlignedLoad(ptr);
    if (mLoopDepth != 0 && mCarryInfo->hasExplicitSummary()) {
        Type * const carryTy = builder->getBitBlockType();
        builder->CreateBlockAlignedStore(Constant::getNullValue(carryTy), ptr);
    }
    return summary;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeCarryOutSummary
 ** ------------------------------------------------------------------------------------------------------------- */
inline void CarryManager::writeCarryOutSummary(IDISA::IDISA_Builder * const builder, Value * const summary, ConstantInt * index) const {
    Value * ptr = nullptr;
    assert (mCarryInfo->hasExplicitSummary());
    if (mLoopDepth > 0) {
        ptr = builder->CreateGEP(mCurrentFrame, {builder->getInt32(0), index, mNextLoopSelector});
    } else {
        ptr = builder->CreateGEP(mCurrentFrame, {builder->getInt32(0), index});
    }
    builder->CreateBlockAlignedStore(summary, ptr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addToCarryOutSummary
 ** ------------------------------------------------------------------------------------------------------------- */
inline void CarryManager::addToCarryOutSummary(IDISA::IDISA_Builder * const builder, Value * const value) {
    assert ("cannot add null summary value!" && value);    
    assert ("summary stack is empty!" && !mCarrySummaryStack.empty());
    assert (mCarryInfo->hasSummary());
    mCarrySummaryStack.back() = builder->CreateOr(value, mCarrySummaryStack.back());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enumerate
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned CarryManager::getScopeCount(const PabloBlock * const scope, unsigned index) {
    for (const Statement * stmt : *scope) {
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
StructType * CarryManager::analyse(IDISA::IDISA_Builder * const builder, const PabloBlock * const scope, const unsigned ifDepth, const unsigned loopDepth, const bool isNestedWithinNonCarryCollapsingLoop) {
    assert ("scope cannot be null!" && scope);
    assert (mCarryScopes == 0 ? (scope == mKernel->getEntryBlock()) : (scope != mKernel->getEntryBlock()));
    assert (mCarryScopes < mCarryMetadata.size());
    Type * const carryTy = builder->getBitBlockType();
    Type * const blockTy = builder->getBitBlockType();

    const unsigned carryScopeIndex = mCarryScopes++;
    const bool nonCarryCollapsingMode = hasIterationSpecificAssignment(scope);
    Type * const carryPackType = (loopDepth == 0) ? carryTy : ArrayType::get(carryTy, 2);
    std::vector<Type *> state;

    for (const Statement * stmt : *scope) {
        if (LLVM_UNLIKELY(isa<Advance>(stmt))) {
            const auto amount = cast<Advance>(stmt)->getAmount();
            Type * type = carryPackType;
            if (LLVM_UNLIKELY(amount >= LONG_ADVANCE_BREAKPOINT)) {
                const unsigned blocks = ceil_udiv(amount, builder->getBitBlockWidth());
                type = ArrayType::get(blockTy, nearest_pow2(blocks + ((loopDepth != 0) ? 1 : 0)));
                if (LLVM_UNLIKELY(ifDepth > 0 && amount > builder->getBitBlockWidth())) {
                    // 1 bit will mark the presense of any bit in each block.
                    Type * carryType = ArrayType::get(blockTy, ceil_udiv(amount, std::pow(builder->getBitBlockWidth(), 2)));
                    state.push_back(carryType);
                }
                mHasLongAdvance = true;                
            }
            state.push_back(type);
        } else if (LLVM_UNLIKELY(isNonAdvanceCarryGeneratingStatement(stmt))) {
            state.push_back(carryPackType);
        } else if (LLVM_UNLIKELY(isa<If>(stmt))) {
            state.push_back(analyse(builder, cast<If>(stmt)->getBody(), ifDepth + 1, loopDepth, nonCarryCollapsingMode | isNestedWithinNonCarryCollapsingLoop));
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            mHasLoop = true;
            state.push_back(analyse(builder, cast<While>(stmt)->getBody(), ifDepth, loopDepth + 1, nonCarryCollapsingMode | isNestedWithinNonCarryCollapsingLoop));
        }
    }
    // Build the carry state struct and add the summary pack if needed.
    CarryData & cd = mCarryMetadata[carryScopeIndex];
    StructType * carryState = nullptr;
    CarryData::SummaryType summaryType = CarryData::NoSummary;
    if (LLVM_UNLIKELY(state.empty())) {
        carryState = StructType::get(builder->getContext());
    } else {
        //if (ifDepth > 0 || (nonCarryCollapsingMode | isNestedWithinNonCarryCollapsingLoop)) {
        if (dyn_cast_or_null<If>(scope->getBranch()) || nonCarryCollapsingMode || isNestedWithinNonCarryCollapsingLoop) {
            if (LLVM_LIKELY(state.size() > 1)) {
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
        carryState = StructType::get(builder->getContext(), state);
        // If we're in a loop and cannot use collapsing carry mode, convert the carry state struct into a capacity,
        // carry state pointer, and summary pointer struct.
        if (LLVM_UNLIKELY(nonCarryCollapsingMode)) {
            carryState = StructType::get(builder->getSizeTy(), carryState->getPointerTo(), carryTy->getPointerTo(), nullptr);
        }
        cd.setNonCollapsingCarryMode(nonCarryCollapsingMode);
    }
    cd.setSummaryType(summaryType);
    return carryState;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
CarryManager::CarryManager() noexcept
: mKernel(nullptr)
, mCurrentFrame(nullptr)
, mCurrentFrameIndex(0)
, mCurrentScope(nullptr)
, mCarryInfo(nullptr)
, mNextSummaryTest(nullptr)
, mIfDepth(0)
, mHasLongAdvance(false)
, mHasLoop(false)
, mLoopDepth(0)
, mLoopSelector(nullptr)
, mNextLoopSelector(nullptr)
, mCarryPackPtr(nullptr)
, mCarryScopes(0) {

}

}
