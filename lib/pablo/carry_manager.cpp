/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/carry_manager.h>

#include <pablo/carry_data.h>
#include <pablo/codegenstate.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/Transforms/Utils/Local.h>
#include <pablo/branch.h>
#include <pablo/pablo_intrinsic.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_var.h>
#include <kernel/core/kernel_builder.h>
#include <toolchain/toolchain.h>
#include <array>

using namespace llvm;

using BuilderRef = pablo::CarryManager::BuilderRef;

namespace pablo {

inline static unsigned ceil_log2(const unsigned v) {
    assert ("log2(0) is undefined!" && v != 0);
    return (sizeof(unsigned) * CHAR_BIT) - __builtin_clz(v - 1U);
}

inline static unsigned floor_log2(const unsigned v) {
    assert ("log2(0) is undefined!" && v != 0);
    return ((sizeof(unsigned) * CHAR_BIT) - 1U) - __builtin_clz(v);
}

inline static unsigned nearest_pow2(const unsigned v) {
    assert (v > 0 && v < (UINT32_MAX / 2));
    return (v < 2) ? 1 : (1U << ceil_log2(v));
}

inline static unsigned udiv(const unsigned x, const unsigned y) {
    assert (is_power_2(y));
    const unsigned z = x >> floor_log2(y);
    assert (z == (x / y));
    return z;
}

inline static unsigned ceil_udiv(const unsigned x, const unsigned y) {
    return (((x - 1) | (y - 1)) + 1) / y;
}

using TypeId = PabloAST::ClassTypeId;

inline static bool isNonAdvanceCarryGeneratingStatement(const Statement * const stmt) {
    if (IntrinsicCall const * call = dyn_cast<IntrinsicCall>(stmt)) {
        return call->isCarryProducing() && !call->isAdvanceType();
    } else {
        return isa<CarryProducingStatement>(stmt) && !isa<Advance>(stmt) && !isa<IndexedAdvance>(stmt);
    }
}

#define LONG_ADVANCE_BREAKPOINT 64

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeCarryData
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::initializeCarryData(BuilderRef b, PabloKernel * const kernel) {

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
    mKernel = kernel;

    mCarryScopes = 0;

    PabloBlock * const entryScope = kernel->getEntryScope();
    mCarryMetadata.resize(getScopeCount(entryScope));

    Type * const carryStateTy = analyse(b, entryScope);
    if (LLVM_UNLIKELY(carryStateTy->isEmptyTy())) {
        // temporary refactoring step
        kernel->addNonPersistentScalar(carryStateTy, "carries");
    } else {
        kernel->addInternalScalar(carryStateTy, "carries");
    }

    if (mHasLoop) {
        kernel->addInternalScalar(b->getInt32Ty(), "selector");
    }
    if (mHasLongAdvance) {
        kernel->addInternalScalar(b->getInt32Ty(), "CarryBlockIndex");
    }
    for (unsigned i = 0; i < mIndexedLongAdvanceTotal; i++) {
        kernel->addInternalScalar(b->getSizeTy(), "IndexedAdvancePosition" + std::to_string(i));
    }
}

bool isDynamicallyAllocatedType(const Type * const ty) {
    if (isa<StructType>(ty) && ty->getStructNumElements() == 3) {
        return (ty->getStructElementType(1)->isPointerTy() && ty->getStructElementType(2)->isPointerTy() && ty->getStructElementType(0)->isIntegerTy());
    }
    return false;
}

bool containsDynamicallyAllocatedType(const Type * const ty) {
    if (isa<StructType>(ty)) {
        for (unsigned i = 0; i < ty->getStructNumElements(); ++i) {
            if (isDynamicallyAllocatedType(ty->getStructElementType(i))) {
                return true;
            }
        }
    }
    return false;
}

void freeDynamicallyAllocatedMemory(BuilderRef b, Value * const frame) {
    StructType * const ty = cast<StructType>(frame->getType()->getPointerElementType());
    std::array<Value *, 3> indices;
    indices[0] = b->getInt32(0);
    for (unsigned i = 0; i < ty->getStructNumElements(); ++i) {
        if (isDynamicallyAllocatedType(ty->getStructElementType(i))) {
            indices[1] = b->getInt32(i);
            indices[2] = b->getInt32(1);
            Value * const innerFrame = b->CreateLoad(b->CreateGEP(frame, ArrayRef<Value*>(indices.data(), 3)));
            if (containsDynamicallyAllocatedType(innerFrame->getType())) {
                indices[2] = indices[0];
                Value *  const count = b->CreateLoad(b->CreateGEP(frame, ArrayRef<Value*>(indices.data(), 3)));
                BasicBlock * const entry = b->GetInsertBlock();
                BasicBlock * const cond = b->CreateBasicBlock("freeCarryDataCond");
                BasicBlock * const body = b->CreateBasicBlock("freeCarryDataLoop");
                BasicBlock * const exit = b->CreateBasicBlock("freeCarryDataExit");
                b->CreateBr(cond);
                b->SetInsertPoint(cond);
                PHINode * const index = b->CreatePHI(count->getType(), 2);
                index->addIncoming(ConstantInt::getNullValue(count->getType()), entry);
                Value * test = b->CreateICmpNE(index, count);
                b->CreateCondBr(test, body, exit);
                b->SetInsertPoint(body);
                freeDynamicallyAllocatedMemory(b, b->CreateGEP(innerFrame, index));
                index->addIncoming(b->CreateAdd(index, ConstantInt::get(count->getType(), 1)), body);
                b->CreateBr(cond);
                b->SetInsertPoint(exit);
            }
            b->CreateFree(innerFrame);
            indices[2] = b->getInt32(2);
            Value *  const summary = b->CreateLoad(b->CreateGEP(frame, ArrayRef<Value*>(indices.data(), 3)));
            b->CreateFree(summary);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief releaseCarryData
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::releaseCarryData(BuilderRef idb) {
    if (mHasNonCarryCollapsingLoops) {
        freeDynamicallyAllocatedMemory(idb, idb->getScalarFieldPtr("carries"));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief clearCarryState
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::clearCarryData(BuilderRef idb) {



}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeCodeGen
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::initializeCodeGen(BuilderRef b) {

    assert(!mCarryMetadata.empty());
    mCarryInfo = &mCarryMetadata[0];
    assert (!mCarryInfo->hasSummary());
    mCurrentFrame = b->getScalarFieldPtr("carries");
    // assert (mKernel->isStateful() || mCurrentFrame->getType()->isEmptyTy());
    mCurrentFrameIndex = 0;
    mCarryScopes = 0;
    mCarryScopeIndex.push_back(0);
    assert (mCarryFrameStack.empty());
    assert (mCarrySummaryStack.empty());

    Type * const carryTy = b->getBitBlockType();
    mCarrySummaryStack.push_back(Constant::getNullValue(carryTy));
    if (mHasLoop) {
        mLoopSelector = b->getScalarField("selector");
        mNextLoopSelector = b->CreateXor(mLoopSelector, ConstantInt::get(mLoopSelector->getType(), 1));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalizeCodeGen
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::finalizeCodeGen(BuilderRef b) {
    if (mHasLoop) {
        b->setScalarField("selector", mNextLoopSelector);
    }
    if (mHasLongAdvance) {
        Value * idx = b->getScalarField("CarryBlockIndex");
        idx = b->CreateAdd(idx, b->getInt32(1));
        b->setScalarField("CarryBlockIndex", idx);
    }
    assert (mNestedLoopCarryInMaskPhi == nullptr);
    assert (mCarryFrameStack.empty());
    assert ("base summary value was deleted!" && mCarrySummaryStack.size() >= 1);
    assert ("not all summaries were used!" && mCarrySummaryStack.size() == 1);
    assert ("base summary value was overwritten with non-zero value!" && isa<Constant>(mCarrySummaryStack[0]) && cast<Constant>(mCarrySummaryStack[0])->isNullValue());
    mCarrySummaryStack.clear();
    assert (mCarryScopeIndex.size() == 1);
    mCarryScopeIndex.clear();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterLoopScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterLoopScope(BuilderRef b, const PabloBlock * const /* scope */) {
    assert (mHasLoop);
    ++mLoopDepth;
    enterScope(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterLoopBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterLoopBody(BuilderRef b, BasicBlock * const entryBlock) {
    assert (mLoopDepth > 0);

    Type * const carryTy = b->getBitBlockType();
    if (mLoopDepth == 1) {
        Constant * const ONES = Constant::getAllOnesValue(carryTy);
        mNestedLoopCarryInMaskPhi = b->CreatePHI(carryTy, 2, "loopCarryInMask");
        mNestedLoopCarryInMaskPhi->addIncoming(ONES, entryBlock);
    }

    if (mCarryInfo->hasSummary()) {
        Constant * const ZEROES = Constant::getNullValue(carryTy);
        PHINode * const phiCarryOutSummary = b->CreatePHI(carryTy, 2, "whileCarryOutSummary");
        phiCarryOutSummary->addIncoming(ZEROES, entryBlock);

        // Replace the incoming carry summary with the phi node and add the phi node to the stack  so that we can
        // properly OR it into the outgoing summary value.
        // NOTE: this may change the base summary value; when exiting to the base scope, replace this summary with
        // a null value to prevent subsequent nested scopes from inheriting the summary of this scope.

        // (1) Carry-ins: (a) incoming carry data first iterations, (b) zero thereafter
        // (2) Carry-out accumulators: (a) zero first iteration, (b) |= carry-out of each iteration
        mCarrySummaryStack.push_back(phiCarryOutSummary); // original carry out summary phi
        mCarrySummaryStack.push_back(phiCarryOutSummary); // accumulated carry out summary value
    }

    if (LLVM_UNLIKELY(mCarryInfo->nonCarryCollapsingMode())) {

        assert (mCarryInfo->hasSummary());

        Type * const int8PtrTy = b->getInt8PtrTy();
        Type * const carryTy = b->getBitBlockType();
        PointerType * const carryPtrTy = carryTy->getPointerTo();

        // Check whether we need to resize the carry state
        PHINode * index = b->CreatePHI(b->getSizeTy(), 2);
        mLoopIndicies.push_back(index);
        index->addIncoming(b->getSize(0), entryBlock);
        Value * capacityPtr = b->CreateGEP(mCurrentFrame, {b->getInt32(0), b->getInt32(0)});
        Value * capacity = b->CreateLoad(capacityPtr, "capacity");
        Constant * const ONE = ConstantInt::get(capacity->getType(), 1);
        Value * arrayPtr = b->CreateGEP(mCurrentFrame, {b->getInt32(0), b->getInt32(1)});
        Value * array = b->CreateLoad(arrayPtr, "array");
        BasicBlock * const entry = b->GetInsertBlock();
        BasicBlock * const resizeCarryState = b->CreateBasicBlock("ResizeCarryState");
        BasicBlock * const reallocExisting = b->CreateBasicBlock("ReallocExisting");
        BasicBlock * const createNew = b->CreateBasicBlock("CreateNew");
        BasicBlock * const resumeKernel = b->CreateBasicBlock("ResumeKernel");
        b->CreateLikelyCondBr(b->CreateICmpNE(index, capacity), resumeKernel, resizeCarryState);

        // RESIZE CARRY BLOCK
        b->SetInsertPoint(resizeCarryState);
        const auto BlockWidth = b->getBitBlockWidth() / 8;
        const auto Log2BlockWidth = floor_log2(BlockWidth);
        Constant * const carryStateWidth = ConstantExpr::getIntegerCast(ConstantExpr::getSizeOf(array->getType()->getPointerElementType()), b->getSizeTy(), false);
        Value * const summaryPtr = b->CreateGEP(mCurrentFrame, {b->getInt32(0), b->getInt32(2)});
        Value * const hasCarryState = b->CreateICmpNE(array, ConstantPointerNull::get(cast<PointerType>(array->getType())));
        b->CreateLikelyCondBr(hasCarryState, reallocExisting, createNew);

        // REALLOCATE EXISTING
        b->SetInsertPoint(reallocExisting);
        Value * const capacitySize = b->CreateMul(capacity, carryStateWidth);
        Value * const newCapacitySize = b->CreateShl(capacitySize, 1); // x 2
        Value * const newArray = b->CreateCacheAlignedMalloc(newCapacitySize);
        b->CreateMemCpy(newArray, array, capacitySize, b->getCacheAlignment());
        b->CreateFree(array);
        b->CreateStore(newArray, arrayPtr);
        Value * const startNewArrayPtr = b->CreateGEP(b->CreatePointerCast(newArray, int8PtrTy), capacitySize);
        b->CreateMemZero(startNewArrayPtr, capacitySize, BlockWidth);
        Value * const newCapacity = b->CreateShl(capacity, 1);
        b->CreateStore(newCapacity, capacityPtr);
        Value * const summary = b->CreateLoad(summaryPtr, false);
        Value * const summarySize = b->CreateShl(b->CreateAdd(b->CreateCeilLog2(capacity), ONE), Log2BlockWidth + 1);
        Constant * const additionalSpace = b->getSize(2 * BlockWidth);
        Value * const newSummarySize = b->CreateAdd(summarySize, additionalSpace);
        Value * const newSummary = b->CreateBlockAlignedMalloc(newSummarySize);
        b->CreateMemCpy(newSummary, summary, summarySize, BlockWidth);
        b->CreateFree(summary);
        b->CreateStore(b->CreatePointerCast(newSummary, carryPtrTy), summaryPtr);
        Value * const startNewSummaryPtr = b->CreateGEP(b->CreatePointerCast(newSummary, int8PtrTy), summarySize);
        b->CreateMemZero(startNewSummaryPtr, additionalSpace, BlockWidth);
        b->CreateBr(resumeKernel);

        // CREATE NEW
        b->SetInsertPoint(createNew);
        Constant * const initialLog2Capacity = b->getInt64(4);
        Constant * const initialCapacity = ConstantExpr::getShl(ONE, initialLog2Capacity);
        b->CreateStore(initialCapacity, capacityPtr);
        Constant * const initialCapacitySize = ConstantExpr::getMul(initialCapacity, carryStateWidth);
        Value * initialArray = b->CreateCacheAlignedMalloc(initialCapacitySize);
        b->CreateMemZero(initialArray, initialCapacitySize, BlockWidth);
        initialArray = b->CreatePointerCast(initialArray, array->getType());
        b->CreateStore(initialArray, arrayPtr);
        Constant * initialSummarySize = ConstantExpr::getShl(ConstantExpr::getAdd(initialLog2Capacity, b->getInt64(1)), b->getInt64(Log2BlockWidth + 1));
        Value * initialSummary = b->CreateBlockAlignedMalloc(initialSummarySize);
        b->CreateMemZero(initialSummary, initialSummarySize, BlockWidth);
        initialSummary = b->CreatePointerCast(initialSummary, carryPtrTy);
        b->CreateStore(initialSummary, summaryPtr);
        b->CreateBr(resumeKernel);

        // RESUME KERNEL
        b->SetInsertPoint(resumeKernel);
        PHINode * phiArrayPtr = b->CreatePHI(array->getType(), 3);
        phiArrayPtr->addIncoming(array, entry);
        phiArrayPtr->addIncoming(initialArray, createNew);
        phiArrayPtr->addIncoming(newArray, reallocExisting);

        // NOTE: the 3 here is only to pass the assertion later. It refers to the number of elements in the carry data struct.
        mCarryFrameStack.emplace_back(mCurrentFrame, 3);
        mCurrentFrame = b->CreateGEP(phiArrayPtr, index);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveLoopBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveLoopBody(BuilderRef b, BasicBlock * const exitBlock) {
    assert (mLoopDepth > 0);
    Type * const carryTy = b->getBitBlockType();
    if (LLVM_UNLIKELY(mCarryInfo->nonCarryCollapsingMode())) {

        assert (mCarryInfo->hasSummary());

        Value * const carryInAccumulator = readCarryInSummary(b);
        Value * const carryOutAccumulator = mCarrySummaryStack.back();

        if (mCarryInfo->hasExplicitSummary()) {
            writeCarryOutSummary(b, carryOutAccumulator);
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
        // Otherwise we will end up with an incorrect result or being trapped in an infinite loop.

        Value * capacityPtr = b->CreateGEP(mCurrentFrame, {b->getInt32(0), b->getInt32(0)});
        Value * capacity = b->CreateLoad(capacityPtr, false);
        Value * summaryPtr = b->CreateGEP(mCurrentFrame, {b->getInt32(0), b->getInt32(2)});
        Value * summary = b->CreateLoad(summaryPtr, false);

        Constant * const ONE = ConstantInt::get(capacity->getType(), 1);

        Value * loopSelector = b->CreateZExt(mLoopSelector, capacity->getType());

        BasicBlock * entry = b->GetInsertBlock();
        BasicBlock * update = b->CreateBasicBlock("UpdateNonCarryCollapsingSummary");
        BasicBlock * resume = b->CreateBasicBlock("ResumeAfterUpdatingNonCarryCollapsingSummary");

        b->CreateBr(update);

        b->SetInsertPoint(update);
        PHINode * i = b->CreatePHI(capacity->getType(), 2);
        i->addIncoming(ConstantInt::getNullValue(capacity->getType()), entry);
        PHINode * const borrow = b->CreatePHI(carryInAccumulator->getType(), 2);
        borrow->addIncoming(carryInAccumulator, entry);
        PHINode * const carry = b->CreatePHI(carryOutAccumulator->getType(), 2);
        carry->addIncoming(carryOutAccumulator, entry);
        // OR the updated carry in summary later for the summaryTest
        PHINode * const carryInSummary = b->CreatePHI(carryTy, 2);
        carryInSummary->addIncoming(Constant::getNullValue(carryTy), entry);

        // half subtractor
        Value * const carryInOffset = b->CreateOr(b->CreateShl(i, 1), loopSelector);
        Value * const carryInPtr = b->CreateGEP(summary, carryInOffset);
        Value * const carryIn = b->CreateBlockAlignedLoad(carryInPtr);
        Value * const nextCarryIn = b->CreateXor(carryIn, borrow);
        Value * const nextSummary = b->CreateOr(carryInSummary, nextCarryIn);

        b->CreateBlockAlignedStore(nextCarryIn, carryInPtr);
        carryInSummary->addIncoming(nextSummary, update);
        Value * finalBorrow = b->CreateAnd(b->CreateNot(carryIn), borrow);
        borrow->addIncoming(finalBorrow, update);

        // half adder
        Value * const carryOutOffset = b->CreateXor(carryInOffset, ConstantInt::get(carryInOffset->getType(), 1));
        Value * const carryOutPtr = b->CreateGEP(summary, carryOutOffset);
        Value * const carryOut = b->CreateBlockAlignedLoad(carryOutPtr);
        Value * const nextCarryOut = b->CreateXor(carryOut, carry);

        b->CreateBlockAlignedStore(nextCarryOut, carryOutPtr);
        Value * finalCarry = b->CreateAnd(carryOut, carry);
        carry->addIncoming(finalCarry, update);

        // loop condition
        i->addIncoming(b->CreateAdd(i, ONE), update);
        b->CreateCondBr(b->CreateICmpNE(b->CreateShl(ONE, i), capacity), update, resume);

        b->SetInsertPoint(resume);

        b->CreateAssertZero(b->CreateOr(finalBorrow, finalCarry),
                                   "CarryManager: loop post-condition violated: final borrow and carry must be zero!");

        assert (!mLoopIndicies.empty());
        PHINode * index = mLoopIndicies.back();
        index->addIncoming(b->CreateAdd(index, b->getSize(1)), resume);
        mLoopIndicies.pop_back();
        mNextSummaryTest = nextSummary;
    }

    if (mLoopDepth == 1) {
        Constant * const ZEROES = Constant::getNullValue(carryTy);
        mNestedLoopCarryInMaskPhi->addIncoming(ZEROES, exitBlock);
    }

    if (mCarryInfo->hasSummary()) {
        const auto n = mCarrySummaryStack.size(); assert (n > 2);

        // (1) Carry-ins: (a) incoming carry data first iterations, (b) zero thereafter
        // (2) Carry-out accumulators: (a) zero first iteration, (b) |= carry-out of each iteration

        Value * const carryOut = mCarrySummaryStack[n - 1];
        PHINode * const phiCarryOut = cast<PHINode>(mCarrySummaryStack[n - 2]);
        phiCarryOut->addIncoming(carryOut, exitBlock);
        mCarrySummaryStack[n - 2] = carryOut; // replace summary out phi with the final value
        mCarrySummaryStack.pop_back(); // discard updated carry out value
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveLoopScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveLoopScope(BuilderRef b, BasicBlock * const entryBlock, BasicBlock * const exitBlock) {
    phiCurrentCarryOutSummary(b, entryBlock, exitBlock);
    writeCurrentCarryOutSummary(b);
    combineCarryOutSummary(b, 2);
    leaveScope();
    assert (mLoopDepth > 0);
    assert (mCarrySummaryStack.size() > 0);
    --mLoopDepth;
    if (mLoopDepth == 0) {
        mNestedLoopCarryInMaskPhi = nullptr;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterIfScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterIfScope(BuilderRef b, const PabloBlock * const /* scope */) {
    ++mIfDepth;
    enterScope(b);
    // We zero-initialized the nested summary value and later OR in the current summary into the escaping summary
    // so that upon processing the subsequent block iteration, we branch into this If scope iff a carry out was
    // generated by a statement within this If scope and not by a dominating statement in the outer scope.
    if (mCarryInfo->hasSummary()) {
        Type * const carryTy = b->getBitBlockType();
        mCarrySummaryStack.push_back(Constant::getNullValue(carryTy)); // new carry out summary accumulator
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterIfBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterIfBody(BuilderRef /* b */, BasicBlock * const entryBlock) {
    assert (mIfDepth > 0);
    assert (entryBlock);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveIfBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveIfBody(BuilderRef b, BasicBlock * const exitBlock) {
    assert (mIfDepth > 0);
    assert (exitBlock);
    writeCurrentCarryOutSummary(b);
    combineCarryOutSummary(b, 1);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveIfScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveIfScope(BuilderRef b, BasicBlock * const entryBlock, BasicBlock * const exitBlock) {
    assert (mIfDepth > 0);
    --mIfDepth;
    phiOuterCarryOutSummary(b, entryBlock, exitBlock);
    leaveScope();
}

/** ------------------------------------------------------------------------------------------------------------ *
 * @brief enterScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterScope(BuilderRef b) {
    // Store the state of the current frame and update the scope state
    mCarryFrameStack.emplace_back(mCurrentFrame, mCurrentFrameIndex + 1);
    mCarryScopeIndex.push_back(++mCarryScopes);
    mCarryInfo = &mCarryMetadata[mCarryScopes];
    // Check whether we're still within our struct bounds; if this fails, either the Pablo program changed during
    // compilation or a memory corruption has occured.
    assert (mCurrentFrameIndex < mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    mCurrentFrame = b->CreateGEP(mCurrentFrame, {b->getInt32(0), b->getInt32(mCurrentFrameIndex)});
    // Verify we're pointing to a carry frame struct
    assert(mCurrentFrame->getType()->getPointerElementType()->isStructTy());
    mCurrentFrameIndex = 0;
    mNextSummaryTest = nullptr;
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        mNextSummaryTest = readCarryInSummary(b);
        if (mCarryInfo->hasExplicitSummary()) {
            mCurrentFrameIndex = 1;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateSummaryTest
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::generateSummaryTest(BuilderRef b, Value * condition) {
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        assert ("summary condition cannot be null!" && condition);
        assert ("summary test was not generated" && mNextSummaryTest);
        assert ("summary condition and test must have the same context!" && &condition->getContext() == &mNextSummaryTest->getContext());
        assert ("summary condition and test must be equivalent types!" && (condition->getType()->canLosslesslyBitCastTo(mNextSummaryTest->getType())));
        condition = b->simd_or(condition, mNextSummaryTest, "condWithSummaryTest");
        mNextSummaryTest = nullptr;
    }
    assert ("summary test was not consumed" && (mNextSummaryTest == nullptr));
    return condition;
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
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        mCarrySummaryStack.pop_back();
    }
    mCarryInfo = &mCarryMetadata[mCarryScopeIndex.back()];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief combineCarryOutSummary
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::combineCarryOutSummary(BuilderRef b, const unsigned offset) {
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        const auto n = mCarrySummaryStack.size(); assert (n > 0);
        // combine the outer summary with the nested summary so that when
        // we leave the scope, we'll properly phi out the value of the new
        // outer summary
        if (n > 2) {
            Value * const nested = mCarrySummaryStack[n - 1];
            Value * const outer = mCarrySummaryStack[n - 2];
            assert (nested->getType() == outer->getType());
            mCarrySummaryStack[n - offset] =  b->CreateOr(outer, nested);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeNextCarryOutSummary
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::writeCurrentCarryOutSummary(BuilderRef b) {
    if (LLVM_LIKELY(mCarryInfo->hasExplicitSummary())) {
        const auto n = mCarrySummaryStack.size(); assert (n > 0);
        writeCarryOutSummary(b, mCarrySummaryStack[n - 1]);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief phiCurrentCarryOutSummary
 ** ------------------------------------------------------------------------------------------------------------- */
inline void CarryManager::phiCurrentCarryOutSummary(BuilderRef b, BasicBlock * const entryBlock, BasicBlock * const exitBlock) {
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        const auto n = mCarrySummaryStack.size(); assert (n > 0);
        Type * const carryTy = b->getBitBlockType();
        Constant * const ZEROES = Constant::getNullValue(carryTy);
        Value * const nested = mCarrySummaryStack[n - 1];
        PHINode * const phi = b->CreatePHI(nested->getType(), 2, "summary");
        phi->addIncoming(ZEROES, entryBlock);
        phi->addIncoming(nested, exitBlock);
        mCarrySummaryStack[n - 1] = phi;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief phiOuterCarryOutSummary
 ** ------------------------------------------------------------------------------------------------------------- */
inline void CarryManager::phiOuterCarryOutSummary(BuilderRef b, BasicBlock * const entryBlock, BasicBlock * const exitBlock) {
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        const auto n = mCarrySummaryStack.size(); assert (n > 0);
        if (n > 2) {
            // When leaving a nested If scope with a summary value, phi out the summary to ensure the
            // appropriate summary is stored in the outer scope.
            Value * nested = mCarrySummaryStack[n - 1];
            Value * outer = mCarrySummaryStack[n - 2];
            assert (nested->getType() == outer->getType());
            PHINode * const phi = b->CreatePHI(nested->getType(), 2, "summary");
            phi->addIncoming(outer, entryBlock);
            phi->addIncoming(nested, exitBlock);
            mCarrySummaryStack[n - 2] = phi;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::addCarryInCarryOut(BuilderRef b, const Statement * const operation, Value * const e1, Value * const e2) {
    assert (operation && (isNonAdvanceCarryGeneratingStatement(operation)));
    Value * const carryIn = getNextCarryIn(b);
    Value * carryOut, * result;
    std::tie(carryOut, result) = b->bitblock_add_with_carry(e1, e2, carryIn);
    setNextCarryOut(b, carryOut);
    assert (result->getType() == b->getBitBlockType());
    return result;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief subBorrowInBorrowOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::subBorrowInBorrowOut(BuilderRef b, const Statement * operation, Value * const e1, Value * const e2) {
    assert (operation);
    Value * const borrowIn = getNextCarryIn(b);
    Value * borrowOut, * result;
    std::tie(borrowOut, result) = b->bitblock_subtract_with_propagate(e1, e2, borrowIn);
    setNextCarryOut(b, borrowOut);
    assert (result->getType() == b->getBitBlockType());
    return result;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief advanceCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::advanceCarryInCarryOut(BuilderRef b, const Advance * const advance, Value * const value) {
    const auto shiftAmount = advance->getAmount();
    if (LLVM_LIKELY(shiftAmount < LONG_ADVANCE_BREAKPOINT)) {
        Value * const carryIn = getNextCarryIn(b);
        Value * carryOut, * result;
        std::tie(carryOut, result) = b->bitblock_advance(value, carryIn, shiftAmount);
        setNextCarryOut(b, carryOut);
        assert (result->getType() == b->getBitBlockType());
        return result;
    } else {
        return longAdvanceCarryInCarryOut(b, value, shiftAmount);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief indexedAdvanceCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::indexedAdvanceCarryInCarryOut(BuilderRef b, const IndexedAdvance * const advance, Value * const strm, Value * const index_strm) {
    const auto shiftAmount = advance->getAmount();
    if (LLVM_LIKELY(shiftAmount < LONG_ADVANCE_BREAKPOINT)) {
        Value * const carryIn = getNextCarryIn(b);
        Value * carryOut, * result;
        std::tie(carryOut, result) = b->bitblock_indexed_advance(strm, index_strm, carryIn, shiftAmount);
        setNextCarryOut(b, carryOut);
        return result;
    } else if (shiftAmount <= b->getBitBlockWidth()) {
        Value * carryPtr = b->CreateGEP(mCurrentFrame, {b->getInt32(0), b->getInt32(mCurrentFrameIndex++), b->getInt32(0)});
        Value * carryIn = b->CreateBlockAlignedLoad(carryPtr);
        Value * carryOut, * result;
        std::tie(carryOut, result) = b->bitblock_indexed_advance(strm, index_strm, carryIn, shiftAmount);
        b->CreateBlockAlignedStore(carryOut, carryPtr);
        if (mCarryInfo->hasExplicitSummary()) {
            addToCarryOutSummary(b, strm);
        }
        return result;
    } else {
        unsigned summaryFrame = mCurrentFrameIndex;
        if (mIfDepth > 0 || mLoopDepth > 0) {
            // Skip over summary frame to perform the long indexed advance.
            mCurrentFrameIndex++;
        }
        Type * iBitBlock = b->getIntNTy(b->getBitBlockWidth());
        Constant * blockWidth = b->getSize(b->getBitBlockWidth());
        Constant * blockWidth_1 = b->getSize(b->getBitBlockWidth() - 1);
        Value * carryPosition = b->getScalarField("IndexedAdvancePosition" + std::to_string(mIndexedLongAdvanceIndex));
        Value * carryBlockEndPos = b->CreateAdd(carryPosition, blockWidth_1);
        unsigned carry_blocks = nearest_pow2(1+ceil_udiv(shiftAmount, b->getBitBlockWidth()));
        Constant * carryQueueBlocks = b->getSize(carry_blocks);
        Value * carryBlock = b->CreateTrunc(b->CreateURem(b->CreateUDiv(carryPosition, blockWidth), carryQueueBlocks), b->getInt32Ty());
        Value * carryEndBlock = b->CreateTrunc(b->CreateURem(b->CreateUDiv(carryBlockEndPos, blockWidth), carryQueueBlocks), b->getInt32Ty());
        Value * lo_GEP = b->CreateGEP(mCurrentFrame, {b->getInt32(0), b->getInt32(mCurrentFrameIndex), carryBlock});
        Value * hi_GEP = b->CreateGEP(mCurrentFrame, {b->getInt32(0), b->getInt32(mCurrentFrameIndex), carryEndBlock});
        Value * c_lo = b->CreateBitCast(b->CreateBlockAlignedLoad(lo_GEP), iBitBlock);
        Value * c_hi = b->CreateBitCast(b->CreateBlockAlignedLoad(hi_GEP), iBitBlock);
        Value * lo_shift = b->CreateZExt(b->CreateURem(carryPosition, blockWidth), iBitBlock);
        Value * hi_shift = b->CreateZExt(b->CreateSub(blockWidth_1, b->CreateURem(carryBlockEndPos, blockWidth)), iBitBlock);
        Value * carryIn = b->CreateOr(b->CreateLShr(c_lo, lo_shift), b->CreateShl(c_hi, hi_shift));
        Value * carryOut, * result;
        std::tie(carryOut, result) = b->bitblock_indexed_advance(strm, index_strm, carryIn, shiftAmount);
        carryOut = b->CreateBitCast(carryOut, iBitBlock);
        Value * adv = b->mvmd_extract(sizeof(size_t) * 8, b->simd_popcount(b->getBitBlockWidth(), index_strm), 0);
        b->setScalarField("IndexedAdvancePosition" + std::to_string(mIndexedLongAdvanceIndex), b->CreateAdd(carryPosition, adv));
        Value * carryOutPosition = b->CreateAdd(carryPosition, b->getSize(shiftAmount));
        Value * carryOutEndPos = b->CreateAdd(carryOutPosition, blockWidth_1);
        carryBlock = b->CreateTrunc(b->CreateURem(b->CreateUDiv(carryOutPosition, blockWidth), carryQueueBlocks), b->getInt32Ty());
        carryEndBlock = b->CreateTrunc(b->CreateURem(b->CreateUDiv(carryOutEndPos, blockWidth), carryQueueBlocks), b->getInt32Ty());
        lo_GEP = b->CreateGEP(mCurrentFrame, {b->getInt32(0), b->getInt32(mCurrentFrameIndex), carryBlock});
        hi_GEP = b->CreateGEP(mCurrentFrame, {b->getInt32(0), b->getInt32(mCurrentFrameIndex), carryEndBlock});
        lo_shift = b->CreateZExt(b->CreateURem(carryOutPosition, blockWidth), iBitBlock);
        hi_shift = b->CreateZExt(b->CreateSub(blockWidth_1, b->CreateURem(carryOutEndPos, blockWidth)), iBitBlock);
        c_lo = b->CreateOr(b->CreateBitCast(b->CreateBlockAlignedLoad(lo_GEP), iBitBlock), b->CreateShl(carryOut, lo_shift));
        c_hi = b->CreateLShr(carryOut, hi_shift);
        b->CreateBlockAlignedStore(b->CreateBitCast(c_lo, b->getBitBlockType()), lo_GEP);
        b->CreateBlockAlignedStore(b->CreateBitCast(c_hi, b->getBitBlockType()), hi_GEP);
        mIndexedLongAdvanceIndex++;
        mCurrentFrameIndex++;
        // Now handle the summary.
        if (mIfDepth > 0 || mLoopDepth > 0) {
            const auto summaryBlocks = ceil_udiv(shiftAmount, b->getBitBlockWidth());
            const auto summarySize = ceil_udiv(summaryBlocks, b->getBitBlockWidth());
            for (unsigned i = 0; i < summarySize; i++) {
                // All ones summary for now.
                b->CreateBlockAlignedStore(b->allOnes(), b->CreateGEP(mCurrentFrame, {b->getInt32(0), b->getInt32(summaryFrame), b->getInt32(i)}));
            }
        }
        return result;
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief longAdvanceCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * CarryManager::longAdvanceCarryInCarryOut(BuilderRef b, Value * const value, const unsigned shiftAmount) {

    assert (mHasLongAdvance);
    assert (shiftAmount >= LONG_ADVANCE_BREAKPOINT);
    assert (value);

    const auto blockWidth = b->getBitBlockWidth();
    Type * const streamTy = b->getIntNTy(blockWidth);
    Constant * const ZERO = b->getInt32(0);
    FixedArray<Value *, 3> indices;

    indices[0] = ZERO;

    if (mCarryInfo->hasSummary()) {
        if (shiftAmount > blockWidth) {

            // TODO: once CEILING(shiftAmount / 256) > 2, consider using a half-adder/subtractor strategy?

            Value * carry = b->CreateZExt(b->bitblock_any(value), streamTy);
            const auto summaryBlocks = ceil_udiv(shiftAmount, blockWidth);
            const auto summarySize = ceil_udiv(summaryBlocks, blockWidth);
            VectorType * const bitBlockTy = b->getBitBlockType();
            IntegerType * const laneTy = cast<IntegerType>(bitBlockTy->getVectorElementType());
            const auto laneWidth = laneTy->getIntegerBitWidth();

            assert (summarySize > 0);
            assert (is_power_2(laneWidth));

            indices[1] = b->getInt32(mCurrentFrameIndex++);

            for (unsigned i = 1;;++i) {

                assert (i <= summarySize);

                indices[2] = b->getInt32(i - 1);

                Value * const ptr = b->CreateGEP(mCurrentFrame, indices);
                Value * const prior = b->CreateBitCast(b->CreateBlockAlignedLoad(ptr), streamTy);

                Value * advanced = nullptr;
                if (LLVM_LIKELY(summaryBlocks < laneWidth)) {
                    advanced = b->CreateOr(b->CreateShl(prior, 1), carry);
                    carry = b->CreateLShr(prior, summaryBlocks - 1);
                } else {
                    std::tie(carry, advanced) = b->bitblock_advance(prior, carry, 1);
                }
                Value * stream = b->CreateBitCast(advanced, bitBlockTy);
                if (LLVM_LIKELY(i == summarySize)) {
                    const auto n = llvm::cast<llvm::VectorType>(bitBlockTy)->getNumElements();
                    SmallVector<Constant *, 16> mask(n);
                    const auto m = udiv(summaryBlocks, laneWidth);
                    if (m) {
                        std::fill_n(mask.data(), m, ConstantInt::getAllOnesValue(laneTy));
                    }
                    mask[m] = ConstantInt::get(laneTy, (1UL << (summaryBlocks & (laneWidth - 1))) - 1UL);
                    if (n > m) {
                        std::fill_n(mask.data() + m + 1, n - m, UndefValue::get(laneTy));
                    }
                    stream = b->CreateAnd(stream, ConstantVector::get(mask));
                    addToCarryOutSummary(b, stream);
                    b->CreateBlockAlignedStore(stream, ptr);
                    break;
                }
                addToCarryOutSummary(b, stream);
                b->CreateBlockAlignedStore(stream, ptr);
            }

        } else {
            addToCarryOutSummary(b, value);
        }
    }

    indices[1] = b->getInt32(mCurrentFrameIndex++);

    // special case using a single buffer entry and the carry_out value.
    if (LLVM_LIKELY((shiftAmount < blockWidth) && (mLoopDepth == 0))) {

        indices[2] = ZERO;

        Value * const buffer = b->CreateGEP(mCurrentFrame, indices);
        assert (buffer->getType()->getPointerElementType() == b->getBitBlockType());
        Value * const carryIn = b->CreateBlockAlignedLoad(buffer);

        b->CreateBlockAlignedStore(value, buffer);
        /* Very special case - no combine */
        if (LLVM_UNLIKELY(shiftAmount == blockWidth)) {
            return b->CreateBitCast(carryIn, b->getBitBlockType());
        }
        Value * const block0_shr = b->CreateLShr(b->CreateBitCast(carryIn, streamTy), blockWidth - shiftAmount);
        Value * const block1_shl = b->CreateShl(b->CreateBitCast(value, streamTy), shiftAmount);
        return b->CreateBitCast(b->CreateOr(block1_shl, block0_shr), b->getBitBlockType());
    } else { //
        const unsigned blockShift = shiftAmount & (blockWidth - 1);
        const unsigned summaryBlocks = ceil_udiv(shiftAmount, blockWidth);

        // Create a mask to implement circular buffer indexing
        Value * const indexMask = b->getInt32(nearest_pow2(summaryBlocks) - 1);
        Value * const blockIndex = b->getScalarField("CarryBlockIndex");
        assert (blockIndex->getType() == indexMask->getType());
        Value * const carryIndex0 = b->CreateSub(blockIndex, b->getInt32(summaryBlocks));
        indices[2] = b->CreateAnd(carryIndex0, indexMask);

        Value * const carryInPtr = b->CreateGEP(mCurrentFrame, indices);
        Value * const carryIn = b->CreateBlockAlignedLoad(carryInPtr);

        indices[2] = b->CreateAnd(blockIndex, indexMask);
        Value * const carryOutPtr = b->CreateGEP(mCurrentFrame, indices);
        assert (carryIn->getType() == b->getBitBlockType());

        // If the long advance is an exact multiple of BitBlockWidth, we simply return the oldest
        // block in the long advance carry data area.
        if (LLVM_UNLIKELY(blockShift == 0)) {
            b->CreateBlockAlignedStore(value, carryOutPtr);
            return carryIn;
        } else { // Otherwise we need to combine data from the two oldest blocks.
            Value * const carryIndex1 = b->CreateSub(blockIndex, b->getInt32(summaryBlocks - 1));
            indices[2] = b->CreateAnd(carryIndex1, indexMask);

            Value * const carryInPtr2 = b->CreateGEP(mCurrentFrame, indices);
            Value * const carryIn2 = b->CreateBlockAlignedLoad(carryInPtr2);
            assert (carryOutPtr->getType()->getPointerElementType() == value->getType());
            b->CreateBlockAlignedStore(value, carryOutPtr);

            Value * const b0 = b->CreateLShr(b->CreateBitCast(carryIn, streamTy), blockWidth - blockShift);
            Value * const b1 = b->CreateShl(b->CreateBitCast(carryIn2, streamTy), blockShift);
            return b->CreateBitCast(b->CreateOr(b1, b0), b->getBitBlockType());
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNextCarryIn
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::getNextCarryIn(BuilderRef b) {
    assert (mCurrentFrameIndex < mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    Constant * const ZERO = b->getInt32(0);
    FixedArray<Value *, 3> indices;
    indices[0] = ZERO;
    indices[1] = b->getInt32(mCurrentFrameIndex);
    if (mLoopDepth == 0) {
        indices[2] = ZERO;
    } else {
        indices[2] = mLoopSelector;
    }
    mCarryPackPtr = b->CreateGEP(mCurrentFrame, indices);
    Type * const carryTy = b->getBitBlockType();
    assert (mCarryPackPtr->getType()->getPointerElementType() == carryTy);
    Value * const carryIn = b->CreateBlockAlignedLoad(mCarryPackPtr);
    if (mLoopDepth > 0) {
        b->CreateBlockAlignedStore(Constant::getNullValue(carryTy), mCarryPackPtr);
    }
    return carryIn;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setNextCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::setNextCarryOut(BuilderRef b, Value * carryOut) {
    Type * const carryTy = b->getBitBlockType();
    assert (mCurrentFrameIndex < mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    carryOut = b->CreateBitCast(carryOut, carryTy);
    if (mCarryInfo->hasSummary()) {
        addToCarryOutSummary(b, carryOut);
    }
    if (mLoopDepth != 0) {
        mCarryPackPtr = b->CreateGEP(mCurrentFrame, {b->getInt32(0), b->getInt32(mCurrentFrameIndex), mNextLoopSelector});
        if (LLVM_LIKELY(!mCarryInfo->nonCarryCollapsingMode())) {
            Value * accum = b->CreateBlockAlignedLoad(mCarryPackPtr);
            carryOut = b->CreateOr(carryOut, accum);
        }
    }
    ++mCurrentFrameIndex;
    assert (mCarryPackPtr->getType()->getPointerElementType() == carryTy);
    b->CreateBlockAlignedStore(carryOut, mCarryPackPtr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readCarryInSummary
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::readCarryInSummary(BuilderRef b) const {
    assert (mCarryInfo->hasSummary());

    unsigned count = 2;
    if (LLVM_UNLIKELY(mCarryInfo->hasBorrowedSummary())) {
        Type * frameTy = mCurrentFrame->getType()->getPointerElementType();
        count = 1;
        while (frameTy->isStructTy()) {
            ++count;
            assert (frameTy->getStructNumElements() > 0);
            frameTy = frameTy->getStructElementType(0);
        }
    }
    const unsigned length = count + 1;
    SmallVector<Value *, 16> indicies(length);
    Constant * const ZERO = b->getInt32(0);
    std::fill(indicies.begin(), indicies.end(), ZERO);
    if (mLoopDepth != 0) {
        indicies[count] = mLoopSelector;
    }

    Value * const ptr = b->CreateGEP(mCurrentFrame, indicies);
    assert (ptr->getType()->getPointerElementType() == b->getBitBlockType());
    Value * summary = b->CreateBlockAlignedLoad(ptr);
    if (mNestedLoopCarryInMaskPhi) {
        summary = b->CreateAnd(summary, mNestedLoopCarryInMaskPhi);
    }
    return summary;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeCarryOutSummary
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::writeCarryOutSummary(BuilderRef b, Value * const summary) const {
    assert (mCarryInfo->hasExplicitSummary());
    Constant * const ZERO = b->getInt32(0);
    FixedArray<Value *, 3> indices;
    indices[0] = ZERO;
    indices[1] = ZERO;
    if (mLoopDepth == 0) {
        indices[2] = ZERO;
    } else {
        indices[2] = mNextLoopSelector;
    }
    Value * ptr = b->CreateGEP(mCurrentFrame, indices);
    assert (ptr->getType()->getPointerElementType() == b->getBitBlockType());
    b->CreateBlockAlignedStore(summary, ptr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addToCarryOutSummary
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::addToCarryOutSummary(BuilderRef b, Value * const value) {
    assert ("cannot add null summary value!" && value);
    assert ("summary stack is empty!" && !mCarrySummaryStack.empty());
    assert (mCarryInfo->hasSummary());
    const auto n = mCarrySummaryStack.size();
    Value * const currentSummary = mCarrySummaryStack[n - 1];
    mCarrySummaryStack[n - 1] = b->CreateOr(value, currentSummary);
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
bool isNonRegularLanguage(const PabloBlock * const scope) {
    if (const Branch * br = scope->getBranch()) {
        return !br->isRegular();
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasNonEmptyCarryStruct
 ** ------------------------------------------------------------------------------------------------------------- */
bool CarryManager::hasNonEmptyCarryStruct(const Type * const frameTy) {
    if (frameTy->isStructTy()) {
        for (unsigned i = 0; i < frameTy->getStructNumElements(); ++i) {
            if (hasNonEmptyCarryStruct(frameTy->getStructElementType(i))) {
                return true;
            }
        }
        return false;
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isEmptyCarryStruct
 ** ------------------------------------------------------------------------------------------------------------- */
bool CarryManager::isEmptyCarryStruct(const std::vector<Type *> & frameTys) {
    for (const Type * const t : frameTys) {
        if (LLVM_LIKELY(hasNonEmptyCarryStruct(t))) {
            return false;
        }
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief analyse
 ** ------------------------------------------------------------------------------------------------------------- */
StructType * CarryManager::analyse(BuilderRef b, const PabloBlock * const scope,
                                   const unsigned ifDepth, const unsigned loopDepth, const bool inNonCarryCollapsingLoop) {
    assert ("scope cannot be null!" && scope);
    assert ("entry scope (and only the entry scope) must be in scope 0"
            && (mCarryScopes == 0 ? (scope == mKernel->getEntryScope()) : (scope != mKernel->getEntryScope())));
    assert (mCarryScopes < mCarryMetadata.size());
    Type * const carryTy = b->getBitBlockType();
    Type * const blockTy = b->getBitBlockType();

    const unsigned carryScopeIndex = mCarryScopes++;
    const bool nonCarryCollapsingMode = inNonCarryCollapsingLoop || isNonRegularLanguage(scope);
    Type * const carryPackType = ArrayType::get(carryTy, ((loopDepth == 0) ? 1 : 2));
    std::vector<Type *> state;
    for (const Statement * stmt : *scope) {
        if (LLVM_UNLIKELY(isa<Advance>(stmt) || isa<IndexedAdvance>(stmt))) {
            int64_t amount;
            if (LLVM_LIKELY(isa<Advance>(stmt))) {
                amount = cast<Advance>(stmt)->getAmount();
            } else {
                amount = cast<IndexedAdvance>(stmt)->getAmount();
            }
            Type * type = carryPackType;
            if (LLVM_UNLIKELY(amount >= LONG_ADVANCE_BREAKPOINT)) {
                const auto blockWidth = b->getBitBlockWidth();
                const auto blocks = ceil_udiv(amount, blockWidth);
                unsigned required = blocks;
                if (loopDepth > 0) {
                    required++;
                }
                if (LLVM_UNLIKELY(isa<IndexedAdvance>(stmt))) {
                    required++;
                    mIndexedLongAdvanceTotal++;
                }
                type = ArrayType::get(blockTy, nearest_pow2(required));
                if (LLVM_UNLIKELY(blocks != 1 && (ifDepth > 0 || loopDepth > 0))) {
                    const auto summarySize = ceil_udiv(blocks, blockWidth);
                    // 1 bit will mark the presense of any bit in each block.
                    state.push_back(ArrayType::get(blockTy, summarySize));
                }
                mHasLongAdvance = true;
            }
            state.push_back(type);
        } else if (LLVM_UNLIKELY(isNonAdvanceCarryGeneratingStatement(stmt))) {
            state.push_back(carryPackType);
        } else if (LLVM_UNLIKELY(isa<If>(stmt))) {
            state.push_back(analyse(b, cast<If>(stmt)->getBody(), ifDepth + 1, loopDepth, nonCarryCollapsingMode));
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            mHasLoop = true;
            state.push_back(analyse(b, cast<While>(stmt)->getBody(), ifDepth, loopDepth + 1, nonCarryCollapsingMode));
        }
    }
    // Build the carry state struct and add the summary pack if needed.
    CarryData & cd = mCarryMetadata[carryScopeIndex];
    StructType * carryState = nullptr;
    CarryData::SummaryType summaryType = CarryData::NoSummary;

    // if we have at least one non-empty carry state, check if we need a summary
    if (LLVM_UNLIKELY(isEmptyCarryStruct(state))) {
        carryState = StructType::get(b->getContext(), state);
    } else {
        // do we have a summary or a sequence of nested empty structs?
        if (LLVM_LIKELY(ifDepth > 0 || loopDepth > 0)) {
            if (LLVM_LIKELY(state.size() > 1)) {
                summaryType = CarryData::ExplicitSummary;
                state.insert(state.begin(), carryPackType);
            } else {
                summaryType = CarryData::ImplicitSummary;
                if (hasNonEmptyCarryStruct(state[0])) {
                    summaryType = CarryData::BorrowedSummary;
                }
            }
        }

        carryState = StructType::get(b->getContext(), state);


        // If we're in a loop and cannot use collapsing carry mode, convert the carry state struct into a capacity,
        // carry state pointer, and summary pointer struct.
        if (LLVM_UNLIKELY(nonCarryCollapsingMode)) {
            mHasNonCarryCollapsingLoops = true;
            carryState = StructType::get(b->getContext(), {b->getSizeTy(), carryState->getPointerTo(), carryPackType->getPointerTo()});
            assert (isDynamicallyAllocatedType(carryState));
        }
        cd.setNonCollapsingCarryMode(nonCarryCollapsingMode);
    }
    cd.setSummaryType(summaryType);
    assert (carryState);
    return carryState;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
CarryManager::CarryManager() noexcept
: mKernel(nullptr)
, mCurrentFrame(nullptr)
, mCurrentFrameIndex(0)
, mCarryInfo(nullptr)
, mNextSummaryTest(nullptr)
, mIfDepth(0)
, mHasLongAdvance(false)
, mIndexedLongAdvanceTotal(0)
, mIndexedLongAdvanceIndex(0)
, mHasNonCarryCollapsingLoops(false)
, mHasLoop(false)
, mLoopDepth(0)
, mNestedLoopCarryInMaskPhi(nullptr)
, mLoopSelector(nullptr)
, mNextLoopSelector(nullptr)
, mCarryPackPtr(nullptr)
, mCarryScopes(0)
, mCarrySummaryStack() {

}

}
