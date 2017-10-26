/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "carry_manager.h"
#include <pablo/carry_data.h>
#include <pablo/codegenstate.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/Transforms/Utils/Local.h>
#include <pablo/branch.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_var.h>
#include <kernels/kernel_builder.h>
#include <toolchain/toolchain.h>
#include <array>

using namespace llvm;

namespace pablo {

inline static bool is_power_2(const unsigned n) {
    return (n && ((n & (n - 1)) == 0));
}

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

inline static unsigned nearest_multiple(const unsigned n, const unsigned m) {
    assert (is_power_2(m));
    const unsigned r = (n + m - 1U) & -m;
    assert (r >= n);
    return r;
}

inline static bool is_multiple_of(const unsigned n, const unsigned m) {
    return nearest_multiple(n, m) == n;
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
    return isa<CarryProducingStatement>(stmt) && !isa<Advance>(stmt) && !isa<IndexedAdvance>(stmt);
}

#define LONG_ADVANCE_BREAKPOINT 64

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeCarryData
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::initializeCarryData(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, PabloKernel * const kernel) {

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

    Type * const carryStateTy = analyse(iBuilder, mCurrentScope);

    kernel->addScalar(carryStateTy, "carries");

    if (mHasLoop) {
        kernel->addScalar(iBuilder->getInt32Ty(), "selector");
    }
    if (mHasLongAdvance) {
        kernel->addScalar(iBuilder->getSizeTy(), "CarryBlockIndex");
    }
    for (unsigned i = 0; i < mIndexedLongAdvanceTotal; i++) {
        kernel->addScalar(iBuilder->getSizeTy(), "LongAdvancePosition" + std::to_string(i));
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

void freeDynamicallyAllocatedMemory(const std::unique_ptr<kernel::KernelBuilder> & idb, Value * const frame) {
    StructType * const ty = cast<StructType>(frame->getType()->getPointerElementType());
    std::array<Value *, 3> indices;
    indices[0] = idb->getInt32(0);
    for (unsigned i = 0; i < ty->getStructNumElements(); ++i) {
        if (isDynamicallyAllocatedType(ty->getStructElementType(i))) {
            indices[1] = idb->getInt32(i);
            indices[2] = idb->getInt32(1);
            Value * const innerFrame = idb->CreateLoad(idb->CreateGEP(frame, ArrayRef<Value*>(indices.data(), 3)));
            if (containsDynamicallyAllocatedType(innerFrame->getType())) {
                indices[2] = indices[0];
                Value *  const count = idb->CreateLoad(idb->CreateGEP(frame, ArrayRef<Value*>(indices.data(), 3)));
                BasicBlock * const entry = idb->GetInsertBlock();
                BasicBlock * const cond = idb->CreateBasicBlock("freeCarryDataCond");
                BasicBlock * const body = idb->CreateBasicBlock("freeCarryDataLoop");
                BasicBlock * const exit = idb->CreateBasicBlock("freeCarryDataExit");
                idb->CreateBr(cond);
                idb->SetInsertPoint(cond);
                PHINode * const index = idb->CreatePHI(count->getType(), 2);
                index->addIncoming(ConstantInt::getNullValue(count->getType()), entry);
                Value * test = idb->CreateICmpNE(index, count);
                idb->CreateCondBr(test, body, exit);
                idb->SetInsertPoint(body);
                freeDynamicallyAllocatedMemory(idb, idb->CreateGEP(innerFrame, index));
                index->addIncoming(idb->CreateAdd(index, ConstantInt::get(count->getType(), 1)), body);
                idb->CreateBr(cond);
                idb->SetInsertPoint(exit);
            }
            idb->CreateFree(innerFrame);
            indices[2] = idb->getInt32(2);
            Value *  const summary = idb->CreateLoad(idb->CreateGEP(frame, ArrayRef<Value*>(indices.data(), 3)));
            idb->CreateFree(summary);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief releaseCarryData
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::releaseCarryData(const std::unique_ptr<kernel::KernelBuilder> & idb) {
    if (mHasNonCarryCollapsingLoops) {
        freeDynamicallyAllocatedMemory(idb, idb->getScalarFieldPtr("carries"));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeCodeGen
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::initializeCodeGen(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {

    assert(!mCarryMetadata.empty());
    mCarryInfo = &mCarryMetadata[0];
    assert (!mCarryInfo->hasSummary());

    mCurrentFrame = iBuilder->getScalarFieldPtr("carries");
    mCurrentFrameIndex = 0;
    mCarryScopes = 0;
    mCarryScopeIndex.push_back(0);

    assert (mCarryFrameStack.empty());

    assert (mCarrySummaryStack.empty());

    Type * const carryTy = iBuilder->getBitBlockType();

    mCarrySummaryStack.push_back(Constant::getNullValue(carryTy));

    if (mHasLoop) {        
        mLoopSelector = iBuilder->getScalarField("selector");
        mNextLoopSelector = iBuilder->CreateXor(mLoopSelector, ConstantInt::get(mLoopSelector->getType(), 1));
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalizeCodeGen
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::finalizeCodeGen(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    if (mHasLoop) {
        iBuilder->setScalarField("selector", mNextLoopSelector);
    }
    if (mHasLongAdvance) {
        Value * idx = iBuilder->getScalarField("CarryBlockIndex");
        idx = iBuilder->CreateAdd(idx, iBuilder->getSize(1));
        iBuilder->setScalarField("CarryBlockIndex", idx);
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
void CarryManager::enterLoopScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloBlock * const scope) {
    assert (scope);
    assert (mHasLoop);
    ++mLoopDepth;
    enterScope(iBuilder, scope);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterLoopBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterLoopBody(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, BasicBlock * const entryBlock) {
    if (mCarryInfo->hasSummary()) {
        Type * const carryTy = iBuilder->getBitBlockType();
        PHINode * phiCarryOutSummary = iBuilder->CreatePHI(carryTy, 2, "summary");
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

        Type * const int8PtrTy = iBuilder->getInt8PtrTy();
        Type * const carryTy = iBuilder->getBitBlockType();
        PointerType * const carryPtrTy = carryTy->getPointerTo();

        // Check whether we need to resize the carry state
        PHINode * index = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
        mLoopIndicies.push_back(index);
        index->addIncoming(iBuilder->getSize(0), entryBlock);
        Value * capacityPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
        Value * capacity = iBuilder->CreateLoad(capacityPtr, "capacity");
        Constant * const ONE = ConstantInt::get(capacity->getType(), 1);
        Value * arrayPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
        Value * array = iBuilder->CreateLoad(arrayPtr, "array");
        BasicBlock * const entry = iBuilder->GetInsertBlock();
        BasicBlock * const resizeCarryState = iBuilder->CreateBasicBlock("ResizeCarryState");
        BasicBlock * const reallocExisting = iBuilder->CreateBasicBlock("ReallocExisting");
        BasicBlock * const createNew = iBuilder->CreateBasicBlock("CreateNew");
        BasicBlock * const resumeKernel = iBuilder->CreateBasicBlock("ResumeKernel");
        iBuilder->CreateLikelyCondBr(iBuilder->CreateICmpNE(index, capacity), resumeKernel, resizeCarryState);

        // RESIZE CARRY BLOCK
        iBuilder->SetInsertPoint(resizeCarryState);
        const auto BlockWidth = iBuilder->getBitBlockWidth() / 8;
        const auto Log2BlockWidth = floor_log2(BlockWidth);
        Constant * const carryStateWidth = ConstantExpr::getIntegerCast(ConstantExpr::getSizeOf(array->getType()->getPointerElementType()), iBuilder->getSizeTy(), false);
        Value * const summaryPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(2)});
        Value * const hasCarryState = iBuilder->CreateICmpNE(array, ConstantPointerNull::get(cast<PointerType>(array->getType())));
        iBuilder->CreateLikelyCondBr(hasCarryState, reallocExisting, createNew);

        // REALLOCATE EXISTING
        iBuilder->SetInsertPoint(reallocExisting);
        Value * const capacitySize = iBuilder->CreateMul(capacity, carryStateWidth);
        Value * const newCapacitySize = iBuilder->CreateShl(capacitySize, 1); // x 2
        Value * const newArray = iBuilder->CreateCacheAlignedMalloc(newCapacitySize);
        iBuilder->CreateMemCpy(newArray, array, capacitySize, iBuilder->getCacheAlignment());
        iBuilder->CreateFree(array);
        iBuilder->CreateStore(newArray, arrayPtr);
        Value * const startNewArrayPtr = iBuilder->CreateGEP(iBuilder->CreatePointerCast(newArray, int8PtrTy), capacitySize);
        iBuilder->CreateMemZero(startNewArrayPtr, capacitySize, BlockWidth);
        Value * const newCapacity = iBuilder->CreateShl(capacity, 1);
        iBuilder->CreateStore(newCapacity, capacityPtr);
        Value * const summary = iBuilder->CreateLoad(summaryPtr, false);
        Value * const summarySize = iBuilder->CreateShl(iBuilder->CreateAdd(iBuilder->CreateCeilLog2(capacity), ONE), Log2BlockWidth + 1);
        Constant * const additionalSpace = iBuilder->getSize(2 * BlockWidth);
        Value * const newSummarySize = iBuilder->CreateAdd(summarySize, additionalSpace);
        Value * const newSummary = iBuilder->CreateBlockAlignedMalloc(newSummarySize);
        iBuilder->CreateMemCpy(newSummary, summary, summarySize, BlockWidth);
        iBuilder->CreateFree(summary);
        iBuilder->CreateStore(iBuilder->CreatePointerCast(newSummary, carryPtrTy), summaryPtr);
        Value * const startNewSummaryPtr = iBuilder->CreateGEP(iBuilder->CreatePointerCast(newSummary, int8PtrTy), summarySize);
        iBuilder->CreateMemZero(startNewSummaryPtr, additionalSpace, BlockWidth);
        iBuilder->CreateBr(resumeKernel);

        // CREATE NEW
        iBuilder->SetInsertPoint(createNew);
        Constant * const initialLog2Capacity = iBuilder->getInt64(4);
        Constant * const initialCapacity = ConstantExpr::getShl(ONE, initialLog2Capacity);
        iBuilder->CreateStore(initialCapacity, capacityPtr);
        Constant * const initialCapacitySize = ConstantExpr::getMul(initialCapacity, carryStateWidth);
        Value * initialArray = iBuilder->CreateCacheAlignedMalloc(initialCapacitySize);
        iBuilder->CreateMemZero(initialArray, initialCapacitySize, BlockWidth);
        initialArray = iBuilder->CreatePointerCast(initialArray, array->getType());
        iBuilder->CreateStore(initialArray, arrayPtr);
        Constant * initialSummarySize = ConstantExpr::getShl(ConstantExpr::getAdd(initialLog2Capacity, iBuilder->getInt64(1)), iBuilder->getInt64(Log2BlockWidth + 1));
        Value * initialSummary = iBuilder->CreateBlockAlignedMalloc(initialSummarySize);
        iBuilder->CreateMemZero(initialSummary, initialSummarySize, BlockWidth);
        initialSummary = iBuilder->CreatePointerCast(initialSummary, carryPtrTy);
        iBuilder->CreateStore(initialSummary, summaryPtr);
        iBuilder->CreateBr(resumeKernel);

        // RESUME KERNEL
        iBuilder->SetInsertPoint(resumeKernel);
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
void CarryManager::leaveLoopBody(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, BasicBlock * /* exitBlock */) {

    Type * const carryTy = iBuilder->getBitBlockType();

    if (LLVM_UNLIKELY(mCarryInfo->nonCarryCollapsingMode())) {

        assert (mCarryInfo->hasSummary());

        ConstantInt * const summaryIndex = iBuilder->getInt32(mCarryInfo->hasExplicitSummary() ? mCurrentFrameIndex : (mCurrentFrameIndex - 1));

        Value * const carryInAccumulator = readCarryInSummary(iBuilder, summaryIndex);
        Value * const carryOutAccumulator = mCarrySummaryStack.back();

        if (mCarryInfo->hasExplicitSummary()) {
            writeCarryOutSummary(iBuilder, carryOutAccumulator, summaryIndex);
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

        Value * capacityPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
        Value * capacity = iBuilder->CreateLoad(capacityPtr, false);
        Value * summaryPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(2)});
        Value * summary = iBuilder->CreateLoad(summaryPtr, false);

        Constant * const ONE = ConstantInt::get(capacity->getType(), 1);

        Value * loopSelector = iBuilder->CreateZExt(mLoopSelector, capacity->getType());

        BasicBlock * entry = iBuilder->GetInsertBlock();
        BasicBlock * update = iBuilder->CreateBasicBlock("UpdateNonCarryCollapsingSummary");
        BasicBlock * resume = iBuilder->CreateBasicBlock("ResumeAfterUpdatingNonCarryCollapsingSummary");

        iBuilder->CreateBr(update);

        iBuilder->SetInsertPoint(update);
        PHINode * i = iBuilder->CreatePHI(capacity->getType(), 2);
        i->addIncoming(ConstantInt::getNullValue(capacity->getType()), entry);
        PHINode * const borrow = iBuilder->CreatePHI(carryInAccumulator->getType(), 2);
        borrow->addIncoming(carryInAccumulator, entry);
        PHINode * const carry = iBuilder->CreatePHI(carryOutAccumulator->getType(), 2);
        carry->addIncoming(carryOutAccumulator, entry);
        // OR the updated carry in summary later for the summaryTest
        PHINode * const carryInSummary = iBuilder->CreatePHI(carryTy, 2);
        carryInSummary->addIncoming(Constant::getNullValue(carryTy), entry);

        // half subtractor
        Value * const carryInOffset = iBuilder->CreateOr(iBuilder->CreateShl(i, 1), loopSelector);
        Value * const carryInPtr = iBuilder->CreateGEP(summary, carryInOffset);
        Value * const carryIn = iBuilder->CreateBlockAlignedLoad(carryInPtr);
        Value * const nextCarryIn = iBuilder->CreateXor(carryIn, borrow);
        Value * const nextSummary = iBuilder->CreateOr(carryInSummary, nextCarryIn);

        iBuilder->CreateBlockAlignedStore(nextCarryIn, carryInPtr);
        carryInSummary->addIncoming(nextSummary, update);
        Value * finalBorrow = iBuilder->CreateAnd(iBuilder->CreateNot(carryIn), borrow);
        borrow->addIncoming(finalBorrow, update);

        // half adder
        Value * const carryOutOffset = iBuilder->CreateXor(carryInOffset, ConstantInt::get(carryInOffset->getType(), 1));
        Value * const carryOutPtr = iBuilder->CreateGEP(summary, carryOutOffset);
        Value * const carryOut = iBuilder->CreateBlockAlignedLoad(carryOutPtr);
        Value * const nextCarryOut = iBuilder->CreateXor(carryOut, carry);

        iBuilder->CreateBlockAlignedStore(nextCarryOut, carryOutPtr);
        Value * finalCarry = iBuilder->CreateAnd(carryOut, carry);
        carry->addIncoming(finalCarry, update);

        // loop condition
        i->addIncoming(iBuilder->CreateAdd(i, ONE), update);
        iBuilder->CreateCondBr(iBuilder->CreateICmpNE(iBuilder->CreateShl(ONE, i), capacity), update, resume);

        iBuilder->SetInsertPoint(resume);

        if (codegen::EnableAsserts) {
            iBuilder->CreateAssertZero(iBuilder->CreateOr(finalBorrow, finalCarry),
                                       "CarryManager: loop post-condition violated: final borrow and carry must be zero!");
        }

        assert (!mLoopIndicies.empty());
        PHINode * index = mLoopIndicies.back();
        index->addIncoming(iBuilder->CreateAdd(index, iBuilder->getSize(1)), resume);
        mLoopIndicies.pop_back();

        mNextSummaryTest = nextSummary;
    }
    if (mCarryInfo->hasSummary()) {
        const auto n = mCarrySummaryStack.size(); assert (n > 1);
        Value * carryOut = mCarrySummaryStack.back();
        mCarrySummaryStack.pop_back();
        PHINode * phiCarryOut = cast<PHINode>(mCarrySummaryStack.back());
        phiCarryOut->addIncoming(carryOut, iBuilder->GetInsertBlock());
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
void CarryManager::leaveLoopScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, BasicBlock * const /* entryBlock */, BasicBlock * const /* exitBlock */) {
    assert (mLoopDepth > 0);
    --mLoopDepth;
    leaveScope(iBuilder);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterIfScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterIfScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloBlock * const scope) {
    ++mIfDepth;
    enterScope(iBuilder, scope);
    // We zero-initialized the nested summary value and later OR in the current summary into the escaping summary
    // so that upon processing the subsequent block iteration, we branch into this If scope iff a carry out was
    // generated by a statement within this If scope and not by a dominating statement in the outer scope.
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        assert (mCurrentFrameIndex == 0);
        mNextSummaryTest = readCarryInSummary(iBuilder, iBuilder->getInt32(0));
        if (mCarryInfo->hasExplicitSummary()) {
            mCurrentFrameIndex = 1;
        }
    }
    Type * const carryTy = iBuilder->getBitBlockType();
    mCarrySummaryStack.push_back(Constant::getNullValue(carryTy));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateSummaryTest
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::generateSummaryTest(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, Value * condition) {
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
void CarryManager::enterIfBody(const std::unique_ptr<kernel::KernelBuilder> & /* iBuilder */, BasicBlock * const entryBlock) {
    assert (entryBlock);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveIfBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveIfBody(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, BasicBlock * const exitBlock) {
    assert (exitBlock);
    const auto n = mCarrySummaryStack.size();
    if (LLVM_LIKELY(mCarryInfo->hasExplicitSummary())) {
        writeCarryOutSummary(iBuilder, mCarrySummaryStack[n - 1], iBuilder->getInt32(0));
    }
    if (n > 2) {
        mCarrySummaryStack[n - 1] = iBuilder->CreateOr(mCarrySummaryStack[n - 1], mCarrySummaryStack[n - 2], "summary");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveIfScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveIfScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, BasicBlock * const entryBlock, BasicBlock * const exitBlock) {
    assert (mIfDepth > 0);
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        const auto n = mCarrySummaryStack.size(); assert (n > 0);
        if (n > 2) {
            // When leaving a nested If scope with a summary value, phi out the summary to ensure the
            // appropriate summary is stored in the outer scope.
            Value * nested = mCarrySummaryStack[n - 1];
            Value * outer = mCarrySummaryStack[n - 2];
            assert (nested->getType() == outer->getType());
            PHINode * const phi = iBuilder->CreatePHI(nested->getType(), 2, "summary");
            phi->addIncoming(outer, entryBlock);
            phi->addIncoming(nested, exitBlock);
            mCarrySummaryStack[n - 2] = phi;
        }
    }
    --mIfDepth;
    leaveScope(iBuilder);
    mCarrySummaryStack.pop_back();
}

/** ------------------------------------------------------------------------------------------------------------ *
 * @brief enterScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::enterScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloBlock * const scope) {
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
void CarryManager::leaveScope(const std::unique_ptr<kernel::KernelBuilder> & /* iBuilder */) {

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
Value * CarryManager::addCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const Statement * const operation, Value * const e1, Value * const e2) {
    assert (operation && (isNonAdvanceCarryGeneratingStatement(operation)));
    Value * const carryIn = getNextCarryIn(iBuilder);
    Value * carryOut, * result;
    std::tie(carryOut, result) = iBuilder->bitblock_add_with_carry(e1, e2, carryIn);
    setNextCarryOut(iBuilder, carryOut);
    assert (result->getType() == iBuilder->getBitBlockType());
    return result;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief advanceCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::advanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const Advance * const advance, Value * const value) {
    const auto shiftAmount = advance->getAmount();
    if (LLVM_LIKELY(shiftAmount < LONG_ADVANCE_BREAKPOINT)) {
        Value * const carryIn = getNextCarryIn(iBuilder);
        Value * carryOut, * result;
        std::tie(carryOut, result) = iBuilder->bitblock_advance(value, carryIn, shiftAmount);
        setNextCarryOut(iBuilder, carryOut);
        assert (result->getType() == iBuilder->getBitBlockType());
        return result;
    } else {
        return longAdvanceCarryInCarryOut(iBuilder, value, shiftAmount);
    }
}

Value * CarryManager::indexedAdvanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & b, const IndexedAdvance * const advance, Value * const strm, Value * const index_strm) {
    const auto shiftAmount = advance->getAmount();
    if (LLVM_LIKELY(shiftAmount < LONG_ADVANCE_BREAKPOINT)) {
        Value * const carryIn = getNextCarryIn(b);
        unsigned bitWidth = sizeof(size_t) * 8;
        Value * popcount_f = Intrinsic::getDeclaration(b->getModule(), Intrinsic::ctpop, b->getSizeTy());
        Value * PEXT_f = nullptr;
        Value * PDEP_f = nullptr;
        if (bitWidth == 64) {
            PEXT_f = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pext_64);
            PDEP_f = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pdep_64);
        }
        else if ((bitWidth == 32)  && (shiftAmount < 32)) {
            PEXT_f = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pext_32);
            PDEP_f = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pdep_32);
        }
        else {
            llvm::report_fatal_error("indexed_advance unsupported bit width");
        }
        Value * carry = b->mvmd_extract(bitWidth, carryIn, 0);
        Value * result = b->allZeroes();
        for (unsigned i = 0; i < b->getBitBlockWidth()/bitWidth; i++) {
            Value * s = b->mvmd_extract(bitWidth, strm, i);
            Value * ix = b->mvmd_extract(bitWidth, index_strm, i);
            Value * ix_popcnt = b->CreateCall(popcount_f, {ix});
            Value * bits = b->CreateCall(PEXT_f, {s, ix});
            Value * adv = b->CreateOr(b->CreateShl(bits, shiftAmount), carry);
            Value * overflow = b->CreateLShr(bits, bitWidth - shiftAmount);
            result = b->mvmd_insert(bitWidth, result, b->CreateCall(PDEP_f, {adv, ix}), i);
            carry = b->CreateOr(b->CreateLShr(adv, ix_popcnt), b->CreateShl(overflow, b->CreateSub(b->getSize(bitWidth), ix_popcnt)));
        }
        setNextCarryOut(b, carry);
        return result;
    } else {
        llvm::report_fatal_error("IndexedAdvance > LONG_ADVANCE_BREAKPOINT not yet supported.");
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief longAdvanceCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * CarryManager::longAdvanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, Value * const value, const unsigned shiftAmount) {

    assert (mHasLongAdvance);
    assert (shiftAmount >= LONG_ADVANCE_BREAKPOINT);
    assert (value);

    const auto blockWidth = iBuilder->getBitBlockWidth();
    Type * const streamTy = iBuilder->getIntNTy(blockWidth);

    Value * indices[3];

    indices[0] = iBuilder->getInt32(0);

    if (mIfDepth > 0) {
        if (shiftAmount > blockWidth) {

            // TODO: once CEILING(shiftAmount / 256) > 2, consider using a half-adder/subtractor strategy?

            Value * carry = iBuilder->CreateZExt(iBuilder->bitblock_any(value), streamTy);
            const auto summaryBlocks = ceil_udiv(shiftAmount, blockWidth);
            const auto summarySize = ceil_udiv(summaryBlocks, blockWidth);
            VectorType * const bitBlockTy = iBuilder->getBitBlockType();
            IntegerType * const laneTy = cast<IntegerType>(bitBlockTy->getVectorElementType());
            const auto laneWidth = laneTy->getIntegerBitWidth();

            assert (summarySize > 0);
            assert (is_power_2(laneWidth));

            indices[1] = iBuilder->getInt32(mCurrentFrameIndex++);

            for (unsigned i = 1;;++i) {

                assert (i <= summarySize);

                indices[2] = iBuilder->getInt32(i - 1);

                Value * const ptr = iBuilder->CreateGEP(mCurrentFrame, indices);
                Value * const prior = iBuilder->CreateBitCast(iBuilder->CreateBlockAlignedLoad(ptr), streamTy);

                Value * advanced = nullptr;
                if (LLVM_LIKELY(summaryBlocks < laneWidth)) {
                    advanced = iBuilder->CreateOr(iBuilder->CreateShl(prior, 1), carry);
                    carry = iBuilder->CreateLShr(prior, summaryBlocks - 1);
                } else {
                    std::tie(advanced, carry) = iBuilder->bitblock_advance(prior, carry, 1);
                }
                Value * stream = iBuilder->CreateBitCast(advanced, bitBlockTy);
                if (LLVM_LIKELY(i == summarySize)) {
                    const auto n = bitBlockTy->getVectorNumElements();
                    Constant * mask[n];                                        
                    const auto m = udiv(summaryBlocks, laneWidth);
                    if (m) {
                        std::fill_n(mask, m, ConstantInt::getAllOnesValue(laneTy));
                    }
                    mask[m] = ConstantInt::get(laneTy, (1UL << (summaryBlocks & (laneWidth - 1))) - 1UL);
                    if (n > m) {
                        std::fill_n(mask + m + 1, n - m, UndefValue::get(laneTy));
                    }
                    stream = iBuilder->CreateAnd(stream, ConstantVector::get(ArrayRef<Constant *>(mask, n)));
                    addToCarryOutSummary(iBuilder, stream);
                    iBuilder->CreateBlockAlignedStore(stream, ptr);
                    break;
                }
                addToCarryOutSummary(iBuilder, stream);
                iBuilder->CreateBlockAlignedStore(stream, ptr);
            }

        } else if (LLVM_LIKELY(mCarryInfo->hasExplicitSummary())) {
            addToCarryOutSummary(iBuilder, value);
        }
    }

    indices[1] = iBuilder->getInt32(mCurrentFrameIndex++);

    // special case using a single buffer entry and the carry_out value.
    if (LLVM_LIKELY((shiftAmount < blockWidth) && (mLoopDepth == 0))) {

        indices[2] = indices[0]; // iBuilder->getInt32(0)
        assert (cast<ConstantInt>(indices[2])->isNullValue());

        Value * const buffer = iBuilder->CreateGEP(mCurrentFrame, indices);
        assert (buffer->getType()->getPointerElementType() == iBuilder->getBitBlockType());
        Value * carryIn = iBuilder->CreateBlockAlignedLoad(buffer);

        iBuilder->CreateBlockAlignedStore(value, buffer);
        /* Very special case - no combine */
        if (LLVM_UNLIKELY(shiftAmount == blockWidth)) {
            return iBuilder->CreateBitCast(carryIn, iBuilder->getBitBlockType());
        }
        Value* block0_shr = iBuilder->CreateLShr(iBuilder->CreateBitCast(carryIn, streamTy), blockWidth - shiftAmount);
        Value* block1_shl = iBuilder->CreateShl(iBuilder->CreateBitCast(value, streamTy), shiftAmount);
        return iBuilder->CreateBitCast(iBuilder->CreateOr(block1_shl, block0_shr), iBuilder->getBitBlockType());
    } else { //
        const unsigned blockShift = shiftAmount & (blockWidth - 1);
        const unsigned summaryBlocks = ceil_udiv(shiftAmount, blockWidth);

        // Create a mask to implement circular buffer indexing
        Value * indexMask = iBuilder->getSize(nearest_pow2(summaryBlocks) - 1);
        Value * blockIndex = iBuilder->getScalarField("CarryBlockIndex");

        Value * carryIndex0 = iBuilder->CreateSub(blockIndex, iBuilder->getSize(summaryBlocks));
        indices[2] = iBuilder->CreateAnd(carryIndex0, indexMask);
        Value * const carryInPtr = iBuilder->CreateGEP(mCurrentFrame, indices);
        Value * carryIn = iBuilder->CreateBlockAlignedLoad(carryInPtr);

        indices[2] = iBuilder->CreateAnd(blockIndex, indexMask);
        Value * const carryOutPtr = iBuilder->CreateGEP(mCurrentFrame, indices);
        assert (carryIn->getType() == iBuilder->getBitBlockType());

        // If the long advance is an exact multiple of BitBlockWidth, we simply return the oldest
        // block in the long advance carry data area.
        if (LLVM_UNLIKELY(blockShift == 0)) {
            iBuilder->CreateBlockAlignedStore(value, carryOutPtr);
            return carryIn;
        } else { // Otherwise we need to combine data from the two oldest blocks.
            Value * const carryIndex1 = iBuilder->CreateSub(blockIndex, iBuilder->getSize(summaryBlocks - 1));
            indices[2] = iBuilder->CreateAnd(carryIndex1, indexMask);

            Value * const carryInPtr2 = iBuilder->CreateGEP(mCurrentFrame, indices);
            Value * const carryIn2 = iBuilder->CreateBlockAlignedLoad(carryInPtr2);
            assert (carryOutPtr->getType()->getPointerElementType() == value->getType());
            iBuilder->CreateBlockAlignedStore(value, carryOutPtr);

            Value * const b0 = iBuilder->CreateLShr(iBuilder->CreateBitCast(carryIn, streamTy), blockWidth - blockShift);
            Value * const b1 = iBuilder->CreateShl(iBuilder->CreateBitCast(carryIn2, streamTy), blockShift);
            return iBuilder->CreateBitCast(iBuilder->CreateOr(b1, b0), iBuilder->getBitBlockType());
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNextCarryIn
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::getNextCarryIn(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    assert (mCurrentFrameIndex < mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    if (mLoopDepth == 0) {
        mCarryPackPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(mCurrentFrameIndex)});
    } else {
        mCarryPackPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(mCurrentFrameIndex), mLoopSelector});
    }
    Type * const carryTy = iBuilder->getBitBlockType();
    assert (mCarryPackPtr->getType()->getPointerElementType() == carryTy);
    Value * const carryIn = iBuilder->CreateBlockAlignedLoad(mCarryPackPtr);
    if (mLoopDepth > 0) {
        iBuilder->CreateBlockAlignedStore(Constant::getNullValue(carryTy), mCarryPackPtr);
    }
    return carryIn;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setNextCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::setNextCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, Value * carryOut) {
    Type * const carryTy = iBuilder->getBitBlockType();
    assert (mCurrentFrameIndex < mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    carryOut = iBuilder->CreateBitCast(carryOut, carryTy);
    if (mCarryInfo->hasSummary()) {
        addToCarryOutSummary(iBuilder, carryOut);
    }
    if (mLoopDepth != 0) {
        mCarryPackPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(mCurrentFrameIndex), mNextLoopSelector});
        if (LLVM_LIKELY(!mCarryInfo->nonCarryCollapsingMode())) {
            Value * accum = iBuilder->CreateBlockAlignedLoad(mCarryPackPtr);
            carryOut = iBuilder->CreateOr(carryOut, accum);
        }
    }
    ++mCurrentFrameIndex;
    assert (mCarryPackPtr->getType()->getPointerElementType() == carryTy);
    iBuilder->CreateBlockAlignedStore(carryOut, mCarryPackPtr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readCarryInSummary
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::readCarryInSummary(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, ConstantInt * index) const {
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
    const unsigned length = (mLoopDepth == 0) ? count : (count + 1);
    Value * indicies[length];
    std::fill(indicies, indicies + count - 1, iBuilder->getInt32(0));
    indicies[count - 1] = index;
    if (mLoopDepth != 0) {
        indicies[count] = mLoopSelector;
    }

    ArrayRef<Value *> ar(indicies, length);
    Value * const ptr = iBuilder->CreateGEP(mCurrentFrame, ar);
    Value * const summary = iBuilder->CreateBlockAlignedLoad(ptr);
    if (mLoopDepth != 0 && mCarryInfo->hasExplicitSummary()) {
        Type * const carryTy = iBuilder->getBitBlockType();
        iBuilder->CreateBlockAlignedStore(Constant::getNullValue(carryTy), ptr);
    }
    return summary;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeCarryOutSummary
 ** ------------------------------------------------------------------------------------------------------------- */
inline void CarryManager::writeCarryOutSummary(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, Value * const summary, ConstantInt * index) const {
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
inline void CarryManager::addToCarryOutSummary(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, Value * const value) {
    assert ("cannot add null summary value!" && value);    
    assert ("summary stack is empty!" && !mCarrySummaryStack.empty());
    assert (mCarryInfo->hasSummary());
    mCarrySummaryStack.back() = iBuilder->CreateOr(value, mCarrySummaryStack.back());
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

static bool hasNonEmptyCarryStruct(const Type * const frameTy) {
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

static bool hasNonEmptyCarryStruct(const std::vector<Type *> & state) {
    for (const Type * const frameTy : state) {
        if (hasNonEmptyCarryStruct(frameTy)) {
            return true;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief analyse
 ** ------------------------------------------------------------------------------------------------------------- */
StructType * CarryManager::analyse(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloBlock * const scope,
                                   const unsigned ifDepth, const unsigned loopDepth, const bool isNestedWithinNonCarryCollapsingLoop) {
    assert ("scope cannot be null!" && scope);
    assert ("entry scope (and only the entry scope) must be in scope 0"
            && (mCarryScopes == 0 ? (scope == mKernel->getEntryBlock()) : (scope != mKernel->getEntryBlock())));
    assert (mCarryScopes < mCarryMetadata.size());
    Type * const carryTy = iBuilder->getBitBlockType();
    Type * const blockTy = iBuilder->getBitBlockType();

    const unsigned carryScopeIndex = mCarryScopes++;
    const bool nonCarryCollapsingMode = isNonRegularLanguage(scope);
    Type * const carryPackType = (loopDepth == 0) ? carryTy : ArrayType::get(carryTy, 2);
    std::vector<Type *> state;
    for (const Statement * stmt : *scope) {
        if (LLVM_UNLIKELY(isa<Advance>(stmt) || isa<IndexedAdvance>(stmt))) {
            int64_t amount;
            if (isa<Advance>(stmt)) amount = cast<Advance>(stmt)->getAmount();
            else amount = cast<IndexedAdvance>(stmt)->getAmount();
            Type * type = carryPackType;
            if (LLVM_UNLIKELY(amount >= LONG_ADVANCE_BREAKPOINT)) {
                const auto blockWidth = iBuilder->getBitBlockWidth();
                const auto blocks = ceil_udiv(amount, blockWidth);
                type = ArrayType::get(blockTy, nearest_pow2(blocks + ((loopDepth != 0) ? 1 : 0)));
                if (LLVM_UNLIKELY(ifDepth > 0 && blocks != 1)) {
                    const auto summarySize = ceil_udiv(blocks, blockWidth);
                    // 1 bit will mark the presense of any bit in each block.
                    state.push_back(ArrayType::get(blockTy, summarySize));
                }
                mHasLongAdvance = true;
                if (isa<IndexedAdvance>(stmt)) mIndexedLongAdvanceTotal++;
            }
            state.push_back(type);
        } else if (LLVM_UNLIKELY(isNonAdvanceCarryGeneratingStatement(stmt))) {
            state.push_back(carryPackType);
        } else if (LLVM_UNLIKELY(isa<If>(stmt))) {
            state.push_back(analyse(iBuilder, cast<If>(stmt)->getBody(), ifDepth + 1, loopDepth, nonCarryCollapsingMode | isNestedWithinNonCarryCollapsingLoop));
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            mHasLoop = true;
            state.push_back(analyse(iBuilder, cast<While>(stmt)->getBody(), ifDepth, loopDepth + 1, nonCarryCollapsingMode | isNestedWithinNonCarryCollapsingLoop));
        }
    }
    // Build the carry state struct and add the summary pack if needed.
    CarryData & cd = mCarryMetadata[carryScopeIndex];
    StructType * carryState = nullptr;
    CarryData::SummaryType summaryType = CarryData::NoSummary;
    if (LLVM_UNLIKELY(state.empty())) {
        carryState = StructType::get(iBuilder->getContext());
    } else {
        // do we have a summary or a sequence of nested empty structs?
        if (hasNonEmptyCarryStruct(state)) {
            if (dyn_cast_or_null<If>(scope->getBranch()) || nonCarryCollapsingMode || isNestedWithinNonCarryCollapsingLoop) {
                if (LLVM_LIKELY(state.size() > 1)) {
                    summaryType = CarryData::ExplicitSummary;
                    // NOTE: summaries are stored differently depending whether we're entering an If or While branch. With an If branch, they
                    // preceed the carry state data and with a While loop they succeed it. This is to help cache prefectching performance.
                    state.insert(isa<If>(scope->getBranch()) ? state.begin() : state.end(), carryPackType);
                } else {
                    summaryType = CarryData::ImplicitSummary;
                    if (hasNonEmptyCarryStruct(state[0])) {
                        summaryType = CarryData::BorrowedSummary;
                    }
                }
            }
        }
        carryState = StructType::get(iBuilder->getContext(), state);
        // If we're in a loop and cannot use collapsing carry mode, convert the carry state struct into a capacity,
        // carry state pointer, and summary pointer struct.
        if (LLVM_UNLIKELY(nonCarryCollapsingMode)) {
            mHasNonCarryCollapsingLoops = true;
            carryState = StructType::get(iBuilder->getSizeTy(), carryState->getPointerTo(), carryTy->getPointerTo(), nullptr);
            assert (isDynamicallyAllocatedType(carryState));
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
, mIndexedLongAdvanceTotal(0)
, mIndexedLongAdvanceIndex(0)
, mHasNonCarryCollapsingLoops(false)
, mHasLoop(false)
, mLoopDepth(0)
, mLoopSelector(nullptr)
, mNextLoopSelector(nullptr)
, mCarryPackPtr(nullptr)
, mCarryScopes(0) {

}

}
