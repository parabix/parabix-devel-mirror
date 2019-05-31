/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "carrypack_manager.h"
#include <pablo/carry_data.h>
#include <pablo/codegenstate.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/DerivedTypes.h>
#include <pablo/branch.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_var.h>
#include <kernels/kernel_builder.h>
#include <toolchain/toolchain.h>
#include <array>

#include <llvm/Support/CommandLine.h>

#include <llvm/Support/raw_ostream.h>

using namespace llvm;

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

inline static unsigned nearest_multiple(const unsigned n, const unsigned m) {
    assert (is_power_2(m));
    return (n + m - 1U) & ~(m - 1U);
}

inline static unsigned udiv(const unsigned x, const unsigned y) {
    assert (is_power_2(y));
    const unsigned z = x >> floor_log2(y);
    assert (z == (x / y));
    return z;
}

inline static unsigned ceil_udiv(const unsigned x, const unsigned y) {
    assert (is_power_2(x) && is_power_2(y));
    return udiv(((x - 1U) | (y - 1U)) + 1U, y);
}

inline static unsigned getElementWidth(Type * ty) {
    if (LLVM_LIKELY(isa<VectorType>(ty))) {
        ty = cast<IntegerType>(ty->getVectorElementType());
    }
    return cast<IntegerType>(ty)->getBitWidth();
}

inline static unsigned gcd(unsigned a, unsigned b) {
  while (a) {
     assert (is_power_2(a));
     const unsigned c = a;
     a = b & (a - 1);
     b = c;
  }
  return b;
}

inline static unsigned getPackingSize(const unsigned elementWidth, const unsigned depth) {
    return std::max<unsigned>(elementWidth / nearest_pow2(depth + 1), 1);
}

using TypeId = PabloAST::ClassTypeId;

#define ELEMENTS_IN_DYNAMICALLY_ALLOCATED_CARRY_STRUCT 3

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

    mCurrentScope = kernel->getEntryScope();
    mKernel = kernel;

    Type * const carryTy = iBuilder->getBitBlockType();
    mVectorWidth = carryTy->getPrimitiveSizeInBits();
    mElementWidth = getElementWidth(carryTy);
    assert (is_power_2(mElementWidth));

    mCarryScopes = 0;
    mCarryMetadata.resize(getScopeCount(mCurrentScope));
    mCarryGroup.resize(assignDefaultCarryGroups(kernel->getEntryScope()));

    kernel->setCarryDataTy(analyse(iBuilder, mCurrentScope));

    kernel->addInternalScalar(kernel->getCarryDataTy(), "carries");

    if (mHasLoop) {
        kernel->addInternalScalar(iBuilder->getInt32Ty(), "selector");
    }
    if (mHasLongAdvance) {
        kernel->addInternalScalar(iBuilder->getSizeTy(), "CarryBlockIndex");
    }
}

bool isDynamicallyAllocatedType(const Type * const ty) {
    if (isa<StructType>(ty) && ty->getStructNumElements() == ELEMENTS_IN_DYNAMICALLY_ALLOCATED_CARRY_STRUCT) {
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

    assert (mCurrentFrameOffset.empty());

    mCurrentFrameOffset.push_back(iBuilder->getInt32(0));

    assert (mNonCarryCollapsingLoopCarryFrameStack.empty());

    assert (mCarrySummaryStack.empty());

    Type * const carryTy = iBuilder->getBitBlockType();

    mCarrySummaryStack.push_back(Constant::getNullValue(carryTy));

    if (mHasLoop) {
        mLoopSelector[0] = iBuilder->getScalarField("selector");
        mLoopSelector[1] = iBuilder->CreateXor(mLoopSelector[0], ConstantInt::get(mLoopSelector[0]->getType(), 1));
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalizeCodeGen
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::finalizeCodeGen(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    if (mHasLoop) {
        iBuilder->setScalarField("selector", mLoopSelector[1]);
    }
    if (mHasLongAdvance) {
        Value * idx = iBuilder->getScalarField("CarryBlockIndex");
        idx = iBuilder->CreateAdd(idx, iBuilder->getSize(1));
        iBuilder->setScalarField("CarryBlockIndex", idx);
    }
    assert (mNonCarryCollapsingLoopCarryFrameStack.empty());
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
        mNonCarryCollapsingLoopCarryFrameStack.emplace_back(mCurrentFrame, std::vector<Value *>{mCurrentFrameOffset.begin(), mCurrentFrameOffset.end()});
        mCurrentFrame = iBuilder->CreateGEP(phiArrayPtr, index);
        assert (&std::get<1>(mNonCarryCollapsingLoopCarryFrameStack.back()) != &mCurrentFrameOffset);
        mCurrentFrameOffset.resize(1);
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
        std::tie(mCurrentFrame, mCurrentFrameOffset) = mNonCarryCollapsingLoopCarryFrameStack.back();
        mNonCarryCollapsingLoopCarryFrameStack.pop_back();

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

        Value * loopSelector = iBuilder->CreateZExt(mLoopSelector[0], capacity->getType());

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

        iBuilder->CreateAssertZero(iBuilder->CreateOr(finalBorrow, finalCarry),
                                   "CarryPackManager: loop post-condition violated: final borrow and carry must be zero!");

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
    mCurrentFrameOffset.push_back(iBuilder->getInt32(mCurrentFrameIndex));
    mCurrentScope = scope;
    mCarryScopeIndex.push_back(++mCarryScopes);
    mCarryInfo = &mCarryMetadata[mCarryScopes];
    mCurrentFrameIndex = 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveScope(const std::unique_ptr<kernel::KernelBuilder> & /* iBuilder */) {

    assert (mCurrentFrameOffset.size() > 1);
    ConstantInt * const carryIndex = cast<ConstantInt>(mCurrentFrameOffset.back());
    mCurrentFrameIndex = carryIndex->getLimitedValue() + 1;
    mCurrentFrameOffset.pop_back();

    mCarryScopeIndex.pop_back();
    assert (!mCarryScopeIndex.empty());
    mCurrentScope = mCurrentScope->getPredecessor();
    assert (mCurrentScope);
    mCarryInfo = &mCarryMetadata[mCarryScopeIndex.back()];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::addCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const CarryProducingStatement * const op, Value * const e1, Value * const e2) {
    assert (op && !isa<Advance>(op));
    Value * const carryIn = getCarryIn(iBuilder, op);
    Value * carryOut, * result;
    std::tie(carryOut, result) = iBuilder->bitblock_add_with_carry(e1, e2, carryIn);
    setCarryOut(iBuilder, op, carryOut);
    assert (result->getType() == iBuilder->getBitBlockType());
    return result;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief advanceCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::advanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const Advance * const advance, Value * const value) {
    const auto shiftAmount = advance->getAmount();
    if (LLVM_LIKELY(shiftAmount < mElementWidth)) {
        Value * const carryIn = getCarryIn(iBuilder, advance);
        Value * carryOut, * result;
        std::tie(carryOut, result) = iBuilder->bitblock_advance(value, carryIn, shiftAmount);
        setCarryOut(iBuilder, advance, carryOut);
        assert (result->getType() == iBuilder->getBitBlockType());
        return result;
    } else {
        return longAdvanceCarryInCarryOut(iBuilder, value, shiftAmount);
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief indexedAdvanceCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::indexedAdvanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & b, const IndexedAdvance * const advance, Value * const strm, Value * const index_strm) {
    const auto shiftAmount = advance->getAmount();
    if (LLVM_LIKELY(shiftAmount < mElementWidth)) {
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
        if ((mIfDepth > 0) && mCarryInfo->hasExplicitSummary()) {
            addToCarryOutSummary(b, strm);
        }
        return result;
    } else {
        mIndexedLongAdvanceIndex++;
        llvm::report_fatal_error("IndexedAdvance > BlockSize not yet supported.");
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief longAdvanceCarryInCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * CarryManager::longAdvanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, Value * const value, const unsigned shiftAmount) {

    assert (mHasLongAdvance);
    assert (shiftAmount >= mElementWidth);

    Type * const streamTy = iBuilder->getIntNTy(iBuilder->getBitBlockWidth());

    if (mIfDepth > 0) {
        if (shiftAmount > iBuilder->getBitBlockWidth()) {

            mCurrentFrameOffset.push_back(iBuilder->getInt32(mCurrentFrameIndex++));
            mCurrentFrameOffset.push_back(nullptr);

            Value * carry = iBuilder->CreateZExt(iBuilder->bitblock_any(value), streamTy);
            const unsigned summarySize = ceil_udiv(shiftAmount, iBuilder->getBitBlockWidth() * iBuilder->getBitBlockWidth());
            for (unsigned i = 0;;++i) {

                mCurrentFrameOffset.back() = iBuilder->getInt32(i);

                Value * const ptr = iBuilder->CreateGEP(mCurrentFrame, mCurrentFrameOffset);
                Value * const prior = iBuilder->CreateBitCast(iBuilder->CreateBlockAlignedLoad(ptr), streamTy);
                Value * const stream = iBuilder->CreateBitCast(iBuilder->CreateOr(iBuilder->CreateShl(prior, 1), carry), iBuilder->getBitBlockType());
                if (LLVM_LIKELY(i == summarySize)) {
                    Value * const summeryOffset = iBuilder->getInt32(summarySize % iBuilder->getBitBlockWidth());
                    Value * const maskedStream = iBuilder->CreateAnd(stream, iBuilder->bitblock_mask_from(summeryOffset, true));
                    addToCarryOutSummary(iBuilder, maskedStream);
                    iBuilder->CreateBlockAlignedStore(maskedStream, ptr);
                    break;
                }
                addToCarryOutSummary(iBuilder, stream);
                iBuilder->CreateBlockAlignedStore(stream, ptr);
                carry = iBuilder->CreateLShr(prior, iBuilder->getBitBlockWidth() - 1);
            }

            mCurrentFrameOffset.pop_back();
            mCurrentFrameOffset.pop_back();

        } else if (LLVM_LIKELY(mCarryInfo->hasExplicitSummary())) {
            addToCarryOutSummary(iBuilder, value);
        }
    }
    mCurrentFrameOffset.push_back(iBuilder->getInt32(mCurrentFrameIndex++));
    Value * result = nullptr;
    // special case using a single buffer entry and the carry_out value.
    if (LLVM_LIKELY((shiftAmount < iBuilder->getBitBlockWidth()) && (mLoopDepth == 0))) {
        mCurrentFrameOffset.push_back(iBuilder->getInt32(0));
        Value * const buffer = iBuilder->CreateGEP(mCurrentFrame, mCurrentFrameOffset);
        assert (buffer->getType()->getPointerElementType() == iBuilder->getBitBlockType());
        Value * carryIn = iBuilder->CreateBlockAlignedLoad(buffer);
        iBuilder->CreateBlockAlignedStore(value, buffer);
        /* Very special case - no combine */
        if (LLVM_UNLIKELY(shiftAmount == iBuilder->getBitBlockWidth())) {
            return iBuilder->CreateBitCast(carryIn, iBuilder->getBitBlockType());
        }
        Value * block0_shr = iBuilder->CreateLShr(iBuilder->CreateBitCast(carryIn, streamTy), iBuilder->getBitBlockWidth() - shiftAmount);
        Value * block1_shl = iBuilder->CreateShl(iBuilder->CreateBitCast(value, streamTy), shiftAmount);
        result = iBuilder->CreateBitCast(iBuilder->CreateOr(block1_shl, block0_shr), iBuilder->getBitBlockType());
    } else { //
        const unsigned blockShift = shiftAmount % iBuilder->getBitBlockWidth();
        const unsigned blocks = ceil_udiv(shiftAmount, iBuilder->getBitBlockWidth());
        // Create a mask to implement circular buffer indexing
        Value * const indexMask = iBuilder->getSize(nearest_pow2(blocks + ((mLoopDepth != 0) ? 1 : 0)) - 1);
        Value * const blockIndex = iBuilder->getScalarField("CarryBlockIndex");
        Value * const carryIndex0 = iBuilder->CreateSub(blockIndex, iBuilder->getSize(blocks));
        Value * const loadIndex0 = iBuilder->CreateAnd(carryIndex0, indexMask);
        mCurrentFrameOffset.push_back(loadIndex0);

        Value * const carryInPtr = iBuilder->CreateGEP(mCurrentFrame, mCurrentFrameOffset);
        Value * const carryIn = iBuilder->CreateBlockAlignedLoad(carryInPtr);
        Value * const storeIndex = iBuilder->CreateAnd(blockIndex, indexMask);
        mCurrentFrameOffset.back() = storeIndex;

        Value * const carryOutPtr = iBuilder->CreateGEP(mCurrentFrame, mCurrentFrameOffset);
        assert (carryIn->getType() == iBuilder->getBitBlockType());
        // If the long advance is an exact multiple of BitBlockWidth, we simply return the oldest
        // block in the long advance carry data area.
        if (LLVM_UNLIKELY(blockShift == 0)) {
            iBuilder->CreateBlockAlignedStore(value, carryOutPtr);
            result = carryIn;
        } else { // Otherwise we need to combine data from the two oldest blocks.
            Value * carryIndex1 = iBuilder->CreateSub(blockIndex, iBuilder->getSize(blocks - 1));
            Value * loadIndex1 = iBuilder->CreateAnd(carryIndex1, indexMask);
            mCurrentFrameOffset.back() = loadIndex1;
            Value * const carryInPtr2 = iBuilder->CreateGEP(mCurrentFrame, mCurrentFrameOffset);
            Value * carry_block1 = iBuilder->CreateBlockAlignedLoad(carryInPtr2);
            Value * block0_shr = iBuilder->CreateLShr(iBuilder->CreateBitCast(carryIn, streamTy), iBuilder->getBitBlockWidth() - blockShift);
            Value * block1_shl = iBuilder->CreateShl(iBuilder->CreateBitCast(carry_block1, streamTy), blockShift);
            iBuilder->CreateBlockAlignedStore(value, carryOutPtr);
            result = iBuilder->CreateBitCast(iBuilder->CreateOr(block1_shl, block0_shr), iBuilder->getBitBlockType());
        }
    }
    mCurrentFrameOffset.pop_back();
    mCurrentFrameOffset.pop_back();
    return result;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNextCarryIn
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::getCarryIn(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const CarryProducingStatement * const op) {

    VectorType * const carryTy = iBuilder->getBitBlockType();

    const auto carryGroup = op->getCarryGroup();
    assert (carryGroup < mCarryGroup.size());
    CarryGroup & group = mCarryGroup[carryGroup];

    Value * carryIn = group.carryIn;
    if (carryIn) {
        // if this carry crosses a non-carry-collapsing loop frame, compute the address for this carry in the 'new' frame.
        Value * const frame = cast<GetElementPtrInst>(cast<LoadInst>(carryIn)->getPointerOperand())->getPointerOperand();
        carryIn = (frame == mCurrentFrame) ? carryIn : nullptr;
    }

    if (carryIn == nullptr) {
        mCurrentFrameOffset.push_back(iBuilder->getInt32(mCurrentFrameIndex));
        if (LLVM_UNLIKELY(mLoopDepth != 0)) {
            assert ("Loop selector was not initialized!" && mLoopSelector[0]);
            mCurrentFrameOffset.push_back(mLoopSelector[0]);
        }
        Value * ptr = iBuilder->CreateGEP(mCurrentFrame, mCurrentFrameOffset);
        carryIn = iBuilder->CreateBlockAlignedLoad(ptr);
        carryIn = iBuilder->CreateBitCast(carryIn, carryTy);

        group.carryIn = carryIn;
        group.carryOut = nullptr;
        group.frameIndex = mCurrentFrameIndex++;
        group.packedSize = 0;

        // If we're within a loop, we need to clear out the carry in value for the subsequent loop iteration
        if (LLVM_UNLIKELY(mLoopDepth != 0)) {
            iBuilder->CreateBlockAlignedStore(Constant::getNullValue(carryTy), ptr);
            mCurrentFrameOffset.pop_back();
        }
        mCurrentFrameOffset.pop_back();
        if (LLVM_UNLIKELY(group.groupSize == 1)) {
            return carryIn;
        }
    }

    // Initially, our carry in is densely packed. Our goal is to move the carry to the 0-th position and leave
    // all other values zeroed out. We can do so easily using ShuffleVector by moving the appropriate element
    // and selecting 0s for all other elements but this produces poor code by LLVM.

    // Instead we divide this process into two stages:

    // First use the blend instruction to select only the elements of the carry we want



    const auto width = std::max<unsigned>(gcd(std::min<unsigned>(op->getCarryWidth(), mElementWidth), group.packedSize), 8);
    const auto n = udiv(iBuilder->getBitBlockWidth(), width);

    VectorType * packTy = VectorType::get(iBuilder->getIntNTy(width), n);

    Constant * const zero = Constant::getNullValue(packTy);

    carryIn = iBuilder->CreateBitCast(carryIn, packTy);

    if (LLVM_UNLIKELY(group.groupSize > 1)) {
        int blend_mask[n];
        for (unsigned i = 0; i < n; ++i) {
            const auto w = i * width;
            if (w >= group.packedSize && w < (group.packedSize + op->getCarryWidth())) {
                blend_mask[i] = i;
            } else {
                blend_mask[i] = i + n;
            }
        }
        carryIn = iBuilder->CreateShuffleVector(carryIn, zero, ArrayRef<int>(blend_mask, n));
    }

    // Then use a byte shift to move them to the 0-th position

    const auto offset = udiv(group.packedSize, width);

    if (offset) {
        int shift[n];
        for (unsigned i = 0; i < n; ++i) {
            shift[i] = offset + i;
        }
        carryIn = iBuilder->CreateShuffleVector(carryIn, zero, ArrayRef<int>(shift, n));
    }

    // if this element contains more than one carry, we need to mask mask off any subsequent carries
    const unsigned nextPackedSize = group.packedSize + op->getCarryWidth();
    const unsigned subsequentOffset = (-nextPackedSize) & (width - 1U);
    if (LLVM_UNLIKELY(subsequentOffset != 0)) {
        carryIn = iBuilder->CreateShl(carryIn, subsequentOffset);
    }

    // then shift out the prior carries while moving the current carry to the correct position
    const unsigned trailingOffset = subsequentOffset + (group.packedSize & (width - 1U));
    assert (trailingOffset < width);
    if (LLVM_UNLIKELY(trailingOffset != 0)) {
        carryIn = iBuilder->CreateLShr(carryIn, trailingOffset);
    }

    carryIn = iBuilder->CreateBitCast(carryIn, carryTy, "CarryIn_" + op->getName());

    return carryIn;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setNextCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::setCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const CarryProducingStatement * const op, Value * carryOut) {

    VectorType * const carryTy = iBuilder->getBitBlockType();

    assert ("Unexpected carry out value type!" && carryOut->getType()->canLosslesslyBitCastTo(carryTy));

    const auto carryGroup = op->getCarryGroup();
    assert (carryGroup < mCarryGroup.size());
    CarryGroup & group = mCarryGroup[carryGroup];

    const auto width = std::max<unsigned>(gcd(op->getCarryWidth(), group.packedSize), 8);
    const auto count = udiv(iBuilder->getBitBlockWidth(), width);
    VectorType * packTy = VectorType::get(iBuilder->getIntNTy(width), count);

    carryOut = iBuilder->CreateBitCast(carryOut, packTy);

    // shuffle the carryOut to have the 0-th slot being in the correct packed offset
    const auto index = udiv(group.packedSize, width);

    if (index) {
        assert (index < count);
        int position[count];
        for (unsigned i = 0; i < count; ++i) {
            position[i] = ((count - index) + i);
        }
        Constant * const zero = Constant::getNullValue(packTy);
        carryOut = iBuilder->CreateShuffleVector(zero, carryOut, ArrayRef<int>(position, count));
    }

    const auto offset = group.packedSize & (width - 1U);
    if (LLVM_UNLIKELY(offset != 0)) {
        carryOut = iBuilder->CreateShl(carryOut, offset);
    }
    carryOut = iBuilder->CreateBitCast(carryOut, carryTy);

    assert (op->getCarryWidth() > 0);

    if (group.carryOut == nullptr) {
        group.carryOut = carryOut;
    } else {
        #ifndef NDEBUG
        iBuilder->CreateAssertZero(iBuilder->CreateAnd(group.carryOut, carryOut),
                               "CarryPackManager: bit collision detected when combining partial carry outs");
        #endif
        group.carryOut = iBuilder->CreateOr(group.carryOut, carryOut);
    }

    group.packedSize += op->getCarryWidth();

    if (LLVM_UNLIKELY(group.groupSize == 1)) {
        Value * ptr = nullptr;
        if (LLVM_LIKELY(mLoopDepth == 0)) {
            ptr = cast<LoadInst>(group.carryIn)->getPointerOperand();
        } else {
            mCurrentFrameOffset.push_back(iBuilder->getInt32(group.frameIndex));
            mCurrentFrameOffset.push_back(mLoopSelector[1]);
            ptr = iBuilder->CreateGEP(mCurrentFrame, mCurrentFrameOffset);
            mCurrentFrameOffset.pop_back();
            mCurrentFrameOffset.pop_back();
            if (LLVM_LIKELY(!mCarryInfo->nonCarryCollapsingMode())) {
                group.carryOut = iBuilder->CreateOr(group.carryOut, iBuilder->CreateBlockAlignedLoad(ptr));
            }
        }

        if (mCarryInfo->hasSummary()) {
            // TODO: if this group is all within a single scope, only add the packed carry to the summary
            addToCarryOutSummary(iBuilder, group.carryOut);
        }

        iBuilder->CreateBlockAlignedStore(group.carryOut, ptr);
    }

    group.groupSize -= 1;

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readCarryInSummary
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::readCarryInSummary(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, ConstantInt * index) {
    assert (mCarryInfo->hasSummary());
    const auto size = mCurrentFrameOffset.size();
    // If we have a borrowed summary, we cannot be sure how deeply nested it is but do know that it will be the first
    // element of a nested struct that is not a struct. Traverse inwards until we find it.
    if (LLVM_UNLIKELY(mCarryInfo->hasBorrowedSummary())) {
        Type * frameTy = mCurrentFrame->getType()->getPointerElementType();
        for (unsigned i = 1; i < mCurrentFrameOffset.size(); ++i) {
            frameTy = frameTy->getStructElementType(cast<ConstantInt>(mCurrentFrameOffset[i])->getLimitedValue());
        }
        assert (frameTy->isStructTy());
        unsigned count = 1;
        for (;;) {
            frameTy = frameTy->getStructElementType(0);
            if (!frameTy->isStructTy()) {
                break;
            }
            ++count;
        }
        assert (count > 0);
        mCurrentFrameOffset.insert(mCurrentFrameOffset.end(), count - 1, iBuilder->getInt32(0));
    }
    mCurrentFrameOffset.push_back(index);
    if (mLoopDepth != 0) {
        mCurrentFrameOffset.push_back(mLoopSelector[0]);
    }
    Value * const summaryPtr = iBuilder->CreateGEP(mCurrentFrame, mCurrentFrameOffset);
    mCurrentFrameOffset.resize(size);
    Value * const summary = iBuilder->CreateBlockAlignedLoad(summaryPtr);
    if (mLoopDepth != 0 && mCarryInfo->hasExplicitSummary()) {
        Type * const carryTy = iBuilder->getBitBlockType();
        iBuilder->CreateBlockAlignedStore(Constant::getNullValue(carryTy), summaryPtr);
    }
    return summary;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeCarryOutSummary
 ** ------------------------------------------------------------------------------------------------------------- */
inline void CarryManager::writeCarryOutSummary(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, Value * const summary, ConstantInt * index) {
    assert (mCarryInfo->hasExplicitSummary());
    mCurrentFrameOffset.push_back(index);
    if (mLoopDepth != 0) {
        mCurrentFrameOffset.push_back(mLoopSelector[1]);
    }
    Value * const summaryPtr = iBuilder->CreateGEP(mCurrentFrame, mCurrentFrameOffset);
    iBuilder->CreateBlockAlignedStore(summary, summaryPtr);
    if (mLoopDepth != 0) {
        mCurrentFrameOffset.pop_back();
    }
    mCurrentFrameOffset.pop_back();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addToCarryOutSummary
 ** ------------------------------------------------------------------------------------------------------------- */
inline void CarryManager::addToCarryOutSummary(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, Value * const value) {
    assert ("cannot add null summary value!" && value);
    assert ("summary stack is empty!" && !mCarrySummaryStack.empty());
    assert ("current scope does not have a summary!" && mCarryInfo->hasSummary());
    Value * const currentSummary = mCarrySummaryStack.back();
    mCarrySummaryStack.back() = iBuilder->CreateOr(value, currentSummary);
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
#if 0
    return dyn_cast_or_null<While>(scope->getBranch()) != nullptr;
#else
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
#endif
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief assignDefaultCarryGroups
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned CarryManager::assignDefaultCarryGroups(PabloBlock * const scope, const unsigned ifDepth, const unsigned loopDepth, unsigned carryGroups) {
    const auto packingSize = getPackingSize(mVectorWidth, (ifDepth + loopDepth));
    unsigned packedWidth = 0;
    for (Statement * stmt : *scope) {
        if (LLVM_UNLIKELY(isa<CarryProducingStatement>(stmt))) {
            unsigned amount = 1;
            bool canPack = true;
            if (LLVM_LIKELY(isa<Advance>(stmt))) {
                amount = cast<Advance>(stmt)->getAmount();
                canPack = (amount < mElementWidth);
            }
            else if (isa<IndexedAdvance>(stmt)) {
                amount = cast<Advance>(stmt)->getAmount();
                canPack = (amount < mElementWidth);
            }
            if (packedWidth == 0) {
                ++carryGroups;
            }
            if (LLVM_LIKELY(canPack)) {
                amount = nearest_multiple(amount, packingSize);
                // if we cannot insert this carry into the current pack, start a new group
                if ((packedWidth + amount) > mVectorWidth) {
                    ++carryGroups;
                    packedWidth = amount;
                } else {
                    packedWidth += amount;
                }
            } else if (LLVM_LIKELY(packedWidth != 0)) {
                ++carryGroups;
                packedWidth = 0;
            }
            assert (carryGroups > 0);
            cast<CarryProducingStatement>(stmt)->setCarryGroup(carryGroups - 1);
            cast<CarryProducingStatement>(stmt)->setCarryWidth(amount);
        } else if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
            const auto nestedIfDepth = ifDepth + isa<If>(stmt) ? 1 : 0;
            const auto nestedLoopDepth = loopDepth + isa<If>(stmt) ? 0 : 1;
            carryGroups = assignDefaultCarryGroups(cast<Branch>(stmt)->getBody(), nestedIfDepth, nestedLoopDepth, carryGroups);
            packedWidth = 0;
        }
    }
    return carryGroups;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief analyse
 ** ------------------------------------------------------------------------------------------------------------- */
StructType * CarryManager::analyse(const std::unique_ptr<kernel::KernelBuilder> & iBuilder,
                                   const PabloBlock * const scope,
                                   const unsigned ifDepth, const unsigned loopDepth,
                                   const bool isNestedWithinNonCarryCollapsingLoop) {

    assert ("scope cannot be null!" && scope);
    assert ("the entry scope -- and only the entry scope -- must be in carry scope 0"
            && (mCarryScopes == 0 ? (scope == mKernel->getEntryScope()) : (scope != mKernel->getEntryScope())));
    assert (mCarryScopes < mCarryMetadata.size());

    Type * const carryTy = iBuilder->getBitBlockType();

    CarryData & cd = mCarryMetadata[mCarryScopes++];
    const bool nonCarryCollapsingMode = hasIterationSpecificAssignment(scope);
    Type * const carryPackTy = (loopDepth == 0) ? carryTy : ArrayType::get(carryTy, 2);
    std::vector<Type *> state;

    for (const Statement * stmt : *scope) {
        if (LLVM_UNLIKELY(isa<CarryProducingStatement>(stmt))) {
            const auto index = cast<CarryProducingStatement>(stmt)->getCarryGroup();
            assert (index < mCarryGroup.size());
            CarryGroup & carryGroup = mCarryGroup[index];
            if (carryGroup.groupSize == 0) {
                Type * packTy = carryPackTy;
                if (LLVM_UNLIKELY(isa<Advance>(stmt))) {
                    const auto amount = cast<Advance>(stmt)->getAmount();
                    if (LLVM_UNLIKELY(amount >= mElementWidth)) {
                        if (LLVM_UNLIKELY(ifDepth > 0 && amount > iBuilder->getBitBlockWidth())) {
                            // 1 bit will mark the presense of any bit in each block.
                            state.push_back(ArrayType::get(carryTy, ceil_udiv(amount, std::pow(iBuilder->getBitBlockWidth(), 2))));
                        }
                        mHasLongAdvance = true;
                        const auto blocks = ceil_udiv(amount, iBuilder->getBitBlockWidth());
                        packTy = ArrayType::get(carryTy, nearest_pow2(blocks + ((loopDepth != 0) ? 1 : 0)));
                    }
                }
                if (LLVM_UNLIKELY(isa<IndexedAdvance>(stmt))) {
                    // The carry data for the indexed advance stores N bits of carry data,
                    // organized in packs that can be processed with GR instructions (such as PEXT, PDEP, popcount).
                    // A circular buffer is used.  Because the number of bits to be dequeued
                    // and enqueued is variable (based on the popcount of the index), an extra
                    // pack stores the offset position in the circular buffer.
                    const auto amount = cast<IndexedAdvance>(stmt)->getAmount();
                    const auto packWidth = sizeof(size_t) * 8;
                    const auto packs = ceil_udiv(amount, packWidth);
                    packTy = ArrayType::get(iBuilder->getSizeTy(), nearest_pow2(packs) + 1);
                }
                state.push_back(packTy);
            }
            carryGroup.groupSize += 1;
        } else if (LLVM_UNLIKELY(isa<Branch>(stmt))) {
            unsigned nestedIfDepth = ifDepth;
            unsigned nestedLoopDepth = loopDepth;
            if (LLVM_LIKELY(isa<If>(stmt))) {
                ++nestedIfDepth;
            } else {
                mHasLoop = true;
                ++nestedLoopDepth;
            }
            state.push_back(analyse(iBuilder, cast<Branch>(stmt)->getBody(),
                                    nestedIfDepth, nestedLoopDepth, nonCarryCollapsingMode | isNestedWithinNonCarryCollapsingLoop));
        }
    }
    // Build the carry state struct and add the summary pack if needed.
    StructType * carryState = nullptr;
    CarryData::SummaryType summaryType = CarryData::NoSummary;
    if (LLVM_UNLIKELY(state.empty())) {
        carryState = StructType::get(iBuilder->getContext());
    } else {
        if (dyn_cast_or_null<If>(scope->getBranch()) || nonCarryCollapsingMode || isNestedWithinNonCarryCollapsingLoop) {
            if (LLVM_LIKELY(state.size() > 1)) {
                summaryType = CarryData::ExplicitSummary;
                // NOTE: summaries are stored differently depending whether we're entering an If or While branch. With an If branch, they
                // preceed the carry state data and with a While loop they succeed it. This is to help cache prefectching performance.
                state.insert(isa<If>(scope->getBranch()) ? state.begin() : state.end(), carryPackTy);
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
            mHasNonCarryCollapsingLoops = true;
            carryState = StructType::get(iBuilder->getContext(), {iBuilder->getSizeTy(), carryState->getPointerTo(), carryTy->getPointerTo()};
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
, mVectorWidth(0)
, mElementWidth(0)
, mCurrentFrame(nullptr)
, mCurrentFrameIndex(0)
, mCurrentScope(nullptr)
, mCarryInfo(nullptr)
, mNextSummaryTest(nullptr)
, mIfDepth(0)
, mHasLongAdvance(false)
, mHasNonCarryCollapsingLoops(false)
, mHasLoop(false)
, mLoopDepth(0)
, mLoopSelector{nullptr, nullptr}
, mCarryScopes(0) {

}

}
