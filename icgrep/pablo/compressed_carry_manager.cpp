/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "compressed_carry_manager.h"
#include <kernels/kernel_builder.h>

#define LONG_ADVANCE_BREAKPOINT 64

using namespace llvm;

namespace pablo {

inline static Value * ZExtToBitBlockTy(const std::unique_ptr<kernel::KernelBuilder> & b, Value * value) {
    return b->CreateBitCast(b->CreateZExt(value, b->getIntNTy(b->getBitBlockWidth())), b->getBitBlockType());
}

llvm::Value * CompressedCarryManager::getNextCarryIn(const std::unique_ptr<kernel::KernelBuilder> & b) {
    assert (mCurrentFrameIndex < mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    Constant * const ZERO = b->getInt32(0);
    Value * indices[3];
    indices[0] = ZERO;
    indices[1] = b->getInt32(mCurrentFrameIndex);
    if (mLoopDepth == 0) {
        indices[2] = ZERO;
    } else {
        indices[2] = mLoopSelector;
    }
    ArrayRef<Value *> ar(indices, 3);
    mCarryPackPtr = b->CreateGEP(mCurrentFrame, ar);
    Type * const carryTy = b->getInt64Ty();
    assert (mCarryPackPtr->getType()->getPointerElementType() == carryTy);
    Value * const carryIn = b->CreateLoad(mCarryPackPtr);
    if (mLoopDepth > 0) {
        b->CreateStore(Constant::getNullValue(carryTy), mCarryPackPtr);
    }

    return ZExtToBitBlockTy(b, carryIn);
}

void CompressedCarryManager::setNextCarryOut(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Value * carryOut) {
    Type * const carryTy = b->getInt64Ty();
    assert (mCurrentFrameIndex < mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    if (carryOut->getType() == b->getBitBlockType()) {
        carryOut = b->CreateBitCast(carryOut, b->getIntNTy(b->getBitBlockWidth()));
        carryOut = b->CreateTrunc(carryOut, carryTy);
    }
    assert(carryOut->getType() == carryTy);
    if (mCarryInfo->hasSummary()) {
        addToCarryOutSummary(b, carryOut);
    }
    if (mLoopDepth != 0) {
        mCarryPackPtr = b->CreateGEP(mCurrentFrame, {b->getInt32(0), b->getInt32(mCurrentFrameIndex), mNextLoopSelector});
        if (LLVM_LIKELY(!mCarryInfo->nonCarryCollapsingMode())) {
            Value * accum = b->CreateLoad(mCarryPackPtr);
            if (LLVM_LIKELY(accum->getType() == carryTy)) {
                carryOut = b->CreateOr(carryOut, accum);
            } else {
                carryOut = b->CreateOr(ZExtToBitBlockTy(b, carryOut), accum);
            }
        }
    }
    ++mCurrentFrameIndex;
    assert (mCarryPackPtr->getType()->getPointerElementType() == carryTy);
    b->CreateStore(carryOut, mCarryPackPtr);
}

void CompressedCarryManager::addToCarryOutSummary(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Value * const value) {
    assert ("cannot add null summary value!" && value);
    assert ("summary stack is empty!" && !mCarrySummaryStack.empty());
    assert (mCarryInfo->hasSummary());
    Value * carryOut = value;
    if (value->getType() == b->getInt64Ty()) {
        carryOut = ZExtToBitBlockTy(b, value);
    }
    mCarrySummaryStack.back() = b->CreateOr(carryOut, mCarrySummaryStack.back());
}

Value * CompressedCarryManager::readCarryInSummary(const std::unique_ptr<kernel::KernelBuilder> & b, ConstantInt * index) const {
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
    std::fill(indicies.begin(), indicies.end(), b->getInt32(0));
    indicies[count - 1] = index;
    if (mLoopDepth != 0) {
        indicies[count] = mLoopSelector;
    }
    Value * const ptr = b->CreateGEP(mCurrentFrame, indicies);
    Value * summary = b->CreateLoad(ptr);
    if (mLoopDepth != 0 && mCarryInfo->hasExplicitSummary()) {
        Type * const carryTy = ptr->getType()->getPointerElementType();
        b->CreateStore(Constant::getNullValue(carryTy), ptr);
    }
    if (summary->getType() == b->getInt64Ty()) {
        summary = ZExtToBitBlockTy(b, summary);
    }
    return summary;
}

StructType * CompressedCarryManager::analyse(const std::unique_ptr<kernel::KernelBuilder> & b, const PabloBlock * const scope,
                                   const unsigned ifDepth, const unsigned loopDepth, const bool isNestedWithinNonCarryCollapsingLoop) {
    return CarryManager::analyse(b, scope, ifDepth, loopDepth, isNestedWithinNonCarryCollapsingLoop, b->getInt64Ty(), b->getBitBlockType());
}

CompressedCarryManager::CompressedCarryManager() noexcept
: CarryManager()
{}

}
