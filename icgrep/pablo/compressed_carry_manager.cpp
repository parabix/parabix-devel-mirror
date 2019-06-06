/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "compressed_carry_manager.h"
#include <pablo/carry_data.h>
#include <pablo/codegenstate.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/DataLayout.h>
#include <llvm/Transforms/Utils/Local.h>
#include <pablo/branch.h>
#include <pablo/pablo_intrinsic.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_var.h>
#include <kernels/kernel_builder.h>
#include <toolchain/toolchain.h>
#include <array>

#define LONG_ADVANCE_BREAKPOINT 64

using namespace llvm;

namespace pablo {

/* Local Helper Functions */

inline static bool isNonAdvanceCarryGeneratingStatement(const Statement * const stmt) {
    if (IntrinsicCall const * call = dyn_cast<IntrinsicCall>(stmt)) {
        return call->isCarryProducing() && !call->isAdvanceType();
    } else {
        return isa<CarryProducingStatement>(stmt) && !isa<Advance>(stmt) && !isa<IndexedAdvance>(stmt);
    }
}

static inline bool isNonRegularLanguage(const PabloBlock * const scope) {
    if (const Branch * br = scope->getBranch()) {
        return !br->isRegular();
    }
    return false;
}


static inline bool isDynamicallyAllocatedType(const Type * const ty) {
    if (isa<StructType>(ty) && ty->getStructNumElements() == 3) {
        return (ty->getStructElementType(1)->isPointerTy() && ty->getStructElementType(2)->isPointerTy() && ty->getStructElementType(0)->isIntegerTy());
    }
    return false;
}


static inline bool hasNonEmptyCarryStruct(const Type * const frameTy) {
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


static inline bool hasNonEmptyCarryStruct(const std::vector<Type *> & state) {
    for (const Type * const frameTy : state) {
        if (hasNonEmptyCarryStruct(frameTy)) {
            return true;
        }
    }
    return false;
}


static inline unsigned ceil_log2(const unsigned v) {
    assert ("log2(0) is undefined!" && v != 0);
    return (sizeof(unsigned) * CHAR_BIT) - __builtin_clz(v - 1U);
}


static inline unsigned nearest_pow2(const unsigned v) {
    assert (v > 0 && v < (UINT32_MAX / 2));
    return (v < 2) ? 1 : (1U << ceil_log2(v));
}


static inline unsigned ceil_udiv(const unsigned x, const unsigned y) {
    return (((x - 1) | (y - 1)) + 1) / y;
}


static Value * castToSummaryType(const std::unique_ptr<kernel::KernelBuilder> & b, Value * carryOut, Type * summaryTy) {
    if (!(carryOut->getType()->isIntegerTy() || carryOut->getType() == b->getBitBlockType())) {
        assert (false);
    }

    if (carryOut->getType() == summaryTy) {
        return carryOut;
    } else if (summaryTy == b->getBitBlockType()) {
        return b->CreateBitCast(b->CreateZExt(carryOut, b->getIntNTy(b->getBitBlockWidth())), b->getBitBlockType());
    } else {
        // assert (carryOut->getType()->getPrimitiveSizeInBits() <= summaryTy->getPrimitiveSizeInBits());
        return b->CreateZExt(carryOut, summaryTy);
    }
}


// Recursively determines the minimum summary size needed, in bits, for a given pablo block.
static int32_t analyseSummarySize(int32_t blockWidth, const PabloBlock * const scope) {
    int32_t carrySize = 0;
    for (const Statement * stmt : *scope) {
        if (LLVM_UNLIKELY(isa<Advance>(stmt) || isa<IndexedAdvance>(stmt))) {
            int64_t amount = isa<Advance>(stmt)
                ? cast<Advance>(stmt)->getAmount()
                : cast<IndexedAdvance>(stmt)->getAmount();
            if (LLVM_LIKELY(amount < 8)) {
                carrySize = std::max(8, carrySize);
            } else if (amount >= 8 && amount < LONG_ADVANCE_BREAKPOINT) {
                carrySize = std::max(64, carrySize);
            } else {
                carrySize = blockWidth;
            }
        } else if (LLVM_UNLIKELY(isNonAdvanceCarryGeneratingStatement(stmt))) {
            carrySize = std::max(8, carrySize);
        } else if (LLVM_UNLIKELY(isa<If>(stmt))) {
            carrySize = std::max(analyseSummarySize(blockWidth, cast<If>(stmt)->getBody()), carrySize);
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            carrySize = std::max(analyseSummarySize(blockWidth, cast<While>(stmt)->getBody()), carrySize);
        }
    }
    assert (carrySize >= 0 && carrySize <= blockWidth);
    return carrySize;
}


static Type * toSummaryType(const std::unique_ptr<kernel::KernelBuilder> & b, int32_t summarySize) {
    switch (summarySize) {
    case 8:
        return b->getInt8Ty();
    case 64:
        return b->getInt64Ty();
    default:
        assert ("unexpected summary type" && (uint32_t) summarySize == b->getBitBlockWidth());
        return b->getBitBlockType();
    }
}


static Value * compressImplicitSummary(const std::unique_ptr<kernel::KernelBuilder> & b, Value * summary) {
    if (summary->getType() == b->getBitBlockType()) {
        summary = b->CreateBitCast(summary, b->getIntNTy(b->getBitBlockWidth()));
    }
    return b->CreateICmpNE(summary, Constant::getNullValue(summary->getType()));
}


/* ===== Initialization ===== */


void CompressedCarryManager::initializeCodeGen(const std::unique_ptr<kernel::KernelBuilder> & b) {

    assert(!mCarryMetadata.empty());
    mCarryInfo = &mCarryMetadata[0];
    assert (!mCarryInfo->hasSummary());
    mCurrentFrame = b->getScalarFieldPtr("carries");
    mCurrentFrameIndex = 0;
    mCarryScopes = 0;
    mCarryScopeIndex.push_back(0);
    assert (mCarryFrameStack.empty());
    assert (mCarrySummaryStack.empty());

    int32_t baseSummarySize = analyseSummarySize(b->getBitBlockWidth(), mKernel->getEntryScope());
    mBaseSummaryType = baseSummarySize > 0 ? toSummaryType(b, baseSummarySize) : b->getInt8Ty();
    mCarrySummaryStack.push_back(Constant::getNullValue(mBaseSummaryType));

    if (mHasLoop) {
        mLoopSelector = b->getScalarField("selector");
        mNextLoopSelector = b->CreateXor(mLoopSelector, ConstantInt::get(mLoopSelector->getType(), 1));
    }
}


/* ===== Scope Changes ===== */


void CompressedCarryManager::enterLoopBody(const std::unique_ptr<kernel::KernelBuilder> & b, BasicBlock * const entryBlock) {
    CarryManager::enterLoopBody(b, entryBlock);
    // if (mCarryInfo->hasSummary()) {
    //     assert (!mCarrySummaryStack.empty());

    //     Value * & summary = mCarrySummaryStack.back();
    //     Type * const summaryTy = summary->getType();
    //     PHINode * phiCarryOutSummary = b->CreatePHI(summaryTy, 2, "summary");
    //     phiCarryOutSummary->addIncoming(summary, entryBlock);
    //     // Replace the incoming carry summary with the phi node and add the phi node to the stack  so that we can
    //     // properly OR it into the outgoing summary value.
    //     // NOTE: this may change the base summary value; when exiting to the base scope, replace this summary with
    //     // a null value to prevent subsequent nested scopes from inheriting the summary of this scope.
    //     summary = phiCarryOutSummary;
    //     mCarrySummaryStack.push_back(phiCarryOutSummary);
    // }
    // if (LLVM_UNLIKELY(mCarryInfo->nonCarryCollapsingMode())) {
    //     assert ("non-carry collapsing mode is not supported yet" && false);
    // }
}


void CompressedCarryManager::leaveLoopBody(const std::unique_ptr<kernel::KernelBuilder> & b, BasicBlock * exitBlock) {
    CarryManager::leaveLoopBody(b, exitBlock);
    // if (LLVM_UNLIKELY(mCarryInfo->nonCarryCollapsingMode())) {
    //     assert ("non-carry collapsing mode is not supported yet" && false);
    // }

    // if (mCarryInfo->hasSummary()) {
    //     const auto n = mCarrySummaryStack.size(); assert (n > 1);
    //     Value * carryOut = nullptr;
    //     if (mCarryInfo->hasImplicitSummary()) {
    //         carryOut = convertFrameToImplicitSummary(b);
    //         Type * const summaryTy = mCarrySummaryStack.back()->getType();
    //         if (carryOut->getType() != summaryTy) {
    //             carryOut = b->CreateICmpNE(carryOut, Constant::getNullValue(carryOut->getType()));
    //             carryOut = b->CreateZExt(carryOut, summaryTy);
    //         }
    //     } else {
    //         carryOut = mCarrySummaryStack.back();
    //         mCarrySummaryStack.pop_back();
    //     }

    //     assert(isa<PHINode>(mCarrySummaryStack.back()));
    //     PHINode * phiCarryOut = cast<PHINode>(mCarrySummaryStack.back());
    //     carryOut = castToSummaryType(b, carryOut, phiCarryOut->getType());
    //     phiCarryOut->addIncoming(carryOut, b->GetInsertBlock());
    //     // If we're returning to the base scope, reset our accumulated summary value.
    //     if (n == 2) {
    //         carryOut = Constant::getNullValue(mBaseSummaryType);
    //     }
    //     mCarrySummaryStack.back() = carryOut;
    // }
}


void CompressedCarryManager::enterIfScope(const std::unique_ptr<kernel::KernelBuilder> & b, const PabloBlock * const /*scope*/) {
    ++mIfDepth;
    enterScope(b);
    // We zero-initialized the nested summary value and later OR in the current summary into the escaping summary
    // so that upon processing the subsequent block iteration, we branch into this If scope iff a carry out was
    // generated by a statement within this If scope and not by a dominating statement in the outer scope.
    Type * summaryTy = b->getInt8Ty();
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        // errs() << mCurrentFrameIndex << "\n";
        // assert (mCurrentFrameIndex == 0);
        mNextSummaryTest = readCarryInSummary(b);
        if (mCarryInfo->hasExplicitSummary()) {
            Type * const frameTy = mCurrentFrame->getType()->getPointerElementType();
            assert (frameTy->isStructTy() && frameTy->getStructNumElements() > 0);
            summaryTy = frameTy->getStructElementType(mCurrentFrameIndex)->getArrayElementType();
            // mCurrentFrameIndex = 1;
        }
        mCarrySummaryStack.push_back(Constant::getNullValue(summaryTy));
    }
}


void CompressedCarryManager::leaveIfBody(const std::unique_ptr<kernel::KernelBuilder> & b, BasicBlock * const exitBlock) {
    assert (exitBlock);
    const auto n = mCarrySummaryStack.size();
    if (LLVM_LIKELY(mCarryInfo->hasExplicitSummary())) {
        writeCarryOutSummary(b, mCarrySummaryStack[n - 1]);
    }

    if (n > 2) {
        Value * & nested = mCarrySummaryStack[n - 1];
        Value * & outer = mCarrySummaryStack[n - 2];

        if (mCarryInfo->hasImplicitSummary()) {
            nested = convertFrameToImplicitSummary(b);
            if (nested->getType() != outer->getType()) {
                nested = compressImplicitSummary(b, nested);
            }
        }

        nested = castToSummaryType(b, nested, outer->getType());
        nested = b->CreateOr(nested, outer, "summary");
    }
}


void CompressedCarryManager::leaveScope() {
    // Did we use all of the packs in this carry struct?
    // We can't check this for frames with implicit summaries as there is extra data to pad the frame.
    assert (mCarryInfo->hasImplicitSummary() || mCurrentFrameIndex == mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
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


/* ===== Operations ===== */


Value * CompressedCarryManager::addCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & b, const Statement * const operation, Value * const e1, Value * const e2) {
    assert (operation && (isNonAdvanceCarryGeneratingStatement(operation)));
    Value * carryIn = getNextCarryIn(b);
    assert (carryIn->getType() == b->getInt8Ty());
    Value * carryOut, * result;
    std::tie(carryOut, result) = b->bitblock_add_with_carry(e1, e2, carryIn);
    assert (result->getType() == b->getBitBlockType());
    assert (carryOut->getType() == b->getInt8Ty());
    setNextCarryOut(b, carryOut);
    return result;
}


Value * CompressedCarryManager::advanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & b, const Advance * const advance, Value * const value) {
    const auto shiftAmount = advance->getAmount();
    if (LLVM_LIKELY(shiftAmount < LONG_ADVANCE_BREAKPOINT)) {
        Value * carryIn = getNextCarryIn(b);
        Value * carryOut, * result;
        std::tie(carryOut, result) = b->bitblock_advance(value, carryIn, shiftAmount);
        assert (result->getType() == b->getBitBlockType());
        assert (carryOut->getType() == carryIn->getType());
        setNextCarryOut(b, carryOut);
        return result;
    } else {
        return longAdvanceCarryInCarryOut(b, value, shiftAmount);
    }
}

Value * CompressedCarryManager::indexedAdvanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & b,
                                                              const IndexedAdvance * const advance,
                                                              Value * const strm,
                                                              Value * const index_strm)
{
    const auto shiftAmount = advance->getAmount();
    if (LLVM_LIKELY(shiftAmount < LONG_ADVANCE_BREAKPOINT)) {
        Value * carryIn = getNextCarryIn(b);
        carryIn = b->CreateBitCast(b->CreateZExt(carryIn, b->getIntNTy(b->getBitBlockWidth())), b->getBitBlockType());
        Value * carryOut, * result;
        std::tie(carryOut, result) = b->bitblock_indexed_advance(strm, index_strm, carryIn, shiftAmount);
        const uint32_t fw = shiftAmount < 8 ? 8 : 64;
        carryOut = b->mvmd_extract(fw, carryOut, 0);
        setNextCarryOut(b, carryOut);
        return result;
    } else if (shiftAmount <= b->getBitBlockWidth()) {
        Value * carryPtr = b->CreateGEP(mCurrentFrame, {b->getInt32(0), b->getInt32(mCurrentFrameIndex++), b->getInt32(0)});
        assert (carryPtr->getType()->getPointerElementType() == b->getBitBlockType());
        Value * carryIn = b->CreateBlockAlignedLoad(carryPtr);
        Value * carryOut, * result;
        std::tie(carryOut, result) = b->bitblock_indexed_advance(strm, index_strm, carryIn, shiftAmount);
        assert (carryOut->getType() == b->getBitBlockType());
        b->CreateBlockAlignedStore(carryOut, carryPtr);
        if ((mIfDepth > 0) && mCarryInfo->hasExplicitSummary()) {
            addToCarryOutSummary(b, strm);
        }
        return result;
    } else {
        unsigned summaryFrame = mCurrentFrameIndex;
        if (mIfDepth > 0) {
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
        if (mIfDepth > 0) {
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


Value * CompressedCarryManager::generateSummaryTest(const std::unique_ptr<kernel::KernelBuilder> & b, Value * condition) {
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        assert ("summary condition cannot be null!" && condition);
        assert ("summary test was not generated" && mNextSummaryTest);
        assert ("summary condition and test must have the same context!" && &condition->getContext() == &mNextSummaryTest->getContext());
        if (mNextSummaryTest->getType() != condition->getType()) {
            assert (condition->getType() == b->getBitBlockType());
            mNextSummaryTest = castToSummaryType(b, mNextSummaryTest, condition->getType());
        }
        condition = b->simd_or(condition, mNextSummaryTest);
        mNextSummaryTest = nullptr;
    }
    assert ("summary test was not consumed" && (mNextSummaryTest == nullptr));
    return condition;
}


/* ===== Single Carry Get/Set ===== */


Value * CompressedCarryManager::getNextCarryIn(const std::unique_ptr<kernel::KernelBuilder> & b) {
    assert (mCurrentFrameIndex < mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    Constant * const ZERO = b->getInt32(0);
    Value * indices[3] { ZERO, b->getInt32(mCurrentFrameIndex), (mLoopDepth == 0 ? ZERO : mLoopSelector) };
    ArrayRef<Value *> ar(indices, 3);
    mCarryPackPtr = b->CreateGEP(mCurrentFrame, ar);
    Type * const carryTy = mCarryPackPtr->getType()->getPointerElementType();
    Value * const carryIn = b->CreateLoad(mCarryPackPtr);
    if (mLoopDepth > 0) {
        b->CreateStore(Constant::getNullValue(carryTy), mCarryPackPtr);
    }
    return carryIn;
}


void CompressedCarryManager::setNextCarryOut(const std::unique_ptr<kernel::KernelBuilder> & b, Value * carryOut) {
    assert (mCurrentFrameIndex < mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    // errs() << "carryout: " << *carryOut->getType() << "\n";
    assert("carry out type does not match carry location type" && carryOut->getType() == mCarryPackPtr->getType()->getPointerElementType());
    if (mCarryInfo->hasSummary()) {
        addToCarryOutSummary(b, carryOut);
    }
    if (mLoopDepth != 0) {
        mCarryPackPtr = b->CreateGEP(mCurrentFrame, {b->getInt32(0), b->getInt32(mCurrentFrameIndex), mNextLoopSelector});
        assert("carry out type does not match carry location type" && carryOut->getType() == mCarryPackPtr->getType()->getPointerElementType());
        if (LLVM_LIKELY(!mCarryInfo->nonCarryCollapsingMode())) {
            Value * accum = b->CreateLoad(mCarryPackPtr);
            carryOut = b->CreateOr(carryOut, accum);
        }
    }
    ++mCurrentFrameIndex;
    b->CreateStore(carryOut, mCarryPackPtr);
}


/* ===== Summary Operations ===== */


Value * CompressedCarryManager::readCarryInSummary(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    assert (mCarryInfo->hasSummary());

    if (LLVM_LIKELY(mCarryInfo->hasImplicitSummary())) {
        Value * summary = convertFrameToImplicitSummary(b);
        return summary;
    } else {
        assert (mCarryInfo->hasExplicitSummary());
        Constant * const ZERO = b->getInt32(0);
        Value * indices[3] { ZERO, ZERO, (mLoopDepth == 0 ? ZERO : mLoopSelector) };
        ArrayRef<Value *> ar(indices, 3);
        Value * const ptr = b->CreateGEP(mCurrentFrame, ar);
        Value * const summary = b->CreateLoad(ptr);
        if (mLoopDepth != 0) {
            b->CreateStore(Constant::getNullValue(summary->getType()), ptr);
        }
        return summary;
    }
}


void CompressedCarryManager::writeCarryOutSummary(const std::unique_ptr<kernel::KernelBuilder> & b, Value * const summary) const {
    assert (mCarryInfo->hasExplicitSummary());
    Constant * const ZERO = b->getInt32(0);
    Value * indices[3] { ZERO, ZERO, (mLoopDepth == 0 ? ZERO : mLoopSelector) };
    ArrayRef<Value *> ar(indices, 3);
    Value * ptr = b->CreateGEP(mCurrentFrame, ar);
    // errs() << *ptr->getType()->getPointerElementType() << "\n";
    // errs() << *summary->getType() << "\n";
    assert ("summary type does not match defined type in frame" && ptr->getType()->getPointerElementType() == summary->getType());
    b->CreateStore(summary, ptr);
}


void CompressedCarryManager::addToCarryOutSummary(const std::unique_ptr<kernel::KernelBuilder> & b, Value * const value) {
    assert (mCarryInfo->hasSummary());

    // No need to add to summary if using an implicit summary
    if (mCarryInfo->hasExplicitSummary()) {
        assert ("cannot add null summary value!" && value);
        assert ("summary stack is empty!" && !mCarrySummaryStack.empty());

        Value * & summary = mCarrySummaryStack.back();
        Value * const carryOut = castToSummaryType(b, value, summary->getType());
        summary = b->CreateOr(summary, carryOut);
    }
}


Value * CompressedCarryManager::convertFrameToImplicitSummary(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    assert (mCarryInfo->hasSummary());
    assert (mCarryInfo->hasImplicitSummary());

    Type * const frameTy = mCurrentFrame->getType()->getPointerElementType();
    assert (frameTy->isStructTy());
    auto DL = b->getModule()->getDataLayout();
    auto frameSize = DL.getStructLayout(cast<StructType>(frameTy))->getSizeInBits();
    assert (frameSize == 64 || frameSize == b->getBitBlockWidth());

    Type * const summaryTy = frameSize == 64 ? (Type *) b->getInt64Ty() : b->getBitBlockType();
    Value * const ptr = b->CreatePointerCast(mCurrentFrame, summaryTy->getPointerTo());
    Value * const summary = b->CreateAlignedLoad(ptr, 1);
    return summary;
}


/* ==== Scope Analyse ===== */


StructType * CompressedCarryManager::analyse(const std::unique_ptr<kernel::KernelBuilder> & b,
                                             const PabloBlock * const scope,
                                             const unsigned ifDepth,
                                             const unsigned whileDepth,
                                             const bool isNestedWithinNonCarryCollapsingLoop)
{
    assert ("scope cannot be null!" && scope);
    assert ("entry scope (and only the entry scope) must be in scope 0"
            && (mCarryScopes == 0 ? (scope == mKernel->getEntryScope()) : (scope != mKernel->getEntryScope())));
    assert (mCarryScopes < mCarryMetadata.size());

    Type * const i8Ty = b->getInt8Ty();
    Type * const i64Ty = b->getInt64Ty();
    Type * const blockTy = b->getBitBlockType();
    const uint32_t blockWidth = b->getBitBlockWidth();

    const uint32_t carryScopeIndex = mCarryScopes++;
    const bool nonCarryCollapsingMode = isNonRegularLanguage(scope);
    assert ("non carry collapsing mode is not yet supported by this carry manager" && !nonCarryCollapsingMode);

    const uint64_t packSize = whileDepth == 0 ? 1 : 2;
    Type * const i8PackTy = ArrayType::get(i8Ty, packSize);
    Type * const i64PackTy = ArrayType::get(i64Ty, packSize);
    Type * const blockPackTy = ArrayType::get(blockTy, packSize);

    bool canUseImplicitSummary = packSize == 1 && !nonCarryCollapsingMode;
    std::size_t carryProducingStatementCount = 0;
    const std::size_t maxNumSmallCarriesForImplicitSummary = blockWidth / 8;

    /* Get Carry Types */

    std::vector<Type *> state;
    for (const Statement * stmt : *scope) {
        if (LLVM_UNLIKELY(isa<Advance>(stmt) || isa<IndexedAdvance>(stmt))) {
            carryProducingStatementCount++;
            int64_t amount = isa<Advance>(stmt)
                ? cast<Advance>(stmt)->getAmount()
                : cast<IndexedAdvance>(stmt)->getAmount();
            Type * type = i8PackTy;
            if (LLVM_UNLIKELY(amount >= 8 && amount < LONG_ADVANCE_BREAKPOINT)) {
                canUseImplicitSummary = false;
                type = i64PackTy;
            } else if (LLVM_UNLIKELY(amount >= LONG_ADVANCE_BREAKPOINT)) {
                canUseImplicitSummary = false;
                const auto blockWidth = b->getBitBlockWidth();
                const auto blocks = ceil_udiv(amount, blockWidth);
                type = ArrayType::get(blockTy, nearest_pow2(blocks + (isa<IndexedAdvance>(stmt) ? 1:0) + ((whileDepth != 0) ? 1 : 0)));
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
            carryProducingStatementCount++;
            state.push_back(i8PackTy);
        } else if (LLVM_UNLIKELY(isa<If>(stmt))) {
            canUseImplicitSummary = false;
            state.push_back(analyse(b, cast<If>(stmt)->getBody(), ifDepth+1, whileDepth, nonCarryCollapsingMode | isNestedWithinNonCarryCollapsingLoop));
        } else if (LLVM_UNLIKELY(isa<While>(stmt))) {
            canUseImplicitSummary = false;
            mHasLoop = true;
            state.push_back(analyse(b, cast<While>(stmt)->getBody(), ifDepth, whileDepth+1, nonCarryCollapsingMode | isNestedWithinNonCarryCollapsingLoop));
        }

        if (carryProducingStatementCount >= maxNumSmallCarriesForImplicitSummary)
            canUseImplicitSummary = false;
    }

    /* Construct Carry State Struct */
    CarryData & cd = mCarryMetadata[carryScopeIndex];
    StructType * carryStruct = nullptr;
    CarryData::SummaryType summaryType = CarryData::NoSummary;

    if (LLVM_UNLIKELY(state.empty())) {
        carryStruct = StructType::get(b->getContext());
    } else {
        if (LLVM_LIKELY(ifDepth > 0 || whileDepth > 0)) {
            if (LLVM_LIKELY(canUseImplicitSummary)) {
                summaryType = CarryData::ImplicitSummary;

                // If needed, pad the structure to 64 bits or the bitblock width. This allows us to bitcast the structure to an i64 or
                // bitblock to get the summary.
                carryStruct = StructType::get(b->getContext(), state);
                const DataLayout & DL = b->getModule()->getDataLayout();
                uint64_t structBitWidth = DL.getStructLayout(carryStruct)->getSizeInBits();
                const std::size_t targetWidth = structBitWidth <= 64 ? 64 : blockWidth;
                while (structBitWidth < targetWidth) {
                    state.push_back(i8Ty);
                    structBitWidth += 8;
                }
            } else {
                summaryType = CarryData::ExplicitSummary;

                // Insert the smallest possible summary for this scope.
                int32_t summarySize = analyseSummarySize((int32_t) blockWidth, scope);
                Type * summaryType = toSummaryType(b, summarySize);
                state.insert(state.begin(), ArrayType::get(summaryType, packSize));
            }
        }

        carryStruct = StructType::get(b->getContext(), state);
        // errs() << *carryStruct << "\n\n";

        // If we're in a loop and cannot use collapsing carry mode, convert the carry state struct into a capacity,
        // carry state pointer, and summary pointer struct.
        if (LLVM_UNLIKELY(nonCarryCollapsingMode)) {
            assert ("non carry collapsing mode is not supported yet" && false);
        }
        cd.setNonCollapsingCarryMode(nonCarryCollapsingMode);
    }

    cd.setSummaryType(summaryType);
    return carryStruct;
}

CompressedCarryManager::CompressedCarryManager() noexcept
: CarryManager()
{}

} // namespace pablo
