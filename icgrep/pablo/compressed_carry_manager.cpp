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


void CompressedCarryManager::initializeCodeGen(BuilderRef b) {

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

void CompressedCarryManager::enterLoopBody(BuilderRef b, BasicBlock * const entryBlock) {
    assert (mLoopDepth > 0);

    Type * const carryTy = getSummaryTypeFromCurrentFrame();
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
        assert ("non carry collapsing mode is not supported yet" && false);
    }
}


void CompressedCarryManager::leaveLoopBody(BuilderRef b, BasicBlock * exitBlock) {
    assert (mLoopDepth > 0);
    if (LLVM_UNLIKELY(mCarryInfo->nonCarryCollapsingMode())) {
        assert ("non carry collapsing mode is not supported yet" && false);
    }

    if (mLoopDepth == 1) {
        Constant * const ZEROES = Constant::getNullValue(mNestedLoopCarryInMaskPhi->getType());
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


void CompressedCarryManager::enterIfScope(BuilderRef b, const PabloBlock * const /*scope*/) {
    ++mIfDepth;++mIfDepth;
    enterScope(b);
    // We zero-initialized the nested summary value and later OR in the current summary into the escaping summary
    // so that upon processing the subsequent block iteration, we branch into this If scope iff a carry out was
    // generated by a statement within this If scope and not by a dominating statement in the outer scope.
    if (mCarryInfo->hasExplicitSummary()) {
        Type * const summaryTy = getSummaryTypeFromCurrentFrame();
        mCarrySummaryStack.push_back(Constant::getNullValue(summaryTy)); // new carry out summary accumulator
    } else if (mCarryInfo->hasImplicitSummary()) {
        mCarrySummaryStack.push_back(convertFrameToImplicitSummaryPtr(b));
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


void CompressedCarryManager::phiCurrentCarryOutSummary(BuilderRef b, BasicBlock * const entryBlock, BasicBlock * const exitBlock) {
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        const auto n = mCarrySummaryStack.size(); assert (n > 0);
        Value * const nested = mCarrySummaryStack[n - 1];
        PHINode * const phi = b->CreatePHI(nested->getType(), 2, "summary");
        Constant * const ZEROES = Constant::getNullValue(nested->getType());
        phi->addIncoming(ZEROES, entryBlock);
        phi->addIncoming(nested, exitBlock);
        mCarrySummaryStack[n - 1] = phi;
    }
}


void CompressedCarryManager::phiOuterCarryOutSummary(BuilderRef b, llvm::BasicBlock * const entryBlock, llvm::BasicBlock * const exitBlock) {
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        const auto n = mCarrySummaryStack.size(); assert (n > 0);
        if (n > 2) {
            // When leaving a nested If scope with a summary value, phi out the summary to ensure the
            // appropriate summary is stored in the outer scope.
            Value * nested = mCarrySummaryStack[n - 1];
            Value * outer = mCarrySummaryStack[n - 2];
            if (nested->getType() != outer->getType()) {
                if (outer->getType() == b->getBitBlockType()) {
                    nested = b->bitCast(b->CreateZExt(nested, b->getIntNTy(b->getBitBlockWidth())));
                } else {
                    nested = b->CreateZExt(nested, outer->getType());
                }
            }
            PHINode * const phi = b->CreatePHI(outer->getType(), 2, "summary");
            phi->addIncoming(outer, entryBlock);
            phi->addIncoming(nested, exitBlock);
            mCarrySummaryStack[n - 2] = phi;
        }
    }
}


void CompressedCarryManager::combineCarryOutSummary(BuilderRef b, const unsigned offset) {
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
        const auto n = mCarrySummaryStack.size(); assert (n > 0);
        // combine the outer summary with the nested summary so that when
        // we leave the scope, we'll properly phi out the value of the new
        // outer summary
        if (n > 2) {
            Value * nested = mCarrySummaryStack[n - 1];
            if (mCarryInfo->hasImplicitSummary()) {
                nested = loadImplicitSummaryFromPtr(b, nested);
            }
            Value * const outer = mCarrySummaryStack[n - 2];
            if (nested->getType() != outer->getType()) {
                nested = compressImplicitSummary(b, nested);
                if (outer->getType() == b->getBitBlockType()) {
                    nested = b->bitCast(b->CreateZExt(nested, b->getIntNTy(b->getBitBlockWidth())));
                } else {
                    nested = b->CreateZExt(nested, outer->getType());
                }
            }
            mCarrySummaryStack[n - offset] = b->CreateOr(outer, nested);
        }
    }
}


/* ===== Operations ===== */


Value * CompressedCarryManager::addCarryInCarryOut(BuilderRef b, const Statement * const operation, Value * const e1, Value * const e2) {
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


Value * CompressedCarryManager::advanceCarryInCarryOut(BuilderRef b, const Advance * const advance, Value * const value) {
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

Value * CompressedCarryManager::indexedAdvanceCarryInCarryOut(BuilderRef b,
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


Value * CompressedCarryManager::generateSummaryTest(BuilderRef b, Value * condition) {
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


Value * CompressedCarryManager::getNextCarryIn(BuilderRef b) {
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


void CompressedCarryManager::setNextCarryOut(BuilderRef b, Value * carryOut) {
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


Value * CompressedCarryManager::readCarryInSummary(BuilderRef b) const {
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
        Value * summary = b->CreateLoad(ptr);
        if (mNestedLoopCarryInMaskPhi) {
            summary = b->CreateAnd(summary, mNestedLoopCarryInMaskPhi);
        }
        return summary;
    }
}


void CompressedCarryManager::writeCarryOutSummary(BuilderRef b, Value * const summary) const {
    assert (mCarryInfo->hasExplicitSummary());
    Constant * const ZERO = b->getInt32(0);
    Value * indices[3] { ZERO, ZERO, (mLoopDepth == 0 ? ZERO : mNextLoopSelector) };
    ArrayRef<Value *> ar(indices, 3);
    Value * ptr = b->CreateGEP(mCurrentFrame, ar);
    assert ("summary type does not match defined type in frame" && ptr->getType()->getPointerElementType() == summary->getType());
    b->CreateStore(summary, ptr);
}


void CompressedCarryManager::addToCarryOutSummary(BuilderRef b, Value * const value) {
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


Value * CompressedCarryManager::convertFrameToImplicitSummary(BuilderRef b) const {
    Value * const ptr = convertFrameToImplicitSummaryPtr(b);
    Value * const summary = b->CreateAlignedLoad(ptr, 1);
    return summary;
}


Value * CompressedCarryManager::convertFrameToImplicitSummaryPtr(BuilderRef b) const {
    assert (mCarryInfo->hasSummary() && mCarryInfo->hasImplicitSummary());

    Type * const frameTy = mCurrentFrame->getType()->getPointerElementType();
    assert (frameTy->isStructTy());
    auto DL = b->getModule()->getDataLayout();
    auto frameSize = DL.getStructLayout(cast<StructType>(frameTy))->getSizeInBits();
    assert (frameSize == 64 || frameSize == b->getBitBlockWidth());

    Type * const summaryTy = frameSize == 64 ? (Type *) b->getInt64Ty() : b->getBitBlockType();
    return b->CreatePointerCast(mCurrentFrame, summaryTy->getPointerTo());
}


Value * CompressedCarryManager::loadImplicitSummaryFromPtr(BuilderRef b, Value * ptr) const {
    assert (ptr->getType()->isPointerTy());
    return b->CreateAlignedLoad(ptr, 1);
}


Type * CompressedCarryManager::getSummaryTypeFromCurrentFrame() const {
    assert (mCurrentFrame->getType()->isPointerTy());
    assert (mCurrentFrame->getType()->getPointerElementType()->isStructTy());
    return mCurrentFrame->getType()->getPointerElementType()->getStructElementType(0)->getArrayElementType();
}


/* ==== Scope Analyse ===== */


StructType * CompressedCarryManager::analyse(BuilderRef b,
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
