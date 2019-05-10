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
#include <pablo/pe_advance.h>
#include <pablo/pe_scanthru.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_var.h>
#include <kernels/kernel_builder.h>
#include <toolchain/toolchain.h>
#include <array>

#define LONG_ADVANCE_BREAKPOINT 64

#define NON_ADVANCE_CARRY_STMT(stmt) \
    isa<CarryProducingStatement>(stmt) && !isa<Advance>(stmt) && !isa<IndexedAdvance>(stmt)

using namespace llvm;

namespace pablo {

static inline bool isNonRegularLanguage(const PabloBlock * const scope) {
    if (const Branch * br = scope->getBranch()) {
        return !br->isRegular();
    }
    return false;
}

inline static bool isDynamicallyAllocatedType(const Type * const ty) {
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


void CompressedCarryManager::releaseCarryData(const std::unique_ptr<kernel::KernelBuilder> & idb) {
    CarryManager::releaseCarryData(idb);
    // errs() << "# Implicit Summaries: " << mNumImplicit << "\n"
    //        << "# Explicit Summaries: " << mNumExplicit << "\n"
    //        << "# Empty Carry State : " << mEmptyCarryState << "\n\n";
    mNumImplicit = 0;
    mNumExplicit = 0;
}


/* ===== Scope Changes ===== */


void CompressedCarryManager::enterLoopBody(const std::unique_ptr<kernel::KernelBuilder> & b, BasicBlock * const entryBlock) {
    if (mCarryInfo->hasSummary()) {
        Type * const carryTy = b->getBitBlockType();
        PHINode * phiCarryOutSummary = b->CreatePHI(carryTy, 2, "summary");
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
        assert ("non-carry collapsing mode is not supported yet" && false);
    }
}


void CompressedCarryManager::leaveLoopBody(const std::unique_ptr<kernel::KernelBuilder> & b, BasicBlock * /* exitBlock */) {

    Type * const carryTy = b->getBitBlockType();

    if (LLVM_UNLIKELY(mCarryInfo->nonCarryCollapsingMode())) {
        assert ("non-carry collapsing mode is not supported yet" && false);
    }
    if (mCarryInfo->hasSummary()) {
        const auto n = mCarrySummaryStack.size(); assert (n > 1);
        Value * carryOut = nullptr;
        if (mCarryInfo->hasImplicitSummary()) {
            carryOut = convertFrameToImplicitSummary(b);
            if (carryOut->getType() != carryTy)
                carryOut = b->CreateBitCast(b->CreateZExt(carryOut, b->getIntNTy(b->getBitBlockWidth())), carryTy);
        } else {
            carryOut = mCarrySummaryStack.back();
            mCarrySummaryStack.pop_back();
        }
        PHINode * phiCarryOut = cast<PHINode>(mCarrySummaryStack.back());
        phiCarryOut->addIncoming(carryOut, b->GetInsertBlock());
        // If we're returning to the base scope, reset our accumulated summary value.
        if (n == 2) {
            carryOut = Constant::getNullValue(carryTy);
        }
        mCarrySummaryStack.back() = carryOut;
    }
}


void CompressedCarryManager::leaveIfBody(const std::unique_ptr<kernel::KernelBuilder> & b, BasicBlock * const exitBlock) {
    assert (exitBlock);
    const auto n = mCarrySummaryStack.size();
    if (LLVM_LIKELY(mCarryInfo->hasExplicitSummary())) {
        writeCarryOutSummary(b, mCarrySummaryStack[n - 1], b->getInt32(0));
    }
    if (n > 2) {
        if (mCarryInfo->hasImplicitSummary()) {
            Value * summary = convertFrameToImplicitSummary(b);
            if (summary->getType() != b->getBitBlockType())
                summary = b->CreateBitCast(b->CreateZExt(summary, b->getIntNTy(b->getBitBlockWidth())), b->getBitBlockType());
            assert ("summary type does not match previous summary type" && summary->getType() == mCarrySummaryStack[n - 2]->getType());
            mCarrySummaryStack[n - 1] = b->CreateOr(summary, mCarrySummaryStack[n - 2], "summary");
        } else if (mCarryInfo->hasExplicitSummary()) {
            mCarrySummaryStack[n - 1] = b->CreateOr(mCarrySummaryStack[n - 1], mCarrySummaryStack[n - 2], "summary");
        }
    }
}


void CompressedCarryManager::leaveScope(const std::unique_ptr<kernel::KernelBuilder> & /* b */) {

    // Did we use all of the packs in this carry struct?
    if (!mCarryInfo->hasImplicitSummary())
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


/* ===== Operations ===== */


Value * CompressedCarryManager::addCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & b, const Statement * const operation, Value * const e1, Value * const e2) {
    assert (operation && (NON_ADVANCE_CARRY_STMT(operation)));
    Value * carryIn = getNextCarryIn(b);
    assert(carryIn->getType() == b->getInt8Ty());
    carryIn = b->CreateBitCast(b->CreateZExt(carryIn, b->getIntNTy(b->getBitBlockWidth())), b->getBitBlockType());
    Value * carryOut, * result;
    std::tie(carryOut, result) = b->bitblock_add_with_carry(e1, e2, carryIn);
    carryOut = b->mvmd_extract(8, carryOut, 0);
    assert (result->getType() == b->getBitBlockType());
    assert (carryOut->getType() == b->getInt8Ty());
    setNextCarryOut(b, carryOut);
    return result;
}


Value * CompressedCarryManager::advanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & b, const Advance * const advance, Value * const value) {
    const auto shiftAmount = advance->getAmount();
    if (LLVM_LIKELY(shiftAmount < LONG_ADVANCE_BREAKPOINT)) {
        Value * carryIn = getNextCarryIn(b);
        carryIn = b->CreateBitCast(b->CreateZExt(carryIn, b->getIntNTy(b->getBitBlockWidth())), b->getBitBlockType());
        Value * carryOut, * result;
        std::tie(carryOut, result) = b->bitblock_advance(value, carryIn, shiftAmount);
        const uint32_t fw = shiftAmount < 8 ? 8 : 64;
        carryOut = b->mvmd_extract(fw, carryOut, 0);
        assert (result->getType() == b->getBitBlockType());
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


/* ===== Single Carry Get/Set ===== */


Value * CompressedCarryManager::getNextCarryIn(const std::unique_ptr<kernel::KernelBuilder> & b) {
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
    Type * const carryTy = mCarryPackPtr->getType()->getPointerElementType();
    Value * const carryIn = b->CreateLoad(mCarryPackPtr);
    if (mLoopDepth > 0) {
        b->CreateStore(Constant::getNullValue(carryTy), mCarryPackPtr);
    }
    return carryIn;
}


void CompressedCarryManager::setNextCarryOut(const std::unique_ptr<kernel::KernelBuilder> & b, Value * carryOut) {
    assert (mCurrentFrameIndex < mCurrentFrame->getType()->getPointerElementType()->getStructNumElements());
    Type * const carryTy = mCarryPackPtr->getType()->getPointerElementType();
    assert("carry out type does not match carry location type" && carryOut->getType() == carryTy);
    if (mCarryInfo->hasSummary()) {
        addToCarryOutSummary(b, carryOut);
    }
    if (mLoopDepth != 0) {
        mCarryPackPtr = b->CreateGEP(mCurrentFrame, {b->getInt32(0), b->getInt32(mCurrentFrameIndex), mNextLoopSelector});
        assert("carry out type does not match carry location type" && carryOut->getType() == carryTy);
        if (LLVM_LIKELY(!mCarryInfo->nonCarryCollapsingMode())) {
            Value * accum = b->CreateLoad(mCarryPackPtr);
            carryOut = b->CreateOr(carryOut, accum);
        }
    }
    ++mCurrentFrameIndex;
    b->CreateStore(carryOut, mCarryPackPtr);
}


/* ===== Summary Operations ===== */


Value * CompressedCarryManager::readCarryInSummary(const std::unique_ptr<kernel::KernelBuilder> & b, ConstantInt * index) const {
    assert (mCarryInfo->hasSummary());

    if (LLVM_LIKELY(mCarryInfo->hasImplicitSummary())) {
        Value * summary = convertFrameToImplicitSummary(b);
        if (summary->getType() != b->getBitBlockType())
            summary = b->CreateBitCast(b->CreateZExt(summary, b->getIntNTy(b->getBitBlockWidth())), b->getBitBlockType());
        return summary;
    } else {
        assert (mCarryInfo->hasExplicitSummary());
        Constant * const ZERO = b->getInt32(0);
        Value * indices[3] = { ZERO, index, (mLoopDepth == 0 ? ZERO : mLoopSelector) };
        Value * const ptr = b->CreateGEP(mCurrentFrame, ArrayRef<Value *>(indices, 3));
        assert(ptr->getType()->getPointerElementType() == b->getBitBlockType());
        Value * const summary = b->CreateBlockAlignedLoad(ptr);
        if (mLoopDepth != 0) {
            b->CreateBlockAlignedStore(Constant::getNullValue(b->getBitBlockType()), ptr);
        }
        return summary;
    }
}


void CompressedCarryManager::writeCarryOutSummary(const std::unique_ptr<kernel::KernelBuilder> & b, Value * const summary, ConstantInt * index) const {
    assert (mCarryInfo->hasExplicitSummary());
    Constant * const ZERO = b->getInt32(0);
    Value * indices[3];
    indices[0] = ZERO;
    indices[1] = index;
    if (mLoopDepth == 0) {
        indices[2] = ZERO;
    } else {
        indices[2] = mNextLoopSelector;
    }
    ArrayRef<Value *> ar(indices, 3);
    Value * ptr = b->CreateGEP(mCurrentFrame, ar);
    assert (ptr->getType()->getPointerElementType() == b->getBitBlockType());
    b->CreateBlockAlignedStore(summary, ptr);
}


void CompressedCarryManager::addToCarryOutSummary(const std::unique_ptr<kernel::KernelBuilder> & b, Value * const value) {
    assert (mCarryInfo->hasSummary());

    // No need to add to summary if using an implicit summary
    if (mCarryInfo->hasExplicitSummary()) {
        assert ("cannot add null summary value!" && value);
        assert ("summary stack is empty!" && !mCarrySummaryStack.empty());
        Value * & summary = mCarrySummaryStack.back();
        assert("explicit summary must be of bitblock type" && summary->getType() == b->getBitBlockType());
        Value * carryOut = value;
        if (carryOut->getType() != b->getBitBlockType())
            carryOut = b->CreateBitCast(b->CreateZExt(carryOut, b->getIntNTy(b->getBitBlockWidth())), b->getBitBlockType());
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

    Type * const summaryTy = frameTy->getStructNumElements() == 8 ? (Type *) b->getInt64Ty() : b->getBitBlockType();
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
    assert ("non carry colapsing mode is not yet supported by this carry manager" && !nonCarryCollapsingMode);

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
        } else if (LLVM_UNLIKELY(NON_ADVANCE_CARRY_STMT(stmt))) {
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
        mEmptyCarryState++;
    } else {
        if (hasNonEmptyCarryStruct(state)) {
            if (dyn_cast_or_null<If>(scope->getBranch()) || nonCarryCollapsingMode || isNestedWithinNonCarryCollapsingLoop) {
                if (LLVM_LIKELY(canUseImplicitSummary)) {
                    summaryType = CarryData::ImplicitSummary;

                    // If needed, pad the structure to 64 bits or the bitblock width
                    // This allows us to bitcast the structure to an i64 or bitblock to get the summary
                    std::size_t structBitWidth = state.size() * 8; // all carries are 8 bits
                    std::size_t const width = structBitWidth;
                    assert(structBitWidth <= blockWidth);
                    const std::size_t targetWidth = structBitWidth <= 64 ? 64 : blockWidth;
                    while (structBitWidth < targetWidth) {
                        state.push_back(i8Ty);
                        structBitWidth += 8;
                    }

                    mNumImplicit++;
                } else {
                    summaryType = CarryData::ExplicitSummary;
                    // NOTE: summaries are stored differently depending whether we're entering an If or While branch. With an If branch, they
                    // preceed the carry state data and with a While loop they succeed it. This is to help cache prefectching performance.
                    state.insert(isa<If>(scope->getBranch()) ? state.begin() : state.end(), blockPackTy);

                    mNumExplicit++;
                }
            }
        }

        
        carryStruct = StructType::get(b->getContext(), state);
        auto DL = b->getModule()->getDataLayout();
        std::string summaryTypeString;
        if (summaryType == CarryData::ImplicitSummary)
            summaryTypeString = "Implicit Summary";
        else if (summaryType == CarryData::ExplicitSummary)
            summaryTypeString = "Explicit Summary";
        else if (summaryType == CarryData::NoSummary)
            summaryTypeString = "No Summary";
        
        // if (summaryType == CarryData::ImplicitSummary) {
            // errs() << summaryTypeString << "\n"
            //        << *carryStruct << "\n"
            //        << "struct size in bits: " << DL.getStructLayout(carryStruct)->getSizeInBits() << "\n"
            //        << "struct alignment   : " << DL.getStructLayout(carryStruct)->getAlignment() << "\n\n";
        // }
        // If we're in a loop and cannot use collapsing carry mode, convert the carry state struct into a capacity,
        // carry state pointer, and summary pointer struct.
        if (LLVM_UNLIKELY(nonCarryCollapsingMode)) {
            mHasNonCarryCollapsingLoops = true;
            carryStruct = StructType::get(b->getContext(), {b->getSizeTy(), carryStruct->getPointerTo(), blockPackTy->getPointerTo()});
            assert (isDynamicallyAllocatedType(carryStruct));
        }
        cd.setNonCollapsingCarryMode(nonCarryCollapsingMode);
    }

    cd.setSummaryType(summaryType);
    return carryStruct;
}

CompressedCarryManager::CompressedCarryManager() noexcept
: CarryManager()
{}

}
