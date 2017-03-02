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
    assert (mCarrySummary.empty());

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
    assert (mCarrySummary.empty());
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
        PHINode * carrySummary = iBuilder->CreatePHI(mCarryPackType, 2, "summary");
        assert (!mCarrySummary.empty());
        carrySummary->addIncoming(mCarrySummary.back(), entryBlock);
        // Replace the incoming carry summary with the phi node and add the phi node to the stack
        // so that we can properly OR it into the outgoing summary value.
        mCarrySummary.back() = carrySummary;
        mCarrySummary.push_back(carrySummary);
    }

    if (LLVM_UNLIKELY(mCarryInfo->nonCarryCollapsingMode())) {
        // Check whether we need to resize the carry state
        PHINode * index = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
        mLoopIndicies.push_back(index);
        index->addIncoming(iBuilder->getSize(0), entryBlock);
        Value * capacityPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
        Value * capacity = iBuilder->CreateLoad(capacityPtr, false, "capacity");
        Value * arrayPtr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(1)});
        Value * array = iBuilder->CreateLoad(arrayPtr, false, "array");

        BasicBlock * entry = iBuilder->GetInsertBlock();
        BasicBlock * resizeBlock = mKernel->CreateBasicBlock("");
        BasicBlock * cleanUpBlock = mKernel->CreateBasicBlock("");
        BasicBlock * zeroBlock = mKernel->CreateBasicBlock("");
        BasicBlock * codeBlock = mKernel->CreateBasicBlock("");

        Value * cond = iBuilder->CreateICmpULT(index, capacity);
        iBuilder->CreateCondBr(cond, codeBlock, resizeBlock);
        iBuilder->SetInsertPoint(resizeBlock);

        Type * const carryStateType = array->getType()->getPointerElementType();

        Constant * carryStateWidth = ConstantExpr::getIntegerCast(ConstantExpr::getSizeOf(carryStateType), capacity->getType(), false);
        Value * newCapacity = iBuilder->CreateSelect(iBuilder->CreateICmpNE(index, ConstantInt::getNullValue(index->getType())), iBuilder->CreateMul(index, iBuilder->getSize(2)), iBuilder->getSize(4));
        Value * newArray = iBuilder->CreateAlignedMalloc(carryStateType, newCapacity, iBuilder->getCacheAlignment());

        assert (newCapacity->getType() == capacity->getType());
        assert (newArray->getType() == array->getType());

        Value * isNullCarryState = iBuilder->CreateICmpEQ(array, ConstantPointerNull::get(cast<PointerType>(array->getType())));

        iBuilder->CreateCondBr(isNullCarryState, zeroBlock, cleanUpBlock);
        iBuilder->SetInsertPoint(cleanUpBlock);

        iBuilder->CreateMemCpy(newArray, array, iBuilder->CreateMul(capacity, carryStateWidth), iBuilder->getCacheAlignment());
        iBuilder->CreateAlignedFree(array);
        iBuilder->CreateBr(zeroBlock);

        iBuilder->SetInsertPoint(zeroBlock);

        iBuilder->CreateMemZero(iBuilder->CreateGEP(newArray, capacity), iBuilder->CreateMul(iBuilder->CreateSub(newCapacity, capacity), carryStateWidth), iBuilder->getCacheAlignment());
        iBuilder->CreateStore(newCapacity, capacityPtr);
        iBuilder->CreateStore(newArray, arrayPtr);

        iBuilder->CreateBr(codeBlock);

        // Load the appropriate carry stat block
        iBuilder->SetInsertPoint(codeBlock);

        PHINode * phiArray = iBuilder->CreatePHI(array->getType(), 2);
        phiArray->addIncoming(array, entry);
        phiArray->addIncoming(newArray, zeroBlock);

        mCurrentFrame = iBuilder->CreateGEP(phiArray, index);

    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveLoopBody
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveLoopBody(BasicBlock * const exitBlock) {
    if (mCarryInfo->hasSummary()) {
        const auto n = mCarrySummary.size(); assert (n > 1);
        cast<PHINode>(mCarrySummary[n - 2])->addIncoming(mCarrySummary[n - 1], exitBlock);
        mCarrySummary.pop_back();
    }
    if (LLVM_UNLIKELY(mCarryInfo->nonCarryCollapsingMode())) {
        assert (!mLoopIndicies.empty());
        PHINode * index = mLoopIndicies.back();
        index->addIncoming(iBuilder->CreateAdd(index, iBuilder->getSize(1)), exitBlock);
        mLoopIndicies.pop_back();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveLoopScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveLoopScope(BasicBlock * const entryBlock, BasicBlock * const exitBlock) {
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
    mCarrySummary.push_back(Constant::getNullValue(mCarryPackType));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateSummaryTest
 ** ------------------------------------------------------------------------------------------------------------- */
Value * CarryManager::generateSummaryTest(Value * condition) {
    if (LLVM_LIKELY(mCarryInfo->hasSummary())) {
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
        Value * summary = iBuilder->CreateBlockAlignedLoad(ptr);
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
    const auto n = mCarrySummary.size();
    if (LLVM_LIKELY(mCarryInfo->hasExplicitSummary())) {
        assert (!mCarrySummary.empty());
        Value * ptr = iBuilder->CreateGEP(mCurrentFrame, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
        Value * const value = iBuilder->CreateBitCast(mCarrySummary.back(), mBitBlockType);
        iBuilder->CreateBlockAlignedStore(value, ptr);
    }
    if (n > 1) {
        mCarrySummary[n - 1] = iBuilder->CreateOr(mCarrySummary[n - 1], mCarrySummary[n - 2], "summary");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief leaveIfScope
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::leaveIfScope(BasicBlock * const entryBlock, BasicBlock * const exitBlock) {
    assert (mIfDepth > 0);
    if (mCarryInfo->hasSummary()) {
        const auto n = mCarrySummary.size(); assert (n > 0);
        if (n > 1) {
            // When leaving a nested If scope with a summary value, phi out the summary to ensure the
            // appropriate summary is stored in the outer scope.
            Value * nested = mCarrySummary[n - 1];
            Value * outer = mCarrySummary[n - 2];
            if (LLVM_LIKELY(nested != outer)) {
                assert (nested->getType() == outer->getType());
                PHINode * const phi = iBuilder->CreatePHI(nested->getType(), 2, "summary");
                phi->addIncoming(outer, entryBlock);
                phi->addIncoming(nested, exitBlock);
                mCarrySummary[n - 2] = phi;
            }
        }        
    }
    --mIfDepth;
    leaveScope();
    mCarrySummary.pop_back();
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
    if (mLoopDepth > 0) {
        iBuilder->CreateBlockAlignedStore(Constant::getNullValue(mCarryPackType), carryInPtr);
    }
    return carryIn;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setNextCarryOut
 ** ------------------------------------------------------------------------------------------------------------- */
void CarryManager::setNextCarryOut(Value * carryOut) {
    if (LLVM_LIKELY(mCarryInfo->hasExplicitSummary())) {        
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
    assert (mIfDepth > 0 && !mCarrySummary.empty());
    Value * const summary = mCarrySummary.back(); assert (summary);
    if (LLVM_UNLIKELY(summary == value)) {
        return;  //Nothing to add.
    }
    value = iBuilder->CreateBitCast(value, mCarryPackType);
    if (LLVM_UNLIKELY(isa<Constant>(value))) {
        if (LLVM_UNLIKELY(cast<Constant>(value)->isZeroValue())) {
            return;
        } else if (LLVM_UNLIKELY(cast<Constant>(value)->isAllOnesValue())) {
            mCarrySummary.back() = value;
            return;
        }
    } else if (LLVM_UNLIKELY(isa<Constant>(summary))) {
        if (LLVM_UNLIKELY(cast<Constant>(summary)->isZeroValue())) {
            mCarrySummary.back() = value;
            return;
        } else if (LLVM_UNLIKELY(cast<Constant>(summary)->isAllOnesValue())) {
            return;
        }
    }    
    mCarrySummary.back() = iBuilder->CreateOr(summary, value);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief collapsingCarryMode
 ** ------------------------------------------------------------------------------------------------------------- */
bool CarryManager::inCollapsingCarryMode() const {
    return (mCurrentScope->getBranch() && isa<While>(mCurrentScope->getBranch()) && !mCarryInfo->nonCarryCollapsingMode());
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
        cd.setNonCarryCollapsingMode(loopDepth > 0 && hasIterationSpecificAssignment(scope));
        if (ifDepth > 0) {
            // A non-collapsing loop requires a unique summary for each iteration. Thus whenever
            // we have a non-collapsing While within an If scope with an implicit summary, the If
            // scope requires an explicit summary.
            if (cd.nonCarryCollapsingMode() || isa<If>(scope->getBranch())) {
                if (LLVM_LIKELY(state.size() > 1 || hasLongAdvances)) {
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
        // If we in a loop and cannot use collapsing carry mode, convert the struct into a capacity and pointer pair.
        if (LLVM_UNLIKELY(cd.nonCarryCollapsingMode())) {
            mHasVariableLengthCarryData = true;
            carryState = StructType::get(iBuilder->getSizeTy(), carryState->getPointerTo(), nullptr);
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
, mHasVariableLengthCarryData(false)
, mHasLoop(false)
, mLoopDepth(0)
, mLoopSelector(nullptr)
, mCarryScopes(0) {

}

}

