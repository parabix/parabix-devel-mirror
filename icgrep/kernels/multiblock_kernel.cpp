/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include <kernels/kernel.h>

#include <toolchain/toolchain.h>
#include <kernels/streamset.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/MDBuilder.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(4, 0, 0)
#include <llvm/Bitcode/ReaderWriter.h>
#else
#include <llvm/Bitcode/BitcodeWriter.h>
#endif
#include <llvm/Transforms/Utils/Local.h>
#include <kernels/streamset.h>
#include <sstream>
#include <kernels/kernel_builder.h>
#include <boost/math/common_factor.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <llvm/Support/Debug.h>

using namespace llvm;
using namespace parabix;
using namespace boost;
using namespace boost::math;

// #define DEBUG_LOG

#define POPCOUNT_RATE_MAX_STRIDES_PER_ROUND (31)

namespace kernel {

using RateValue = ProcessingRate::RateValue;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiBlockKernel::generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) {

    if (LLVM_UNLIKELY((mStride % b->getBitBlockWidth()) != 0)) {
        report_fatal_error(getName() + ": the Stride (" + std::to_string(mStride) + ") of MultiBlockKernel "
                           "must be a multiple of the BitBlockWidth (" + std::to_string(b->getBitBlockWidth()) + ")");
    }

    const auto inputSetCount = mStreamSetInputs.size();
    const auto outputSetCount = mStreamSetOutputs.size();

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * terminatedTwice = b->CreateAnd(mIsFinal, b->getTerminationSignal());
        Value * unprocessedData = nullptr;
        for (unsigned i = 0; i < inputSetCount; i++) {
            Value * processed = b->getProcessedItemCount(mStreamSetInputs[i].getName());
            Value * const check = b->CreateICmpNE(processed, mAvailableItemCount[i]);
            unprocessedData = unprocessedData ? b->CreateOr(unprocessedData, check) : check;
        }
        b->CreateAssertZero(b->CreateAnd(terminatedTwice, unprocessedData),
                            getName() + " was called after its termination with additional input data");
        b->CreateAssertZero(terminatedTwice,
                            getName() + " was called after its termination");
    }

    mInitialAvailableItemCount.assign(mAvailableItemCount.begin(), mAvailableItemCount.end());
    mInitialProcessedItemCount.resize(inputSetCount, nullptr);
    mAccessibleInputItems.resize(inputSetCount, nullptr);
    mInputStrideLength.resize(inputSetCount, nullptr);
    mPopCountRateArray.resize(inputSetCount, nullptr);

    mInitialProducedItemCount.resize(outputSetCount, nullptr);
    mWritableOutputItems.resize(outputSetCount, nullptr);
    mOutputStrideLength.resize(outputSetCount, nullptr);

    mInitiallyFinal = mIsFinal;
    #ifdef DEBUG_LOG
    for (unsigned i = 0; i < inputSetCount; ++i) {
        b->CallPrintInt(getName() + "_" + mStreamSetInputs[i].getName() + "_available", mAvailableItemCount[i]);
    }
    b->CallPrintInt(getName() + "_initiallyFinal", mInitiallyFinal);
    #endif
    mTreatUnsafeKernelOperationsAsErrors = true;

    // Now proceed with creation of the doSegment method.
    BasicBlock * const segmentLoop = b->CreateBasicBlock("SegmentLoop");
    b->CreateBr(segmentLoop);

    b->SetInsertPoint(segmentLoop);
    writeMultiBlockLogic(b);
    writeCopyBackLogic(b);
    BasicBlock * const handleFinalBlock = b->CreateBasicBlock("FinalBlock");
    BasicBlock * const strideDone = b->CreateBasicBlock("StridesDone");
    b->CreateUnlikelyCondBr(mIsFinal, handleFinalBlock, strideDone);


    /// FINAL STRIDE ADJUSTMENT
    handleFinalBlock->moveAfter(b->GetInsertBlock());
    b->SetInsertPoint(handleFinalBlock);
    updateFinalDerivedItemCounts(b);
    b->setTerminationSignal(b->getTrue());
    BasicBlock * const segmentDone = b->CreateBasicBlock("SegmentDone");
    b->CreateBr(segmentDone);

    /// CHECK FOR ANOTHER STRIDE
    strideDone->moveAfter(b->GetInsertBlock());
    b->SetInsertPoint(strideDone);
    // TODO: don't bother computing this for block oriented kernels
    updateDerivedItemCounts(b);
    b->CreateCondBr(hasAnotherStride(b), segmentLoop, segmentDone);

    /// SEGMENT DONE
    segmentDone->moveAfter(b->GetInsertBlock());
    b->SetInsertPoint(segmentDone);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeMultiBlock
 ** ------------------------------------------------------------------------------------------------------------- */
inline void MultiBlockKernel::writeMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b) {
    mNumOfStrides = nullptr;
    mNumOfStridesInFinalSegment = b->getSize(1);
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        checkInputStream(b, i);
    }
    mIsFinal = b->CreateICmpEQ(mNumOfStrides, b->getSize(0));
	mIsFinal = b->CreateAnd(mIsFinal, mInitiallyFinal);
    mNumOfStrides = b->CreateSelect(mIsFinal, mNumOfStridesInFinalSegment, mNumOfStrides);
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        checkOutputStream(b, i);
    }
    #ifdef DEBUG_LOG
    b->CallPrintInt(getName() + "_isFinal", mIsFinal);
    b->CallPrintInt(getName() + "_numOfStrides", mNumOfStrides);
    #endif
    calculateDerivedItemCounts(b);
    prepareOverflowBuffers(b);
    generateMultiBlockLogic(b, mNumOfStrides);
    checkTerminationSignal(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkInputStream
 ** ------------------------------------------------------------------------------------------------------------- */
inline void MultiBlockKernel::checkInputStream(const std::unique_ptr<KernelBuilder> & b, const unsigned index) {
    const Binding & input = mStreamSetInputs[index];
    const auto & name = input.getName();
    Value * const processed = b->getNonDeferredProcessedItemCount(input);
    mInitialProcessedItemCount[index] = processed;
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        b->CreateAssert(b->CreateICmpULE(processed, mAvailableItemCount[index]),
                        getName() + ": " + name + " processed item count exceeds available item count");
    }
    Value * accessible = b->CreateSub(mAvailableItemCount[index], processed);
    if (LLVM_UNLIKELY(requiresLinearAccess(input))) {
        accessible = b->getLinearlyAccessibleItems(name, processed, accessible);
    }
    // NOTE: mAccessibleInputItems must initially reflect the # of items PRIOR to any lookahead.
    // This is so that we can use this number directly for any final stride calculations.
    mAccessibleInputItems[index] = accessible;
    if (input.hasLookahead()) {
        Constant * const lookahead = b->getSize(input.getLookahead());
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            b->CreateAssert(b->CreateICmpULE(lookahead, accessible),
                            getName() + ": " + name + " lookahead exceeds accessible item count");
        }
        accessible = b->CreateSub(accessible, lookahead);
    }
    Value * accessibleStrides = nullptr;
    const ProcessingRate & rate = input.getRate();
    if (LLVM_UNLIKELY(rate.isPopCount() || rate.isNegatedPopCount())) {
        accessibleStrides = computePopCountRate(b, rate, accessible);
    } else {
        mInputStrideLength[index] = b->getSize(ceiling(getUpperBound(rate) * mStride));
        accessibleStrides = b->CreateUDiv(accessible, mInputStrideLength[index]);
    }
    assert ("input cannot have an unknown number of strides" && accessibleStrides);
    #ifdef DEBUG_LOG
    b->CallPrintInt(getName() + "_" + name + "_availableStrides", accessibleStrides);
    #endif
    mNumOfStrides = b->CreateUMin(mNumOfStrides, accessibleStrides);
    if (LLVM_UNLIKELY(input.hasAttribute(Attribute::KindId::Principal))) {
        if (rate.isFixed()) {
            accessibleStrides = b->CreateCeilUDiv(accessible, mInputStrideLength[index]);
        }
        mNumOfStridesInFinalSegment = b->CreateUMax(accessibleStrides, mNumOfStridesInFinalSegment);
    }
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const hasData = b->CreateIsNotNull(accessibleStrides);
        Value * const hasStride = b->CreateOr(mInitiallyFinal, hasData);
        b->CreateAssert(hasStride, getName() + ": " + name + " has insufficient input data for one stride");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkOutputStream
 ** ------------------------------------------------------------------------------------------------------------- */
inline void MultiBlockKernel::checkOutputStream(const std::unique_ptr<KernelBuilder> & b, const unsigned index) {
    const Binding & output = mStreamSetOutputs[index];
    const auto & name = output.getName();
    Value * const produced = b->getNonDeferredProducedItemCount(output);
    mInitialProducedItemCount[index] = produced;
    Value * writable = nullptr;
    if (LLVM_UNLIKELY(permitsNonLinearAccess(output))) {
        Value * const consumed = b->getConsumedItemCount(name);
        Value * const unconsumed = b->CreateSub(produced, consumed);
        Value * const capacity = b->getCapacity(name);
        writable = b->CreateSub(capacity, unconsumed);
    } else {
        writable = b->getLinearlyWritableItems(name, produced);
    }
    Value * writableStrides = nullptr;
    const ProcessingRate & rate = output.getRate();
    if (LLVM_UNLIKELY(rate.isPopCount() || rate.isNegatedPopCount())) {
        writableStrides = computePopCountRate(b, rate, writable);
    } else {
        mOutputStrideLength[index] = b->getSize(ceiling(getUpperBound(rate) * getStride()));
        writableStrides = b->CreateUDiv(writable, mOutputStrideLength[index]);
    }
    assert ("output cannot have an unknown number of strides" && writableStrides);
    #ifdef DEBUG_LOG
    b->CallPrintInt(getName() + "_" + name + "_writableStrides", writableStrides);
    #endif
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const hasSpace = b->CreateIsNotNull(writableStrides);
        b->CreateAssert(hasSpace, getName() + ": " + name + " has insufficient output space for one stride");
    }
    mNumOfStrides = b->CreateUMin(mNumOfStrides, writableStrides);
    mWritableOutputItems[index] = writable;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computePopCountRate
 ** ------------------------------------------------------------------------------------------------------------- */
Value * MultiBlockKernel::computePopCountRate(const std::unique_ptr<KernelBuilder> & b, const ProcessingRate & rate, Value * const maxItems) {

    // To auto increment a popcount rate stream, we need to store how many positions could be incremented per
    // stride index since our final numOfStrides may be smaller than the current one.

    Port refPort;
    unsigned refIndex = 0;
    std::tie(refPort, refIndex) = getStreamPort(rate.getReference());
    assert (refPort == Port::Input);

    if (mPopCountRateArray[refIndex]) {
        // TODO: if we already have a popcount rate array for this reference stream, reuse it.

        // NOTE: the maxItems of the first stream could be less than this one. To do this safely, the kernel must
        // first check what the largest possible maxItem count is when initially creating the popcount stream

        assert (false);
        return nullptr;

    } else {

        const Binding & ref = mStreamSetInputs[refIndex];

        // TODO: we can use 16-bit integers instead here but would need to make it safe for > 1024-bit block width

        Constant * const ONE = b->getSize(1);
        Constant * const MAX = b->getSize(POPCOUNT_RATE_MAX_STRIDES_PER_ROUND);
        Value * const maxNumOfStrides = b->CreateUMin(mNumOfStrides, MAX);
        Constant * const ARRAY_SIZE = b->getSize(POPCOUNT_RATE_MAX_STRIDES_PER_ROUND + 1);
        AllocaInst * const partialSumArray = b->CreateAlloca(b->getSizeTy(), ARRAY_SIZE);

        BasicBlock * const entry = b->GetInsertBlock();
        BasicBlock * const popCountLoop = b->CreateBasicBlock();
        BasicBlock * const popCountAppend = b->CreateBasicBlock();
        BasicBlock * const popCountExit = b->CreateBasicBlock();

        Constant * const ZERO = b->getSize(0);
        b->CreateBr(popCountLoop);

        b->SetInsertPoint(popCountLoop);
        PHINode * const strideIndex = b->CreatePHI(b->getSizeTy(), 2);
        strideIndex->addIncoming(ZERO, entry);
        PHINode * const partialSum = b->CreatePHI(b->getSizeTy(), 2);
        partialSum->addIncoming(ZERO, entry);
        // TODO: what if we have a stream set of more than one stream element? or swizzled input?
        Value * const ptr = b->getInputStreamBlockPtr(ref.getName(), ZERO, strideIndex);
        Value * markers = b->CreateBlockAlignedLoad(ptr);
        if (rate.isNegatedPopCount()) {
            markers = b->CreateNot(markers);
        }
        Value * const count = b->bitblock_popcount(markers);
        Value * const partialSum2 = b->CreateAdd(partialSum, count);
        Value * const ptr2 = b->CreateGEP(partialSumArray, strideIndex);
        b->CreateStore(partialSum2, ptr2);
        Value * const hasEnoughSourceItems = b->CreateICmpULE(partialSum2, maxItems);
        b->CreateCondBr(hasEnoughSourceItems, popCountAppend, popCountExit);

        b->SetInsertPoint(popCountAppend);
        partialSum->addIncoming(partialSum2, popCountAppend);
        Value * const nextStrideIndex = b->CreateAdd(strideIndex, ONE);
        Value * const hasMore = b->CreateICmpNE(nextStrideIndex, maxNumOfStrides);
        strideIndex->addIncoming(nextStrideIndex, popCountAppend);
        b->CreateCondBr(hasMore, popCountLoop, popCountExit);

        b->SetInsertPoint(popCountExit);
        PHINode * const strideCount = b->CreatePHI(b->getSizeTy(), 2);
        strideCount->addIncoming(strideIndex, popCountLoop);
        strideCount->addIncoming(nextStrideIndex, popCountAppend);
        // add in a sentinal to simplify the "hasAnotherStride" logic
        Value * const ptr3 = b->CreateGEP(partialSumArray, MAX);
        b->CreateStore(ConstantInt::getAllOnesValue(b->getSizeTy()), ptr3);

        mPopCountRateArray[refIndex] = partialSumArray;

        return strideCount;
    }

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeCopyBackLogic
 ** ------------------------------------------------------------------------------------------------------------- */
inline void MultiBlockKernel::writeCopyBackLogic(const std::unique_ptr<KernelBuilder> & b) {
    // Do copybacks if necessary.
    for (unsigned i = 0; i < mStreamSetOutputs.size(); ++i) {
        const Binding & output = mStreamSetOutputs[i];
        if (requiresCopyBack(output) && mStreamSetOutputBuffers[i]->supportsCopyBack()) {
            const auto & name = output.getName();
            BasicBlock * const copyBack = b->CreateBasicBlock(name + "CopyBack");
            BasicBlock * const done = b->CreateBasicBlock(name + "CopyBackDone");
            Value * const capacity = b->getCapacity(name);
            Value * const priorOffset = b->CreateURem(mInitialProducedItemCount[i], capacity);
            Value * const produced = b->getProducedItemCount(name);
            Value * const currentOffset = b->CreateURem(produced, capacity);
            b->CreateUnlikelyCondBr(b->CreateICmpULT(currentOffset, priorOffset), copyBack, done);

            b->SetInsertPoint(copyBack);
            Value * overflowOffset = nullptr;
            if (permitsNonLinearAccess(output)) {
                // if we can compute the overflowPosition, do so
                const ProcessingRate & rate = output.getRate();
                if (rate.isPopCount() || rate.isNegatedPopCount()) {
                    Value * const limit = b->CreateSub(capacity, priorOffset);
                    BasicBlock * const popCountLoop = b->CreateBasicBlock();
                    BasicBlock * const popCountDone = b->CreateBasicBlock();
                    b->CreateBr(popCountLoop);

                    b->SetInsertPoint(popCountLoop);
                    PHINode * const strideIndex = b->CreatePHI(b->getSizeTy(), 2);
                    strideIndex->addIncoming(b->getSize(0), copyBack);
                    Value * const accessible = getPopCountRateItems(b, rate, strideIndex);
                    Value * const writesToOverflow = b->CreateICmpULT(accessible, limit);
                    b->CreateCondBr(writesToOverflow, popCountDone, popCountLoop);

                    b->SetInsertPoint(popCountDone);
                    overflowOffset = b->CreateAdd(priorOffset, accessible);
                }
                b->CreateNonLinearCopyFromOverflow(output, currentOffset, overflowOffset);
            } else {
                b->CreateCopyFromOverflow(output, currentOffset);
            }

            b->CreateBr(done);

            b->SetInsertPoint(done);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasAnotherStride
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * MultiBlockKernel::hasAnotherStride(const std::unique_ptr<KernelBuilder> & b) {
    // if any binding requires linear space, then it's possible we didn't fully consume all of the input
    Value * hasMoreStrides = nullptr;
    if (LLVM_UNLIKELY(anyBindingRequiresLinearSpace())) {
        // do we have enough input data for another stride?
        hasMoreStrides = b->getTrue();
        for (unsigned i = 0; i < mStreamSetInputs.size(); ++i) {
            const Binding & input = mStreamSetInputs[i];
            const ProcessingRate & rate = input.getRate();
            const auto & name = input.getName();
            Value * const processed = b->getNonDeferredProcessedItemCount(input);
            Value * const avail = mInitialAvailableItemCount[i];
            if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
                b->CreateAssert(b->CreateICmpULE(processed, avail),
                                getName() + ": " + name + " processed data exceeds available data");
            }
            Value * required = nullptr;
            if (rate.isPopCount() || rate.isNegatedPopCount()) {
                required = b->CreateSub(getPopCountRateItems(b, rate, mNumOfStrides), mAccessibleInputItems[i]);
            } else {
                required = mInputStrideLength[i];
            }
            if (LLVM_UNLIKELY(input.hasLookahead())) {
                required = b->CreateAdd(required, b->getSize(input.getLookahead()));
            }
            Value * const remaining = b->CreateSub(avail, processed);
            Value * const hasRemainingStrides = b->CreateICmpUGE(remaining, required);
            hasMoreStrides = b->CreateAnd(hasMoreStrides, hasRemainingStrides);
        }
        // even if we do not have enough input data for a full stride but it is our final stride, allow it ...
        hasMoreStrides = b->CreateOr(hasMoreStrides, mInitiallyFinal);
    } else {
        hasMoreStrides = mInitiallyFinal;
    }
    // do we have enough output space for another stride?
    for (unsigned i = 0; i < mStreamSetOutputs.size(); ++i) {
        const Binding & output = mStreamSetOutputs[i];
        const ProcessingRate & rate = output.getRate();
        if (LLVM_UNLIKELY(rate.isUnknown())) {
            continue;
        }
        const auto & name = output.getName();
        Value * const produced = b->getNonDeferredProducedItemCount(output);
        Value * const consumed = b->getConsumedItemCount(name);
        Value * const unconsumed = b->CreateSub(produced, consumed);
        Value * const capacity = b->getCapacity(name);
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            b->CreateAssert(b->CreateICmpULE(consumed, produced),
                            getName() + ": " + name + " consumed data exceeds produced data");
            b->CreateAssert(b->CreateICmpULE(unconsumed, capacity),
                            getName() + ": " + name + " more data was written than its capacity allows");
        }
        // compute the upperbound of how much output space is needed to store another stride ...
        Value * required = nullptr;
        if (rate.isPopCount() || rate.isNegatedPopCount()) {
            required = b->CreateSub(getPopCountRateItems(b, rate, mNumOfStrides), mWritableOutputItems[i]);
        } else {
            required = mOutputStrideLength[i];
        }
        Value * const remaining = b->CreateSub(capacity, unconsumed);
        Value * const hasRemainingStrides = b->CreateICmpUGE(remaining, required);
        hasMoreStrides = b->CreateAnd(hasMoreStrides, hasRemainingStrides);
    }
    #ifdef DEBUG_LOG
    b->CallPrintInt(getName() + "_hasMoreStrides", hasMoreStrides);
    #endif
    return hasMoreStrides;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateDerivedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void MultiBlockKernel::calculateDerivedItemCounts(const std::unique_ptr<KernelBuilder> & b) {

    // Update the accessible and writable item count to reflect the # of strides
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        const Binding & input = mStreamSetInputs[i];
        const ProcessingRate & rate = input.getRate();
        if (rate.isFixed()) {
            Value * const processable = b->CreateMul(mInputStrideLength[i], mNumOfStrides);
            mAccessibleInputItems[i] = b->CreateSelect(mIsFinal, mAccessibleInputItems[i], processable);
        } else if (rate.isPopCount() || rate.isNegatedPopCount()) {
            Value * const strideIndex = b->CreateSub(mNumOfStrides, b->getSize(1));
            mAccessibleInputItems[i] = getPopCountRateItems(b, rate, strideIndex);
        }
        #ifdef DEBUG_LOG
        b->CallPrintInt(getName() + "_" + input.getName() + "_accessible", mAccessibleInputItems[i]);
        #endif
        // TODO: make it so the available item count is never changed; anything that relies on it should use the accessible item count.
        mAvailableItemCount[i] = mAccessibleInputItems[i];
    }

    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        const Binding & output = mStreamSetOutputs[i];
        const ProcessingRate & rate = output.getRate();
        if (rate.isFixed()) {
            Value * const producable = b->CreateMul(mOutputStrideLength[i], mNumOfStrides);
            mWritableOutputItems[i] = producable; // b->CreateSelect(mIsFinal, mWritableOutputItems[i], producable);
        } else if (rate.isPopCount() || rate.isNegatedPopCount()) {
            Value * const strideIndex = b->CreateSub(mNumOfStrides, b->getSize(1));
            mWritableOutputItems[i] = getPopCountRateItems(b, rate, strideIndex);
        }
        #ifdef DEBUG_LOG
        b->CallPrintInt(getName() + "_" + output.getName() + "_writable", mWritableOutputItems[i]);
        #endif
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateDerivedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void MultiBlockKernel::updateDerivedItemCounts(const std::unique_ptr<KernelBuilder> & b) {
    mTreatUnsafeKernelOperationsAsErrors = false;
    // Update the processed item counts
    for (unsigned i = 0; i < mStreamSetInputs.size(); ++i) {
        const Binding & input = mStreamSetInputs[i];        
        if (hasDerivedItemCount(input)) {
            Value * const processed = b->CreateAdd(mInitialProcessedItemCount[i], mAccessibleInputItems[i]);
            b->setNonDeferredProcessedItemCount(input, processed);
        }
        #ifdef DEBUG_LOG
        b->CallPrintInt(getName() + "_" + input.getName() + "_processed'", b->getProcessedItemCount(input.getName()));
        #endif
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            Value * const processed = b->getNonDeferredProcessedItemCount(input);
            Value * const newlyProcessed = b->CreateSub(processed, mInitialProcessedItemCount[i]);
            Value * const withinCapacity = b->CreateICmpULE(newlyProcessed, mAccessibleInputItems[i]);
            std::string tmp;
            raw_string_ostream out(tmp);
            out << getName() << ": \"" << input.getName() << "\" has processed more items than accessible";
            b->CreateAssert(withinCapacity, out.str());
        }
    }
    // Update the produced item counts
    for (unsigned i = 0; i < mStreamSetOutputs.size(); ++i) {
        const Binding & output = mStreamSetOutputs[i];
        if (hasDerivedItemCount(output)) {
            Value * const produced = b->CreateAdd(mInitialProducedItemCount[i], mWritableOutputItems[i]);
            b->setNonDeferredProducedItemCount(output, produced);
        }
        #ifdef DEBUG_LOG
        b->CallPrintInt(getName() + "_" + output.getName() + "_produced'", b->getProducedItemCount(output.getName()));
        #endif
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            Value * const produced = b->getProducedItemCount(output.getName());
            Value * const newlyProduced = b->CreateSub(produced, mInitialProducedItemCount[i]);
            Value * const withinCapacity = b->CreateICmpULE(newlyProduced, mWritableOutputItems[i]);
            std::string tmp;
            raw_string_ostream out(tmp);
            out << getName() << ": \"" << output.getName() << "\" has produced more items than writable";
            b->CreateAssert(withinCapacity, out.str());
        }
    }
    mTreatUnsafeKernelOperationsAsErrors = true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateFinalDerivedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void MultiBlockKernel::updateFinalDerivedItemCounts(const std::unique_ptr<KernelBuilder> & b) {

    ProcessingRate::RateValue rateLCM(1);

    bool noPrincipalStream = true;
    bool hasFixedRateInput = false;

    for (const Binding & input : mStreamSetInputs) {
        const ProcessingRate & rate = input.getRate();
        if (rate.isFixed()) {
            hasFixedRateInput = true;
            if (LLVM_UNLIKELY(input.isPrincipal())) {
                rateLCM = rate.getRate();
                noPrincipalStream = false;
                break;
            }
            rateLCM = lcm(rateLCM, rate.getRate());
        }
    }

    mTreatUnsafeKernelOperationsAsErrors = false;

    // Update the processed item counts
    for (unsigned i = 0; i < mStreamSetInputs.size(); ++i) {
        const Binding & input = mStreamSetInputs[i];
        if (hasDerivedItemCount(input)) {
            Value * const processed = b->CreateAdd(mInitialProcessedItemCount[i], mAccessibleInputItems[i]);
            b->setNonDeferredProcessedItemCount(input, processed);
        }
    }

    // Update the produced item counts
    if (hasFixedRateInput) {

        bool hasFixedRateOutput = false;
        for (const Binding & output : mStreamSetOutputs) {
            if (LLVM_UNLIKELY(output.isDeferred())) continue;
            const ProcessingRate & rate = output.getRate();
            if (rate.isFixed()) {
                rateLCM = lcm(rateLCM, rate.getRate());
                hasFixedRateOutput = true;
            }
        }

        if (hasFixedRateOutput) {

            Value * baseInitialProcessedItemCount = nullptr;
            Value * scaledInverseOfAvailItemCount = nullptr;

            // For each Fixed output stream, this calculates:

            //    CEILING(MIN(Available Item Count / Fixed Input Rate) * Fixed Output Rate)

            // But avoids the possibility of overflow errors (assuming that each processed item count does not overflow)

            for (unsigned i = 0; i < mStreamSetInputs.size(); ++i) {
                const Binding & input = mStreamSetInputs[i];
                if (noPrincipalStream || input.isPrincipal()) {
                    const ProcessingRate & rate = input.getRate();
                    if (rate.isFixed()) {
                        Value * const processed0 = mInitialProcessedItemCount[i];
                        Value * const unprocessed0 = b->CreateSub(mInitialAvailableItemCount[i], processed0);
                        Value * const unprocessed = b->CreateMul2(unprocessed0, rateLCM / rate.getRate());
                        scaledInverseOfAvailItemCount = b->CreateUMin(scaledInverseOfAvailItemCount, unprocessed);
                        Value * const processed = b->CreateUDiv2(processed0, rate.getRate());
                        baseInitialProcessedItemCount = b->CreateUMin(baseInitialProcessedItemCount, processed);
                    }
                }
            }

            for (const Binding & output : mStreamSetOutputs) {
                const ProcessingRate & rate = output.getRate();
                if (rate.isFixed() && !output.isDeferred()) {
                    assert (baseInitialProcessedItemCount && scaledInverseOfAvailItemCount);
                    Value * const initial = b->CreateMul2(baseInitialProcessedItemCount, rate.getRate());
                    Value * const produced = b->CreateCeilUDiv2(scaledInverseOfAvailItemCount, rateLCM / rate.getRate());
                    b->setProducedItemCount(output.getName(), b->CreateAdd(initial, produced));
                }
            }
        }
    }

    for (const Binding & output : mStreamSetOutputs) {
        bool hasModifiers = false;
        for (const Attribute & attr : output.getAttributes()) {
            if (attr.isAdd() || attr.isRoundUpTo()) {
                hasModifiers = true;
                break;
            }
        }
        if (LLVM_UNLIKELY(hasModifiers)) {
            Value * produced = b->getProducedItemCount(output.getName());
            for (const Attribute & attr : output.getAttributes()) {
                if (attr.isAdd()) {
                    produced = b->CreateAdd(produced, b->getSize(attr.amount()));
                } else if (attr.isRoundUpTo()) {
                    produced = b->CreateRoundUp(produced, b->getSize(attr.amount()));
                }
            }
            b->setProducedItemCount(output.getName(), produced);
        }
        #ifdef DEBUG_LOG
        b->CallPrintInt(getName() + "_" + output.getName() + "_produced\"", b->getProducedItemCount(output.getName()));
        #endif
    }

    mTreatUnsafeKernelOperationsAsErrors = true;

    // verify deferred input streams have fully processed all data
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        for (const Binding & input : mStreamSetInputs) {
            if (LLVM_UNLIKELY(input.isDeferred())) {
                Value * const processed = b->getProcessedItemCount(input.getName());
                Value * const available = b->getNonDeferredProcessedItemCount(input);
                std::string tmp;
                raw_string_ostream out(tmp);
                out << getName() << ": deferred stream \"" << input.getName() << "\" has not processed "
                                    "all of its available input upon termination";
                b->CreateAssert(b->CreateICmpEQ(processed, available), out.str());
            }
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
inline void MultiBlockKernel::checkTerminationSignal(const std::unique_ptr<KernelBuilder> & b) {
    if (hasAttribute(Attribute::KindId::MustExplicitlyTerminate)) {
        mIsFinal = b->getTerminationSignal();
    } else if (hasAttribute(Attribute::KindId::CanTerminateEarly)) {
        mIsFinal = b->CreateOr(mIsFinal, b->getTerminationSignal());
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareOverflowBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
inline void MultiBlockKernel::prepareOverflowBuffers(const std::unique_ptr<KernelBuilder> & b) {
    for (unsigned i = 0; i < mStreamSetOutputs.size(); ++i) {
        const Binding & output = mStreamSetOutputs[i];
        if (mustClearOverflowPriorToCopyback(output) && mStreamSetOutputBuffers[i]->supportsCopyBack()) {
            b->CreatePrepareOverflow(output.getName());
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasDerivedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
bool MultiBlockKernel::hasDerivedItemCount(const Binding & binding) {
    const ProcessingRate & rate = binding.getRate();
    return rate.isFixed() || rate.isPopCount() || rate.isNegatedPopCount();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPopCountRateItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * MultiBlockKernel::getPopCountRateItems(const std::unique_ptr<KernelBuilder> & b, const ProcessingRate & rate, Value * const strideIndex) {
    assert (rate.isPopCount() || rate.isNegatedPopCount());
    Port refPort;
    unsigned refIndex = 0;
    std::tie(refPort, refIndex) = getStreamPort(rate.getReference());
    assert (refPort == Port::Input);
    return b->CreateLoad(b->CreateGEP(mPopCountRateArray[refIndex], strideIndex));
}

// MULTI-BLOCK KERNEL CONSTRUCTOR
MultiBlockKernel::MultiBlockKernel(std::string && kernelName,
                                   Bindings && stream_inputs,
                                   Bindings && stream_outputs,
                                   Bindings && scalar_parameters,
                                   Bindings && scalar_outputs,
                                   Bindings && internal_scalars)
: Kernel(std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars))
, mInitiallyFinal(nullptr)
, mNumOfStrides(nullptr)
, mNumOfStridesInFinalSegment(nullptr) {

}

}
