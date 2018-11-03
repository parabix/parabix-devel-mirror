#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief beginKernel
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::setActiveKernel(BuilderRef b, const unsigned index) {
    assert (index < mPipeline.size());
    mKernelIndex = index;
    mKernel = mPipeline[index];
    assert (mKernel);
    b->setKernel(mPipelineKernel);
    Value * handle = b->getScalarField(makeKernelName(index));
    if (mKernel->hasFamilyName()) {
        handle = b->CreateBitCast(handle, mKernel->getKernelType()->getPointerTo());
    }
    mPipeline[index]->setHandle(b, handle);
    b->setKernel(mKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determineNumOfLogicalStrides
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::checkForSufficientInputDataAndOutputSpace(BuilderRef b) {

    assert (b->getKernel() == mKernel);

    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    mInputStrideLength.clear();
    mInputStrideLength.resize(numOfInputs, nullptr);
    mAccessibleInputItems.clear();
    mAccessibleInputItems.resize(numOfInputs, nullptr);

    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    mOutputStrideLength.clear();
    mOutputStrideLength.resize(numOfOutputs, nullptr);
    mWritableOutputItems.clear();
    mWritableOutputItems.resize(numOfOutputs, nullptr);

    // TODO: current analysis didn't take linear accessibility into account. For now, test everything but
    // return to this once this pipeline is responsible for determining buffer sizes / types.

    // TODO: if popcounts stated consume k items per marker, the popcount dependency graph needs to reflect it.

    initializePopCounts(b);
    mNonFinal = nullptr;
    for (unsigned i = 0; i < numOfInputs; ++i) {
        checkForSufficientInputData(b, i);
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        checkForSufficientOutputSpaceOrExpand(b, i);
    }
    assert (mNonFinal || numOfInputs == 0);
    return mNonFinal;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForSufficientInputData
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::checkForSufficientInputData(BuilderRef b, const unsigned index) {
    const Binding & input = mKernel->getInputStreamSetBinding(index);
    Value * const accessible = getAccessibleInputItems(b, index);
    Value * const strideLength = getInputStrideLength(b, index);
    Value * const requiredInput = addLookahead(b, index, strideLength);
    Value * const hasEnough = b->CreateICmpUGE(accessible, requiredInput);
    Value * const sufficientInput = b->CreateOr(hasEnough, mNoMore);
    const auto prefix = mKernel->getName() + "_" + input.getName();
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(prefix + "_requiredInput", requiredInput);
    #endif
    BasicBlock * const target = b->CreateBasicBlock(prefix + "_hasSufficientInput", mKernelLoopCall);
    branchToTargetOrLoopExit(b, sufficientInput, target);
    mNonFinal = mNonFinal ? b->CreateAnd(mNonFinal, hasEnough) : hasEnough;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getAccessibleInputItems
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getAccessibleInputItems(BuilderRef b, const unsigned index) {
    assert (index < mAccessibleInputItems.size());
    if (mAccessibleInputItems[index]) {
        return mAccessibleInputItems[index];
    } else {
        const Binding & input = mKernel->getInputStreamSetBinding(index);
        const StreamSetBuffer * const buffer = getInputBuffer(index);
        Value * const totalItems = getTotalItemCount(b, buffer);
        Value * const processed = b->getNonDeferredProcessedItemCount(input);
        Value * const accessible = buffer->getLinearlyAccessibleItems(b, processed, totalItems, getFacsimile(buffer));
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = mKernel->getName() + "." + input.getName();
        b->CallPrintInt(prefix + "_totalItems", totalItems);
        b->CallPrintInt(prefix + "_processed", processed);
        b->CallPrintInt(prefix + "_accessible", accessible);
        #endif
        storePopCountSourceItemCount(b, Port::Input, index, processed, accessible);
        mAccessibleInputItems[index] = accessible;
        return accessible;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForSufficientOutputSpaceOrExpand
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::checkForSufficientOutputSpaceOrExpand(BuilderRef b, const unsigned index) {
    // If the buffer is managed by the kernel, ignore it
    const StreamSetBuffer * const buffer = getOutputBuffer(index);
    if (LLVM_UNLIKELY(isa<ExternalBuffer>(buffer))) {
        return;
    }    
    const Binding & output = mKernel->getOutputStreamSetBinding(index);
    Value * const strideLength = getOutputStrideLength(b, index);
    Value * const writable = getWritableOutputItems(b, index);
    const auto prefix = mKernel->getName() + "_" + output.getName();
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(prefix + "_requiredSpace", strideLength);
    #endif
    Value * hasEnough = b->CreateICmpULE(strideLength, writable, prefix + "_hasEnough");
    if (requiresCopyBack(buffer)) { // check whether the potential overflow copy will overwrite the buffer
        Value * const produced = b->getNonDeferredProducedItemCount(output);
        Value * const consumed = getConsumedItemCount(b, index);
        Value * const unconsumed = b->CreateSub(produced, consumed);
        Value * const capacity = b->CreateSub(buffer->getCapacity(b.get()), strideLength);
        Value * const noOverwrites = b->CreateICmpULT(unconsumed, capacity, prefix + "_noOverflowOverwrite");
        hasEnough = b->CreateAnd(hasEnough, noOverwrites);
    }

    BasicBlock * const target = b->CreateBasicBlock(prefix + "_hasOutputSpace", mKernelLoopCall);
    if (LLVM_UNLIKELY(isa<DynamicBuffer>(buffer))) {
        expandOutputBuffer(b, hasEnough, index, target);
    } else {
        branchToTargetOrLoopExit(b, hasEnough, target);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief branchToTargetOrLoopExit
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::branchToTargetOrLoopExit(BuilderRef b, Value * const cond, BasicBlock * const target) {
    b->CreateLikelyCondBr(cond, target, mKernelLoopExit);
    BasicBlock * const exitBlock = b->GetInsertBlock();
    mTerminatedPhi->addIncoming(b->getFalse(), exitBlock);
    if (mHasProgressedPhi) {
        mHasProgressedPhi->addIncoming(mAlreadyProgressedPhi, exitBlock);
    }
    b->SetInsertPoint(target);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief expandOutputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::expandOutputBuffer(BuilderRef b, Value * const hasEnough, const unsigned index, BasicBlock * const target) {

    Value * const writable = mWritableOutputItems[index]; assert (writable);
    const Binding & output = mKernel->getOutputStreamSetBinding(index);
    const auto prefix = mKernel->getName() + "_" + output.getName();
    BasicBlock * const expand = b->CreateBasicBlock(prefix + "_expandOutputBuffer", target);
    b->CreateLikelyCondBr(hasEnough, target, expand);

    BasicBlock * const entryBlock = b->GetInsertBlock();
    b->SetInsertPoint(target);
    PHINode * const writablePhi = b->CreatePHI(writable->getType(), 2);
    writablePhi->addIncoming(writable, entryBlock);

    b->SetInsertPoint(expand);
    const StreamSetBuffer * const buffer = cast<DynamicBuffer>(getOutputBuffer(index));
    buffer->setCapacity(b.get(), calculateBufferExpansionSize(b, index));
    // reset the # of writable items to reflect the expanded buffer
    mWritableOutputItems[index] = nullptr;
    Value * const newWritableItems = getWritableOutputItems(b, index);
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(prefix + "_writable'", newWritableItems);
    #endif
    BasicBlock * const expansionExitBlock = b->GetInsertBlock();
    writablePhi->addIncoming(newWritableItems, expansionExitBlock);
    b->CreateBr(target);

    b->SetInsertPoint(target);
    // update the cached entry to be the phi node
    mWritableOutputItems[index] = writablePhi;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getWritableOutputItems
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getWritableOutputItems(BuilderRef b, const unsigned index) {
    assert (index < mWritableOutputItems.size());
    if (mWritableOutputItems[index]) {
        return mWritableOutputItems[index];
    } else {
        const Binding & output = mKernel->getOutputStreamSetBinding(index);
        const StreamSetBuffer * const buffer = getOutputBuffer(index);
        Value * const produced = b->getNonDeferredProducedItemCount(output);
        Value * const consumed = getConsumedItemCount(b, index);
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            Value * const sanityCheck = b->CreateICmpULE(consumed, produced);
            b->CreateAssert(sanityCheck,
                            mKernel->getName() + "." + output.getName() +
                            ": consumed count exceeds produced count");
        }
        Value * const writable = buffer->getLinearlyWritableItems(b, produced, consumed, getCopyBack(buffer));
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = mKernel->getName() + "." + output.getName();
        b->CallPrintInt(prefix + "_produced", produced);
        b->CallPrintInt(prefix + "_consumed", consumed);
        b->CallPrintInt(prefix + "_writable", writable);
        #endif
        storePopCountSourceItemCount(b, Port::Output, index, produced, writable);
        mWritableOutputItems[index] = writable;
        return writable;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateBufferExpansionSize
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::calculateBufferExpansionSize(BuilderRef b, const unsigned index) {
    const Binding & output = mKernel->getOutputStreamSetBinding(index);
    Value * const produced = b->getNonDeferredProducedItemCount(output);
    Value * const consumed = getConsumedItemCount(b, index);
    Value * const unconsumed = b->CreateSub(produced, consumed);
    Value * const strideLength = getOutputStrideLength(b, index);
    return b->CreateAdd(unconsumed, strideLength);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determineNumOfLinearStrides
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::determineNumOfLinearStrides(BuilderRef b) {
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    mNumOfLinearStrides = nullptr;
    for (unsigned i = 0; i < numOfInputs; ++i) {
        mNumOfLinearStrides = b->CreateUMin(mNumOfLinearStrides, getNumOfAccessibleStrides(b, i));
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        mNumOfLinearStrides = b->CreateUMin(mNumOfLinearStrides, getNumOfWritableStrides(b, i));
    }
    assert (mNumOfLinearStrides);
    return mNumOfLinearStrides;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfAccessibleStrides
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getNumOfAccessibleStrides(BuilderRef b, const unsigned index) {
    const Binding & input = mKernel->getInputStreamSetBinding(index);
    const ProcessingRate & rate = input.getRate();
    Value * numOfStrides = nullptr;
    if (LLVM_UNLIKELY(rate.isPopCount() || rate.isNegatedPopCount())) {
        numOfStrides = getMaximumNumOfPopCountStrides(b, rate);
    } else {
        Value * const accessible = subtractLookahead(b, index, getAccessibleInputItems(b, index));
        Value * const strideLength = getInputStrideLength(b, index);
        numOfStrides = b->CreateUDiv(accessible, strideLength);
    }
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = mKernel->getName() + "." + input.getName();
    b->CallPrintInt(prefix + "_numOfStrides", numOfStrides);
    #endif
    return numOfStrides;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfWritableStrides
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getNumOfWritableStrides(BuilderRef b, const unsigned index) {
    const StreamSetBuffer * const buffer = getOutputBuffer(index);
    if (LLVM_UNLIKELY(isa<ExternalBuffer>(buffer))) {
        return nullptr;
    } else {
        const Binding & output = mKernel->getOutputStreamSetBinding(index);
        const ProcessingRate & rate = output.getRate();
        Value * numOfStrides = nullptr;
        if (LLVM_UNLIKELY(rate.isPopCount() || rate.isNegatedPopCount())) {
            numOfStrides = getMaximumNumOfPopCountStrides(b, rate);
        } else {
            Value * const writable = getWritableOutputItems(b, index);
            Value * const strideLength = getOutputStrideLength(b, index);
            numOfStrides = b->CreateUDiv(writable, strideLength);
        }
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = mKernel->getName() + "." + output.getName();
        b->CallPrintInt(prefix + "_numOfStrides", numOfStrides);
        #endif
        return numOfStrides;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief provideAllInputAndOutputSpace
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::provideAllInputAndOutputSpace(BuilderRef b) {
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    Value * releasedItems[numOfInputs];
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    Value * pendingItems[numOfOutputs];
    for (unsigned i = 0; i < numOfInputs; ++i) {
        releasedItems[i] = getAccessibleInputItems(b, i);
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (mWritableOutputItemsPhi[i]) {
            pendingItems[i] = getWritableOutputItems(b, i);
        }
    }
    BasicBlock * const exitBlock = b->GetInsertBlock();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        mAccessibleInputItemsPhi[i]->addIncoming(releasedItems[i], exitBlock);
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (mWritableOutputItemsPhi[i]) {
            mWritableOutputItemsPhi[i]->addIncoming(pendingItems[i], exitBlock);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateNonFinalItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::calculateNonFinalItemCounts(BuilderRef b, Value * const numOfStrides) {
    assert (numOfStrides);
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    Value * releasedItems[numOfInputs];
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    Value * pendingItems[numOfOutputs];
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = mKernel->getInputStreamSetBinding(i);
        releasedItems[i] = calculateNumOfLinearItems(b, input, numOfStrides);
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (mWritableOutputItemsPhi[i]) {
            const Binding & output = mKernel->getOutputStreamSetBinding(i);
            pendingItems[i] = calculateNumOfLinearItems(b, output, numOfStrides);
        }
    }
    BasicBlock * const exitBlock = b->GetInsertBlock();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        mAccessibleInputItemsPhi[i]->addIncoming(releasedItems[i], exitBlock);
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (mWritableOutputItemsPhi[i]) {
            mWritableOutputItemsPhi[i]->addIncoming(pendingItems[i], exitBlock);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateFinalItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::calculateFinalItemCounts(BuilderRef b) {
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    Value * accessibleItems[numOfInputs];
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    Value * pendingItems[numOfOutputs];

    for (unsigned i = 0; i < numOfInputs; ++i) {
        accessibleItems[i] = addLookahead(b, i, getAccessibleInputItems(b, i));
    }

    // Record the writable item counts but when determining the number of Fixed writable items calculate:

    //   CEILING(PRINCIPAL_OR_MIN(Accessible Item Count / Fixed Input Rate) * Fixed Output Rate)

    // TODO: ZeroExtend attribute must affect the notion of "min" here too.

    // TODO: the pipeline must size the buffer to accommodate any Add/RoundUpTo attribute.

    RateValue rateLCM(1);
    bool noPrincipalStream = true;
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = mKernel->getInputStreamSetBinding(i);
        const ProcessingRate & rate = input.getRate();
        if (rate.isFixed()) {
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::Principal))) {
                rateLCM = rate.getRate();
                noPrincipalStream = false;
                break;
            }
            rateLCM = lcm(rateLCM, rate.getRate());
        }
    }

    bool hasFixedRateOutput = false;
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = mKernel->getOutputStreamSetBinding(i);
        const ProcessingRate & rate = output.getRate();
        if (rate.isFixed()) {
            rateLCM = lcm(rateLCM, rate.getRate());
            hasFixedRateOutput = true;
        }
    }

    Value * minScaledInverseOfAccessibleInput = nullptr;
    if (hasFixedRateOutput) {
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const Binding & input = mKernel->getInputStreamSetBinding(i);
            const ProcessingRate & rate = input.getRate();
            if (rate.isFixed() && (noPrincipalStream || input.hasAttribute(AttrId::Principal))) {
                Value * const scaledInverseOfAccessibleInput =
                    b->CreateMul2(accessibleItems[i], rateLCM / rate.getRate());
                minScaledInverseOfAccessibleInput =
                    b->CreateUMin(minScaledInverseOfAccessibleInput, scaledInverseOfAccessibleInput);
            }
        }
        assert (minScaledInverseOfAccessibleInput);
    }

    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (mWritableOutputItemsPhi[i]) {
            const Binding & output = mKernel->getOutputStreamSetBinding(i);
            const ProcessingRate & rate = output.getRate();
            Value * writable = nullptr;
            if (rate.isFixed() && minScaledInverseOfAccessibleInput) {
                writable = b->CreateCeilUDiv2(minScaledInverseOfAccessibleInput, rateLCM / rate.getRate());
            } else if (rate.isPopCount() || rate.isNegatedPopCount()) {
                writable = getInitialNumOfLinearPopCountItems(b, rate);
            } else {
                writable = getWritableOutputItems(b, i);
            }
            // update the final item counts with any Add/RoundUp attributes
            for (const Attribute & attr : output.getAttributes()) {
                if (attr.isAdd()) {
                    writable = b->CreateAdd(writable, b->getSize(attr.amount()));
                } else if (attr.isRoundUpTo()) {
                    writable = b->CreateRoundUp(writable, b->getSize(attr.amount()));
                }
            }
            pendingItems[i] = writable;
        }
    }



    BasicBlock * const exitBlock = b->GetInsertBlock();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        mAccessibleInputItemsPhi[i]->addIncoming(accessibleItems[i], exitBlock);
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (mWritableOutputItemsPhi[i]) {
            mWritableOutputItemsPhi[i]->addIncoming(pendingItems[i], exitBlock);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeKernelCall
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::itemCountSanityCheck(BuilderRef b,
                                                   const Binding & binding, const std::string & presentLabel, const std::string & pastLabel,
                                                   Value * const itemCount, Value * const expected, Value * const terminated) {

    const auto prefix = mKernel->getName() + "." + binding.getName();
    if (mKernel->isCountable(binding)) {
        Value * const exact = b->CreateICmpEQ(itemCount, expected);
        Value * const tooFew = b->CreateICmpULT(itemCount, expected);
        Value * const valid = b->CreateOr(exact, b->CreateAnd(tooFew, terminated));
        b->CreateAssert(valid, prefix + " did not " + presentLabel + " the expected number of items");
    } else {
        const auto lb = mKernel->getLowerBound(binding);
        if (lb > 0) {
            Value * const hasEnough = b->CreateICmpULE(itemCount, b->getSize(ceiling(lb * mKernel->getStride())));
            b->CreateAssert(b->CreateOr(hasEnough, terminated), prefix + " " + pastLabel + " fewer items than expected");
        }
        Value * const withinBounds = b->CreateICmpULE(itemCount, expected);
        b->CreateAssert(withinBounds, prefix + " " + pastLabel + " more items than expected");
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeKernelCall
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeKernelCall(BuilderRef b) {
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();

#warning TODO: add MProtect to buffers and their handles.

    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(mKernel->getName() + "_finalNumOfStrides", mNumOfLinearStridesPhi);
    #endif

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts) && numOfInputs > 0)) {
        Value * const hasStrides = b->CreateIsNotNull(mNumOfLinearStridesPhi);
        Value * const valid = b->CreateOr(hasStrides, mNoMore);
        b->CreateAssert(valid, mKernel->getName() + " must process at least one stride");
    }

    std::vector<Value *> arguments;
    arguments.reserve((numOfInputs + numOfOutputs + 1) * 2);
    arguments.push_back(mKernel->getHandle());
    arguments.push_back(mNumOfLinearStridesPhi);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = mKernel->getInputStreamSetBinding(i);
        arguments.push_back(getLogicalInputBaseAddress(b, i));
        arguments.push_back(mAccessibleInputItemsPhi[i]);
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
            arguments.push_back(getPopCountArray(b, i));
        }
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
            arguments.push_back(getNegatedPopCountArray(b, i));
        }
    }

    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (mWritableOutputItemsPhi[i]) {
            arguments.push_back(getLogicalOutputBaseAddress(b, i));
            arguments.push_back(mWritableOutputItemsPhi[i]);
        }        
    }

    std::vector<Value *> previouslyProcessedItemCount(0);
    std::vector<Value *> previouslyProducedItemCount(0);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        previouslyProcessedItemCount.resize(numOfInputs);
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const Binding & input = mKernel->getInputStreamSetBinding(i);
            previouslyProcessedItemCount[i] = b->getNonDeferredProcessedItemCount(input);
        }
        previouslyProducedItemCount.resize(numOfOutputs);
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            if (mWritableOutputItemsPhi[i]) {
                const Binding & output = mKernel->getOutputStreamSetBinding(i);
                previouslyProducedItemCount[i] = b->getNonDeferredProducedItemCount(output);
            }
        }
    }

    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(" *** " + mKernel->getName() + " ***", mNumOfLinearStridesPhi);
    #endif

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mPipelineKernel->getHandle(), CBuilder::Protect::NONE);
    }

    b->CreateCall(getDoSegmentFunction(b), arguments);

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mPipelineKernel->getHandle(), CBuilder::Protect::WRITE);
    }

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const terminated = isTerminated(b);
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const Binding & input = mKernel->getInputStreamSetBinding(i);
            Value * const processed = b->getNonDeferredProcessedItemCount(input);
            Value * const expected = b->CreateAdd(previouslyProcessedItemCount[i], mAccessibleInputItemsPhi[i]);
            itemCountSanityCheck(b, input, "process", "processed", processed, expected, terminated);
        }
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            if (previouslyProducedItemCount[i] == nullptr) continue;
            const Binding & output = mKernel->getOutputStreamSetBinding(i);
            Value * const produced = b->getNonDeferredProducedItemCount(output);
            Value * const expected = b->CreateAdd(previouslyProducedItemCount[i], mWritableOutputItemsPhi[i]);
            itemCountSanityCheck(b, output, "produce", "produced", produced, expected, terminated);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief zeroFillPartiallyWrittenOutputStreams
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::zeroFillPartiallyWrittenOutputStreams(BuilderRef b) {

    // TODO: this ought to check what streams have a lookahead dependency on this and make sure that it
    // zeroes the subsequent block(s) as necessary.

//    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
//    for (unsigned i = 0; i < numOfOutputs; ++i) {
//        const Binding & output = mKernel->getStreamOutput(i);
//        if (LLVM_UNLIKELY(output.hasAttribute(AttrId::ManagedBuffer))) {
//            continue;
//        }

//        Value * const produced = b->getProducedItemCount(output.getName());
//        const StreamSetBuffer * const buffer = getOutputBuffer(i);
//        Constant * const LOG_2_BLOCK_WIDTH = b->getSize(floor_log2(b->getBitBlockWidth()));
//        Value * const blockIndex = b->CreateLShr(produced, LOG_2_BLOCK_WIDTH);
//        const auto itemWidth = getItemWidth(buffer->getBaseType());
//        Constant * const BLOCK_MASK = b->getSize(b->getBitBlockWidth() - 1);
//        Value * packIndex = nullptr;
//        Value * maskOffset = nullptr;
//        if (itemWidth == 1) {
//            maskOffset = b->CreateAnd(produced, BLOCK_MASK);
//        } else {
//            Constant * const ITEM_WIDTH = b->getSize(itemWidth);
//            Value * const offset = b->CreateAnd(produced, BLOCK_MASK);
//            Value * const position = b->CreateMul(offset, ITEM_WIDTH);
//            packIndex = b->CreateLShr(position, LOG_2_BLOCK_WIDTH);
//            maskOffset = b->CreateAnd(position, BLOCK_MASK);
//        }
//        Value * const mask = b->bitblock_mask_to(maskOffset, true);
//        BasicBlock * const entry = b->GetInsertBlock();
//        const auto prefix = mKernel->getName() + "_" + output.getName();
//        BasicBlock * const loop = b->CreateBasicBlock(prefix + "_clearLoop", mKernelExit);
//        BasicBlock * const exit = b->CreateBasicBlock(prefix + "_clearExit", mKernelExit);
//        Value * const n = buffer->getStreamSetCount(b.get());
//        b->CreateBr(loop);

//        b->SetInsertPoint(loop);
//        PHINode * const streamIndex = b->CreatePHI(b->getSizeTy(), 2);
//        streamIndex->addIncoming(b->getSize(0), entry);
//        Value * ptr = nullptr;
//        if (packIndex) {
//            ptr = buffer->getStreamPackPtr(b.get(), streamIndex, blockIndex, packIndex);
//        } else {
//            ptr = buffer->getStreamBlockPtr(b.get(), streamIndex, blockIndex);
//        }
//        Value * const value = b->CreateBlockAlignedLoad(ptr);
//        b->CreateBlockAlignedStore(b->CreateAnd(value, mask), ptr);
//        Value * const nextStreamIndex = b->CreateAdd(streamIndex, b->getSize(1));
//        streamIndex->addIncoming(nextStreamIndex, loop);
//        b->CreateCondBr(b->CreateICmpNE(nextStreamIndex, n), loop, exit);

//        b->SetInsertPoint(exit);
//    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getInputStrideLength(BuilderRef b, const unsigned index) {
    assert (index < mInputStrideLength.size());
    if (mInputStrideLength[index]) {
        return mInputStrideLength[index];
    } else {
        const Binding & input = mKernel->getInputStreamSetBinding(index);
        Value * const strideLength = getInitialStrideLength(b, input);
        mInputStrideLength[index] = strideLength;
        return strideLength;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getOutputStrideLength(BuilderRef b, const unsigned index) {
    assert (index < mOutputStrideLength.size());
    if (mOutputStrideLength[index]) {
        return mOutputStrideLength[index];
    } else {
        const Binding & output = mKernel->getOutputStreamSetBinding(index);
        Value * const strideLength = getInitialStrideLength(b, output);
        mOutputStrideLength[index] = strideLength;
        return strideLength;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitialStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getInitialStrideLength(BuilderRef b, const Binding & binding) {
    const ProcessingRate & rate = binding.getRate();
    if (LLVM_LIKELY(rate.isFixed() || rate.isBounded())) {
        return b->getSize(ceiling(mKernel->getUpperBound(binding) * mKernel->getStride()));
    } else if (LLVM_UNLIKELY(rate.isPopCount() || rate.isNegatedPopCount())) {
        return getInitialNumOfLinearPopCountItems(b, rate);
    } else if (rate.isRelative()) {
        const Binding & ref = mKernel->getStreamBinding(rate.getReference());
        Value * const baseRate = getInitialStrideLength(b, ref);
        return b->CreateMul2(baseRate, rate.getRate());
    }
    llvm_unreachable("unexpected rate type");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateNumOfLinearItems
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::calculateNumOfLinearItems(BuilderRef b, const Binding & binding, Value * const numOfStrides) {
    const ProcessingRate & rate = binding.getRate();
    if (rate.isFixed() || rate.isBounded()) {
        return b->CreateMul2(numOfStrides, mKernel->getUpperBound(binding) * mKernel->getStride());
    } else if (rate.isPopCount() || rate.isNegatedPopCount()) {
        return getNumOfLinearPopCountItems(b, rate, numOfStrides);
    } else if (rate.isRelative()) {
        Value * const baseCount = calculateNumOfLinearItems(b, mKernel->getStreamBinding(rate.getReference()), numOfStrides);
        return b->CreateMul2(baseCount, rate.getRate());
    }
    llvm_unreachable("unexpected rate type");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFullyProcessedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getFullyProcessedItemCount(BuilderRef b, const Binding & input) const {
    Value * const processed = b->getProcessedItemCount(input.getName());
    if (LLVM_UNLIKELY(input.hasAttribute(AttrId::BlockSize))) {
        // If the input rate has a block size attribute then --- for the purpose of determining how many
        // items have been consumed --- we consider a stream set to be fully processed when an entire
        // stride has been processed.
        Constant * const BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());
        Value * const partial = b->CreateAnd(processed, ConstantExpr::getNeg(BLOCK_WIDTH));
        return b->CreateSelect(mTerminatedPhi, processed, partial);
    }
    return processed;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getTotalItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getTotalItemCount(BuilderRef b, const StreamSetBuffer * buffer) const {
    const auto p = mTotalItemCount.find(buffer);
    assert (p != mTotalItemCount.end());
    return p->second;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addLookahead
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::addLookahead(BuilderRef b, const unsigned index, Value * itemCount) const {
    const Binding & input = mKernel->getInputStreamSetBinding(index);
    if (LLVM_UNLIKELY(input.hasLookahead())) {
        Constant * const lookahead = b->getSize(input.getLookahead());
        itemCount = b->CreateAdd(itemCount, lookahead);
    }
    return itemCount;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief subtractLookahead
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::subtractLookahead(BuilderRef b, const unsigned index, Value * itemCount) const {
    const Binding & input = mKernel->getInputStreamSetBinding(index);
    if (LLVM_UNLIKELY(input.hasLookahead())) {
        Constant * const lookahead = b->getSize(input.getLookahead());
        itemCount = b->CreateSub(itemCount, lookahead);
    }
    return itemCount;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief allocateThreadLocalState
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::allocateThreadLocalState(BuilderRef b, const Port port, const unsigned i) {
    const auto & rate = getBinding(port, i).getRate();
    if (rate.isPopCount() || rate.isNegatedPopCount()) {
        allocateLocalPopCountArray(b, rate);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deallocateThreadLocalState
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::deallocateThreadLocalState(BuilderRef b, const Port port, const unsigned i) {
    const auto & rate = getBinding(port, i).getRate();
    if (rate.isPopCount() || rate.isNegatedPopCount()) {
        deallocateLocalPopCountArray(b, rate);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value *PipelineCompiler::getFunctionFromKernelState(BuilderRef b, Type * const type, const std::string & suffix) const {
    const auto kn = makeKernelName(mKernelIndex);
    b->setKernel(mPipelineKernel);
    Value * const funcPtr = b->getScalarField(kn + suffix);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        b->CreateAssert(funcPtr, mKernel->getName() + "." + suffix + " is null");
    }
    b->setKernel(mKernel);
    return b->CreateBitCast(funcPtr, type);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationFunction
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getInitializationFunction(BuilderRef b) const {
    Function * const init = mKernel->getInitFunction(b->getModule());
    if (mKernel->hasFamilyName()) {
        return getFunctionFromKernelState(b, init->getType(), INITIALIZE_FUNCTION_POINTER_SUFFIX);
    }
    return init;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDoSegmentFunction
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value *PipelineCompiler::getDoSegmentFunction(BuilderRef b) const {
    Function * const doSegment = mKernel->getDoSegmentFunction(b->getModule());
    if (mKernel->hasFamilyName()) {
        return getFunctionFromKernelState(b, doSegment->getType(), DO_SEGMENT_FUNCTION_POINTER_SUFFIX);
    }
    return doSegment;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalizeFunction
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getFinalizeFunction(BuilderRef b) const {
    Function * const term = mKernel->getTerminateFunction(b->getModule());
    if (mKernel->hasFamilyName()) {
        return getFunctionFromKernelState(b, term->getType(), FINALIZE_FUNCTION_POINTER_SUFFIX);
    }
    return term;
}

}
