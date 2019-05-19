#ifndef IO_CALCULATION_LOGIC_HPP
#define IO_CALCULATION_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determineNumOfLinearStrides
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::determineNumOfLinearStrides(BuilderRef b) {
    assert (b->getKernel() == mKernel);
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);

    // bound the number of strides by the maximum expected
    const BufferNode & bn = mBufferGraph[mKernelIndex];
    Constant * const maxStrides = b->getSize(ceiling(bn.Upper));
    mNumOfLinearStrides = b->CreateSub(maxStrides, mCurrentNumOfStrides);

    mBoundedKernel = false;
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableBlockingIOCounter) || DebugOptionIsSet(codegen::TraceBlockedIO))) {
        mBranchToLoopExit = b->getFalse();
    }
    for (const auto i : mPortEvaluationOrder) {
        Value * strides = nullptr;
        if (i < numOfInputs) {
            checkForSufficientInputData(b, i);
            strides = getNumOfAccessibleStrides(b, i);
        } else {
            checkForSufficientOutputSpaceOrExpand(b, i - numOfInputs);
            strides = getNumOfWritableStrides(b, i - numOfInputs);
        }
        if (strides) {
            mBoundedKernel = true;
        }
        mNumOfLinearStrides = b->CreateUMin(mNumOfLinearStrides, strides);
    }

    // When tracing blocking I/O, test all I/O streams but do not execute the
    // kernel if any stream is insufficient.
    if (mBranchToLoopExit) {
        BasicBlock * const noStreamIsInsufficient = b->CreateBasicBlock("", mKernelLoopCall);
        b->CreateUnlikelyCondBr(mBranchToLoopExit, mKernelLoopExit, noStreamIsInsufficient);
        updatePHINodesForLoopExit(b, mHalted);
        b->SetInsertPoint(noStreamIsInsufficient);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::calculateItemCounts(BuilderRef b) {


    // TODO: it would be better to try and statically prove whether a kernel will only ever
    // need a single "run" per segment rather than allowing only source kernels to have this
    // optimization.

    Value * isFinal = nullptr;

    const auto numOfInputs = in_degree(mKernelIndex, mBufferGraph);
    Vec<Value *> accessibleItems(numOfInputs);

    const auto numOfOutputs = out_degree(mKernelIndex, mBufferGraph);
    Vec<Value *> writableItems(numOfOutputs);

    calculateInputEpochAddresses(b);

    if (mBoundedKernel) {

        const auto prefix = makeKernelName(mKernelIndex);
        BasicBlock * const enteringNonFinalSegment = b->CreateBasicBlock(prefix + "_nonFinalSegment", mKernelLoopCall);
        BasicBlock * const enteringFinalStride = b->CreateBasicBlock(prefix + "_finalStride", mKernelLoopCall);
        isFinal = b->CreateICmpEQ(mNumOfLinearStrides, b->getSize(0));

        b->CreateUnlikelyCondBr(isFinal, enteringFinalStride, enteringNonFinalSegment);

        /// -------------------------------------------------------------------------------------
        /// KERNEL ENTERING FINAL STRIDE
        /// -------------------------------------------------------------------------------------

        b->SetInsertPoint(enteringFinalStride);
        Value * const finalFactor = calculateFinalItemCounts(b, accessibleItems, writableItems);
        Vec<Value *> inputEpochPhi(numOfInputs);
        zeroInputAfterFinalItemCount(b, accessibleItems, inputEpochPhi);
        phiOutItemCounts(b, accessibleItems, inputEpochPhi, writableItems, finalFactor);
        b->CreateBr(mKernelLoopCall);

        /// -------------------------------------------------------------------------------------
        /// KERNEL ENTERING NON-FINAL SEGMENT
        /// -------------------------------------------------------------------------------------

        b->SetInsertPoint(enteringNonFinalSegment);
        Value * const nonFinalFactor = calculateNonFinalItemCounts(b, accessibleItems, writableItems);
        phiOutItemCounts(b, accessibleItems, mInputEpoch, writableItems, nonFinalFactor);
        b->CreateBr(mKernelLoopCall);

    } else {
        mNumOfLinearStrides = b->getSize(1);
        Value * const nonFinalFactor = calculateNonFinalItemCounts(b, accessibleItems, writableItems);
        phiOutItemCounts(b, accessibleItems, mInputEpoch, writableItems, nonFinalFactor);
        b->CreateBr(mKernelLoopCall);
    }

    return isFinal;

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForSufficientInputData
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::checkForSufficientInputData(BuilderRef b, const unsigned inputPort) {
    // TODO: we could eliminate some checks if we can prove a particular input
    // must have enough data based on its already tested inputs and ignore
    // checking whether an input kernel is terminated if a stronger test has
    // already been done. Work out the logic for these tests globally.
    const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Input, inputPort});
    Value * const accessible = getAccessibleInputItems(b, inputPort);
    mAccessibleInputItems[inputPort] = accessible;
    Value * const strideLength = getInputStrideLength(b, inputPort);
    Value * const required = addLookahead(b, inputPort, strideLength);
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(prefix + "_required", required);
    #endif
    const Binding & input = getInputBinding(inputPort);
    const ProcessingRate & rate = input.getRate();
    Value * sufficientInput = nullptr;
    if (LLVM_UNLIKELY(rate.isGreedy())) {
        sufficientInput = b->CreateICmpUGE(accessible, required);
    } else {
        Value * const hasEnough = b->CreateICmpUGE(accessible, required);
        sufficientInput = b->CreateOr(hasEnough, isClosed(b, inputPort));
    }
    Value * const halting = isPipelineInput(inputPort) ? b->getTrue() : mHalted;
    BasicBlock * const target = b->CreateBasicBlock(prefix + "_hasInputData", mKernelLoopCall);
    branchToTargetOrLoopExit(b, StreamPort(PortType::Input, inputPort), sufficientInput, target, halting);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getAccessibleInputItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getAccessibleInputItems(BuilderRef b, const unsigned inputPort, const bool useOverflow) {
    assert (inputPort < mAccessibleInputItems.size());
    const StreamSetBuffer * const buffer = getInputBuffer(inputPort);
    Value * const available = getLocallyAvailableItemCount(b, inputPort);
    Value * const processed = mAlreadyProcessedPhi[inputPort];
    ConstantInt * facsimile = nullptr;
    if (LLVM_LIKELY(useOverflow)) {
        const auto size = getLookAhead(getInputBufferVertex(inputPort));
        if (size) {
            facsimile = b->getSize(size);
        }
    }
    const Binding & input = getInputBinding(inputPort);
    Value * accessible = buffer->getLinearlyAccessibleItems(b, processed, available, facsimile);
    #ifndef DISABLE_ZERO_EXTEND
    if (LLVM_UNLIKELY(input.hasAttribute(AttrId::ZeroExtended))) {
        // To zero-extend an input stream, we must first exhaust all input for this stream before
        // switching to a "zeroed buffer". The size of the buffer will be determined by the final
        // number of non-zero-extended strides.

        // NOTE: the producer of this stream will zero out all data after its last produced item
        // that can be read by a single iteration of any consuming kernel.

        Constant * const MAX_INT = ConstantInt::getAllOnesValue(b->getSizeTy());
        Value * const closed = isClosed(b, inputPort);
        Value * const exhausted = b->CreateICmpUGE(processed, available);
        Value * const useZeroExtend = b->CreateAnd(closed, exhausted);
        mIsInputZeroExtended[inputPort] = useZeroExtend;
        accessible = b->CreateSelect(useZeroExtend, MAX_INT, accessible);
    }
    #endif
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Input, inputPort});
    b->CallPrintInt(prefix + "_available", available);
    b->CallPrintInt(prefix + "_processed", processed);
    b->CallPrintInt(prefix + "_accessible", accessible);
    #endif
    if (LLVM_UNLIKELY(mCheckAssertions)) {
        Value * sanityCheck = b->CreateICmpULE(processed, available);
        if (mIsInputZeroExtended[inputPort]) {
            sanityCheck = b->CreateOr(mIsInputZeroExtended[inputPort], sanityCheck);
        }
        b->CreateAssert(sanityCheck,
                        input.getName() +
                        ": processed count (%d) exceeds total count (%d)",
                        processed, available);
    }
    return accessible;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForSufficientOutputSpaceOrExpand
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::checkForSufficientOutputSpaceOrExpand(BuilderRef b, const unsigned outputPort) {

    const auto bufferVertex = getOutputBufferVertex(outputPort);
    const BufferNode & bn = mBufferGraph[bufferVertex];

    if (LLVM_UNLIKELY(bn.Type == BufferType::Managed)) {
        #if 0
        // If we have a managed buffer, we cannot directly control its size. However, if we know
        // that total amount of unconsumed data (excluding the unconsumed data of a deferred rate
        // buffer) supports a full segment of all of its consumers, we can skip processing it.

        #error add non-deferred consumed item count!

        // TODO: for this to be safe, we may only abort if *all* outputs are "saturated" not just
        // any outputs. Moreover, we need a nonDeferredConsumedItemCount to be recorded.

        Value * const produced = mInitiallyProducedItemCount[outputPort];
        Value * const consumed = mConsumedItemCount[outputPort];
        Value * const unconsumed = b->CreateSub(produced, consumed);

        RateValue max{0};
        for (const auto e : make_iterator_range(out_edges(bufferVertex, mBufferGraph))) {
            const BufferRateData & rd = mBufferGraph[e];
            max = std::max(max, rd.MaximumFlow);
        }
        Constant * const maxRequired = b->getSize(ceiling(max) * codegen::ThreadNum);
        Value * const notFull = b->CreateICmpULT(unconsumed, maxRequired);
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, outputPort});
        #ifdef PRINT_DEBUG_MESSAGES
        b->CallPrintInt(prefix + "_unconsumed", unconsumed);
        #endif
        BasicBlock * const target = b->CreateBasicBlock(prefix + "_notFull", mKernelLoopCall);
        Value * const halting = isPipelineOutput(outputPort) ? b->getTrue() : mHalted;
        branchToTargetOrLoopExit(b, StreamPort(PortType::Output, outputPort), notFull, target, halting);
        #endif
    } else {
        Value * writable = nullptr;
        const StreamSetBuffer * const buffer = bn.Buffer;
        if (LLVM_UNLIKELY(isa<DynamicBuffer>(buffer))) {
            writable = reserveSufficientCapacity(b, outputPort);
        } else {
            writable = getWritableOutputItems(b, outputPort);
            Value * const required = getOutputStrideLength(b, outputPort);
            const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, outputPort});
            #ifdef PRINT_DEBUG_MESSAGES
            b->CallPrintInt(prefix + "_required", required);
            #endif
            Value * const hasEnough = b->CreateICmpULE(required, writable, prefix + "_hasEnough");
            BasicBlock * const target = b->CreateBasicBlock(prefix + "_hasOutputSpace", mKernelLoopCall);
            Value * const halting = isPipelineOutput(outputPort) ? b->getTrue() : mHalted;
            branchToTargetOrLoopExit(b, StreamPort(PortType::Output, outputPort), hasEnough, target, halting);
        }
        mWritableOutputItems[outputPort] = writable;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief reserveSufficientCapacity
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::reserveSufficientCapacity(BuilderRef b, const unsigned outputPort) {
    const StreamSetBuffer * const buffer = getOutputBuffer(outputPort);
    Value * const produced = mAlreadyProducedPhi[outputPort]; assert (produced);
    Value * const consumed = mConsumedItemCount[outputPort]; assert (consumed);
    Value * const required = getOutputStrideLength(b, outputPort);
    ConstantInt * copyBack = nullptr;
    const auto size = getCopyBack(getOutputBufferVertex(outputPort));
    if (size) {
        copyBack = b->getSize(size - 1);
    }

    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, outputPort});
    b->CallPrintInt(prefix + "_produced", produced);
    b->CallPrintInt(prefix + "_required", required);
    #endif

    Value * const remaining = buffer->getLinearlyWritableItems(b, produced, consumed, copyBack);

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const expandBuffer = b->CreateBasicBlock("expandBuffer", mKernelLoopCall);
    BasicBlock * const expanded = b->CreateBasicBlock("expanded", mKernelLoopCall);

    b->CreateLikelyCondBr(b->CreateICmpULE(required, remaining), expanded, expandBuffer);

    b->SetInsertPoint(expandBuffer);
    Value * const cycleCounterAccumulator = getBufferExpansionCycleCounter(b);
    Value * cycleCounterStart = nullptr;
    if (cycleCounterAccumulator) {
        cycleCounterStart = b->CreateReadCycleCounter();
    }
    Value * const newlyWritable = buffer->reserveCapacity(b, produced, consumed, required, copyBack);
    recordBufferExpansionHistory(b, outputPort, buffer);
    if (cycleCounterAccumulator) {
        Value * const cycleCounterEnd = b->CreateReadCycleCounter();
        Value * const duration = b->CreateSub(cycleCounterEnd, cycleCounterStart);
        Value * const accum = b->CreateAdd(b->CreateLoad(cycleCounterAccumulator), duration);
        b->CreateStore(accum, cycleCounterAccumulator);
    }
    BasicBlock * const expandBufferExit = b->GetInsertBlock();
    b->CreateBr(expanded);

    b->SetInsertPoint(expanded);
    PHINode * const writable = b->CreatePHI(b->getSizeTy(), 2);
    writable->addIncoming(remaining, entryBlock);
    writable->addIncoming(newlyWritable, expandBufferExit);
    return writable;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getWritableOutputItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getWritableOutputItems(BuilderRef b, const unsigned outputPort, const bool useOverflow) {
    assert (outputPort < mWritableOutputItems.size());
    const Binding & output = getOutputBinding(outputPort);
    const StreamSetBuffer * const buffer = getOutputBuffer(outputPort);
    Value * const produced = mAlreadyProducedPhi[outputPort]; assert (produced);
    Value * const consumed = mConsumedItemCount[outputPort]; assert (consumed);
    if (LLVM_UNLIKELY(mCheckAssertions)) {
        Value * const sanityCheck = b->CreateICmpULE(consumed, produced);
        b->CreateAssert(sanityCheck,
                        output.getName() +
                        ": consumed count (%d) exceeds produced count (%d)",
                        consumed, produced);
    }
    ConstantInt * copyBack = nullptr;
    if (LLVM_LIKELY(useOverflow)) {
        const auto size = getCopyBack(getOutputBufferVertex(outputPort));
        if (size) {
            copyBack = b->getSize(size - 1);
        }
    }
    Value * const writable = buffer->getLinearlyWritableItems(b, produced, consumed, copyBack);
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, outputPort});
    b->CallPrintInt(prefix + "_produced", produced);
    b->CallPrintInt(prefix + "_writable", writable);
    #endif
    return writable;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfAccessibleStrides
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getNumOfAccessibleStrides(BuilderRef b, const unsigned inputPort) {
    const Binding & input = getInputBinding(inputPort);
    const ProcessingRate & rate = input.getRate();
    Value * numOfStrides = nullptr;
    if (LLVM_UNLIKELY(rate.isPartialSum())) {
        numOfStrides = getMaximumNumOfPartialSumStrides(b, StreamPort{PortType::Input, inputPort});
    } else if (LLVM_UNLIKELY(rate.isGreedy())) {
        #warning this ought to return nullptr
        Value * const accessible = mAccessibleInputItems[inputPort];
        numOfStrides = subtractLookahead(b, inputPort, accessible);
    } else {
        Value * const accessible = mAccessibleInputItems[inputPort];
        Value * const strideLength = getInputStrideLength(b, inputPort);
        numOfStrides = b->CreateUDiv(subtractLookahead(b, inputPort, accessible), strideLength);
    }
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Input, inputPort});
    #endif
    if (mIsInputZeroExtended[inputPort]) {
        numOfStrides = b->CreateSelect(mIsInputZeroExtended[inputPort], mNumOfLinearStrides, numOfStrides);
    }
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt("< " + prefix + "_numOfStrides", numOfStrides);
    #endif
    return numOfStrides;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfWritableStrides
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getNumOfWritableStrides(BuilderRef b, const unsigned outputPort) {
    Value * numOfStrides = nullptr;
    if (LLVM_LIKELY(getOutputBufferType(outputPort) != BufferType::Managed)) {
        const Binding & output = getOutputBinding(outputPort);
        if (LLVM_UNLIKELY(output.getRate().isPartialSum())) {
            numOfStrides = getMaximumNumOfPartialSumStrides(b, StreamPort{PortType::Output, outputPort});
        } else {
            Value * const writable = mWritableOutputItems[outputPort];
            Value * const strideLength = getOutputStrideLength(b, outputPort);
            numOfStrides = b->CreateUDiv(writable, strideLength);
        }
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, outputPort});
        b->CallPrintInt("> " + prefix + "_numOfStrides", numOfStrides);
        #endif
    }
    return numOfStrides;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateNonFinalItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::calculateNonFinalItemCounts(BuilderRef b, Vec<Value *> & accessibleItems, Vec<Value *> & writableItems) {
    assert (mNumOfLinearStrides);
    Value * fixedRateFactor = nullptr;
    if (mFixedRateFactorPhi) {
        const RateValue stride(mKernel->getStride());
        fixedRateFactor  = b->CreateMul2(mNumOfLinearStrides, stride * mFixedRateLCM);
    }
    const auto numOfInputs = accessibleItems.size();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        accessibleItems[i] = calculateNumOfLinearItems(b, StreamPort{PortType::Input, i});
    }
    const auto numOfOutputs = writableItems.size();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        writableItems[i] = calculateNumOfLinearItems(b, StreamPort{PortType::Output, i});
    }
    return fixedRateFactor;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateFinalItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::calculateFinalItemCounts(BuilderRef b, Vec<Value *> & accessibleItems, Vec<Value *> & writableItems) {
    const auto numOfInputs = accessibleItems.size();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        Value * accessible = mAccessibleInputItems[i];
        if (LLVM_UNLIKELY(mIsInputZeroExtended[i] != nullptr)) {
            // If this input stream is zero extended, the current input items will be MAX_INT.
            // However, since we're now in the final stride, so we can bound the stream to:
            Value * const maxItems = b->CreateAdd(mAlreadyProcessedPhi[i], mFirstInputStrideLength[i]);
            // But since we may not necessarily be in our zero extension region, we must first
            // test whether we are:
            accessible = b->CreateSelect(mIsInputZeroExtended[i], maxItems, accessible);
        }
        accessibleItems[i] = accessible;
    }

    Value * minFixedRateFactor = nullptr;
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = getInputBinding(i);
        const ProcessingRate & rate = input.getRate();
        if (rate.isFixed()) {
            Value * const fixedRateFactor =
                b->CreateMul2(accessibleItems[i], mFixedRateLCM / rate.getRate());
            minFixedRateFactor =
                b->CreateUMin(minFixedRateFactor, fixedRateFactor);
        }
    }

    // truncate any fixed rate input down to the length of the shortest stream
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = getInputBinding(i);
        const ProcessingRate & rate = input.getRate();
        Value * accessible = accessibleItems[i];
        if (rate.isFixed() && minFixedRateFactor) {

            const auto factor = rate.getRate() / mFixedRateLCM;

            Value * calculated = b->CreateCeilUMul2(minFixedRateFactor, factor);
            const auto buffer = getInputBufferVertex(i);
            const RateValue k = mAddGraph[buffer] - mAddGraph[mKernelIndex];
            // ... but ensure that it reflects whether it was produced with an Add(k) rate.
            if (LLVM_UNLIKELY(k != RateValue{0})) {

                // (x + (g/h)) * (c/d) = (xh + g) * c/hd

                Constant * const h = b->getSize(k.denominator());
                Value * const xh = b->CreateMul(minFixedRateFactor, h);
                Constant * const g = b->getSize(k.numerator());
                Value * const y = b->CreateAdd(xh, g);
                const auto r = factor / RateValue{k.denominator()};
                Value * const z = b->CreateCeilUMul2(y, r);
                calculated = b->CreateSelect(isClosed(b, i), z, calculated);
            }
            if (LLVM_UNLIKELY(mCheckAssertions)) {
                Value * correctItemCount = b->CreateICmpEQ(calculated, accessibleItems[i]);
                if (LLVM_UNLIKELY(mIsInputZeroExtended[i] != nullptr)) {
                    correctItemCount = b->CreateOr(correctItemCount, mIsInputZeroExtended[i]);
                }
                b->CreateAssert(correctItemCount,
                                input.getName() +
                                ": expected final fixed rate item count (%d) "
                                "does not match accessible item count (%d)",
                                calculated, accessibleItems[i]);
            }
            accessible = calculated;
        }
        accessibleItems[i] = addLookahead(b, i, accessible);
    }

    const auto numOfOutputs = writableItems.size();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = getOutputBinding(i);
        const ProcessingRate & rate = output.getRate();
        Value * writable = mWritableOutputItems[i];
        if (rate.isPartialSum()) {
            writable = mFirstOutputStrideLength[i];
        } else if (rate.isFixed() && minFixedRateFactor) {
            Value * const calculated = b->CreateCeilUMul2(minFixedRateFactor, rate.getRate() / mFixedRateLCM);
            if (LLVM_UNLIKELY(mCheckAssertions)) {
                b->CreateAssert(b->CreateICmpULE(calculated, writable),
                                output.getName() +
                                ": final calculated fixed rate item count (%d) "
                                "exceeds writable item count (%d)",
                                calculated, writable);
            }
            writable = calculated;
        }

        // update the final item counts with any Add/RoundUp attributes
        for (const Attribute & attr : output.getAttributes()) {
            if (attr.isAdd()) {
                writable = b->CreateAdd(writable, b->getSize(attr.amount()));
            } else if (attr.isRoundUpTo()) {
                writable = b->CreateRoundUp(writable, b->getSize(attr.amount()));
            }
        }
        writableItems[i] = writable;
    }
    return minFixedRateFactor;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getInputStrideLength(BuilderRef b, const unsigned inputPort) {
    assert (inputPort < mFirstInputStrideLength.size());
    if (mFirstInputStrideLength[inputPort]) {
        return mFirstInputStrideLength[inputPort];
    } else {
        Value * const strideLength = getFirstStrideLength(b, StreamPort{PortType::Input, inputPort});
        mFirstInputStrideLength[inputPort] = strideLength;
        return strideLength;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getOutputStrideLength(BuilderRef b, const unsigned outputPort) {
    assert (outputPort < mFirstOutputStrideLength.size());
    if (mFirstOutputStrideLength[outputPort]) {
        return mFirstOutputStrideLength[outputPort];
    } else {
        Value * const strideLength = getFirstStrideLength(b, StreamPort{PortType::Output, outputPort});
        mFirstOutputStrideLength[outputPort] = strideLength;
        return strideLength;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPartialSumItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getPartialSumItemCount(BuilderRef b, const StreamPort port, Value * const offset) const {
    const auto ref = getReference(port);
    assert (ref.Type == PortType::Input);
    const auto inputPort = ref.Number;
    const StreamSetBuffer * const buffer = getInputBuffer(inputPort);

    Value * prior = nullptr;
    if (port.Type == PortType::Input) {
        prior = mAlreadyProcessedPhi[port.Number];
    } else { // if (port.Type == PortType::Output) {
        prior = mAlreadyProducedPhi[port.Number];
    }

    Constant * const ZERO = b->getSize(0);
    Value * position = mAlreadyProcessedPhi[inputPort];
    if (offset) {
        if (LLVM_UNLIKELY(mCheckAssertions)) {
            const auto & binding = getBinding(port);
            b->CreateAssert(b->CreateICmpNE(offset, ZERO),
                            binding.getName() + ": partial sum offset must be non-zero");
        }
        Constant * const ONE = b->getSize(1);
        position = b->CreateAdd(position, b->CreateSub(offset, ONE));
    }

    Value * const currentPtr = buffer->getRawItemPointer(b.get(), ZERO, position);
    Value * current = b->CreateLoad(currentPtr);
    if (mBranchToLoopExit) {
        current = b->CreateSelect(mBranchToLoopExit, prior, current);
    }
    if (LLVM_UNLIKELY(mCheckAssertions)) {
        const auto & binding = getBinding(port);
        b->CreateAssert(b->CreateICmpULE(prior, current),
                        binding.getName() + ": partial sum is not non-decreasing "
                                            "(prior %d > current %d)", prior, current);
    }
    return b->CreateSub(current, prior);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getMaximumNumOfPartialSumStrides
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getMaximumNumOfPartialSumStrides(BuilderRef b, const StreamPort port) {
    IntegerType * const sizeTy = b->getSizeTy();
    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);
    Constant * const MAX_INT = ConstantInt::getAllOnesValue(sizeTy);


    Value * initialItemCount = nullptr;
    Value * sourceItemCount = nullptr;
    Value * peekableItemCount = nullptr;

    const auto portNum = port.Number;
    if (port.Type == PortType::Input) {
        initialItemCount = mAlreadyProcessedPhi[portNum];
        Value * const accessible = mAccessibleInputItems[portNum];
        if (requiresLookAhead(getInputBufferVertex(portNum))) {
            Value * const nonOverflowItems = getAccessibleInputItems(b, portNum, false);
            sourceItemCount = b->CreateAdd(initialItemCount, nonOverflowItems);
            peekableItemCount = b->CreateAdd(initialItemCount, accessible);
        } else {
            sourceItemCount = b->CreateAdd(initialItemCount, accessible);
        }
        sourceItemCount = subtractLookahead(b, portNum, sourceItemCount);
    } else { // if (port.Type == PortType::Output) {
        initialItemCount = mAlreadyProducedPhi[portNum];
        Value * const writable = mWritableOutputItems[portNum];
        if (requiresCopyBack(getOutputBufferVertex(portNum))) {
            Value * const nonOverflowItems = getWritableOutputItems(b, portNum, false);
            sourceItemCount = b->CreateAdd(initialItemCount, nonOverflowItems);
            peekableItemCount = b->CreateAdd(initialItemCount, writable);
        } else {
            sourceItemCount = b->CreateAdd(initialItemCount, writable);
        }
    }

    const auto ref = getReference(port);
    assert (ref.Type == PortType::Input);
    const auto refPortNum = ref.Number;

    const StreamSetBuffer * const buffer = getInputBuffer(refPortNum);
    const auto prefix = makeBufferName(mKernelIndex, ref) + "_readPartialSum";

    BasicBlock * const popCountLoop =
        b->CreateBasicBlock(prefix + "Loop", mKernelLoopCall);
    BasicBlock * const popCountLoopExit =
        b->CreateBasicBlock(prefix + "LoopExit", mKernelLoopCall);

    Value * const baseOffset = mAlreadyProcessedPhi[refPortNum];
    Value * const baseAddress = buffer->getRawItemPointer(b.get(), ZERO, baseOffset);

    BasicBlock * const popCountEntry = b->GetInsertBlock();
    Value * minimumItemCount = nullptr;
    if (peekableItemCount) {
        minimumItemCount = b->CreateLoad(baseAddress);
        Value * const mustUseOverflow = b->CreateICmpULT(sourceItemCount, minimumItemCount);
        b->CreateUnlikelyCondBr(mustUseOverflow, popCountLoopExit, popCountLoop);
    } else {
        b->CreateBr(popCountLoop);
    }

    // TODO: replace this with a parallel icmp check and bitscan? binary search with initial
    // check on the rightmost entry?

    b->SetInsertPoint(popCountLoop);
    PHINode * const numOfStrides = b->CreatePHI(sizeTy, 2);
    numOfStrides->addIncoming(mNumOfLinearStrides, popCountEntry);
    PHINode * const nextRequiredItems = b->CreatePHI(sizeTy, 2);
    nextRequiredItems->addIncoming(MAX_INT, popCountEntry);
    Value * const strideIndex = b->CreateSub(numOfStrides, ONE);
    Value * const ptr = b->CreateGEP(baseAddress, strideIndex);
    Value * const requiredItems = b->CreateLoad(ptr);
    Value * const hasEnough = b->CreateICmpULE(requiredItems, sourceItemCount);
    nextRequiredItems->addIncoming(requiredItems, popCountLoop);
    numOfStrides->addIncoming(strideIndex, popCountLoop);
    b->CreateCondBr(hasEnough, popCountLoopExit, popCountLoop);

    b->SetInsertPoint(popCountLoopExit);
    PHINode * const numOfStridesPhi = b->CreatePHI(sizeTy, 2);
    numOfStridesPhi->addIncoming(numOfStrides, popCountLoop);
    PHINode * const requiredItemsPhi = b->CreatePHI(sizeTy, 2);
    requiredItemsPhi->addIncoming(requiredItems, popCountLoop);
    PHINode * const nextRequiredItemsPhi = b->CreatePHI(sizeTy, 2);
    nextRequiredItemsPhi->addIncoming(nextRequiredItems, popCountLoop);
    if (peekableItemCount) {
        numOfStridesPhi->addIncoming(ZERO, popCountEntry);
        requiredItemsPhi->addIncoming(ZERO, popCountEntry);
        nextRequiredItemsPhi->addIncoming(minimumItemCount, popCountEntry);
    }
    Value * finalNumOfStrides = numOfStridesPhi;
    if (peekableItemCount) {
        // Since we want to allow the stream to peek into the overflow but not start
        // in it, check to see if we can support one more stride by using it.
        Value * const endedPriorToBufferEnd = b->CreateICmpNE(requiredItemsPhi, sourceItemCount);
        Value * const canPeekIntoOverflow = b->CreateICmpULE(nextRequiredItemsPhi, peekableItemCount);
        Value * const useOverflow = b->CreateAnd(endedPriorToBufferEnd, canPeekIntoOverflow);
        finalNumOfStrides = b->CreateSelect(useOverflow, b->CreateAdd(numOfStridesPhi, ONE), numOfStridesPhi);
    }
    return finalNumOfStrides;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFirstStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getFirstStrideLength(BuilderRef b, const StreamPort port) {
    const Binding & binding = getBinding(port);
    const ProcessingRate & rate = binding.getRate();
    if (LLVM_LIKELY(rate.isFixed() || rate.isBounded())) {
        const RateValue ub = rate.getUpperBound() * mKernel->getStride();
        if (LLVM_UNLIKELY(ub.denominator() != 1)) {
            std::string tmp;
            raw_string_ostream out(tmp);
            out << mKernel->getName() << "." << binding.getName()
                << ": rate upper-bound is not a multiple of kernel stride.";
            report_fatal_error(out.str());
        }
        return b->getSize(ub.numerator());
    } else if (rate.isPartialSum()) {
        return getPartialSumItemCount(b, port);
    } else if (rate.isGreedy()) {
        if (LLVM_UNLIKELY(port.Type == PortType::Output)) {
            if (LLVM_UNLIKELY(port.Type == PortType::Output)) {
                SmallVector<char, 0> tmp;
                raw_svector_ostream out(tmp);
                out << "output " << mKernel->getName() << "." << binding.getName()
                    << " cannot have a Greedy rate.";
                report_fatal_error(out.str());
            }

        }
        const RateValue lb = rate.getLowerBound(); // * mKernel->getStride();
        const auto ilb = floor(lb);
        Value * firstBound = b->getSize(ilb);
        if (LLVM_UNLIKELY(ilb > 0)) {
            Constant * const ZERO = b->getSize(0);
            firstBound = b->CreateSelect(isClosed(b, port.Number), ZERO, firstBound);
        }
        Constant * const subsequentBound = b->getSize(ceiling(lb) + 1);
        return b->CreateSelect(mExecutedAtLeastOncePhi, subsequentBound, firstBound);
    } else if (rate.isRelative()) {
        Value * const baseRate = getFirstStrideLength(b, getReference(port));
        return b->CreateMul2(baseRate, rate.getRate());
    }
    llvm_unreachable("unexpected rate type");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateNumOfLinearItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::calculateNumOfLinearItems(BuilderRef b, const StreamPort port) {
    const Binding & binding = getBinding(port);
    const ProcessingRate & rate = binding.getRate();
    if (rate.isFixed() || rate.isBounded()) {
        return b->CreateMul2(mNumOfLinearStrides, rate.getUpperBound() * mKernel->getStride());
    } else if (rate.isGreedy()) {
        assert (port.Type == PortType::Input);
        return mAccessibleInputItems[port.Number];
    } else if (rate.isPartialSum()) {
        return getPartialSumItemCount(b, port, mNumOfLinearStrides);
    } else if (rate.isRelative()) {
        Value * const baseCount = calculateNumOfLinearItems(b, getReference(port));
        return b->CreateMul2(baseCount, rate.getRate());
    }
    llvm_unreachable("unexpected rate type");
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief branchToTargetOrLoopExit
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::branchToTargetOrLoopExit(BuilderRef b, const StreamPort port,
                                                Value * const cond, BasicBlock * const target,
                                                Value * const halting) {

    BasicBlock * recordBlockedIO = nullptr;
    BasicBlock * insufficentIO = mKernelLoopExit;

    if (mBranchToLoopExit) {
        const auto prefix = makeBufferName(mKernelIndex, port);
        recordBlockedIO = b->CreateBasicBlock(prefix + "_recordBlockedIO", mKernelLoopExit);
        insufficentIO = recordBlockedIO;
    }

    BasicBlock * const entryBlock = b->GetInsertBlock();


    Value * test = cond;
    Value * insufficient = mBranchToLoopExit;
    if (mBranchToLoopExit) {
        // do not record the block if this not the first execution of the
        // kernel but ensure that the system knows at least one failed.
        test = b->CreateOr(cond, mExecutedAtLeastOncePhi);
        insufficient = b->CreateOr(mBranchToLoopExit, b->CreateNot(cond));
    }

    b->CreateLikelyCondBr(test, target, insufficentIO);

    // When tracing blocking I/O, test all I/O streams but do not execute
    // the kernel if any stream is insufficient.
    if (mBranchToLoopExit) {
        b->SetInsertPoint(recordBlockedIO);
        recordBlockingIO(b, port);
        BasicBlock * const exitBlock = b->GetInsertBlock();
        b->CreateBr(target);

        b->SetInsertPoint(target);
        IntegerType * const boolTy = b->getInt1Ty();

        PHINode * const anyInsufficient = b->CreatePHI(boolTy, 2);
        anyInsufficient->addIncoming(insufficient, entryBlock);
        anyInsufficient->addIncoming(b->getTrue(), exitBlock);
        mBranchToLoopExit = anyInsufficient;
        PHINode * const halted = b->CreatePHI(boolTy, 2);
        halted->addIncoming(mHalted, entryBlock);
        halted->addIncoming(halting, exitBlock);
        mHalted = halted;

    } else {
        updatePHINodesForLoopExit(b, halting);
        b->SetInsertPoint(target);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updatePHINodesForLoopExit
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::updatePHINodesForLoopExit(BuilderRef b, Value * halting) {

    BasicBlock * const exitBlock = b->GetInsertBlock();
    mTerminatedPhi->addIncoming(mTerminatedInitially, exitBlock);
    mHasProgressedPhi->addIncoming(mAlreadyProgressedPhi, exitBlock);
    mTotalNumOfStrides->addIncoming(mCurrentNumOfStrides, exitBlock);
    mHaltingPhi->addIncoming(halting, exitBlock);

    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        mUpdatedProcessedPhi[i]->addIncoming(mAlreadyProcessedPhi[i], exitBlock);
        if (mUpdatedProcessedDeferredPhi[i]) {
            mUpdatedProcessedDeferredPhi[i]->addIncoming(mAlreadyProcessedDeferredPhi[i], exitBlock);
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        mUpdatedProducedPhi[i]->addIncoming(mAlreadyProducedPhi[i], exitBlock);
    }

}

}

#endif // IO_CALCULATION_LOGIC_HPP
