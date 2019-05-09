#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief reset
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename Vec>
inline void reset(Vec & vec, const unsigned n) {
    vec.resize(n);
    std::fill_n(vec.begin(), n, nullptr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief beginKernel
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::setActiveKernel(BuilderRef b, const unsigned index) {
    assert (index >= FirstKernel && index <= LastKernel);
    mKernelIndex = index;
    mKernel = getKernel(index);
    b->setKernel(mPipelineKernel);
    if (LLVM_LIKELY(mKernel->isStateful())) {
        Value * handle = nullptr;
        if (mKernel->hasFamilyName()) {
            handle = b->getScalarField(makeFamilyPrefix(index));
            handle = b->CreateBitCast(handle, mKernel->getSharedStateType()->getPointerTo());
        } else {
            handle = b->getScalarField(makeKernelName(index));
        }
        mKernel->setHandle(b, handle);
    }
    b->setKernel(mKernel);
}

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
 * @brief checkForSufficientInputData
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::checkForSufficientInputData(BuilderRef b, const unsigned inputPort) {
    // TODO: we could eliminate some checks if we can prove a particular input
    // must have enough data based on its already tested inputs and ignore
    // checking whether an input kernel is terminated if a stronger test has
    // already been done. Work out the logic for these tests globally.
    Value * const accessible = getAccessibleInputItems(b, inputPort);
    Value * const strideLength = getInputStrideLength(b, inputPort);
    Value * const required = addLookahead(b, inputPort, strideLength);
    const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Input, inputPort});
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(prefix + "_required", required);
    #endif
    Value * const hasEnough = b->CreateICmpUGE(accessible, required);
    Value * const sufficientInput = b->CreateOr(hasEnough, isClosed(b, inputPort));
    mAccessibleInputItems[inputPort] = accessible;
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
inline void PipelineCompiler::checkForSufficientOutputSpaceOrExpand(BuilderRef b, const unsigned outputPort) {

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
        if (LLVM_UNLIKELY(isa<DynamicBuffer>(buffer) || isa<LinearBuffer>(buffer))) {
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
inline Value * PipelineCompiler::reserveSufficientCapacity(BuilderRef b, const unsigned outputPort) {
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfAccessibleStrides
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getNumOfAccessibleStrides(BuilderRef b, const unsigned inputPort) {
    const Binding & input = getInputBinding(inputPort);
    const ProcessingRate & rate = input.getRate();
    Value * numOfStrides = nullptr;
    if (LLVM_UNLIKELY(rate.isPartialSum())) {
        numOfStrides = getMaximumNumOfPartialSumStrides(b, StreamPort{PortType::Input, inputPort});
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
inline Value * PipelineCompiler::getNumOfWritableStrides(BuilderRef b, const unsigned outputPort) {
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
inline void PipelineCompiler::calculateNonFinalItemCounts(BuilderRef b) {
    assert (mNumOfLinearStrides);
    const auto numOfInputs = in_degree(mKernelIndex, mBufferGraph);
    Vec<Value *> linearInputItems(numOfInputs);
    const auto numOfOutputs = out_degree(mKernelIndex, mBufferGraph);
    Vec<Value *> linearOutputItems(numOfOutputs);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        linearInputItems[i] = calculateNumOfLinearItems(b, StreamPort{PortType::Input, i});
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        linearOutputItems[i] = calculateNumOfLinearItems(b, StreamPort{PortType::Output, i});
    }
    BasicBlock * const exitBlock = b->GetInsertBlock();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        mLinearInputItemsPhi[i]->addIncoming(linearInputItems[i], exitBlock);
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        mLinearOutputItemsPhi[i]->addIncoming(linearOutputItems[i], exitBlock);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateFinalItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::calculateFinalItemCounts(BuilderRef b) {
    const auto numOfInputs = in_degree(mKernelIndex, mBufferGraph);
    Vec<Value *> accessibleItems(numOfInputs);
    const auto numOfOutputs = out_degree(mKernelIndex, mBufferGraph);
    Vec<Value *> pendingItems(numOfOutputs);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        accessibleItems[i] = addLookahead(b, i, mAccessibleInputItems[i]);
    }

    // Record the writable item counts but when determining the number of Fixed writable items calculate:

    //   CEILING(PRINCIPAL_OR_MIN(Accessible Item Count / Fixed Input Rate) * Fixed Output Rate)

    // TODO: ZeroExtend attribute must affect the notion of "min" here too.

    RateValue rateLCM(1);
    bool noPrincipalStream = true;
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = getInputBinding(i);
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
        const Binding & output = getOutputBinding(i);
        const ProcessingRate & rate = output.getRate();
        if (rate.isFixed()) {
            rateLCM = lcm(rateLCM, rate.getRate());
            hasFixedRateOutput = true;
        }
    }

    Value * minScaledInverseOfAccessibleInput = nullptr;
    if (hasFixedRateOutput) {
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const Binding & input = getInputBinding(i);
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
        const Binding & output = getOutputBinding(i);
        const ProcessingRate & rate = output.getRate();
        Value * writable = mWritableOutputItems[i];
        if (rate.isPartialSum()) {
            writable = mOutputStrideLength[i]; // getPartialSumItemCount(b, StreamPort{PortType::Output, i});
        } else if (rate.isFixed() && minScaledInverseOfAccessibleInput) {
            Value * const calculated = b->CreateCeilUDiv2(minScaledInverseOfAccessibleInput, rateLCM / rate.getRate());
            if (LLVM_UNLIKELY(mCheckAssertions)) {
                b->CreateAssert(b->CreateICmpULE(calculated, writable),
                                output.getName() +
                                ": final calculated fixed rate item count (%d) "
                                "exceeds maximum item count (%d)",
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
        pendingItems[i] = writable;
    }

    BasicBlock * const exitBlock = b->GetInsertBlock();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        mLinearInputItemsPhi[i]->addIncoming(accessibleItems[i], exitBlock);
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        mLinearOutputItemsPhi[i]->addIncoming(pendingItems[i], exitBlock);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareLocalZeroExtendSpace
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::prepareLocalZeroExtendSpace(BuilderRef b) {
    if (mHasZeroExtendedStream) {
        const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
        mZeroExtendBufferPhi = nullptr;
        Value * requiredSpace = nullptr;
        for (unsigned i = 0; i < numOfInputs; ++i) {
            if (mIsInputZeroExtended[i]) {
                const auto bufferVertex = getInputBufferVertex(i);
                const BufferNode & bn = mBufferGraph[bufferVertex];

                const Binding & input = getInputBinding(i);
                Value * requiredBytes = mLinearInputItemsPhi[i]; assert (requiredBytes);
                Constant * const BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());
                requiredBytes = b->CreateRoundUp(requiredBytes, BLOCK_WIDTH);
                if (bn.LookAhead) {
                    requiredBytes = b->CreateAdd(requiredBytes, b->getSize(bn.LookAhead));
                }
                requiredBytes = b->CreateMul(requiredBytes, bn.Buffer->getStreamSetCount(b.get()));

                const auto fieldWidth = input.getFieldWidth();
                if (fieldWidth < 8) {
                    requiredBytes = b->CreateUDiv(requiredBytes, b->getSize(8 / fieldWidth));
                } else if (fieldWidth > 8) {
                    requiredBytes = b->CreateMul(requiredBytes, b->getSize(fieldWidth / 8));
                }
                Constant * const ZERO = b->getSize(0);
                requiredBytes = b->CreateSelect(mIsInputZeroExtended[i], requiredBytes, ZERO);
                requiredSpace = b->CreateUMax(requiredSpace, requiredBytes);
            }
        }
        if (requiredSpace) {
            const auto prefix = makeKernelName(mKernelIndex);
            BasicBlock * const entry = b->GetInsertBlock();
            BasicBlock * const expandZeroExtension = b->CreateBasicBlock(prefix + "_expandZeroExtensionBuffer", mKernelTerminationCheck);
            BasicBlock * const executeKernel = b->CreateBasicBlock(prefix + "_executeKernelAfterZeroExtension", mKernelTerminationCheck);

            Value * const currentSpace = b->CreateLoad(mZeroExtendSpace);
            Value * const currentBuffer = b->CreateLoad(mZeroExtendBuffer);

            requiredSpace = b->CreateRoundUp(requiredSpace, b->getSize(b->getCacheAlignment()));

            Value * const largeEnough = b->CreateICmpUGE(currentSpace, requiredSpace);
            b->CreateLikelyCondBr(largeEnough, executeKernel, expandZeroExtension);

            b->SetInsertPoint(expandZeroExtension);
            assert (b->getCacheAlignment() >= (b->getBitBlockWidth() / 8));
            b->CreateFree(currentBuffer);
            Value * const newBuffer = b->CreateCacheAlignedMalloc(requiredSpace);
            b->CreateMemZero(newBuffer, requiredSpace, b->getCacheAlignment());
            b->CreateStore(requiredSpace, mZeroExtendSpace);
            b->CreateStore(newBuffer, mZeroExtendBuffer);
            b->CreateBr(executeKernel);

            b->SetInsertPoint(executeKernel);
            PHINode * const zeroBuffer = b->CreatePHI(b->getVoidPtrTy(), 2);
            zeroBuffer->addIncoming(currentBuffer, entry);
            zeroBuffer->addIncoming(newBuffer, expandZeroExtension);
            mZeroExtendBufferPhi = zeroBuffer;

        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForLastPartialSegment
 *
 * If this kernel is internally synchronized, determine whether there are any more segments.
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::checkForLastPartialSegment(BuilderRef b, Value * isFinal) {
    assert (b->getKernel() == mKernel);
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    mLastPartialSegment = nullptr;
    if (mKernel->hasAttribute(AttrId::InternallySynchronized)) {
        mLastPartialSegment = isFinal ? isFinal : b->getFalse();
        for (const auto i : mPortEvaluationOrder) {
            if (i < numOfInputs) {
                mLastPartialSegment = b->CreateOr(mLastPartialSegment, noMoreInputData(b, i));
            } else {
                mLastPartialSegment = b->CreateOr(mLastPartialSegment, noMoreOutputData(b, i - numOfInputs));
            }
        }
        assert (mLastPartialSegment);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief noMoreInputData
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::noMoreInputData(BuilderRef b, const unsigned inputPort) {
    const StreamSetBuffer * const buffer = getInputBuffer(inputPort);
    Value * const available = getLocallyAvailableItemCount(b, inputPort);
    Value * const pending = b->CreateAdd(mAlreadyProcessedPhi[inputPort], mLinearInputItemsPhi[inputPort]);
    Value * const accessible = buffer->getLinearlyAccessibleItems(b, pending, available);
    Value * const strideLength = getInputStrideLength(b, inputPort);
    Value * const required = addLookahead(b, inputPort, strideLength);
    return b->CreateICmpULE(required, accessible);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief noMoreOutputData
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::noMoreOutputData(BuilderRef b, const unsigned outputPort) {
    // TODO: not right for popcount rates. will have to branch (to account for # of ref items)
    // then check with the ref rate's pending offset.
    const StreamSetBuffer * const buffer = getOutputBuffer(outputPort);
    if (isa<DynamicBuffer>(buffer)) {
        return b->getFalse();
    }
    Value * const consumed = mConsumedItemCount[outputPort]; assert (consumed);
    Value * const pending = b->CreateAdd(mAlreadyProducedPhi[outputPort], mLinearOutputItemsPhi[outputPort]);
    Value * const writable = buffer->getLinearlyWritableItems(b, pending, consumed);
    Value * const required = getOutputStrideLength(b, outputPort);
    return b->CreateICmpULE(required, writable);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeKernelCall
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeKernelCall(BuilderRef b) {
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);

    mNumOfAddressableItemCount = 0;

    // TODO: add MProtect to buffers and their handles.

    // TODO: send in the # of output items we want in the external buffers

    b->setKernel(mPipelineKernel);

    ArgVec args;
    args.reserve(4 + (numOfInputs + numOfOutputs) * 4);
    if (LLVM_LIKELY(mKernel->isStateful())) {
        args.push_back(mKernel->getHandle()); assert (mKernel->getHandle());
    }
    if (LLVM_UNLIKELY(mKernel->hasThreadLocal())) {
        args.push_back(b->getScalarFieldPtr(makeKernelName(mKernelIndex) + KERNEL_THREAD_LOCAL_SUFFIX));
    }

    // If a kernel is internally synchronized, pass the iteration
    // count. Note: this is not the same as the pipeline's logical
    // segment number since unless we can prove that a kernel,
    // regardless of buffer state, will be called only once per
    // segment, we can only state the iteration count is >= the
    // segment number.

    // We may hit the same kernel with both threads simultaneously
    // before the first has finished updating?

    // Can we pass in the outer pipeline's synch num for this kernel
    // and increment it early? We'd need to pass in a temp one until
    // we know this is the last iteration of the segment.

    // That requires being able to know apriori what our resulting
    // state will be after completion for the input positions. We
    // can rely on the kernel itself to handle output synchronization.

//    if (mLastPartialSegment) {
//        Value * const iterationPtr = b->getScalarFieldPtr(makeKernelName(mKernelIndex) + ITERATION_COUNT_SUFFIX);
//        Value * const iterationCount = b->CreateAtomicFetchAndAdd(b->getSize(1), iterationPtr);
//        args.push_back(iterationCount);
//    }
    args.push_back(mNumOfLinearStrides); assert (mNumOfLinearStrides);

    RelationshipType prior_in{};

    for (const auto & e : make_iterator_range(in_edges(mKernelIndex, mBufferGraph))) {
        const BufferRateData & rt = mBufferGraph[e];
        assert (rt.Port.Type == PortType::Input);
        assert (prior_in < rt.Port);
        prior_in = rt.Port;

        if (LLVM_LIKELY(rt.Port.Reason == ReasonType::Explicit)) {

            // calculate the deferred processed item count
            PHINode * processed = nullptr;
            bool deferred = false;

            const auto i = rt.Port.Number;
            if (mAlreadyProcessedDeferredPhi[i]) {
                processed = mAlreadyProcessedDeferredPhi[i];
                deferred = true;
            } else {
                processed = mAlreadyProcessedPhi[i];
            }

            // calculate how many linear items are from the *deferred* position
            Value * inputItems = mLinearInputItemsPhi[i];
            if (deferred) {
                Value * diff = b->CreateSub(mAlreadyProcessedPhi[i], mAlreadyProcessedDeferredPhi[i]);
                inputItems = b->CreateAdd(inputItems, diff);
            }
            const auto buffer = source(e, mBufferGraph);
            const BufferNode & bn = mBufferGraph[buffer];
            const Binding & input = rt.Binding;
            assert ("input buffer type mismatch?" && (input.getType() == bn.Buffer->getBaseType()));

            args.push_back(epoch(b, input, bn.Buffer, processed, mIsInputZeroExtended[i]));
            mReturnedProcessedItemCountPtr[i] = addItemCountArg(b, input, deferred, processed, args);
            args.push_back(inputItems); assert (inputItems);

            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
                //args.push_back(getPositivePopCountArray(b, i));
                assert (false);
            }
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
                // args.push_back(getNegativePopCountArray(b, i));
                assert (false);
            }

        }
    }

    const auto canTerminate = mKernel->canSetTerminateSignal();

    RelationshipType prior_out{};
    for (const auto & e : make_iterator_range(out_edges(mKernelIndex, mBufferGraph))) {
        const BufferRateData & rt = mBufferGraph[e];
        assert (rt.Port.Reason == ReasonType::Explicit);
        assert (rt.Port.Type == PortType::Output);
        assert (prior_out < rt.Port);
        prior_out = rt.Port;
        const auto i = rt.Port.Number;

        PHINode * const produced = mAlreadyProducedPhi[i];
        const auto buffer = target(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[buffer];
        const Binding & output = rt.Binding;

        assert ("output buffer type mismatch?" && (output.getType() == bn.Buffer->getBaseType()));

        if (LLVM_LIKELY(bn.Type != BufferType::Managed)) {
            args.push_back(epoch(b, output, bn.Buffer, produced));
        }
        mReturnedProducedItemCountPtr[i] = addItemCountArg(b, output, canTerminate, produced, args);
        if (LLVM_LIKELY(bn.Type == BufferType::Managed)) {
            args.push_back(mConsumedItemCount[i]); assert (mConsumedItemCount[i]);
        } else {
            args.push_back(mLinearOutputItemsPhi[i]);  assert (mLinearOutputItemsPhi[i]);
        }
    }

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mPipelineKernel->getHandle(), CBuilder::Protect::NONE);
    }

    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeKernelName(mKernelIndex);
    b->CallPrintInt("* " + prefix + "_executing", mNumOfLinearStrides);
    #endif

    startCycleCounter(b, CycleCounter::BEFORE_KERNEL_CALL);

    Value * const doSegment = getDoSegmentFunction(b);
    if (mRethrowException) {
        BasicBlock * const invokeOk = b->CreateBasicBlock("", mKernelTerminationCheck);
        mTerminatedExplicitly = b->CreateInvoke(doSegment, invokeOk, mRethrowException, args);
        b->SetInsertPoint(invokeOk);
    } else {
        mTerminatedExplicitly = b->CreateCall(getDoSegmentFunction(b), args);
    }

    updateCycleCounter(b, CycleCounter::BEFORE_KERNEL_CALL, CycleCounter::AFTER_KERNEL_CALL);

    mUpdatedNumOfStrides = b->CreateAdd(mCurrentNumOfStrides, mNumOfLinearStrides);
    if (LLVM_LIKELY(!canTerminate)) {
        mTerminatedExplicitly = b->getFalse();
    }

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mPipelineKernel->getHandle(), CBuilder::Protect::WRITE);
    }

    // calculate or read the item counts (assuming this kernel did not terminate)
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = getInputBinding(i);
        const ProcessingRate & rate = input.getRate();
        if (rate.isFixed() || rate.isPartialSum()) {
            mProcessedItemCount[i] = b->CreateAdd(mAlreadyProcessedPhi[i], mLinearInputItemsPhi[i]);
            if (mAlreadyProcessedDeferredPhi[i]) {
                assert (mReturnedProcessedItemCountPtr[i]);
                mProcessedDeferredItemCount[i] = b->CreateLoad(mReturnedProcessedItemCountPtr[i]);
                if (LLVM_UNLIKELY(mCheckAssertions)) {
                    Value * const deferred = mProcessedDeferredItemCount[i];
                    Value * const processed = mProcessedItemCount[i];
                    Value * const isDeferred = b->CreateICmpULE(deferred, processed);
                    b->CreateAssert(isDeferred, input.getName() +
                                    ": deferred processed item count (%d) "
                                    "exceeds non-deferred (%d)",
                                    deferred, processed);
                }
            }
        } else if (rate.isBounded() || rate.isUnknown()) {
            mProcessedItemCount[i] = b->CreateLoad(mReturnedProcessedItemCountPtr[i]);
        }
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Input, i});
        b->CallPrintInt(prefix + "_processed'", mProcessedItemCount[i]);
        #endif
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = getOutputBinding(i);
        const ProcessingRate & rate = output.getRate();
        if (rate.isFixed() || rate.isPartialSum()) {
            mProducedItemCount[i] = b->CreateAdd(mAlreadyProducedPhi[i], mLinearOutputItemsPhi[i]);
        } else if (rate.isBounded() || rate.isUnknown()) {
            mProducedItemCount[i] = b->CreateLoad(mReturnedProducedItemCountPtr[i]);
        }
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, i});
        b->CallPrintInt(prefix + "_produced'", mProducedItemCount[i]);
        #endif
    }
    b->setKernel(mKernel);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternallySynchronizedArg
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addInternallySynchronizedArg(BuilderRef b, ArgVec & args) {
    if (mKernel->hasAttribute(AttrId::InternallySynchronized)) {
        Value * const iterationPtr = b->getScalarFieldPtr(makeKernelName(mKernelIndex) + ITERATION_COUNT_SUFFIX);
        Value * const iterationCount = b->CreateAtomicFetchAndAdd(b->getSize(1), iterationPtr);
        args.push_back(iterationCount);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addItemCountArg
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::addItemCountArg(BuilderRef b, const Binding & binding,
                                          const bool addressable,
                                          PHINode * const itemCount,
                                          ArgVec & args) {
    const ProcessingRate & rate = binding.getRate();
    if (LLVM_UNLIKELY(rate.isRelative())) {
        return nullptr;
    }
    Value * ptr = nullptr;
    if (addressable || isAddressable(binding)) {
        if (mNumOfAddressableItemCount == mAddressableItemCountPtr.size()) {
            BasicBlock * bb = b->GetInsertBlock();
            b->SetInsertPoint(mPipelineEntryBranch);
            AllocaInst * const aic = b->CreateAlloca(b->getSizeTy(), nullptr, "AddressableItemCount");
            b->SetInsertPoint(bb);
            mAddressableItemCountPtr.push_back(aic);
        }
        ptr = mAddressableItemCountPtr[mNumOfAddressableItemCount++];
        b->CreateStore(itemCount, ptr);
        args.push_back(ptr);
    } else if (isCountable(binding)) {
        args.push_back(itemCount);
    }
    return ptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadItemCountsOfCountableRateStreams
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::loadItemCountsOfCountableRateStreams(BuilderRef b) {
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; i++) {
        const Binding & input = getInputBinding(i);
        const ProcessingRate & rate = input.getRate();
        if (mReturnedProcessedItemCountPtr[i] && (rate.isFixed() || rate.isPopCount() || rate.isNegatedPopCount())) {
            mProcessedItemCount[i] = b->CreateLoad(mReturnedProcessedItemCountPtr[i]);
        }
    }
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const Binding & output = getOutputBinding(i);
        const ProcessingRate & rate = output.getRate();
        if (mReturnedProducedItemCountPtr[i] && (rate.isFixed() || rate.isPopCount() || rate.isNegatedPopCount())) {
            mProducedItemCount[i] = b->CreateLoad(mReturnedProducedItemCountPtr[i]);
        }
    }
    BasicBlock * const exitBlock = b->GetInsertBlock();
    for (unsigned i = 0; i < numOfInputs; i++) {
        mFinalProcessedPhi[i]->addIncoming(mProcessedItemCount[i], exitBlock);
    }
    for (unsigned i = 0; i < numOfOutputs; i++) {
        mFinalProducedPhi[i]->addIncoming(mProducedItemCount[i], exitBlock);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief clearUnwrittenOutputData
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::clearUnwrittenOutputData(BuilderRef b) {

    const auto blockWidth = b->getBitBlockWidth();
    const auto log2BlockWidth = floor_log2(blockWidth);
    Constant * const LOG_2_BLOCK_WIDTH = b->getSize(log2BlockWidth);
    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);
    Constant * const BLOCK_MASK = b->getSize(blockWidth - 1);

    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const StreamSetBuffer * const buffer = getOutputBuffer(i);
        if (LLVM_UNLIKELY(isa<ExternalBuffer>(buffer))) {
            continue;
        }
        const auto itemWidth = getItemWidth(buffer->getBaseType());
        // Determine the maximum lookahead dependency on this stream and zero fill
        // the appropriate number of additional blocks.
        unsigned maximumLookahead = 0;
        RateValue strideLength{0};
        const auto bufferVertex = getOutputBufferVertex(i);
        for (const auto & e : make_iterator_range(out_edges(bufferVertex, mBufferGraph))) {
            const BufferRateData & rd = mBufferGraph[e];
            const Binding & input = rd.Binding;
            strideLength = std::max(strideLength, rd.Maximum);
            if (LLVM_UNLIKELY(input.hasLookahead())) {
                maximumLookahead = std::max(maximumLookahead, input.getLookahead());
            }
        }

        const auto numOfBlocks = ceiling((strideLength * itemWidth) / blockWidth);
        const auto numOfLookaheadBlocks = ceiling((maximumLookahead * itemWidth) / blockWidth);
        const auto blocksToZero = numOfBlocks + numOfLookaheadBlocks;

        const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, i});
        Value * const produced = mFinalProducedPhi[i];
        Value * const blockIndex = b->CreateLShr(produced, LOG_2_BLOCK_WIDTH);
        Constant * const ITEM_WIDTH = b->getSize(itemWidth);
        Value * packIndex = nullptr;
        Value * maskOffset = b->CreateAnd(produced, BLOCK_MASK);
        if (itemWidth > 1) {
            Value * const position = b->CreateMul(maskOffset, ITEM_WIDTH);
            packIndex = b->CreateLShr(position, LOG_2_BLOCK_WIDTH);
            maskOffset = b->CreateAnd(position, BLOCK_MASK);
        }
        Value * const mask = b->CreateNot(b->bitblock_mask_from(maskOffset));
        BasicBlock * const maskLoop = b->CreateBasicBlock(prefix + "_zeroFillLoop", mKernelLoopExit);
        BasicBlock * const maskExit = b->CreateBasicBlock(prefix + "_zeroFillExit", mKernelLoopExit);
        Value * const numOfStreams = buffer->getStreamSetCount(b.get());
        Value * const baseAddress = buffer->getBaseAddress(b.get());
        BasicBlock * const entry = b->GetInsertBlock();
        b->CreateBr(maskLoop);

        b->SetInsertPoint(maskLoop);
        PHINode * const streamIndex = b->CreatePHI(b->getSizeTy(), 2);
        streamIndex->addIncoming(ZERO, entry);
        Value * ptr = nullptr;
        if (itemWidth > 1) {
            ptr = buffer->getStreamPackPtr(b.get(), baseAddress, streamIndex, blockIndex, packIndex);
        } else {
            ptr = buffer->getStreamBlockPtr(b.get(), baseAddress, streamIndex, blockIndex);
        }
        Value * const value = b->CreateBlockAlignedLoad(ptr);
        Value * const maskedValue = b->CreateAnd(value, mask);
        b->CreateBlockAlignedStore(maskedValue, ptr);
        if (itemWidth > 1) {
            // Since packs are laid out sequentially in memory, it will hopefully be cheaper to zero them out here
            // because they may be within the same cache line.
            DataLayout DL(b->getModule());
            Type * const intPtrTy = DL.getIntPtrType(ptr->getType());
            Value * const nextPackIndex = b->CreateAdd(packIndex, ONE);
            Value * const start = buffer->getStreamPackPtr(b.get(), baseAddress, streamIndex, blockIndex, nextPackIndex);
            Value * const startInt = b->CreatePtrToInt(start, intPtrTy);
            Value * const end = buffer->getStreamPackPtr(b.get(), baseAddress, streamIndex, blockIndex, b->getSize(itemWidth));
            Value * const endInt = b->CreatePtrToInt(end, intPtrTy);
            Value * const remainingPackBytes = b->CreateSub(endInt, startInt);
            #ifdef PRINT_DEBUG_MESSAGES
            b->CallPrintInt(prefix + "_zeroFill_remainingPackBytes", remainingPackBytes);
            #endif
            b->CreateMemZero(start, remainingPackBytes, blockWidth / 8);
        }
        Value * const nextStreamIndex = b->CreateAdd(streamIndex, ONE);
        BasicBlock * const maskLoopExit = b->GetInsertBlock();
        streamIndex->addIncoming(nextStreamIndex, maskLoopExit);
        Value * const notDone = b->CreateICmpNE(nextStreamIndex, numOfStreams);
        b->CreateCondBr(notDone, maskLoop, maskExit);

        b->SetInsertPoint(maskExit);
        // Zero out any blocks we could potentially touch
        if (blocksToZero > 1) {
            Value * const nextBlockIndex = b->CreateAdd(blockIndex, ONE);
            Constant * const MAX_BLOCKS = b->getSize(blocksToZero);
            Value * const ptr = buffer->getStreamBlockPtr(b.get(), baseAddress, ZERO, nextBlockIndex);
            Value * remainingBlocks = b->CreateSub(MAX_BLOCKS, nextBlockIndex);
            Value * const allCleared = b->CreateICmpULT(MAX_BLOCKS, nextBlockIndex);
            remainingBlocks = b->CreateSelect(allCleared, ZERO, remainingBlocks);
            remainingBlocks = b->CreateMul(numOfStreams, remainingBlocks);
            Constant * const LOG_2_BYTES_PER_BLOCK = b->getSize(floor_log2(blockWidth / 8));
            Value * const remainingBytes = b->CreateShl(remainingBlocks, LOG_2_BYTES_PER_BLOCK);
            #ifdef PRINT_DEBUG_MESSAGES
            b->CallPrintInt(prefix + "_zeroFill_remainingBytes", remainingBytes);
            #endif
            b->CreateMemZero(ptr, remainingBytes, blockWidth / 8);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeFullyProcessedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::computeFullyProcessedItemCounts(BuilderRef b) {
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = getInputBinding(i);
        Value * processed = nullptr;
        if (mUpdatedProcessedDeferredPhi[i]) {
            processed = mUpdatedProcessedDeferredPhi[i];
        } else {
            processed = mUpdatedProcessedPhi[i];
        }
        processed = truncateBlockSize(b, input, processed);
        mFullyProcessedItemCount[i] = processed;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeFullyProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::computeFullyProducedItemCounts(BuilderRef b) {

    // TODO: we only need to consider the blocksize attribute if it's possible this
    // stream could be read before being fully written. This might occur if one of
    // it's consumers has a non-Fixed rate that does not have a matching BlockSize
    // attribute.

    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = getOutputBinding(i);
        Value * produced = truncateBlockSize(b, output, mUpdatedProducedPhi[i]);
        mFullyProducedItemCount[i]->addIncoming(produced, mKernelLoopExitPhiCatch);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getInputStrideLength(BuilderRef b, const unsigned inputPort) {
    assert (inputPort < mInputStrideLength.size());
    if (mInputStrideLength[inputPort]) {
        return mInputStrideLength[inputPort];
    } else {
        Value * const strideLength = getInitialStrideLength(b, StreamPort{PortType::Input, inputPort});
        mInputStrideLength[inputPort] = strideLength;
        return strideLength;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getOutputStrideLength(BuilderRef b, const unsigned outputPort) {
    assert (outputPort < mOutputStrideLength.size());
    if (mOutputStrideLength[outputPort]) {
        return mOutputStrideLength[outputPort];
    } else {
        Value * const strideLength = getInitialStrideLength(b, StreamPort{PortType::Output, outputPort});
        mOutputStrideLength[outputPort] = strideLength;
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
 * @brief getInitialStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getInitialStrideLength(BuilderRef b, const StreamPort port) {
    const auto & binding = getBinding(port);
    const auto & rate = binding.getRate();
    if (LLVM_LIKELY(rate.isFixed() || rate.isBounded())) {
        const ProcessingRate & rate = binding.getRate();
        const RateValue ub = rate.getUpperBound() * mKernel->getStride();
        if (LLVM_UNLIKELY(ub.denominator() != 1)) {
            std::string tmp;
            raw_string_ostream out(tmp);
            out << mKernel->getName() << "." << binding.getName()
                << ": rate upper-bound is not a multiple of kernel stride.";
            report_fatal_error(out.str());
        }
        return b->getSize(ub.numerator());
    } else if (LLVM_UNLIKELY(rate.isPartialSum())) {
        return getPartialSumItemCount(b, port);
    } else if (LLVM_UNLIKELY(rate.isPopCount() || rate.isNegatedPopCount())) {
        report_fatal_error("internal error: PopCount and NegatedPopCount rates should have been converted to PartialSums.");
    } else if (rate.isRelative()) {
        Value * const baseRate = getInitialStrideLength(b, getReference(port));
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
    } else if (rate.isPartialSum()) {
        return getPartialSumItemCount(b, port, mNumOfLinearStrides);
    } else if (rate.isRelative()) {
        Value * const baseCount = calculateNumOfLinearItems(b, getReference(port));
        return b->CreateMul2(baseCount, rate.getRate());
    }
    llvm_unreachable("unexpected rate type");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getTotalItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getLocallyAvailableItemCount(BuilderRef /* b */, const unsigned inputPort) const {
    const auto bufferVertex = getInputBufferVertex(inputPort);
    return mLocallyAvailableItems[getBufferIndex(bufferVertex)];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addLookahead
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::addLookahead(BuilderRef b, const unsigned inputPort, Value * const itemCount) const {
    Constant * const lookAhead = getLookahead(b, inputPort);
    if (LLVM_LIKELY(lookAhead == nullptr)) {
        return itemCount;
    }
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Input, inputPort});
    b->CallPrintInt(prefix + "_lookAhead", lookAhead);
    #endif
    return b->CreateAdd(itemCount, lookAhead);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief subtractLookahead
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::subtractLookahead(BuilderRef b, const unsigned inputPort, Value * const itemCount) {
    Constant * const lookAhead = getLookahead(b, inputPort);
    if (LLVM_LIKELY(lookAhead == nullptr)) {
        return itemCount;
    }
    Value * const reducedItemCount = b->CreateSub(itemCount, lookAhead);
    return b->CreateSelect(isClosed(b, inputPort), itemCount, reducedItemCount);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getLookahead
 ** ------------------------------------------------------------------------------------------------------------- */
Constant * PipelineCompiler::getLookahead(BuilderRef b, const unsigned inputPort) const {
    const Binding & input = getInputBinding(inputPort);
    if (LLVM_UNLIKELY(input.hasLookahead())) {
        return b->getSize(input.getLookahead());
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief maskBlockSize
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::truncateBlockSize(BuilderRef b, const Binding & binding, Value * itemCount) const {
    // TODO: if we determine all of the inputs of a stream have a blocksize attribute, or the output has one,
    // we can skip masking it on input



    if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::BlockSize))) {
        // If the input rate has a block size attribute then --- for the purpose of determining how many
        // items have been consumed --- we consider a stream set to be fully processed when an entire
        // stride has been processed.
        Constant * const BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());
        Value * const maskedItemCount = b->CreateAnd(itemCount, ConstantExpr::getNeg(BLOCK_WIDTH));
        Value * const terminated = hasKernelTerminated(b, mKernelIndex);
        itemCount = b->CreateSelect(terminated, itemCount, maskedItemCount);
    }
    return itemCount;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getFunctionFromKernelState(BuilderRef b, Type * const type, const std::string & suffix) const {
    const auto prefix = makeFamilyPrefix(mKernelIndex);
    b->setKernel(mPipelineKernel);
    Value * const funcPtr = b->getScalarField(prefix + suffix);
    if (LLVM_UNLIKELY(mCheckAssertions)) {
        b->CreateAssert(funcPtr, prefix + suffix + " is null");
    }
    b->setKernel(mKernel);
    return b->CreateBitCast(funcPtr, type);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationFunction
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getInitializeFunction(BuilderRef b) const {
    Function * const init = mKernel->getInitializeFunction(b);
    if (mKernel->hasFamilyName()) {
        return getFunctionFromKernelState(b, init->getType(), INITIALIZE_FUNCTION_POINTER_SUFFIX);
    }
    return init;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationThreadLocalFunction
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getInitializeThreadLocalFunction(BuilderRef b) const {
    Function * const init = mKernel->getInitializeThreadLocalFunction(b);
    if (mKernel->hasFamilyName()) {
        return getFunctionFromKernelState(b, init->getType(), INITIALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX);
    }
    return init;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDoSegmentFunction
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getDoSegmentFunction(BuilderRef b) const {
    Function * const doSegment = mKernel->getDoSegmentFunction(b);
    if (mKernel->hasFamilyName()) {
        return getFunctionFromKernelState(b, doSegment->getType(), DO_SEGMENT_FUNCTION_POINTER_SUFFIX);
    }
    return doSegment;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationThreadLocalFunction
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getFinalizeThreadLocalFunction(BuilderRef b) const {
    Function * const init = mKernel->getFinalizeThreadLocalFunction(b);
    if (mKernel->hasFamilyName()) {
        return getFunctionFromKernelState(b, init->getType(), FINALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX);
    }
    return init;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalizeFunction
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getFinalizeFunction(BuilderRef b) const {
    Function * const term = mKernel->getFinalizeFunction(b);
    if (mKernel->hasFamilyName()) {
        return getFunctionFromKernelState(b, term->getType(), FINALIZE_FUNCTION_POINTER_SUFFIX);
    }
    return term;
}

#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifyInputItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::verifyInputItemCount(BuilderRef b, Value * processed, const unsigned inputPort) const {
    if (LLVM_UNLIKELY(mCheckAssertions)) {
        const Binding & input = getInputBinding(inputPort);
        Value * const expected = b->CreateAdd(mAlreadyProcessedPhi[inputPort], mLinearInputItemsPhi[inputPort]);
        itemCountSanityCheck(b, input, "processed", processed, expected);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifyOutputItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::verifyOutputItemCount(BuilderRef b, Value * produced, const unsigned outputPort) const {
    if (LLVM_UNLIKELY(mCheckAssertions)) {
        const Binding & output = getOutputBinding(outputPort);
        Value * const expected = b->CreateAdd(mAlreadyProducedPhi[outputPort], mLinearOutputItemsPhi[outputPort]);
        itemCountSanityCheck(b, output, "produced", produced, expected);
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief itemCountSanityCheck
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::itemCountSanityCheck(BuilderRef b, const Binding & binding,
                                            const std::string & label,
                                            Value * const itemCount, Value * const expected) const {

    const auto prefix = makeBufferName(mKernelIndex, binding);
    const auto lb = mKernel->getLowerBound(binding);
    if (lb > 0 && !binding.hasAttribute(AttrId::Deferred)) {
        Constant * const strideSize = b->getSize(ceiling(lb * mKernel->getStride()));
        Value * hasEnough = b->CreateICmpULE(itemCount, strideSize);
        hasEnough = b->CreateOr(hasEnough, mTerminationExplicitly);
        b->CreateAssert(hasEnough, prefix + " " + label + " fewer items than expected");
    }
    Value * const withinBounds = b->CreateICmpULE(itemCount, expected);
    b->CreateAssert(withinBounds, prefix + " " + label + " more items than expected");

}

#endif

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resetMemoizedFields
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::resetMemoizedFields() {
    const auto numOfInputs = in_degree(mKernelIndex, mBufferGraph);
    reset(mIsInputClosed, numOfInputs);
    reset(mIsInputZeroExtended, numOfInputs);
    reset(mInitiallyProcessedItemCount, numOfInputs);
    reset(mInitiallyProcessedDeferredItemCount, numOfInputs);
    reset(mAlreadyProcessedPhi, numOfInputs);
    reset(mAlreadyProcessedDeferredPhi, numOfInputs);
    reset(mInputStrideLength, numOfInputs);
    reset(mAccessibleInputItems, numOfInputs);
    reset(mLinearInputItemsPhi, numOfInputs);
    reset(mReturnedProcessedItemCountPtr, numOfInputs);
    reset(mProcessedItemCount, numOfInputs);
    reset(mProcessedDeferredItemCount, numOfInputs);
    reset(mFinalProcessedPhi, numOfInputs);
    reset(mUpdatedProcessedPhi, numOfInputs);
    reset(mUpdatedProcessedDeferredPhi, numOfInputs);
    reset(mFullyProcessedItemCount, numOfInputs);
    const auto numOfOutputs = out_degree(mKernelIndex, mBufferGraph);
    reset(mInitiallyProducedItemCount, numOfOutputs);
    reset(mInitiallyProducedDeferredItemCount, numOfOutputs);
    reset(mAlreadyProducedPhi, numOfOutputs);
    reset(mAlreadyProducedDeferredPhi, numOfOutputs);
    reset(mOutputStrideLength, numOfOutputs);
    reset(mWritableOutputItems, numOfOutputs);
    reset(mConsumedItemCount, numOfOutputs);
    reset(mLinearOutputItemsPhi, numOfOutputs);
    reset(mReturnedProducedItemCountPtr, numOfOutputs);
    reset(mProducedItemCount, numOfOutputs);
    reset(mProducedDeferredItemCount, numOfOutputs);
    reset(mFinalProducedPhi, numOfOutputs);
    reset(mUpdatedProducedPhi, numOfOutputs);
    reset(mUpdatedProducedDeferredPhi, numOfOutputs);
    reset(mFullyProducedItemCount, numOfOutputs);
}


}
