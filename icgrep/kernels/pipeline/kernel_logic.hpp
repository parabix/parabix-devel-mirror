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
 * @brief determineNumOfLogicalStrides
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::checkForSufficientInputDataAndOutputSpace(BuilderRef b) {
    assert (b->getKernel() == mKernel);
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    for (const auto i : mPortEvaluationOrder) {
        if (i < numOfInputs) {
            checkForSufficientInputData(b, i);
        } else {
            checkForSufficientOutputSpaceOrExpand(b, i - numOfInputs);
        }
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

    Value * const accessible = getAccessibleInputItems(b, inputPort, true);
    Value * const strideLength = getInputStrideLength(b, inputPort);
    Value * const requiredInput = addLookahead(b, inputPort, strideLength);
    const Binding & input = getInputBinding(inputPort);
    const auto prefix = makeBufferName(mKernelIndex, input);
    Value * const hasEnough = b->CreateICmpUGE(accessible, requiredInput);
    Value * const sufficientInput = b->CreateOr(hasEnough, isClosed(b, inputPort));
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(prefix + "_accessible", accessible);
    b->CallPrintInt(prefix + "_requiredInput", requiredInput);
    b->CallPrintInt(prefix + "_sufficientInput", sufficientInput);
    #endif
    mAccessibleInputItems[inputPort] = accessible;

    Value * halting = mHalted;
    for (const auto & e : make_iterator_range(in_edges(mKernelIndex, mPipelineIOGraph))) {
        if (LLVM_LIKELY(mPipelineIOGraph[e] == inputPort)) {
            halting = b->getTrue();
            break;
        }
    }

    BasicBlock * const target = b->CreateBasicBlock(prefix + "_hasInputData", mKernelLoopCall);
    branchToTargetOrLoopExit(b, sufficientInput, target, halting);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getAccessibleInputItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getAccessibleInputItems(BuilderRef b, const unsigned inputPort, const bool addFacsimile) {
    assert (inputPort < mAccessibleInputItems.size());
    const StreamSetBuffer * const buffer = getInputBuffer(inputPort);
    Value * const available = getLocallyAvailableItemCount(b, inputPort);
    Value * const processed = mAlreadyProcessedPhi[inputPort];
    ConstantInt * facsimile = nullptr;
    if (addFacsimile) {
        const auto size = getFacsimile(getInputBufferVertex(inputPort));
        if (size) {
            facsimile = b->getSize(size);
        }
    }

    const Binding & input = getInputBinding(inputPort);
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelIndex, input);
    b->CallPrintInt(prefix + "_available", available);
    b->CallPrintInt(prefix + "_processed", processed);
    #endif

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

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * sanityCheck = b->CreateICmpULE(processed, available);
        if (mIsInputZeroExtended[inputPort]) {
            sanityCheck = b->CreateOr(mIsInputZeroExtended[inputPort], sanityCheck);
        }
        b->CreateAssert(sanityCheck,
                        mKernel->getName() + "_" + input.getName() +
                        ": processed count exceeds total count");
    }
    return accessible;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForSufficientOutputSpaceOrExpand
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::checkForSufficientOutputSpaceOrExpand(BuilderRef b, const unsigned outputPort) {
    // If the buffer is managed by the kernel, ignore it
    if (LLVM_UNLIKELY(getOutputBufferType(outputPort) == BufferType::Managed)) {
        return;
    }
    const StreamSetBuffer * const buffer = getOutputBuffer(outputPort);
    Value * const writable = getWritableOutputItems(b, outputPort, true);
    Value * const strideLength = getOutputStrideLength(b, outputPort);
    const Binding & output = getOutputBinding(outputPort);
    const auto prefix = makeBufferName(mKernelIndex, output);
    Value * const hasEnough = b->CreateICmpULE(strideLength, writable, prefix + "_hasEnough");
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(prefix + "_writable", writable);
    b->CallPrintInt(prefix + "_requiredOutput", strideLength);
    b->CallPrintInt(prefix + "_hasEnough", hasEnough);
    #endif
    BasicBlock * const target = b->CreateBasicBlock(prefix + "_hasOutputSpace", mKernelLoopCall);
    mWritableOutputItems[outputPort] = writable;

    if (LLVM_UNLIKELY(isa<DynamicBuffer>(buffer))) {
        expandOutputBuffer(b, outputPort, hasEnough, target);
    } else {
        Value * halting = mHalted;
        for (const auto & e : make_iterator_range(out_edges(mKernelIndex, mPipelineIOGraph))) {
            if (LLVM_LIKELY(mPipelineIOGraph[e] == outputPort)) {
                halting = b->getTrue();
                break;
            }
        }
        branchToTargetOrLoopExit(b, hasEnough, target, halting);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief branchToTargetOrLoopExit
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::branchToTargetOrLoopExit(BuilderRef b, Value * const cond, BasicBlock * const target, Value * const halting) {

    BasicBlock * const exitBlock = b->GetInsertBlock();
    mTerminatedPhi->addIncoming(mTerminatedInitially, exitBlock);
    mHasProgressedPhi->addIncoming(mAlreadyProgressedPhi, exitBlock);
    mHaltingPhi->addIncoming(halting, exitBlock);

    b->CreateLikelyCondBr(cond, target, mKernelLoopExit);
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
    b->SetInsertPoint(target);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getWritableOutputItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getWritableOutputItems(BuilderRef b, const unsigned outputPort, const bool addOverflow) {
    assert (outputPort < mWritableOutputItems.size());
    const Binding & output = getOutputBinding(outputPort);
    const StreamSetBuffer * const buffer = getOutputBuffer(outputPort);
    Value * const produced = mAlreadyProducedPhi[outputPort]; assert (produced);
    Value * const consumed = mConsumedItemCount[outputPort]; assert (consumed);
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelIndex, output);
    b->CallPrintInt(prefix + "_produced", produced);
    b->CallPrintInt(prefix + "_consumed", consumed);
    #endif
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const sanityCheck = b->CreateICmpULE(consumed, produced);
        b->CreateAssert(sanityCheck,
                        mKernel->getName() + "_" + output.getName() +
                        ": consumed count exceeds produced count");
    }
    ConstantInt * copyBack = nullptr;
    if (addOverflow) {
        const auto size = getCopyBack(getOutputBufferVertex(outputPort));
        if (size) {
            copyBack = b->getSize(size - 1);
        }
    }
    return buffer->getLinearlyWritableItems(b, produced, consumed, copyBack);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determineNumOfLinearStrides
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::determineNumOfLinearStrides(BuilderRef b) {
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    mNumOfLinearStrides = nullptr;
    for (const auto i : mPortEvaluationOrder) {
        Value * strides = nullptr;
        if (i < numOfInputs) {
            strides = getNumOfAccessibleStrides(b, i);
        } else {
            strides = getNumOfWritableStrides(b, i - numOfInputs);
        }
        mNumOfLinearStrides = b->CreateUMin(mNumOfLinearStrides, strides);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfAccessibleStrides
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getNumOfAccessibleStrides(BuilderRef b, const unsigned inputPort) {
    const Binding & input = getInputBinding(inputPort);
    Value * numOfStrides = nullptr;
    Value * const accessible = mAccessibleInputItems[inputPort];
    if (LLVM_UNLIKELY(isAnyPopCount(input))) {
        Value * const inBuffer = getAccessibleInputItems(b, inputPort, false);
        Constant * const lookAhead = getLookahead(b, inputPort);
        numOfStrides = getMaximumNumOfPopCountStrides(b, input, inBuffer, accessible, lookAhead);
    } else {
        Value * const strideLength = getInputStrideLength(b, inputPort);
        numOfStrides = b->CreateUDiv(subtractLookahead(b, inputPort, accessible), strideLength);
    }
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelIndex, input);
    #endif
    // When zero extended, this stream does not affect the linear strides calculation
    if (mIsInputZeroExtended[inputPort]) {
        numOfStrides = b->CreateSelect(mIsInputZeroExtended[inputPort], mNumOfLinearStrides, numOfStrides);
    }
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt("> " + prefix + "_numOfStrides", numOfStrides);
    #endif
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const closed = isClosed(b, inputPort);
        Value * const hasEnough = b->CreateIsNotNull(numOfStrides);
        Value * const progress = b->CreateOr(hasEnough, closed);
        b->CreateAssert(progress,
                        mKernel->getName() + "." + input.getName() +
                        ": unexpected end of input stream");
    }
    return numOfStrides;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfWritableStrides
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getNumOfWritableStrides(BuilderRef b, const unsigned outputPort) {
    Value * numOfStrides = nullptr;
    if (LLVM_LIKELY(getOutputBufferType(outputPort) != BufferType::Managed)) {
        const Binding & output = getOutputBinding(outputPort);
        Value * const writable = mWritableOutputItems[outputPort];
        if (LLVM_UNLIKELY(isAnyPopCount(output))) {
            Value * const inBuffer = getWritableOutputItems(b, outputPort, false);
            numOfStrides = getMaximumNumOfPopCountStrides(b, output, inBuffer, writable);
        } else {
            Value * const strideLength = getOutputStrideLength(b, outputPort);
            numOfStrides = b->CreateUDiv(writable, strideLength);
        }
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelIndex, output);
        b->CallPrintInt("< " + prefix + "_numOfStrides", numOfStrides);
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
        linearInputItems[i] = calculateNumOfLinearItems(b, getInputBinding(i));
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        linearOutputItems[i] = calculateNumOfLinearItems(b, getOutputBinding(i));
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
        if (LLVM_UNLIKELY(isAnyPopCount(output))) {
            writable = getMinimumNumOfLinearPopCountItems(b, output);
        } else if (rate.isFixed() && minScaledInverseOfAccessibleInput) {
            writable = b->CreateCeilUDiv2(minScaledInverseOfAccessibleInput, rateLCM / rate.getUpperBound());
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
 * @brief calculateBufferExpansionSize
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::calculateBufferExpansionSize(BuilderRef b, const unsigned outputPort) {
    Value * const produced = mAlreadyProducedPhi[outputPort];
    Value * const consumed = mConsumedItemCount[outputPort];
    Value * const unconsumed = b->CreateSub(produced, consumed);
    Value * const strideLength = getOutputStrideLength(b, outputPort);
    return b->CreateAdd(unconsumed, strideLength);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief expandOutputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::expandOutputBuffer(BuilderRef b, const unsigned outputPort, Value * const hasEnough, BasicBlock * const target) {
    const Binding & output = getOutputBinding(outputPort);
    const auto prefix = makeBufferName(mKernelIndex, output);
    BasicBlock * const expand = b->CreateBasicBlock(prefix + "_expandOutputBuffer", target);
    BasicBlock * const entryBlock = b->GetInsertBlock();
    Value * const currentWritableItems = mWritableOutputItems[outputPort];
    b->CreateLikelyCondBr(hasEnough, target, expand);

    b->SetInsertPoint(expand);
    Value * const size = calculateBufferExpansionSize(b, outputPort);
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(prefix + "_expandingToSize", size);
    #endif
    const StreamSetBuffer * const buffer = cast<DynamicBuffer>(getOutputBuffer(outputPort));
    buffer->setCapacity(b.get(), size);
    Value * const newWritableItems = getWritableOutputItems(b, outputPort, true);
    BasicBlock * const expandEnd = b->GetInsertBlock();
    b->CreateBr(target);

    b->SetInsertPoint(target);
    PHINode * const writablePhi = b->CreatePHI(b->getSizeTy(), 2);
    writablePhi->addIncoming(currentWritableItems, entryBlock);
    writablePhi->addIncoming(newWritableItems, expandEnd);
    mWritableOutputItems[outputPort] = writablePhi;
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
                if (bn.Fasimile) {
                    requiredBytes = b->CreateAdd(requiredBytes, b->getSize(bn.Fasimile));
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
 * @brief writeKernelCall
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeKernelCall(BuilderRef b) {
    const auto numOfInputs = getNumOfStreamInputs(mKernelIndex);
    const auto numOfOutputs = getNumOfStreamOutputs(mKernelIndex);

    mNumOfAddressableItemCount = 0;

    // TODO: add MProtect to buffers and their handles.

    // TODO: send in the # of output items we want in the external buffers

    b->setKernel(mPipelineKernel);

    Vec<Value *, 64> args;
    args.reserve(4 + (numOfInputs + numOfOutputs) * 4);
    if (LLVM_LIKELY(mKernel->isStateful())) {
        args.push_back(mKernel->getHandle()); assert (mKernel->getHandle());
    }
    args.push_back(mNumOfLinearStrides); assert (mNumOfLinearStrides);
    for (unsigned i = 0; i < numOfInputs; ++i) {

        // calculate the deferred processed item count
        PHINode * processed = nullptr;
        bool deferred = false;

        const Binding & input = getInputBinding(i);
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
        args.push_back(epoch(b, input, getInputBuffer(i), processed, mIsInputZeroExtended[i]));
        mReturnedProcessedItemCountPtr[i] =
            addItemCountArg(b, input, deferred, processed, args);
        args.push_back(inputItems); assert (inputItems);

        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
            args.push_back(getPositivePopCountArray(b, i));
        }
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
            args.push_back(getNegativePopCountArray(b, i));
        }
    }

    const auto canTerminate = mKernel->canSetTerminateSignal();

    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto nonManaged = getOutputBufferType(i) != BufferType::Managed;
        const Binding & output = getOutputBinding(i);
        PHINode * const produced = mAlreadyProducedPhi[i];
        if (LLVM_LIKELY(nonManaged)) {
            args.push_back(epoch(b, output, getOutputBuffer(i), produced));
        }
        mReturnedProducedItemCountPtr[i] =
            addItemCountArg(b, output, canTerminate, produced, args);
        if (LLVM_LIKELY(nonManaged)) {
            args.push_back(mLinearOutputItemsPhi[i]);
        } else {
            args.push_back(mConsumedItemCount[i]); assert (mConsumedItemCount[i]);
        }
    }

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mPipelineKernel->getHandle(), CBuilder::Protect::NONE);
    }

    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeKernelName(mKernelIndex);
    b->CallPrintInt("* " + prefix + "_executing", mNumOfLinearStrides);
    #endif

    mTerminatedExplicitly = b->CreateCall(getDoSegmentFunction(b), args);
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
        if (rate.isFixed() || rate.isPopCount() || rate.isNegatedPopCount()) {
            mProcessedItemCount[i] = b->CreateAdd(mAlreadyProcessedPhi[i], mLinearInputItemsPhi[i]);
            if (mAlreadyProcessedDeferredPhi[i]) {
                assert (mReturnedProcessedItemCountPtr[i]);
                mProcessedDeferredItemCount[i] = b->CreateLoad(mReturnedProcessedItemCountPtr[i]);
                if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
                    const auto prefix = makeBufferName(mKernelIndex, input);
                    Value * const isDeferred = b->CreateICmpULE(mProcessedDeferredItemCount[i], mProcessedItemCount[i]);
                    b->CreateAssert(isDeferred, prefix + ": deferred processed item count exceeds non-deferred");
                }
            }
        } else if (rate.isBounded() || rate.isUnknown()) {
            mProcessedItemCount[i] = b->CreateLoad(mReturnedProcessedItemCountPtr[i]);
        }
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelIndex, input);
        b->CallPrintInt(prefix + "_processed'", mProcessedItemCount[i]);
        #endif
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = getOutputBinding(i);
        const ProcessingRate & rate = output.getRate();
        if (rate.isFixed() || rate.isPopCount() || rate.isNegatedPopCount()) {
            mProducedItemCount[i] = b->CreateAdd(mAlreadyProducedPhi[i], mLinearOutputItemsPhi[i]);
        } else if (rate.isBounded() || rate.isUnknown()) {
            mProducedItemCount[i] = b->CreateLoad(mReturnedProducedItemCountPtr[i]);
        }
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelIndex, output);
        b->CallPrintInt(prefix + "_produced'", mProducedItemCount[i]);
        #endif
    }
    b->setKernel(mKernel);

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addKernelCallArgument
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::addItemCountArg(BuilderRef b, const Binding & binding,
                                          const bool addressable,
                                          PHINode * const itemCount,
                                          Vec<Value *, 64> & args) {
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
        const Binding & output = getOutputBinding(i);
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
        const auto numOfBlocks = ceiling(strideLength) >> log2BlockWidth;
        const auto numOfLookaheadBlocks = ((maximumLookahead * itemWidth) + (blockWidth - 1)) >> log2BlockWidth;
        const auto blocksToZero = numOfBlocks + numOfLookaheadBlocks;

        const auto prefix = makeBufferName(mKernelIndex, output);
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
        BasicBlock * const entry = b->GetInsertBlock();
        BasicBlock * const maskLoop = b->CreateBasicBlock(prefix + "_zeroFillLoop", mKernelLoopExit);
        BasicBlock * const maskExit = b->CreateBasicBlock(prefix + "_zeroFillExit", mKernelLoopExit);
        Value * const numOfStreams = buffer->getStreamSetCount(b.get());
        Value * const baseAddress = buffer->getBaseAddress(b.get());
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
            Constant * const MAX_PACK_INDEX = b->getSize(itemWidth - 1);
            Value * const remainingPackBytes = b->CreateShl(b->CreateSub(MAX_PACK_INDEX, packIndex), LOG_2_BLOCK_WIDTH);
            #ifdef PRINT_DEBUG_MESSAGES
            b->CallPrintInt(prefix + "_zeroFill_remainingPackBytes", remainingPackBytes);
            #endif
            Value * const nextPackIndex = b->CreateAdd(packIndex, ONE);
            Value * const ptr = buffer->getStreamPackPtr(b.get(), baseAddress, streamIndex, blockIndex, nextPackIndex);
            b->CreateMemZero(ptr, remainingPackBytes, blockWidth / 8);
        }
        Value * const nextStreamIndex = b->CreateAdd(streamIndex, ONE);
        streamIndex->addIncoming(nextStreamIndex, maskLoop);
        Value * const notDone = b->CreateICmpNE(nextStreamIndex, numOfStreams);
        b->CreateCondBr(notDone, maskLoop, maskExit);

        b->SetInsertPoint(maskExit);
        // Zero out any blocks we could potentially touch
        if (blocksToZero > 1) {
            Constant * const MAX_BLOCKS = b->getSize(blocksToZero);
            Value * const nextBlockIndex = b->CreateAdd(blockIndex, ONE);
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
 * @brief getInitialStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getInitialStrideLength(BuilderRef b, const StreamPort port) {
    const auto & binding = getBinding(port);
    const auto & rate = binding.getRate();
    if (LLVM_LIKELY(rate.isFixed() || rate.isBounded())) {
        return b->getSize(ceiling(mKernel->getUpperBound(binding) * mKernel->getStride()));
    } else if (LLVM_UNLIKELY(rate.isPopCount() || rate.isNegatedPopCount())) {
        return getMinimumNumOfLinearPopCountItems(b, binding);
    } else if (rate.isRelative()) {
        auto refPort = mKernel->getStreamPort(rate.getReference());
        Value * const baseRate = getInitialStrideLength(b, refPort);
        return b->CreateMul2(baseRate, rate.getRate());
    }
    llvm_unreachable("unexpected rate type");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getMaximumStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getMaximumStrideLength(BuilderRef b, const Binding & binding) {
    const auto & rate = binding.getRate();
    if (LLVM_LIKELY(rate.isFixed() || rate.isBounded() || rate.isPopCount() || rate.isNegatedPopCount())) {
        return b->getSize(ceiling(mKernel->getUpperBound(binding) * mKernel->getStride()));
    } else if (LLVM_LIKELY(rate.isUnknown())) {
        return b->getSize(0);
    } else if (rate.isRelative()) {
        const Binding & ref = mKernel->getStreamBinding(rate.getReference());
        Value * const baseRate = getMaximumStrideLength(b, ref);
        return b->CreateMul2(baseRate, rate.getRate());
    }
    llvm_unreachable("unexpected rate type");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateNumOfLinearItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::calculateNumOfLinearItems(BuilderRef b, const Binding & binding) {
    const ProcessingRate & rate = binding.getRate();
    if (rate.isFixed() || rate.isBounded()) {
        return b->CreateMul2(mNumOfLinearStrides, mKernel->getUpperBound(binding) * mKernel->getStride());
    } else if (rate.isPopCount() || rate.isNegatedPopCount()) {
        return getNumOfLinearPopCountItems(b, binding);
    } else if (rate.isRelative()) {
        const Binding & ref = mKernel->getStreamBinding(rate.getReference());
        Value * const baseCount = calculateNumOfLinearItems(b, ref);
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
inline Value * PipelineCompiler::addLookahead(BuilderRef b, const unsigned inputPort, Value * itemCount) const {
    Constant * const lookAhead = getLookahead(b, inputPort);
    return (lookAhead) ? b->CreateAdd(itemCount, lookAhead) : itemCount;
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
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        b->CreateAssert(funcPtr, prefix + suffix + " is null");
    }
    b->setKernel(mKernel);
    return b->CreateBitCast(funcPtr, type);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationFunction
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getInitializeFunction(BuilderRef b) const {
    Function * const init = mKernel->getInitializeFunction(b->getModule());
    if (mKernel->hasFamilyName()) {
        return getFunctionFromKernelState(b, init->getType(), INITIALIZE_FUNCTION_POINTER_SUFFIX);
    }
    return init;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationThreadLocalFunction
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getInitializeThreadLocalFunction(BuilderRef b) const {
    Function * const init = mKernel->getInitializeThreadLocalFunction(b->getModule());
    if (mKernel->hasFamilyName()) {
        return getFunctionFromKernelState(b, init->getType(), INITIALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX);
    }
    return init;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDoSegmentFunction
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getDoSegmentFunction(BuilderRef b) const {
    Function * const doSegment = mKernel->getDoSegmentFunction(b->getModule());
    if (mKernel->hasFamilyName()) {
        return getFunctionFromKernelState(b, doSegment->getType(), DO_SEGMENT_FUNCTION_POINTER_SUFFIX);
    }
    return doSegment;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationThreadLocalFunction
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getFinalizeThreadLocalFunction(BuilderRef b) const {
    Function * const init = mKernel->getFinalizeThreadLocalFunction(b->getModule());
    if (mKernel->hasFamilyName()) {
        return getFunctionFromKernelState(b, init->getType(), FINALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX);
    }
    return init;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalizeFunction
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getFinalizeFunction(BuilderRef b) const {
    Function * const term = mKernel->getFinalizeFunction(b->getModule());
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
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        const Binding & input = getInputBinding(inputPort);
        Value * const expected = b->CreateAdd(mAlreadyProcessedPhi[inputPort], mLinearInputItemsPhi[inputPort]);
        itemCountSanityCheck(b, input, "processed", processed, expected);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifyOutputItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::verifyOutputItemCount(BuilderRef b, Value * produced, const unsigned outputPort) const {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
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
