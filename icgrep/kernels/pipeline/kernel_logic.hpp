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
void PipelineCompiler::checkForSufficientInputDataAndOutputSpace(BuilderRef b) {
    assert (b->getKernel() == mKernel);
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    for (const auto i : mPortOrdering) {
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

    Value * const accessible = getAccessibleInputItems(b, inputPort);
    Value * const strideLength = getInputStrideLength(b, inputPort);
    Value * const requiredInput = addLookahead(b, inputPort, strideLength);
    const Binding & input = mKernel->getInputStreamSetBinding(inputPort);
    const auto prefix = makeBufferName(mKernelIndex, input);
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(prefix + "_requiredInput", requiredInput);
    #endif
    Value * const hasEnough = b->CreateICmpUGE(accessible, requiredInput);
    Value * const hasTerminated = hasProducerTerminated(b, inputPort);
    Value * const sufficientInput = b->CreateOr(hasEnough, hasTerminated);
    mAccessibleInputItems[inputPort] = accessible;
    BasicBlock * const target = b->CreateBasicBlock(prefix + "_hasInputData", mKernelLoopCall);

    b->CreateLikelyCondBr(sufficientInput, target, mKernelLoopExit);
    BasicBlock * const exitBlock = b->GetInsertBlock();
    mTerminatedPhi->addIncoming(b->getFalse(), exitBlock);
    if (mHasProgressedPhi) {
        mHasProgressedPhi->addIncoming(mAlreadyProgressedPhi, exitBlock);
    }
    b->SetInsertPoint(target);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getAlreadyProcessedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getAlreadyProcessedItemCount(BuilderRef b, const unsigned inputPort) {
    if (mAlreadyProcessedItemCount[inputPort]) {
        return mAlreadyProcessedItemCount[inputPort];
    }
    const Binding & input = mKernel->getInputStreamSetBinding(inputPort);
    Value * const processed = b->getNonDeferredProcessedItemCount(input);
    mAlreadyProcessedItemCount[inputPort] = processed;
    return processed;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasProducerTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::hasProducerTerminated(BuilderRef /* b */, const unsigned inputPort) const {
    const auto bufferVertex = getInputBufferVertex(inputPort);
    const auto producerVertex = parent(bufferVertex, mBufferGraph);
    return mTerminationGraph[producerVertex];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getAccessibleInputItems
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getAccessibleInputItems(BuilderRef b, const unsigned inputPort) {
    assert (inputPort < mAccessibleInputItems.size());
    const Binding & input = mKernel->getInputStreamSetBinding(inputPort);
    const StreamSetBuffer * const buffer = getInputBuffer(inputPort);
    Value * const totalItems = getTotalItemCount(b, inputPort);
    Value * const processed = getAlreadyProcessedItemCount(b, inputPort);
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelIndex, input);
    b->CallPrintInt(prefix + "_capacity", buffer->getCapacity(b.get()));
    b->CallPrintInt(prefix + "_totalItems", totalItems);
    b->CallPrintInt(prefix + "_processed", processed);
    #endif
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const sanityCheck = b->CreateICmpULE(processed, totalItems);
        b->CreateAssert(sanityCheck,
                        mKernel->getName() + "_" + input.getName() +
                        ": processed count exceeds total count");
    }
    const auto overflow = getFacsimile(getInputBufferVertex(inputPort));
    Value * const accessible = buffer->getLinearlyAccessibleItems(b, processed, totalItems, overflow);
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(prefix + "_accessible", accessible);
    #endif
    return accessible;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForSufficientOutputSpaceOrExpand
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::checkForSufficientOutputSpaceOrExpand(BuilderRef b, const unsigned outputPort) {
    // If the buffer is managed by the kernel, ignore it
    if (mLinearOutputItemsPhi[outputPort]) {
        const StreamSetBuffer * const buffer = getOutputBuffer(outputPort);
        Value * const writable = getWritableOutputItems(b, outputPort);
        Value * const strideLength = getOutputStrideLength(b, outputPort);
        const Binding & output = mKernel->getOutputStreamSetBinding(outputPort);
        const auto prefix = makeBufferName(mKernelIndex, output);
        #ifdef PRINT_DEBUG_MESSAGES
        b->CallPrintInt(prefix + "_requiredOutput", strideLength);
        #endif
        Value * const hasEnough = b->CreateICmpULE(strideLength, writable, prefix + "_hasEnough");
        Value * const check = b->CreateAnd(hasEnough, willNotOverwriteOverflow(b, outputPort));
        BasicBlock * const target = b->CreateBasicBlock(prefix + "_hasOutputSpace", mKernelLoopCall);
        mWritableOutputItems[outputPort] = writable;
        if (LLVM_UNLIKELY(isa<DynamicBuffer>(buffer))) {
            expandOutputBuffer(b, outputPort, check, target);
        } else {
            b->CreateLikelyCondBr(check, target, mKernelLoopExit);
            BasicBlock * const exitBlock = b->GetInsertBlock();
            mTerminatedPhi->addIncoming(b->getFalse(), exitBlock);
            if (mHasProgressedPhi) {
                mHasProgressedPhi->addIncoming(mAlreadyProgressedPhi, exitBlock);
            }
            b->SetInsertPoint(target);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief willNotOverwriteOverflow
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::willNotOverwriteOverflow(BuilderRef b, const unsigned outputPort) {
    if (LLVM_UNLIKELY(requiresCopyBack(getOutputBufferVertex(outputPort)))) { // check whether the potential overflow copy will overwrite the buffer
        Value * const produced = getAlreadyProducedItemCount(b, outputPort);
        Value * const consumed = getConsumedItemCount(b, outputPort);
        Value * const unconsumed = b->CreateSub(produced, consumed);
        Value * const strideLength = getOutputStrideLength(b, outputPort);
        Value * const required = b->CreateAdd(unconsumed, strideLength);
        const StreamSetBuffer * const buffer = getOutputBuffer(outputPort);
        Value * const capacity = buffer->getCapacity(b.get());
        const Binding & output = mKernel->getOutputStreamSetBinding(outputPort);
        const auto prefix = makeBufferName(mKernelIndex, output);
        return b->CreateICmpULT(required, capacity, prefix + "_noOverflowOverwrite");
    } else {
        return b->getTrue();
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
 * @brief getAlreadyProducedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getAlreadyProducedItemCount(BuilderRef b, const unsigned outputPort) {
    if (mAlreadyProducedItemCount[outputPort]) {
        return mAlreadyProducedItemCount[outputPort];
    }
    const Binding & output = mKernel->getOutputStreamSetBinding(outputPort);
    Value * const produced = b->getNonDeferredProducedItemCount(output);
    mAlreadyProducedItemCount[outputPort] = produced;
    return produced;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getWritableOutputItems
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getWritableOutputItems(BuilderRef b, const unsigned outputPort) {
    assert (outputPort < mWritableOutputItems.size());
    const Binding & output = mKernel->getOutputStreamSetBinding(outputPort);
    const StreamSetBuffer * const buffer = getOutputBuffer(outputPort);
    Value * const produced = getAlreadyProducedItemCount(b, outputPort);
    Value * const consumed = getConsumedItemCount(b, outputPort);
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelIndex, output);
    b->CallPrintInt(prefix + "_capacity", buffer->getCapacity(b.get()));
    b->CallPrintInt(prefix + "_produced", produced);
    b->CallPrintInt(prefix + "_consumed", consumed);
    #endif
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const sanityCheck = b->CreateICmpULE(consumed, produced);
        b->CreateAssert(sanityCheck,
                        mKernel->getName() + "_" + output.getName() +
                        ": consumed count exceeds produced count");
    }
    const auto overflow = getCopyBack(getOutputBufferVertex(outputPort));
    Value * const writable = buffer->getLinearlyWritableItems(b, produced, consumed, overflow);
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(prefix + "_writable", writable);
    #endif
    return writable;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determineNumOfLinearStrides
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::determineNumOfLinearStrides(BuilderRef b) {
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    mNumOfLinearStrides = nullptr;
    for (const auto i : mPortOrdering) {
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
    const Binding & input = mKernel->getInputStreamSetBinding(inputPort);
    const ProcessingRate & rate = input.getRate();
    Value * numOfStrides = nullptr;
    Value * const accessible = mAccessibleInputItems[inputPort];
    if (LLVM_UNLIKELY(rate.isPopCount() || rate.isNegatedPopCount())) {
        numOfStrides = getMaximumNumOfPopCountStrides(b, input, accessible, getLookahead(b, inputPort));
    } else {
        Value * const strideLength = getInputStrideLength(b, inputPort);
        numOfStrides = b->CreateUDiv(subtractLookahead(b, inputPort, accessible), strideLength);
    }
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelIndex, input);
    b->CallPrintInt("> " + prefix + "_numOfStrides", numOfStrides);
    #endif
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const term = hasProducerTerminated(b, inputPort);
        Value * const work = b->CreateIsNotNull(numOfStrides);
        Value * const progress = b->CreateOr(work, term);
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
    if (mLinearOutputItemsPhi[outputPort]) {
        const Binding & output = mKernel->getOutputStreamSetBinding(outputPort);
        const ProcessingRate & rate = output.getRate();
        Value * const writable = mWritableOutputItems[outputPort];
        if (LLVM_UNLIKELY(rate.isPopCount() || rate.isNegatedPopCount())) {
            numOfStrides = getMaximumNumOfPopCountStrides(b, output, writable);
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
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    Value * linearInputItems[numOfInputs];
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    Value * linearOutputItems[numOfOutputs];
    for (unsigned i = 0; i < numOfInputs; ++i) {
        linearInputItems[i] = calculateNumOfLinearItems(b, Port::Input, i);
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        linearOutputItems[i] = calculateNumOfLinearItems(b, Port::Output, i);
    }
    BasicBlock * const exitBlock = b->GetInsertBlock();
    for (unsigned i = 0; i < numOfInputs; ++i) {
        mLinearInputItemsPhi[i]->addIncoming(linearInputItems[i], exitBlock);
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (mLinearOutputItemsPhi[i]) {
            mLinearOutputItemsPhi[i]->addIncoming(linearOutputItems[i], exitBlock);
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
        accessibleItems[i] = addLookahead(b, i, mAccessibleInputItems[i]);
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
        if (mLinearOutputItemsPhi[i]) {
            const Binding & output = mKernel->getOutputStreamSetBinding(i);
            const ProcessingRate & rate = output.getRate();
            Value * writable = nullptr;
            if (rate.isFixed() && minScaledInverseOfAccessibleInput) {
                writable = b->CreateCeilUDiv2(minScaledInverseOfAccessibleInput, rateLCM / rate.getRate());
            } else if (rate.isPopCount() || rate.isNegatedPopCount()) {
                writable = getMinimumNumOfLinearPopCountItems(b, output);
            } else {
                writable = mWritableOutputItems[i];
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
        mLinearInputItemsPhi[i]->addIncoming(accessibleItems[i], exitBlock);
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (mLinearOutputItemsPhi[i]) {
            mLinearOutputItemsPhi[i]->addIncoming(pendingItems[i], exitBlock);
        }
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateBufferExpansionSize
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::calculateBufferExpansionSize(BuilderRef b, const unsigned outputPort) {
    Value * const produced = getAlreadyProducedItemCount(b, outputPort);
    Value * const consumed = getConsumedItemCount(b, outputPort);
    Value * const unconsumed = b->CreateSub(produced, consumed);
    Value * const strideLength = getOutputStrideLength(b, outputPort);
    return b->CreateAdd(unconsumed, strideLength);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief expandOutputBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::expandOutputBuffers(BuilderRef b) {

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief expandOutputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::expandOutputBuffer(BuilderRef b, const unsigned outputPort, Value * const hasEnough, BasicBlock * const target) {
    const Binding & output = mKernel->getOutputStreamSetBinding(outputPort);
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
    Value * const newWritableItems = getWritableOutputItems(b, outputPort);
    BasicBlock * const expandEnd = b->GetInsertBlock();
    b->CreateBr(target);

    b->SetInsertPoint(target);
    PHINode * const writablePhi = b->CreatePHI(b->getSizeTy(), 2);
    writablePhi->addIncoming(currentWritableItems, entryBlock);
    writablePhi->addIncoming(newWritableItems, expandEnd);
    mWritableOutputItems[outputPort] = writablePhi;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeKernelCall
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeKernelCall(BuilderRef b) {
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();

#warning TODO: add MProtect to buffers and their handles.

#warning TODO: send in the # of output items we want in the external buffers

    std::vector<Value *> arguments;
    arguments.reserve((numOfInputs + numOfOutputs + 1) * 2);
    arguments.push_back(mKernel->getHandle());
    arguments.push_back(mNumOfLinearStrides);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = mKernel->getInputStreamSetBinding(i);
        arguments.push_back(getLogicalInputBaseAddress(b, i));
        arguments.push_back(mLinearInputItemsPhi[i]);
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
            arguments.push_back(getPopCountArray(b, i));
        }
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
            arguments.push_back(getNegatedPopCountArray(b, i));
        }
    }

    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (mLinearOutputItemsPhi[i]) {
            arguments.push_back(getLogicalOutputBaseAddress(b, i));
            arguments.push_back(mLinearOutputItemsPhi[i]);
        }
    }

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mPipelineKernel->getHandle(), CBuilder::Protect::NONE);
    }

    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeKernelName(mKernelIndex);
    b->CallPrintInt("* " + prefix + "_executing", mNumOfLinearStrides);
    #endif

    b->CreateCall(getDoSegmentFunction(b), arguments);

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mPipelineKernel->getHandle(), CBuilder::Protect::WRITE);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeFullyProcessedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeFullyProcessedItemCounts(BuilderRef b) {
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    mFullyProcessedItemCount.resize(numOfInputs);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = mKernel->getInputStreamSetBinding(i);
        Value * processed = b->getProcessedItemCount(input.getName());
        processed = truncateBlockSize(b, input, processed, mTerminatedPhi);
        mFullyProcessedItemCount[i] = processed;
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelIndex, input);
        b->CallPrintInt(prefix + "_processed'", processed);
        #endif
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
//        const auto prefix = makeBufferName(mKernelIndex, output);
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
 * @brief incrementItemCountsOfCountableRateStreams
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::incrementItemCountsOfCountableRateStreams(BuilderRef b) {
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    for (unsigned i = 0; i < numOfInputs; i++) {
        const Binding & input = mKernel->getInputStreamSetBinding(i);
        const ProcessingRate & rate = input.getRate();
        if (rate.isFixed() || rate.isPopCount() || rate.isNegatedPopCount()) {
            Value * const processed = getAlreadyProcessedItemCount(b, i);
            Value * const items = b->CreateAdd(processed, mLinearInputItemsPhi[i]);
            b->setNonDeferredProcessedItemCount(input, items);
        }
    }
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; i++) {
        if (mLinearOutputItemsPhi[i]) {
            const Binding & output = mKernel->getOutputStreamSetBinding(i);
            const ProcessingRate & rate = output.getRate();
            if (rate.isFixed() || rate.isPopCount() || rate.isNegatedPopCount()) {
                Value * const produced = getAlreadyProducedItemCount(b, i);
                Value * const items = b->CreateAdd(produced, mLinearOutputItemsPhi[i]);
                b->setNonDeferredProducedItemCount(output, items);
            }
        }
    }

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const Binding & input = mKernel->getInputStreamSetBinding(i);
            const ProcessingRate & rate = input.getRate();
            if (rate.isBounded() || rate.isUnknown()) {
                Value * const processed = getAlreadyProcessedItemCount(b, i);
                Value * const expected = b->CreateAdd(processed, mLinearInputItemsPhi[i]);
                itemCountSanityCheck(b, input, "processed", processed, expected);
            }
        }
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            if (mLinearOutputItemsPhi[i]) {
                const Binding & output = mKernel->getOutputStreamSetBinding(i);
                const ProcessingRate & rate = output.getRate();
                if (rate.isBounded() || rate.isUnknown()) {
                    Value * const produced = getAlreadyProducedItemCount(b, i);
                    Value * const expected = b->CreateAdd(produced, mLinearOutputItemsPhi[i]);
                    itemCountSanityCheck(b, output, "produced", produced, expected);
                }
            }
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief itemCountSanityCheck
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::itemCountSanityCheck(BuilderRef b, const Binding & binding,
                                            const std::string & label,
                                            Value * const itemCount, Value * const expected) {

    const auto prefix = makeBufferName(mKernelIndex, binding);
    const auto lb = mKernel->getLowerBound(binding);
    if (lb > 0 && !binding.hasAttribute(AttrId::Deferred)) {
        Constant * const strideSize = b->getSize(ceiling(lb * mKernel->getStride()));
        Value * hasEnough = b->CreateICmpULE(itemCount, strideSize);
        hasEnough = b->CreateOr(hasEnough, terminatedExplicitly(b));
        b->CreateAssert(hasEnough, prefix + " " + label + " fewer items than expected");
    }
    Value * const withinBounds = b->CreateICmpULE(itemCount, expected);
    b->CreateAssert(withinBounds, prefix + " " + label + " more items than expected");

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getInputStrideLength(BuilderRef b, const unsigned inputPort) {
    assert (inputPort < mInputStrideLength.size());
    if (mInputStrideLength[inputPort]) {
        return mInputStrideLength[inputPort];
    } else {
        Value * const strideLength = getInitialStrideLength(b, Port::Input, inputPort);
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
        Value * const strideLength = getInitialStrideLength(b, Port::Output, outputPort);
        mOutputStrideLength[outputPort] = strideLength;
        return strideLength;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitialStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getInitialStrideLength(BuilderRef b, const Port port, const unsigned portNum) {
    const Binding & binding = getBinding(mKernel, port, portNum);
    const ProcessingRate & rate = binding.getRate();
    if (LLVM_LIKELY(rate.isFixed() || rate.isBounded())) {
        return b->getSize(ceiling(mKernel->getUpperBound(binding) * mKernel->getStride()));
    } else if (LLVM_UNLIKELY(rate.isPopCount() || rate.isNegatedPopCount())) {
        return getMinimumNumOfLinearPopCountItems(b, binding);
    } else if (rate.isRelative()) {
        Port refPort; unsigned refPortNum;
        std::tie(refPort, refPortNum) = mKernel->getStreamPort(rate.getReference());
        Value * const baseRate = getInitialStrideLength(b, refPort, refPortNum);
        return b->CreateMul2(baseRate, rate.getRate());
    }
    llvm_unreachable("unexpected rate type");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getMaximumStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getMaximumStrideLength(BuilderRef b, const Port port, const unsigned portNum) {
    const Binding & binding = getBinding(mKernel, port, portNum);
    const ProcessingRate & rate = binding.getRate();
    if (LLVM_LIKELY(rate.isFixed() || rate.isBounded() || rate.isPopCount() || rate.isNegatedPopCount())) {
        return b->getSize(ceiling(mKernel->getUpperBound(binding) * mKernel->getStride()));
    } else if (LLVM_LIKELY(rate.isUnknown())) {
        return b->getSize(0);
    } else if (rate.isRelative()) {
        Port refPort; unsigned refPortNum;
        std::tie(refPort, refPortNum) = mKernel->getStreamPort(rate.getReference());
        Value * const baseRate = getMaximumStrideLength(b, refPort, refPortNum);
        return b->CreateMul2(baseRate, rate.getRate());
    }
    llvm_unreachable("unexpected rate type");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateNumOfLinearItems
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::calculateNumOfLinearItems(BuilderRef b, const Port portType,  const unsigned portNum) {
    const Binding & binding = getBinding(mKernel, portType, portNum);
    const ProcessingRate & rate = binding.getRate();
    if (rate.isFixed() || rate.isBounded()) {
        return b->CreateMul2(mNumOfLinearStrides, mKernel->getUpperBound(binding) * mKernel->getStride());
    } else if (rate.isPopCount() || rate.isNegatedPopCount()) {
        return getNumOfLinearPopCountItems(b, binding);
    } else if (rate.isRelative()) {
        Port refPort; unsigned refPortNum;
        std::tie(refPort, refPortNum) = mKernel->getStreamPort(rate.getReference());
        Value * const baseCount = calculateNumOfLinearItems(b, refPort, refPortNum);
        return b->CreateMul2(baseCount, rate.getRate());
    }
    llvm_unreachable("unexpected rate type");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getTotalItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getTotalItemCount(BuilderRef /* b */, const unsigned inputPort) const {
    return mBufferGraph[getInputBufferVertex(inputPort)].TotalItems;
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
inline Value * PipelineCompiler::subtractLookahead(BuilderRef b, const unsigned inputPort, Value * itemCount) const {
    Constant * const lookAhead = getLookahead(b, inputPort);
    return (lookAhead) ? b->CreateSub(itemCount, lookAhead) : itemCount;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getLookahead
 ** ------------------------------------------------------------------------------------------------------------- */
Constant * PipelineCompiler::getLookahead(BuilderRef b, const unsigned inputPort) const {
    const Binding & input = mKernel->getInputStreamSetBinding(inputPort);
    if (LLVM_UNLIKELY(input.hasLookahead())) {
        return b->getSize(input.getLookahead());
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief maskBlockSize
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::truncateBlockSize(BuilderRef b, const Binding & binding, Value * itemCount, Value * all) const {
    // TODO: if we determine all of the inputs of a stream have a blocksize attribute, or the output has one,
    // we can skip masking it on input
    if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::BlockSize))) {
        // If the input rate has a block size attribute then --- for the purpose of determining how many
        // items have been consumed --- we consider a stream set to be fully processed when an entire
        // stride has been processed.
        Constant * const BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());
        Value * const maskedItemCount = b->CreateAnd(itemCount, ConstantExpr::getNeg(BLOCK_WIDTH));
        itemCount = b->CreateSelect(all, itemCount, maskedItemCount);
    }
    return itemCount;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getFunctionFromKernelState(BuilderRef b, Type * const type, const std::string & suffix) const {
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
inline Value * PipelineCompiler::getDoSegmentFunction(BuilderRef b) const {
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

template <typename Vec>
inline void reset(Vec & vec, const unsigned n) {
    vec.resize(n);
    std::fill_n(vec.begin(), n, nullptr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief resetMemoizedFields
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::resetMemoizedFields() {
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    reset(mAlreadyProcessedItemCount, numOfInputs);
    reset(mInputStrideLength, numOfInputs);
    reset(mAccessibleInputItems, numOfInputs);
    reset(mLinearInputItemsPhi, numOfInputs);
    reset(mFullyProcessedItemCount, numOfInputs);
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    reset(mInitiallyProducedItemCount, numOfOutputs);
    reset(mAlreadyProducedItemCount, numOfOutputs);
    reset(mOutputStrideLength, numOfOutputs);
    reset(mWritableOutputItems, numOfOutputs);
    reset(mLinearOutputItemsPhi, numOfOutputs);
}


}
