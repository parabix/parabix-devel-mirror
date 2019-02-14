#include "pipeline_compiler.hpp"

namespace kernel {

namespace {

enum : int {
    BASE_OFFSET_INDEX       // the block # of the zeroth
    , CAPACITY_INDEX        // capacity of the popcount array
    , POSITIVE_ARRAY_INDEX  // pointer to the popcount partial sum array
    , NEGATED_ARRAY_INDEX   // pointer to the negated popcount partial sum array
// -------------------------
    , POP_COUNT_MAX_FIELDS
};

const auto REFERENCE_PROCESSED_COUNT = "PopRefProc";

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writePopCountComputationLogic
 *
 * Compute the partial sum of the popcount for every output stream that will potentially be used as a pop count
 * reference stream.
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writePopCountComputationLogic(BuilderRef b) {

    forEachOutputBufferThatIsAPopCountReference(mKernelIndex, [&](const unsigned bufferVertex) {

        const auto producerLink = in_edge(bufferVertex, mBufferGraph);
        const auto bufferPort = mBufferGraph[producerLink].outputPort();
        const Binding & output = mKernel->getOutputStreamSetBinding(bufferPort);

        const auto prefix = makeBufferName(mKernelIndex, output) + "_genPopCount";
        BasicBlock * const popCountBuild =
            b->CreateBasicBlock(prefix + "Build", mKernelLoopCall);
        BasicBlock * const popCountExpand =
            b->CreateBasicBlock(prefix + "Expand", mKernelLoopCall);
        BasicBlock * const popCountLoop =
            b->CreateBasicBlock(prefix + "Loop", mKernelLoopCall);
        BasicBlock * const popCountExit =
            b->CreateBasicBlock(prefix + "Exit", mKernelLoopCall);

        IntegerType * const sizeTy = b->getSizeTy();

        Constant * const ZERO = b->getSize(0);
        Constant * const ONE = b->getSize(1);
        Constant * const BLOCK_SIZE_MINUS_1 = b->getSize(b->getBitBlockWidth() - 1);

        Constant * const LOG2_BLOCK_WIDTH = getLog2BlockWidth(b);
        Value * const start = getPopCountNextBaseOffset(b, bufferVertex);
        Value * const startIndex = b->CreateLShr(start, LOG2_BLOCK_WIDTH);
        Value * const produced = mUpdatedProducedPhi[bufferPort];
        // If this is the producer's final stride, round the index position up
        // to account for a partial stride.
        Value * const terminated = hasKernelTerminated(b, mKernelIndex);
        Value * const rounding = b->CreateSelect(terminated, BLOCK_SIZE_MINUS_1, ZERO);
        Value * const endIndex = b->CreateLShr(b->CreateAdd(produced, rounding), LOG2_BLOCK_WIDTH);

        // TODO: if the source items of the consumes of this pop count ref
        // were already produced, we could limit how many items are considered
        // here. This would likely require that the lexographical ordering
        // took this into account and tried to insert an edge into the
        // graph provided it does not induce a cycle.

        std::vector<Value *> indices(3);
        indices[0] = b->getInt32(0);
        indices[1] = b->getInt32(bufferVertex);

        indices[2] = b->getInt32(BASE_OFFSET_INDEX);
        Value * const offsetPtr = b->CreateGEP(mPopCountState, indices);
        b->CreateStore(startIndex, offsetPtr);

        Value * const hasAnyStrides = b->CreateICmpNE(startIndex, endIndex);
        b->CreateLikelyCondBr(hasAnyStrides, popCountBuild, popCountExit);

        b->SetInsertPoint(popCountBuild);
        indices[2] = b->getInt32(CAPACITY_INDEX);
        Value * const capacityPtr = b->CreateGEP(mPopCountState, indices);
        Value * const capacity = b->CreateLoad(capacityPtr);

        Value * positiveArrayPtr = nullptr;
        Value * basePositiveArray = nullptr;
        if (hasPositivePopCountArray(bufferVertex)) {
            indices[2] = b->getInt32(POSITIVE_ARRAY_INDEX);
            positiveArrayPtr = b->CreateGEP(mPopCountState, indices);
            basePositiveArray = b->CreateLoad(positiveArrayPtr);
        }

        Value * negativeArrayPtr = nullptr;
        Value * baseNegativeArray = nullptr;
        if (hasNegativePopCountArray(bufferVertex)) {
            indices[2] = b->getInt32(NEGATED_ARRAY_INDEX);
            negativeArrayPtr = b->CreateGEP(mPopCountState, indices);
            baseNegativeArray = b->CreateLoad(negativeArrayPtr);
        }

        Value * const required = b->CreateAdd(b->CreateSub(endIndex, startIndex), ONE);
        Value * const arrayIsLargeEnough = b->CreateICmpULE(required, capacity);
        BasicBlock * const popCountCheckEnd = b->GetInsertBlock();
        b->CreateLikelyCondBr(arrayIsLargeEnough, popCountLoop, popCountExpand);

        // Expand the popcount array buffer if it is not large enough
        b->SetInsertPoint(popCountExpand);
        Value * const newCapacity = b->CreateRoundUp(required, capacity);
        b->CreateStore(newCapacity, capacityPtr);
        Value * newPositiveArray = nullptr;
        if (basePositiveArray) {
            newPositiveArray = b->CreateRealloc(sizeTy, basePositiveArray, newCapacity);
            b->CreateStore(newPositiveArray, positiveArrayPtr);
            b->CreateStore(ZERO, newPositiveArray);
        }
        Value * newNegativeArray = nullptr;
        if (baseNegativeArray) {
            newNegativeArray = b->CreateRealloc(sizeTy, baseNegativeArray, newCapacity);
            b->CreateStore(newNegativeArray, negativeArrayPtr);
            b->CreateStore(ZERO, newNegativeArray);
        }
        BasicBlock * const popCountExpandExit = b->GetInsertBlock();
        b->CreateBr(popCountLoop);

        // count up the actual entries
        b->SetInsertPoint(popCountLoop);
        PHINode * positiveArray = nullptr;
        if (basePositiveArray) {
            positiveArray = b->CreatePHI(basePositiveArray->getType(), 3);
            positiveArray->addIncoming(basePositiveArray, popCountCheckEnd);
            positiveArray->addIncoming(newPositiveArray, popCountExpandExit);
        }
        PHINode * negativeArray = nullptr;
        if (baseNegativeArray) {
            negativeArray = b->CreatePHI(baseNegativeArray->getType(), 3);
            negativeArray->addIncoming(baseNegativeArray, popCountCheckEnd);
            negativeArray->addIncoming(newNegativeArray, popCountExpandExit);
        }
        PHINode * const sourceIndex = b->CreatePHI(b->getSizeTy(), 3);
        sourceIndex->addIncoming(startIndex, popCountCheckEnd);
        sourceIndex->addIncoming(startIndex, popCountExpandExit);
        PHINode * const arrayIndex = b->CreatePHI(b->getSizeTy(), 3);
        arrayIndex->addIncoming(ZERO, popCountCheckEnd);
        arrayIndex->addIncoming(ZERO, popCountExpandExit);
        PHINode * const positiveSum = b->CreatePHI(sizeTy, 3);
        positiveSum->addIncoming(ZERO, popCountCheckEnd);
        positiveSum->addIncoming(ZERO, popCountExpandExit);
        PHINode * const negativeSum = b->CreatePHI(sizeTy, 3);
        negativeSum->addIncoming(ZERO, popCountCheckEnd);
        negativeSum->addIncoming(ZERO, popCountExpandExit);

        const StreamSetBuffer * const buffer = getOutputBuffer(bufferPort);
        Value * const baseAddress = buffer->getBaseAddress(b.get());
        Value * const dataPtr = buffer->getStreamBlockPtr(b.get(), baseAddress, ZERO, sourceIndex);
        Value * markers = b->CreateBlockAlignedLoad(dataPtr);

        // If this popcount field is only used for negated rates, just invert the markers.
        if (LLVM_UNLIKELY(positiveArray == nullptr)) {
            markers = b->CreateNot(markers);
        }

        // TODO: parallelize the partial sum when we're reasonably sure that we'll
        // have enough data to be worth it on average. Ideally, we'd also want
        // to know whether we'd ever need to finish counting sequentially.

        Value * const count = b->CreateZExtOrTrunc(b->bitblock_popcount(markers), sizeTy);
        Value * const nextArrayIndex = b->CreateAdd(arrayIndex, ONE);

        Value * positivePartialSum = positiveSum;
        if (positiveArray) {
            positivePartialSum = b->CreateAdd(positiveSum, count);
            b->CreateStore(positivePartialSum, b->CreateGEP(positiveArray, nextArrayIndex));
        }

        Value * negativePartialSum = negativeSum;
        if (negativeArray) {
            Value * negCount = count;
            if (positiveArray) {
                Constant * blockWidth = b->getSize(b->getBitBlockWidth());
                negCount = b->CreateSub(blockWidth, count);
            }
            negativePartialSum = b->CreateAdd(negativeSum, negCount);
            b->CreateStore(negativePartialSum, b->CreateGEP(negativeArray, nextArrayIndex));
        }

        Value * const nextSourceIndex = b->CreateAdd(sourceIndex, ONE);

        BasicBlock * const popCountLoopExit = b->GetInsertBlock();
        sourceIndex->addIncoming(nextSourceIndex, popCountLoopExit);
        arrayIndex->addIncoming(nextArrayIndex, popCountLoopExit);
        positiveSum->addIncoming(positivePartialSum, popCountLoopExit);
        negativeSum->addIncoming(negativePartialSum, popCountLoopExit);
        if (positiveArray) {
            positiveArray->addIncoming(positiveArray, popCountLoopExit);
        }
        if (negativeArray) {
            negativeArray->addIncoming(negativeArray, popCountLoopExit);
        }
        Value * const hasMoreStrides = b->CreateICmpNE(nextSourceIndex, endIndex);
        b->CreateLikelyCondBr(hasMoreStrides, popCountLoop, popCountExit);

        b->SetInsertPoint(popCountExit);
    });
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitialPopCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getMinimumNumOfLinearPopCountItems(BuilderRef b, const Binding & binding) {

    const ProcessingRate & rate = binding.getRate();
    const auto bufferVertex = getPopCountReferenceBuffer(mKernel, rate);

    std::vector<Value *> indices(3);
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(bufferVertex);
    indices[2] = b->getInt32(rate.isPopCount() ? POSITIVE_ARRAY_INDEX : NEGATED_ARRAY_INDEX);

    // Calculate the min item count by subtracting the (i-1) from the i-th entry of the partial sum.
    Value * const offset = getPopCountBaseOffset(b, binding, bufferVertex);
    Value * const sumArray = b->CreateLoad(b->CreateGEP(mPopCountState, indices));
    Value * const prior = b->CreateLoad(b->CreateGEP(sumArray, offset));
    Value * const current = b->CreateAdd(offset, b->getSize(1));
    Value * const itemCount = b->CreateLoad(b->CreateGEP(sumArray, current));
    return b->CreateSub(itemCount, prior);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getMaximumNumOfPopCountStrides
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getMaximumNumOfPopCountStrides(BuilderRef b, const Binding & binding,
                                                         not_null<Value *> sourceItemCount,
                                                         not_null<Value *> peekableItemCount,
                                                         Constant * const lookAhead) {

    const ProcessingRate & rate = binding.getRate();
    const auto bufferVertex = getPopCountReferenceBuffer(mKernel, rate);

    IntegerType * const sizeTy = b->getSizeTy();

    std::vector<Value *> indices(3);
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(bufferVertex);
    indices[2] = b->getInt32(rate.isPopCount() ? POSITIVE_ARRAY_INDEX : NEGATED_ARRAY_INDEX);

    Value * const sumArray = b->CreateLoad(b->CreateGEP(mPopCountState, indices));
    const auto prefix = makeBufferName(mKernelIndex, binding) + "_readPopCount";

    BasicBlock * const popCountEntry =
        b->GetInsertBlock();
    BasicBlock * const popCountLoop =
        b->CreateBasicBlock(prefix + "Loop", mKernelLoopCall);
    BasicBlock * const popCountExit =
        b->CreateBasicBlock(prefix + "Exit", mKernelLoopCall);

    Constant * const ONE = b->getSize(1);
    Constant * const MAX_INT = ConstantInt::getAllOnesValue(sizeTy);
    // It's possible that our partial sum started "before" the processed position of this kernel...
    Value * const offset = getPopCountBaseOffset(b, binding, bufferVertex);
    Value * const initialIndex = b->CreateAdd(mNumOfLinearStrides, offset);
    // Add the "skipped items" to the source and peekable item counts
    Value * const skippedItems = b->CreateLoad(b->CreateGEP(sumArray, offset));
    Value * const availableItems = b->CreateAdd(sourceItemCount, skippedItems);
    Value * const peekableItems = b->CreateAdd(peekableItemCount, skippedItems);
    b->CreateBr(popCountLoop);

    // TODO: replace this with a parallel icmp check and bitscan?
    b->SetInsertPoint(popCountLoop);
    PHINode * const index = b->CreatePHI(sizeTy, 2);
    index->addIncoming(initialIndex, popCountEntry);
    PHINode * const nextRequiredItems = b->CreatePHI(sizeTy, 2);
    nextRequiredItems->addIncoming(MAX_INT, popCountEntry);

    Value * requiredItems = b->CreateLoad(b->CreateGEP(sumArray, index));
    if (lookAhead) {
        requiredItems = b->CreateAdd(requiredItems, lookAhead);
    }

    Value * const hasEnough = b->CreateICmpULE(requiredItems, availableItems);
    BasicBlock * const popCountLoopEnd = b->GetInsertBlock();
    Value * const priorIndex = b->CreateSub(index, ONE);
    index->addIncoming(priorIndex, popCountLoopEnd);
    nextRequiredItems->addIncoming(requiredItems, popCountLoopEnd);
    b->CreateCondBr(hasEnough, popCountExit, popCountLoop);

    b->SetInsertPoint(popCountExit);
    // Since we want to allow the stream to peek into the overflow but not start
    // in it, check to see if we can support one more stride by using it.
    Value * const numOfStrides = b->CreateSub(index, offset);
    Value * const endedPriorToBufferEnd = b->CreateICmpNE(requiredItems, availableItems);
    Value * const canPeekIntoOverflow = b->CreateICmpULE(nextRequiredItems, peekableItems);
    Value * const useOverflow = b->CreateAnd(endedPriorToBufferEnd, canPeekIntoOverflow);
    return b->CreateSelect(useOverflow, b->CreateAdd(numOfStrides, ONE), numOfStrides);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfLinearPopCountItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getNumOfLinearPopCountItems(BuilderRef b, const Binding & binding) {
    const ProcessingRate & rate = binding.getRate();
    const auto bufferVertex = getPopCountReferenceBuffer(mKernel, rate);

    std::vector<Value *> indices(3);
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(bufferVertex);
    indices[2] = b->getInt32(rate.isPopCount() ? POSITIVE_ARRAY_INDEX : NEGATED_ARRAY_INDEX);

    Value * const partialSum = b->CreateLoad(b->CreateGEP(mPopCountState, indices));
    Value * const initialOffset = getPopCountBaseOffset(b, binding, bufferVertex);
    Value * const baseItemCount = b->CreateLoad(b->CreateGEP(partialSum, initialOffset));
    Value * const strideOffset = b->CreateAdd(mNumOfLinearStrides, initialOffset);
    Value * const itemCount = b->CreateLoad(b->CreateGEP(partialSum, strideOffset));

    return b->CreateSub(itemCount, baseItemCount);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPopCountReferenceBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getPopCountReferencePort(const Kernel * kernel, const ProcessingRate & rate) const {
    assert (rate.isPopCount() || rate.isNegatedPopCount());
    Port refPort; unsigned refPortNum;
    std::tie(refPort, refPortNum) = kernel->getStreamPort(rate.getReference());
    assert (refPort == Port::Input);
    return refPortNum;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPopCountReferenceBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCompiler::getPopCountReferenceBuffer(const Kernel * kernel, const ProcessingRate & rate) const {
    return getInputBufferVertex(getPopCountReferencePort(kernel, rate));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPopCountBaseOffset
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getPopCountBaseOffset(BuilderRef b, const Binding & binding, const unsigned bufferVertex) {
    PopCountData & pc = getPopCountData(bufferVertex);
    if (pc.InitialOffset == nullptr) {
        std::vector<Value *> indices(3);
        indices[0] = b->getInt32(0);
        indices[1] = b->getInt32(bufferVertex);
        indices[2] = b->getInt32(BASE_OFFSET_INDEX);
        Value * const baseOffset = b->CreateLoad(b->CreateGEP(mPopCountState, indices));
        const auto refPortNum = getPopCountReferencePort(mKernel, binding.getRate());
        Value * const itemCount = mAlreadyProcessedPhi[refPortNum];
        Value * const strideLength = getInputStrideLength(b, refPortNum);
        Value * const strideOffset = b->CreateUDiv(itemCount, strideLength);
        pc.InitialOffset = b->CreateSub(strideOffset, baseOffset);
    }
    return pc.InitialOffset;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPopCountNextBaseOffset
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getPopCountNextBaseOffset(BuilderRef b, const unsigned bufferVertex) const {
    const auto e = in_edge(bufferVertex, mBufferGraph);
    const auto outputPort = mBufferGraph[e].outputPort();
    const PopCountData & pc = getPopCountData(bufferVertex);
    if (pc.UsesConsumedCount) {
        return mConsumedItemCount[outputPort];
    } else {
        b->setKernel(mPipelineKernel);
        const Binding & output = mKernel->getOutputStreamSetBinding(outputPort);
        const auto bufferName = makeBufferName(mKernelIndex, output);
        const auto fieldName = bufferName + REFERENCE_PROCESSED_COUNT;
        Value * const processed = b->getScalarField(fieldName);
        b->setKernel(mKernel);
        return processed;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializePopCountReferenceCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializePopCountReferenceItemCount(BuilderRef /*b*/, const unsigned bufferVertex, not_null<Value *> produced) {
    if (LLVM_UNLIKELY(in_degree(bufferVertex, mPopCountGraph) == 0)) {
        return;
    }
    assert (out_degree(bufferVertex, mConsumerGraph) != 0);
    PopCountData & pc = getPopCountData(bufferVertex);
    pc.Processed = pc.UsesConsumedCount ? nullptr : produced.get();
    pc.PhiNode = nullptr;
    pc.Encountered = 0;
    pc.InitialOffset = nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createPopCountReferenceCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::createPopCountReferenceCounts(BuilderRef b) {
    forEachOutputBufferThatIsAPopCountReference(mKernelIndex, [&](const unsigned bufferVertex) {
        PopCountData & pc = getPopCountData(bufferVertex);
        if (pc.Processed) {
            assert (mKernelEntry);
            PHINode * const processedPhi = b->CreatePHI(b->getSizeTy(), 2);
            processedPhi->addIncoming(pc.Processed, mKernelEntry);
            assert (pc.PhiNode == nullptr && pc.Encountered == 0);
            pc.PhiNode = processedPhi;
        }
    });
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeMinimumPopCountReferenceCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::computeMinimumPopCountReferenceCounts(BuilderRef b) {
    forEachPopCountReferenceInputPort(mKernelIndex, [&](const unsigned bufferVertex, const unsigned inputPort) {
        PopCountData & pc = getPopCountData(bufferVertex);
        if (pc.Processed) { assert (pc.PhiNode);
            Value * const processed = mFullyProcessedItemCount[inputPort];
            pc.Processed = b->CreateUMin(pc.Processed, processed);
        }
    });
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createPopCountReferenceCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::updatePopCountReferenceCounts(BuilderRef b) {
    forEachPopCountReferenceInputPort(mKernelIndex, [&](const unsigned bufferVertex, const unsigned inputPort) {
        PopCountData & pc = getPopCountData(bufferVertex);
        if (pc.Processed) { assert (pc.PhiNode);
            pc.PhiNode->addIncoming(pc.Processed, mKernelLoopExitPhiCatch);
            pc.Processed = pc.PhiNode;
            // Have we encountered all of the pop count refs? If so, update the scalar field.
            // NOTE: we cannot get here if this ref uses the consumed items as its ref item count.
            if (++pc.Encountered == in_degree(bufferVertex, mPopCountGraph)) {
                const Binding & output = mKernel->getOutputStreamSetBinding(inputPort);
                const auto bufferName = makeBufferName(mKernelIndex, output);
                b->setKernel(mPipelineKernel);
                b->setScalarField(bufferName + REFERENCE_PROCESSED_COUNT, pc.Processed);
                b->setKernel(mKernel);
            }
            assert (pc.Encountered <= in_degree(bufferVertex, mPopCountGraph));
        }
    });
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addPopCountScalarsToPipelineKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addPopCountScalarsToPipelineKernel(BuilderRef b, const unsigned index) {
    forEachOutputBufferThatIsAPopCountReference(index, [&](const unsigned bufferVertex) {
        const PopCountData & pc = getPopCountData(bufferVertex);
        if (LLVM_UNLIKELY(!pc.UsesConsumedCount)) {
            const auto e = in_edge(bufferVertex, mBufferGraph);
            const auto port = mBufferGraph[e].outputPort();
            const Binding & output = mPipeline[index]->getOutputStreamSetBinding(port);
            const auto bufferName = makeBufferName(index, output);
            mPipelineKernel->addInternalScalar(b->getSizeTy(), bufferName + REFERENCE_PROCESSED_COUNT);
        }
    });
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNegatedPopCountArray
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getIndividualPopCountArray(BuilderRef b, const unsigned inputPort, const unsigned arrayTypeIndex) {
    const auto bufferVertex = getInputBufferVertex(inputPort);
    std::vector<Value *> indices(3);
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(bufferVertex);
    indices[2] = b->getInt32(arrayTypeIndex);
    Value * array = b->CreateLoad(b->CreateGEP(mPopCountState, indices));
    indices[2] = b->getInt32(BASE_OFFSET_INDEX);
    Value * const baseOffset = b->CreateLoad(b->CreateGEP(mPopCountState, indices));
    Value * const processed = mAlreadyProcessedPhi[inputPort];
    Value * const processedOffset = b->CreateLShr(processed, getLog2BlockWidth(b));
    Value * const offset = b->CreateSub(processedOffset, baseOffset);
    return b->CreateGEP(array, offset);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPopCountArray
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getPositivePopCountArray(BuilderRef b, const unsigned inputPort) {
    return getIndividualPopCountArray(b, inputPort, POSITIVE_ARRAY_INDEX);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNegatedPopCountArray
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getNegativePopCountArray(BuilderRef b, const unsigned inputPort) {
    return getIndividualPopCountArray(b, inputPort, NEGATED_ARRAY_INDEX);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief allocatePopCountArrays
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::allocatePopCountArrays(BuilderRef b, Value * const popCountState) {
    if (num_edges(mPopCountGraph) == 0) {
        return;
    }

    const auto firstBuffer = mLastKernel + 1;
    const auto lastBuffer = num_vertices(mBufferGraph);
    assert (firstBuffer <= lastBuffer);

    Constant * const CAPACITY = b->getInt32(CAPACITY_INDEX);
    Constant * const POSITIVE_ARRAY = b->getInt32(POSITIVE_ARRAY_INDEX);
    Constant * const NEGATIVE_ARRAY = b->getInt32(NEGATED_ARRAY_INDEX);
    Constant * const ZERO = b->getSize(0);
    IntegerType * const sizeTy = b->getSizeTy();

    std::vector<Value *> indices(3);
    indices[0] = b->getInt32(0);

    for (auto i = firstBuffer; i != lastBuffer; ++i) {
        // is this buffer a reference stream for some pop count rate?
        if (LLVM_UNLIKELY(in_degree(i, mPopCountGraph) != 0)) {
            const auto e = in_edge(i, mBufferGraph);
            const BufferRateData & rd = mBufferGraph[e];
            const PopCountData & pc = getPopCountData(i);
            const auto required = rd.Maximum / pc.FieldWidth;
            Constant * const initialSize = b->getSize(ceiling(required));

            indices[1] = b->getInt32(i);
            indices[2] = CAPACITY;
            b->CreateStore(initialSize, b->CreateGEP(popCountState, indices));

            if (hasPositivePopCountArray(i)) {
                indices[2] = POSITIVE_ARRAY;
                Value * const array = b->CreateCacheAlignedMalloc(sizeTy, initialSize);
                b->CreateStore(array, b->CreateGEP(popCountState, indices));
                b->CreateStore(ZERO, array);
            }
            if (hasNegativePopCountArray(i)) {
                indices[2] = NEGATIVE_ARRAY;
                Value * const array = b->CreateCacheAlignedMalloc(sizeTy, initialSize);
                b->CreateStore(array, b->CreateGEP(popCountState, indices));
                b->CreateStore(ZERO, array);
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deallocatePopCountArrays
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::deallocatePopCountArrays(BuilderRef b, Value * const popCountState) {
    if (num_edges(mPopCountGraph) == 0) {
        return;
    }
    const auto firstBuffer = mLastKernel + 1;
    const auto lastBuffer = num_vertices(mBufferGraph);
    assert (firstBuffer <= lastBuffer);
    Constant * const POSITIVE_ARRAY = b->getInt32(POSITIVE_ARRAY_INDEX);
    Constant * const NEGATIVE_ARRAY = b->getInt32(NEGATED_ARRAY_INDEX);

    std::vector<Value *> indices(3);
    indices[0] = b->getInt32(0);
    for (auto i = firstBuffer; i != lastBuffer; ++i) {
        // is this buffer a reference stream for some pop count rate?
        if (LLVM_UNLIKELY(in_degree(i, mPopCountGraph) != 0)) {
            indices[1] = b->getInt32(i);
            if (hasPositivePopCountArray(i)) {
                indices[2] = POSITIVE_ARRAY;
                b->CreateFree(b->CreateLoad(b->CreateGEP(popCountState, indices)));
            }
            if (hasNegativePopCountArray(i)) {
                indices[2] = NEGATIVE_ARRAY;
                b->CreateFree(b->CreateLoad(b->CreateGEP(popCountState, indices)));
            }
        }
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializePopCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializePopCounts() {
    const auto firstBuffer = mLastKernel + 1;
    const auto lastBuffer = num_vertices(mBufferGraph);
    assert (firstBuffer < lastBuffer);
    for (auto i = firstBuffer; i != lastBuffer; ++i) {
        if (LLVM_UNLIKELY(in_degree(i, mPopCountGraph) != 0)) {
            mPopCountData.emplace(i, analyzePopCountReference(i));
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializePopCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline StructType * PipelineCompiler::getPopCountThreadLocalStateType(BuilderRef b) {

    const auto firstBuffer = mLastKernel + 1;
    const auto lastBuffer = num_vertices(mBufferGraph);
    assert (firstBuffer < lastBuffer);

    Type * const emptyTy = StructType::get(b->getContext());

    std::vector<Type *> types(lastBuffer, emptyTy);
    for (auto i = firstBuffer; i != lastBuffer; ++i) {
        // is this buffer a reference stream for some pop count rate?
        if (LLVM_UNLIKELY(in_degree(i, mPopCountGraph) != 0)) {

            Type * const sizeTy = b->getSizeTy();
            Type * const sizePtrTy = sizeTy->getPointerTo();
            std::vector<Type *> state(POP_COUNT_MAX_FIELDS);

            state[BASE_OFFSET_INDEX] = sizeTy;
            state[CAPACITY_INDEX] = sizeTy;
            state[POSITIVE_ARRAY_INDEX] = hasPositivePopCountArray(i) ? sizePtrTy : emptyTy;
            state[NEGATED_ARRAY_INDEX] = hasNegativePopCountArray(i) ? sizePtrTy : emptyTy;
            types[i] = StructType::get(b->getContext(), state);
        }
    }
    return StructType::get(b->getContext(), types);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPopCountReference
 ** ------------------------------------------------------------------------------------------------------------- */
LLVM_READNONE PopCountData & PipelineCompiler::getPopCountData(const unsigned bufferVertex) const {
    const auto f = mPopCountData.find(bufferVertex);
    assert (f != mPopCountData.end());
    return const_cast<PopCountData &>(f->second);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief analyzePopCountReference
 ** ------------------------------------------------------------------------------------------------------------- */
PopCountData PipelineCompiler::analyzePopCountReference(const unsigned bufferVertex) const {
    PopCountData pc{};
    pc.FieldWidth =
        popCountReferenceFieldWidth(bufferVertex);
    pc.UsesConsumedCount =
        popCountReferenceCanUseConsumedItemCount(bufferVertex);
    return pc;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPopCountFieldWidth
 *
 * Return the GCD of the stride length of all reference streams associated with this buffer
 ** ------------------------------------------------------------------------------------------------------------- */
RateValue PipelineCompiler::popCountReferenceFieldWidth(const unsigned bufferVertex) const {
    RateValue fw;
    bool first = true;
    for (const auto e : make_iterator_range(in_edges(bufferVertex, mPopCountGraph))) {
        const auto refPort = mPopCountGraph[e].Port;
        const auto kernelVertex = parent(source(e, mPopCountGraph), mPopCountGraph);
        const Kernel * const k = mPipeline[kernelVertex];
        const Binding & b = k->getInputStreamSetBinding(refPort);
        assert (b.getRate().isFixed());
        RateValue sw = lowerBound(k, b);
        if (first) {
            fw = sw;
            first = false;
        } else {
            fw = gcd(fw, sw);
        }
    }
    return fw;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief popCountReferenceCanUseConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool PipelineCompiler::popCountReferenceCanUseConsumedItemCount(const unsigned bufferVertex) const {

    const auto numOfConsumers = in_degree(bufferVertex, mBufferGraph);
    const auto numOfPopRefs = in_degree(bufferVertex, mPopCountGraph);
    assert (numOfPopRefs <= numOfConsumers);
    // Are the consumers of this stream all pop count references? if so, we can
    // reuse the consumed item count for it.
    if (LLVM_LIKELY(numOfPopRefs == numOfConsumers)) {
        return true;
    }
    // If the reference stream is always consumed at a non-deferred fixed rate,
    // we can still use it.
    for (const auto e : make_iterator_range(out_edges(bufferVertex, mBufferGraph))) {
        const auto port = mBufferGraph[e].inputPort();
        const auto kernelVertex = target(e, mBufferGraph);
        const Kernel * const consumer = mPipeline[kernelVertex];
        const Binding & input = consumer->getInputStreamSetBinding(port);
        if (input.isDeferred() || !input.getRate().isFixed()) {
            return false;
        }
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasPositivePopCountArray
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::hasPositivePopCountArray(const unsigned bufferVertex) const {
    for (const auto & e : make_iterator_range(in_edges(bufferVertex, mPopCountGraph))) {
        if (mPopCountGraph[e].Type & CountingType::Positive) {
            return true;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasNegativePopCountArray
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::hasNegativePopCountArray(const unsigned bufferVertex) const {
    for (const auto & e : make_iterator_range(in_edges(bufferVertex, mPopCountGraph))) {
        if (mPopCountGraph[e].Type & CountingType::Negative) {
            return true;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief forEachOutputBufferThatIsAPopCountReference
 *
 * Helper function to more clearly express what the other functions are doing.
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename LambdaFunction>
inline void PipelineCompiler::forEachOutputBufferThatIsAPopCountReference(const unsigned kernelIndex, LambdaFunction func) {
    if (LLVM_LIKELY(num_edges(mPopCountGraph) == 0)) {
        return;
    }
    for (const auto e : make_iterator_range(out_edges(kernelIndex, mBufferGraph))) {
        const auto bufferVertex = target(e, mBufferGraph);
        if (LLVM_UNLIKELY(in_degree(bufferVertex, mPopCountGraph) != 0)) {
            func(bufferVertex);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief forEachOutputBufferThatIsAPopCountReference
 *
 * Helper function to more clearly express what the other functions are doing.
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename LambdaFunction>
inline void PipelineCompiler::forEachPopCountReferenceInputPort(const unsigned kernelIndex, LambdaFunction func) {
    if (LLVM_LIKELY(num_edges(mPopCountGraph) == 0)) {
        return;
    }
    for (const auto e : make_iterator_range(in_edges(kernelIndex, mBufferGraph))) {
        const auto bufferVertex = source(e, mBufferGraph);
        if (LLVM_UNLIKELY(in_degree(bufferVertex, mPopCountGraph) != 0)) {
            func(bufferVertex, mBufferGraph[e].inputPort());
        }
    }
}

}
