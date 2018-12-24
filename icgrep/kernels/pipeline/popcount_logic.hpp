#include "pipeline_compiler.hpp"

namespace kernel {

namespace {

enum : int {
    BASE_OFFSET_INDEX       // the block # of the zeroth
    , CAPACITY_INDEX        // capacity of the popcount array
    , PARTIAL_SUM_INDEX     // pointer to the popcount partial sum array
    , POSITIVE_ARRAY_INDEX
    , NEGATED_ARRAY_INDEX
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

        const auto bufferPort = mBufferGraph[in_edge(bufferVertex, mBufferGraph)].Port;
        const Binding & output = mKernel->getOutputStreamSetBinding(bufferPort);

        PopCountData & pc = getPopCountReference(bufferVertex);

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
        Value * const consumed = getPopCountReferenceConsumedCount(b, bufferVertex);
        Value * const startIndex = b->CreateLShr(consumed, LOG2_BLOCK_WIDTH);
        Value * const produced = mUpdatedProducedPhi[bufferPort];
        // If this is the producer's final stride, round the index position up
        // to account for a partial stride.
        Value * const rounding = b->CreateSelect(mTerminatedPhi, BLOCK_SIZE_MINUS_1, ZERO);
        Value * const endIndex = b->CreateLShr(b->CreateAdd(produced, rounding), LOG2_BLOCK_WIDTH);





        // TODO: if the source items of the consumes of this pop count ref
        // were already produced, we could limit how many items are considered
        // here. This would likely require that the lexographical ordering
        // took this into account and tried to insert an edge into the
        // graph provided it does not induce a cycle.

        std::vector<Value *> indices(3);
        indices[0] = b->getInt32(0);
        indices[1] = b->getInt32(bufferVertex);
        indices[2] = b->getInt32(PARTIAL_SUM_INDEX);
        Value * const partialSumPtr = b->CreateGEP(mPopCountState, indices);
        Value * const basePartialSumArray = b->CreateLoad(partialSumPtr);
        assert (basePartialSumArray->getType()->isPointerTy());

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
        if (pc.HasArray) {
            indices[2] = b->getInt32(POSITIVE_ARRAY_INDEX);
            positiveArrayPtr = b->CreateGEP(mPopCountState, indices);
            basePositiveArray = b->CreateLoad(positiveArrayPtr);
        }

        Value * negativeArrayPtr = nullptr;
        Value * baseNegativeArray = nullptr;
        if (pc.HasNegatedArray) {
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
        Value * const newPartialSum = b->CreateRealloc(sizeTy, basePartialSumArray, newCapacity);
        b->CreateStore(newPartialSum, partialSumPtr);
        b->CreateStore(ZERO, newPartialSum);
        Value * newPositiveArray = nullptr;
        if (basePositiveArray) {
            newPositiveArray = b->CreateRealloc(sizeTy, basePositiveArray, newCapacity);
            b->CreateStore(newPositiveArray, positiveArrayPtr);
        }
        Value * newNegativeArray = nullptr;
        if (baseNegativeArray) {
            newNegativeArray = b->CreateRealloc(sizeTy, baseNegativeArray, newCapacity);
            b->CreateStore(newNegativeArray, negativeArrayPtr);
        }
        BasicBlock * const popCountExpandExit = b->GetInsertBlock();
        b->CreateBr(popCountLoop);

        // count up the actual entries
        b->SetInsertPoint(popCountLoop);
        PHINode * const partialSumArray = b->CreatePHI(basePartialSumArray->getType(), 3);
        partialSumArray->addIncoming(basePartialSumArray, popCountCheckEnd);
        partialSumArray->addIncoming(newPartialSum, popCountExpandExit);
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
        PHINode * const totalSum = b->CreatePHI(sizeTy, 3);
        totalSum->addIncoming(ZERO, popCountCheckEnd);
        totalSum->addIncoming(ZERO, popCountExpandExit);

        const StreamSetBuffer * const buffer = getOutputBuffer(bufferPort);
        Value * const dataPtr = buffer->getStreamBlockPtr(b.get(), ZERO, sourceIndex);
        Value * markers = b->CreateBlockAlignedLoad(dataPtr);

        // If this popcount field is only used for negated rates, just invert the markers.
        if (LLVM_UNLIKELY(pc.AlwaysNegated)) {
            markers = b->CreateNot(markers);
        }

        // TODO: parallelize the partial sum when we're reasonably sure that we'll
        // have enough data to be worth it on average. Ideally, we'd also want
        // to know whether we'd ever need to finish counting sequentially.

        Value * const count = b->CreateZExtOrTrunc(b->bitblock_popcount(markers), sizeTy);

        if (positiveArray) { assert (!pc.AlwaysNegated);
            b->CreateStore(count, b->CreateGEP(positiveArray, arrayIndex));
        }

        if (negativeArray) {
            Value * negCount = count;
            if (!pc.AlwaysNegated) {
                Constant * blockWidth = b->getSize(b->getBitBlockWidth());
                negCount = b->CreateSub(blockWidth, count);
            }
            b->CreateStore(negCount, b->CreateGEP(negativeArray, arrayIndex));
        }

        Value * const partialSum = b->CreateAdd(totalSum, count);
        Value * const nextArrayIndex = b->CreateAdd(arrayIndex, ONE);
        b->CreateStore(partialSum, b->CreateGEP(partialSumArray, nextArrayIndex));

        BasicBlock * const popCountLoopExit = b->GetInsertBlock();
        partialSumArray->addIncoming(partialSumArray, popCountLoopExit);
        Value * const nextSourceIndex = b->CreateAdd(sourceIndex, ONE);
        sourceIndex->addIncoming(nextSourceIndex, popCountLoopExit);
        arrayIndex->addIncoming(nextArrayIndex, popCountLoopExit);
        totalSum->addIncoming(partialSum, popCountLoopExit);
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
    PopCountData & pc = getPopCountReference(bufferVertex);

    std::vector<Value *> indices(3);
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(bufferVertex);

    Value * const offset = getPopCountInitialOffset(b, binding, bufferVertex, pc);

    Value * minCount = nullptr;
    bool invertCount = false;

    // If we have the independent count, we can read from it to get a count
    // w/o subtracting the i-1 th entry of the partial sum.
    if (pc.HasArray || pc.HasNegatedArray) {
        const bool isNegPopCount = rate.isNegatedPopCount() && pc.HasNegatedArray;
        const bool useNegArray = (isNegPopCount || !pc.HasArray);
        indices[2] = b->getInt32(useNegArray ? NEGATED_ARRAY_INDEX : POSITIVE_ARRAY_INDEX);
        Value * const array = b->CreateLoad(b->CreateGEP(mPopCountState, indices));
        minCount = b->CreateLoad(b->CreateGEP(array, offset));
        invertCount = useNegArray ^ rate.isNegatedPopCount();
    } else { // use the partial sum to calculate the min item count
        indices[2] = b->getInt32(PARTIAL_SUM_INDEX);
        Value * const partialSum = b->CreateLoad(b->CreateGEP(mPopCountState, indices));
        Value * const prior = b->CreateLoad(b->CreateGEP(partialSum, offset));
        Constant * const ONE = b->getSize(1);
        Value * const current = b->CreateAdd(offset, ONE);
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            const auto refPortNum = getPopCountReferencePort(mKernel, rate);
            Value * const total = getTotalItemCount(b, refPortNum);
            Value * const strideLength = getInputStrideLength(b, refPortNum);
            Value * const term = producerTerminated(refPortNum);
            Value * const strideLengthMinus1 = b->CreateSub(strideLength, ONE);
            Value * const padding = b->CreateSelect(term, strideLengthMinus1, b->getSize(0));
            Value * const paddedTotal =  b->CreateAdd(total, padding);
            Value * const countLength = b->CreateUDiv(paddedTotal, strideLength);
            Value * const sanityCheck = b->CreateICmpULE(current, countLength);
            b->CreateAssert(sanityCheck,
                            mKernel->getName() + "_" + binding.getName() + ":"
                            " pop count minimum linear item count lookup"
                            " exceeds reference stream length");
        }
        Value * const itemCount = b->CreateLoad(b->CreateGEP(partialSum, current));
        minCount = b->CreateSub(itemCount, prior);
        invertCount = rate.isNegatedPopCount() ^ pc.AlwaysNegated;
    }

    if (invertCount) {
        Constant * const BlockWidth = b->getSize(b->getBitBlockWidth());
        minCount = b->CreateSub(BlockWidth, minCount);
    }
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelIndex, binding);
    b->CallPrintInt(prefix + "_minCount", minCount);
    #endif
    return minCount;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getMaximumNumOfPopCountStrides
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getMaximumNumOfPopCountStrides(BuilderRef b, const Binding & binding, not_null<Value *> sourceItemCount, Constant * const lookAhead) {

    const ProcessingRate & rate = binding.getRate();
    const auto bufferVertex = getPopCountReferenceBuffer(mKernel, rate);
    PopCountData & pc = getPopCountReference(bufferVertex);

    IntegerType * const sizeTy = b->getSizeTy();

    std::vector<Value *> indices(3);
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(bufferVertex);
    indices[2] = b->getInt32(PARTIAL_SUM_INDEX);

    Value * const array = b->CreateLoad(b->CreateGEP(mPopCountState, indices));
    const auto prefix = makeBufferName(mKernelIndex, binding) + "_readPopCount";

    BasicBlock * const popCountEntry =
        b->GetInsertBlock();
    BasicBlock * const popCountLoop =
        b->CreateBasicBlock(prefix + "Loop", mKernelLoopCall);
    BasicBlock * const popCountExit =
        b->CreateBasicBlock(prefix + "Exit", mKernelLoopCall);

    // If we have a base offset, then it's possible that our partial sum started "before"
    // the processed position of this kernel. Add the "skipped items" to the sourceItemCount.
    Value * const offset = getPopCountInitialOffset(b, binding, bufferVertex, pc); assert (offset);
    Value * const skippedItems = b->CreateLoad(b->CreateGEP(array, offset));

    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(prefix + "_sourceItemCount", sourceItemCount);
    b->CallPrintInt(prefix + "_skippedItems", skippedItems);
    #endif

    Value * available = b->CreateAdd(sourceItemCount, skippedItems);

    assert (mNumOfLinearStrides);
    Value * const strides = b->CreateAdd(mNumOfLinearStrides, offset);

    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(prefix + "_linearStrides", mNumOfLinearStrides);
    b->CallPrintInt(prefix + "_offset", offset);
    #endif

    // TODO: replace this with a parallel icmp check and bitscan?
    b->CreateBr(popCountLoop);

    b->SetInsertPoint(popCountLoop);
    PHINode * const index = b->CreatePHI(sizeTy, 2);
    index->addIncoming(strides, popCountEntry);
    Value * requiredItems = b->CreateLoad(b->CreateGEP(array, index));

    if (LLVM_UNLIKELY(pc.AlwaysNegated ^ rate.isNegatedPopCount())) {
        Constant * const Log2BlockWidth = b->getSize(std::log2(b->getBitBlockWidth()));
        Value * const total = b->CreateShl(index, Log2BlockWidth);
        requiredItems = b->CreateSub(total, requiredItems);
    }

    if (lookAhead) {
        requiredItems = b->CreateAdd(requiredItems, lookAhead);
    }

    Value * const hasEnough = b->CreateICmpULE(requiredItems, available);
    BasicBlock * const popCountLoopEnd = b->GetInsertBlock();
    Constant * const ONE = b->getSize(1);
    Value * const nextIndex = b->CreateSub(index, ONE);
    index->addIncoming(nextIndex, popCountLoopEnd);
    b->CreateCondBr(hasEnough, popCountExit, popCountLoop);

    b->SetInsertPoint(popCountExit);
    Value * const maxStrides = b->CreateSub(index, offset);
    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt(prefix + "_maxStrides", maxStrides);
    #endif
    return maxStrides;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfLinearPopCountItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getNumOfLinearPopCountItems(BuilderRef b, const Binding & binding) {
    const ProcessingRate & rate = binding.getRate();
    const auto bufferVertex = getPopCountReferenceBuffer(mKernel, rate);
    PopCountData & pc = getPopCountReference(bufferVertex);
    std::vector<Value *> indices(3);
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(bufferVertex);
    indices[2] = b->getInt32(PARTIAL_SUM_INDEX);

    Value * const partialSum = b->CreateLoad(b->CreateGEP(mPopCountState, indices));
    Value * const initialOffset = getPopCountInitialOffset(b, binding, bufferVertex, pc);
    Value * const baseItemCount = b->CreateLoad(b->CreateGEP(partialSum, initialOffset));
    Value * const strideOffset = b->CreateAdd(mNumOfLinearStrides, initialOffset);
    Value * itemCount = b->CreateLoad(b->CreateGEP(partialSum, strideOffset));

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const sanityCheck = b->CreateICmpULE(baseItemCount, itemCount);
        b->CreateAssert(sanityCheck,
                        mKernel->getName() + "_" + binding.getName() + ":"
                        " pop count base items exceeds total items");
    }

    itemCount = b->CreateSub(itemCount, baseItemCount);

    const auto invertCount = rate.isNegatedPopCount() ^ pc.AlwaysNegated;
    if (invertCount) {
        Constant * const BlockWidth = b->getSize(b->getBitBlockWidth());
        Value * const totalItems = b->CreateMul(mNumOfLinearStrides, BlockWidth);
        itemCount = b->CreateSub(totalItems, itemCount);
    }

    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelIndex, binding);
    b->CallPrintInt(prefix + "_popCount_itemCount", itemCount);
    #endif

    return itemCount;
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
 * @brief getPopCountReferenceProcessedCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getPopCountReferenceConsumedCount(BuilderRef b, const unsigned bufferVertex) {
    const auto e = in_edge(bufferVertex, mBufferGraph);
    const auto outputPort = mBufferGraph[e].Port;
    PopCountData & pc = getPopCountReference(bufferVertex);
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
 * @brief getPopCountBaseOffsetIndex
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getPopCountInitialOffset(BuilderRef b, const Binding & binding, const unsigned bufferVertex, PopCountData & pc) {
    if (pc.InitialOffset == nullptr) {
        std::vector<Value *> indices(3);
        indices[0] = b->getInt32(0);
        indices[1] = b->getInt32(bufferVertex);
        indices[2] = b->getInt32(BASE_OFFSET_INDEX);
        Value * const baseOffset = b->CreateLoad(b->CreateGEP(mPopCountState, indices));
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelIndex, binding) + "_popCount";
        b->CallPrintInt(prefix + "_baseOffset", baseOffset);
        #endif
        Value * const strideOffset = getReferenceStreamOffset(b, binding);
        #ifdef PRINT_DEBUG_MESSAGES
        b->CallPrintInt(prefix + "_strideOffset", strideOffset);
        #endif
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            Value * const sanityCheck = b->CreateICmpUGE(strideOffset, baseOffset);
            b->CreateAssert(sanityCheck,
                            makeBufferName(mKernelIndex, binding) +
                            ": pop count base offset exceeds stride offset");
        }
        pc.InitialOffset = b->CreateSub(strideOffset, baseOffset);
        #ifdef PRINT_DEBUG_MESSAGES
        b->CallPrintInt(prefix + "_initialOffset", pc.InitialOffset);
        #endif
    }
    return pc.InitialOffset;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getReferenceStreamOffset
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getReferenceStreamOffset(BuilderRef b, const Binding & binding) {
    const auto refPortNum = getPopCountReferencePort(mKernel, binding.getRate());
    Value * const itemCount = mAlreadyProcessedPhi[refPortNum];
    Value * const strideLength = getInputStrideLength(b, refPortNum);
    return b->CreateUDiv(itemCount, strideLength);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializePopCountReferenceCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializePopCountReferenceItemCount(BuilderRef /*b*/, const unsigned bufferVertex, not_null<Value *> produced) {
    if (LLVM_UNLIKELY(in_degree(bufferVertex, mPopCountGraph) == 0)) {
        return;
    }
    assert (out_degree(bufferVertex, mConsumerGraph) != 0);
    PopCountData & pc = getPopCountReference(bufferVertex);
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
        PopCountData & pc = getPopCountReference(bufferVertex);
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
        PopCountData & pc = getPopCountReference(bufferVertex);
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
        PopCountData & pc = getPopCountReference(bufferVertex);
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
        const PopCountData & pc = getPopCountReference(bufferVertex);
        if (LLVM_UNLIKELY(!pc.UsesConsumedCount)) {
            const auto e = in_edge(bufferVertex, mBufferGraph);
            const auto port = mBufferGraph[e].Port;
            const Binding & output = mPipeline[index]->getOutputStreamSetBinding(port);
            const auto bufferName = makeBufferName(index, output);
            mPipelineKernel->addInternalScalar(b->getSizeTy(), bufferName + REFERENCE_PROCESSED_COUNT);
        }
    });
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNegatedPopCountArray
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getIndividualPopCountArray(BuilderRef b, const unsigned inputPort, const unsigned popCountArrayIndex) {
    const auto bufferVertex = getInputBufferVertex(inputPort);
    std::vector<Value *> indices(3);
    indices[0] = b->getInt32(0);
    indices[1] = b->getInt32(bufferVertex);
    indices[2] = b->getInt32(popCountArrayIndex);
    Value * array = b->CreateLoad(b->CreateGEP(mPopCountState, indices));
    indices[2] = b->getInt32(BASE_OFFSET_INDEX);
    Value * const baseOffset = b->CreateLoad(b->CreateGEP(mPopCountState, indices));
    Value * const processed = mAlreadyProcessedPhi[inputPort];
    Constant * const LOG2_COUNT_WIDTH = getLog2BlockWidth(b);
    Value * const processedOffset = b->CreateLShr(processed, LOG2_COUNT_WIDTH);
    Value * const offset = b->CreateSub(processedOffset, baseOffset);
    return b->CreateGEP(array, offset);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPopCountArray
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getPopCountArray(BuilderRef b, const unsigned inputPort) {
    return getIndividualPopCountArray(b, inputPort, POSITIVE_ARRAY_INDEX);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNegatedPopCountArray
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getNegatedPopCountArray(BuilderRef b, const unsigned inputPort) {
    return getIndividualPopCountArray(b, inputPort, NEGATED_ARRAY_INDEX);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief allocatePopCountArrays
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::allocatePopCountArrays(BuilderRef b, Value * const popCountState) {
    if (num_edges(mPopCountGraph) == 0) {
        return;
    }

    const auto firstBuffer = mPipeline.size() + 1;
    const auto lastBuffer = num_vertices(mBufferGraph);
    assert (firstBuffer <= lastBuffer);

    Constant * const CAPACITY = b->getInt32(CAPACITY_INDEX);
    Constant * const PARTIAL_SUM = b->getInt32(PARTIAL_SUM_INDEX);
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
            // const BufferNode & bn = mBufferGraph[source(e, mBufferGraph)];
            const PopCountData & pc = getPopCountReference(i);
            // const auto required = (bn.upper * rd.maximum) / pc.FieldWidth;
            const auto required = rd.Maximum / pc.FieldWidth;
            Constant * const initialSize = b->getSize(ceiling(required));

            indices[1] = b->getInt32(i);
            indices[2] = CAPACITY;
            b->CreateStore(initialSize, b->CreateGEP(popCountState, indices));
            indices[2] = PARTIAL_SUM;


            Value * const partialSum = b->CreateCacheAlignedMalloc(sizeTy, initialSize);
            b->CreateStore(partialSum, b->CreateGEP(popCountState, indices));
            b->CreateStore(ZERO, partialSum);
            if (pc.HasArray) {
                indices[2] = POSITIVE_ARRAY;
                Value * const array = b->CreateCacheAlignedMalloc(sizeTy, initialSize);
                b->CreateStore(array, b->CreateGEP(popCountState, indices));
            }
            if (pc.HasNegatedArray) {
                indices[2] = NEGATIVE_ARRAY;
                Value * const array = b->CreateCacheAlignedMalloc(sizeTy, initialSize);
                b->CreateStore(array, b->CreateGEP(popCountState, indices));
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
    const auto firstBuffer = mPipeline.size() + 1;
    const auto lastBuffer = num_vertices(mBufferGraph);
    assert (firstBuffer <= lastBuffer);

    Constant * const PARTIAL_SUM = b->getInt32(PARTIAL_SUM_INDEX);
    Constant * const POSITIVE_ARRAY = b->getInt32(POSITIVE_ARRAY_INDEX);
    Constant * const NEGATIVE_ARRAY = b->getInt32(NEGATED_ARRAY_INDEX);

    std::vector<Value *> indices(3);
    indices[0] = b->getInt32(0);
    for (auto i = firstBuffer; i != lastBuffer; ++i) {
        // is this buffer a reference stream for some pop count rate?
        if (LLVM_UNLIKELY(in_degree(i, mPopCountGraph) != 0)) {
            indices[1] = b->getInt32(i);
            indices[2] = PARTIAL_SUM;
            b->CreateFree(b->CreateLoad(b->CreateGEP(popCountState, indices)));
            const PopCountData & pc = getPopCountReference(i);
            if (pc.HasArray) {
                indices[2] = POSITIVE_ARRAY;
                b->CreateFree(b->CreateLoad(b->CreateGEP(popCountState, indices)));
            }
            if (pc.HasNegatedArray) {
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
    const auto firstBuffer = mPipeline.size() + 1;
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

    const auto firstBuffer = mPipeline.size() + 1;
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

            const auto & pc = getPopCountReference(i);
            state[BASE_OFFSET_INDEX] = sizeTy;
            state[CAPACITY_INDEX] = sizeTy;
            state[PARTIAL_SUM_INDEX] = sizePtrTy;
            state[POSITIVE_ARRAY_INDEX] = pc.HasArray ? sizePtrTy : emptyTy;
            state[NEGATED_ARRAY_INDEX] = pc.HasNegatedArray ? sizePtrTy : emptyTy;
            types[i] = StructType::get(b->getContext(), state);
        }
    }
    return StructType::get(b->getContext(), types);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPopCountReference
 ** ------------------------------------------------------------------------------------------------------------- */
LLVM_READNONE PopCountData & PipelineCompiler::getPopCountReference(const unsigned bufferVertex) {
    const auto f = mPopCountData.find(bufferVertex);
    assert (f != mPopCountData.end());
    return f->second;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief analyzePopCountReference
 ** ------------------------------------------------------------------------------------------------------------- */
PopCountData PipelineCompiler::analyzePopCountReference(const unsigned bufferVertex) const {
    PopCountData pc{};
    pc.FieldWidth =
        popCountReferenceFieldWidth(bufferVertex);
    std::tie(pc.HasArray, pc.HasNegatedArray) =
        popCountReferenceRequiresPopCountArray(bufferVertex);
    pc.AlwaysNegated =
        pc.HasArray ? false : popCountReferenceIsAlwaysNegated(bufferVertex);
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
        const auto kernelIndex = parent(source(e, mPopCountGraph), mPopCountGraph);
        Kernel * const k = mPipeline[kernelIndex];
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

#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief popCountReferenceRequiresBaseOffset
 *
 * Determine that all kernels that use this buffer progress at the same rate. This requires not only that the
 * inputs are processed at the same rate but also that they are produced at it. Similarly, any data the kernel
 * produces that is produced into a non-DynamicBuffer must also guaranteed to be consumed at the same rate.
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::popCountReferenceRequiresBaseOffset(const unsigned bufferVertex) const {
// TODO: do we need to prove that the single kernel only ever requires a single iteration to fully
// consume it's data?


//    // if this reference is used by one kernel, we're trivially guaranteed this holds
//    if (popCountBufferIsUsedBySingleKernel(bufferVertex)) {
//        return false;
//    }


//    // TODO: take the transitive closure of the buffer graph, annotated with the processing rates.
//    // we can handle fixed rates by simply checking that the upper/lower bound of every reachable
//    // kernel is equivalent but the same notion doesn't work for popcounts. If a kernel is not
//    // dependent on two different popcount rates (w.r.t. the reference buffer), can we use the
//    // partial sum?
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief popCountBufferIsUsedBySingleKernel
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::popCountBufferIsUsedBySingleKernel(const unsigned bufferVertex) const {
    // if we have only one incoming port, we're trivially guaranteed this holds
    if (in_degree(bufferVertex, mPopCountGraph) == 1) {
        return true;
    }
    flat_set<unsigned> K;
    // buffer <- port <- kernel
    for (const auto e : make_iterator_range(in_edges(bufferVertex, mPopCountGraph))) {
        const auto p = source(e, mPopCountGraph);
        const auto f = first_in_edge(p, mPopCountGraph);
        const auto k = source(f, mPopCountGraph);
        K.insert(k);
    }
    return K.size() == 1;
}

#endif

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief popCountReferenceRequiresPopCountArray
 *
 * Check if the kernel itself requires a popcount array to perform iteration.
 ** ------------------------------------------------------------------------------------------------------------- */
inline std::pair<bool, bool> PipelineCompiler::popCountReferenceRequiresPopCountArray(const unsigned bufferVertex) const {
    bool hasArray = false, hasNegatedArray = false;
    for (const auto e : make_iterator_range(out_edges(bufferVertex, mBufferGraph))) {
        const auto port = mBufferGraph[e].Port;
        Kernel * const consumer = mBufferGraph[target(e, mBufferGraph)].Kernel;
        const Binding & b = consumer->getInputStreamSetBinding(port);
        if (LLVM_UNLIKELY(b.hasAttribute(AttrId::RequiresPopCountArray))) {
            hasArray = true;
        }
        if (LLVM_UNLIKELY(b.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
            hasNegatedArray = true;
        }
    }
    return std::make_pair(hasArray, hasNegatedArray);
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
        const auto port = mBufferGraph[e].Port;
        Kernel * const consumer = mBufferGraph[target(e, mBufferGraph)].Kernel;
        const Binding & input = consumer->getInputStreamSetBinding(port);
        if (input.isDeferred() || !input.getRate().isFixed()) {
            return false;
        }
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief popCountReferenceIsAlwaysNegated
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool PipelineCompiler::popCountReferenceIsAlwaysNegated(const unsigned bufferVertex) const {
    for (const auto e : make_iterator_range(in_edges(bufferVertex, mPopCountGraph))) {
        if (LLVM_LIKELY(mPopCountGraph[e].Type != CountingType::Negative)) {
            return false;
        }
    }
    return true;
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
            func(bufferVertex, mBufferGraph[e].Port);
        }
    }
}


}
