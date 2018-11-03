#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializePopCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::initializePopCounts(BuilderRef b) {
    mPopCountDependencyGraph = makePopCountStreamDependencyGraph(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitialPopCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getInitialNumOfLinearPopCountItems(BuilderRef b, const ProcessingRate & rate) {
    assert (rate.isPopCount() || rate.isNegatedPopCount());
    Port refPort; unsigned refIndex;
    std::tie(refPort, refIndex) = mKernel->getStreamPort(rate.getReference());
    assert (refPort == Port::Input);
    PopCountData & popCount = findOrAddPopCountData(b, refIndex, rate.isNegatedPopCount());
    return getInitialNumOfLinearPopCountItems(b, popCount, refIndex, rate.isNegatedPopCount());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitialPopCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getInitialNumOfLinearPopCountItems(BuilderRef b, PopCountData & pc, const unsigned index, const bool negated) {
    // If we've already counted this stream, return the correct pop count
    if (pc.initial) {
        return pc.initial;
    }
    // Otherwise calculate it and store it for reuse.
    Value * markers = getSourceMarkers(b, pc, index, nullptr);
    if (negated) {
        markers = b->CreateNot(markers);
    }
    Value * const count = b->bitblock_popcount(markers);
    pc.initial = count;
    return count;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getMaximumNumOfPopCountStrides
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getMaximumNumOfPopCountStrides(BuilderRef b, const ProcessingRate & rate) {
    return makePopCountArray(b, rate).maximumNumOfStrides;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPopCountArray
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getPopCountArray(BuilderRef b, const unsigned index) {
    return b->CreateLoad(makePopCountArray(b, index, false).individualCountArray);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNegatedPopCountArray
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getNegatedPopCountArray(BuilderRef b, const unsigned index) {
    return b->CreateLoad(makePopCountArray(b, index, true).individualCountArray);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfLinearPopCountItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getNumOfLinearPopCountItems(BuilderRef b, const ProcessingRate & rate, Value * const numOfStrides) {
    PopCountData & popCount = makePopCountArray(b, rate);
    ConstantInt * const ONE = b->getSize(1);
    Value *  const strideIndex = b->CreateSub(numOfStrides, ONE);
    return b->CreateLoad(b->CreateGEP(b->CreateLoad(popCount.partialSumArray), strideIndex));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief allocateLocalPopCountArray
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::allocateLocalPopCountArray(BuilderRef b, const ProcessingRate & rate) {
    assert (rate.isPopCount() || rate.isNegatedPopCount());
    Port port; unsigned index;
    std::tie(port, index) = mKernel->getStreamPort(rate.getReference());
    assert (port == Port::Input);
    const auto negated = rate.isNegatedPopCount();
    PopCountData & pc = findOrAddPopCountData(b, index, negated);
    if (pc.partialSumArray == nullptr) {
        IntegerType * const sizeTy = b->getSizeTy();
        Constant * sizeOfSizeTy = ConstantExpr::getSizeOf(sizeTy);
        // TODO: this initial value should be more intelligently chosen
        Constant * maxNumOfStrides = ConstantInt::get(sizeOfSizeTy->getType(), codegen::SegmentSize / mKernel->getStride());
        pc.strideCapacity = b->CreateAlloca(maxNumOfStrides->getType());
        b->CreateStore(maxNumOfStrides, pc.strideCapacity);
        Constant * const arraySize = ConstantExpr::getMul(maxNumOfStrides, sizeOfSizeTy);
        PointerType * const sizePtrTy = sizeTy->getPointerTo();
        pc.partialSumArray = b->CreateAlloca(sizePtrTy);
        Value * const ptr = b->CreatePointerCast(b->CreateCacheAlignedMalloc(arraySize), sizePtrTy);
        b->CreateStore(ptr, pc.partialSumArray);
        const Binding & input = mKernel->getInputStreamSetBinding(index);
        if (input.hasAttribute(negated ? AttrId::RequiresNegatedPopCountArray : AttrId::RequiresPopCountArray)) {
            pc.individualCountArray = b->CreateAlloca(sizePtrTy);
            Value * const ptr = b->CreatePointerCast(b->CreateCacheAlignedMalloc(arraySize), sizePtrTy);
            b->CreateStore(ptr, pc.individualCountArray);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief deallocateLocalPopCountArray
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::deallocateLocalPopCountArray(BuilderRef b, const ProcessingRate & rate) {
    PopCountData & pc = findOrAddPopCountData(b, rate);
    if (pc.partialSumArray) {
        b->CreateFree(b->CreateLoad(pc.partialSumArray));
        pc.partialSumArray = nullptr;
        if (pc.individualCountArray) {
            b->CreateFree(b->CreateLoad(pc.individualCountArray));
            pc.individualCountArray = nullptr;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePopCountArray
 ** ------------------------------------------------------------------------------------------------------------- */
PopCountData & PipelineCompiler::makePopCountArray(BuilderRef b, const ProcessingRate & rate) {
    assert (rate.isPopCount() || rate.isNegatedPopCount());
    Port refPort; unsigned refIndex;
    std::tie(refPort, refIndex) = mKernel->getStreamPort(rate.getReference());
    assert (refPort == Port::Input);
    return makePopCountArray(b, refIndex, rate.isNegatedPopCount());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePopCountArray
 ** ------------------------------------------------------------------------------------------------------------- */
PopCountData & PipelineCompiler::makePopCountArray(BuilderRef b, const unsigned index, const bool negated) {

    assert (mNumOfLinearStrides);
    PopCountData & pc = findOrAddPopCountData(b, index, negated);
    assert (pc.partialSumArray);

    // TODO: if the same popcounts were shared amongst multiple kernels but the state is preallocated,
    // we could attempt to "append" to the prior array(s) and reuse the calculations.
    if (LLVM_UNLIKELY(pc.hasConstructedArray == mKernelIndex)) {
        return pc;
    }

    // TODO: if we have both a popcount and negated popcount that refer to the same reference stream,
    // we could optimize the calculation and compute both at the same time.

    // NOTE: this won't correctly handle the case where we have two input ports that are popcount
    // reference streams that accept the same stream set buffer but are processed at different rates.
    // To fix this, the mapping must take the potential item position(s) imposed by reference rate
    // into account.

    const Binding & input = mKernel->getInputStreamSetBinding(index);

    BasicBlock * const entryBlock = b->GetInsertBlock();

    const auto prefix = mKernel->getName() + "_" + input.getName() + "_popCountRate";
    BasicBlock * const popCountExpand =
        b->CreateBasicBlock(prefix + "Expand", mKernelLoopCall);
    BasicBlock * const popCountStart =
        b->CreateBasicBlock(prefix + "Entry", mKernelLoopCall);
    BasicBlock * const popCountLoop =
        b->CreateBasicBlock(prefix + "Loop", mKernelLoopCall);
    BasicBlock * const checkedNumOfItems =
        b->CreateBasicBlock(prefix + "Next", mKernelLoopCall);
    BasicBlock * const popCountExit =
        b->CreateBasicBlock(prefix + "Exit", mKernelLoopCall);

    Value * const strideCapacity = b->CreateLoad(pc.strideCapacity);

    Value * const initialPartialSumArray = b->CreateLoad(pc.partialSumArray);
    Value * initialIndividualCountArray = nullptr;
    if (pc.individualCountArray) {
        initialIndividualCountArray = b->CreateLoad(pc.individualCountArray);
    }

    Value * const arrayIsLargeEnough = b->CreateICmpULE(mNumOfLinearStrides, strideCapacity);
    b->CreateLikelyCondBr(arrayIsLargeEnough, popCountStart, popCountExpand);
    // Expand the popcount array buffer(s) if they are not large enough to hold the max num of linear strides
    // NOTE: this should almost never happen. Currently the initial default is probably not large enough to
    // hold the correct number strides as it doesn't consider how the source data is transformed/generated
    // by preceeding kernels.
    b->SetInsertPoint(popCountExpand);
    IntegerType * const sizeTy = b->getSizeTy();
    Constant * sizeOfSizeTy = ConstantExpr::getSizeOf(sizeTy);
    Value * const newStrideCapacity = b->CreateRoundUp(mNumOfLinearStrides, strideCapacity);
    b->CreateStore(newStrideCapacity, pc.strideCapacity);
    Value * const newStrideSize = b->CreateMul(newStrideCapacity, sizeOfSizeTy);
    PointerType * const sizePtrTy = sizeTy->getPointerTo();
    Value * const newPartialSumArray = b->CreatePointerCast(b->CreateRealloc(initialPartialSumArray, newStrideSize), sizePtrTy);
    b->CreateStore(newPartialSumArray, pc.partialSumArray);
    Value * newIndividualCountArray = nullptr;
    if (initialIndividualCountArray) {
        newIndividualCountArray = b->CreatePointerCast(b->CreateRealloc(initialIndividualCountArray, newStrideSize), sizePtrTy);
        b->CreateStore(newIndividualCountArray, pc.individualCountArray);
    }
    BasicBlock * const popCountCheckExit = b->GetInsertBlock();
    b->CreateBr(popCountStart);

    b->SetInsertPoint(popCountStart);
    PHINode * const partialSumArray = b->CreatePHI(sizePtrTy, 2);
    partialSumArray->addIncoming(initialPartialSumArray, entryBlock);
    partialSumArray->addIncoming(newPartialSumArray, popCountCheckExit);
    PHINode * individualCountArray = nullptr;
    if (initialIndividualCountArray) {
        individualCountArray = b->CreatePHI(sizePtrTy, 2);
        individualCountArray->addIncoming(initialIndividualCountArray, entryBlock);
        individualCountArray->addIncoming(newIndividualCountArray, popCountCheckExit);
    }

    Value * const minSourceItemCount = getMinimumNumOfSourceItems(b, pc);
    Value * const initialValue = getInitialNumOfLinearPopCountItems(b, pc, index, negated);
    ConstantInt * const ONE = b->getSize(1);
    // write the popcount of the zeroth block
    b->CreateStore(initialValue, partialSumArray);
    if (individualCountArray) {
        b->CreateStore(initialValue, individualCountArray);
    }
    BasicBlock * const popCountEntry = b->GetInsertBlock();
    Value * const hasAnyStrides = b->CreateICmpNE(mNumOfLinearStrides, ONE);
    b->CreateLikelyCondBr(hasAnyStrides, popCountLoop, popCountExit);

    b->SetInsertPoint(popCountExit);
    PHINode * const finalSum = b->CreatePHI(sizeTy, 3);
    finalSum->addIncoming(initialValue, popCountEntry);

    PHINode * const strideCount = b->CreatePHI(sizeTy, 3);
    strideCount->addIncoming(mNumOfLinearStrides, popCountEntry);

    b->SetInsertPoint(popCountLoop);
    PHINode * const strideIndex = b->CreatePHI(sizeTy, 2);
    strideIndex->addIncoming(ONE, popCountEntry);
    PHINode * const partialSum = b->CreatePHI(sizeTy, 2);
    partialSum->addIncoming(initialValue, popCountEntry);
    // TODO: what if we have a stream set of more than one stream element? or swizzled input?
    Value * markers = getSourceMarkers(b, pc, index, strideIndex);
    if (negated) {
        markers = b->CreateNot(markers);
    }
    Value * const count = b->bitblock_popcount(markers);
    if (individualCountArray) {
        b->CreateStore(count, b->CreateGEP(individualCountArray, strideIndex));
    }
    Value * const sum = b->CreateAdd(partialSum, count);    
    b->CreateStore(sum, b->CreateGEP(partialSumArray, strideIndex));
    Value * hasEnough = b->getTrue();
    if (minSourceItemCount) {
        // INVESTIGATE: If I compare using ULE here and the sum exactly hits the limit and
        // the limit is the end of the buffer, an "unsafe" kernel may try to access the block
        // after. Is it better to rely on the source having an overflow for safety or on a
        // "programming by contract" design?
        Value * const hasEnoughItems = b->CreateICmpULT(sum, minSourceItemCount);
        hasEnough = b->CreateAnd(hasEnough, hasEnoughItems);
    }
    finalSum->addIncoming(partialSum, popCountLoop);
    strideCount->addIncoming(strideIndex, popCountLoop);
    b->CreateCondBr(hasEnough, checkedNumOfItems, popCountExit);

    b->SetInsertPoint(checkedNumOfItems);
    partialSum->addIncoming(sum, checkedNumOfItems);
    Value * const nextStrideIndex = b->CreateAdd(strideIndex, ONE);
    strideIndex->addIncoming(nextStrideIndex, checkedNumOfItems);
    Value * const hasMoreStrides = b->CreateICmpNE(nextStrideIndex, mNumOfLinearStrides);
    finalSum->addIncoming(sum, checkedNumOfItems);
    strideCount->addIncoming(mNumOfLinearStrides, checkedNumOfItems);
    b->CreateCondBr(hasMoreStrides, popCountLoop, popCountExit);

    b->SetInsertPoint(popCountExit);   
    pc.hasConstructedArray = mKernelIndex;
    pc.maximumNumOfStrides = strideCount;
    pc.finalPartialSum = finalSum;
    return pc;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief storeSourceItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::storePopCountSourceItemCount(BuilderRef b, const Port port, const unsigned index, Value * /* offset */, Value * processable) {
    const auto u = (port == Port::Input) ? index : index + mKernel->getNumOfStreamInputs();
    const Binding & binding = (port == Port::Input) ? mKernel->getInputStreamSetBinding(index) : mKernel->getOutputStreamSetBinding(index);
    if (LLVM_UNLIKELY(binding.hasLookahead())) {
        Constant * const lookahead = b->getSize(binding.getLookahead());
        processable = b->CreateSub(processable, lookahead);
    }
    mPopCountDependencyGraph[u] = processable;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePopCountDependencyGraph
 ** ------------------------------------------------------------------------------------------------------------- */
inline PopCountStreamDependencyGraph PipelineCompiler::makePopCountStreamDependencyGraph(BuilderRef b) {
    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    PopCountStreamDependencyGraph G(numOfInputs + numOfOutputs);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        addPopCountStreamDependency(b, i, mKernel->getInputStreamSetBinding(i), G);
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        addPopCountStreamDependency(b, i + numOfInputs, mKernel->getOutputStreamSetBinding(i), G);
    }
    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addPopCountStreamDependency
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addPopCountStreamDependency(BuilderRef b, const unsigned index, const Binding & binding, PopCountStreamDependencyGraph & G) {
    const ProcessingRate & rate = binding.getRate();
    if (LLVM_UNLIKELY(rate.isPopCount() || rate.isNegatedPopCount())) {
        Port port; unsigned j;
        std::tie(port, j) = mKernel->getStreamPort(rate.getReference());
        assert ("pop count rate cannot refer to an output stream" && port == Port::Input);
        PopCountData & popCount = findOrAddPopCountData(b, j, rate.isNegatedPopCount());
        // Since we need at least 2 streams for one of them to be a pop count stream, we know that the vertex
        // representing this pop count'ed stream cannot be 0. Thus we're safe using this as a test to see whether
        // the stream has been added to the graph yet.
        if (LLVM_LIKELY(popCount.vertex == 0)) {
            popCount.vertex = add_vertex(nullptr, G);
            assert (popCount.vertex > 1);
        }
        add_edge(popCount.vertex, index, G);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getMinimumNumOfSourceItems
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getMinimumNumOfSourceItems(BuilderRef b, const PopCountData &pc) {
    Value * minSourceItemCount = nullptr;
    assert (out_degree(pc.vertex, mPopCountDependencyGraph) > 0);
    for (const auto e : make_iterator_range(out_edges(pc.vertex, mPopCountDependencyGraph))) {
        const auto port = target(e, mPopCountDependencyGraph);
        minSourceItemCount = b->CreateUMin(minSourceItemCount, mPopCountDependencyGraph[port]);
    }
    // NOTE: this may return null when the item streams are assured to be sufficient by the pipeline's
    // processed/produced rates. In that case, we are simply building up the partial sum array
    return minSourceItemCount;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief findOrAddPopCountData
 ** ------------------------------------------------------------------------------------------------------------- */
PopCountData & PipelineCompiler::findOrAddPopCountData(BuilderRef b, const ProcessingRate & rate) {
    assert (rate.isPopCount() || rate.isNegatedPopCount());
    Port refPort; unsigned refIndex;
    std::tie(refPort, refIndex) = mKernel->getStreamPort(rate.getReference());
    assert (refPort == Port::Input);
    return findOrAddPopCountData(b, refIndex, rate.isNegatedPopCount());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief findOrAddPopCountData
 ** ------------------------------------------------------------------------------------------------------------- */
inline PopCountData & PipelineCompiler::findOrAddPopCountData(BuilderRef b, const unsigned index, const bool negated) {
    const StreamSetBuffer * const buffer = getInputBuffer(index);
    // Although unlikely it's possible that multiple rates refer to the same buffer for their popcount
    // data, even if they do not from the kernel's perspective. This abstracts that from the system.
    return  mPopCountDataMap[std::make_pair(buffer, negated)];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getSourceMarkers
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getSourceMarkers(BuilderRef b, PopCountData & pc, const unsigned index, Value * const offset) const {
    if (pc.baseIndex == nullptr) {
        const Binding & input = mKernel->getInputStreamSetBinding(index);
        Value * const processed = b->getNonDeferredProcessedItemCount(input);
        pc.baseIndex = b->CreateLShr(processed, floor_log2(b->getBitBlockWidth()));
    }
    Value * blockIndex = pc.baseIndex;
    if (offset) {
        assert (offset->getType() == blockIndex->getType());
        blockIndex = b->CreateAdd(blockIndex, offset);
    }
    const StreamSetBuffer * const buffer = getInputBuffer(index);
    Value * const ptr = buffer->getStreamBlockPtr(b.get(), b->getSize(0), blockIndex);
    return b->CreateBlockAlignedLoad(ptr);
}

}
