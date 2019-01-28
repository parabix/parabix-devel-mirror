#ifndef REGION_SELECTION_LOGIC_HPP
#define REGION_SELECTION_LOGIC_HPP

#include "pipeline_compiler.hpp"
#include <tuple>

namespace kernel {

#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeConditionalLogic
 *
 * With conditional kernels, we process a stride if and only if there is a marker in the appropriate
 * conditional indicator stream. The processed item of any non-indicator stream indicates what the
 * last unprocessed starting position of each stream.
 *
 * In a forward-processing kernel, we skip any strides prior to the next starting position and ---
 * assuming these are not also independent regions --- zero out any items prior to that position.
 * To avoid waiting until we see an end of a region, we maintain a state variable to indicate
 * whether we're currently within a conditional region and consume all input at the described rate.
 *
 * With reverse kernels, we must wait until we see the end of a region before consuming any input
 * prior to the last unprocessed starting position of that region. We defer consumption of any
 * non-indicator streams. This impacts the pipeline since those stream buffers must be dynamic.
 * However, because we can only process a region with a single iteration of this kernel, we only
 * need a state variable if the conditional indicator stream marks both beginning and ending
 * positions.
 *
 * NOTE: in either case, each region is processed sequentially, from the first to last one. Reverse
 * kernel regions are *not* processed in reverse order because that would require all of the input
 * data to be produced prior to processing the "first" region of the kernel.
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::beginRegionLoop(BuilderRef b) {

    if (hasRegions()) {


        const auto prefix = makeKernelName(mKernelIndex);

        BasicBlock * const entry = b->GetInsertBlock();
        mKernelRegionEntryLoop = b->CreateBasicBlock(prefix + "_regionLoop", mKernelCalculateItemCounts);

        mProcessRegionSpan = b->CreateBasicBlock(prefix + "processRegionSpan", mKernelCalculateItemCounts);
        mWithinPendingRegion = b->CreateBasicBlock(prefix + "withinPendingRegion", mKernelCalculateItemCounts);



        Value * inRegion = nullptr;
        if (hasIndependentRegions()) {
            inRegion = b->getScalarField(prefix + "inRegion");
        }

        Value * initialConditionCarry = nullptr;
        if (hasConditionalStream()) {
            Value * const inConditionalRegion = b->getScalarField(prefix + "inConditionalRegion");
            initialConditionCarry = b->CreateZExt(inConditionalRegion, b->getBitBlockType());
        }

        Value * const initialAlreadyProcessed = b->getScalarField(prefix + "regionAlreadyProcessed");
        if (hasIndependentRegions()) {
            // Value * const initialAlreadyProcessed = b->getScalarField(prefix + "regionAlreadyProcessed");
        }
        b->CreateBr(mKernelRegionEntryLoop);

        IntegerType * sizeTy = b->getSizeTy();
        // predeclare the selected region phi nodes
        b->SetInsertPoint(mProcessRegionSpan);
        mSelectedRegionStart = b->CreatePHI(sizeTy, 3);
        mSelectedRegionEnd = b->CreatePHI(sizeTy, 3);
        mSelectedRegionCurrent = b->CreatePHI(sizeTy, 3);

        //  begin constructing the loop
        b->SetInsertPoint(mKernelRegionEntryLoop);
        mInitialRegionStartIndex = b->CreatePHI(sizeTy, 2);
        mInitialRegionStartIndex->addIncoming(b->getSize(0), entry);
        if (initialConditionCarry) {
            mInitialConditionalCarryIn = b->CreatePHI(initialConditionCarry->getType(), 2);
            mInitialConditionalCarryIn->addIncoming(initialConditionCarry, entry);
        }

        mInitialAlreadyProcessed = b->CreatePHI(initialAlreadyProcessed->getType(), 2);
        mInitialAlreadyProcessed->addIncoming(initialAlreadyProcessed, entry);

        if (inRegion) {
            mInRegion = b->CreatePHI(inRegion->getType(), 2);
            mInRegion->addIncoming(inRegion, entry);
            computeRegionSpanForIndependentBeginEndStreams(b);
        } else {
            computeRegionSpanForSingleBeginEndStream(b);
        }







    //    // copy the data to temporary buffers but zero out any items prior to the start of the first region and after last (reversing if necessary)

    //    const bool hasReversedRegions = hasAttribute(AttrId::ReverseAdapter);

    //    copyToTemporaryBuffers(b, startIndex, endIndex, hasReversedRegions);

    //    Value * const numOfStrides = b->CreateAdd(b->CreateSub(endIndex, startIndex), ONE);

    //    // TODO: if we're processing the final segment but the final k > 1 strides are unselected, this doesn't mark last selected stride as final.
    //    // More importantly, if our entire last segment is unselected, we will never set isFinal. Should regioned kernels have a "final stride"?
    //    mIsFinal = b->CreateAnd(mIsFinal, b->CreateICmpEQ(endIndex, mNumOfStrides));

    //    mKernel->enteringMultiBlockRegion(b);

    //    mKernel->generateMultiBlockLogic(b, numOfStrides);

    //    copyFromTemporaryBuffers(b, startIndex, endIndex, hasReversedRegions);

    //    b->CreateBr(mFinishedProcessingRegions);

    //    b->SetInsertPoint(mFinishedProcessingRegions);


    } else {

        b->CreateBr(mKernelCalculateItemCounts);

    }

}


void PipelineCompiler::writeRegionedKernelCall(BuilderRef b) {

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeRegionSpanForIndependentBeginEndStreams
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeRegionSpanForIndependentBeginEndStreams(BuilderRef b) {

    Constant * const ONE = b->getSize(1);

    IntegerType * const sizeTy = b->getSizeTy();

    const auto prefix = makeKernelName(mKernelIndex);

    BasicBlock * const checkForStart =
        b->CreateBasicBlock(prefix + "checkForStart", mWithinPendingRegion);
    BasicBlock * const checkNextStart =
        b->CreateBasicBlock(prefix + "checkNextStart", mWithinPendingRegion);
    BasicBlock * const checkForEnd =
        b->CreateBasicBlock(prefix + "checkForEnd", mWithinPendingRegion);
    BasicBlock * const checkNextEnd =
        b->CreateBasicBlock(prefix + "checkNextEnd", mWithinPendingRegion);

    VectorType * const bbTy = b->getBitBlockType();
    Constant * const BITBLOCK_ZERO = Constant::getNullValue(bbTy);
    Constant * const BITBLOCK_ONE = ConstantExpr::getBitCast(b->getIntN(b->getBitBlockWidth(), 1), bbTy);

    b->CreateCondBr(mInRegion, checkForEnd, checkForStart);

    // We did not begin this stride in a conditional region, try to locate a conditional marker
    b->SetInsertPoint(checkForStart);
    mRegionStartIndex = b->CreatePHI(sizeTy, 3);
    mRegionStartIndex->addIncoming(mInitialRegionStartIndex, mKernelRegionEntryLoop);
    mRegionStartIndex->addIncoming(baseIndexPhi, checkForStart);
    mAlreadyProcessed = b->CreatePHI(bbTy, 3);
    mAlreadyProcessed->addIncoming(mInitialAlreadyProcessed, mKernelRegionEntryLoop);
    Value * const regionStarts = b->CreateAnd(getRegionStarts(), b->CreateNot(mAlreadyProcessed));
    b->CreateLikelyCondBr(b->CreateIsNull(regionStarts), checkNextStart, checkForEnd);

    // There was no starting marker in this stride; check the next stride
    b->SetInsertPoint(checkNextStart);
    Value * const nextStartIndex = b->CreateAdd(mRegionStartIndex, ONE);
    mRegionStartIndex->addIncoming(nextStartIndex, checkNextStart);
    mAlreadyProcessed->addIncoming(BITBLOCK_ZERO, checkNextStart);
    Value * const hasMoreStrides = b->CreateICmpNE(nextStartIndex, mNumOfLinearStrides);

    // if we begin processing regions from the "checkNextStart" block, then we have not found
    // any starting markers in the remaining portion of the linear segment.
    mSelectedRegionStart->addIncoming(mNumOfLinearStrides, checkNextStart);
    mSelectedRegionEnd->addIncoming(mNumOfLinearStrides, checkNextStart);
    mSelectedRegionCurrent->addIncoming(mNumOfLinearStrides, checkNextStart);
    b->CreateLikelyCondBr(hasMoreStrides, checkForStart, mProcessRegionSpan);

    // We found at least one start marker.
    b->SetInsertPoint(checkForEnd);
    mRegionEndIndex = b->CreatePHI(sizeTy, 3);
    mRegionEndIndex->addIncoming(mInitialRegionEndIndex, mKernelRegionEntryLoop);
    mRegionEndIndex->addIncoming(mRegionStartIndex, checkForStart);

    mRegionCurrentIndex = b->CreatePHI(sizeTy, 3);
    mRegionCurrentIndex->addIncoming(mInitialRegionCurrentIndex, mKernelRegionEntryLoop);
    mRegionCurrentIndex->addIncoming(mRegionStartIndex, checkForStart);

    mRegionCarryIn = b->CreatePHI(bbTy, 3);
    mRegionCarryIn->addIncoming(BITBLOCK_ONE, mKernelRegionEntryLoop);
    mRegionCarryIn->addIncoming(BITBLOCK_ZERO, checkForStart);

    if (mInitialConditionalCarryIn) {
        PHINode * const conditionalCarryIn = b->CreatePHI(mConditionalCarryIn->getType(), 2);
        conditionalCarryIn->addIncoming(mInitialConditionalCarryIn, mKernelRegionEntryLoop);
        conditionalCarryIn->addIncoming(BITBLOCK_ZERO, checkForStart);
        mConditionalCarryIn = conditionalCarryIn;
    }

    // If we're within a region, even if we find a region end, we may find some region begin after it.
    // Since it'd be preferable for the kernel to process the longest (reasonable) sequence of strides,
    // we use ScanTo(regionStarts, regionEnds) and test the carry out.

    Value * regionEnds = getRegionEnds();
    mRegionCarryOut, * regionsSpans;
    std::tie(mRegionCarryOut, regionsSpans) = b->bitblock_add_with_carry(regionStarts, b->CreateNot(regionEnds), mRegionCarryIn);
    mRegionCarryIn->addIncoming(mRegionCarryOut, checkNextEnd);
    regionEnds = b->CreateAnd(regionEnds, regionsSpans);

    if (mConditionalCarryIn) {

        writeRegionSelectionLogic(b, regionEnds, regionsSpans, checkNextStart, checkNextEnd);
        b->SetInsertPoint(checkNextEnd);

    } else {

        // If we do not have conditional regions then we must process all regions. However, we can defer
        // processing until we find a region end without another beginning within the same stride.

        Value * const hasRegionEnds = b->bitblock_any(regionEnds);
        Value * const nextEndIndex = b->CreateSelect(hasRegionEnds, mRegionCurrentIndex, mRegionEndIndex);
        Value * const inPartialRegion = b->bitblock_any(mRegionCarryOut);
        Value * const nextPendingEndIndex = b->CreateSelect(inPartialRegion, mRegionCurrentIndex, mRegionPendingIndex);
        b->CreateCondBr(inPartialRegion, checkNextEnd, mProcessRegionSpan);

        b->SetInsertPoint(checkNextEnd);
        mRegionStartIndex->addIncoming(mRegionStartIndex, checkNextEnd);
        mRegionPendingIndex->addIncoming(nextPendingEndIndex, checkNextEnd);
        mRegionEndIndex->addIncoming(nextEndIndex, checkNextEnd);
        mRegionCarryIn->addIncoming(mRegionCarryOut, checkNextEnd);

    }



    // Iterate to the next stride until we've processed all possible strides. Once the entire segment has been processed,
    // check the region span to see if we have any regions to process.
    Value * const nextCurrentIndex = b->CreateAdd(mRegionCurrentIndex, ONE);
    mRegionCurrentIndex->addIncoming(nextCurrentIndex, checkNextEnd);
    Value * const hasRemainingStrides = b->CreateICmpNE(nextCurrentIndex, mNumOfLinearStrides);
    b->CreateLikelyCondBr(hasRemainingStrides, checkForEnd, mProcessRegionSpan);



}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeRegionSpanForSingleBeginEndStream
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeRegionSpanForSingleBeginEndStream(BuilderRef b) {

    IntegerType * const sizeTy = b->getSizeTy();
    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);

    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const findRegionSpan = b->CreateBasicBlock("findRegionSpan", mProcessRegionSpan);

    // If we have a single stream, every marker is considered both a region start and end.

    if (mConditionalCarryIn) {

        b->CreateBr(findRegionSpan);

        b->SetInsertPoint(findRegionSpan);
        mRegionStartIndex = b->CreatePHI(sizeTy, 2);
        mRegionStartIndex->addIncoming(mInitialRegionStartIndex, entry);

        mRegionCurrentIndex = b->CreatePHI(sizeTy, 2);
        mRegionCurrentIndex->addIncoming(mInitialRegionCurrentIndex, entry);

        PHINode * const conditionalCarryIn = b->CreatePHI(mConditionalCarryIn->getType(), 2);
        conditionalCarryIn->addIncoming(mConditionalCarryIn, entry);
        mConditionalCarryIn->addIncoming(conditionalCarryIn, mConditionalPhiCatch);
        mConditionalCarryIn = conditionalCarryIn;

        Value * const regionEnds = getRegionEnds();
        Value * const regionSpan = b->CreateNot(regionEnds);

        BasicBlock * const checkNextStride = b->CreateBasicBlock("checkNextStride", mProcessRegionSpan);

        Value * nextStartIndex, * nextEndIndex;

        std::tie(nextStartIndex, nextEndIndex) = writeRegionSelectionLogic(b, regionEnds, regionSpan, findRegionSpan, checkNextStride);

        // Iterate to the next stride until we've processed all possible strides. Once the entire segment has been processed,
        // check the region span to see if we have any regions to process.
        b->SetInsertPoint(checkNextStride);
        mRegionStartIndex->addIncoming(mRegionStartIndex, checkNextStride);
        mRegionPendingIndex->addIncoming(mRegionPendingIndex, checkNextStride);
        Value * const nextIndex = b->CreateAdd(mRegionCurrentIndex, ONE);
        mRegionCurrentIndex->addIncoming(nextIndex, checkNextStride);
        Value * const hasMoreStrides = b->CreateICmpNE(nextIndex, mNumOfLinearStrides);
        b->CreateLikelyCondBr(hasMoreStrides, findRegionSpan, mNoSelectedRegions);

    } else { // no conditional selector

        b->CreateBr(findRegionSpan);

        b->SetInsertPoint(findRegionSpan);
        mRegionStartIndex = b->CreatePHI(sizeTy, 2);
        mRegionStartIndex->addIncoming(ZERO, entry);

        mRegionCurrentIndex = b->CreatePHI(sizeTy, 2);
        mRegionCurrentIndex->addIncoming(mNumOfLinearStrides, entry);

        // iterate backwards through the region stream until we find a region marker
        BasicBlock * const checkPriorStride = b->CreateBasicBlock("priorStride", mProcessRegionSpan);
        Value * const previousIndex = b->CreateSub(currentIndex, ONE);
        Value * const region = getRegionStarts();
        b->CreateCondBr(b->bitblock_any(region), mProcessRegionSpan, checkPriorStride);

        b->SetInsertPoint(checkPriorStride);
        mRegionStartIndex->addIncoming(ZERO, checkPriorStride);
        mRegionCurrentIndex->addIncoming(previousIndex, checkPriorStride);
        Value * const hasMoreStrides = b->CreateICmpNE(previousIndex, ZERO);
        b->CreateLikelyCondBr(hasMoreStrides, findRegionSpan, mNoSelectedRegions);

        mRegionPendingIndex = mRegionCurrentIndex;
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeRegionSelectionLogic
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeRegionSelectionLogic(BuilderRef b,
                                                 Value * const regionEnds,
                                                 Value * const regionSpans,
                                                 BasicBlock * const checkNextStart,
                                                 BasicBlock * const checkNextEnd) {

    VectorType * const bbTy = b->getBitBlockType();
    Constant * const BITBLOCK_ZERO = Constant::getNullValue(bbTy);
    Constant * const BITBLOCK_ONE = ConstantExpr::getBitCast(b->getIntN(b->getBitBlockWidth(), 1), bbTy);

    // Use MatchStar(conditional, regionSpan) to ensure any conditional selectors are aligned with the region ends.
    Value * const condition = getConditionalStream();
    Value * const selectors = b->simd_and(condition, regionSpans);
    Value * condCarryOut, * partialCondSpan;
    std::tie(condCarryOut, partialCondSpan) = b->bitblock_add_with_carry(selectors, regionSpans, mConditionalCarryIn);

    // If we encounter a region end, check to see if it belongs to a selected region
    BasicBlock * const extendRegionSpan = b->CreateBasicBlock("extendRegionSpan", checkNextEnd);
    BasicBlock * const hasRegionEnd = b->CreateBasicBlock("hasRegionEnds", checkNextEnd);
    Value * const testRegionEnd = b->bitblock_any(regionEnds);
    Value * const nextCurrentIndex = b->CreateAdd(mRegionCurrentIndex, b->getSize(1));
    BasicBlock * const exitCheckForEnd = b->GetInsertBlock();

    mRegionStartIndex->addIncoming(mRegionStartIndex, exitCheckForEnd);
    mRegionEndIndex->addIncoming(mRegionEndIndex, exitCheckForEnd);
    mRegionCurrentIndex->addIncoming(nextCurrentIndex, exitCheckForEnd);
    mConditionalCarryIn->addIncoming(condCarryOut, exitCheckForEnd);
    mRegionCarryIn->addIncoming(BITBLOCK_ONE, exitCheckForEnd);

    b->CreateCondBr(testRegionEnd, hasRegionEnd, checkNextEnd);

    b->SetInsertPoint(hasRegionEnd);

    //                          -1               0               1

    //           1)   |####E  S########|#####E  S#E S###|######E         |   keep start
    //                                       ^

    //           2)   |####E  S########|#####E  S#E S###|######E         |   advance start to current
    //                                            ^                             ?

    //           3)   |####E  S########|#####E  S#E S###|######E         |   advance start to current
    //                                                                          ^


    // When determining whether to keep or advance the start index to the current index, we need to determine whether
    // it is possible that we could encounter a conditional marker prior to the end of the "current" region (1). Since
    // we cannot know that until we see it, we avoiding advancing it to the current index until we get to the end
    // of the "current" region and determine whether it was selected.

    Value * const selectedRegionEnds = b->CreateOr(b->CreateXor(partialCondSpan, regionSpans), selectors);
    Value * firstRegionCarryOut, * firstRegionSpan;
    std::tie(firstRegionCarryOut, firstRegionSpan) = b->bitblock_add_with_carry(BITBLOCK_ZERO, regionSpans, BITBLOCK_ONE);
    Value * const currentRegionIsSelected = b->bitblock_any(b->CreateAnd(firstRegionSpan, selectedRegionEnds));
    Value * const nextStartIndex = b->CreateSelect(currentRegionIsSelected, mRegionStartIndex, mRegionCurrentIndex);

    //                          -1               0               1

    //           a)   |####E  S########|#####E  S#E S###|######E         |  advance end to current
    //                     ?                 ^

    //           b)   |####E  S########|#####E  S#E S###|######E         |  advance end to current
    //                     ^                      ^

    //           c)   |####E  S########|#####E  S#E S###|######E         |  keep end
    //                     ?                                   ?

    //           d)   |####E  S########|#####E  S#E S###|######E         |
    //                     ?                                   ?

    // If we have some selected regions and some unselected regions in the current stride then some region within
    // this stride demarcates a (potentially zero-length) span of selected regions from a span of unselected regions.

    // If the first region of this stride is a selected region then the selected span is between the startIndex and
    // the currentIndex (a). Otherwise the region that was under consideration is not a selected region so our
    // selected span still is between the startIndex and the endIndex (b). However, if the current index is
    // immediately after the last span end index, we extend the span to contain it.

    // In either case, we defer processing until we find a full-stride gap between two spans of regions to minimize the
    // cost of masking off any partial regions at the first or last stride (and potentially the cost of reversing the
    // span itself.)

    // If *all* of our regions are selected, we proceed like case (a). Since, trivially, having all of the regions
    // selected entails that the first region must be selected so this case is not explicitly considered.

    // Conversely, if *no* regions are selected in our current stride then our selected span remains between the
    // original startIndex and the endIndex. However, we defer processing until we determine whether there is a
    // 0-length gap between the end of the prior selected region span and start of the new one (c).

    Value * const nextEndIndex = b->CreateSelect(currentRegionIsSelected, mRegionCurrentIndex, mRegionEndIndex);
    Value * const tryToExtendSpan = b->bitblock_any(mRegionCarryOut);
    b->CreateCondBr(tryToExtendSpan, checkNextEnd, mProcessRegionSpan);

    mRegionStartIndex->addIncoming(nextStartIndex, hasRegionEnd);
    mRegionEndIndex->addIncoming(nextEndIndex, hasRegionEnd);
    mRegionCurrentIndex->addIncoming(nextCurrentIndex, hasRegionEnd);
    mConditionalCarryIn->addIncoming(condCarryOut, hasRegionEnd);
    mRegionCarryIn->addIncoming(mRegionCarryOut, hasRegionEnd);

    mSelectedRegionStart->addIncoming(mRegionStartIndex, extendRegionSpan);
    mSelectedRegionEnd->addIncoming(mRegionEndIndex, extendRegionSpan);
    mSelectedRegionCurrent->addIncoming(mRegionCurrentIndex, extendRegionSpan);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasRegions
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::hasRegions() const {
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasConditionalStream
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::hasConditionalStream() const {
    for (auto buffer : make_iterator_range(in_edges(mKernelIndex, mRegionGraph))) {
        const RegionData & rb = mRegionGraph[buffer];
        if (rb.Type == AttrId::RegionSelector) {
            return true;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getConditionalStream
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getConditionalStream() {
    for (auto buffer : make_iterator_range(in_edges(mKernelIndex, mRegionGraph))) {
        const RegionData & rb = mRegionGraph[buffer];
        if (rb.Type == AttrId::RegionSelector) {
            const StreamSetBuffer * const buffer = getInputBuffer(rb.Port);
            Constant * const LOG2_BLOCK_WIDTH = getLog2BlockWidth(b);
            Value * const processed = mAlreadyProcessedPhi[inputPort];
            Value * sourceIndex = b->CreateLShr(processed, LOG2_BLOCK_WIDTH);
            sourceIndex = b->CreateAdd(sourceIndex,  mRegionCurrentIndex);
            Value * const dataPtr = buffer->getStreamBlockPtr(b.get(), ZERO, sourceIndex);
            return b->CreateBlockAlignedLoad(dataPtr);
        }
    }
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasIndependentStreams
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::hasIndependentRegions() const {
//    for (const Binding & input : mKernel->getInputStreamSetBindings()) {
//        const auto isEnd = input.hasAttribute(AttrId::ConditionalRegionEnd);
//        if (input.hasAttribute(AttrId::ConditionalRegionBegin)) {
//            return !isEnd;
//        } else if (isEnd) {
//            return true;
//        }
//    }
//    llvm_unreachable("could not find conditional region attributes");
    return false;
}

#endif

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeRegionComputationLogic
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeRegionComputationLogic(BuilderRef b) {

    // do we have any regions associated with any stream produced by this kernel?
    if (LLVM_LIKELY(out_degree(mKernelIndex, mRegionGraph) == 0)) {
        return;
    }

    // yes; check if we've produced all of the streams necessary for any regions
    flat_set<unsigned> regions;
    for (const auto & e : make_iterator_range(out_edges(mKernelIndex, mRegionGraph))) {
        const auto buffer = target(e, mRegionGraph);
        for (const auto e : make_iterator_range(out_edges(buffer, mRegionGraph))) {
            const auto region = target(e, mRegionGraph);
            regions.insert(region);
        }
    }
    assert (!regions.empty());
    std::vector<unsigned> ready;
    ready.reserve(regions.size());
    for (const auto region : regions) {
        bool r = true;
        for (const auto & e : make_iterator_range(in_edges(region, mRegionGraph))) {
            const auto buffer = source(e, mRegionGraph);
            const auto producer = parent(buffer, mRegionGraph);
            if (LLVM_UNLIKELY(producer > mKernelIndex)) {
                r = false;
                break;
            }
        }
        if (LLVM_LIKELY(r)) {
            ready.push_back(region);
        }
    }

    for (const auto region : ready) {
        writeRegionComputationLogic(b, region);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeRegionComputationLogic
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeRegionComputationLogic(BuilderRef b, const unsigned region) {

    if (LLVM_LIKELY(hasSelectorStream(region))) {
        fineSelectedRegions(b, region);
    } else {
        findLongestSequenceOfAdjacentRegions(b, region);
    }


}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getRegionStartConsumedOffset
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getRegionStartConsumedOffset(BuilderRef /* b */, const unsigned region) const {
    for (const auto & e : make_iterator_range(in_edges(region, mRegionGraph))) {
        const RegionData & rd = mRegionGraph[e];
        if (rd.Type != AttrId::IndependentRegionBegin) {
            const auto buffer = source(e, mRegionGraph);
            return mPriorConsumedItemCount[getBufferIndex(buffer)];
        }
    }
    llvm_unreachable("region graph does not contain a start edge?");
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief fineSelectedRegions
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::fineSelectedRegions(BuilderRef b, const unsigned region) {
#if 0
    BasicBlock * const loopEntry = b->CreateBasicBlock("regionIdentificationLoop");
    BasicBlock * const checkEnds = b->CreateBasicBlock("regionCheckEnds");
    BasicBlock * const checkNext = b->CreateBasicBlock("regionCheckNext");

    IntegerType * const sizeTy = b->getSizeTy();
    const auto fieldWidth = sizeTy->getBitWidth();
    IntegerType * const fwTy = sizeTy; // b->getIntNTy(fieldWidth);
    const unsigned numOfFields = b->getBitBlockWidth() / fieldWidth;
    VectorType * const fwVecTy = b->fwVectorType(fieldWidth);


    Value * const initialRegionConsumedOffset = getRegionStartConsumedOffset(b, region);
    Value * initialRegionStartOffset = nullptr;
    Value * initialRegionStartEnd = nullptr;

    Value * regionSpanBuffer = nullptr;
    Value * regionEndBuffer = nullptr;

    b->CreateBr(loopEntry);

    b->SetInsertPoint(loopEntry);
    PHINode * regionCarryIn = nullptr;
    if (hasSeperateStartEndStreams(region)) {

    }
    PHINode * selectorCarryIn = nullptr;
    PHINode * regionEndOffsetPhi = nullptr;
    PHINode * regionStartIndexPhi = nullptr;
    PHINode * pendingRegion = nullptr;

    Value * regionStarts = nullptr;
    Value * regionEnds = getRegionEnds();
    Value * regionSpan = nullptr;

    // TODO: we may have multiple start/end positions to consider; while it's easy to select the first such position of each
    // but selecting the last position is harder (without an additional phase of deletion/loops/etc)

    if (hasSeperateStartEndStreams(region)) {
        regionStarts = getRegionStarts();
        Value * const regionMask = b->CreateOr(regionStarts, b->CreateNot(regionEnds));
        Value * regionCarryOut;
        std::tie(regionCarryOut, regionSpan) = b->bitblock_add_with_carry(regionStarts, regionMask, regionCarryIn);
        regionCarryIn->addIncoming(regionCarryOut, checkNext);
        regionEnds = b->CreateAnd(regionEnds, regionSpan);
    } else {
        regionStarts = regionEnds;
        regionSpan = b->CreateNot(regionEnds);
    }

    // Store the region end stream for later ...
    Value * const regionEndPtr = b->CreateGEP(regionEndBuffer, regionEndIndexPhi);
    b->CreateStore(regionEndPtr, regionEnds);

    Value * selectors = getSelectorStream();

    // Use MatchStar(conditional, regionSpan) to ensure any conditional selectors are aligned with the region ends.
    selectors = b->simd_and(selectors, regionSpan);
    Value * selectorCarryOut, * partialSelectorSpan;
    std::tie(selectorCarryOut, partialSelectorSpan) = b->bitblock_add_with_carry(selectors, regionSpan, selectorCarryIn);
    selectorCarryIn->addIncoming(selectorCarryOut, checkNext);
    selectors = b->CreateOr(b->CreateXor(partialSelectorSpan, regionSpans), selectors);

    Value * const anyEnds = b->bitblock_any(regionEnds);
    b->CreateCondBr(anyEnds, checkEnds, checkNext);

    b->SetInsertBlock(checkEnds);

    Value * partialSum = b->simd_popcount(fieldWidth, regionEnds);
    for (unsigned i = 1; i < numOfFields; i *= 2) {
        partialSum = b->simd_add(fieldWidth, partialSum, b->mvmd_slli(fieldWidth, partialSum, i));
    }
    Value * const finalSum = b->CreateExtractElement(regions, partialSum - 1);
    Value * const nextRegionEndOffset = b->CreateAdd(regionEndOffsetPhi, finalSum);

    Constant * const fieldWidthMask = b->getSize(fieldWidth - 1);
    Value * initialOffset = b->CreateAnd(regionEndOffsetPhi, fieldWidthMask);
    initialOffset = b->CreateZExtOrTrunc(initialOffset, fwTy);
    Value * const splatPending = b->simd_fill(fieldWidth, initialOffset);
    partialSum = b->mvmd_slli(fieldWidth, partialSum, 1);
    partialSum = b->simd_add(fieldWidth, pendingSum, splatPending);

    // extract the pending selectors into 32/64-bit lanes

    Value * pendingSelectors = b->simd_pext(fieldWidth, selectors, regionEnds);

    // pack the pending selectors into a "region selection bit-queue"

    Constant * const intFieldMask = ConstantInt::get(fwTy, fieldWidth - 1);
    Constant * splatFieldMask = ConstantVector::getSplat(numOfFields, intFieldMask);
    Value * const shifts = b->simd_and(partialSum, splatFieldMask);

    Value * selectorFields = b->simd_sllv(fieldWidth, pendingSelectors, shifts);

    Constant * const intFieldWidth = ConstantInt::get(fwTy, fieldWidth);
    Constant * const splatFieldWidth = ConstantVector::getSplat(numOfFields, intFieldWidth);
    Value * const backShifts = b->simd_sub(fieldWidth, splatFieldWidth, shifts);

    Value * nextSelectorFields = b->simd_srlv(fieldWidth, pendingSelectors, backShifts);

    Value * shiftedSelectorFields = b->mvmd_slli(fieldWidth, nextSelectorFields, 1);

    selectorFields = b->CreateOr(selectorFields, shiftedSelectorFields);

    Value * fieldNo = b->simd_srli(fieldWidth, partialSum, std::log2(fieldWidth));
    Value * newRegion = UndefValue::get(fwVecTy);
    for (unsigned i = 0; i < numOfFields; ++i) {
        Constant * const selected = ConstantVector::getSplat(numOfFields, ConstantInt::get(fwTy, i));
        Value * const selectedMask = b->simd_eq(fieldWidth, fieldNo, selected);
        Value * regions = b->CreateAnd(selectorFields, selectedMask);
        for (unsigned i = 1; i < numOfFields; i *= 2) {
            regions = b->simd_or(fieldWidth, regions, b->mvmd_slli(fieldWidth, regions, i));
        }
        Value * field = b->CreateExtractElement(regions, numOfFields - 1);
        newRegion = b->CreateInsertElement(newRegion, field, i);
    }
    newRegion = b->CreateOr(newRegion, pendingRegion);

    // then deposit them in a selected region start stream






    // then fill in the region span stream using SpanTo(selectedRegion, regionEnds)

    b->CreateBr(checkNext);

    b->SetInsertBlock(checkNext);
    PHINode * const nextRegionEndOffsetPhi = b->CreatePHI(sizeTy, 2);
    nextRegionEndOffsetPhi->addIncoming(regionEndOffsetPhi, loopEntry);
    nextRegionEndOffsetPhi->addIncoming(nextRegionEndOffset, checkEnds);
    regionEndOffsetPhi->addIncoming(nextRegionEndOffsetPhi, checkNext);

#endif
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief findLongestSequenceOfAdjacentRegions
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::findLongestSequenceOfAdjacentRegions(BuilderRef b, const unsigned region) {

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasSeperateStartEndStreams
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool PipelineCompiler::hasSeperateStartEndStreams(const unsigned region) const {
    unsigned buffer = 0;
    unsigned stream = 0;
    for (const auto & e : make_iterator_range(in_edges(region, mRegionGraph))) {
        const RegionData & rd = mRegionGraph[e];
        if (rd.Type != AttrId::RegionSelector) {
            const auto otherBuffer = source(e, mRegionGraph);
            const auto otherStream = rd.Stream;
            if (buffer == 0) {
                buffer = otherBuffer;
                stream = otherStream;
            } else {
                return (buffer != otherBuffer) || (stream != otherStream);
            }
        }
    }
    llvm_unreachable("region graph does not contain both a start and end edge?");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasSelectorStream
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool PipelineCompiler::hasSelectorStream(const unsigned region) const {
    return in_degree(region, mRegionGraph) == 3;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeRegionGraph
 *
 * producer -> buffer -> region node bundle -> regioned kernel (consumer)
 ** ------------------------------------------------------------------------------------------------------------- */
RegionGraph PipelineCompiler::makeRegionGraph() const {

    enum : unsigned {
        SELECTOR_BUFFER = 0
        , SELECTOR_STREAM = 1
        , REGION_START_BUFFER = 2
        , REGION_START_STREAM = 3
        , REGION_END_BUFFER = 4
        , REGION_END_STREAM = 5
    };

    using Condition = std::array<unsigned, 6>; // {selector, start, end} x {bufferVertex, streamIndex}

    const auto firstBuffer = mLastKernel + 1;
    const auto lastBuffer = num_vertices(mBufferGraph);

    RegionGraph G(lastBuffer);
    flat_map<Condition, unsigned> existingRegion;

    for (auto i = mFirstKernel; i < mLastKernel; ++i) {
        const Kernel * const kernel = mPipeline[i];
        Condition cond{};
        bool hasRegions = false;
        for (const auto & e : make_iterator_range(in_edges(i, mBufferGraph))) {
            const auto port = mBufferGraph[e].inputPort();
            const Binding & input = kernel->getInputStreamSetBinding(port);
            auto setIfAttributeExists = [&](const AttrId attrId, const unsigned index) {
                if (LLVM_UNLIKELY(input.hasAttribute(attrId))) {
                    const ProcessingRate & rate = input.getRate();
                    if (LLVM_UNLIKELY(!rate.isFixed() || rate.getRate() != RateValue(1))) {
                        report_fatal_error(kernel->getName() + ": region streams must be FixedRate(1).");
                    }
                    if (LLVM_UNLIKELY(cond[index] != 0)) {
                        std::string tmp;
                        raw_string_ostream msg(tmp);
                        msg << kernel->getName()
                            << " cannot have multiple region ";
                        switch (attrId) {
                            case AttrId::RegionSelector:
                                msg << "selector"; break;
                            case AttrId::IndependentRegionBegin:
                                msg << "start"; break;
                            case AttrId::IndependentRegionEnd:
                                msg << "end"; break;
                            default: llvm_unreachable("unknown region attribute type");
                        }
                        msg << " attributes";
                        report_fatal_error(msg.str());
                    }
                    const Attribute & region = input.findAttribute(attrId);
                    cond[index] = source(e, mBufferGraph);
                    assert ("buffer graph error! first buffer cannot be less than 2" && (cond[index] > 2));
                    cond[index + 1] = region.amount();
                    hasRegions = true;
                }
            };
            setIfAttributeExists(AttrId::RegionSelector, SELECTOR_BUFFER);
            setIfAttributeExists(AttrId::IndependentRegionBegin, REGION_START_BUFFER);
            setIfAttributeExists(AttrId::IndependentRegionEnd, REGION_END_BUFFER);
        }

        if (LLVM_UNLIKELY(hasRegions)) {

            // TODO: when we support sequentially dependent regions, make sure to test that the start/end are
            // of the same type.


            // If we have a region with a single start/end region stream that does not have a selector stream,
            // and the kernel itself is not a reverse adaptor, discard this entry as the regions will only
            // complicate processing.
            if (LLVM_UNLIKELY((cond[SELECTOR_BUFFER] == 0) &&
                              (cond[REGION_START_BUFFER] == cond[REGION_END_BUFFER]) &&
                              (cond[REGION_START_STREAM] == cond[REGION_END_STREAM]) &&
                              (!kernel->hasAttribute(AttrId::ReverseAdapter)))) {
                continue;
            }

            auto f = existingRegion.emplace(cond, 0);
            unsigned region = 0;
            if (LLVM_LIKELY(f.second)) {
                region = add_vertex(G);
                if (cond[SELECTOR_BUFFER]) {
                    add_edge(cond[SELECTOR_BUFFER], region, {AttrId::RegionSelector, cond[SELECTOR_STREAM]}, G);
                }
                if (cond[REGION_START_BUFFER] == 0 || cond[REGION_END_BUFFER] == 0) {
                    report_fatal_error(kernel->getName() + " must have both a region start and end");
                }
                add_edge(cond[REGION_START_BUFFER], region, {AttrId::IndependentRegionBegin, cond[REGION_START_STREAM]}, G);
                add_edge(cond[REGION_END_BUFFER], region, {AttrId::IndependentRegionEnd, cond[REGION_END_STREAM]}, G);
                f.first->second = region;
            } else {
                region = f.first->second; assert (region);
            }
            add_edge(i, region, G);
        }
    }



    if (LLVM_UNLIKELY(num_edges(G) != 0)) {
        for (auto i = firstBuffer; i < lastBuffer; ++i) {
            if (LLVM_UNLIKELY(out_degree(i, G) != 0)) {
                const auto producer = parent(i, mBufferGraph);
                add_edge(producer, i, G);
            }
        }
    }

    return G;
}



}

#endif // REGION_SELECTION_LOGIC_HPP
