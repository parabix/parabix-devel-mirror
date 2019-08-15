#include "regionselectionkernel.h"

#include <kernel/core/kernel_builder.h>

using namespace llvm;

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPopCountAsInt
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * popCountOf(const std::unique_ptr<KernelBuilder> & b, const unsigned fieldWidth, const unsigned numOfFields, Value * const toCount) {
    Value * partialSum = b->hsimd_partial_sum(fieldWidth, b->simd_popcount(fieldWidth, toCount));
    return b->CreateExtractElement(partialSum, numOfFields - 1);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief anyNonZero
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * splatCarryBit(const std::unique_ptr<KernelBuilder> & b, const unsigned fieldWidth, const unsigned numOfFields, Value * const value) {
    Value * partialMask = b->simd_eq(fieldWidth, value, b->allOnes());
    for (unsigned i = 1; i < numOfFields; i *= 2) {
        partialMask = b->simd_or(partialMask, b->mvmd_slli(fieldWidth, partialMask, i));
    }
    return b->CreateBitCast(partialMask, value->getType());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief fineSelectedRegions
 ** ------------------------------------------------------------------------------------------------------------- */
void RegionSelectionKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfStrides) {

    BasicBlock * const entry = b->GetInsertBlock();

    BasicBlock * const loopEntry = b->CreateBasicBlock("loop");
    BasicBlock * const hasRegionEnd = b->CreateBasicBlock("hasRegionEnd");
    BasicBlock * const checkNextEnd = b->CreateBasicBlock("checkNextEnd");

    BasicBlock * const writeRegionSpan = b->CreateBasicBlock("writeSpan");
    BasicBlock * const extendRegion = b->CreateBasicBlock("extendRegionLoop");
    BasicBlock * const extendRegionLoop = b->CreateBasicBlock("extendRegion");
    BasicBlock * const checkRemainingStarts = b->CreateBasicBlock("checkRemaining");
    BasicBlock * const checkNextStart = b->CreateBasicBlock("checkNext");
    BasicBlock * const readNextStart = b->CreateBasicBlock("prepareNext");

    BasicBlock * const checkFinal = b->CreateBasicBlock("checkFinal");
    BasicBlock * const writeFinal = b->CreateBasicBlock("writeFinal");
    BasicBlock * const extendFinal = b->CreateBasicBlock("extendFinal");

    BasicBlock * const exit = b->CreateBasicBlock("exit");

    IntegerType * const sizeTy = b->getSizeTy();
    const auto fieldWidth = sizeTy->getBitWidth();
    const auto blockWidth = b->getBitBlockWidth();
    const unsigned numOfFields = blockWidth / fieldWidth;
    IntegerType * const bbIntTy = b->getIntNTy(blockWidth);
    VectorType * const bbTy = b->getBitBlockType();

    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);
    Constant * const LOG_2_BLOCK_WIDTH = b->getSize(std::log2(blockWidth));
    Constant * const BB_ZERO = Constant::getNullValue(bbTy);
    Constant * const INT_ZEROS = Constant::getNullValue(bbIntTy);

    #define LOAD_CARRY_BIT(NAME) \
        b->CreateBitCast(b->CreateZExt(b->getScalarField(NAME), bbIntTy), bbTy)

    #define STORE_CARRY_BIT(NAME, VALUE) \
        b->setScalarField(NAME, b->CreateTrunc(b->CreateExtractElement(VALUE, ZERO), b->getInt1Ty()))

    Value * const initiallyProduced = b->getProducedItemCount("regionSpan");
    Value * const initialRegionStartIndex = b->CreateLShr(initiallyProduced, LOG_2_BLOCK_WIDTH);
    Value * const initialRegionEndIndex = b->CreateLShr(getNumOfRegionEnds(b), LOG_2_BLOCK_WIDTH);

    // If we processed all of the data prior to entering the kernel, the cached regionStarts value
    // will be incorrect. To limit branching, load the current start stream value along with it and
    // select the appropriate value.

    Value * const regionStarts = b->getScalarField("regionStarts");
    Value * loadedRegionStarts = getRegionStarts(b, initialRegionEndIndex);

    Value * initialSelectedReadCarryIn = nullptr;
    if (mAlwaysExtendSelectedRegionsToRegionEnds) {

        // If we exit the kernel within an incomplete but known to be selected region, we "prematurely"
        // fill in the region up to the segment end. However, this leaves a region end marker without a
        // matching start. Instead we insert a fake start into the loaded value and initially start with
        // a null selectedReadCarryIn state.

        loadedRegionStarts = b->CreateOr(loadedRegionStarts, regionStarts);
        initialSelectedReadCarryIn = BB_ZERO;
    } else {
        initialSelectedReadCarryIn = LOAD_CARRY_BIT("selectedReadCarry");
    }

    Value * const readLoadedStarts = b->CreateICmpEQ(initialRegionStartIndex, initialRegionEndIndex);
    Value * const initialRegionStarts = b->CreateSelect(readLoadedStarts, loadedRegionStarts, regionStarts);
    Value * const initialNumOfRegionStarts = popCountOf(b, fieldWidth, numOfFields, initialRegionStarts);

    Value * const initialSelectedRegionEnds = b->getScalarField("selectedRegionEnds");
    Value * const initialSelectedWriteCarryIn = LOAD_CARRY_BIT("selectedWriteCarry");
    Value * const initialQueue = b->getScalarField("queue");
    Value * const initialQueueLength = b->getScalarField("queueLength");
    Value * initialRegionCarryIn = nullptr;


    if (hasIndependentStartEndStreams()) {
        initialRegionCarryIn = LOAD_CARRY_BIT("regionCarry");
    }

    b->CreateBr(loopEntry);

    /// --------------------------------------------------------------------------------------
    /// Search for end position while recording the selection state of the current region
    /// --------------------------------------------------------------------------------------

    b->SetInsertPoint(loopEntry);
    PHINode * const regionStartIndexPhi = b->CreatePHI(sizeTy, 3);
    regionStartIndexPhi->addIncoming(initialRegionStartIndex, entry);
    PHINode * const regionEndIndexPhi = b->CreatePHI(sizeTy, 3);
    regionEndIndexPhi->addIncoming(initialRegionEndIndex, entry);
    PHINode * const priorRegionStartsPhi = b->CreatePHI(bbTy, 3);
    priorRegionStartsPhi->addIncoming(initialRegionStarts, entry);
    PHINode * const numOfRegionStartsPhi = b->CreatePHI(sizeTy, 3);
    numOfRegionStartsPhi->addIncoming(initialNumOfRegionStarts, entry);
    PHINode * const priorSelectedRegionEndsPhi = b->CreatePHI(bbTy, 3);
    priorSelectedRegionEndsPhi->addIncoming(initialSelectedRegionEnds, entry);
    PHINode * const selectedReadCarryInPhi = b->CreatePHI(bbTy, 3);  // extract
    selectedReadCarryInPhi->addIncoming(initialSelectedReadCarryIn, entry);
    PHINode * const selectedWriteCarryInPhi = b->CreatePHI(bbTy, 3); // deposit
    selectedWriteCarryInPhi->addIncoming(initialSelectedWriteCarryIn, entry);
    PHINode * const depositQueuePhi = b->CreatePHI(bbTy, 3);
    depositQueuePhi->addIncoming(initialQueue, entry);
    PHINode * const depositQueueLengthPhi = b->CreatePHI(sizeTy, 3);
    depositQueueLengthPhi->addIncoming(initialQueueLength, entry);

    PHINode * regionCarryIn = nullptr;
    if (initialRegionCarryIn) {
        regionCarryIn = b->CreatePHI(bbTy, 2);
        regionCarryIn->addIncoming(initialRegionCarryIn, entry);
    }

    Value * currentRegionEnds = getRegionEnds(b, regionEndIndexPhi);
    Value * currentRegionSpans = nullptr;
    Value * currentRegionStarts = nullptr;

    // TODO: we may have multiple start/end positions to consider
    Value * regionCarryOut = nullptr;
    if (regionCarryIn) {
        currentRegionStarts = getRegionStarts(b, regionEndIndexPhi);
        Value * const regionMask = b->CreateOr(currentRegionStarts, b->CreateNot(currentRegionEnds));
        std::tie(regionCarryOut, currentRegionSpans) =
            b->bitblock_add_with_carry(currentRegionStarts, regionMask, regionCarryIn);
        regionCarryIn->addIncoming(regionCarryOut, checkNextEnd);
        regionCarryIn->addIncoming(regionCarryOut, readNextStart);
        currentRegionEnds = b->CreateAnd(currentRegionEnds, currentRegionSpans);
    } else {
        currentRegionSpans = b->CreateNot(currentRegionEnds);
    }
    Value * selectedReadCarryOut = nullptr;
    Value * selectedRegionEnds = getSelectorStream(b, regionEndIndexPhi);

    // Use MatchStar(conditional, regionSpan) to ensure any conditional selectors are aligned with the region ends.
    if (!mSelectorsAreAlignedWithRegionEnds) {
        selectedRegionEnds = b->simd_and(selectedRegionEnds, currentRegionSpans);
        Value * partialSelectorSpan;
        std::tie(selectedReadCarryOut, partialSelectorSpan) =
            b->bitblock_add_with_carry(selectedRegionEnds, currentRegionSpans, selectedReadCarryInPhi);
        selectedReadCarryInPhi->addIncoming(selectedReadCarryOut, checkNextEnd);
        selectedReadCarryInPhi->addIncoming(selectedReadCarryOut, readNextStart);
        selectedRegionEnds = b->CreateOr(b->CreateXor(partialSelectorSpan, currentRegionSpans), selectedRegionEnds);
    }

    Value * const nextRegionEndIndex = b->CreateAdd(regionEndIndexPhi, ONE);

    Value * const anyEnds = b->bitblock_any(currentRegionEnds);
    b->CreateCondBr(anyEnds, hasRegionEnd, checkNextEnd);

    /// --------------------------------------------------------------------------------------
    /// We've found an end position; check if we can fill the start block
    /// --------------------------------------------------------------------------------------

    b->SetInsertPoint(hasRegionEnd);

    // Determine how many new selectors we'll enqueue

    Value * pendingQueueLength = popCountOf(b, fieldWidth, numOfFields, currentRegionEnds);

    // Extract the new selectors into a sequential stream

    Value * const pendingQueue = b->simd_pext(blockWidth, selectedRegionEnds, currentRegionEnds);

    // Combine the pending region ends with the new region ends

    // Since every end position has a single start position, we must have at most one unmatched
    // starting position. However, we cannot deposit any selectors in a block until we've found
    // all of the region ends for them. So, we have [0, BLOCK_WIDTH) queued region ends.

    Value * queuedRegionEnds = b->CreateShl(pendingQueue, depositQueueLengthPhi);
    queuedRegionEnds = b->CreateOr(queuedRegionEnds, depositQueuePhi);

    pendingQueueLength = b->CreateAdd(pendingQueueLength, depositQueueLengthPhi);

    // If regionStartIndexPhi < regionEndIndexPhi, canFill must be true; otherwise it will be
    // true 50% of the time when we have independent start/end streams and never true with a
    // single stream.

    Value * const canFill = b->CreateICmpUGE(pendingQueueLength, numOfRegionStartsPhi);
    b->CreateLikelyCondBr(canFill, writeRegionSpan, checkNextEnd);

    /// --------------------------------------------------------------------------------------
    /// Cannot fill the last region start block; advance the region end index.
    /// --------------------------------------------------------------------------------------

    b->SetInsertPoint(checkNextEnd);
    PHINode * const nextQueuedRegionEnds = b->CreatePHI(bbTy, 2);
    nextQueuedRegionEnds->addIncoming(depositQueuePhi, loopEntry);
    nextQueuedRegionEnds->addIncoming(queuedRegionEnds, hasRegionEnd);
    PHINode * const nextQueuedRegionEndLength = b->CreatePHI(sizeTy, 2);
    nextQueuedRegionEndLength->addIncoming(depositQueuePhi, loopEntry);
    nextQueuedRegionEndLength->addIncoming(queuedRegionEnds, hasRegionEnd);

    regionStartIndexPhi->addIncoming(regionStartIndexPhi, checkNextEnd);
    regionEndIndexPhi->addIncoming(nextRegionEndIndex, checkNextEnd);
    priorRegionStartsPhi->addIncoming(priorRegionStartsPhi, checkNextEnd);
    numOfRegionStartsPhi->addIncoming(numOfRegionStartsPhi, checkNextEnd);
    priorSelectedRegionEndsPhi->addIncoming(priorSelectedRegionEndsPhi, checkNextEnd);
    selectedWriteCarryInPhi->addIncoming(selectedWriteCarryInPhi, checkNextEnd);
    depositQueuePhi->addIncoming(nextQueuedRegionEnds, checkNextEnd);
    depositQueueLengthPhi->addIncoming(nextQueuedRegionEndLength, checkNextEnd);

    Value * const hasAnotherEnd = b->CreateICmpNE(nextRegionEndIndex, numOfStrides);
    b->CreateLikelyCondBr(hasAnotherEnd, loopEntry, checkFinal);

    /// --------------------------------------------------------------------------------------
    /// Deposit the selected regions markers in a selected region start stream.
    /// --------------------------------------------------------------------------------------

    b->SetInsertPoint(writeRegionSpan);
    PHINode * const allRegionStartsPhi = b->CreatePHI(bbTy, 2);
    allRegionStartsPhi->addIncoming(priorRegionStartsPhi, hasRegionEnd);
    PHINode * const currentQueuePhi = b->CreatePHI(bbIntTy, 2);
    currentQueuePhi->addIncoming(queuedRegionEnds, hasRegionEnd);
    PHINode * const selectedRegionEndsPhi = b->CreatePHI(bbTy, 2);
    selectedRegionEndsPhi->addIncoming(priorSelectedRegionEndsPhi, hasRegionEnd);
    PHINode * const currentIndexPhi = b->CreatePHI(sizeTy, 2);
    currentIndexPhi->addIncoming(regionStartIndexPhi, hasRegionEnd);
    PHINode * const numOfRequiredRegionStartsPhi = b->CreatePHI(sizeTy, 2);
    numOfRequiredRegionStartsPhi->addIncoming(numOfRegionStartsPhi, hasRegionEnd);
    PHINode * const currentQueueLengthPhi = b->CreatePHI(sizeTy, 2);
    currentQueueLengthPhi->addIncoming(pendingQueueLength, hasRegionEnd);
    PHINode * const selectedWriteCarryInPhi2 = b->CreatePHI(bbTy, 2);
    selectedWriteCarryInPhi2->addIncoming(selectedWriteCarryInPhi, hasRegionEnd);

    Value * const selectedStarts =
        b->simd_pdep(blockWidth, currentQueuePhi, allRegionStartsPhi);

    // Fill in the region span stream using SpanTo(selectedStarts, selectedEnds)

    Value * selectedWriteSpan, * selectedWriteCarryOut;
    std::tie(selectedWriteCarryOut, selectedWriteSpan) =
        b->bitblock_subtract_with_borrow(selectedStarts, selectedRegionEndsPhi, selectedWriteCarryInPhi2);
    b->storeOutputStreamBlock("regionSpan", ZERO, currentIndexPhi, selectedWriteSpan);

    // Since there is only one unmatched region start in the prior stream and there must be an end
    // in the current, we're guaranteed to consume the pending queue and one of our new selectors.

    Value * const nextQueueLength =
        b->CreateSub(currentQueueLengthPhi, numOfRequiredRegionStartsPhi);
    Value * const nextQueue =
        b->CreateLShr(pendingQueue, ONE);

    Value * const finished = b->CreateICmpEQ(currentIndexPhi, regionEndIndexPhi);
    b->CreateCondBr(finished, checkNextStart, extendRegion);

    /// --------------------------------------------------------------------------------------
    /// Extend the current region with 0/1 bits until we've caught up with the region end
    /// --------------------------------------------------------------------------------------

    b->SetInsertPoint(extendRegion);

    // Since there can be no starts prior to the next end, we only need to recall the last
    // incomplete and the current region block. Any blocks between the two must be all 0.
    // Consequentally, we can 0/1 fill any blocks between these two based on the state of
    // selectedCarryOut of the previously incomplete block.

    Value * const regionFill = splatCarryBit(b, fieldWidth, numOfFields, selectedWriteCarryOut);
    b->CreateBr(extendRegionLoop);

    /// --------------------------------------------------------------------------------------
    /// Extend any region with 0/1 bits until we've caught up with the region end
    /// --------------------------------------------------------------------------------------

    b->SetInsertPoint(extendRegionLoop);
    PHINode * const extensionIndex = b->CreatePHI(sizeTy, 2);
    extensionIndex->addIncoming(currentIndexPhi, writeRegionSpan);
    b->storeOutputStreamBlock("regionSpan", ZERO, extensionIndex, regionFill);
    Value * const nextExtensionIndex = b->CreateAdd(extensionIndex, ONE);
    extensionIndex->addIncoming(nextExtensionIndex, extendRegion);
    Value * const extendMore = b->CreateICmpNE(extensionIndex, regionEndIndexPhi);
    b->CreateCondBr(extendMore, extendRegionLoop, checkRemainingStarts);

    /// --------------------------------------------------------------------------------------
    /// Check if we can completely fill the current region block with the remaining selectors
    /// --------------------------------------------------------------------------------------

    b->SetInsertPoint(checkRemainingStarts);
    if (currentRegionStarts == nullptr) {
        currentRegionStarts = getRegionStarts(b, regionEndIndexPhi);
    }
    Value * const requiredNumOfRegionStarts = popCountOf(b, fieldWidth, numOfFields, currentRegionStarts);
    Value * hasEnough = nullptr;
    if (hasIndependentStartEndStreams()) {
        hasEnough = b->CreateICmpEQ(requiredNumOfRegionStarts, nextQueueLength);
    } else {
        hasEnough = b->getFalse();
    }
    allRegionStartsPhi->addIncoming(currentRegionStarts, checkRemainingStarts);
    selectedRegionEndsPhi->addIncoming(selectedRegionEnds, checkRemainingStarts);
    currentIndexPhi->addIncoming(regionEndIndexPhi, checkRemainingStarts);
    currentQueuePhi->addIncoming(nextQueue, checkRemainingStarts);
    selectedWriteCarryInPhi2->addIncoming(selectedWriteCarryOut, checkRemainingStarts);
    numOfRequiredRegionStartsPhi->addIncoming(requiredNumOfRegionStarts, checkRemainingStarts);
    currentQueueLengthPhi->addIncoming(nextQueueLength, checkRemainingStarts);
    b->CreateUnlikelyCondBr(hasEnough, writeRegionSpan, checkNextStart);

    /// --------------------------------------------------------------------------------------
    /// Check whether we have any more blocks to process
    /// --------------------------------------------------------------------------------------

    b->SetInsertPoint(checkNextStart);
    PHINode * const nextRegionStartIndexPhi = b->CreatePHI(sizeTy, 2);
    nextRegionStartIndexPhi->addIncoming(nextRegionEndIndex, writeRegionSpan);
    nextRegionStartIndexPhi->addIncoming(regionEndIndexPhi, checkRemainingStarts);

    PHINode * const nextSelectedWriteCarryInPhi = b->CreatePHI(bbTy, 2);
    nextSelectedWriteCarryInPhi->addIncoming(BB_ZERO, writeRegionSpan);
    nextSelectedWriteCarryInPhi->addIncoming(selectedWriteCarryOut, checkRemainingStarts);

    PHINode * const nextSelectedRegionEndsPhi = b->CreatePHI(bbTy, 2);
    nextSelectedRegionEndsPhi->addIncoming(BB_ZERO, writeRegionSpan);
    nextSelectedRegionEndsPhi->addIncoming(pendingQueue, checkRemainingStarts);

    PHINode * const nextQueuePhi = b->CreatePHI(sizeTy, 2);
    nextQueuePhi->addIncoming(BB_ZERO, writeRegionSpan);
    nextQueuePhi->addIncoming(nextQueue, checkRemainingStarts);

    PHINode * const nextQueueLengthPhi = b->CreatePHI(sizeTy, 2);
    nextQueueLengthPhi->addIncoming(ZERO, writeRegionSpan);
    nextQueueLengthPhi->addIncoming(nextQueueLength, checkRemainingStarts);

    Value * const readStart = b->CreateICmpNE(nextRegionStartIndexPhi, numOfStrides);
    b->CreateLikelyCondBr(readStart, readNextStart, checkFinal);

    /// --------------------------------------------------------------------------------------
    /// We can safely read the next start/end region. Load in any new data.
    /// --------------------------------------------------------------------------------------

    b->SetInsertPoint(readNextStart);
    Value * const nextRegionStarts = getRegionStarts(b, nextRegionStartIndexPhi);
    Value * const nextNumOfRegionStarts = popCountOf(b, fieldWidth, numOfFields, nextRegionStarts);
    Value * const notDone = b->CreateICmpNE(nextRegionEndIndex, numOfStrides);
    b->CreateLikelyCondBr(notDone, loopEntry, checkFinal);

    regionStartIndexPhi->addIncoming(nextRegionStartIndexPhi, readNextStart);
    regionEndIndexPhi->addIncoming(nextRegionEndIndex, readNextStart);
    priorSelectedRegionEndsPhi->addIncoming(nextSelectedRegionEndsPhi, readNextStart);
    selectedWriteCarryInPhi->addIncoming(nextSelectedWriteCarryInPhi, readNextStart);
    numOfRegionStartsPhi->addIncoming(nextNumOfRegionStarts, readNextStart);
    depositQueuePhi->addIncoming(nextQueuePhi, readNextStart);
    depositQueueLengthPhi->addIncoming(nextQueueLengthPhi, readNextStart);
    priorRegionStartsPhi->addIncoming(nextRegionStarts, readNextStart);
    numOfRegionStartsPhi->addIncoming(nextNumOfRegionStarts, readNextStart);

    /// --------------------------------------------------------------------------------------
    /// Final Block
    /// --------------------------------------------------------------------------------------

    b->SetInsertPoint(checkFinal);
    PHINode * const lastRegionStartIndexPhi = b->CreatePHI(sizeTy, 3);
    lastRegionStartIndexPhi->addIncoming(regionStartIndexPhi, checkNextEnd);
    lastRegionStartIndexPhi->addIncoming(numOfStrides, checkNextStart);
    lastRegionStartIndexPhi->addIncoming(nextRegionStartIndexPhi, readNextStart);
    PHINode * const lastRegionStartsPhi = b->CreatePHI(bbTy, 3);
    lastRegionStartsPhi->addIncoming(priorRegionStartsPhi, checkNextEnd);
    lastRegionStartsPhi->addIncoming(currentRegionStarts, checkNextStart);
    lastRegionStartsPhi->addIncoming(nextRegionStarts, readNextStart);
    PHINode * const lastSelectedRegionEndsPhi = b->CreatePHI(bbTy, 3);
    lastSelectedRegionEndsPhi->addIncoming(priorSelectedRegionEndsPhi, checkNextEnd);
    lastSelectedRegionEndsPhi->addIncoming(nextSelectedRegionEndsPhi, checkNextStart);
    lastSelectedRegionEndsPhi->addIncoming(nextSelectedRegionEndsPhi, readNextStart);
    PHINode * const lastSelectedWriteCarryInPhi = b->CreatePHI(bbTy, 3);
    lastSelectedWriteCarryInPhi->addIncoming(selectedWriteCarryInPhi, checkNextEnd);
    lastSelectedWriteCarryInPhi->addIncoming(nextSelectedWriteCarryInPhi, checkNextStart);
    lastSelectedWriteCarryInPhi->addIncoming(nextSelectedWriteCarryInPhi, readNextStart);
    PHINode * const lastDepositQueuePhi = b->CreatePHI(bbTy, 3);
    lastDepositQueuePhi->addIncoming(nextQueuedRegionEnds, checkNextEnd);
    lastDepositQueuePhi->addIncoming(nextQueuePhi, checkNextStart);
    lastDepositQueuePhi->addIncoming(nextQueuePhi, readNextStart);
    PHINode * const lastDepositQueueLengthPhi = b->CreatePHI(sizeTy, 3);
    lastDepositQueueLengthPhi->addIncoming(nextQueuedRegionEndLength, checkNextEnd);
    lastDepositQueueLengthPhi->addIncoming(nextQueueLengthPhi, checkNextStart);
    lastDepositQueueLengthPhi->addIncoming(nextQueueLengthPhi, readNextStart);

    // If the selected region carry out is non-zero, we do not have to wait to find the
    // region end before depositing a 1-bit in the last region start position. But to take
    // advantage of that, we must "ignore" the next end position that yet still fill the
    // span up to it.

    // NOTE: if we planned to reverse the stream later, we cannot do this.
    Value * writeToEnd = mIsFinal;
    if (mAlwaysExtendSelectedRegionsToRegionEnds) {
        writeToEnd = b->CreateOr(writeToEnd, b->bitblock_any(selectedReadCarryOut));
    }

    // If we're not in any region, we can zero fill the region span to the end
    if (regionCarryOut) {
        Value * const notWithinRegion = b->CreateNot(b->bitblock_any(regionCarryOut));
        writeToEnd = b->CreateOr(writeToEnd, notWithinRegion);
    }

    Value * const writtenItems = b->CreateShl(lastRegionStartIndexPhi, LOG_2_BLOCK_WIDTH);
    b->CreateUnlikelyCondBr(writeToEnd, writeFinal, exit);

    b->SetInsertPoint(writeFinal);
    Value * earlySelector = b->CreateShl(selectedReadCarryOut, lastDepositQueueLengthPhi);
    Value * finalRegionEnds = b->CreateOr(nextQueue, earlySelector);

    Value * const finalSelectedStarts =
        b->simd_pdep(blockWidth, finalRegionEnds, lastRegionStartsPhi);
    std::tie(selectedWriteCarryOut, selectedWriteSpan) =
        b->bitblock_subtract_with_borrow(finalSelectedStarts, lastSelectedRegionEndsPhi, lastSelectedWriteCarryInPhi);
    b->storeOutputStreamBlock("regionSpan", ZERO, lastRegionStartIndexPhi, selectedWriteSpan);

    Value * const finalRegionFill = splatCarryBit(b, fieldWidth, numOfFields, selectedWriteCarryOut);
    Value * const extendFinalSpan = b->CreateICmpNE(lastRegionStartIndexPhi, numOfStrides);
    Value * const writableOutputItems = getWritableOutputItems(0);
    b->CreateCondBr(extendFinalSpan, extendFinal, exit);

    b->SetInsertPoint(extendFinal);
    PHINode * const index = b->CreatePHI(sizeTy, 2);
    index->addIncoming(nextRegionStartIndexPhi, writeFinal);
    b->storeOutputStreamBlock("regionSpan", ZERO, index, finalRegionFill);
    Value * const nextIndex = b->CreateAdd(index, ONE);
    index->addIncoming(nextIndex, extendFinal);
    Value * const extendFinalMore = b->CreateICmpNE(index, numOfStrides);
    b->CreateCondBr(extendFinalMore, extendFinal, exit);

    /// --------------------------------------------------------------------------------------
    /// Exit the kernel, storing any state for the next segment
    /// --------------------------------------------------------------------------------------

    b->SetInsertPoint(exit);
    PHINode * const finalProducedItemCount = b->CreatePHI(sizeTy, 3);
    finalProducedItemCount->addIncoming(writtenItems, checkFinal);
    finalProducedItemCount->addIncoming(writableOutputItems, writeFinal);
    finalProducedItemCount->addIncoming(writableOutputItems, extendFinal);
    PHINode * const finalRegionStarts = b->CreatePHI(bbTy, 3);
    finalRegionStarts->addIncoming(lastRegionStartsPhi, checkFinal);
    Value * newRegionStart = BB_ZERO;
    if (mAlwaysExtendSelectedRegionsToRegionEnds) {
        newRegionStart = selectedReadCarryOut;
    }
    finalRegionStarts->addIncoming(newRegionStart, writeFinal);
    finalRegionStarts->addIncoming(newRegionStart, extendFinal);

    PHINode * const finalQueue = b->CreatePHI(bbIntTy, 3);
    finalQueue->addIncoming(lastDepositQueuePhi, checkFinal);
    finalQueue->addIncoming(INT_ZEROS, writeFinal);
    finalQueue->addIncoming(INT_ZEROS, extendFinal);
    PHINode * const finalQueueLength = b->CreatePHI(sizeTy, 3);
    finalQueueLength->addIncoming(lastDepositQueueLengthPhi, checkFinal);
    finalQueueLength->addIncoming(ZERO, writeFinal);
    finalQueueLength->addIncoming(ZERO, extendFinal);
    PHINode * const finalSelectedWriteCarry = b->CreatePHI(bbTy, 3);
    finalSelectedWriteCarry->addIncoming(lastSelectedWriteCarryInPhi, checkFinal);
    finalSelectedWriteCarry->addIncoming(selectedWriteCarryOut, writeFinal);
    finalSelectedWriteCarry->addIncoming(selectedWriteCarryOut, extendFinal);

    Value * const produced = b->CreateAdd(initiallyProduced, finalProducedItemCount);
    b->setProcessedItemCount("regionSpan", produced);
    b->setScalarField("regionStarts", finalRegionStarts);
    b->setScalarField("selectedRegionEnds", lastSelectedRegionEndsPhi);
    if (!mAlwaysExtendSelectedRegionsToRegionEnds) {
        STORE_CARRY_BIT("selectedReadCarry", selectedReadCarryOut);
    }
    STORE_CARRY_BIT("selectedWriteCarry", finalSelectedWriteCarry);
    b->setScalarField("queue", finalQueue);
    b->setScalarField("queueLength", finalQueueLength);
    if (regionCarryOut) {
        STORE_CARRY_BIT("regionCarry", regionCarryOut);
    }

}

Value * RegionSelectionKernel::getRegionStarts(const std::unique_ptr<kernel::KernelBuilder> & b, Value * const offset) const {
    if (hasIndependentStartEndStreams()) {
        return b->getInputStreamBlockPtr("starts", b->getSize(mStartStreamIndex), offset);
    } else {
        return b->getInputStreamBlockPtr("demarcators", b->getSize(mStartStreamIndex), offset);
    }
}

Value * RegionSelectionKernel::getNumOfRegionStarts(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    if (hasIndependentStartEndStreams()) {
        return b->getProcessedItemCount("starts");
    } else {
        return b->getProcessedItemCount("demarcators");
    }
}

Value * RegionSelectionKernel::getRegionEnds(const std::unique_ptr<kernel::KernelBuilder> & b, Value * const offset) const {
    if (hasIndependentStartEndStreams()) {
        return b->getInputStreamBlockPtr("ends", b->getSize(mEndStreamIndex), offset);
    } else {
        return b->getInputStreamBlockPtr("demarcators", b->getSize(mStartStreamIndex), offset);
    }
}

Value * RegionSelectionKernel::getNumOfRegionEnds(const std::unique_ptr<kernel::KernelBuilder> & b) const {
    if (hasIndependentStartEndStreams()) {
        return b->getProcessedItemCount("ends");
    } else {
        return b->getProcessedItemCount("demarcators");
    }
}

Value * RegionSelectionKernel::getSelectorStream(const std::unique_ptr<kernel::KernelBuilder> & b, Value * const offset) const {
    assert (hasSelectorStream());
    return b->getInputStreamBlockPtr("selectors", b->getSize(mSelectorStreamIndex), offset);
}

bool RegionSelectionKernel::hasSelectorStream() const {
    return mSelectorStreamIndex != -1U;
}

bool RegionSelectionKernel::hasIndependentStartEndStreams() const {
    return mEndStreamIndex != -1U;
}

#define NAME_PREFIX "RegionSelect"

RegionSelectionKernel::RegionSelectionKernel(const std::unique_ptr<kernel::KernelBuilder> & b, Starts starts, Ends ends, StreamSet * const regionSpans)
: MultiBlockKernel(b, NAME_PREFIX "A" + std::to_string(starts.Index) + "." + std::to_string(ends.Index)
// input streams
,{Binding{"starts", starts.Stream}
, Binding{"ends", ends.Stream}}
// output stream
,{Binding{"regionSpans", regionSpans, FixedRate(), Deferred()}}
// unnused scalars
,{},{},{})
, mStartStreamIndex(starts.Index)
, mEndStreamIndex(ends.Index)
, mSelectorStreamIndex(-1U)
, mSelectorsAreAlignedWithRegionEnds(false)
, mAlwaysExtendSelectedRegionsToRegionEnds(true) {

    report_fatal_error("Non-selector region kernel is not supported yet.");
}

RegionSelectionKernel::RegionSelectionKernel(const std::unique_ptr<kernel::KernelBuilder> & b, Demarcators demarcators, Selectors selectors, StreamSet * const regionSpans)
: MultiBlockKernel(b, NAME_PREFIX "S" + std::to_string(demarcators.Index) + "." + std::to_string(selectors.Index)
// input streams
,{Binding{"demarcators", demarcators.Stream}
, Binding{"selectors", selectors.Stream}}
// output stream
,{Binding{"regionSpans", regionSpans, FixedRate(), Deferred()}}
// unnused scalars
,{} ,{},{})
, mStartStreamIndex(demarcators.Index)
, mEndStreamIndex(-1U)
, mSelectorStreamIndex(selectors.Index)
, mSelectorsAreAlignedWithRegionEnds(false)
, mAlwaysExtendSelectedRegionsToRegionEnds(true) {

    // addInternalScalar(b->getSizeTy(), "pendingCount");

}

RegionSelectionKernel::RegionSelectionKernel(const std::unique_ptr<kernel::KernelBuilder> & b, Starts starts, Ends ends, Selectors selectors, StreamSet * const regionSpans)
: MultiBlockKernel(b, NAME_PREFIX "S" + std::to_string(starts.Index) + "." + std::to_string(ends.Index) + "." + std::to_string(selectors.Index)
// input streams
,{Binding{"starts", starts.Stream}
, Binding{"ends", ends.Stream}
, Binding{"selectors", selectors.Stream}}
// output stream
,{Binding{"regionSpans", regionSpans, FixedRate(), Deferred()}}
// unnused scalars
,{} ,{},{})
, mStartStreamIndex(starts.Index)
, mEndStreamIndex(ends.Index)
, mSelectorStreamIndex(selectors.Index)
, mSelectorsAreAlignedWithRegionEnds(false)
, mAlwaysExtendSelectedRegionsToRegionEnds(true) {

}

}
