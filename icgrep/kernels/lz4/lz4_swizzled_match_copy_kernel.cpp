//
// Created by wxy325 on 2018/3/9.
//

#include "lz4_swizzled_match_copy_kernel.h"
#include <kernels/kernel_builder.h>

using namespace llvm;

namespace kernel {

void LZ4SwizzledMatchCopyKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {

    ConstantInt * const SIZE_ZERO = iBuilder->getSize(0);
    ConstantInt * const SIZE_ONE = iBuilder->getSize(1);
    ConstantInt * const SIZE_PDEP_WIDTH = iBuilder->getSize(mPDEPWidth);
    ConstantInt * const SIZE_4_MEGS = iBuilder->getSize(4 * 1024 * 1024);
    ConstantInt * const SIZE_BLOCK_WIDTH = iBuilder->getSize(iBuilder->getBitBlockWidth());

    BasicBlock * const entryBlock = iBuilder->GetInsertBlock();

    Value * const available = iBuilder->getAvailableItemCount("sourceStreamSet0");
    Value * const processed = iBuilder->getProcessedItemCount("sourceStreamSet0");

    Value * const itemsToDo = iBuilder->CreateUMin(iBuilder->CreateSub(available, processed), SIZE_4_MEGS);

    iBuilder->setTerminationSignal(iBuilder->CreateICmpULT(itemsToDo, SIZE_4_MEGS));

    Value * previousProducedItemCount = iBuilder->getProducedItemCount("outputStreamSet0");

    // Output Copy
    generateOutputCopy(iBuilder);

    Value * const toProcessItemCount = iBuilder->CreateAdd(processed, itemsToDo);

    // Match Copy
    Value * const initM0StartProcessIndex = iBuilder->getProcessedItemCount("m0Start");
    Value * const totalM0StartItemsCount = iBuilder->getAvailableItemCount("m0Start");

    Value * const initMatchOffset = iBuilder->getScalarField("pendingMatchOffset");
    Value * const initMatchLength = iBuilder->getScalarField("pendingMatchLength");
    Value * const initMatchPos = iBuilder->getScalarField("pendingMatchPos");

    BasicBlock * const matchCopyLoopCon = iBuilder->CreateBasicBlock("matchCopyLoopCon");
    iBuilder->CreateBr(matchCopyLoopCon);

    iBuilder->SetInsertPoint(matchCopyLoopCon);
    PHINode * const phiProcessIndex = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3);
    phiProcessIndex->addIncoming(initM0StartProcessIndex, entryBlock);
    PHINode * const phiMatchOffset = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3);
    phiMatchOffset->addIncoming(initMatchOffset, entryBlock);
    PHINode * const phiMatchLength = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3);
    phiMatchLength->addIncoming(initMatchLength, entryBlock);
    PHINode * const phiMatchPos = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3);
    phiMatchPos->addIncoming(initMatchPos, entryBlock);

    BasicBlock * const loadNextMatchInfoConBlock = iBuilder->CreateBasicBlock("loadNewMatchInfoConBlock");
    BasicBlock * const loadNextMatchInfoBodyBlock = iBuilder->CreateBasicBlock("loadNewMatchInfoBodyBlock");

    BasicBlock * const matchCopyConBlock = iBuilder->CreateBasicBlock("matchCopyConBlock");
    BasicBlock * const matchCopyBodyBlock = iBuilder->CreateBasicBlock("matchCopyBodyBlock");

    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(phiMatchLength, SIZE_ZERO), loadNextMatchInfoConBlock, matchCopyConBlock);

    iBuilder->SetInsertPoint(loadNextMatchInfoConBlock);
    Value * const hasMoreMatchInfo = iBuilder->CreateICmpULT(phiProcessIndex, totalM0StartItemsCount);
    BasicBlock * const processExitBlock = iBuilder->CreateBasicBlock("exit_block");
    iBuilder->CreateCondBr(hasMoreMatchInfo, loadNextMatchInfoBodyBlock, processExitBlock);

    iBuilder->SetInsertPoint(loadNextMatchInfoBodyBlock);

    Value * const newM0Start = loadOffset(iBuilder, "m0Start", phiProcessIndex);
    Value * const newM0End = loadOffset(iBuilder, "m0End", phiProcessIndex);
    Value * const newMatchOffset = loadOffset(iBuilder, "matchOffset", phiProcessIndex);
    Value * const newMatchLength = iBuilder->CreateAdd(iBuilder->CreateSub(newM0End, newM0Start), iBuilder->getInt64(1));

    phiProcessIndex->addIncoming(iBuilder->CreateAdd(phiProcessIndex, SIZE_ONE), loadNextMatchInfoBodyBlock);

    phiMatchPos->addIncoming(newM0Start, loadNextMatchInfoBodyBlock);
    phiMatchOffset->addIncoming(newMatchOffset, loadNextMatchInfoBodyBlock);
    phiMatchLength->addIncoming(newMatchLength, loadNextMatchInfoBodyBlock);

    iBuilder->CreateBr(matchCopyLoopCon);

    iBuilder->SetInsertPoint(matchCopyConBlock);

    Value * const hasNotReachEnd = iBuilder->CreateICmpULT(phiMatchPos, toProcessItemCount);
    iBuilder->CreateCondBr(hasNotReachEnd, matchCopyBodyBlock, processExitBlock);

    iBuilder->SetInsertPoint(matchCopyBodyBlock);

    Value * const matchCopyTargetPos = iBuilder->CreateSub(phiMatchPos, previousProducedItemCount);
    Value * const matchCopyTargetBlockIndex = iBuilder->CreateUDiv(matchCopyTargetPos, SIZE_BLOCK_WIDTH);
    Value * const matchCopyTargetStreamIndex = iBuilder->CreateUDiv(iBuilder->CreateURem(matchCopyTargetPos, SIZE_BLOCK_WIDTH), SIZE_PDEP_WIDTH); // should SIZE_PDEP_WIDTH be SIZE_STREAM_COUNT?
    Value * const matchCopyTargetBlockOffset = iBuilder->CreateURem(phiMatchPos, SIZE_PDEP_WIDTH);

    Value * const matchCopyFromPos = iBuilder->CreateSub(matchCopyTargetPos, phiMatchOffset);
    Value * const matchCopyFromBlockIndex = iBuilder->CreateUDiv(matchCopyFromPos, SIZE_BLOCK_WIDTH);
    Value * const matchCopyFromStreamIndex = iBuilder->CreateUDiv(iBuilder->CreateURem(matchCopyFromPos, SIZE_BLOCK_WIDTH), SIZE_PDEP_WIDTH);
    Value * const matchCopyFromBlockOffset = iBuilder->CreateURem(matchCopyFromPos, SIZE_PDEP_WIDTH);

    Value * currentCopySize = iBuilder->CreateSub(SIZE_PDEP_WIDTH, iBuilder->CreateUMax(matchCopyFromBlockOffset, matchCopyTargetBlockOffset));
    currentCopySize = iBuilder->CreateUMin(currentCopySize, phiMatchOffset);
    currentCopySize = iBuilder->CreateUMin(currentCopySize, phiMatchLength);
    currentCopySize = iBuilder->CreateUMin(currentCopySize, iBuilder->CreateSub(toProcessItemCount, phiMatchPos));
    currentCopySize = iBuilder->CreateSelect(iBuilder->CreateICmpEQ(currentCopySize, SIZE_ZERO), SIZE_ONE, currentCopySize); //Workaround for the last byte

    Value * const shiftOffset = iBuilder->CreateAdd(matchCopyFromBlockOffset, currentCopySize);
    Value * highOffset = iBuilder->CreateShl(SIZE_ONE, shiftOffset);
    highOffset = iBuilder->CreateSelect(iBuilder->CreateICmpEQ(currentCopySize, SIZE_PDEP_WIDTH), SIZE_ZERO, highOffset); // When currentCopySize == SIZE_PDEP_WIDTH, shl will overflow
    Value * const lowOffset = iBuilder->CreateShl(SIZE_ONE, matchCopyFromBlockOffset);
    Value * const maskVector = iBuilder->simd_fill(mPDEPWidth, iBuilder->CreateSub(highOffset, lowOffset));
    Value * const fromBlockOffsetVector = iBuilder->simd_fill(mPDEPWidth, matchCopyFromBlockOffset);
    Value * const targetBlockOffsetVector = iBuilder->simd_fill(mPDEPWidth, matchCopyTargetBlockOffset);

    for (unsigned i = 0; i < mStreamSize; i++) {
        Value * const matchCopyFromBlockPtr = iBuilder->getOutputStreamBlockPtr("outputStreamSet" + std::to_string(i), matchCopyFromStreamIndex, matchCopyFromBlockIndex);
        Value * const fromBlockValue = iBuilder->CreateBlockAlignedLoad(matchCopyFromBlockPtr);

        Value * const outputTargetBlockPtr = iBuilder->getOutputStreamBlockPtr("outputStreamSet" + std::to_string(i), matchCopyTargetStreamIndex, matchCopyTargetBlockIndex);
        Value * const targetOriginalValue = iBuilder->CreateBlockAlignedLoad(outputTargetBlockPtr);

        Value * copiedValue = iBuilder->simd_and(fromBlockValue, maskVector);
        copiedValue = iBuilder->CreateLShr(copiedValue, fromBlockOffsetVector);
        copiedValue = iBuilder->CreateShl(copiedValue, targetBlockOffsetVector);
        Value * const finalValue = iBuilder->CreateOr(targetOriginalValue, copiedValue);

        iBuilder->CreateStore(finalValue, outputTargetBlockPtr);
    }

    phiProcessIndex->addIncoming(phiProcessIndex, matchCopyBodyBlock);
    phiMatchOffset->addIncoming(phiMatchOffset, matchCopyBodyBlock);
    phiMatchPos->addIncoming(iBuilder->CreateAdd(phiMatchPos, currentCopySize), matchCopyBodyBlock);
    phiMatchLength->addIncoming(iBuilder->CreateSub(phiMatchLength, currentCopySize), matchCopyBodyBlock);

    iBuilder->CreateBr(matchCopyLoopCon);

    iBuilder->SetInsertPoint(processExitBlock);
    iBuilder->setScalarField("pendingMatchOffset", phiMatchOffset);
    iBuilder->setScalarField("pendingMatchLength", phiMatchLength);
    iBuilder->setScalarField("pendingMatchPos", phiMatchPos);
    iBuilder->setProcessedItemCount("m0Start", phiProcessIndex);
    iBuilder->setProcessedItemCount("m0End", phiProcessIndex);
    iBuilder->setProcessedItemCount("matchOffset", phiProcessIndex);
    iBuilder->setProcessedItemCount("sourceStreamSet0", toProcessItemCount);
}

void LZ4SwizzledMatchCopyKernel::generateOutputCopy(const std::unique_ptr<KernelBuilder> & iBuilder) {
    Constant * SIZE_ZERO = iBuilder->getSize(0);
    Constant * COPY_BYTES = iBuilder->getSize(4 * 1024 * 1024 * mStreamCount / 8);
    for (unsigned i = 0; i < mStreamSize; i++) {
        Value * inputBasePtr = iBuilder->getInputStreamBlockPtr("sourceStreamSet" + std::to_string(i), SIZE_ZERO);
        Value * outputBasePtr = iBuilder->getOutputStreamBlockPtr("outputStreamSet" + std::to_string(i), SIZE_ZERO);
        iBuilder->CreateMemCpy(outputBasePtr, inputBasePtr, COPY_BYTES, 1); // Not align guaranteed in final block
    }
}

Value* LZ4SwizzledMatchCopyKernel::loadOffset(const std::unique_ptr<KernelBuilder> & iBuilder, const std::string & bufferName, Value* offset) {
    return iBuilder->CreateLoad(iBuilder->getRawInputPointer(bufferName, offset));
}

LZ4SwizzledMatchCopyKernel::LZ4SwizzledMatchCopyKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder, unsigned streamCount/*=4*/, unsigned streamSize/*=2*/, unsigned swizzleFactor/*=4*/, unsigned PDEP_width/*64*/)
: SegmentOrientedKernel("LZ4SwizzledMatchCopyKernel",
// Inputs
{
       Binding{iBuilder->getStreamSetTy(1, 64), "m0Start", BoundedRate(0, 1), DisableSufficientChecking()},
       Binding{iBuilder->getStreamSetTy(1, 64), "m0End", BoundedRate(0, 1), DisableSufficientChecking()},
       Binding{iBuilder->getStreamSetTy(1, 64), "matchOffset", BoundedRate(0, 1), DisableSufficientChecking()},
},
// Outputs
{},
// Arguments
{
       Binding{iBuilder->getSizeTy(), "fileSize"} //TODO remove
},
{},
{
       Binding{iBuilder->getSizeTy(), "currentProcessIndex"},
       Binding{iBuilder->getSizeTy(), "pendingMatchPos"},
       Binding{iBuilder->getSizeTy(), "pendingMatchOffset"},
       Binding{iBuilder->getSizeTy(), "pendingMatchLength"},
})
, mSwizzleFactor(swizzleFactor)
, mPDEPWidth(PDEP_width)
, mStreamSize(streamSize)
, mStreamCount(streamCount) {

    assert((mSwizzleFactor == (iBuilder->getBitBlockWidth() / PDEP_width)) && "swizzle factor must equal bitBlockWidth / PDEP_width");
    assert((mPDEPWidth == 64 || mPDEPWidth == 32) && "PDEP width must be 32 or 64");
    setStride(4 * 1024 * 1024);
    addAttribute(MustExplicitlyTerminate());

    mStreamSetInputs.push_back(Binding{iBuilder->getStreamSetTy(streamCount), "sourceStreamSet0", BoundedRate(0, 1), Swizzled()});
    mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(streamCount), "outputStreamSet0", RateEqualTo("sourceStreamSet0")});

    for (unsigned i = 1; i < streamSize; i++) {
        mStreamSetInputs.push_back(Binding{iBuilder->getStreamSetTy(streamCount), "sourceStreamSet" + std::to_string(i), RateEqualTo("sourceStreamSet0"), Swizzled()});
        mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(streamCount), "outputStreamSet" + std::to_string(i), RateEqualTo("sourceStreamSet0")});
    }
}

}
