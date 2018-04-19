//
// Created by wxy325 on 2018/3/9.
//

#include "lz4_swizzled_match_copy_kernel.h"
#include <kernels/kernel_builder.h>
#include <kernels/streamset.h>
#include <toolchain/toolchain.h>
#include <vector>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;
using namespace std;
namespace kernel {

Value *LZ4SwizzledMatchCopyKernel::advanceUntilNextBit(const std::unique_ptr<KernelBuilder> &iBuilder, string inputName, Value *startPos, bool isNextOne) {
    BasicBlock* entryBlock = iBuilder->GetInsertBlock();

    Constant* SIZE_0 = iBuilder->getSize(0);
    Constant* SIZE_1 = iBuilder->getSize(1);
    Value* SIZE_64 = iBuilder->getSize(64); // maybe need to handle 32 bit machine
    Value* SIZE_INPUT_64_COUNT = iBuilder->getSize(this->getInputStreamSetBuffer(inputName)->getBufferBlocks() * iBuilder->getBitBlockWidth() / 64);

    Value* initCurrentPos = startPos;

    Value* offsetMarkerRawPtr = iBuilder->CreatePointerCast(iBuilder->getRawInputPointer(inputName, SIZE_0), iBuilder->getInt64Ty()->getPointerTo());

    BasicBlock* findNextMatchOffsetConBlock = iBuilder->CreateBasicBlock("findNextMatchOffsetConBlock");
    BasicBlock* findNextMatchOffsetBodyBlock = iBuilder->CreateBasicBlock("findNextMatchOffsetBodyBlock");

    iBuilder->CreateBr(findNextMatchOffsetConBlock);
    iBuilder->SetInsertPoint(findNextMatchOffsetConBlock);
    // Find position marker bit of next 1 bit

    PHINode* phiCurrentPos = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
    phiCurrentPos->addIncoming(initCurrentPos, entryBlock);

    Value* currentPosGlobalBlockIndex = iBuilder->CreateUDiv(phiCurrentPos, SIZE_64);
    Value* currentPosLocalBlockIndex = iBuilder->CreateURem(currentPosGlobalBlockIndex, SIZE_INPUT_64_COUNT);
    Value* currentPosBlockOffset = iBuilder->CreateURem(phiCurrentPos, SIZE_64);
    Value* currentValue = iBuilder->CreateLoad(iBuilder->CreateGEP(offsetMarkerRawPtr, currentPosLocalBlockIndex));

    Value* countValue = iBuilder->CreateLShr(currentValue, currentPosBlockOffset);
    if (!isNextOne) {
        countValue = iBuilder->CreateNot(countValue);
    }
    Value* forwardZero = iBuilder->CreateCountForwardZeroes(countValue);
    Value* realForwardZero = iBuilder->CreateAdd(currentPosBlockOffset, forwardZero);

    // If targetMarker == 0, move to next block, otherwise count forward zero
    phiCurrentPos->addIncoming(iBuilder->CreateMul(SIZE_64, iBuilder->CreateAdd(currentPosGlobalBlockIndex, SIZE_1)), iBuilder->GetInsertBlock());
    iBuilder->CreateCondBr(iBuilder->CreateICmpUGE(realForwardZero, SIZE_64), findNextMatchOffsetConBlock, findNextMatchOffsetBodyBlock);

    iBuilder->SetInsertPoint(findNextMatchOffsetBodyBlock);

    Value* newPosition = iBuilder->CreateAdd(iBuilder->CreateMul(currentPosGlobalBlockIndex, SIZE_64), realForwardZero);

    return newPosition;
}

pair<Value*, Value*> LZ4SwizzledMatchCopyKernel::loadNextMatchOffset(const unique_ptr<KernelBuilder> &iBuilder) {
    Value* initCurrentPos = iBuilder->CreateAdd(iBuilder->getScalarField("currentOffsetMarkerPos"), iBuilder->getSize(1));
    Value* newPosition = this->advanceUntilNextBit(iBuilder, "MatchOffsetMarker", initCurrentPos, true);

    // Load Match Offset from newPosition
    Value* matchOffsetPtr = iBuilder->getRawInputPointer("byteStream", newPosition);
    // For now, it is safe to cast matchOffset pointer into i16 since the input byte stream is always linear available
    matchOffsetPtr = iBuilder->CreatePointerCast(matchOffsetPtr, iBuilder->getInt16Ty()->getPointerTo());
    Value* matchOffset = iBuilder->CreateZExt(iBuilder->CreateLoad(matchOffsetPtr), iBuilder->getSizeTy());

    return std::make_pair(matchOffset, newPosition);
}

pair<Value*, Value*> LZ4SwizzledMatchCopyKernel::loadNextM0StartEnd(const unique_ptr<KernelBuilder> &iBuilder) {
    Value* initCurrentPos = iBuilder->getScalarField("currentM0MarkerPos");
    Value* m0Start = this->advanceUntilNextBit(iBuilder, "M0Marker", initCurrentPos, true);
    Value* m0End = this->advanceUntilNextBit(iBuilder, "M0Marker", m0Start, false);
    return std::make_pair(m0Start, m0End);
};


void LZ4SwizzledMatchCopyKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    ConstantInt * const SIZE_4_MEGS = iBuilder->getSize(4 * 1024 * 1024);

    BasicBlock * const entryBlock = iBuilder->GetInsertBlock();

    Value * const available = iBuilder->getAvailableItemCount("sourceStreamSet0");
    Value * const processed = iBuilder->getProcessedItemCount("sourceStreamSet0");

    Value * const itemsToDo = iBuilder->CreateUMin(iBuilder->CreateSub(available, processed), SIZE_4_MEGS);
    iBuilder->setTerminationSignal(iBuilder->CreateICmpULT(itemsToDo, SIZE_4_MEGS));


    // Output Copy
    generateOutputCopy(iBuilder);

    Value * const toProcessItemCount = iBuilder->CreateAdd(processed, itemsToDo);

    // Match Copy
    Value *initM0StartProcessIndex = iBuilder->getProcessedItemCount("M0CountMarker");
    Value *totalM0StartItemsCount = iBuilder->getAvailableItemCount("M0CountMarker");

    BasicBlock * const matchCopyLoopCon = iBuilder->CreateBasicBlock("matchCopyLoopCon");
    BasicBlock * const processExitBlock = iBuilder->CreateBasicBlock("exit_block");

    BasicBlock * const loadNextMatchInfoBodyBlock = iBuilder->CreateBasicBlock("loadNewMatchInfoBodyBlock");
    BasicBlock * const matchCopyConBlock = iBuilder->CreateBasicBlock("matchCopyConBlock");
    BasicBlock * const matchCopyBodyBlock = iBuilder->CreateBasicBlock("matchCopyBodyBlock");


    iBuilder->CreateBr(matchCopyLoopCon);

    iBuilder->SetInsertPoint(matchCopyLoopCon);
    PHINode * const phiProcessIndex = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
    phiProcessIndex->addIncoming(initM0StartProcessIndex, entryBlock);

    Value * const hasMoreMatchInfo = iBuilder->CreateICmpULT(phiProcessIndex, totalM0StartItemsCount);

    iBuilder->CreateCondBr(hasMoreMatchInfo, loadNextMatchInfoBodyBlock, processExitBlock);

    iBuilder->SetInsertPoint(loadNextMatchInfoBodyBlock);

    auto ret = this->loadNextM0StartEnd(iBuilder);
    Value *newM0Start = ret.first;
    Value *newM0End = ret.second;

    auto matchOffsetRet = this->loadNextMatchOffset(iBuilder);
    Value *newMatchOffset = matchOffsetRet.first;
    Value* newMatchOffsetPos = matchOffsetRet.second;

    Value * const newMatchLength = iBuilder->CreateAdd(iBuilder->CreateSub(newM0End, newM0Start), iBuilder->getInt64(1));

    iBuilder->CreateBr(matchCopyConBlock);
    iBuilder->SetInsertPoint(matchCopyConBlock);

    Value * const hasNotReachEnd = iBuilder->CreateICmpULT(newM0Start, toProcessItemCount);
    iBuilder->CreateLikelyCondBr(hasNotReachEnd, matchCopyBodyBlock, processExitBlock);

    iBuilder->SetInsertPoint(matchCopyBodyBlock);

    iBuilder->setScalarField("currentOffsetMarkerPos", newMatchOffsetPos);
    iBuilder->setProcessedItemCount("MatchOffsetMarker", newMatchOffsetPos);
    iBuilder->setScalarField("currentM0MarkerPos", newM0End);
    iBuilder->setProcessedItemCount("M0Marker", newM0End);


    BasicBlock* copyLoopCon = iBuilder->CreateBasicBlock("copyLoopCon");
    BasicBlock* copyLoopBody = iBuilder->CreateBasicBlock("copyLoopBody");
    iBuilder->CreateBr(copyLoopCon);
    iBuilder->SetInsertPoint(copyLoopCon);
    PHINode* phiMatchLength = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
    PHINode* phiMatchPos = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);

    phiMatchLength->addIncoming(newMatchLength, matchCopyBodyBlock);
    phiMatchPos->addIncoming(newM0Start, matchCopyBodyBlock);

    phiProcessIndex->addIncoming(iBuilder->CreateAdd(phiProcessIndex, iBuilder->getSize(1)), iBuilder->GetInsertBlock());


    iBuilder->CreateLikelyCondBr(iBuilder->CreateICmpNE(phiMatchLength, iBuilder->getSize(0)), copyLoopBody, matchCopyLoopCon);

    iBuilder->SetInsertPoint(copyLoopBody);
    Value* copySize = this->doMatchCopy(iBuilder, phiMatchPos, newMatchOffset, phiMatchLength);
    phiMatchLength->addIncoming(iBuilder->CreateSub(phiMatchLength, copySize), iBuilder->GetInsertBlock());
    phiMatchPos->addIncoming(iBuilder->CreateAdd(phiMatchPos, copySize), iBuilder->GetInsertBlock());
    iBuilder->CreateBr(copyLoopCon);

    iBuilder->SetInsertPoint(processExitBlock);
    iBuilder->setProcessedItemCount("M0CountMarker", phiProcessIndex);
    iBuilder->setProcessedItemCount("M0Marker", toProcessItemCount);
    iBuilder->setProcessedItemCount("sourceStreamSet0", toProcessItemCount);
    iBuilder->setScalarField("currentM0MarkerPos", toProcessItemCount);

}

llvm::Value* LZ4SwizzledMatchCopyKernel::doMatchCopy(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value* phiMatchPos, llvm::Value* phiMatchOffset, llvm::Value* phiMatchLength) {
    ConstantInt * const SIZE_ZERO = iBuilder->getSize(0);
    ConstantInt * const SIZE_ONE = iBuilder->getSize(1);
    ConstantInt * const SIZE_PDEP_WIDTH = iBuilder->getSize(mPDEPWidth);
    ConstantInt * const SIZE_BLOCK_WIDTH = iBuilder->getSize(iBuilder->getBitBlockWidth());

    ConstantInt * const outputBufferBlocks = iBuilder->getSize(this->getAnyStreamSetBuffer("outputStreamSet0")->getBufferBlocks());

    Value* matchPosLocalBlockIndex = iBuilder->CreateURem(iBuilder->CreateUDiv(phiMatchPos, SIZE_BLOCK_WIDTH), outputBufferBlocks);
    Value * const matchCopyTargetStreamIndex = iBuilder->CreateURem(iBuilder->CreateUDiv(phiMatchPos, SIZE_PDEP_WIDTH), iBuilder->getSize(mStreamCount));
    Value * const matchCopyTargetBlockOffset = iBuilder->CreateURem(phiMatchPos, SIZE_PDEP_WIDTH);

    Value * const matchCopyFromPos = iBuilder->CreateSub(phiMatchPos, phiMatchOffset);

    Value* matchCopyFromLocalBlockIndex = iBuilder->CreateURem(iBuilder->CreateUDiv(matchCopyFromPos, SIZE_BLOCK_WIDTH), outputBufferBlocks);
    Value * const matchCopyFromStreamIndex = iBuilder->CreateURem(iBuilder->CreateUDiv(matchCopyFromPos, SIZE_PDEP_WIDTH), iBuilder->getSize(mStreamCount));
    Value * const matchCopyFromBlockOffset = iBuilder->CreateURem(matchCopyFromPos, SIZE_PDEP_WIDTH);

    Value * currentCopySize = iBuilder->CreateSub(SIZE_PDEP_WIDTH, iBuilder->CreateUMax(matchCopyFromBlockOffset, matchCopyTargetBlockOffset));
    currentCopySize = iBuilder->CreateUMin(currentCopySize, phiMatchOffset);
    currentCopySize = iBuilder->CreateUMin(currentCopySize, phiMatchLength);
    currentCopySize = iBuilder->CreateSelect(iBuilder->CreateICmpEQ(currentCopySize, SIZE_ZERO), SIZE_ONE, currentCopySize); //Workaround for the last byte

    Value * const shiftOffset = iBuilder->CreateAdd(matchCopyFromBlockOffset, currentCopySize);
    Value * highOffset = iBuilder->CreateShl(SIZE_ONE, shiftOffset);
    highOffset = iBuilder->CreateSelect(iBuilder->CreateICmpEQ(currentCopySize, SIZE_PDEP_WIDTH), SIZE_ZERO, highOffset); // When currentCopySize == SIZE_PDEP_WIDTH, shl will overflow
    Value * const lowOffset = iBuilder->CreateShl(SIZE_ONE, matchCopyFromBlockOffset);
    Value * const maskVector = iBuilder->simd_fill(mPDEPWidth, iBuilder->CreateSub(highOffset, lowOffset));
    Value * const fromBlockOffsetVector = iBuilder->simd_fill(mPDEPWidth, matchCopyFromBlockOffset);
    Value * const targetBlockOffsetVector = iBuilder->simd_fill(mPDEPWidth, matchCopyTargetBlockOffset);

    for (unsigned i = 0; i < mStreamSize; i++) {
        Value* basePtr = iBuilder->CreatePointerCast(iBuilder->getRawOutputPointer("outputStreamSet" + std::to_string(i), SIZE_ZERO), iBuilder->getBitBlockType()->getPointerTo());

        Value * const matchCopyFromBlockPtr = iBuilder->CreateGEP(basePtr, iBuilder->CreateAdd(iBuilder->CreateMul(matchCopyFromLocalBlockIndex, iBuilder->getSize(mStreamCount)), matchCopyFromStreamIndex));
        Value * const fromBlockValue = iBuilder->CreateBlockAlignedLoad(matchCopyFromBlockPtr);

        Value * const outputTargetBlockPtr = iBuilder->CreateGEP(basePtr, iBuilder->CreateAdd(iBuilder->CreateMul(matchPosLocalBlockIndex, iBuilder->getSize(mStreamCount)), matchCopyTargetStreamIndex));
        Value * const targetOriginalValue = iBuilder->CreateBlockAlignedLoad(outputTargetBlockPtr);

        Value * copiedValue = iBuilder->simd_and(fromBlockValue, maskVector);
        copiedValue = iBuilder->CreateLShr(copiedValue, fromBlockOffsetVector);
        copiedValue = iBuilder->CreateShl(copiedValue, targetBlockOffsetVector);
        Value * const finalValue = iBuilder->CreateOr(targetOriginalValue, copiedValue);

        iBuilder->CreateStore(finalValue, outputTargetBlockPtr);
    }
    return currentCopySize;
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


LZ4SwizzledMatchCopyKernel::LZ4SwizzledMatchCopyKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder, unsigned streamCount/*=4*/, unsigned streamSize/*=2*/, unsigned swizzleFactor/*=4*/, unsigned PDEP_width/*64*/)
: SegmentOrientedKernel("LZ4SwizzledMatchCopyKernel",
// Inputs
{
                                   Binding{iBuilder->getStreamSetTy(1, 1), "MatchOffsetMarker", BoundedRate(0, 1), {DisableSufficientChecking()}},
                                   Binding{iBuilder->getStreamSetTy(1, 1), "M0Marker", BoundedRate(0, 1), {DisableSufficientChecking()}},
                                   Binding{iBuilder->getStreamSetTy(1, 1), "M0CountMarker", BoundedRate(0, 1), {DisableSufficientChecking()}},
                                   Binding{iBuilder->getStreamSetTy(1, 8), "byteStream", BoundedRate(0, 1)}
},
// Outputs
{},
// Arguments
{
},
{},
{
       Binding{iBuilder->getSizeTy(), "currentProcessIndex"},
       Binding{iBuilder->getSizeTy(), "pendingMatchPos"},
       Binding{iBuilder->getSizeTy(), "pendingMatchOffset"},
       Binding{iBuilder->getSizeTy(), "pendingMatchLength"},
       Binding(iBuilder->getSizeTy(), "currentOffsetMarkerPos"),
       Binding(iBuilder->getSizeTy(), "currentM0MarkerPos")
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
