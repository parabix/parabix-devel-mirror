//
// Created by wxy325 on 2018/3/9.
//

#include "lz4_swizzled_match_copy_kernel.h"
#include <kernels/kernel_builder.h>
#include <kernels/streamset.h>
#include <toolchain/toolchain.h>


using namespace llvm;
using namespace kernel;
using namespace std;

Value* LZ4SwizzledMatchCopyKernel::loadInt64NumberInput(const unique_ptr<KernelBuilder> &iBuilder, string bufferName, Value* offset) {
    // GEP here is safe
    Constant* SIZE_ZERO = iBuilder->getSize(0);
    Type* int64PtrType = iBuilder->getInt64Ty()->getPointerTo();

    Value* tmpOffset = iBuilder->CreateURem(offset, iBuilder->getSize(this->getAnyStreamSetBuffer(bufferName)->getBufferBlocks() * iBuilder->getBitBlockWidth()));
    Value* outputRawPtr = iBuilder->CreatePointerCast(iBuilder->getRawInputPointer(bufferName, SIZE_ZERO), int64PtrType);
    Value* ptr2 = iBuilder->CreateGEP(outputRawPtr, tmpOffset);

    return iBuilder->CreateLoad(ptr2);
}

void LZ4SwizzledMatchCopyKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
//void LZ4SwizzledMatchCopyKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value * const numOfStrides)  {
    // Const
    Constant *SIZE_ZERO = iBuilder->getSize(0);
    Constant *SIZE_ONE = iBuilder->getSize(1);
    Constant *SIZE_BIT_BLOCK_WIDTH = iBuilder->getSize(iBuilder->getBitBlockWidth());
    Constant *SIZE_PDEP_WIDTH = iBuilder->getSize(mPDEPWidth);

    BasicBlock* exitBlock = iBuilder->CreateBasicBlock("exitBlock");

    Value *totalItemCount = iBuilder->getAvailableItemCount("sourceStreamSet0");
    Value *itemsToDo = iBuilder->CreateSub(totalItemCount, iBuilder->getProcessedItemCount("sourceStreamSet0"));

    Value *isFinalBlock = iBuilder->CreateICmpULT(itemsToDo, iBuilder->getSize(4 * 1024 * 1024));
    this->mIsFinalBlock = isFinalBlock;
    iBuilder->setTerminationSignal(isFinalBlock);

    Value *previousProducedItemCount = iBuilder->getProducedItemCount("outputStreamSet0");

    // Space Calculation
    Value *outputBufferBlocks = iBuilder->getSize(
            this->getAnyStreamSetBuffer("outputStreamSet0")->getBufferBlocks());

    Value *outputBlocks = iBuilder->getSize(4 * 1024 * 1024 / iBuilder->getBitBlockWidth()); // Always be 4MB


    BasicBlock* processBlock = iBuilder->CreateBasicBlock("processBlock");
    Value* isInputEnough = iBuilder->CreateOr(isFinalBlock, iBuilder->CreateICmpUGE(itemsToDo, iBuilder->getSize(4 * 1024 * 1024)));

    iBuilder->CreateCondBr(isInputEnough, processBlock, exitBlock);

    iBuilder->SetInsertPoint(processBlock);
    // Output Copy
    this->generateOutputCopy(iBuilder, outputBlocks);

    Value *newProducedItemCount = iBuilder->getProducedItemCount("outputStreamSet0");

    BasicBlock *copyEndBlock = iBuilder->CreateBasicBlock("copyEnd");
    iBuilder->CreateBr(copyEndBlock);
    iBuilder->SetInsertPoint(copyEndBlock);

    // Match Copy
    BasicBlock *processExitBlock = iBuilder->CreateBasicBlock("exit_block");

    Value *initM0StartProcessIndex = iBuilder->getProcessedItemCount("m0Start");
    Value *totalM0StartItemsCount = iBuilder->getAvailableItemCount("m0Start");

    Value *initMatchOffset = iBuilder->getScalarField("pendingMatchOffset");
    Value *initMatchLength = iBuilder->getScalarField("pendingMatchLength");
    Value *initMatchPos = iBuilder->getScalarField("pendingMatchPos");

    BasicBlock *matchCopyLoopCon = iBuilder->CreateBasicBlock("matchCopyLoopCon");
    iBuilder->CreateBr(matchCopyLoopCon);

    iBuilder->SetInsertPoint(matchCopyLoopCon);


    PHINode *phiProcessIndex = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3);
    phiProcessIndex->addIncoming(initM0StartProcessIndex, copyEndBlock);

    PHINode *phiMatchOffset = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3);
    phiMatchOffset->addIncoming(initMatchOffset, copyEndBlock);

    PHINode *phiMatchLength = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3);
    phiMatchLength->addIncoming(initMatchLength, copyEndBlock);

    PHINode *phiMatchPos = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3);
    phiMatchPos->addIncoming(initMatchPos, copyEndBlock);

    BasicBlock *loadNextMatchInfoConBlock = iBuilder->CreateBasicBlock("loadNewMatchInfoConBlock");
    BasicBlock *loadNextMatchInfoBodyBlock = iBuilder->CreateBasicBlock("loadNewMatchInfoBodyBlock");

    BasicBlock *matchCopyConBlock = iBuilder->CreateBasicBlock("matchCopyConBlock");
    BasicBlock *matchCopyBodyBlock = iBuilder->CreateBasicBlock("matchCopyBodyBlock");


    iBuilder->CreateCondBr(
            iBuilder->CreateICmpEQ(phiMatchLength, iBuilder->getSize(0)),
            loadNextMatchInfoConBlock,
            matchCopyConBlock
    );


    iBuilder->SetInsertPoint(loadNextMatchInfoConBlock);



    Value *hasMoreMatchInfo = iBuilder->CreateICmpULT(phiProcessIndex, totalM0StartItemsCount);
    iBuilder->CreateCondBr(hasMoreMatchInfo, loadNextMatchInfoBodyBlock, processExitBlock);

    iBuilder->SetInsertPoint(loadNextMatchInfoBodyBlock);

    Value *newM0Start = this->loadInt64NumberInput(iBuilder, "m0Start", phiProcessIndex);
    Value *newM0End = this->loadInt64NumberInput(iBuilder, "m0End", phiProcessIndex);
    Value *newMatchOffset = this->loadInt64NumberInput(iBuilder, "matchOffset", phiProcessIndex);

    Value *depositStart = newM0Start;

    Value *depositEnd = iBuilder->CreateAdd(newM0End, iBuilder->getInt64(1));
    Value *newMatchLength = iBuilder->CreateSub(depositEnd, depositStart);
    phiProcessIndex->addIncoming(iBuilder->CreateAdd(phiProcessIndex, SIZE_ONE), iBuilder->GetInsertBlock());

    phiMatchPos->addIncoming(depositStart, iBuilder->GetInsertBlock());
    phiMatchOffset->addIncoming(newMatchOffset, iBuilder->GetInsertBlock());
    phiMatchLength->addIncoming(newMatchLength, iBuilder->GetInsertBlock());

    iBuilder->CreateBr(matchCopyLoopCon);


    iBuilder->SetInsertPoint(matchCopyConBlock);
    Value *hasNotReachEnd = iBuilder->CreateICmpULT(phiMatchPos, newProducedItemCount);
//    iBuilder->CallPrintInt("newProducedItemCount", newProducedItemCount);
    iBuilder->CreateCondBr(hasNotReachEnd, matchCopyBodyBlock, processExitBlock);

    iBuilder->SetInsertPoint(matchCopyBodyBlock);


    Value* matchCopyFromPos = iBuilder->CreateSub(phiMatchPos, phiMatchOffset);
    Value* outputBufferSize = iBuilder->CreateMul(outputBufferBlocks, SIZE_BIT_BLOCK_WIDTH);
    Value* matchCopyFromOffset = iBuilder->CreateURem(matchCopyFromPos, outputBufferSize);
    Value* matchCopyFromBlockIndex = iBuilder->CreateUDiv(matchCopyFromOffset, SIZE_PDEP_WIDTH);
    Value* matchCopyFromBlockOffset = iBuilder->CreateURem(matchCopyFromOffset, SIZE_PDEP_WIDTH);


    Value* matchCopyTargetOffset = iBuilder->CreateURem(phiMatchPos, outputBufferSize);
    Value* matchCopyTargetBlockIndex = iBuilder->CreateUDiv(matchCopyTargetOffset, SIZE_PDEP_WIDTH);
    Value* matchCopyTargetBlockOffset = iBuilder->CreateURem(matchCopyTargetOffset, SIZE_PDEP_WIDTH);


    Value* matchCopyFromRemain = iBuilder->CreateSub(SIZE_PDEP_WIDTH, matchCopyFromBlockOffset);
    Value* matchCopyTargetRemain = iBuilder->CreateSub(SIZE_PDEP_WIDTH, matchCopyTargetBlockOffset);

    Value* currentCopySize = iBuilder->CreateUMin(matchCopyFromRemain, matchCopyTargetRemain);
    currentCopySize = iBuilder->CreateUMin(currentCopySize, phiMatchOffset);
    currentCopySize = iBuilder->CreateUMin(currentCopySize, phiMatchLength);
    currentCopySize = iBuilder->CreateUMin(currentCopySize, iBuilder->CreateSub(newProducedItemCount, phiMatchPos));
    currentCopySize = iBuilder->CreateSelect(iBuilder->CreateICmpEQ(currentCopySize, SIZE_ZERO), SIZE_ONE, currentCopySize); //Workaround for the last byte
    Value* singleMask = iBuilder->CreateSub(
            iBuilder->CreateSelect( // When currentCopySize == SIZE_PDEP_WIDTH, shl will cause overflow
                    iBuilder->CreateICmpEQ(currentCopySize, SIZE_PDEP_WIDTH),
                    SIZE_ZERO,
                    iBuilder->CreateShl(SIZE_ONE, iBuilder->CreateAdd(matchCopyFromBlockOffset, currentCopySize))
            ),
            iBuilder->CreateShl(SIZE_ONE, matchCopyFromBlockOffset)
    );
    Value* fullMask = iBuilder->simd_fill(mPDEPWidth, singleMask);

    for (int i = 0; i < mStreamSize; i++) {
        Value* rawOutputBasePtr = iBuilder->getRawOutputPointer("outputStreamSet" + std::to_string(i), SIZE_ZERO);
        rawOutputBasePtr = iBuilder->CreatePointerCast(rawOutputBasePtr, iBuilder->getBitBlockType()->getPointerTo());
        Value* matchCopyFromBlockPtr = iBuilder->CreateGEP(rawOutputBasePtr, matchCopyFromBlockIndex);

        Value* fromBlockValue = iBuilder->CreateLoad(matchCopyFromBlockPtr);

        Value* copiedValue = iBuilder->simd_and(fromBlockValue, fullMask);

        Value* outputTargetBlockPtr = iBuilder->CreateGEP(rawOutputBasePtr, matchCopyTargetBlockIndex);

//        iBuilder->CallPrintInt("outputTargetBlockPtr", outputTargetBlockPtr);
        Value* targetOriginalValue = iBuilder->CreateLoad(outputTargetBlockPtr);

        Value* finalValue = iBuilder->simd_or(
                targetOriginalValue,
                iBuilder->CreateShl(
                        iBuilder->CreateLShr(
                                copiedValue,
                                iBuilder->simd_fill(mPDEPWidth, matchCopyFromBlockOffset)
                        ),
                        iBuilder->simd_fill(mPDEPWidth, matchCopyTargetBlockOffset)
                )
        );


//        iBuilder->CallPrintRegister("targetOriginalValue", targetOriginalValue);
//        iBuilder->CallPrintRegister("finalValue", finalValue);
//        iBuilder->CallPrintInt("matchCopyTargetBlockOffset", matchCopyTargetBlockOffset);
//        iBuilder->CallPrintInt("currentCopySize", currentCopySize);
        iBuilder->CreateStore(finalValue, outputTargetBlockPtr);
    }

    phiProcessIndex->addIncoming(phiProcessIndex, iBuilder->GetInsertBlock());
    phiMatchOffset->addIncoming(phiMatchOffset, iBuilder->GetInsertBlock());
    phiMatchPos->addIncoming(iBuilder->CreateAdd(phiMatchPos, currentCopySize), iBuilder->GetInsertBlock());
    phiMatchLength->addIncoming(iBuilder->CreateSub(phiMatchLength, currentCopySize), iBuilder->GetInsertBlock());

    iBuilder->CreateBr(matchCopyLoopCon);

    iBuilder->SetInsertPoint(processExitBlock);
    iBuilder->setScalarField("pendingMatchOffset", phiMatchOffset);
    iBuilder->setScalarField("pendingMatchLength", phiMatchLength);
    iBuilder->setScalarField("pendingMatchPos", phiMatchPos);
    iBuilder->setProcessedItemCount("m0Start", phiProcessIndex);
    iBuilder->setProcessedItemCount("m0End", phiProcessIndex);
    iBuilder->setProcessedItemCount("matchOffset", phiProcessIndex);

    iBuilder->CreateBr(exitBlock);
    iBuilder->SetInsertPoint(exitBlock);
//    iBuilder->CallPrintInt("totalM0StartItemsCount", totalM0StartItemsCount);
}

void LZ4SwizzledMatchCopyKernel::generateOutputCopy(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value* outputBlocks) {
    Value *SIZE_ZERO = iBuilder->getSize(0);
    Value *SIZE_BIT_BLOCK_WIDTH = iBuilder->getSize(iBuilder->getBitBlockWidth());
    Constant *INT64_BIT_BLOCK_WIDTH = iBuilder->getInt64(iBuilder->getBitBlockWidth());
    Type* bytePtrType = iBuilder->getInt8PtrTy();

    Value *previousProcessed = iBuilder->getProcessedItemCount("sourceStreamSet0");

    Value *itemsToDo = iBuilder->CreateSub(iBuilder->getAvailableItemCount("sourceStreamSet0"), iBuilder->getProcessedItemCount("sourceStreamSet0"));
    Value *copySize = iBuilder->CreateMul(outputBlocks, SIZE_BIT_BLOCK_WIDTH);
    Value* actualCopySize = iBuilder->CreateUMin(itemsToDo, copySize);
    Value* copyByte = iBuilder->CreateUDivCeil(iBuilder->CreateMul(copySize, iBuilder->getSize(mStreamCount)), iBuilder->getSize(8)); // i8

    Value* outputBufferSize = iBuilder->getSize(this->getAnyStreamSetBuffer("sourceStreamSet0")->getBufferBlocks() * iBuilder->getBitBlockWidth());
    Value* inputOffset = iBuilder->CreateMul(
            iBuilder->CreateAnd(iBuilder->CreateURem(previousProcessed, outputBufferSize), ConstantExpr::getNeg(INT64_BIT_BLOCK_WIDTH)), iBuilder->getInt64(mStreamCount)
    );

    for (int i = 0; i < mStreamSize; i++) {

        Value * inputBasePtr = iBuilder->CreatePointerCast(iBuilder->getRawInputPointer("sourceStreamSet" + std::to_string(i), inputOffset), iBuilder->getBitBlockType()->getPointerTo());
        Value * outputBasePtr = iBuilder->CreatePointerCast(iBuilder->getRawOutputPointer("outputStreamSet" + std::to_string(i), inputOffset), iBuilder->getBitBlockType()->getPointerTo());

        iBuilder->CreateMemCpy(
                iBuilder->CreatePointerCast(outputBasePtr, bytePtrType),
                iBuilder->CreatePointerCast(inputBasePtr, bytePtrType),
                copyByte,
                1 // Not align guaranteed in final block
        );
    }
    Value *newProcessed = iBuilder->CreateAdd(previousProcessed, actualCopySize);
    iBuilder->setProcessedItemCount("sourceStreamSet0", newProcessed);
//    iBuilder->CallPrintInt("swizzledMatchCopy:newProcessed", newProcessed);
    iBuilder->setProducedItemCount("outputStreamSet0", newProcessed);
}

LZ4SwizzledMatchCopyKernel::LZ4SwizzledMatchCopyKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder, unsigned streamCount/*=4*/, unsigned streamSize/*=2*/, unsigned swizzleFactor/*=4*/, unsigned PDEP_width/*64*/)
        : SegmentOrientedKernel("LZ4SwizzledMatchCopyKernel",
        // Inputs
                           {
                                   Binding{iBuilder->getStreamSetTy(1, 64), "m0Start", BoundedRate(0, 1), {DisableTemporaryBuffer(), DisableAvailableItemCountAdjustment(), DisableSufficientChecking()}},
                                   Binding{iBuilder->getStreamSetTy(1, 64), "m0End", BoundedRate(0, 1), {DisableTemporaryBuffer(), DisableAvailableItemCountAdjustment(), DisableSufficientChecking()}},
                                   Binding{iBuilder->getStreamSetTy(1, 64), "matchOffset", BoundedRate(0, 1), {DisableTemporaryBuffer(), DisableAvailableItemCountAdjustment(), DisableSufficientChecking()}},

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
    this->setStride(4 * 1024 * 1024);
    addAttribute(MustExplicitlyTerminate());

    mStreamSetInputs.push_back(Binding{iBuilder->getStreamSetTy(streamCount), "sourceStreamSet0", BoundedRate(0, 1), {Swizzled(), DisableTemporaryBuffer()}});
    mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(streamCount), "outputStreamSet0", BoundedRate(0, 1), DisableTemporaryBuffer()});

    for (int i = 1; i < streamSize; i++) {
        mStreamSetInputs.push_back(Binding{iBuilder->getStreamSetTy(streamCount), "sourceStreamSet" + std::to_string(i), RateEqualTo("sourceStreamSet0"), {Swizzled(), DisableTemporaryBuffer()}});
        mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(streamCount), "outputStreamSet" + std::to_string(i), RateEqualTo("outputStreamSet0"), DisableTemporaryBuffer()});
    }
}
