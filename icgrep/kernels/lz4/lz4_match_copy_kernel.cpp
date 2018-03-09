//
//

#include "lz4_match_copy_kernel.h"
#include <kernels/kernel_builder.h>
#include <kernels/streamset.h>
#include <toolchain/toolchain.h>

#define OUTPUT_BIT_STREAM_NAME "outputStream"

using namespace llvm;
using namespace kernel;
using namespace std;

void LZ4MatchCopyKernel::generateOutputCopy(const std::unique_ptr<KernelBuilder> &iBuilder, Value *outputBlocks) {

    Value *SIZE_ZERO = iBuilder->getSize(0);
    Value *SIZE_BIT_BLOCK_WIDTH = iBuilder->getSize(iBuilder->getBitBlockWidth());

    Value *previousProcessed = iBuilder->getProcessedItemCount("decompressedStream");

//    BasicBlock *entryBlock = iBuilder->GetInsertBlock();
    Value *inputBasePtr = iBuilder->getInputStreamBlockPtr("decompressedStream", SIZE_ZERO);

    Value *outputBasePtr = iBuilder->getOutputStreamBlockPtr(OUTPUT_BIT_STREAM_NAME, SIZE_ZERO);
    Value *itemsToDo = mAvailableItemCount[0];
    Value *copySize = iBuilder->CreateUMin(
            itemsToDo,
            iBuilder->CreateMul(outputBlocks, SIZE_BIT_BLOCK_WIDTH)
    );
//    iBuilder->CallPrintInt("itemsToDo", itemsToDo);
//    iBuilder->CallPrintInt("itemsToDo1", mAvailableItemCount[1]);
//    iBuilder->CallPrintInt("itemsToDo2", mAvailableItemCount[2]);
//    iBuilder->CallPrintInt("itemsToDo3", mAvailableItemCount[3]);
//    iBuilder->CallPrintInt("copySize", copySize);

    iBuilder->CreateMemCpy(
            outputBasePtr,
            inputBasePtr,
            copySize,
            1 // Not align guaranteed in final block
    );
//    iBuilder->CallPrintInt("outputCpyPtr", outputBasePtr);
//    iBuilder->CallPrintInt("outputBlocks", outputBlocks);
    Value *newProcessed = iBuilder->CreateAdd(previousProcessed, copySize);
    iBuilder->setProcessedItemCount("decompressedStream", newProcessed);
    iBuilder->setProducedItemCount(OUTPUT_BIT_STREAM_NAME, newProcessed);

}

Value *LZ4MatchCopyKernel::getMaximumMatchCopyBlock(const unique_ptr<KernelBuilder> &iBuilder) {
    Value *SIZE_BIT_BLOCK_WIDTH = iBuilder->getSize(iBuilder->getBitBlockWidth());
    Value *SIZE_ZERO = iBuilder->getSize(0);
    Value *SIZE_ONE = iBuilder->getSize(1);
    Value *m0EndInitOffset = iBuilder->CreateURem(iBuilder->getProcessedItemCount("m0End"), SIZE_BIT_BLOCK_WIDTH);
    Value *m0EndItemsToDo = mAvailableItemCount[2];
    Value *m0EndBasePtr = iBuilder->getInputStreamBlockPtr("m0End", SIZE_ZERO);
    m0EndBasePtr = iBuilder->CreatePointerCast(m0EndBasePtr, iBuilder->getInt64Ty()->getPointerTo());
    Value *lastM0 = iBuilder->CreateLoad(
            iBuilder->CreateGEP(
                    m0EndBasePtr,
                    iBuilder->CreateSub(
                            iBuilder->CreateAdd(m0EndInitOffset, m0EndItemsToDo),
                            SIZE_ONE
                    )

            )
    );
    Value *lastDepositPosition = iBuilder->CreateAdd(lastM0, SIZE_ONE);

    // TODO maybe we can not use mIsFinal here
    Value *currentMaxBlock = iBuilder->CreateSelect(
            this->mIsFinalBlock,
            iBuilder->CreateUDivCeil(lastDepositPosition, SIZE_BIT_BLOCK_WIDTH),
            iBuilder->CreateUDiv(lastDepositPosition, SIZE_BIT_BLOCK_WIDTH)
    );

    // Produced Item Count will always be full bitblock except for final block
    Value *previousProducedBlocks = iBuilder->CreateUDiv(
            iBuilder->getProducedItemCount(OUTPUT_BIT_STREAM_NAME),
            SIZE_BIT_BLOCK_WIDTH
    );

    // (m0 + 1) / BitBlockWidth - produceItemCount / BitBlockWidth
    return iBuilder->CreateSub(currentMaxBlock, previousProducedBlocks);
}

void LZ4MatchCopyKernel::generateMultiBlockLogic(const unique_ptr<KernelBuilder> &iBuilder, Value *const numOfStrides) {
    // Const
    Constant *SIZE_ZERO = iBuilder->getSize(0);
    Constant *SIZE_ONE = iBuilder->getSize(1);
    Constant *SIZE_BIT_BLOCK_WIDTH = iBuilder->getSize(iBuilder->getBitBlockWidth());

    Value *itemsToDo = mAvailableItemCount[0];

    Value *isFinalBlock =
            iBuilder->CreateOr(
                    iBuilder->CreateICmpULT(itemsToDo, iBuilder->CreateMul(numOfStrides, SIZE_BIT_BLOCK_WIDTH)),
                    iBuilder->CreateICmpEQ(itemsToDo, iBuilder->getSize(0))
            );

    this->mIsFinalBlock = isFinalBlock;
//    iBuilder->CallPrintInt("isFinalBlock", isFinalBlock);
    iBuilder->setTerminationSignal(isFinalBlock);




    Value *previousProducedItemCount = iBuilder->getProducedItemCount(OUTPUT_BIT_STREAM_NAME);


    // Space Calculation
    Value *outputBufferBlocks = iBuilder->getSize(
            this->getAnyStreamSetBuffer(OUTPUT_BIT_STREAM_NAME)->getBufferBlocks());
    // TODO need to take previous produced size into account


    Value *outputRawBeginPtr = iBuilder->CreatePointerCast(
            iBuilder->getRawOutputPointer(OUTPUT_BIT_STREAM_NAME, SIZE_ZERO),
            iBuilder->getBitBlockType()->getPointerTo());
    Value *outputCurrentPtr = iBuilder->getOutputStreamBlockPtr(OUTPUT_BIT_STREAM_NAME, SIZE_ZERO);
    Value *producedOffset = iBuilder->CreatePtrDiff(outputCurrentPtr, outputRawBeginPtr);
    Value *remainSpace = iBuilder->CreateSub(outputBufferBlocks, producedOffset);
    Value *matchCopyWindowBlock = iBuilder->getSize(256 * 256 / codegen::BlockSize);
    Value *remainWindowBlock = iBuilder->CreateSelect(
            iBuilder->CreateICmpUGE(producedOffset, matchCopyWindowBlock),
            iBuilder->getSize(0),
            iBuilder->CreateSub(matchCopyWindowBlock, producedOffset)
    );
    Value *writableBlocks = iBuilder->CreateSub(remainSpace,
                                                remainWindowBlock); //TODO handle beginning, if producedItemCount / bitblockWidth < windowBlock, there is no need for the substraction here
//    iBuilder->CallPrintInt("remainSpace", remainSpace);
//    iBuilder->CallPrintInt("writableBlocks", writableBlocks);
    Value *outputBlocks = iBuilder->CreateUMin(writableBlocks, numOfStrides);
    // outputBlock === min(writableBlocks, numOfStrides, (matchOffsetPosition + matchLength - producedItemCount) / bitBlockWidth )

    outputBlocks = iBuilder->CreateUMin(outputBlocks, this->getMaximumMatchCopyBlock(iBuilder));


//    BasicBlock * entryBlock = iBuilder->GetInsertBlock();

    // Output Copy
    this->generateOutputCopy(iBuilder, outputBlocks);
//    return;

    Value *newProducedItemCount = iBuilder->getProducedItemCount(OUTPUT_BIT_STREAM_NAME);

    BasicBlock *copyEndBlock = iBuilder->CreateBasicBlock("copyEnd");
    iBuilder->CreateBr(copyEndBlock);
    iBuilder->SetInsertPoint(copyEndBlock);

    // TODO match Copy
    BasicBlock *exitBlock = iBuilder->CreateBasicBlock("exit_block");

    Value *initM0StartProcessIndex = iBuilder->getProcessedItemCount("m0Start");
    Value *totalM0StartItemsCount = iBuilder->CreateAdd(initM0StartProcessIndex, mAvailableItemCount[1]);

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
    iBuilder->CreateCondBr(hasMoreMatchInfo, loadNextMatchInfoBodyBlock, exitBlock);

    iBuilder->SetInsertPoint(loadNextMatchInfoBodyBlock);

    Value *m0StartBasePtr = iBuilder->CreatePointerCast(iBuilder->getInputStreamBlockPtr("m0Start", SIZE_ZERO), iBuilder->getInt64Ty()->getPointerTo());
    Value *m0EndBasePtr = iBuilder->CreatePointerCast(iBuilder->getInputStreamBlockPtr("m0End", SIZE_ZERO), iBuilder->getInt64Ty()->getPointerTo());
    Value *matchOffsetBasePtr = iBuilder->CreatePointerCast(iBuilder->getInputStreamBlockPtr("matchOffset", SIZE_ZERO), iBuilder->getInt64Ty()->getPointerTo());


    Value *m0StartBaseOffset = iBuilder->CreateURem(initM0StartProcessIndex, SIZE_BIT_BLOCK_WIDTH);
//    iBuilder->CallPrintInt("rawPtr", iBuilder->getRawInputPointer("m0Start", SIZE_ZERO));
//    iBuilder->CallPrintInt("ptr", m0StartBasePtr);
//    iBuilder->CallPrintInt("initM0StartProcessIndex", initM0StartProcessIndex);
    Value *m0StartLoadOffset = iBuilder->CreateAdd(m0StartBaseOffset,
                                                   iBuilder->CreateSub(phiProcessIndex, initM0StartProcessIndex));

    Value *newM0Start = iBuilder->CreateLoad(iBuilder->CreateGEP(m0StartBasePtr, m0StartLoadOffset));
    Value *newM0End = iBuilder->CreateLoad(iBuilder->CreateGEP(m0EndBasePtr, m0StartLoadOffset));
    Value *newMatchOffset = iBuilder->CreateLoad(iBuilder->CreateGEP(matchOffsetBasePtr, m0StartLoadOffset));

    Value *depositStart = newM0Start;
//    iBuilder->CallPrintInt("depositStart", depositStart);
//    iBuilder->CallPrintInt("newMatchLength", newMatchLength);

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
    iBuilder->CreateCondBr(hasNotReachEnd, matchCopyBodyBlock, exitBlock);

    iBuilder->SetInsertPoint(matchCopyBodyBlock);
    Value* matchCopyFromPos = iBuilder->CreateSub(phiMatchPos, phiMatchOffset);
    Value* rawOutputBasePtr = iBuilder->CreatePointerCast(iBuilder->getRawOutputPointer(OUTPUT_BIT_STREAM_NAME, SIZE_ZERO), iBuilder->getInt8PtrTy());
//    iBuilder->CallPrintInt("rawOutputBasePtr", rawOutputBasePtr);
//    iBuilder->CallPrintInt("rawOutputBasePtr1", iBuilder->CreateGEP(
//            rawOutputBasePtr,
//            iBuilder->CreateURem(matchCopyFromPos, iBuilder->CreateMul(outputBufferBlocks, SIZE_BIT_BLOCK_WIDTH))
//    ));
    Value* matchCopyFromValue = iBuilder->CreateLoad(
            iBuilder->CreateGEP(
                    rawOutputBasePtr,
                    iBuilder->CreateURem(matchCopyFromPos, iBuilder->CreateMul(outputBufferBlocks, SIZE_BIT_BLOCK_WIDTH))
            ));

    // Output is guranteed to be full bit block except for final block
    Value* outputBlockBasePtr = iBuilder->CreatePointerCast(iBuilder->getOutputStreamBlockPtr(OUTPUT_BIT_STREAM_NAME, SIZE_ZERO), iBuilder->getInt8PtrTy());
    Value* outputTargetPtr = iBuilder->CreateGEP(outputBlockBasePtr, iBuilder->CreateSub(phiMatchPos, previousProducedItemCount));
//    iBuilder->CallPrintInt("matchCopyFromValue", matchCopyFromValue);
//    iBuilder->CallPrintInt("phiMatchPos", phiMatchPos);
//    iBuilder->CallPrintInt("aa", iBuilder->CreateSub(phiMatchPos, previousProducedItemCount));
    iBuilder->CreateStore(matchCopyFromValue, outputTargetPtr);

    phiProcessIndex->addIncoming(phiProcessIndex, iBuilder->GetInsertBlock());
    phiMatchOffset->addIncoming(phiMatchOffset, iBuilder->GetInsertBlock());
    phiMatchPos->addIncoming(iBuilder->CreateAdd(phiMatchPos, SIZE_ONE), iBuilder->GetInsertBlock());
    phiMatchLength->addIncoming(iBuilder->CreateSub(phiMatchLength, SIZE_ONE), iBuilder->GetInsertBlock());

    iBuilder->CreateBr(matchCopyLoopCon);

    iBuilder->SetInsertPoint(exitBlock);
//    iBuilder->CallPrintInt("test", SIZE_ZERO);
    iBuilder->setScalarField("pendingMatchOffset", phiMatchOffset);
    iBuilder->setScalarField("pendingMatchLength", phiMatchLength);
    iBuilder->setScalarField("pendingMatchPos", phiMatchPos);
//    iBuilder->CallPrintInt("pendingMatchLength", phiMatchLength);
    iBuilder->setProcessedItemCount("m0Start", phiProcessIndex);
    iBuilder->setProcessedItemCount("m0End", phiProcessIndex);
    iBuilder->setProcessedItemCount("matchOffset", phiProcessIndex);
}


void LZ4MatchCopyKernel::generateStoreCircularOutput(const unique_ptr<KernelBuilder> &iBuilder, string outputBufferName,
                                                     Value *offset, Type *pointerType, Value *value) {
    size_t inputSize = this->getOutputBufferSize(iBuilder, outputBufferName);
    Value *offsetMask = iBuilder->getSize(inputSize - 1);
    Value *maskedOffset = iBuilder->CreateAnd(offsetMask, offset);

    Value *outputBufferPtr = iBuilder->getRawOutputPointer(outputBufferName, iBuilder->getSize(0));

    outputBufferPtr = iBuilder->CreatePointerCast(outputBufferPtr, pointerType);
    iBuilder->CreateStore(value, iBuilder->CreateGEP(outputBufferPtr, maskedOffset));
}

Value *LZ4MatchCopyKernel::generateLoadCircularOutput(const unique_ptr<KernelBuilder> &iBuilder, string inputBufferName,
                                                      Value *offset, Type *pointerType) {
    size_t inputSize = this->getOutputBufferSize(iBuilder, inputBufferName);
    Value *offsetMask = iBuilder->getSize(inputSize - 1);
    Value *maskedOffset = iBuilder->CreateAnd(offsetMask, offset);

    Value *inputBufferPtr = iBuilder->getRawOutputPointer(inputBufferName, iBuilder->getSize(0));

    inputBufferPtr = iBuilder->CreatePointerCast(inputBufferPtr, pointerType);
    return iBuilder->CreateLoad(iBuilder->CreateGEP(inputBufferPtr, maskedOffset));
}

Value *LZ4MatchCopyKernel::generateLoadCircularInput(const unique_ptr<KernelBuilder> &iBuilder, string inputBufferName,
                                                     Value *offset, Type *pointerType) {
    size_t inputSize = this->getInputBufferSize(iBuilder, inputBufferName);
    Value *offsetMask = iBuilder->getSize(inputSize - 1);
    Value *maskedOffset = iBuilder->CreateAnd(offsetMask, offset);

    Value *inputBufferPtr = iBuilder->getRawInputPointer(inputBufferName, iBuilder->getSize(0));

    inputBufferPtr = iBuilder->CreatePointerCast(inputBufferPtr, pointerType);
    return iBuilder->CreateLoad(iBuilder->CreateGEP(inputBufferPtr, maskedOffset));
}

size_t LZ4MatchCopyKernel::getInputBufferSize(const unique_ptr<KernelBuilder> &iBuilder, string bufferName) {
    return this->getInputStreamSetBuffer(bufferName)->getBufferBlocks() * iBuilder->getStride();
}

size_t LZ4MatchCopyKernel::getOutputBufferSize(const unique_ptr<KernelBuilder> &iBuilder, string bufferName) {
    return this->getOutputStreamSetBuffer(bufferName)->getBufferBlocks() * iBuilder->getStride();
}

LZ4MatchCopyKernel::LZ4MatchCopyKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder)
        : MultiBlockKernel("lz4MatchCopyKernel",
        // Inputs
                           {
                                   Binding{iBuilder->getStreamSetTy(1, 8), "decompressedStream", BoundedRate(0, 1), AlwaysConsume()},
                                   Binding{iBuilder->getStreamSetTy(1, 64), "m0Start", BoundedRate(0, 1), AlwaysConsume()},
                                   Binding{iBuilder->getStreamSetTy(1, 64), "m0End", BoundedRate(0, 1), AlwaysConsume()},
                                   Binding{iBuilder->getStreamSetTy(1, 64), "matchOffset", BoundedRate(0, 1), AlwaysConsume()},

                           },
        // Outputs
                           {Binding{iBuilder->getStreamSetTy(1, 8), OUTPUT_BIT_STREAM_NAME, BoundedRate(0, 1)}},
        // Arguments
                           {},
                           {},
                           {
                                   Binding{iBuilder->getSizeTy(), "currentProcessIndex"},
                                   Binding{iBuilder->getSizeTy(), "pendingMatchPos"},
                                   Binding{iBuilder->getSizeTy(), "pendingMatchOffset"},
                                   Binding{iBuilder->getSizeTy(), "pendingMatchLength"},
                           }) {
//    setNoTerminateAttribute(true);
    addAttribute(MustExplicitlyTerminate());
}
