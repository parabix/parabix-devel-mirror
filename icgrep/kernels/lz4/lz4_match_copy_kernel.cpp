//
//

#include "lz4_match_copy_kernel.h"
#include <kernels/kernel_builder.h>
#include <kernels/streamset.h>

using namespace llvm;
using namespace kernel;
using namespace std;

void LZ4MatchCopyKernel::generateOutputCopy(const std::unique_ptr<KernelBuilder> &iBuilder) {
    BasicBlock *entryBlock = iBuilder->GetInsertBlock();
    Value *previousCopy = iBuilder->getScalarField("previousCopy");
    Value* previousProduced = iBuilder->getProducedItemCount("outputStream");
    Value* copyStart = iBuilder->CreateSelect(
            iBuilder->CreateICmpULT(previousCopy, previousProduced),
            previousProduced,
            previousCopy
    );


    Value * itemsToDo = mAvailableItemCount[0];
    Value *itemsAvailable = iBuilder->CreateAdd(iBuilder->getAvailableItemCount("decompressedStream"), previousCopy);
//    iBuilder->CallPrintInt("itemsAvailable", iBuilder->getAvailableItemCount("decompressedStream"));

//    iBuilder->CallPrintInt("itemsToDo", itemsToDo);


    size_t decompressedStreamBufferSize = this->getInputBufferSize(iBuilder, "decompressedStream");
    Value *bufferSize = iBuilder->getSize(decompressedStreamBufferSize);

    Value *inputBasePtr = iBuilder->getRawInputPointer("decompressedStream", iBuilder->getSize(0));
    Value *outputBasePtr = iBuilder->getRawOutputPointer("outputStream", iBuilder->getSize(0));
//    iBuilder->CallPrintInt("copyStart", copyStart);


    Value *previousRound = iBuilder->CreateUDiv(copyStart, bufferSize);
    Value *previousOffset = iBuilder->CreateURem(copyStart, bufferSize);

    Value *curRound = iBuilder->CreateUDiv(itemsAvailable, bufferSize);
    Value *curOffset = iBuilder->CreateURem(itemsAvailable, bufferSize);


//    iBuilder->CallPrintInt("previousRound", previousRound);
//    iBuilder->CallPrintInt("previousOffset", previousOffset);
//    iBuilder->CallPrintInt("curRound", curRound);
//    iBuilder->CallPrintInt("curOffset", curOffset);

    Value *notReachEnd = iBuilder->CreateICmpEQ(previousRound, curRound);
    Value *copyEndOffset1 = iBuilder->CreateSelect(notReachEnd, curOffset, bufferSize);
    Value *copySize1 = iBuilder->CreateSub(copyEndOffset1, previousOffset);


    iBuilder->CreateMemCpy(
            iBuilder->CreateGEP(outputBasePtr, previousOffset),
            iBuilder->CreateGEP(inputBasePtr, previousOffset),
            copySize1,
            1 // Not align guaranteed
    );
//    iBuilder->CallPrintInt("bbb", iBuilder->getSize(0));
    iBuilder->CreateMemCpy(
            iBuilder->CreateGEP(outputBasePtr, iBuilder->getSize(0)),
            iBuilder->CreateGEP(inputBasePtr, iBuilder->getSize(0)),
            iBuilder->CreateSelect(notReachEnd, iBuilder->getSize(0), curOffset),
            1 // Not align guaranteed
    );
//    iBuilder->CallPrintInt("ccc", iBuilder->getSize(0));

    iBuilder->setProcessedItemCount("decompressedStream", itemsAvailable);
//    iBuilder->setProducedItemCount("outputStream", itemsAvailable);
//    iBuilder->CallPrintInt("producedItemCount", iBuilder->getProducedItemCount("outputStream"));
    iBuilder->setScalarField("previousCopy", itemsAvailable);

}

void LZ4MatchCopyKernel::generateMultiBlockLogic(const unique_ptr<KernelBuilder> &iBuilder, Value * const numOfStrides) {
//    BasicBlock * entryBlock = iBuilder->GetInsertBlock();

    // Copy
//    iBuilder->CallPrintInt("available", iBuilder->getAvailableItemCount("decompressedStream"));
    this->generateOutputCopy(iBuilder);


    BasicBlock *copyEndBlock = iBuilder->CreateBasicBlock("copyEnd");
    iBuilder->CreateBr(copyEndBlock);
    iBuilder->SetInsertPoint(copyEndBlock);
//    return;
    BasicBlock *exitBlock = iBuilder->CreateBasicBlock("exit_block");
    Value *initProcessIndex = iBuilder->getScalarField("currentProcessIndex");
    Value *itemsAvailable = iBuilder->CreateAdd(
            iBuilder->getProcessedItemCount("m0Start"),
            iBuilder->getAvailableItemCount("m0Start")
    );


    BasicBlock *iterLoopCon = iBuilder->CreateBasicBlock("iter_loop_con");
    BasicBlock *iterLoopBody = iBuilder->CreateBasicBlock("iter_loop_body");
    BasicBlock *iterLoopExit = iBuilder->CreateBasicBlock("iter_loop_exit");

    iBuilder->CreateBr(iterLoopCon);

    // Con
    iBuilder->SetInsertPoint(iterLoopCon);
    PHINode *currentProcessIndex = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
    currentProcessIndex->addIncoming(initProcessIndex, copyEndBlock);

    iBuilder->CreateCondBr(
            iBuilder->CreateICmpULT(currentProcessIndex, itemsAvailable),
            iterLoopBody,
            iterLoopExit
    );

    // Body
    iBuilder->SetInsertPoint(iterLoopBody);


    Value *currentM0Start = this->generateLoadCircularInput(iBuilder, "m0Start", currentProcessIndex,
                                                            iBuilder->getInt64Ty()->getPointerTo());
    Value *currentDepositStart = currentM0Start;

    BasicBlock *matchCopyBody = iBuilder->CreateBasicBlock("match_copy_body");
    Value *producedItemsCount = iBuilder->getProcessedItemCount("decompressedStream");

    iBuilder->CreateCondBr(
            iBuilder->CreateICmpULE(
                    iBuilder->CreateSub(currentDepositStart, iBuilder->getInt64(1)),
                    producedItemsCount
            ),
            matchCopyBody,
            iterLoopExit
    );

    // matchCopyBody
    iBuilder->SetInsertPoint(matchCopyBody);
    this->generateMatchCopy(iBuilder, currentProcessIndex); // TODO main logic here
    BasicBlock *matchCopyFinishBlock = iBuilder->CreateBasicBlock("match_copy_finish");
    iBuilder->CreateBr(matchCopyFinishBlock);
    iBuilder->SetInsertPoint(matchCopyFinishBlock);


    Value *m0End = this->generateLoadCircularInput(iBuilder, "m0End", currentProcessIndex,
                                                   iBuilder->getInt64Ty()->getPointerTo());
    Value *depositEnd = iBuilder->CreateAdd(m0End, iBuilder->getInt64(1));
    Value *maxProducedCount = iBuilder->CreateSelect(
            iBuilder->CreateICmpUGT(
                    producedItemsCount,
                    depositEnd
            ),
            producedItemsCount,
            depositEnd
    );
    iBuilder->setProducedItemCount("outputStream", maxProducedCount);
    currentProcessIndex->addIncoming(
            iBuilder->CreateAdd(currentProcessIndex, iBuilder->getSize(1)),
            matchCopyFinishBlock
    );
    iBuilder->CreateBr(iterLoopCon);


    // loop exit
    iBuilder->SetInsertPoint(iterLoopExit);
    iBuilder->setScalarField("currentProcessIndex", currentProcessIndex);

    iBuilder->CreateBr(exitBlock);

    // Exit
    iBuilder->SetInsertPoint(exitBlock);
}

Value *LZ4MatchCopyKernel::generateMatchCopy(const unique_ptr<KernelBuilder> &iBuilder, Value *currentProcessIndex) {

    BasicBlock *entryBlock = iBuilder->GetInsertBlock();


    Value *m0Start = this->generateLoadCircularInput(iBuilder, "m0Start", currentProcessIndex,
                                                     iBuilder->getInt64Ty()->getPointerTo());
    Value *depositStart = m0Start;
    Value *m0End = this->generateLoadCircularInput(iBuilder, "m0End", currentProcessIndex,
                                                   iBuilder->getInt64Ty()->getPointerTo());
    Value *depositEnd = iBuilder->CreateAdd(m0End, iBuilder->getInt64(1));

    Value *matchOffset = this->generateLoadCircularInput(iBuilder, "matchOffset", currentProcessIndex,
                                                         iBuilder->getInt64Ty()->getPointerTo());

    Value *matchLength = iBuilder->CreateSub(depositEnd, depositStart);

    Value *matchStart = iBuilder->CreateSub(depositStart, matchOffset);
//    iBuilder->CallPrintInt("depositStart", depositStart);
//    iBuilder->CallPrintInt("matchOffset", matchOffset);
//    iBuilder->CallPrintInt("matchStart", matchStart);
//    iBuilder->CallPrintInt("matchLength", matchLength);


    BasicBlock* copyLoopCon = iBuilder->CreateBasicBlock("copy_loop_con");
    BasicBlock* copyLoopBody = iBuilder->CreateBasicBlock("copy_loop_body");
    BasicBlock* copyLoopExit = iBuilder->CreateBasicBlock("copy_loop_exit");

    iBuilder->CreateBr(copyLoopCon);
    iBuilder->SetInsertPoint(copyLoopCon);

    PHINode* currentCopyIndex = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
    currentCopyIndex->addIncoming(iBuilder->getSize(0), entryBlock);

    iBuilder->CreateCondBr(iBuilder->CreateICmpULT(currentCopyIndex, matchLength), copyLoopBody, copyLoopExit);

    iBuilder->SetInsertPoint(copyLoopBody);
    Value* value = this->generateLoadCircularOutput(iBuilder, "outputStream", iBuilder->CreateAdd(matchStart, currentCopyIndex), iBuilder->getInt8Ty()->getPointerTo());
//    iBuilder->CallPrintInt("value", value);
//    iBuilder->CallPrintInt("storePos", iBuilder->CreateAdd(currentCopyIndex, depositStart));
    this->generateStoreCircularOutput(iBuilder, "outputStream", iBuilder->CreateAdd(currentCopyIndex, depositStart),iBuilder->getInt8Ty()->getPointerTo(), value);
    currentCopyIndex->addIncoming(iBuilder->CreateAdd(currentCopyIndex, iBuilder->getSize(1)), copyLoopBody);

    iBuilder->CreateBr(copyLoopCon);

    iBuilder->SetInsertPoint(copyLoopExit);

}

void LZ4MatchCopyKernel::generateStoreCircularOutput(const unique_ptr<KernelBuilder> &iBuilder, string outputBufferName, Value* offset, Type* pointerType, Value* value) {
    size_t inputSize = this->getOutputBufferSize(iBuilder, outputBufferName);
    Value* offsetMask = iBuilder->getSize(inputSize - 1);
    Value* maskedOffset = iBuilder->CreateAnd(offsetMask, offset);

    Value* outputBufferPtr = iBuilder->getRawOutputPointer(outputBufferName, iBuilder->getSize(0));

    outputBufferPtr = iBuilder->CreatePointerCast(outputBufferPtr, pointerType);
    iBuilder->CreateStore(value, iBuilder->CreateGEP(outputBufferPtr, maskedOffset));
}

Value* LZ4MatchCopyKernel::generateLoadCircularOutput(const unique_ptr<KernelBuilder> &iBuilder, string inputBufferName, Value* offset, Type* pointerType) {
    size_t inputSize = this->getOutputBufferSize(iBuilder, inputBufferName);
    Value* offsetMask = iBuilder->getSize(inputSize - 1);
    Value* maskedOffset = iBuilder->CreateAnd(offsetMask, offset);

    Value* inputBufferPtr = iBuilder->getRawOutputPointer(inputBufferName, iBuilder->getSize(0));

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
                                      Binding{iBuilder->getStreamSetTy(1, 8), "decompressedStream"},
                                      Binding{iBuilder->getStreamSetTy(1, 64), "m0Start", BoundedRate(0, 1)},
                                      Binding{iBuilder->getStreamSetTy(1, 64), "m0End", BoundedRate(0, 1)},
                                      Binding{iBuilder->getStreamSetTy(1, 64), "matchOffset", BoundedRate(0, 1)},

                              },
        // Outputs
                              {Binding{iBuilder->getStreamSetTy(1, 8), "outputStream", UnknownRate()}},
        // Arguments
                              {},
                              {},
                              {
                                      Binding{iBuilder->getSizeTy(), "currentProcessIndex"},
                                      Binding{iBuilder->getSizeTy(), "previousCopy"}
                              }) {
//    setNoTerminateAttribute(true);
}
