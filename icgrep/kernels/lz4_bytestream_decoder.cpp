/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "lz4_bytestream_decoder.h"
#include <kernels/kernel_builder.h>

using namespace llvm;
using namespace kernel;

Value * getInputPtr(const std::unique_ptr<KernelBuilder> & iBuilder, Value * blockStartPtr, Value * offset) {
    return iBuilder->CreateGEP(
            iBuilder->CreatePointerCast(blockStartPtr, iBuilder->getInt32Ty()->getPointerTo()),
            offset
            );
}

Value * selectMin(const std::unique_ptr<KernelBuilder> & iBuilder, Value * a, Value * b) {
    return iBuilder->CreateSelect(iBuilder->CreateICmpULT(a, b), a, b);
}

void LZ4ByteStreamDecoderKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    BasicBlock * entry_block = iBuilder->GetInsertBlock();
    BasicBlock * loopBody = iBuilder->CreateBasicBlock("bytestream_block_loop_body");
    BasicBlock * loopExit = iBuilder->CreateBasicBlock("bytestream_block_loop_exit");

    Value * bufferSize = iBuilder->getSize(mBufferSize);
    Value * bufferSizeMask = iBuilder->CreateSub(bufferSize, iBuilder->getSize(1));
    Value * iterations = selectMin(iBuilder,
            iBuilder->getSize(iBuilder->getBitBlockWidth()),
            iBuilder->CreateSub(iBuilder->getAvailableItemCount("literalIndexes"), iBuilder->getProcessedItemCount("literalIndexes")));
    Value * inputBufferBasePtr = iBuilder->getRawInputPointer("inputStream", iBuilder->getSize(0));
    Value * outputBufferBasePtr = iBuilder->getRawOutputPointer("outputStream", iBuilder->getSize(0));
    iBuilder->CreateBr(loopBody);

    iBuilder->SetInsertPoint(loopBody);
    PHINode * phiInputIndex = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "inputIndex");
    phiInputIndex->addIncoming(iBuilder->getSize(0), entry_block);

    // =================================================
    // Indexes extraction.
    Value * literalStartPtr = getInputPtr(iBuilder,
            iBuilder->getInputStreamBlockPtr("literalIndexes", iBuilder->getSize(0)), phiInputIndex);
    Value * literalLengthPtr = getInputPtr(iBuilder,
            iBuilder->getInputStreamBlockPtr("literalIndexes", iBuilder->getSize(1)), phiInputIndex);
    Value * matchOffsetPtr = getInputPtr(iBuilder,
            iBuilder->getInputStreamBlockPtr("matchIndexes", iBuilder->getSize(0)), phiInputIndex);
    Value * matchLengthPtr = getInputPtr(iBuilder,
            iBuilder->getInputStreamBlockPtr("matchIndexes", iBuilder->getSize(1)), phiInputIndex);
    Value * literalStart = iBuilder->CreateZExt(iBuilder->CreateLoad(literalStartPtr), iBuilder->getSizeTy());
    Value * literalLength = iBuilder->CreateZExt(iBuilder->CreateLoad(literalLengthPtr), iBuilder->getSizeTy());
    Value * matchOffset = iBuilder->CreateZExt(iBuilder->CreateLoad(matchOffsetPtr), iBuilder->getSizeTy());
    Value * matchLength = iBuilder->CreateZExt(iBuilder->CreateLoad(matchLengthPtr), iBuilder->getSizeTy());

//    iBuilder->CallPrintInt(" ----- literalStart", literalStart);
//    iBuilder->CallPrintInt(" ----- literalLength", literalLength);
//    iBuilder->CallPrintInt(" ----- matchOffset", matchOffset);
//    iBuilder->CallPrintInt(" ----- matchLength", matchLength);

//#if 0
//    Value * processedItem = iBuilder->CreateAdd(iBuilder->getProcessedItemCount("literalIndexes"), phiInputIndex);
//    iBuilder->CallPrintInt("ProccessedItem", processedItem);
//    iBuilder->CallPrintInt("LiteralStart", literalStart);
//    iBuilder->CallPrintInt("LiteralLength", literalLength);
//    iBuilder->CallPrintInt("MatchOffset", matchOffset);
//    iBuilder->CallPrintInt("MatchLength", matchLength);
//#endif

    // =================================================
    // Literals.
    Value * outputItems = iBuilder->getProducedItemCount("outputStream");
    Value * bufferOffset = iBuilder->CreateAnd(outputItems, bufferSizeMask);
    Value * remainingBuffer = iBuilder->CreateSub(bufferSize, bufferOffset);
    Value * copyLength1 = selectMin(iBuilder, remainingBuffer, literalLength);
    iBuilder->CreateMemCpy(
            iBuilder->CreateGEP(outputBufferBasePtr, bufferOffset),
            iBuilder->CreateGEP(inputBufferBasePtr, literalStart),
            copyLength1, 1);    // no alignment guaranteed
    // Potential wrap around.
    iBuilder->CreateMemCpy(
            outputBufferBasePtr,
            iBuilder->CreateGEP(inputBufferBasePtr, iBuilder->CreateAdd(literalStart, copyLength1)),
            iBuilder->CreateSub(literalLength, copyLength1), 1); // Buffer start is aligned.
    // NOTE: Test case reported non-8-byte alignment
    outputItems = iBuilder->CreateAdd(outputItems, literalLength);

    // =================================================
    // Match copy.
    // Conceptually, copy [cur-matchOffset, cur-matchOffset+matchLength] to
    // [cur, cur+matchLength] sequentially, with two ranges potentially overlapping.
    // If matchOffset is larger than 4, we copy 4 bytes at a time; otherwise, one byte a time.
    Value * matchStart = iBuilder->CreateSub(outputItems, matchOffset);
    Value * baseSrcOffset = iBuilder->CreateAnd(matchStart, bufferSizeMask);
    Value * baseDstOffset = iBuilder->CreateAnd(outputItems, bufferSizeMask);
    Value * copyStep = iBuilder->CreateSelect(
            iBuilder->CreateICmpULT(matchOffset, iBuilder->getSize(4)),
            iBuilder->getSize(1),
            iBuilder->getSize(4)
            );
    BasicBlock * cpyLoopCond = iBuilder->CreateBasicBlock("matchcopy_loop_cond");
    BasicBlock * cpyLoopBody = iBuilder->CreateBasicBlock("matchcopy_loop_body");
    BasicBlock * cpyLoopExit = iBuilder->CreateBasicBlock("matchcopy_loop_exit");
    iBuilder->CreateBr(cpyLoopCond);

    iBuilder->SetInsertPoint(cpyLoopCond);
    PHINode * phiSrcOffset = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3, "srcOffset");
    PHINode * phiDstOffset = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3, "dstOffset");
    PHINode * phiIter = iBuilder->CreatePHI(iBuilder->getSizeTy(), 3, "iterator");
    phiSrcOffset->addIncoming(baseSrcOffset, loopBody);
    phiDstOffset->addIncoming(baseDstOffset, loopBody);
    phiIter->addIncoming(iBuilder->getSize(0), loopBody);
    iBuilder->CreateCondBr(
            iBuilder->CreateICmpUGE(phiIter, matchLength),
            cpyLoopExit,
            cpyLoopBody
            );

    iBuilder->SetInsertPoint(cpyLoopBody);
//#ifndef NDEBUG
//    iBuilder->CallPrintIntToStderr("srcOffset", phiSrcOffset);
//    iBuilder->CallPrintIntToStderr("dstOffset", phiDstOffset);
//#endif
    BasicBlock * reachingBufferEnd_then = iBuilder->CreateBasicBlock("matchcopy_reaching_buf_end_then");
    BasicBlock * reachingBufferEnd_else = iBuilder->CreateBasicBlock("matchcopy_reaching_buf_end_else");
    Value * distSrcEnd = iBuilder->CreateSub(bufferSize, phiSrcOffset);
    Value * distDstEnd = iBuilder->CreateSub(bufferSize, phiDstOffset);
    Value * minDist = selectMin(iBuilder, distSrcEnd, distDstEnd);
    iBuilder->CreateUnlikelyCondBr(
            iBuilder->CreateICmpULE(minDist, iBuilder->getSize(4)),
            reachingBufferEnd_then,
            reachingBufferEnd_else
            );

    iBuilder->SetInsertPoint(reachingBufferEnd_then);
    Value * src8 = iBuilder->CreateGEP(outputBufferBasePtr, phiSrcOffset);
    Value * dst8 = iBuilder->CreateGEP(outputBufferBasePtr, phiDstOffset);
    iBuilder->CreateStore(iBuilder->CreateLoad(src8), dst8);
    Value * newSrcOffset = iBuilder->CreateAnd(
            iBuilder->CreateAdd(phiSrcOffset, iBuilder->getSize(1)),
            bufferSizeMask
            );
    Value * newDstOffset = iBuilder->CreateAnd(
            iBuilder->CreateAdd(phiDstOffset, iBuilder->getSize(1)),
            bufferSizeMask
            );
    phiSrcOffset->addIncoming(newSrcOffset, reachingBufferEnd_then);
    phiDstOffset->addIncoming(newDstOffset, reachingBufferEnd_then);
    phiIter->addIncoming(iBuilder->CreateAdd(phiIter, iBuilder->getSize(1)), reachingBufferEnd_then);
    iBuilder->CreateBr(cpyLoopCond);

    iBuilder->SetInsertPoint(reachingBufferEnd_else);
    // Copy 4 bytes at a time (regardless of step length).
    Value * src32 = iBuilder->CreatePointerCast(
            iBuilder->CreateGEP(outputBufferBasePtr, phiSrcOffset),
            iBuilder->getInt32Ty()->getPointerTo());
    Value * dst32 = iBuilder->CreatePointerCast(
            iBuilder->CreateGEP(outputBufferBasePtr, phiDstOffset),
            iBuilder->getInt32Ty()->getPointerTo());
    // Force unaligned load/store of an int32.
    iBuilder->CreateAlignedStore(iBuilder->CreateAlignedLoad(src32, 1), dst32, 1);
    newSrcOffset = iBuilder->CreateAnd(
            iBuilder->CreateAdd(phiSrcOffset, copyStep),
            bufferSizeMask
            );
    newDstOffset = iBuilder->CreateAnd(
            iBuilder->CreateAdd(phiDstOffset, copyStep),
            bufferSizeMask
            );
    phiSrcOffset->addIncoming(newSrcOffset, reachingBufferEnd_else);
    phiDstOffset->addIncoming(newDstOffset, reachingBufferEnd_else);
    phiIter->addIncoming(iBuilder->CreateAdd(phiIter, copyStep), reachingBufferEnd_else);
    iBuilder->CreateBr(cpyLoopCond);

    iBuilder->SetInsertPoint(cpyLoopExit);
    outputItems = iBuilder->CreateAdd(outputItems, matchLength);
    iBuilder->setProducedItemCount("outputStream", outputItems);

    Value * newInputIndex = iBuilder->CreateAdd(phiInputIndex, iBuilder->getSize(1));
    phiInputIndex->addIncoming(newInputIndex, cpyLoopExit);
    iBuilder->CreateUnlikelyCondBr(
            iBuilder->CreateICmpEQ(newInputIndex, iterations),
            loopExit,
            loopBody
            );

    iBuilder->SetInsertPoint(loopExit);
//#ifndef NDEBUG
//    iBuilder->CallPrintInt("Decompressed bytes", iBuilder->getProducedItemCount("outputStream"));
//#endif
}


LZ4ByteStreamDecoderKernel::LZ4ByteStreamDecoderKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, size_t bufferSize)
: BlockOrientedKernel("lz4ByteStreamDecoder",
    // Inputs
    {Binding{iBuilder->getStreamSetTy(2, 32), "literalIndexes"},
     Binding{iBuilder->getStreamSetTy(2, 32), "matchIndexes"},
     Binding{iBuilder->getStreamSetTy(1, 8), "inputStream", UnknownRate(), LookBehind(65536)}},
    // Outputs
    {Binding{iBuilder->getStreamSetTy(1, 8), "outputStream", UnknownRate()}},
    // Arguments
    {},
    {},
    {}),
 mBufferSize(bufferSize) {
    setNoTerminateAttribute(true);
}
