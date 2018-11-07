/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "lz4_bytestream_decoder.h"
#include <kernels/kernel_builder.h>

using namespace llvm;
using namespace kernel;

void LZ4ByteStreamDecoderKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * numOfStrides) {

    BasicBlock * entry_block = b->GetInsertBlock();
    BasicBlock * loopBody = b->CreateBasicBlock("bytestream_block_loop_body");
    BasicBlock * loopExit = b->CreateBasicBlock("bytestream_block_loop_exit");
    Type * const i32PtrTy = b->getInt32Ty()->getPointerTo();
    Type * const sizeTy = b->getSizeTy();
    assert (mBufferSize > 0);
    Value * bufferSize = b->getSize(mBufferSize);
    Value * bufferSizeMask = b->getSize(mBufferSize - 1);
    Value * const iterations = b->getAvailableItemCount("literalIndexes");
    Value * const inputBufferBasePtr = b->getRawInputPointer("inputStream", b->getInt32(0));
    Value * const outputBufferBasePtr = b->getRawOutputPointer("outputStream", b->getInt32(0));
    Value * baseLiteralStartPtr = b->getInputStreamBlockPtr("literalIndexes", b->getSize(0));
    baseLiteralStartPtr = b->CreatePointerCast(baseLiteralStartPtr, i32PtrTy);
    Value * baseLiteralLengthPtr = b->getInputStreamBlockPtr("literalIndexes", b->getSize(1));
    baseLiteralLengthPtr = b->CreatePointerCast(baseLiteralLengthPtr, i32PtrTy);
    Value * baseMatchOffsetPtr = b->getInputStreamBlockPtr("matchIndexes", b->getSize(0));
    baseMatchOffsetPtr = b->CreatePointerCast(baseMatchOffsetPtr, i32PtrTy);
    Value * baseMatchLengthPtr = b->getInputStreamBlockPtr("matchIndexes", b->getSize(1));
    baseMatchLengthPtr = b->CreatePointerCast(baseMatchLengthPtr, i32PtrTy);
    b->CreateBr(loopBody);

    b->SetInsertPoint(loopBody);
    PHINode * phiInputIndex = b->CreatePHI(sizeTy, 2, "inputIndex");
    phiInputIndex->addIncoming(b->getSize(0), entry_block);

    // =================================================
    // Indexes extraction.


    Value * literalStartPtr = b->CreateGEP(baseLiteralStartPtr, phiInputIndex);
    Value * literalLengthPtr = b->CreateGEP(baseLiteralLengthPtr, phiInputIndex);
    Value * matchOffsetPtr = b->CreateGEP(baseMatchOffsetPtr, phiInputIndex);
    Value * matchLengthPtr = b->CreateGEP(baseMatchLengthPtr, phiInputIndex);

    Value * literalStart = b->CreateZExt(b->CreateLoad(literalStartPtr), sizeTy);
    Value * literalLength = b->CreateZExt(b->CreateLoad(literalLengthPtr), sizeTy);
    Value * matchOffset = b->CreateZExt(b->CreateLoad(matchOffsetPtr), sizeTy);
    Value * matchLength = b->CreateZExt(b->CreateLoad(matchLengthPtr), sizeTy);

    // =================================================
    // Literals.
    Value * outputItems = b->getProducedItemCount("outputStream");
    Value * bufferOffset = b->CreateAnd(outputItems, bufferSizeMask);
    Value * remainingBuffer = b->CreateSub(bufferSize, bufferOffset);
    Value * copyLength1 = b->CreateUMin(remainingBuffer, literalLength);
    b->CreateMemCpy(
            b->CreateGEP(outputBufferBasePtr, bufferOffset),
            b->CreateGEP(inputBufferBasePtr, literalStart),
            copyLength1, 1);    // no alignment guaranteed
    // Potential wrap around.
    b->CreateMemCpy(
            outputBufferBasePtr,
            b->CreateGEP(inputBufferBasePtr, b->CreateAdd(literalStart, copyLength1)),
            b->CreateSub(literalLength, copyLength1), 1); // Buffer start is aligned.
    // NOTE: Test case reported non-8-byte alignment
    outputItems = b->CreateAdd(outputItems, literalLength);

    // =================================================
    // Match copy.
    // Conceptually, copy [cur-matchOffset, cur-matchOffset+matchLength] to
    // [cur, cur+matchLength] sequentially, with two ranges potentially overlapping.
    // If matchOffset is larger than 4, we copy 4 bytes at a time; otherwise, one byte a time.
    Value * matchStart = b->CreateSub(outputItems, matchOffset);
    Value * baseSrcOffset = b->CreateAnd(matchStart, bufferSizeMask);
    Value * baseDstOffset = b->CreateAnd(outputItems, bufferSizeMask);
    Value * const copyStep = b->CreateSelect(
            b->CreateICmpULT(matchOffset, b->getSize(4)),
            b->getSize(1),
            b->getSize(4));
    BasicBlock * cpyLoopCond = b->CreateBasicBlock("matchcopy_loop_cond");
    BasicBlock * cpyLoopBody = b->CreateBasicBlock("matchcopy_loop_body");
    BasicBlock * cpyLoopExit = b->CreateBasicBlock("matchcopy_loop_exit");
    b->CreateBr(cpyLoopCond);

    b->SetInsertPoint(cpyLoopCond);
    PHINode * phiSrcOffset = b->CreatePHI(sizeTy, 3, "srcOffset");
    PHINode * phiDstOffset = b->CreatePHI(sizeTy, 3, "dstOffset");
    PHINode * phiIter = b->CreatePHI(sizeTy, 3, "iterator");
    phiSrcOffset->addIncoming(baseSrcOffset, loopBody);
    phiDstOffset->addIncoming(baseDstOffset, loopBody);
    phiIter->addIncoming(b->getSize(0), loopBody);
    b->CreateCondBr(
            b->CreateICmpUGE(phiIter, matchLength),
            cpyLoopExit,
            cpyLoopBody
            );

    b->SetInsertPoint(cpyLoopBody);
//#ifndef NDEBUG
//    iBuilder->CallPrintInt("srcOffset", phiSrcOffset);
//    iBuilder->CallPrintInt("dstOffset", phiDstOffset);
//#endif
    BasicBlock * reachingBufferEnd_then = b->CreateBasicBlock("matchcopy_reaching_buf_end_then");
    BasicBlock * reachingBufferEnd_else = b->CreateBasicBlock("matchcopy_reaching_buf_end_else");
    Value * distSrcEnd = b->CreateSub(bufferSize, phiSrcOffset);
    Value * distDstEnd = b->CreateSub(bufferSize, phiDstOffset);
    Value * minDist = b->CreateUMin(distSrcEnd, distDstEnd);
    b->CreateUnlikelyCondBr(
            b->CreateICmpULE(minDist, b->getSize(4)),
            reachingBufferEnd_then,
            reachingBufferEnd_else
            );

    b->SetInsertPoint(reachingBufferEnd_then);
    Value * src8 = b->CreateGEP(outputBufferBasePtr, phiSrcOffset);
    Value * dst8 = b->CreateGEP(outputBufferBasePtr, phiDstOffset);
    b->CreateStore(b->CreateLoad(src8), dst8);
    Value * newSrcOffset = b->CreateAnd(
            b->CreateAdd(phiSrcOffset, b->getSize(1)),
            bufferSizeMask
            );
    Value * newDstOffset = b->CreateAnd(
            b->CreateAdd(phiDstOffset, b->getSize(1)),
            bufferSizeMask
            );
    phiSrcOffset->addIncoming(newSrcOffset, reachingBufferEnd_then);
    phiDstOffset->addIncoming(newDstOffset, reachingBufferEnd_then);
    phiIter->addIncoming(b->CreateAdd(phiIter, b->getSize(1)), reachingBufferEnd_then);
    b->CreateBr(cpyLoopCond);

    b->SetInsertPoint(reachingBufferEnd_else);
    // Copy 4 bytes at a time (regardless of step length).
    Value * src32 = b->CreatePointerCast(
            b->CreateGEP(outputBufferBasePtr, phiSrcOffset),
            b->getInt32Ty()->getPointerTo());
    Value * dst32 = b->CreatePointerCast(
            b->CreateGEP(outputBufferBasePtr, phiDstOffset),
            b->getInt32Ty()->getPointerTo());
    // Force unaligned load/store of an int32.
    b->CreateAlignedStore(b->CreateAlignedLoad(src32, 1), dst32, 1);
    newSrcOffset = b->CreateAnd(
            b->CreateAdd(phiSrcOffset, copyStep),
            bufferSizeMask
            );
    newDstOffset = b->CreateAnd(
            b->CreateAdd(phiDstOffset, copyStep),
            bufferSizeMask
            );
    phiSrcOffset->addIncoming(newSrcOffset, reachingBufferEnd_else);
    phiDstOffset->addIncoming(newDstOffset, reachingBufferEnd_else);
    phiIter->addIncoming(b->CreateAdd(phiIter, copyStep), reachingBufferEnd_else);
    b->CreateBr(cpyLoopCond);

    b->SetInsertPoint(cpyLoopExit);
    outputItems = b->CreateAdd(outputItems, matchLength);
    b->setProducedItemCount("outputStream", outputItems);

    Value * newInputIndex = b->CreateAdd(phiInputIndex, b->getSize(1));
    phiInputIndex->addIncoming(newInputIndex, cpyLoopExit);
    b->CreateUnlikelyCondBr(
            b->CreateICmpEQ(newInputIndex, iterations),
            loopExit,
            loopBody
            );

    b->SetInsertPoint(loopExit);
}


LZ4ByteStreamDecoderKernel::LZ4ByteStreamDecoderKernel(const std::unique_ptr<kernel::KernelBuilder> &,
                                                       // inputs
                                                       StreamSet * literalIndexes,
                                                       StreamSet * matchIndexes,
                                                       StreamSet * inputStream,
                                                       // output
                                                       StreamSet * outputStream)
: MultiBlockKernel("lz4ByteStreamDecoder",
// Inputs
{Binding{"literalIndexes", literalIndexes},
 Binding{"matchIndexes", matchIndexes},
 Binding{"inputStream", inputStream, FixedRate(), { Deferred(), Misaligned(), LookBehind(65536) }}},
// Outputs
{Binding{"outputStream", outputStream, UnknownRate()}},
// Arguments
{},
{},
{})
, mBufferSize(4 * 1024 * 1024) {
    setStride(mBufferSize);
}


