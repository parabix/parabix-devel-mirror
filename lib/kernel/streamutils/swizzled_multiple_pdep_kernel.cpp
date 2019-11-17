/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/streamutils/swizzled_multiple_pdep_kernel.h>

#include <kernel/core/kernel_builder.h>
#include <toolchain/toolchain.h>
#include <llvm/IR/Intrinsics.h>

using namespace llvm;

namespace kernel {

SwizzledMultiplePDEPkernel::SwizzledMultiplePDEPkernel(BuilderRef b, const unsigned swizzleFactor, const unsigned numberOfStreamSet, std::string name)
: MultiBlockKernel(b, std::move(name),
// input stream sets
{Binding{b->getStreamSetTy(), "marker", FixedRate(), Principal()},
Binding{b->getStreamSetTy(swizzleFactor), "source0", PopcountOf("marker"), BlockSize(b->getBitBlockWidth() / swizzleFactor) }},
// output stream set
{Binding{b->getStreamSetTy(swizzleFactor), "output0", FixedRate(), BlockSize(b->getBitBlockWidth() / swizzleFactor)}},
{}, {}, {})
, mSwizzleFactor(swizzleFactor), mNumberOfStreamSet(numberOfStreamSet) {
    for (unsigned i = 1; i < numberOfStreamSet; i++) {
        mInputStreamSets.push_back(Binding{b->getStreamSetTy(swizzleFactor), "source" + std::to_string(i), RateEqualTo("source0"), BlockSize(b->getBitBlockWidth() / swizzleFactor) });
        mOutputStreamSets.push_back(Binding{b->getStreamSetTy(swizzleFactor), "output" + std::to_string(i), RateEqualTo("output0"), BlockSize(b->getBitBlockWidth() / swizzleFactor)});
    }
}

void SwizzledMultiplePDEPkernel::generateMultiBlockLogic(BuilderRef b, Value * const numOfBlocks) {
    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const processBlock = b->CreateBasicBlock("processBlock");
    BasicBlock * const finishedStrides = b->CreateBasicBlock("finishedStrides");
    const auto pdepWidth = b->getBitBlockWidth() / mSwizzleFactor;
    ConstantInt * const BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());
    ConstantInt * const PDEP_WIDTH = b->getSize(pdepWidth);

    Constant * const ZERO = b->getSize(0);
    Value * const sourceItemCount = b->getProcessedItemCount("source0");

    Value * const initialSourceOffset = b->CreateURem(sourceItemCount, BLOCK_WIDTH);
    b->CreateBr(processBlock);

    b->SetInsertPoint(processBlock);
    PHINode * const strideIndex = b->CreatePHI(b->getSizeTy(), 2);
    strideIndex->addIncoming(ZERO, entry);

    std::vector<PHINode*> bufferPhiArray(mNumberOfStreamSet, NULL);
    std::vector<Value*> bufferArray(mNumberOfStreamSet, NULL);
    for (unsigned iStreamSetIndex = 0; iStreamSetIndex < mNumberOfStreamSet; iStreamSetIndex++) {
        PHINode * const bufferPhi = b->CreatePHI(b->getBitBlockType(), 2);
        bufferPhi->addIncoming(Constant::getNullValue(b->getBitBlockType()), entry);
        bufferPhiArray[iStreamSetIndex] = bufferPhi;
        bufferArray[iStreamSetIndex] = bufferPhi;
    }

    PHINode * const sourceOffsetPhi = b->CreatePHI(b->getSizeTy(), 2);
    sourceOffsetPhi->addIncoming(initialSourceOffset, entry);
    PHINode * const bufferSizePhi = b->CreatePHI(b->getSizeTy(), 2);
    bufferSizePhi->addIncoming(ZERO, entry);

    // Extract the values we will use in the main processing loop
    Value * const markerStream = b->getInputStreamBlockPtr("marker", ZERO, strideIndex);
    Value * const markerValue = b->CreateBlockAlignedLoad(markerStream);
    Value * const selectors = b->fwCast(pdepWidth, markerValue);
    Value * const numOfSelectors = b->simd_popcount(pdepWidth, selectors);

    // For each element of the marker block
    Value * bufferSize = bufferSizePhi;
    Value * sourceOffset = sourceOffsetPhi;
    for (unsigned i = 0; i < mSwizzleFactor; i++) {

        // How many bits will we deposit?
        Value * const required = b->CreateExtractElement(numOfSelectors, b->getSize(i));

        // Aggressively enqueue any additional bits
        BasicBlock * const entry = b->GetInsertBlock();
        BasicBlock * const enqueueBits = b->CreateBasicBlock();
        b->CreateBr(enqueueBits);

        b->SetInsertPoint(enqueueBits);
        PHINode * const updatedBufferSize = b->CreatePHI(bufferSize->getType(), 2);
        updatedBufferSize->addIncoming(bufferSize, entry);
        PHINode * const updatedSourceOffset = b->CreatePHI(sourceOffset->getType(), 2);
        updatedSourceOffset->addIncoming(sourceOffset, entry);

        std::vector<PHINode * > updatedBufferArray(mNumberOfStreamSet, NULL);
        for (unsigned iStreamSetIndex = 0; iStreamSetIndex < mNumberOfStreamSet; iStreamSetIndex++) {
            Value* buffer = bufferArray[iStreamSetIndex];
            PHINode * const updatedBuffer = b->CreatePHI(buffer->getType(), 2);
            updatedBuffer->addIncoming(buffer, entry);
            updatedBufferArray[iStreamSetIndex] = updatedBuffer;
        }

        // Calculate the block and swizzle index of the current swizzle row
        Value * const blockOffset = b->CreateUDiv(updatedSourceOffset, BLOCK_WIDTH);
        Value * const swizzleIndex = b->CreateUDiv(b->CreateURem(updatedSourceOffset, BLOCK_WIDTH), PDEP_WIDTH);

        Value * const swizzleOffset = b->CreateURem(updatedSourceOffset, PDEP_WIDTH);

        for (unsigned iStreamSetIndex = 0; iStreamSetIndex < mNumberOfStreamSet; iStreamSetIndex++) {
            Value * const swizzle = b->CreateBlockAlignedLoad(b->getInputStreamBlockPtr("source" + std::to_string(iStreamSetIndex), swizzleIndex, blockOffset));

            // Shift the swizzle to the right to clear off any used bits ...
            Value * const swizzleShift = b->simd_fill(pdepWidth, swizzleOffset);
            Value * const unreadBits = b->CreateLShr(swizzle, swizzleShift);

            // ... then to the left to align the bits with the buffer and combine them.
            Value * const bufferShift = b->simd_fill(pdepWidth, updatedBufferSize);
            Value * const pendingBits = b->CreateShl(unreadBits, bufferShift);

            bufferArray[iStreamSetIndex] = b->CreateOr(updatedBufferArray[iStreamSetIndex], pendingBits);
            updatedBufferArray[iStreamSetIndex]->addIncoming(bufferArray[iStreamSetIndex], enqueueBits);
        }

        // Update the buffer size with the number of bits we have actually enqueued
        Value * const maxBufferSize = b->CreateAdd(b->CreateSub(PDEP_WIDTH, swizzleOffset), updatedBufferSize);
        bufferSize = b->CreateUMin(maxBufferSize, PDEP_WIDTH);
        updatedBufferSize->addIncoming(bufferSize, enqueueBits);

        // ... and increment the source offset by the number we actually inserted
        Value * const inserted = b->CreateSub(bufferSize, updatedBufferSize);
        sourceOffset = b->CreateAdd(updatedSourceOffset, inserted);
        updatedSourceOffset->addIncoming(sourceOffset, enqueueBits);

        // INVESTIGATE: we can branch at most once here. I'm not sure whether the potential
        // branch misprediction is better or worse than always filling from two swizzles to
        // ensure that we have enough bits to deposit.
        BasicBlock * const depositBits = b->CreateBasicBlock();
        b->CreateUnlikelyCondBr(b->CreateICmpULT(bufferSize, required), enqueueBits, depositBits);

        b->SetInsertPoint(depositBits);

        // Apply PDEP to each element of the combined swizzle using the current PDEP mask
        Value * const mask = b->CreateExtractElement(selectors, i);
        Value * const usedShift = b->simd_fill(pdepWidth, required);
        for (unsigned iStreamSetIndex = 0; iStreamSetIndex < mNumberOfStreamSet; iStreamSetIndex++) {
            Value* result = b->simd_pdep(pdepWidth, bufferArray[iStreamSetIndex], b->simd_fill(pdepWidth, mask));
            // Store the result
            Value * const outputStreamPtr = b->getOutputStreamBlockPtr("output" + std::to_string(iStreamSetIndex), b->getSize(i), strideIndex);
            b->CreateBlockAlignedStore(result, outputStreamPtr);

            // Shift away any used bits from the buffer and decrement our buffer size by the number we used
            bufferArray[iStreamSetIndex] = b->CreateLShr(bufferArray[iStreamSetIndex], usedShift);
        }

        bufferSize = b->CreateSub(bufferSize, required);
    }

    BasicBlock * const finishedBlock = b->GetInsertBlock();
    sourceOffsetPhi->addIncoming(sourceOffset, finishedBlock);
    bufferSizePhi->addIncoming(bufferSize, finishedBlock);
    for (unsigned iStreamSetIndex = 0; iStreamSetIndex < mNumberOfStreamSet; iStreamSetIndex++) {
        bufferPhiArray[iStreamSetIndex]->addIncoming(bufferArray[iStreamSetIndex], finishedBlock);
    }

    Value * const nextStrideIndex = b->CreateAdd(strideIndex, b->getSize(1));
    strideIndex->addIncoming(nextStrideIndex, finishedBlock);
    b->CreateLikelyCondBr(b->CreateICmpNE(nextStrideIndex, numOfBlocks), processBlock, finishedStrides);

    b->SetInsertPoint(finishedStrides);
}

}
