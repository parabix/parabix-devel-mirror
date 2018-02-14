/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "pdep_kernel.h"
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>
#include <toolchain/toolchain.h>

using namespace llvm;

namespace kernel {

PDEPkernel::PDEPkernel(const std::unique_ptr<kernel::KernelBuilder> & b, const unsigned swizzleFactor, std::string name)
: MultiBlockKernel(std::move(name),
// input stream sets
{Binding{b->getStreamSetTy(), "marker", FixedRate(), Principal()},
Binding{b->getStreamSetTy(swizzleFactor), "source", FixedRate(), Deferred()}},
// output stream set
{Binding{b->getStreamSetTy(swizzleFactor), "output"}},
{}, {},
// internal scalars
{Binding{b->getBitBlockType(), "buffer"},
Binding{b->getSizeTy(), "buffered"},
Binding{b->getSizeTy(), "sourceOffset"}})
, mSwizzleFactor(swizzleFactor) {

}

void PDEPkernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfBlocks) {
    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const processBlock = b->CreateBasicBlock("processBlock");
    BasicBlock * const finishedStrides = b->CreateBasicBlock("finishedStrides");
    const auto pdepWidth = b->getBitBlockWidth() / mSwizzleFactor;
    ConstantInt * const BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());
    ConstantInt * const PDEP_WIDTH = b->getSize(pdepWidth);

    Function * pdep = nullptr;
    if (pdepWidth == 64) {
        pdep = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pdep_64);
    } else if (pdepWidth == 32) {
        pdep = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pdep_32);
    } else {
        report_fatal_error(getName() + ": PDEP width must be 32 or 64");
    }

    // We store an internal source offset here because this kernel processes items in an unusual way.
    // The pipeline and multiblock assume that if we report we're on the i-th bit of a stream we have
    // fully processed all of the bits up to the i-th position.

    //                                             v
    //                |XXXXXXXXXXXX XXXXXXXXXXXX XX                       |
    //                |XXXXXXXXXXXX XXXXXXXXXXXX XX                       |
    //                |XXXXXXXXXXXX XXXXXXXXXXXX XX                       |
    //                |XXXXXXXXXXXX XXXXXXXXXXXX XX                       |

    // However, this kernel divides the stream into K elements and fully consumes a single stream of
    // the stream set before consuming the next one. So the same i-th position above is actually:

    //                |XXXXXXXXXXXX|XXXXXXXXXXXX|XXXXXXXXXXXX|XXXXXXXXXXXX|
    //                |XXXXXXXXXXXX|XXXXXXXXXXXX|XXXXXXXXXXXX|XXXXXXXXXXXX|
    //                |XX          |XX          |XX          |XX          |
    //                |            |            |            |            |

    // In the future, we may want the pipeline and multiblock to understand this style of processing
    // but for now, we hide it by delaying writing the actual processed offset until we've fully
    // processed the entire block.

    Value * const initialBuffer = b->getScalarField("buffer");
    Value * const initialBufferSize = b->getScalarField("buffered");
    Value * const initialSourceOffset = b->getScalarField("sourceOffset");
    b->CreateBr(processBlock);

    b->SetInsertPoint(processBlock);
    PHINode * const strideIndex = b->CreatePHI(b->getSizeTy(), 2);
    strideIndex->addIncoming(b->getSize(0), entry);
    PHINode * const bufferPhi = b->CreatePHI(initialBuffer->getType(), 2);
    bufferPhi->addIncoming(initialBuffer, entry);
    PHINode * const sourceOffsetPhi = b->CreatePHI(b->getSizeTy(), 2);
    sourceOffsetPhi->addIncoming(initialSourceOffset, entry);
    PHINode * const bufferSizePhi = b->CreatePHI(b->getSizeTy(), 2);
    bufferSizePhi->addIncoming(initialBufferSize, entry);

    // Extract the values we will use in the main processing loop
    Value * const markerStream = b->getInputStreamBlockPtr("marker", b->getInt32(0), strideIndex);
    Value * const selectors = b->fwCast(pdepWidth, b->CreateBlockAlignedLoad(markerStream));
    Value * const numOfSelectors = b->simd_popcount(pdepWidth, selectors);

    // If we run out of source items here, it is a failure of the MultiBlockKernel and/or PipelineGenerator
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * requiredSourceItems = b->CreateExtractElement(numOfSelectors, b->getInt64(0));
        for (unsigned i = 1; i < mSwizzleFactor; i++) {
            requiredSourceItems = b->CreateAdd(requiredSourceItems, b->CreateExtractElement(numOfSelectors, b->getInt64(i)));
        }
        Value * const availableSourceItems = b->getAvailableItemCount("source");
        Value * const unreadSourceItems = b->CreateSub(availableSourceItems, sourceOffsetPhi);
        Value * const hasSufficientSourceItems = b->CreateICmpULE(requiredSourceItems, unreadSourceItems);
        b->CreateAssert(hasSufficientSourceItems, getName() + " has insufficient source items for the given marker stream");
    }

    // For each element of the marker block
    Value * bufferSize = bufferSizePhi;
    Value * sourceOffset = sourceOffsetPhi;
    Value * buffer = bufferPhi;
    for (unsigned i = 0; i < mSwizzleFactor; i++) {

        // How many bits will we deposit?
        Value * const required = b->CreateExtractElement(numOfSelectors, b->getInt32(i));

        // Aggressively enqueue any additional bits
        BasicBlock * const entry = b->GetInsertBlock();
        BasicBlock * const enqueueBits = b->CreateBasicBlock();
        b->CreateBr(enqueueBits);

        b->SetInsertPoint(enqueueBits);
        PHINode * const bufferSize2 = b->CreatePHI(bufferSize->getType(), 2);
        bufferSize2->addIncoming(bufferSize, entry);
        PHINode * const sourceOffset2 = b->CreatePHI(sourceOffset->getType(), 2);
        sourceOffset2->addIncoming(sourceOffset, entry);
        PHINode * const buffer2 = b->CreatePHI(buffer->getType(), 2);
        buffer2->addIncoming(buffer, entry);

        // Calculate the block and swizzle index of the "current" swizzle
        Value * const block_index = b->CreateUDiv(sourceOffset2, BLOCK_WIDTH);
        Value * const stream_index = b->CreateUDiv(b->CreateURem(sourceOffset2, BLOCK_WIDTH), PDEP_WIDTH);
        Value * const ptr = b->getInputStreamBlockPtr("source", stream_index, block_index);
        Value * const swizzle = b->CreateBlockAlignedLoad(ptr);
        Value * const swizzleOffset = b->CreateURem(sourceOffset2, PDEP_WIDTH);

        // Shift the swizzle to the right to clear off any used bits ...
        Value * const swizzleShift = b->simd_fill(pdepWidth, swizzleOffset);
        Value * const unreadBits = b->CreateLShr(swizzle, swizzleShift);

        // ... then to the left to align the bits with the buffer and combine them.
        Value * const bufferShift = b->simd_fill(pdepWidth, bufferSize2);
        Value * const pendingBits = b->CreateShl(unreadBits, bufferShift);
        buffer = b->CreateOr(buffer, pendingBits);
        buffer2->addIncoming(buffer, enqueueBits);

        // Update the buffer size by the number of bits we have actually enqueued
        Value * const maxBufferSize = b->CreateAdd(b->CreateSub(PDEP_WIDTH, swizzleOffset), bufferSize2);
        bufferSize = b->CreateUMin(maxBufferSize, PDEP_WIDTH);
        // ... and increment the source offset by the number we actually inserted
        sourceOffset = b->CreateAdd(sourceOffset2, b->CreateSub(bufferSize, bufferSize2));
        bufferSize2->addIncoming(bufferSize, enqueueBits);
        sourceOffset2->addIncoming(sourceOffset, enqueueBits);
        BasicBlock * const depositBits = b->CreateBasicBlock();
        b->CreateCondBr(b->CreateICmpULT(bufferSize, required), enqueueBits, depositBits);

        b->SetInsertPoint(depositBits);
        // Apply PDEP to each element of the combined swizzle using the current PDEP mask
        Value * result = UndefValue::get(buffer->getType());
        Value * const mask = b->CreateExtractElement(selectors, i);
        for (unsigned j = 0; j < mSwizzleFactor; j++) {
            Value * source_field = b->CreateExtractElement(buffer, j);
            Value * PDEP_field = b->CreateCall(pdep, {source_field, mask});
            result = b->CreateInsertElement(result, PDEP_field, j);
        }

        // Store the result
        Value * const outputStreamPtr = b->getOutputStreamBlockPtr("output", b->getInt32(i), strideIndex);
        b->CreateBlockAlignedStore(result, outputStreamPtr);

        // Shift away any used bits from the buffer and decrement our buffer size by the number we used
        Value * const usedShift = b->simd_fill(pdepWidth, required);
        buffer = b->CreateLShr(buffer, usedShift);
        bufferSize = b->CreateSub(bufferSize, required);
    }

    BasicBlock * const finishedBlock = b->GetInsertBlock();
    sourceOffsetPhi->addIncoming(sourceOffset, finishedBlock);
    bufferSizePhi->addIncoming(bufferSize, finishedBlock);
    bufferPhi->addIncoming(buffer, finishedBlock);
    Value * const nextStrideIndex = b->CreateAdd(strideIndex, b->getSize(1));
    strideIndex->addIncoming(nextStrideIndex, finishedBlock);
    b->CreateLikelyCondBr(b->CreateICmpNE(nextStrideIndex, numOfBlocks), processBlock, finishedStrides);

    b->SetInsertPoint(finishedStrides);
    Value * const sourceItemsProcessed = b->CreateMul(b->CreateUDiv(sourceOffset, BLOCK_WIDTH), BLOCK_WIDTH);
    b->setProcessedItemCount("source", b->CreateAdd(b->getProcessedItemCount("source"), sourceItemsProcessed));
    b->setScalarField("buffer", buffer);
    b->setScalarField("buffered", bufferSize);
    b->setScalarField("sourceOffset", b->CreateURem(sourceOffset, BLOCK_WIDTH));
}

}
