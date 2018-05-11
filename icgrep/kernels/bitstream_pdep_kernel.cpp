/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "bitstream_pdep_kernel.h"
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>
#include <toolchain/toolchain.h>

using namespace llvm;

namespace kernel {

    BitStreamPDEPKernel::BitStreamPDEPKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const unsigned numberOfStream, std::string name)
            : MultiBlockKernel(std::move(name),
// input stream sets
                               {Binding{b->getStreamSetTy(), "marker", FixedRate(), Principal()},
                                Binding{b->getStreamSetTy(numberOfStream), "source", PopcountOf("marker")}},
// output stream set
                               {Binding{b->getStreamSetTy(numberOfStream), "output", FixedRate()}},
                               {}, {}, {}),
              mNumberOfStream(numberOfStream)
             {

    }

    void BitStreamPDEPKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfBlocks) {
        BasicBlock * const entry = b->GetInsertBlock();
        BasicBlock * const processBlock = b->CreateBasicBlock("processBlock");
        BasicBlock * const finishedStrides = b->CreateBasicBlock("finishedStrides");
        const auto pdepWidth = 64; // TODO handle 32 bit machine
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

        Constant * const ZERO = b->getSize(0);
        Value * const sourceItemCount = b->getProcessedItemCount("source");

        Value * const initialSourceOffset = b->CreateURem(sourceItemCount, BLOCK_WIDTH);
        b->CreateBr(processBlock);

        b->SetInsertPoint(processBlock);
        PHINode * const strideIndex = b->CreatePHI(b->getSizeTy(), 2);
        strideIndex->addIncoming(ZERO, entry);

        std::vector<PHINode*> bufferPhiArray(mNumberOfStream, NULL);
        std::vector<Value*> bufferArray(mNumberOfStream, NULL);
        for (int iStreamIndex = 0; iStreamIndex < mNumberOfStream; iStreamIndex++) {
            PHINode * const bufferPhi = b->CreatePHI(b->getIntNTy(pdepWidth), 2);
            bufferPhi->addIncoming(Constant::getNullValue(b->getIntNTy(pdepWidth)), entry);
            bufferPhiArray[iStreamIndex] = bufferPhi;
            bufferArray[iStreamIndex] = bufferPhi;
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


        std::vector<llvm::Value*> resultArray(mNumberOfStream, UndefValue::get(b->getBitBlockType()));
        /**
         * TODO Assumed that the bitblocktype is always <4 * i64>
         *                    The i < 4 here comes from  ^
         */
        for (unsigned i = 0; i < 4; i++) {

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

            std::vector<PHINode * > updatedBufferArray(mNumberOfStream, NULL);
            for (int iStreamIndex = 0; iStreamIndex < mNumberOfStream; iStreamIndex++) {
                Value* buffer = bufferArray[iStreamIndex];
                PHINode * const updatedBuffer = b->CreatePHI(buffer->getType(), 2);
                updatedBuffer->addIncoming(buffer, entry);
                updatedBufferArray[iStreamIndex] = updatedBuffer;
            }

            // Calculate the block and swizzle index of the current swizzle row
            Value * const blockOffset = b->CreateUDiv(updatedSourceOffset, BLOCK_WIDTH);
            Value * const swizzleIndex = b->CreateUDiv(b->CreateURem(updatedSourceOffset, BLOCK_WIDTH), PDEP_WIDTH);

            Value * const swizzleOffset = b->CreateURem(updatedSourceOffset, PDEP_WIDTH);

            for (int iStreamIndex = 0; iStreamIndex < mNumberOfStream; iStreamIndex++) {
                // TODO improve the performance of these load instructions by gather
                Value * const swizzleBlock = b->CreateBlockAlignedLoad(b->getInputStreamBlockPtr("source", b->getSize(iStreamIndex), blockOffset));

                Value * const swizzle = b->CreateExtractElement(swizzleBlock, swizzleIndex);
                // Shift the swizzle to the right to clear off any used bits ...
                Value * const unreadBits = b->CreateSelect(b->CreateICmpEQ(swizzleOffset, b->getSize(0x40)), b->getInt64(0), b->CreateLShr(swizzle, swizzleOffset));
                // ... then to the left to align the bits with the buffer and combine them.
                Value * const pendingBits = b->CreateSelect(b->CreateICmpEQ(updatedBufferSize, b->getSize(0x40)), b->getInt64(0), b->CreateShl(unreadBits, updatedBufferSize));

                bufferArray[iStreamIndex] = b->CreateOr(updatedBufferArray[iStreamIndex], pendingBits);
                updatedBufferArray[iStreamIndex]->addIncoming(bufferArray[iStreamIndex], enqueueBits);
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
            for (int iStreamIndex = 0; iStreamIndex < mNumberOfStream; iStreamIndex++) {
                Value * source_field = bufferArray[iStreamIndex];

                Value * PDEP_field = b->CreateCall(pdep, {source_field, mask});

                resultArray[iStreamIndex] = b->CreateInsertElement(resultArray[iStreamIndex], PDEP_field, i);

                // Shift away any used bits from the buffer and decrement our buffer size by the number we used
                bufferArray[iStreamIndex] = b->CreateSelect(b->CreateICmpEQ(required, b->getInt64(0x40)), b->getInt64(0), b->CreateLShr(bufferArray[iStreamIndex], required));
            }

            bufferSize = b->CreateSub(bufferSize, required);
        }

        for (int iStreamIndex = 0; iStreamIndex < mNumberOfStream; iStreamIndex++) {
            // Store the result
            Value * const outputStreamPtr = b->getOutputStreamBlockPtr("output", b->getSize(iStreamIndex), strideIndex);
            b->CreateBlockAlignedStore(resultArray[iStreamIndex], outputStreamPtr);
        }

        BasicBlock * const finishedBlock = b->GetInsertBlock();
        sourceOffsetPhi->addIncoming(sourceOffset, finishedBlock);
        bufferSizePhi->addIncoming(bufferSize, finishedBlock);
        for (int iStreamIndex = 0; iStreamIndex < mNumberOfStream; iStreamIndex++) {
            bufferPhiArray[iStreamIndex]->addIncoming(bufferArray[iStreamIndex], finishedBlock);
        }

        Value * const nextStrideIndex = b->CreateAdd(strideIndex, b->getSize(1));
        strideIndex->addIncoming(nextStrideIndex, finishedBlock);
        b->CreateLikelyCondBr(b->CreateICmpNE(nextStrideIndex, numOfBlocks), processBlock, finishedStrides);

        b->SetInsertPoint(finishedStrides);
    }

}
