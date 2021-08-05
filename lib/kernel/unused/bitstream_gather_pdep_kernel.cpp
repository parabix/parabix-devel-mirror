
#include <kernel/util/bitstream_gather_pdep_kernel.h>
#include <kernel/core/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>
#include <toolchain/toolchain.h>
#include <llvm/IR/Intrinsics.h>

using namespace llvm;

namespace kernel {

    BitStreamGatherPDEPKernel::BitStreamGatherPDEPKernel(BuilderRef b, const unsigned numberOfStream, std::string name)
            : MultiBlockKernel(b, std::move(name),
// input stream sets
                               {Binding{b->getStreamSetTy(), "marker", FixedRate(), Principal()},
                                Binding{b->getStreamSetTy(numberOfStream), "source", PopcountOf("marker")}},
// output stream set
                               {Binding{b->getStreamSetTy(numberOfStream), "output", FixedRate()}},
                               {}, {}, {}),
              mNumberOfStream(numberOfStream)
             {

    }

    void BitStreamGatherPDEPKernel::generateMultiBlockLogic(BuilderRef b, Value * const numOfBlocks) {
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

        std::vector<PHINode*> bufferVecPhiArray(mNumberOfStream / 4, NULL);
        std::vector<Value*> bufferVecArray(mNumberOfStream / 4, NULL);
        for (unsigned iStreamSetIndex = 0; iStreamSetIndex < mNumberOfStream; iStreamSetIndex += 4) {
            PHINode * const bufferPhi = b->CreatePHI(b->getBitBlockType(), 2, "bufferPhi");
            bufferPhi->addIncoming(Constant::getNullValue(b->getBitBlockType()), entry);
            bufferVecPhiArray[iStreamSetIndex / 4] = bufferPhi;
            bufferVecArray[iStreamSetIndex / 4] = bufferPhi;
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

            std::vector<PHINode * > updatedBufferVecArray(mNumberOfStream / 4, NULL);
            for (unsigned iStreamSetIndex = 0; iStreamSetIndex < mNumberOfStream; iStreamSetIndex += 4) {
                Value* buffer = bufferVecArray[iStreamSetIndex / 4];
                PHINode * const updatedBuffer = b->CreatePHI(buffer->getType(), 2);
                updatedBuffer->addIncoming(buffer, entry);
                updatedBufferVecArray[iStreamSetIndex / 4] = updatedBuffer;
            }

            // Calculate the block and swizzle index of the current swizzle row
            Value * const blockOffset = b->CreateUDiv(updatedSourceOffset, BLOCK_WIDTH);
            Value * const swizzleIndex = b->CreateUDiv(b->CreateURem(updatedSourceOffset, BLOCK_WIDTH), PDEP_WIDTH);

            Value * const swizzleOffset = b->CreateURem(updatedSourceOffset, PDEP_WIDTH);


            for (unsigned iStreamSetIndex = 0; iStreamSetIndex < mNumberOfStream; iStreamSetIndex += 4) {
                // gather instruction can process 4 streams each time
                Value *streamSetBlockBasePtr = b->getInputStreamBlockPtr("source", b->getSize(iStreamSetIndex),
                                                                         blockOffset);

                Function *gatherFunc = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_avx2_gather_d_q_256);
                Value *addresses = ConstantVector::get(
                        {b->getInt32(0), b->getInt32(32), b->getInt32(64), b->getInt32(96)});

                Value *nullAddress = fill_address(b, 32, 4, b->CreateMul(b->CreateTrunc(swizzleIndex, b->getInt32Ty()),
                                                                        b->getInt32(8)));

                addresses = b->CreateAdd(addresses, nullAddress);

                Value *gather_result = b->CreateCall(
                                                     gatherFunc->getFunctionType(),
                                                     gatherFunc,
                        {
                                UndefValue::get(b->getBitBlockType()),
                                b->CreatePointerCast(streamSetBlockBasePtr, b->getInt8PtrTy()),
                                addresses,
                                Constant::getAllOnesValue(b->getBitBlockType()),
                                b->getInt8(1)
                        }
                );
                // Shift the swizzle to the right to clear off any used bits ...
                Value* unreadBitsVec = b->CreateLShr(gather_result, b->simd_fill(64, swizzleOffset));
                // ... then to the left to align the bits with the buffer and combine them.
                Value* pendingBitsVec = b->CreateShl(unreadBitsVec, b->simd_fill(64, updatedBufferSize));

                bufferVecArray[iStreamSetIndex / 4] = b->CreateOr(updatedBufferVecArray[iStreamSetIndex / 4], pendingBitsVec);
                updatedBufferVecArray[iStreamSetIndex / 4]->addIncoming(bufferVecArray[iStreamSetIndex / 4], enqueueBits);
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

            for (unsigned iStreamSetIndex = 0; iStreamSetIndex < mNumberOfStream; iStreamSetIndex += 4) {
                Value * source_field = bufferVecArray[iStreamSetIndex / 4];
                for (unsigned iStreamIndex = iStreamSetIndex; iStreamIndex < iStreamSetIndex + 4; iStreamIndex++) {
                    Value * PDEP_field = b->CreateCall(pdep->getFunctionType(), pdep, {b->CreateExtractElement(source_field, iStreamIndex - iStreamSetIndex), mask});
                    resultArray[iStreamIndex] = b->CreateInsertElement(resultArray[iStreamIndex], PDEP_field, i);
                }
                bufferVecArray[iStreamSetIndex / 4] = b->CreateLShr(bufferVecArray[iStreamSetIndex / 4], b->simd_fill(64, required));
            }

            bufferSize = b->CreateSub(bufferSize, required);
        }

        for (unsigned iStreamIndex = 0; iStreamIndex < mNumberOfStream; iStreamIndex++) {
            // Store the result
            Value * const outputStreamPtr = b->getOutputStreamBlockPtr("output", b->getSize(iStreamIndex), strideIndex);
            b->CreateBlockAlignedStore(resultArray[iStreamIndex], outputStreamPtr);
        }

        BasicBlock * const finishedBlock = b->GetInsertBlock();
        sourceOffsetPhi->addIncoming(sourceOffset, finishedBlock);
        bufferSizePhi->addIncoming(bufferSize, finishedBlock);

        for (unsigned iStreamSetIndex = 0; iStreamSetIndex < mNumberOfStream; iStreamSetIndex += 4) {
            bufferVecPhiArray[iStreamSetIndex / 4]->addIncoming(bufferVecArray[iStreamSetIndex / 4], finishedBlock);
        }

        Value * const nextStrideIndex = b->CreateAdd(strideIndex, b->getSize(1));
        strideIndex->addIncoming(nextStrideIndex, finishedBlock);
        b->CreateLikelyCondBr(b->CreateICmpNE(nextStrideIndex, numOfBlocks), processBlock, finishedStrides);

        b->SetInsertPoint(finishedStrides);
    }

    llvm::Value* BitStreamGatherPDEPKernel::fill_address(BuilderRef b, unsigned fw, unsigned field_count, Value * a) {
        Type * singleFieldVecTy = VectorType::get(b->getIntNTy(fw), 1);
        Value * aVec = b->CreateBitCast(a, singleFieldVecTy);
        return b->CreateShuffleVector(aVec, UndefValue::get(singleFieldVecTy), Constant::getNullValue(VectorType::get(b->getInt32Ty(), field_count)));
    }
}
