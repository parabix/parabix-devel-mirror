//
// Created by wxy325 on 2018/2/9.
//

#include "lz4_multiple_pdep_kernel.h"
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>
#include <iostream>
#include <vector>


using namespace llvm;


namespace kernel {

    LZ4MultiplePDEPkernel::LZ4MultiplePDEPkernel(const std::unique_ptr<kernel::KernelBuilder> & kb, unsigned streamCount, unsigned streamSize, unsigned swizzleFactor, unsigned PDEP_width, std::string name)
            : MultiBlockKernel(name + "",
                               {Binding{kb->getStreamSetTy(), "PDEPmarkerStream", BoundedRate(0, 1)}},
                               {},
                               {}, {}, {})
            , mSwizzleFactor(swizzleFactor)
            , mPDEPWidth(PDEP_width)
            , mStreamSize(streamSize)
    {
        assert((mSwizzleFactor == (kb->getBitBlockWidth() / PDEP_width)) && "swizzle factor must equal bitBlockWidth / PDEP_width");
        assert((mPDEPWidth == 64 || mPDEPWidth == 32) && "PDEP width must be 32 or 64");

        mStreamSetInputs.push_back(Binding{kb->getStreamSetTy(streamCount), "sourceStreamSet0", BoundedRate(0, 1), Swizzled()});
        mStreamSetOutputs.push_back(Binding{kb->getStreamSetTy(streamCount), "outputStreamSet0", RateEqualTo("PDEPmarkerStream")});

        for (int i = 1; i < streamSize; i++) {
            mStreamSetInputs.push_back(Binding{kb->getStreamSetTy(streamCount), "sourceStreamSet" + std::to_string(i), BoundedRate(0, 1), Swizzled()});
            mStreamSetOutputs.push_back(Binding{kb->getStreamSetTy(streamCount), "outputStreamSet" + std::to_string(i), RateEqualTo("outputStreamSet0")});
        }
    }

    void LZ4MultiplePDEPkernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, Value * const numOfStrides) {
        BasicBlock * entry = kb->GetInsertBlock();
//        kb->CallPrintInt("--------------" + this->getName() + " doMultiBlock Start:", kb->getSize(0));
        BasicBlock * checkLoopCond = kb->CreateBasicBlock("checkLoopCond");
        BasicBlock * checkSourceCount = kb->CreateBasicBlock("checkSourceCount");
        BasicBlock * processBlock = kb->CreateBasicBlock("processBlock");
        BasicBlock * terminate = kb->CreateBasicBlock("terminate");

        Value * itemsToDo = mAvailableItemCount[0];
        Value * sourceItemsAvail = mAvailableItemCount[1]; //TODO need to be calculated from numOfStrides

        Constant * blockWidth = kb->getSize(kb->getBitBlockWidth());
        Value * blocksToDo = kb->CreateSelect(mIsFinal, kb->CreateUDivCeil(itemsToDo, blockWidth), kb->CreateUDiv(itemsToDo, blockWidth));
        Value * processedSourceBits = kb->getProcessedItemCount("sourceStreamSet0");
        Value * base_src_blk_idx = kb->CreateUDiv(processedSourceBits, blockWidth);

        Value * pdepWidth = kb->getSize(mPDEPWidth);
        Value * pdepWidth_1 = kb->getSize(mPDEPWidth - 1);
        Value * PDEP_func = nullptr;
        if (mPDEPWidth == 64) {
            PDEP_func = Intrinsic::getDeclaration(kb->getModule(), Intrinsic::x86_bmi_pdep_64);
        } else if (mPDEPWidth == 32) {
            PDEP_func = Intrinsic::getDeclaration(kb->getModule(), Intrinsic::x86_bmi_pdep_32);
        }
        kb->CreateBr(checkLoopCond);

        kb->SetInsertPoint(checkLoopCond);
        // The following PHINodes' values can come from entry or processBlock
        PHINode * blocksToDoPhi = kb->CreatePHI(kb->getSizeTy(), 2);
        PHINode * blockOffsetPhi = kb->CreatePHI(kb->getSizeTy(), 2); // block offset from the base block, e.g. 0, 1, 2, ...
        PHINode * updatedProcessedSourceBitsPhi = kb->CreatePHI(kb->getSizeTy(), 2);
        PHINode * sourceItemsRemaining = kb->CreatePHI(kb->getSizeTy(), 2);
        blocksToDoPhi->addIncoming(blocksToDo, entry);
        blockOffsetPhi->addIncoming(kb->getSize(0), entry);
        updatedProcessedSourceBitsPhi->addIncoming(processedSourceBits, entry);
        sourceItemsRemaining->addIncoming(sourceItemsAvail, entry);

        Value * haveRemBlocks = kb->CreateICmpUGT(blocksToDoPhi, kb->getSize(0));
        kb->CreateCondBr(haveRemBlocks, checkSourceCount, terminate);

        kb->SetInsertPoint(checkSourceCount);
        // Extract the values we will use in the main processing loop
        Value * updatedProcessedSourceBits = updatedProcessedSourceBitsPhi;
        Value * updatedSourceItems = sourceItemsRemaining;
        Value * PDEP_ms_blk = kb->CreateBlockAlignedLoad(kb->getInputStreamBlockPtr("PDEPmarkerStream", kb->getInt32(0), blockOffsetPhi));

        const auto PDEP_masks = get_PDEP_masks(kb, PDEP_ms_blk, mPDEPWidth);
        const auto mask_popcounts = get_block_popcounts(kb, PDEP_ms_blk, mPDEPWidth);

        Value * total_count = mask_popcounts[0];
        for (unsigned j = 1; j < mask_popcounts.size(); j++) {
            total_count = kb->CreateAdd(total_count, mask_popcounts[j]);
        }
//    kb->CallPrintInt("total_count", total_count);
//    kb->CallPrintInt("sourceItemsRemaining", sourceItemsRemaining);
        // Do not check popcount in final block, since there may be some useless pdep marker in the end
        kb->CreateCondBr(kb->CreateOr(kb->CreateICmpULE(total_count, sourceItemsRemaining), mIsFinal), processBlock, terminate);
        kb->SetInsertPoint(processBlock);

        // For each mask extracted from the PDEP marker block
        for (unsigned i = 0; i < mSwizzleFactor; i++) {
            // Do block and swizzle index calculations, then combine the "current" and "next" swizzles

            Value * current_blk_idx = kb->CreateSub(kb->CreateUDiv(updatedProcessedSourceBits, blockWidth), base_src_blk_idx); // blk index == stream set block index
            Value * current_swizzle_idx = kb->CreateUDiv(kb->CreateURem(updatedProcessedSourceBits, blockWidth), pdepWidth);
            Value * ahead_pdep_width_less_1 = kb->CreateAdd(pdepWidth_1, updatedProcessedSourceBits);


            Value * shift_amount = kb->CreateURem(updatedProcessedSourceBits, pdepWidth);

            for (int iStreamIndex = 0; iStreamIndex < mStreamSize; iStreamIndex++) {
                Value * next_blk_idx = kb->CreateSub(kb->CreateUDiv(ahead_pdep_width_less_1, blockWidth), base_src_blk_idx);
                Value * next_swizzle_idx = kb->CreateUDiv(kb->CreateURem(ahead_pdep_width_less_1, blockWidth), pdepWidth);

                // Load current and next BitBlocks/swizzles
                Value* current_swizzle_ptr = kb->getInputStreamBlockPtr("sourceStreamSet" + std::to_string(iStreamIndex), current_swizzle_idx, current_blk_idx);

                Value * current_swizzle = kb->CreateBlockAlignedLoad(current_swizzle_ptr);//Constant::getNullValue(cast<PointerType>(current_swizzle_ptr->getType())->getElementType());


                Value* next_swizzle_ptr = kb->getInputStreamBlockPtr("sourceStreamSet" + std::to_string(iStreamIndex), next_swizzle_idx, next_blk_idx);
                Value * next_swizzle = kb->CreateBlockAlignedLoad(next_swizzle_ptr);//Constant::getNullValue(cast<PointerType>(current_swizzle_ptr->getType())->getElementType());

                // Combine the two swizzles to guarantee we'll have enough source bits for the PDEP operation
                Value * remaining_bits = kb->CreateLShr(current_swizzle, kb->simd_fill(mPDEPWidth, shift_amount)); // shift away bits that have already been used

                Value * borrowed_bits = kb->CreateShl(next_swizzle,
                                                      kb->simd_fill(mPDEPWidth, kb->CreateSub(pdepWidth, shift_amount))); // shift next swizzle left by width of first swizzle
                Value * combined = kb->CreateOr(remaining_bits, borrowed_bits); // combine current swizzle and next swizzle

                Value * segments = kb->fwCast(mPDEPWidth, combined);

//                kb->CallPrintInt("current_swizzle_idx", current_swizzle_idx);
//                kb->CallPrintInt("next_swizzle_idx", next_swizzle_idx);
//                if (iStreamIndex == 1) {
//                    kb->CallPrintInt("current_swizzle_ptr"  + std::to_string(iStreamIndex) , current_swizzle_ptr);
//                    kb->CallPrintRegister("current_" + std::to_string(iStreamIndex) + "_" + std::to_string(i), current_swizzle);
//
//                    kb->CallPrintRegister("next_" + std::to_string(iStreamIndex) + "_" + std::to_string(i), next_swizzle);
//                    kb->CallPrintRegister("segments_" + std::to_string(iStreamIndex) + "_" + std::to_string(i), segments);
//                }

                Value * result_swizzle = Constant::getNullValue(segments->getType());
                // Apply PDEP to each mPDEPWidth segment of the combined swizzle using the current PDEP mask

                Value * PDEP_mask = PDEP_masks[i];
                for (unsigned j = 0; j < mSwizzleFactor; j++) {
                    Value * source_field = kb->CreateExtractElement(segments, j);
                    Value * PDEP_field = kb->CreateCall(PDEP_func, {source_field, PDEP_mask});
                    result_swizzle = kb->CreateInsertElement(result_swizzle, PDEP_field, j);

                }

                // Store the result
                Value* outputPos = kb->getOutputStreamBlockPtr("outputStreamSet" + std::to_string(iStreamIndex), kb->getSize(i), blockOffsetPhi);

                kb->CreateBlockAlignedStore(result_swizzle, outputPos);
            }

            updatedProcessedSourceBits = kb->CreateAdd(updatedProcessedSourceBits, mask_popcounts[i]);
            updatedSourceItems = kb->CreateSub(updatedSourceItems, mask_popcounts[i]);
        }

        updatedProcessedSourceBitsPhi->addIncoming(updatedProcessedSourceBits, processBlock);
        blocksToDoPhi->addIncoming(kb->CreateSub(blocksToDoPhi, kb->getSize(1)), processBlock);
        blockOffsetPhi->addIncoming(kb->CreateAdd(blockOffsetPhi, kb->getSize(1)), processBlock);
        sourceItemsRemaining->addIncoming(updatedSourceItems, processBlock);
        kb->CreateBr(checkLoopCond);

        kb->SetInsertPoint(terminate);
        for (int i = 0; i < mStreamSize; i++) {
            kb->setProcessedItemCount("sourceStreamSet" + std::to_string(i), updatedProcessedSourceBitsPhi);
        }

        Value* processedBlock = kb->CreateSub(blocksToDo, blocksToDoPhi);
//        kb->CallPrintInt("blocksToDoPhi", blocksToDoPhi);

        kb->setProcessedItemCount("PDEPmarkerStream",
                                  kb->CreateSelect(mIsFinal,
                                                   kb->CreateAdd(kb->getProcessedItemCount("PDEPmarkerStream"), itemsToDo),
                                                   kb->CreateAdd(kb->getProcessedItemCount("PDEPmarkerStream"),kb->CreateMul(processedBlock, blockWidth))
                                  )
        );

    }

    std::vector<Value *> LZ4MultiplePDEPkernel::get_block_popcounts(const std::unique_ptr<KernelBuilder> & kb, Value * blk, const unsigned field_width) {
        Value * pop_counts = kb->simd_popcount(field_width, blk);
        std::vector<Value *> counts;
        for (unsigned i = 0; i < kb->getBitBlockWidth() / field_width ; i++) {
            // Store the pop counts for each blk_width field in blk
            counts.push_back(kb->CreateExtractElement(pop_counts, i)); // Extract field i from SIMD register popCounts
        }
        return counts;
    }

    std::vector<Value *> LZ4MultiplePDEPkernel::get_PDEP_masks(const std::unique_ptr<KernelBuilder> & kb, Value * PDEP_ms_blk, const unsigned mask_width) {
        // We apply the PDEP operation mPDEPWidth bits at a time (e.g. if block is 256 bits and mPDEPWidth is 64, apply 4 PDEP ops to full process swizzle).
        // Split the PDEP marker stream block into mPDEPWidth segments.
        Value * masks = kb->fwCast(mask_width, PDEP_ms_blk);
        std::vector<Value *> PDEP_masks;
        for (unsigned i = 0; i < kb->getBitBlockWidth() / mask_width; i++) {
            PDEP_masks.push_back(kb->CreateExtractElement(masks, i));
        }
        return PDEP_masks;
    }
}
