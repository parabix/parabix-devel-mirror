/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "pdep_kernel.h"
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>
#include <iostream>

using namespace llvm;

namespace kernel {

PDEPkernel::PDEPkernel(const std::unique_ptr<kernel::KernelBuilder> & kb, unsigned streamCount, unsigned swizzleFactor, unsigned PDEP_width, std::string name)
: MultiBlockKernel(name + "",
                  {Binding{kb->getStreamSetTy(), "PDEPmarkerStream", BoundedRate(0, 1)},
                   Binding{kb->getStreamSetTy(streamCount), "sourceStreamSet", BoundedRate(0, 1)}},
                  {Binding{kb->getStreamSetTy(streamCount), "outputStreamSet", RateEqualTo("PDEPmarkerStream")}},
                  {}, {}, {})
, mSwizzleFactor(swizzleFactor)
, mPDEPWidth(PDEP_width)
{
    assert((mSwizzleFactor == (kb->getBitBlockWidth() / PDEP_width)) && "swizzle factor must equal bitBlockWidth / PDEP_width");
    assert((mPDEPWidth == 64 || mPDEPWidth == 32) && "PDEP width must be 32 or 64");
}

void PDEPkernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, Value * const numOfStrides) {
    BasicBlock * entry = kb->GetInsertBlock();
//    kb->CallPrintInt("--------------" + this->getName() + " doMultiBlock Start:", kb->getSize(0));
    BasicBlock * checkLoopCond = kb->CreateBasicBlock("checkLoopCond");
    BasicBlock * checkSourceCount = kb->CreateBasicBlock("checkSourceCount");
    BasicBlock * processBlock = kb->CreateBasicBlock("processBlock");
    BasicBlock * terminate = kb->CreateBasicBlock("terminate");

    Value * itemsToDo = mAvailableItemCount[0];

    Value * sourceItemsAvail = mAvailableItemCount[1];
//    kb->CallPrintInt("itemsToDo:", itemsToDo);
//    kb->CallPrintInt("sourceItemsAvail:", sourceItemsAvail);


    Value * PDEPStrmPtr = kb->getInputStreamBlockPtr("PDEPmarkerStream", kb->getInt32(0)); // mStreamBufferPtr[0];
    Value * inputSwizzlesPtr = kb->getInputStreamBlockPtr("sourceStreamSet", kb->getInt32(0)); // mStreamBufferPtr[1];
    // Get pointer to start of the output StreamSetBlock we're currently writing to
    Value * outputStreamPtr = kb->getOutputStreamBlockPtr("outputStreamSet", kb->getInt32(0)); // mStreamBufferPtr[2];

//    kb->CallPrintInt("aaa", outputStreamPtr)

    Constant * blockWidth = kb->getSize(kb->getBitBlockWidth()); // 256
    Value * blocksToDo = kb->CreateUDivCeil(itemsToDo, blockWidth); // 1 if this is the final block TODO the assumption is incorrect here
    Value * processedSourceBits = kb->getProcessedItemCount("sourceStreamSet");
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
    Value * PDEP_ms_blk = kb->CreateBlockAlignedLoad(kb->CreateGEP(PDEPStrmPtr, blockOffsetPhi));

    const auto PDEP_masks = get_PDEP_masks(kb, PDEP_ms_blk, mPDEPWidth);    
    const auto mask_popcounts = get_block_popcounts(kb, PDEP_ms_blk, mPDEPWidth);
    
    Value * total_count = mask_popcounts[0];
    for (unsigned j = 1; j < mask_popcounts.size(); j++) {
        total_count = kb->CreateAdd(total_count, mask_popcounts[j]);
    }
//    kb->CallPrintInt("total_count", total_count);
//    kb->CallPrintInt("sourceItemsRemaining", sourceItemsRemaining);
    kb->CreateCondBr(kb->CreateICmpULE(total_count, sourceItemsRemaining), processBlock, terminate);
    kb->SetInsertPoint(processBlock);

    // For each mask extracted from the PDEP marker block
    for (unsigned i = 0; i < mSwizzleFactor; i++) {
        // Do block and swizzle index calculations, then combine the "current" and "next" swizzles

        Value * current_blk_idx = kb->CreateSub(kb->CreateUDiv(updatedProcessedSourceBits, blockWidth), base_src_blk_idx); // blk index == stream set block index
        Value * current_swizzle_idx = kb->CreateUDiv(kb->CreateURem(updatedProcessedSourceBits, blockWidth), pdepWidth);
        Value * ahead_pdep_width_less_1 = kb->CreateAdd(pdepWidth_1, updatedProcessedSourceBits);

        Value * next_blk_idx = kb->CreateSub(kb->CreateUDiv(ahead_pdep_width_less_1, blockWidth), base_src_blk_idx);
        Value * next_swizzle_idx = kb->CreateUDiv(kb->CreateURem(ahead_pdep_width_less_1, blockWidth), pdepWidth);

//        kb->CallPrintInt("current_blk_idx", current_blk_idx);
//        kb->CallPrintInt("current_swizzle_idx", current_swizzle_idx);

        // Load current and next BitBlocks/swizzles
        // TODO can not guarantee the two GEP is correct, need to check later
        Value * current_swizzle_ptr = kb->CreateGEP(inputSwizzlesPtr, kb->CreateAdd(kb->CreateMul(current_blk_idx, kb->getSize(mSwizzleFactor)), current_swizzle_idx));
        Value * next_swizzle_ptr = kb->CreateGEP(inputSwizzlesPtr, kb->CreateAdd(kb->CreateMul(next_blk_idx, kb->getSize(mSwizzleFactor)), next_swizzle_idx));

        Value * current_swizzle = kb->CreateBlockAlignedLoad(current_swizzle_ptr);//Constant::getNullValue(cast<PointerType>(current_swizzle_ptr->getType())->getElementType());
        Value * next_swizzle = kb->CreateBlockAlignedLoad(next_swizzle_ptr);//Constant::getNullValue(cast<PointerType>(current_swizzle_ptr->getType())->getElementType());
//        kb->CallPrintInt("ptr", current_swizzle_ptr);
//        kb->CallPrintRegister("current_swizzle", current_swizzle);

        // Combine the two swizzles to guarantee we'll have enough source bits for the PDEP operation
        Value * shift_amount = kb->CreateURem(updatedProcessedSourceBits, pdepWidth);
        Value * remaining_bits = kb->CreateLShr(current_swizzle, kb->simd_fill(mPDEPWidth, shift_amount)); // shift away bits that have already been used

        Value * borrowed_bits = kb->CreateShl(next_swizzle,
                                              kb->simd_fill(mPDEPWidth, kb->CreateSub(pdepWidth, shift_amount))); // shift next swizzle left by width of first swizzle
        Value * combined = kb->CreateOr(remaining_bits, borrowed_bits); // combine current swizzle and next swizzle

        Value * segments = kb->fwCast(mPDEPWidth, combined);
        Value * result_swizzle = Constant::getNullValue(segments->getType());
        // Apply PDEP to each mPDEPWidth segment of the combined swizzle using the current PDEP mask




        Value * PDEP_mask = PDEP_masks[i];
        for (unsigned j = 0; j < mSwizzleFactor; j++) {
            Value * source_field = kb->CreateExtractElement(segments, j);
            Value * PDEP_field = kb->CreateCall(PDEP_func, {source_field, PDEP_mask});
            result_swizzle = kb->CreateInsertElement(result_swizzle, PDEP_field, j); 
        }

        // Store the result
        auto outputPos = kb->CreateGEP(outputStreamPtr, kb->CreateAdd(kb->CreateMul(blockOffsetPhi, kb->getSize(mSwizzleFactor)), kb->getSize(i)));
        kb->CreateBlockAlignedStore(result_swizzle, outputPos);
        updatedProcessedSourceBits = kb->CreateAdd(updatedProcessedSourceBits, mask_popcounts[i]);
        updatedSourceItems = kb->CreateSub(updatedSourceItems, mask_popcounts[i]);
    }

    updatedProcessedSourceBitsPhi->addIncoming(updatedProcessedSourceBits, processBlock);
    blocksToDoPhi->addIncoming(kb->CreateSub(blocksToDoPhi, kb->getSize(1)), processBlock);
    blockOffsetPhi->addIncoming(kb->CreateAdd(blockOffsetPhi, kb->getSize(1)), processBlock);
    sourceItemsRemaining->addIncoming(updatedSourceItems, processBlock);
    kb->CreateBr(checkLoopCond);

    kb->SetInsertPoint(terminate);
    Value * itemsDone = kb->CreateMul(blockOffsetPhi, blockWidth);
    itemsDone = kb->CreateSelect(kb->CreateICmpULT(itemsToDo, itemsDone), itemsToDo, itemsDone);
    kb->setProcessedItemCount("PDEPmarkerStream", kb->CreateAdd(itemsDone, kb->getProcessedItemCount("PDEPmarkerStream")));
    kb->setProcessedItemCount("sourceStreamSet", updatedProcessedSourceBitsPhi);


//    kb->CallPrintInt("itemsDone:", itemsDone);
//    kb->CallPrintInt("produced:", kb->getProducedItemCount("outputStreamSet"));
//    kb->CallPrintInt("--------------" + this->getName() + " doMultiBlock End:", kb->getSize(0));
}

std::vector<Value *> PDEPkernel::get_block_popcounts(const std::unique_ptr<KernelBuilder> & kb, Value * blk, const unsigned field_width) {
    Value * pop_counts = kb->simd_popcount(field_width, blk);
    std::vector<Value *> counts;
    for (unsigned i = 0; i < kb->getBitBlockWidth() / field_width ; i++) {
    	// Store the pop counts for each blk_width field in blk
        counts.push_back(kb->CreateExtractElement(pop_counts, i)); // Extract field i from SIMD register popCounts
    }
    return counts;
}

std::vector<Value *> PDEPkernel::get_PDEP_masks(const std::unique_ptr<KernelBuilder> & kb, Value * PDEP_ms_blk, const unsigned mask_width) {
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
