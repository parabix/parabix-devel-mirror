/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "pdep_kernel.h"
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace kernel {

PDEPkernel::PDEPkernel(const std::unique_ptr<kernel::KernelBuilder> & kb, unsigned streamCount, unsigned PDEP_width)
: BlockOrientedKernel("PDEPdel",
                  {Binding{kb->getStreamSetTy(), "PDEPmarkerStream"}, Binding{kb->getStreamSetTy(streamCount), "sourceStreamSet", MaxRatio(1)}},
                  {Binding{kb->getStreamSetTy(streamCount), "outputStreamSet"}}, {}, {}, {})
, mSwizzleFactor(kb->getBitBlockWidth() / PDEP_width)
, mPDEPWidth(PDEP_width)
{
    assert((mPDEPWidth == 64 || mPDEPWidth == 32) && "PDEP width must be 32 or 64");
}

void PDEPkernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & kb) {
    // Extract the values we will use in the main processing loop
    Value * PDEP_ms_blk = kb->loadInputStreamBlock("PDEPmarkerStream", kb->getInt32(0));
    const auto PDEP_masks = get_PDEP_masks(kb, PDEP_ms_blk, mPDEPWidth);    
    const auto mask_popcounts = get_block_popcounts(kb, PDEP_ms_blk, mPDEPWidth);
    Value * processedBits = kb->getProcessedItemCount("sourceStreamSet");
    Value * blockWidth = kb->getSize(kb->getBitBlockWidth());
    Value * base_block_idx = kb->CreateUDiv(processedBits, blockWidth);
    Value * pdepWidth = kb->getSize(mPDEPWidth);
    Value * PDEP_func = nullptr;
    if (mPDEPWidth == 64) {
        PDEP_func = Intrinsic::getDeclaration(kb->getModule(), Intrinsic::x86_bmi_pdep_64);
    } else if (mPDEPWidth == 32) {
        PDEP_func = Intrinsic::getDeclaration(kb->getModule(), Intrinsic::x86_bmi_pdep_32);
    }
    Value * updatedProcessedBits = processedBits;

    // For each mask extracted from the PDEP marker stream
    for (unsigned i = 0; i < mSwizzleFactor; i++) {
        // Do block and swizzle index calculations, then combine the "src" and "next" swizzles
        Value * current_blk_idx = kb->CreateSub(kb->CreateUDiv(updatedProcessedBits, blockWidth), base_block_idx); // blk index == stream set block index
        Value * current_swizzle_idx = kb->CreateUDiv(kb->CreateURem(updatedProcessedBits, blockWidth), pdepWidth);
        Value * next_block_idx = kb->CreateSub(kb->CreateUDiv(kb->CreateAdd(pdepWidth, updatedProcessedBits), blockWidth), base_block_idx);
        Value * next_swizzle_idx = kb->CreateUDiv(kb->CreateURem(kb->CreateAdd(pdepWidth, updatedProcessedBits), blockWidth), pdepWidth);

        // Load current and next BitBlocks/swizzles
        Value * current_blk_ptr = kb->getAdjustedInputStreamBlockPtr(current_blk_idx, "sourceStreamSet", current_swizzle_idx);
        Value * next_blk_ptr = kb->getAdjustedInputStreamBlockPtr(next_block_idx, "sourceStreamSet", next_swizzle_idx);
        Value * current_swizzle = kb->CreateBlockAlignedLoad(current_blk_ptr);
        Value * next_swizzle = kb->CreateBlockAlignedLoad(next_blk_ptr);

        // Combine the two swizzles to guarantee we'll have enough source bits for the PDEP operation
        Value * shift_amount = kb->CreateURem(updatedProcessedBits, pdepWidth);
        Value * remaining_bits = kb->CreateLShr(current_swizzle, kb->simd_fill(mPDEPWidth, shift_amount)); // shift away bits that have already been used
        Value * borrowed_bits = kb->CreateShl(next_swizzle,
                                              kb->simd_fill(mPDEPWidth, kb->CreateSub(pdepWidth, shift_amount))); // shift next swizzle left by width of first swizzle
        Value * combined = kb->CreateOr(remaining_bits, borrowed_bits); // combine current swizzle and next swizzle

        Value * PDEP_mask = PDEP_masks[i];
        Value * segments = kb->fwCast(mPDEPWidth, combined);
        Value * result_swizzle = Constant::getNullValue(segments->getType());
        // Apply PDEP to each mPDEPWidth segment of the combined swizzle using the current PDEP mask
        for (unsigned j = 0; j < mSwizzleFactor; j++) { 
            Value * source_field = kb->CreateExtractElement(segments, j);
            Value * PDEP_field = kb->CreateCall(PDEP_func, {source_field, PDEP_mask});  
            result_swizzle = kb->CreateInsertElement(result_swizzle, PDEP_field, j); 
        }

        // Store the result
        kb->storeOutputStreamBlock("outputStreamSet", kb->getSize(i), result_swizzle);
        updatedProcessedBits = kb->CreateAdd(updatedProcessedBits, mask_popcounts[i]);
    }
    kb->setProcessedItemCount("sourceStreamSet", updatedProcessedBits);
}

std::vector<Value *> PDEPkernel::get_block_popcounts(const std::unique_ptr<KernelBuilder> & kb, Value * blk, const unsigned field_width) {
    Value * pop_counts = kb->simd_popcount(field_width, blk);
    std::vector<Value *> counts;
    for (unsigned i = 0; i < mSwizzleFactor; i++) {
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
    for (unsigned i = 0; i < mSwizzleFactor; i++) {
        PDEP_masks.push_back(kb->CreateExtractElement(masks, i));
    }
    return PDEP_masks;
}
}