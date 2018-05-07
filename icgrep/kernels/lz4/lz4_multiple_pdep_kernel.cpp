

#include "lz4_multiple_pdep_kernel.h"
#include <kernels/kernel_builder.h>

using namespace llvm;

namespace kernel {

LZ4MultiplePDEPkernel::LZ4MultiplePDEPkernel(const std::unique_ptr<kernel::KernelBuilder> & kb, unsigned streamCount, unsigned streamSize, unsigned swizzleFactor, unsigned PDEP_width, std::string name)
: MultiBlockKernel(name + "",
                   {Binding{kb->getStreamSetTy(), "PDEPmarkerStream", FixedRate(), Principal()}},
                   {},
                   {}, {}, {})
, mSwizzleFactor(swizzleFactor)
, mPDEPWidth(PDEP_width)
, mStreamSize(streamSize)
{
    assert((mSwizzleFactor == (kb->getBitBlockWidth() / PDEP_width)) && "swizzle factor must equal bitBlockWidth / PDEP_width");
    assert((mPDEPWidth == 64 || mPDEPWidth == 32) && "PDEP width must be 32 or 64");

    mStreamSetInputs.push_back(Binding{kb->getStreamSetTy(streamCount), "sourceStreamSet0", PopcountOf("PDEPmarkerStream"), Swizzled()});
    mStreamSetOutputs.push_back(Binding{kb->getStreamSetTy(streamCount), "outputStreamSet0"});

    for (unsigned i = 1; i < streamSize; i++) {
        mStreamSetInputs.push_back(Binding{kb->getStreamSetTy(streamCount), "sourceStreamSet" + std::to_string(i), RateEqualTo("sourceStreamSet0"), Swizzled()});
        mStreamSetOutputs.push_back(Binding{kb->getStreamSetTy(streamCount), "outputStreamSet" + std::to_string(i)});
    }
}

void LZ4MultiplePDEPkernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, Value * const blocksToDo) {
    BasicBlock * entry = kb->GetInsertBlock();

    BasicBlock * loopBody = kb->CreateBasicBlock("loopBody");
    BasicBlock * terminate = kb->CreateBasicBlock("terminate");
    Constant * blockWidth = kb->getSize(kb->getBitBlockWidth());
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
    kb->CreateBr(loopBody);

    kb->SetInsertPoint(loopBody);
    // The following PHINodes' values can come from entry or processBlock
    PHINode * blockOffsetPhi = kb->CreatePHI(kb->getSizeTy(), 2); // block offset from the base block, e.g. 0, 1, 2, ...
    PHINode * updatedProcessedSourceBitsPhi = kb->CreatePHI(kb->getSizeTy(), 2);
    blockOffsetPhi->addIncoming(kb->getSize(0), entry);
    updatedProcessedSourceBitsPhi->addIncoming(processedSourceBits, entry);
    Value * updatedProcessedSourceBits = updatedProcessedSourceBitsPhi;
    Value * PDEP_ms_blk = kb->loadInputStreamBlock("PDEPmarkerStream", kb->getInt32(0), blockOffsetPhi);

    const auto PDEP_masks = get_PDEP_masks(kb, PDEP_ms_blk, mPDEPWidth);
    const auto mask_popcounts = get_block_popcounts(kb, PDEP_ms_blk, mPDEPWidth);

    // For each mask extracted from the PDEP marker block
    for (unsigned i = 0; i < mSwizzleFactor; i++) {
        // Do block and swizzle index calculations, then combine the "current" and "next" swizzles

        Value * current_blk_idx = kb->CreateSub(kb->CreateUDiv(updatedProcessedSourceBits, blockWidth), base_src_blk_idx); // blk index == stream set block index
        Value * current_swizzle_idx = kb->CreateUDiv(kb->CreateURem(updatedProcessedSourceBits, blockWidth), pdepWidth);
        Value * ahead_pdep_width_less_1 = kb->CreateAdd(pdepWidth_1, updatedProcessedSourceBits);


        Value * shift_amount = kb->CreateURem(updatedProcessedSourceBits, pdepWidth);

        for (unsigned iStreamIndex = 0; iStreamIndex < mStreamSize; iStreamIndex++) {
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
    }

    updatedProcessedSourceBitsPhi->addIncoming(updatedProcessedSourceBits, loopBody);
    blockOffsetPhi->addIncoming(kb->CreateAdd(blockOffsetPhi, kb->getSize(1)), loopBody);
    Value * haveRemBlocks = kb->CreateICmpNE(blockOffsetPhi, blocksToDo);
    kb->CreateCondBr(haveRemBlocks, loopBody, terminate);

    kb->SetInsertPoint(terminate);
    kb->setProcessedItemCount("sourceStreamSet0", updatedProcessedSourceBitsPhi);

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
