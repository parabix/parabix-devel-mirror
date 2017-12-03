/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef PDEP_KERNEL_H
#define PDEP_KERNEL_H

#include "kernel.h"
#include <llvm/IR/Value.h>
namespace IDISA { class IDISA_Builder; }
/*
What this kernel does:

Given a swizzled input stream set and a PDEP marker stream, apply a PDEP operation to each of the input streams in
the input stream set. The PDEPed result streams are returned in a swizzled output stream set. 

The length of the input stream set (in bits) must be greater than or equal to the total popcount of the PDEP marker
stream, otherwise the PDEP operation will run out of source bits before the entire PDEP stream has been processed.

How it works:

You should know how the PDEP operation works before continuing (Wikipedia has a pretty good explanation.)

The swizzled configuration of the input streams mean that the first blockWidth/mSwizzleFactor bits of each (unswizzled) input
stream are contained in the first BitBlock of the first input StreamSetBlock. The second BitBlock contains the next 
blockWidth/mSwizzleFactor bits for each input stream, and so on. The key observation underpinning the action of the PDEP kernel is that we apply the PDEP operation
using blockWidth/mSwizzleFactor bits of an input stream as the source bits. Since the first BitBlock (i.e. swizzle) contains blockWidth/mSwizzleFactor
bits from each of the input streams, we can begin processing the input streams in the input stream set by applying the first blockWidth/mSwizzleFactor
bits of the PDEP marker stream to each of the swizzle fields in the first BitBlock.

We continue using the first blockWidth/mSwizzleFactor bits of each input stream until we have completely consumed them. This occurs
when the combined popcount of the PDEP masks we've used up to this point > blockWidth/mSwizzleFactor. Once we've exhausted the first
BitBlock (i.e. swizzle), we move on to the next one. This pattern continues until we've consumed
the entire PDEP marker stream. Note that it's possible for the kernel to consume the entire PDEP marker
stream without consuming the entirety of the first BitBlock in the first BitStreamBlock, if the PDEP marker stream has a low popcount
(i.e. > blockWidth/mSwizzleFactor).

There is actually a slight complication that was glossed over in the description above. Consider the following scenario: we've consumed
half of a blockWidth/mSwizzleFactor segment, and we're now starting the PDEP loop again. However, this time the PDEP marker stream segment is
0xffffffff. That is, the popcount is 64. That means we'll consume 64 bits from the source bit stream, but the current segment only contains 64/2 =
32 bits. To get around this issue, we "look ahead" to the next segment, whether that next segment is the next BitBlock in the current StreamSetBlock
or the first BitBlock in the next StreamSetBlock. Regardless of where we find the segment, we combine the current segment and the next segement in 
such a way that we're guarenteed to have 64 source bits to pass to the PDEP operation. The logic responsible for creating this "combined" value
can be found immediately after the opening brace of the outer for loop in the definition of the generateDoBlockMethod function:

Value * current_blk_idx = kb->CreateSub(kb->CreateUDiv(updatedProcessedBits, blockWidth), base_block_idx);  // blk index == stream set block index

// kb->CreateUDiv(updatedProcessedBits, blockWidth) gives us the absolute block idx of the current block.
// However, getAdjustedStreamBlockPtr (used later) calculates an offseted block for us based on the processed item count
// of sourceStreamSet. We want to get the index of the block we're currently processing relative to the 
// "base block" calculated by getAdjustedInputStreamPtr. That's why we subtract base_block_idx from 
// kb->CreateUDiv(updatedProcessedBits, blockWidth)

Value * current_swizzle_idx = kb->CreateUDiv(kb->CreateURem(updatedProcessedBits, blockWidth), pdepWidth);

// updatedProcessedBits % blockWidth is how many bits of the current block we've processed.
// Divide that by pdepWidth to get which BitBlock/swizzle we're currently processing.

Value * next_block_idx = kb->CreateSub(kb->CreateUDiv(kb->CreateAdd(pdepWidth, updatedProcessedBits), blockWidth), base_block_idx);
Value * next_swizzle_idx = kb->CreateUDiv(kb->CreateURem(kb->CreateAdd(pdepWidth, updatedProcessedBits), blockWidth), pdepWidth);
// Q: Why add pdepWidth (i.e. 64) and not 256?
// A: Although it is true that each BitBlock/swizzle contains 256 bits, each swizzle only contains 64 bits from each of the streams
// it is composed of. Each 64 bit field is consumed in parallel, at the same rate. Therefore, once we've consumed "64 bits"
// we've actually consumed 64*4 bits, and it's time to move to the next one.
*/

namespace kernel {
class PDEPkernel : public MultiBlockKernel {
public:
    PDEPkernel(const std::unique_ptr<kernel::KernelBuilder> & kb, unsigned streamCount, unsigned swizzleFactor, unsigned PDEP_width = 64);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    const unsigned mSwizzleFactor;
    const unsigned mPDEPWidth;
    llvm::Value * generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
    std::vector<llvm::Value *> get_PDEP_masks(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * PDEP_ms_blk,
                                              const unsigned mask_width);
    std::vector<llvm::Value *> get_block_popcounts(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * blk,
                                                   const unsigned field_width);
};   
}
    
#endif
