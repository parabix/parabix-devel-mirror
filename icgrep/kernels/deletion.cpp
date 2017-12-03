/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "deletion.h"
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace kernel {

inline std::vector<Value *> parallel_prefix_deletion_masks(const std::unique_ptr<KernelBuilder> & iBuilder, const unsigned fw, Value * del_mask) {
    Value * m = iBuilder->simd_not(del_mask);
    Value * mk = iBuilder->simd_slli(fw, del_mask, 1);
    std::vector<Value *> move_masks;
    for (unsigned shift = 1; shift < fw; shift *= 2) {
        Value * mp = mk;
        for (unsigned lookright = 1; lookright < fw; lookright *= 2) {
            mp = iBuilder->simd_xor(mp, iBuilder->simd_slli(fw, mp, lookright));
        }
        Value * mv = iBuilder->simd_and(mp, m);
        m = iBuilder->simd_or(iBuilder->simd_xor(m, mv), iBuilder->simd_srli(fw, mv, shift));
        mk = iBuilder->simd_and(mk, iBuilder->simd_not(mp));
        move_masks.push_back(mv);
    }
    return move_masks;
}

inline Value * apply_parallel_prefix_deletion(const std::unique_ptr<KernelBuilder> & iBuilder, const unsigned fw, Value * del_mask, const std::vector<Value *> & mv, Value * strm) {
    Value * s = iBuilder->simd_and(strm, iBuilder->simd_not(del_mask));
    for (unsigned i = 0; i < mv.size(); i++) {
        unsigned shift = 1 << i;
        Value * t = iBuilder->simd_and(s, mv[i]);
        s = iBuilder->simd_or(iBuilder->simd_xor(s, t), iBuilder->simd_srli(fw, t, shift));
    }
    return s;
}

inline Value * partial_sum_popcount(const std::unique_ptr<KernelBuilder> & iBuilder, const unsigned fw, Value * mask) {
    Value * field = iBuilder->simd_popcount(fw, mask);
    const auto count = iBuilder->getBitBlockWidth() / fw;
    for (unsigned move = 1; move < count; move *= 2) {
        field = iBuilder->simd_add(fw, field, iBuilder->mvmd_slli(fw, field, move));
    }
    return field;
}

SwizzledDeleteByPEXTkernel::SwizzledDeleteByPEXTkernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned fw, unsigned streamCount, unsigned PEXT_width)
: BlockOrientedKernel("PEXTdel" + std::to_string(fw) + "_" + std::to_string(streamCount),
                  {Binding{iBuilder->getStreamSetTy(streamCount), "inputStreamSet"}, Binding{iBuilder->getStreamSetTy(), "delMaskSet"}},
                  {}, {}, {}, {})
, mDelCountFieldWidth(fw)
, mStreamCount(streamCount)
, mSwizzleFactor(iBuilder->getBitBlockWidth() / PEXT_width)
// add mSwizzleFactor - 1 to mStreamCount before dividing by mSwizzleFactor
// to prevent rounding errors.
, mSwizzleSetCount((mStreamCount + mSwizzleFactor - 1)/mSwizzleFactor)
, mPEXTWidth(PEXT_width)
{
    assert((mDelCountFieldWidth > 0) && ((mDelCountFieldWidth & (mDelCountFieldWidth - 1)) == 0)
        && "mDelCountFieldWidth must be a power of 2");
    assert(mSwizzleFactor > 1 && "mDelCountFieldWidth must be less than the block width");
    assert((mPEXTWidth == 64 || mPEXTWidth == 32) && "PEXT width must be 32 or 64");

    // why, if we have 1 input stream, are there n output swizzle streams rather 1 of n?
    mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(mSwizzleFactor, 1), "outputSwizzle0", BoundedRate(0, 1)});
    addScalar(iBuilder->getBitBlockType(), "pendingSwizzleData0");
    for (unsigned i = 1; i < mSwizzleSetCount; i++) {
        mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(mSwizzleFactor, 1),
            "outputSwizzle" + std::to_string(i), RateEqualTo("outputSwizzle0")});
        addScalar(iBuilder->getBitBlockType(), "pendingSwizzleData" + std::to_string(i));
    }
    addScalar(iBuilder->getSizeTy(), "pendingOffset");
}

void SwizzledDeleteByPEXTkernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    // We use delMask to apply the same PEXT delete operation to each stream in the input stream set
    Value * delMask = iBuilder->loadInputStreamBlock("delMaskSet", iBuilder->getInt32(0));
    const auto masks = get_PEXT_masks(iBuilder, delMask);
    generateProcessingLoop(iBuilder, masks, delMask);
}

void SwizzledDeleteByPEXTkernel::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> &iBuilder, Value * remainingBytes) {
    IntegerType * vecTy = iBuilder->getIntNTy(iBuilder->getBitBlockWidth());
    Value * remaining = iBuilder->CreateZExt(remainingBytes, vecTy);
    Value * EOF_del = iBuilder->bitCast(iBuilder->CreateShl(Constant::getAllOnesValue(vecTy), remaining));
    Value * delMask = iBuilder->CreateOr(EOF_del, iBuilder->loadInputStreamBlock("delMaskSet", iBuilder->getInt32(0)));
    const auto masks = get_PEXT_masks(iBuilder, delMask);
    generateProcessingLoop(iBuilder, masks, delMask);
    Constant * blockOffsetMask = iBuilder->getSize(iBuilder->getBitBlockWidth() - 1);
    Constant * outputIndexShift = iBuilder->getSize(std::log2(mDelCountFieldWidth));
    
    Value * outputProduced = iBuilder->getProducedItemCount("outputSwizzle0"); // All output groups have the same count.
    Value * producedOffset = iBuilder->CreateAnd(outputProduced, blockOffsetMask);
    Value * outputIndex = iBuilder->CreateLShr(producedOffset, outputIndexShift);
    Value * pendingOffset = iBuilder->getScalarField("pendingOffset");

    // Write the pending data.
    for (unsigned i = 0; i < mSwizzleSetCount; i++) {
        Value * pendingData = iBuilder->getScalarField("pendingSwizzleData" + std::to_string(i));
        Value * outputStreamPtr = iBuilder->getOutputStreamBlockPtr("outputSwizzle" + std::to_string(i), iBuilder->getInt32(0));
        iBuilder->CreateBlockAlignedStore(pendingData, iBuilder->CreateGEP(outputStreamPtr, outputIndex));
    }
    iBuilder->setProducedItemCount("outputSwizzle0", iBuilder->CreateAdd(pendingOffset, outputProduced));
}

std::vector<Value *> SwizzledDeleteByPEXTkernel::get_PEXT_masks(const std::unique_ptr<KernelBuilder> & iBuilder, Value * del_mask) {
    // Del mask marks locations of bits we want to delete with 1 bits. Delete marked bits by extracting only the bits not marked in this way.
    // Apply the PEXT operation mPEXTWidth bits at a time (e.g. if block is 256 bits and mPEXTWidth is 64, apply 4 PEXT ops to full process block.
    Value * m = iBuilder->fwCast(mPEXTWidth, iBuilder->simd_not(del_mask)); 
    std::vector<Value *> masks;
    for (unsigned i = 0; i < iBuilder->getBitBlockWidth()/mPEXTWidth; i++) {
        masks.push_back(iBuilder->CreateExtractElement(m, i));
    }
    return masks;
}

void SwizzledDeleteByPEXTkernel::generateProcessingLoop(const std::unique_ptr<KernelBuilder> & iBuilder, const std::vector<Value *> & masks,
                                                Value * delMask) {
    Value * delCount = iBuilder->simd_popcount(mDelCountFieldWidth, iBuilder->simd_not(delMask)); // delMask marks the positions we want to extract
    std::vector<Value *> counts;
    for (unsigned i = 0; i < iBuilder->getBitBlockWidth()/ mPEXTWidth; i++) {
    	// Store the deletion counts for each PEXT field
        counts.push_back(iBuilder->CreateExtractElement(delCount, i)); // Extract field i from SIMD register delCount
    }

    generatePEXTAndSwizzleLoop(iBuilder, masks, counts);
}

/*
What this function does in pseudo code:
for (mSwizzleFactor)
	create a swizzle set containing mSwizzleFactor blocks
	apply PEXT to each block in the swizzle set
	store the swizzleSet in PEXTedSwizzleSets vector
	
for (each swizzle row i)
	for (each swizzle set j)
		processes row i in swizzle set j
		store output in pendingData[j]
*/
void SwizzledDeleteByPEXTkernel::generatePEXTAndSwizzleLoop(const std::unique_ptr<KernelBuilder> & iBuilder, const std::vector<Value *> & masks,
                                                    std::vector<Value *> counts) {
    // For each of the k swizzle sets required to apply PEXT to all input streams
    std::vector<std::vector<Value *>> PEXTedSwizzleSets;
    for (unsigned j = 0; j < mSwizzleSetCount; ++j) {
    // Group input blocks together into input swizzle set. Input set should contain mSwizzleSetCount blocks (e.g. for U8U16 16/4=4).
    // Each block belongs to a different input stream.
        std::vector<Value *> input;
        unsigned streamSelectionIndex = j * mSwizzleFactor;
        for (unsigned i = streamSelectionIndex; i < (streamSelectionIndex + mSwizzleFactor); ++i) {
	    	// Check if i > mStreamCount. If it is, add null streams until we get mSwizzleSetCount streams in the input vector
            if ( i >= mStreamCount) {
				input.push_back(iBuilder->allZeroes());
            } else {
                input.push_back(iBuilder->loadInputStreamBlock("inputStreamSet", iBuilder->getInt32(i)));
            }
        }
        // each partiallyCompressedSwizzleSet is obtained by applying PEXT to each of the blocks in input
        PEXTedSwizzleSets.push_back(apply_PEXT_deletion_with_swizzle(iBuilder, masks, input));
    }
 	// Compress the PEXTedSwizzleSets
    // Output is written and committed to the output buffer one swizzle at a time.
    Constant * blockOffsetMask = iBuilder->getSize(iBuilder->getBitBlockWidth() - 1);
    Constant * outputIndexShift = iBuilder->getSize(std::log2(mDelCountFieldWidth));
    
    Value * outputProduced = iBuilder->getProducedItemCount("outputSwizzle0"); // All output groups have the same count.
    Value * producedOffset = iBuilder->CreateAnd(outputProduced, blockOffsetMask);
    Value * outputIndex = iBuilder->CreateLShr(producedOffset, outputIndexShift);

    // There may be pending data in the kernel state, for up to mDelCountFieldWidth-1 bits per stream.
    Value * pendingOffset = iBuilder->getScalarField("pendingOffset");
    // There is a separate vector of pending data for each swizzle group.
    std::vector<Value *> pendingData;
    std::vector<Value *> outputStreamPtr;

    for (unsigned i = 0; i < mSwizzleSetCount; i++) {
        pendingData.push_back(iBuilder->getScalarField("pendingSwizzleData" + std::to_string(i)));
        outputStreamPtr.push_back(iBuilder->getOutputStreamBlockPtr("outputSwizzle" + std::to_string(i), iBuilder->getInt32(0)));
    }

    // For each row i
    for (unsigned i = 0; i < mSwizzleFactor; i++) {
        // Generate code for each of the mSwizzleFactor fields making up a block.
        // We load the count for the field and process all swizzle groups accordingly.
        Value * newItemCount = counts[i];
        //iBuilder->CallPrintInt("NeW ITeM COUNT!", newItemCount); //TODO remove
        Value * pendingSpace = iBuilder->CreateSub(iBuilder->getSize(mDelCountFieldWidth), pendingOffset);
        Value * pendingSpaceFilled = iBuilder->CreateICmpUGE(newItemCount, pendingSpace);
        
        // Data from the ith swizzle pack of each group is processed
        // according to the same newItemCount, pendingSpace, ...
        for (unsigned j = 0; j < mSwizzleSetCount; j++) {
            Value * newItems = PEXTedSwizzleSets[j][i];
            //iBuilder->CallPrintRegister("NeW ITeMS!", newItems); //TODO remove
            // Combine as many of the new items as possible into the pending group.
            Value * combinedGroup = iBuilder->CreateOr(pendingData[j], iBuilder->CreateShl(newItems, iBuilder->simd_fill(mDelCountFieldWidth,
                pendingOffset)));
	    //iBuilder->CallPrintRegister("ComBineDGROUP", combinedGroup);
            // To avoid an unpredictable branch, always store the combined group, whether full or not.             
            iBuilder->CreateBlockAlignedStore(combinedGroup, iBuilder->CreateGEP(outputStreamPtr[j], outputIndex));
            
            // Any items in excess of the space available in the current pending group overflow for the next group.
            Value * overFlowGroup = iBuilder->CreateLShr(newItems, iBuilder->simd_fill(mDelCountFieldWidth, pendingSpace));
            // If we filled the space, then the overflow group becomes the new pending group and the index is updated.
            pendingData[j] = iBuilder->CreateSelect(pendingSpaceFilled, overFlowGroup, combinedGroup);
        }
        outputIndex = iBuilder->CreateSelect(pendingSpaceFilled, iBuilder->CreateAdd(outputIndex, iBuilder->getSize(1)), outputIndex);
        pendingOffset = iBuilder->CreateAnd(iBuilder->CreateAdd(newItemCount, pendingOffset), iBuilder->getSize(mDelCountFieldWidth-1));
    }
    
    iBuilder->setScalarField("pendingOffset", pendingOffset);
    //iBuilder->CallPrintInt("pendingOffset", pendingOffset);
    
    Value * newlyProduced = iBuilder->CreateSub(iBuilder->CreateShl(outputIndex, outputIndexShift), producedOffset);
    Value * produced = iBuilder->CreateAdd(outputProduced, newlyProduced);
    for (unsigned j = 0; j < mSwizzleSetCount; j++) {
        iBuilder->setScalarField("pendingSwizzleData" + std::to_string(j), pendingData[j]);
        //iBuilder->CallPrintRegister("pendingData[j]", pendingData[j]);
    }
    iBuilder->setProducedItemCount("outputSwizzle0", produced);
}

/*
Apply PEXT deletion to the blocks in strms and swizzle the result.

Q: Why is it advantageous to swizzle the PEXTed streams?

A: PEXT doesn't compress streams, if the input to a PEXT operation is 64 bits wide, the output is also 64 bits wide.

Example:
Input:     11101101
PEXT mask: 11110000
Output:    00001110

PEXT selects the bits we tell it to and stores them at contiguous lower-order bits. Higher-order bits are
cleared. This has implications if we're working with multiple streams.

For example, say we've applied PEXT on the following 4 streams using this deletion mask (inverse of PEXT mask): 00000011 00011111 00111111 00000111
(I think this diagram is backwards, PEXTed bits should be stored in lower-order bits, not higher.)
Stream 1:   abcdef00 ghi00000 jk000000 lmnop000
Stream 2:   qrstuv00 wxy00000 z1000000 23456000
Stream 3:   ABCDEF00 GHI00000 JK000000 LMNOP000
Stream 4:   QRSTUV00 WZY00000 Z1000000 23456000

If we wanted to compress each stream to remove the sequences of 0s, it's tricky. The first 32 bits of each stream
should be compress by 2 bits, the second 32 bits by 5, etc. If we swizzle the streams with a swizzle factor of 4 we have a much easier
time:

The swizzled output using a field width of 8 produces the following swizzles (swizzle factor = block width / pext field width = 4).

Swizzle 1:  abcdef00 qrstuv00 ABCDEF00 QRSTUV00
Swizzle 2:  ghi00000 wxy00000 GHI00000 WZY00000
Swizzle 3:  jk000000 z1000000 JK000000 Z1000000
Swizzle 4:  lmnop000 23456000 LMNOP000 23456000

Now we can compress each 32-bit segment of swizzle 1 by 2, each 32 bit segment of swizzle 2 by 4, etc. Once we've completed the 
compression, we unswizzle to restore the 4 streams. The streams are now fully compressed!

Args:
    strms: the vector of blocks to apply PEXT operations to. strms[i] is the block associated with the ith input stream.
    masks: the PEXT deletion masks to apply to each block in strms (input mask is broken into PEXT width pieces, apply pieces 
        sequentially to PEXT a full block.)

Returns:
    output (vector of Value*): Swizzled, PEXTed version of strms. See example above.
*/
std::vector<Value *> SwizzledDeleteByPEXTkernel::apply_PEXT_deletion_with_swizzle(const std::unique_ptr<KernelBuilder> & iBuilder, 
                                                             const std::vector<Value *> & masks, std::vector<Value *> strms) {
    Value * PEXT_func = nullptr;
    if (mPEXTWidth == 64) {
        PEXT_func = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::x86_bmi_pext_64);
    } else if (mPEXTWidth == 32) {
        PEXT_func = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::x86_bmi_pext_32);
    }
    
    std::vector<Value *> output;     
    for (unsigned i = 0; i < strms.size(); i++) {
        Value * v = iBuilder->fwCast(mPEXTWidth, strms[i]);
        output.push_back(Constant::getNullValue(v->getType())); 
    }

    // For each of the input streams
    for (unsigned j = 0; j < strms.size(); j++) {
        Value * v = iBuilder->fwCast(mPEXTWidth, strms[j]); // load stream j
        // Process the stream's block mPEXTWidth bits at a time (a PEXT operation can't do more than 64 bits at a time)
        for (unsigned i = 0; i < iBuilder->getBitBlockWidth()/mPEXTWidth; i++) {
            Value * field = iBuilder->CreateExtractElement(v, i); // Load from block j at index i (load mPEXTWidth bits)
            Value * PEXTed_field = iBuilder->CreateCall(PEXT_func, {field, masks[i]}); // Apply PEXT deletion to the segment we just loaded
            /*
             We loaded from input at index i within stream j's block. We store result in ouput within stream i's block at position j. This swizzles the output blocks.
             E.g.:

               *i*
            *j* a b c d strms[0]
                e f g h 
                i j k l
                m n o p

             Apply pext deletion at each position, then swizzle results:
               *j*
            *i* a` e` i` m` output[0]
                b` f` j` n`
                c` g` k` o`  
                d` i` l` p`          
            */    
            output[i] = iBuilder->CreateInsertElement(output[i], PEXTed_field, j); 
            /*
            numCompressedBits = 0

            for (each swizzleField position j)
                for (each input swizzle i)
                    get PEXTed_field
                    Shift PEXTed_field left by "numCompressedBits" (in output[i])
                    OR PEXTed_field into output[i] (output[i] is output swizzle buffer for input swizzle i)
                numCompressedBits += popcount(mask[i])
            */
        }
    }
    
    return output;
}

Value * SwizzledDeleteByPEXTkernel::apply_PEXT_deletion(const std::unique_ptr<KernelBuilder> & iBuilder, const std::vector<Value *> & masks, Value * strm) {
    Value * PEXT_func = nullptr;
    if (mPEXTWidth == 64) {
        PEXT_func = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::x86_bmi_pext_64);
    } else if (mPEXTWidth == 32) {
        PEXT_func = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::x86_bmi_pext_32);
    }
       
    Value * v = iBuilder->fwCast(mPEXTWidth, strm);
    Value * output = Constant::getNullValue(v->getType());
    for (unsigned i = 0; i < iBuilder->getBitBlockWidth()/mPEXTWidth; i++) {
        Value * field = iBuilder->CreateExtractElement(v, i);
        Value * compressed = iBuilder->CreateCall(PEXT_func, {field, masks[i]});
        output = iBuilder->CreateInsertElement(output, compressed, i);
    }
    return output;
}

// Apply deletion to a set of stream_count input streams to produce a set of output streams.
// Kernel inputs: stream_count data streams plus one del_mask stream
// Outputs: the deleted streams, plus a partial sum popcount

void DeletionKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    Value * delMask = iBuilder->loadInputStreamBlock("delMaskSet", iBuilder->getInt32(0));
    const auto move_masks = parallel_prefix_deletion_masks(iBuilder, mDeletionFieldWidth, delMask);
    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * input = iBuilder->loadInputStreamBlock("inputStreamSet", iBuilder->getInt32(j));
        Value * output = apply_parallel_prefix_deletion(iBuilder, mDeletionFieldWidth, delMask, move_masks, input);
        iBuilder->storeOutputStreamBlock("outputStreamSet", iBuilder->getInt32(j), output);
    }
    Value * delCount = partial_sum_popcount(iBuilder, mDeletionFieldWidth, iBuilder->simd_not(delMask));
    iBuilder->storeOutputStreamBlock("deletionCounts", iBuilder->getInt32(0), iBuilder->bitCast(delCount));
}

void DeletionKernel::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder, Value * remainingBytes) {
    IntegerType * vecTy = iBuilder->getIntNTy(iBuilder->getBitBlockWidth());
    Value * remaining = iBuilder->CreateZExt(remainingBytes, vecTy);
    Value * EOF_del = iBuilder->bitCast(iBuilder->CreateShl(Constant::getAllOnesValue(vecTy), remaining));
    Value * delMask = iBuilder->CreateOr(EOF_del, iBuilder->loadInputStreamBlock("delMaskSet", iBuilder->getInt32(0)));
    const auto move_masks = parallel_prefix_deletion_masks(iBuilder, mDeletionFieldWidth, delMask);
    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * input = iBuilder->loadInputStreamBlock("inputStreamSet", iBuilder->getInt32(j));
        Value * output = apply_parallel_prefix_deletion(iBuilder, mDeletionFieldWidth, delMask, move_masks, input);
        iBuilder->storeOutputStreamBlock("outputStreamSet", iBuilder->getInt32(j), output);
    }
    Value * const delCount = partial_sum_popcount(iBuilder, mDeletionFieldWidth, iBuilder->simd_not(delMask));
    iBuilder->storeOutputStreamBlock("deletionCounts", iBuilder->getInt32(0), iBuilder->bitCast(delCount));
}

DeletionKernel::DeletionKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const unsigned fieldWidth, const unsigned streamCount)
: BlockOrientedKernel("del" + std::to_string(fieldWidth) + "_" + std::to_string(streamCount),
              {Binding{iBuilder->getStreamSetTy(streamCount), "inputStreamSet"},
               Binding{iBuilder->getStreamSetTy(), "delMaskSet"}},
              {Binding{iBuilder->getStreamSetTy(streamCount), "outputStreamSet"},
               Binding{iBuilder->getStreamSetTy(), "deletionCounts", FixedRate(), RoundUpTo(iBuilder->getBitBlockWidth())}},
              {}, {}, {})
, mDeletionFieldWidth(fieldWidth)
, mStreamCount(streamCount) {
}

const unsigned PEXT_width = 64;

inline std::vector<Value *> get_PEXT_masks(const std::unique_ptr<KernelBuilder> & iBuilder, Value * del_mask) {
    Value * m = iBuilder->fwCast(PEXT_width, iBuilder->simd_not(del_mask));
    std::vector<Value *> masks;
    for (unsigned i = 0; i < iBuilder->getBitBlockWidth()/PEXT_width; i++) {
        masks.push_back(iBuilder->CreateExtractElement(m, i));
    }
    return masks;
}

// Apply PEXT deletion to a collection of blocks and swizzle the result.
// strms contains the blocks to process
inline std::vector<Value *> apply_PEXT_deletion_with_swizzle(const std::unique_ptr<KernelBuilder> & iBuilder, const std::vector<Value *> & masks, std::vector<Value *> strms) {
    Value * PEXT_func = nullptr;
    if (PEXT_width == 64) {
        PEXT_func = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::x86_bmi_pext_64);
    } else if (PEXT_width == 32) {
        PEXT_func = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::x86_bmi_pext_32);
    }
    
    std::vector<Value *> output;     
    for (unsigned i = 0; i < strms.size(); i++) {
        Value * v = iBuilder->fwCast(PEXT_width, strms[i]);
        output.push_back(Constant::getNullValue(v->getType())); 
    }

    // For each of the input streams
    for (unsigned j = 0; j < strms.size(); j++) {
        Value * v = iBuilder->fwCast(PEXT_width, strms[j]); // load stream j
        // Process the stream's block in PEXT_width chunks (PEXT operation can't do more than 64 bits at a time)
        for (unsigned i = 0; i < iBuilder->getBitBlockWidth()/PEXT_width; i++) {
            Value * field = iBuilder->CreateExtractElement(v, i); // Load from block j at index i (fw of j is PEXT_width)
            Value * compressed = iBuilder->CreateCall(PEXT_func, {field, masks[i]}); // Apply PEXT deletion to the block segment we just loaded
            /*
             We loaded from input at index i within stream j's block. We store result in ouput within stream i's block at position j. This swizzles the output blocks . E.g.:

             a b c d
             e f g h 
             i j k l
             m n o p

             Apply pext deletion at each position, then swizzle results:

             a` e` i` m`
             b` f` j` n`
             c` g` k` o`  
             d` i` l` p`          
            */      
            output[i] = iBuilder->CreateInsertElement(output[i], compressed, j); 
        }
    }
    
    return output;
}

inline Value * apply_PEXT_deletion(const std::unique_ptr<KernelBuilder> & iBuilder, const std::vector<Value *> & masks, Value * strm) {
    Value * PEXT_func = nullptr;
    if (PEXT_width == 64) {
        PEXT_func = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::x86_bmi_pext_64);
    } else if (PEXT_width == 32) {
        PEXT_func = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::x86_bmi_pext_32);
    }
       
    Value * v = iBuilder->fwCast(PEXT_width, strm);
    Value * output = Constant::getNullValue(v->getType());
    for (unsigned i = 0; i < iBuilder->getBitBlockWidth()/PEXT_width; i++) {
        Value * field = iBuilder->CreateExtractElement(v, i);
        Value * compressed = iBuilder->CreateCall(PEXT_func, {field, masks[i]});
        output = iBuilder->CreateInsertElement(output, compressed, i);
    }
    return output;
}

// Apply deletion to a set of stream_count input streams and produce a set of swizzled output streams.
// Kernel inputs: stream_count data streams plus one del_mask stream
// Outputs: swizzles containing the swizzled deleted streams, plus a partial sum popcount

void DeleteByPEXTkernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    Value * delMask = iBuilder->loadInputStreamBlock("delMaskSet", iBuilder->getInt32(0));
    const auto masks = get_PEXT_masks(iBuilder, delMask);
    generateProcessingLoop(iBuilder, masks, delMask);
}

void DeleteByPEXTkernel::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> &iBuilder, Value * remainingBytes) {
    IntegerType * vecTy = iBuilder->getIntNTy(iBuilder->getBitBlockWidth());
    Value * remaining = iBuilder->CreateZExt(remainingBytes, vecTy);
    Value * EOF_del = iBuilder->bitCast(iBuilder->CreateShl(Constant::getAllOnesValue(vecTy), remaining));
    Value * delMask = iBuilder->CreateOr(EOF_del, iBuilder->loadInputStreamBlock("delMaskSet", iBuilder->getInt32(0)));
    const auto masks = get_PEXT_masks(iBuilder, delMask);
    generateProcessingLoop(iBuilder, masks, delMask);
}

void DeleteByPEXTkernel::generateProcessingLoop(const std::unique_ptr<KernelBuilder> & iBuilder, const std::vector<Value *> & masks, Value * delMask) {
    if (mShouldSwizzle) {
        generatePEXTAndSwizzleLoop(iBuilder, masks);
    } else {
        generatePEXTLoop(iBuilder, masks);
    }
    //Value * delCount = partial_sum_popcount(iBuilder, mDelCountFieldWidth, apply_PEXT_deletion(iBuilder, masks, iBuilder->simd_not(delMask)));
    Value * delCount = iBuilder->simd_popcount(mDelCountFieldWidth, iBuilder->simd_not(delMask));
    iBuilder->storeOutputStreamBlock("deletionCounts", iBuilder->getInt32(0), iBuilder->bitCast(delCount));
}

void DeleteByPEXTkernel::generatePEXTLoop(const std::unique_ptr<KernelBuilder> &iBuilder, const std::vector<Value *> & masks) {
    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * input = iBuilder->loadInputStreamBlock("inputStreamSet", iBuilder->getInt32(j));
        Value * output = apply_PEXT_deletion(iBuilder, masks, input);
        iBuilder->storeOutputStreamBlock("outputStreamSet", iBuilder->getInt32(j), output);
    }
}

void DeleteByPEXTkernel::generatePEXTAndSwizzleLoop(const std::unique_ptr<KernelBuilder> & iBuilder, const std::vector<Value *> & masks) {
    // Group blocks together into input vector. Input should contain mStreamCount/mSwizzleFactor blocks (e.g. for U8U16 16/4=4)
    // mStreamCount/mSwizzleFactor -> (mStreamCount + mSwizzleFactor - 1) / mSwizzleFactor
    for (unsigned j = 0; j < (mStreamCount + mSwizzleFactor - 1)/mSwizzleFactor; ++j) {
        std::vector<Value *> input;
        unsigned streamSelectionIndex = j * mSwizzleFactor;
        for (unsigned i = streamSelectionIndex; i < (streamSelectionIndex + mSwizzleFactor); ++i) {
	    	// Check if i > mStreamCount. If it is, add null streams until we get mStreamCount/mSwizzleFactor streams in the input vector
            if ( i >= mStreamCount) {
				input.push_back(iBuilder->allZeroes());
            } else {
                input.push_back(iBuilder->loadInputStreamBlock("inputStreamSet", iBuilder->getInt32(i)));
            }
        }
        std::vector<Value *> output = apply_PEXT_deletion_with_swizzle(iBuilder, masks, input);
        for (unsigned i = 0; i < mSwizzleFactor; i++) { 
             iBuilder->storeOutputStreamBlock(std::string(mOutputSwizzleNameBase) + std::to_string(j), iBuilder->getInt32(i), output[i]);
        }
    }
}

DeleteByPEXTkernel::DeleteByPEXTkernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned fw, unsigned streamCount, bool shouldSwizzle)
: BlockOrientedKernel("PEXTdel" + std::to_string(fw) + "_" + std::to_string(streamCount) + (shouldSwizzle ? "swiz" : "noswiz"),
                  {Binding{iBuilder->getStreamSetTy(streamCount), "inputStreamSet"},
                      Binding{iBuilder->getStreamSetTy(), "delMaskSet"}},
                  {}, {}, {}, {})
, mDelCountFieldWidth(fw)
, mStreamCount(streamCount)
, mSwizzleFactor(iBuilder->getBitBlockWidth() / PEXT_width)
, mShouldSwizzle(shouldSwizzle)
{
    if(mShouldSwizzle) {        
        for (unsigned i = 0; i < (mStreamCount + mSwizzleFactor - 1)/mSwizzleFactor; i++) { 
            mStreamSetOutputs.emplace_back(iBuilder->getStreamSetTy(mSwizzleFactor), std::string(mOutputSwizzleNameBase) + std::to_string(i));
        }
    } else {
        // No swizzling. Output results as single stream set
        mStreamSetOutputs.emplace_back(iBuilder->getStreamSetTy(mStreamCount), "outputStreamSet");
    }
    mStreamSetOutputs.emplace_back(iBuilder->getStreamSetTy(), "deletionCounts");
}

    
//
// This kernel performs final stream compression for a set of N bitstreams, given
// (a) a set of bitstreams partially compressed within K-bit fields and stored
//     in K-bit swizzled form, and
// (b) a stream of deletion/extraction counts per K-bit stride.
//
// Restrictions:  At present, only K=64 is supported.
//                At present, N must be an exact multiple of BLOCK_SIZE/K.
//
// The kernel always consumes full blocks of input and emits data into the output
// buffer in swizzles of K items at a time.   Upon completion of a segment,
// up to K-1 pending output items per stream may be stored in the kernel state.
//
// Note: that both input streams and output streams are stored in swizzled form.
//

SwizzledBitstreamCompressByCount::SwizzledBitstreamCompressByCount(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned bitStreamCount, unsigned fieldWidth)
: BlockOrientedKernel("swizzled_compress" + std::to_string(fieldWidth) + "_" + std::to_string(bitStreamCount),
                     {Binding{iBuilder->getStreamSetTy(), "countsPerStride"}}, {}, {}, {}, {})
, mBitStreamCount(bitStreamCount)
    , mFieldWidth(fieldWidth)
    , mSwizzleFactor(iBuilder->getBitBlockWidth() / fieldWidth)
    , mSwizzleSetCount((mBitStreamCount + mSwizzleFactor - 1)/mSwizzleFactor) {
        assert((fieldWidth > 0) && ((fieldWidth & (fieldWidth - 1)) == 0) && "fieldWidth must be a power of 2");
        assert(mSwizzleFactor > 1 && "fieldWidth must be less than the block width");
        mStreamSetInputs.push_back(Binding{iBuilder->getStreamSetTy(mSwizzleFactor, 1), "inputSwizzle0"});
        mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(mSwizzleFactor, 1), "outputSwizzle0", BoundedRate(0, 1)});
        addScalar(iBuilder->getBitBlockType(), "pendingSwizzleData0");
        for (unsigned i = 1; i < mSwizzleSetCount; i++) {
            mStreamSetInputs.push_back(Binding{iBuilder->getStreamSetTy(mSwizzleFactor, 1), "inputSwizzle" + std::to_string(i)});
            mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(mSwizzleFactor, 1), "outputSwizzle" + std::to_string(i), RateEqualTo("outputSwizzle0")});
            addScalar(iBuilder->getBitBlockType(), "pendingSwizzleData" + std::to_string(i));
        }
        addScalar(iBuilder->getSizeTy(), "pendingOffset");
}
    
void SwizzledBitstreamCompressByCount::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
        
    Value * countsPerStridePtr = iBuilder->getInputStreamBlockPtr("countsPerStride", iBuilder->getInt32(0));
    Value * countStreamPtr = iBuilder->CreatePointerCast(countsPerStridePtr, iBuilder->getIntNTy(mFieldWidth)->getPointerTo());
    
    // Output is written and committed to the output buffer one swizzle at a time.
    //
    Constant * blockOffsetMask = iBuilder->getSize(iBuilder->getBitBlockWidth() - 1);
    Constant * outputIndexShift = iBuilder->getSize(std::log2(mFieldWidth));
    
    Value * outputProduced = iBuilder->getProducedItemCount("outputSwizzle0"); // All output groups have the same count.
    Value * producedOffset = iBuilder->CreateAnd(outputProduced, blockOffsetMask);
    Value * outputIndex = iBuilder->CreateLShr(producedOffset, outputIndexShift);

    // There may be pending data in the kernel state, for up to mFieldWidth-1 bits per stream.
    Value * pendingOffset = iBuilder->getScalarField("pendingOffset");
    // There is a separate vector of pending data for each swizzle group.
    std::vector<Value *> pendingData;
    std::vector<Value *> outputStreamPtr;
    for (unsigned i = 0; i < mSwizzleSetCount; i++) {
        pendingData.push_back(iBuilder->getScalarField("pendingSwizzleData" + std::to_string(i)));
        outputStreamPtr.push_back(iBuilder->getOutputStreamBlockPtr("outputSwizzle" + std::to_string(i), iBuilder->getInt32(0)));
    }
    
    // Generate code for each of the mSwizzleFactor fields making up a block.
    // We load the count for the field and process all swizzle groups accordingly.
    for (unsigned i = 0; i < mSwizzleFactor; i++) {
        Value * newItemCount = iBuilder->CreateLoad(iBuilder->CreateGEP(countStreamPtr, iBuilder->getInt32(i)));
    //iBuilder->CallPrintInt("newItemCount", newItemCount);
        Value * pendingSpace = iBuilder->CreateSub(iBuilder->getSize(mFieldWidth), pendingOffset);
        Value * pendingSpaceFilled = iBuilder->CreateICmpUGE(newItemCount, pendingSpace);
        
        // Data from the ith swizzle pack of each group is processed
        // according to the same newItemCount, pendingSpace, ...
        for (unsigned j = 0; j < mSwizzleSetCount; j++) {
            Value * newItems = iBuilder->loadInputStreamBlock("inputSwizzle" + std::to_string(j), iBuilder->getInt32(i));
        //iBuilder->CallPrintRegister("newItems", newItems);
            // Combine as many of the new items as possible into the pending group.
            Value * combinedGroup = iBuilder->CreateOr(pendingData[j], iBuilder->CreateShl(newItems, iBuilder->simd_fill(mFieldWidth, pendingOffset)));
	    //iBuilder->CallPrintRegister("combinedGroup", combinedGroup);
            // To avoid an unpredictable branch, always store the combined group, whether full or not.
                
            iBuilder->CreateBlockAlignedStore(combinedGroup, iBuilder->CreateGEP(outputStreamPtr[j], outputIndex));
            // Any items in excess of the space available in the current pending group overflow for the next group.
            Value * overFlowGroup = iBuilder->CreateLShr(newItems, iBuilder->simd_fill(mFieldWidth, pendingSpace));
            // If we filled the space, then the overflow group becomes the new pending group and the index is updated.
            pendingData[j] = iBuilder->CreateSelect(pendingSpaceFilled, overFlowGroup, combinedGroup);
        }
        outputIndex = iBuilder->CreateSelect(pendingSpaceFilled, iBuilder->CreateAdd(outputIndex, iBuilder->getSize(1)), outputIndex);
        pendingOffset = iBuilder->CreateAnd(iBuilder->CreateAdd(newItemCount, pendingOffset), iBuilder->getSize(mFieldWidth-1));
    }
    iBuilder->setScalarField("pendingOffset", pendingOffset);   
    Value * newlyProduced = iBuilder->CreateSub(iBuilder->CreateShl(outputIndex, outputIndexShift), producedOffset);
    Value * produced = iBuilder->CreateAdd(outputProduced, newlyProduced);
    for (unsigned j = 0; j < mSwizzleSetCount; j++) {
        iBuilder->setScalarField("pendingSwizzleData" + std::to_string(j), pendingData[j]);
        //iBuilder->CallPrintRegister("pendingData[j]", pendingData[j]);

    }
    iBuilder->setProducedItemCount("outputSwizzle0", produced);
}

void SwizzledBitstreamCompressByCount::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder, Value * /* remainingBytes */) {
    CreateDoBlockMethodCall(iBuilder);
    Constant * blockOffsetMask = iBuilder->getSize(iBuilder->getBitBlockWidth() - 1);
    Constant * outputIndexShift = iBuilder->getSize(std::log2(mFieldWidth));
    
    Value * outputProduced = iBuilder->getProducedItemCount("outputSwizzle0"); // All output groups have the same count.
    Value * producedOffset = iBuilder->CreateAnd(outputProduced, blockOffsetMask);
    Value * outputIndex = iBuilder->CreateLShr(producedOffset, outputIndexShift);
    Value * pendingOffset = iBuilder->getScalarField("pendingOffset");

    // Write the pending data.
    for (unsigned i = 0; i < mSwizzleSetCount; i++) {
        Value * pendingData = iBuilder->getScalarField("pendingSwizzleData" + std::to_string(i));
        Value * outputStreamPtr = iBuilder->getOutputStreamBlockPtr("outputSwizzle" + std::to_string(i), iBuilder->getInt32(0));
        iBuilder->CreateBlockAlignedStore(pendingData, iBuilder->CreateGEP(outputStreamPtr, outputIndex));
    }
    iBuilder->setProducedItemCount("outputSwizzle0", iBuilder->CreateAdd(pendingOffset, outputProduced));
}
}
