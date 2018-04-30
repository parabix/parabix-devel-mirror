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
    Value * unitCount = iBuilder->simd_popcount(mDeletionFieldWidth, iBuilder->simd_not(delMask));
    iBuilder->storeOutputStreamBlock("unitCounts", iBuilder->getInt32(0), iBuilder->bitCast(unitCount));
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
    Value * const unitCount = iBuilder->simd_popcount(mDeletionFieldWidth, iBuilder->simd_not(delMask));
    iBuilder->storeOutputStreamBlock("unitCounts", iBuilder->getInt32(0), iBuilder->bitCast(unitCount));
}

DeletionKernel::DeletionKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const unsigned fieldWidth, const unsigned streamCount)
: BlockOrientedKernel("del" + std::to_string(fieldWidth) + "_" + std::to_string(streamCount),
                      {Binding{iBuilder->getStreamSetTy(streamCount), "inputStreamSet"},
                          Binding{iBuilder->getStreamSetTy(), "delMaskSet"}},
                      {Binding{iBuilder->getStreamSetTy(streamCount), "outputStreamSet"},
                          Binding{iBuilder->getStreamSetTy(), "unitCounts", FixedRate(), RoundUpTo(iBuilder->getBitBlockWidth())}},
                      {}, {}, {})
, mDeletionFieldWidth(fieldWidth)
, mStreamCount(streamCount) {
}

    
    
SwizzledDeleteByPEXTkernel::SwizzledDeleteByPEXTkernel(const std::unique_ptr<kernel::KernelBuilder> & b, unsigned streamCount, unsigned PEXT_width)
: BlockOrientedKernel("PEXTdel" + std::to_string(PEXT_width) + "_" + std::to_string(streamCount),
                  {Binding{b->getStreamSetTy(), "delMaskSet"}, Binding{b->getStreamSetTy(streamCount), "inputStreamSet"}},
                  {}, {}, {}, {})
, mStreamCount(streamCount)
, mSwizzleFactor(b->getBitBlockWidth() / PEXT_width)
// add mSwizzleFactor - 1 to mStreamCount before dividing by mSwizzleFactor
// to prevent rounding errors.
, mSwizzleSetCount((mStreamCount + mSwizzleFactor - 1)/mSwizzleFactor)
, mPEXTWidth(PEXT_width)
{
    assert((mPEXTWidth > 0) && ((mPEXTWidth & (mPEXTWidth - 1)) == 0)
        && "mDelCountFieldWidth must be a power of 2");
    assert(mSwizzleFactor > 1 && "mDelCountFieldWidth must be less than the block width");
    assert((mPEXTWidth == 64 || mPEXTWidth == 32) && "PEXT width must be 32 or 64");

    // why, if we have 1 input stream, are there n output swizzle streams rather 1 of n?
    Type * const outputTy = b->getStreamSetTy(mSwizzleFactor, 1);

    mStreamSetOutputs.push_back(Binding{outputTy, "outputSwizzle0", BoundedRate(0, 1), BlockSize(PEXT_width)}); // PopcountOfNot("delMaskSet")
    addScalar(b->getBitBlockType(), "pendingSwizzleData0");
    for (unsigned i = 1; i < mSwizzleSetCount; i++) {
        mStreamSetOutputs.push_back(Binding{outputTy, "outputSwizzle" + std::to_string(i), RateEqualTo("outputSwizzle0"), BlockSize(PEXT_width)});
        addScalar(b->getBitBlockType(), "pendingSwizzleData" + std::to_string(i));
    }
    addScalar(b->getSizeTy(), "pendingOffset");
}

void SwizzledDeleteByPEXTkernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) {
    // We use delMask to apply the same PEXT delete operation to each stream in the input stream set
    Value * const delMask = b->loadInputStreamBlock("delMaskSet", b->getInt32(0));
    generateProcessingLoop(b, delMask, false);
}

void SwizzledDeleteByPEXTkernel::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & b, Value * remainingBytes) {
    IntegerType * const vecTy = b->getIntNTy(b->getBitBlockWidth());
    Value * const remaining = b->CreateZExt(remainingBytes, vecTy);
    Value * const EOFMask = b->bitCast(b->CreateShl(Constant::getAllOnesValue(vecTy), remaining));
    Value * const delMask = b->CreateOr(EOFMask, b->loadInputStreamBlock("delMaskSet", b->getInt32(0)));
    generateProcessingLoop(b, delMask, true);
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

void SwizzledDeleteByPEXTkernel::generateProcessingLoop(const std::unique_ptr<KernelBuilder> & b, Value * const delMask, const bool flush) {

    // selectors marks the positions we want to keep
    Value * const selectors = b->CreateNot(delMask);

    const auto swizzleSets = makeSwizzleSets(b, selectors);

    // Compress the PEXTedSwizzleSets
    // Output is written and committed to the output buffer one swizzle at a time.
    ConstantInt * const BLOCK_WIDTH_MASK = b->getSize(b->getBitBlockWidth() - 1);
    ConstantInt * const PEXT_WIDTH = b->getSize(mPEXTWidth);
    ConstantInt * const LOG_2_PEXT_WIDTH = b->getSize(std::log2(mPEXTWidth));
    ConstantInt * const LOG_2_SWIZZLE_FACTOR = b->getSize(std::log2(mSwizzleFactor));
    ConstantInt * const PEXT_WIDTH_MASK = b->getSize(mPEXTWidth - 1);

    // All output groups have the same count.
    Value * outputProduced = b->getProducedItemCount("outputSwizzle0");
    outputProduced = b->CreateAdd(outputProduced, b->getScalarField("pendingOffset"));
    Value * const producedOffset = b->CreateAnd(outputProduced, BLOCK_WIDTH_MASK);
    Value * outputIndex = b->CreateLShr(producedOffset, LOG_2_PEXT_WIDTH);

    // There is a separate vector of pending data for each swizzle group.
    std::vector<Value *> pendingData;
    for (unsigned i = 0; i < mSwizzleSetCount; i++) {
        pendingData.push_back(b->getScalarField("pendingSwizzleData" + std::to_string(i)));
    }

    Value * const newItemCounts = b->simd_popcount(mPEXTWidth, selectors);

    // For each row i
    for (unsigned i = 0; i < mSwizzleFactor; i++) {

        // Generate code for each of the mSwizzleFactor fields making up a block.
        // We load the count for the field and process all swizzle groups accordingly.
        Value * const pendingOffset = b->CreateAnd(outputProduced, PEXT_WIDTH_MASK);
        Value * const newItemCount = b->CreateExtractElement(newItemCounts, i);
        Value * const pendingSpace = b->CreateSub(PEXT_WIDTH, pendingOffset);
        Value * const pendingSpaceFilled = b->CreateICmpUGE(newItemCount, pendingSpace);

        Value * const swizzleIndex = b->CreateAnd(outputIndex, mSwizzleFactor - 1);
        Value * const blockOffset = b->CreateLShr(outputIndex, LOG_2_SWIZZLE_FACTOR);

        // Data from the ith swizzle pack of each group is processed
        // according to the same newItemCount, pendingSpace, ...
        for (unsigned j = 0; j < mSwizzleSetCount; j++) {
            Value * const newItems = swizzleSets[j][i];
            // Combine as many of the new items as possible into the pending group.
            Value * const shiftVector = b->simd_fill(mPEXTWidth, pendingOffset);
            Value * const shiftedItems = b->CreateShl(newItems, shiftVector);
            Value * const combinedGroup = b->CreateOr(pendingData[j], shiftedItems);
            // To avoid an unpredictable branch, always store the combined group, whether full or not.
            Value * const outputPtr = b->getOutputStreamBlockPtr("outputSwizzle" + std::to_string(j), swizzleIndex, blockOffset);
            b->CreateBlockAlignedStore(combinedGroup, outputPtr);
            // Any items in excess of the space available in the current pending group overflow for the next group.
            Value * overFlowGroup = b->CreateLShr(newItems, b->simd_fill(mPEXTWidth, pendingSpace));
            // If we filled the space, then the overflow group becomes the new pending group and the index is updated.
            pendingData[j] = b->CreateSelect(pendingSpaceFilled, overFlowGroup, combinedGroup);
        }

        Value * const swizzleIncrement = b->CreateZExt(pendingSpaceFilled, b->getSizeTy());
        outputIndex = b->CreateAdd(outputIndex, swizzleIncrement);

        outputProduced = b->CreateAdd(outputProduced, newItemCount);
    }

    if (flush) { // incase we selected the overflow group on the final iteration
        Value * const swizzleIndex = b->CreateAnd(outputIndex, mSwizzleFactor - 1);
        Value * const blockOffset = b->CreateLShr(outputIndex, LOG_2_SWIZZLE_FACTOR);
        for (unsigned i = 0; i < mSwizzleSetCount; i++) {
            Value * const outputPtr = b->getOutputStreamBlockPtr("outputSwizzle" + std::to_string(i), swizzleIndex, blockOffset);
            b->CreateBlockAlignedStore(pendingData[i], outputPtr);
        }
    } else {
        for (unsigned i = 0; i < mSwizzleSetCount; i++) {
            b->setScalarField("pendingSwizzleData" + std::to_string(i), pendingData[i]);
        }
        Value * const pendingOffset = b->CreateAnd(outputProduced, PEXT_WIDTH_MASK);
        b->setScalarField("pendingOffset", pendingOffset);
        // unless this is our final stride, don't report partially written fields.
        outputProduced = b->CreateAnd(outputProduced, b->CreateNot(PEXT_WIDTH_MASK));
    }

    b->setProducedItemCount("outputSwizzle0", outputProduced);
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

std::vector<std::vector<llvm::Value *>> SwizzledDeleteByPEXTkernel::makeSwizzleSets(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const selectors) {

    Constant * pext = nullptr;
    if (mPEXTWidth == 64) {
        pext = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pext_64);
    } else if (mPEXTWidth == 32) {
        pext = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pext_32);
    }

    Value * const m = b->fwCast(mPEXTWidth, selectors);

    std::vector<Value *> masks(mSwizzleFactor);
    for (unsigned i = 0; i < mSwizzleFactor; i++) {
        masks[i] = b->CreateExtractElement(m, i);

    }

    std::vector<std::vector<Value *>> swizzleSets;
    swizzleSets.reserve(mSwizzleSetCount);

    VectorType * const vecTy = b->fwVectorType(mPEXTWidth);

    UndefValue * const outputInitializer = UndefValue::get(vecTy);

    std::vector<Value *> input(mSwizzleFactor);
    // For each of the k swizzle sets required to apply PEXT to all input streams
    for (unsigned i = 0; i < mSwizzleSetCount; ++i) {

        for (unsigned j = 0; j < mSwizzleFactor; ++j) {
            const unsigned k = (i * mSwizzleFactor) + j;
            if (k < mStreamCount) {
                input[j] = b->CreateBitCast(b->loadInputStreamBlock("inputStreamSet", b->getInt32(k)), vecTy);
            } else {
                input[j] = Constant::getNullValue(vecTy);
            }
        }

        // TODO: if a SIMD pext instruction exists, we should first swizzle the lanes
        // then splat the pext mask and apply it to each output row

        std::vector<Value *> output(mSwizzleFactor, outputInitializer);
        // For each of the input streams
        for (unsigned j = 0; j < mSwizzleFactor; j++) {
            for (unsigned k = 0; k < mSwizzleFactor; k++) {
                // Load block j,k
                Value * const field = b->CreateExtractElement(input[j], k);
                // Apply PEXT deletion
                Value * const selected = b->CreateCall(pext, {field, masks[k]});
                // Then store it as our k,j-th output
                output[k] = b->CreateInsertElement(output[k], selected, j);
            }
        }

        swizzleSets.emplace_back(output);
    }

    return swizzleSets;
}

// Apply deletion to a set of stream_count input streams and produce a set of swizzled output streams.
// Kernel inputs: stream_count data streams plus one del_mask stream
// Outputs: swizzles containing the swizzled deleted streams, plus a partial sum popcount

void DeleteByPEXTkernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    Value * delMask = iBuilder->loadInputStreamBlock("delMaskSet", iBuilder->getInt32(0));
    generateProcessingLoop(iBuilder, delMask);
}

void DeleteByPEXTkernel::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> &iBuilder, Value * remainingBytes) {
    IntegerType * vecTy = iBuilder->getIntNTy(iBuilder->getBitBlockWidth());
    Value * remaining = iBuilder->CreateZExt(remainingBytes, vecTy);
    Value * EOF_del = iBuilder->bitCast(iBuilder->CreateShl(Constant::getAllOnesValue(vecTy), remaining));
    Value * delMask = iBuilder->CreateOr(EOF_del, iBuilder->loadInputStreamBlock("delMaskSet", iBuilder->getInt32(0)));
    generateProcessingLoop(iBuilder, delMask);
}

void DeleteByPEXTkernel::generateProcessingLoop(const std::unique_ptr<KernelBuilder> & iBuilder, Value * delMask) {
    Constant * PEXT_func = nullptr;
    if (mPEXTWidth == 64) {
        PEXT_func = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::x86_bmi_pext_64);
    } else if (mPEXTWidth == 32) {
        PEXT_func = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::x86_bmi_pext_32);
    }
    std::vector<Value *> masks(mSwizzleFactor);
    Value * const m = iBuilder->fwCast(mSwizzleFactor, iBuilder->simd_not(delMask));
    for (unsigned i = 0; i < mSwizzleFactor; i++) {
        masks.push_back(iBuilder->CreateExtractElement(m, i));
    }

    for (unsigned i = 0; i < mStreamCount; ++i) {
        Value * input = iBuilder->loadInputStreamBlock("inputStreamSet", iBuilder->getInt32(i));
        Value * value = iBuilder->fwCast(mPEXTWidth, input);
        Value * output = UndefValue::get(value->getType());
        for (unsigned j = 0; j < mSwizzleFactor; j++) {
            Value * field = iBuilder->CreateExtractElement(value, j);
            Value * compressed = iBuilder->CreateCall(PEXT_func, {field, masks[j]});
            output = iBuilder->CreateInsertElement(output, compressed, j);
        }
        iBuilder->storeOutputStreamBlock("outputStreamSet", iBuilder->getInt32(i), output);
    }
    Value * delCount = iBuilder->simd_popcount(mDelCountFieldWidth, iBuilder->simd_not(delMask));
    iBuilder->storeOutputStreamBlock("deletionCounts", iBuilder->getInt32(0), iBuilder->bitCast(delCount));
}

DeleteByPEXTkernel::DeleteByPEXTkernel(const std::unique_ptr<kernel::KernelBuilder> & b, unsigned fw, unsigned streamCount, unsigned PEXT_width)
: BlockOrientedKernel("PEXTdel" + std::to_string(fw) + "_" + std::to_string(streamCount) + "_" + std::to_string(PEXT_width),
              {Binding{b->getStreamSetTy(streamCount), "inputStreamSet"},
                  Binding{b->getStreamSetTy(), "delMaskSet"}},
              {}, {}, {}, {})
, mDelCountFieldWidth(fw)
, mStreamCount(streamCount)
, mSwizzleFactor(b->getBitBlockWidth() / PEXT_width)
, mPEXTWidth(PEXT_width) {
    mStreamSetOutputs.emplace_back(b->getStreamSetTy(mStreamCount), "outputStreamSet", PopcountOfNot("delMaskSet"));
    mStreamSetOutputs.emplace_back(b->getStreamSetTy(), "deletionCounts");
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
        Value * pendingSpace = iBuilder->CreateSub(iBuilder->getSize(mFieldWidth), pendingOffset);
        Value * pendingSpaceFilled = iBuilder->CreateICmpUGE(newItemCount, pendingSpace);
        
        // Data from the ith swizzle pack of each group is processed
        // according to the same newItemCount, pendingSpace, ...
        for (unsigned j = 0; j < mSwizzleSetCount; j++) {
            Value * newItems = iBuilder->loadInputStreamBlock("inputSwizzle" + std::to_string(j), iBuilder->getInt32(i));
            // Combine as many of the new items as possible into the pending group.
            Value * combinedGroup = iBuilder->CreateOr(pendingData[j], iBuilder->CreateShl(newItems, iBuilder->simd_fill(mFieldWidth, pendingOffset)));
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
