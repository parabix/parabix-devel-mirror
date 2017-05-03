/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "deletion.h"
#include <IR_Gen/idisa_builder.h>
#include <llvm/IR/Value.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace kernel {

inline std::vector<Value *> parallel_prefix_deletion_masks(IDISA::IDISA_Builder * iBuilder, const unsigned fw, Value * del_mask) {
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

inline Value * apply_parallel_prefix_deletion(IDISA::IDISA_Builder * iBuilder, const unsigned fw, Value * del_mask, const std::vector<Value *> & mv, Value * strm) {
    Value * s = iBuilder->simd_and(strm, iBuilder->simd_not(del_mask));
    for (unsigned i = 0; i < mv.size(); i++) {
        unsigned shift = 1 << i;
        Value * t = iBuilder->simd_and(s, mv[i]);
        s = iBuilder->simd_or(iBuilder->simd_xor(s, t), iBuilder->simd_srli(fw, t, shift));
    }
    return s;
}

inline Value * partial_sum_popcount(IDISA::IDISA_Builder * iBuilder, const unsigned fw, Value * mask) {
    Value * field = iBuilder->simd_popcount(fw, mask);
    const auto count = iBuilder->getBitBlockWidth() / fw;
    for (unsigned move = 1; move < count; move *= 2) {
        field = iBuilder->simd_add(fw, field, iBuilder->mvmd_slli(fw, field, move));
    }
    return field;
}

// Apply deletion to a set of stream_count input streams to produce a set of output streams.
// Kernel inputs: stream_count data streams plus one del_mask stream
// Outputs: the deleted streams, plus a partial sum popcount

void DeletionKernel::generateDoBlockMethod() {
    Value * delMask = loadInputStreamBlock("delMaskSet", iBuilder->getInt32(0));
    const auto move_masks = parallel_prefix_deletion_masks(iBuilder, mDeletionFieldWidth, delMask);
    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * input = loadInputStreamBlock("inputStreamSet", iBuilder->getInt32(j));
        Value * output = apply_parallel_prefix_deletion(iBuilder, mDeletionFieldWidth, delMask, move_masks, input);
        storeOutputStreamBlock("outputStreamSet", iBuilder->getInt32(j), output);
    }
    Value * delCount = partial_sum_popcount(iBuilder, mDeletionFieldWidth, iBuilder->simd_not(delMask));
    storeOutputStreamBlock("deletionCounts", iBuilder->getInt32(0), iBuilder->bitCast(delCount));
}

void DeletionKernel::generateFinalBlockMethod(Value * remainingBytes) {
    IntegerType * vecTy = iBuilder->getIntNTy(iBuilder->getBitBlockWidth());
    Value * remaining = iBuilder->CreateZExt(remainingBytes, vecTy);
    Value * EOF_del = iBuilder->bitCast(iBuilder->CreateShl(Constant::getAllOnesValue(vecTy), remaining));
    Value * delMask = iBuilder->CreateOr(EOF_del, loadInputStreamBlock("delMaskSet", iBuilder->getInt32(0)));
    const auto move_masks = parallel_prefix_deletion_masks(iBuilder, mDeletionFieldWidth, delMask);
    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * input = loadInputStreamBlock("inputStreamSet", iBuilder->getInt32(j));
        Value * output = apply_parallel_prefix_deletion(iBuilder, mDeletionFieldWidth, delMask, move_masks, input);
        storeOutputStreamBlock("outputStreamSet", iBuilder->getInt32(j), output);
    }
    Value * delCount = partial_sum_popcount(iBuilder, mDeletionFieldWidth, iBuilder->simd_not(delMask));
    storeOutputStreamBlock("deletionCounts", iBuilder->getInt32(0), iBuilder->bitCast(delCount));
}

DeletionKernel::DeletionKernel(IDISA::IDISA_Builder * iBuilder, unsigned fw, unsigned streamCount)
: BlockOrientedKernel(iBuilder, "del" + std::to_string(fw) + "_" + std::to_string(streamCount),
              {Binding{iBuilder->getStreamSetTy(streamCount), "inputStreamSet"},
               Binding{iBuilder->getStreamSetTy(), "delMaskSet"}},
              {Binding{iBuilder->getStreamSetTy(streamCount), "outputStreamSet"},
               Binding{iBuilder->getStreamSetTy(), "deletionCounts"}},
              {}, {}, {})
, mDeletionFieldWidth(fw)
, mStreamCount(streamCount) {
}

const unsigned PEXT_width = 64;

inline std::vector<Value *> get_PEXT_masks(IDISA::IDISA_Builder * iBuilder, Value * del_mask) {
    Value * m = iBuilder->fwCast(PEXT_width, iBuilder->simd_not(del_mask));
    std::vector<Value *> masks;
    for (unsigned i = 0; i < iBuilder->getBitBlockWidth()/PEXT_width; i++) {
        masks.push_back(iBuilder->CreateExtractElement(m, i));
    }
    return masks;
}

// Apply PEXT deletion to a collection of blocks and swizzle the result.
// strms contains the blocks to process
inline std::vector<Value *> apply_PEXT_deletion_with_swizzle(IDISA::IDISA_Builder * iBuilder, const std::vector<Value *> & masks, std::vector<Value *> strms) {    
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

inline Value * apply_PEXT_deletion(IDISA::IDISA_Builder * iBuilder, const std::vector<Value *> & masks, Value * strm) {  
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

void DeleteByPEXTkernel::generateDoBlockMethod() {
    Value * delMask = loadInputStreamBlock("delMaskSet", iBuilder->getInt32(0));
    const auto masks = get_PEXT_masks(iBuilder, delMask);
    generateProcessingLoop(masks, delMask);
}

void DeleteByPEXTkernel::generateFinalBlockMethod(Value * remainingBytes) {
    IntegerType * vecTy = iBuilder->getIntNTy(iBuilder->getBitBlockWidth());
    Value * remaining = iBuilder->CreateZExt(remainingBytes, vecTy);
    Value * EOF_del = iBuilder->bitCast(iBuilder->CreateShl(Constant::getAllOnesValue(vecTy), remaining));
    Value * delMask = iBuilder->CreateOr(EOF_del, loadInputStreamBlock("delMaskSet", iBuilder->getInt32(0)));
    const auto masks = get_PEXT_masks(iBuilder, delMask);
    generateProcessingLoop(masks, delMask);
}

void DeleteByPEXTkernel::generateProcessingLoop(const std::vector<Value *> & masks, Value * delMask) {
    if (mShouldSwizzle)    
        generatePEXTAndSwizzleLoop(masks);
    else
        generatePEXTLoop(masks);    
    
    //Value * delCount = partial_sum_popcount(iBuilder, mDelCountFieldWidth, apply_PEXT_deletion(iBuilder, masks, iBuilder->simd_not(delMask)));
    Value * delCount = iBuilder->simd_popcount(mDelCountFieldWidth, iBuilder->simd_not(delMask));
    storeOutputStreamBlock("deletionCounts", iBuilder->getInt32(0), iBuilder->bitCast(delCount));
}

void DeleteByPEXTkernel::generatePEXTLoop(const std::vector<Value *> & masks) {
    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * input = loadInputStreamBlock("inputStreamSet", iBuilder->getInt32(j));
        Value * output = apply_PEXT_deletion(iBuilder, masks, input);
        storeOutputStreamBlock("outputStreamSet", iBuilder->getInt32(j), output);
    }
}

void DeleteByPEXTkernel::generatePEXTAndSwizzleLoop(const std::vector<Value *> & masks) {
    // Group blocks together into input vector. Input should contain mStreamCount/mSwizzleFactor blocks (e.g. for U8U16 16/4=4)
    // mStreamCount/mSwizzleFactor -> (mStreamCount + mSwizzleFactor - 1) / mSwizzleFactor
    for (unsigned j = 0; j < (mStreamCount + mSwizzleFactor - 1)/mSwizzleFactor; ++j) {
        std::vector<Value *> input;
        unsigned streamSelectionIndex = j * mSwizzleFactor;
        for (unsigned i = streamSelectionIndex; i < (streamSelectionIndex + mSwizzleFactor); ++i) {
	    	// Check if i > mStreamCount. If it is, add null streams until we get mStreamCount/mSwizzleFactor streams in the input vector
            if ( i >= mStreamCount)
				input.push_back(iBuilder->allZeroes());
			else
	    		input.push_back(loadInputStreamBlock("inputStreamSet", iBuilder->getInt32(i)));
        }
        std::vector<Value *> output = apply_PEXT_deletion_with_swizzle(iBuilder, masks, input);
        for (unsigned i = 0; i < mSwizzleFactor; i++) { 
             storeOutputStreamBlock(std::string(mOutputSwizzleNameBase) + std::to_string(j), iBuilder->getInt32(i), output[i]);
        }
    }
}

DeleteByPEXTkernel::DeleteByPEXTkernel(IDISA::IDISA_Builder * iBuilder, unsigned fw, unsigned streamCount, bool shouldSwizzle)
    : BlockOrientedKernel(iBuilder, "PEXTdel" + std::to_string(fw) + "_" + std::to_string(streamCount) + (shouldSwizzle ? "swiz" : "noswiz"),
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

    

SwizzledBitstreamCompressByCount::SwizzledBitstreamCompressByCount(IDISA::IDISA_Builder * iBuilder, unsigned bitStreamCount, unsigned fieldWidth)
    : BlockOrientedKernel(iBuilder, "swizzled_compress" + std::to_string(fieldWidth) + "_" + std::to_string(bitStreamCount),
                         {Binding{iBuilder->getStreamSetTy(), "countsPerStride"}}, {}, {}, {}, {})
, mBitStreamCount(bitStreamCount)
    , mFieldWidth(fieldWidth)
    , mSwizzleFactor(iBuilder->getBitBlockWidth() / fieldWidth)
    , mSwizzleSetCount((mBitStreamCount + mSwizzleFactor - 1)/mSwizzleFactor) {
        assert((fieldWidth > 0) && ((fieldWidth & (fieldWidth - 1)) == 0) && "fieldWidth must be a power of 2");
        assert(mSwizzleFactor > 1 && "fieldWidth must be less than the block width");
        mStreamSetInputs.push_back(Binding{iBuilder->getStreamSetTy(mSwizzleFactor, 1), "inputSwizzle0"});
        mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(mSwizzleFactor, 1), "outputSwizzle0", MaxRatio(1)});
        addScalar(iBuilder->getBitBlockType(), "pendingSwizzleData0");
        for (unsigned i = 1; i < mSwizzleSetCount; i++) {
            mStreamSetInputs.push_back(Binding{iBuilder->getStreamSetTy(mSwizzleFactor, 1), "inputSwizzle" + std::to_string(i)});
            mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(mSwizzleFactor, 1), "outputSwizzle" + std::to_string(i), FixedRatio(1, 1, "outputSwizzle0")});
            addScalar(iBuilder->getBitBlockType(), "pendingSwizzleData" + std::to_string(i));
        }
        addScalar(iBuilder->getSizeTy(), "pendingOffset");
}

    
void SwizzledBitstreamCompressByCount::generateDoBlockMethod() {
        
    Value * countStreamPtr = iBuilder->CreateBitCast(getInputStreamBlockPtr("countsPerStride", iBuilder->getInt32(0)), iBuilder->getIntNTy(mFieldWidth)->getPointerTo());
    
    // Output is written and committed to the output buffer one swizzle at a time.
    //
    Constant * blockOffsetMask = iBuilder->getSize(iBuilder->getBitBlockWidth() - 1);
    Constant * outputIndexShift = iBuilder->getSize(std::log2(mFieldWidth));
    
    Value * outputProduced = getProducedItemCount("outputSwizzle0"); // All output groups have the same count.
    Value * producedOffset = iBuilder->CreateAnd(outputProduced, blockOffsetMask);
    Value * outputIndex = iBuilder->CreateLShr(producedOffset, outputIndexShift);

    // There may be pending data in the kernel state, for up to mFieldWidth-1 bits per stream.
    Value * pendingOffset = getScalarField("pendingOffset");
    // There is a separate vector of pending data for each swizzle group.
    std::vector<Value *> pendingData;
    std::vector<Value *> outputStreamPtr;
    for (unsigned i = 0; i < mSwizzleSetCount; i++) {
        pendingData.push_back(getScalarField("pendingSwizzleData" + std::to_string(i)));
        outputStreamPtr.push_back(getOutputStreamBlockPtr("outputSwizzle" + std::to_string(i), iBuilder->getInt32(0)));
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
            Value * newItems = loadInputStreamBlock("inputSwizzle" + std::to_string(j), iBuilder->getInt32(i));
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
    setScalarField("pendingOffset", pendingOffset);
    
    Value * newlyProduced = iBuilder->CreateSub(iBuilder->CreateShl(outputIndex, outputIndexShift), producedOffset);
    Value * produced = iBuilder->CreateAdd(outputProduced, newlyProduced);
    for (unsigned j = 0; j < mSwizzleSetCount; j++) {
        setScalarField("pendingSwizzleData" + std::to_string(j), pendingData[j]);
    }
    setProducedItemCount("outputSwizzle0", produced);
}

void SwizzledBitstreamCompressByCount::generateFinalBlockMethod(Value * remainingBytes) {
    CreateDoBlockMethodCall();
    Constant * blockOffsetMask = iBuilder->getSize(iBuilder->getBitBlockWidth() - 1);
    Constant * outputIndexShift = iBuilder->getSize(std::log2(mFieldWidth));
    
    Value * outputProduced = getProducedItemCount("outputSwizzle0"); // All output groups have the same count.
    Value * producedOffset = iBuilder->CreateAnd(outputProduced, blockOffsetMask);
    Value * outputIndex = iBuilder->CreateLShr(producedOffset, outputIndexShift);
    Value * pendingOffset = getScalarField("pendingOffset");

    // Write the pending data.
    for (unsigned i = 0; i < mSwizzleSetCount; i++) {
        Value * pendingData = getScalarField("pendingSwizzleData" + std::to_string(i));
        Value * outputStreamPtr = getOutputStreamBlockPtr("outputSwizzle" + std::to_string(i), iBuilder->getInt32(0));
        iBuilder->CreateBlockAlignedStore(pendingData, iBuilder->CreateGEP(outputStreamPtr, outputIndex));
    }
    setProducedItemCount("outputSwizzle0", iBuilder->CreateAdd(pendingOffset, outputProduced));
}
}
