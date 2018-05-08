/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "deletion.h"
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace kernel {

inline std::vector<Value *> parallel_prefix_deletion_masks(const std::unique_ptr<KernelBuilder> & kb, const unsigned fw, Value * del_mask) {
    Value * m = kb->simd_not(del_mask);
    Value * mk = kb->simd_slli(fw, del_mask, 1);
    std::vector<Value *> move_masks;
    for (unsigned shift = 1; shift < fw; shift *= 2) {
        Value * mp = mk;
        for (unsigned lookright = 1; lookright < fw; lookright *= 2) {
            mp = kb->simd_xor(mp, kb->simd_slli(fw, mp, lookright));
        }
        Value * mv = kb->simd_and(mp, m);
        m = kb->simd_or(kb->simd_xor(m, mv), kb->simd_srli(fw, mv, shift));
        mk = kb->simd_and(mk, kb->simd_not(mp));
        move_masks.push_back(mv);
    }
    return move_masks;
}

inline Value * apply_parallel_prefix_deletion(const std::unique_ptr<KernelBuilder> & kb, const unsigned fw, Value * del_mask, const std::vector<Value *> & mv, Value * strm) {
    Value * s = kb->simd_and(strm, kb->simd_not(del_mask));
    for (unsigned i = 0; i < mv.size(); i++) {
        unsigned shift = 1 << i;
        Value * t = kb->simd_and(s, mv[i]);
        s = kb->simd_or(kb->simd_xor(s, t), kb->simd_srli(fw, t, shift));
    }
    return s;
}

// Apply deletion to a set of stream_count input streams to produce a set of output streams.
// Kernel inputs: stream_count data streams plus one del_mask stream
// Outputs: the deleted streams, plus a partial sum popcount

void DeletionKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & kb) {
    Value * delMask = kb->loadInputStreamBlock("delMaskSet", kb->getInt32(0));
    const auto move_masks = parallel_prefix_deletion_masks(kb, mDeletionFieldWidth, delMask);
    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * input = kb->loadInputStreamBlock("inputStreamSet", kb->getInt32(j));
        Value * output = apply_parallel_prefix_deletion(kb, mDeletionFieldWidth, delMask, move_masks, input);
        kb->storeOutputStreamBlock("outputStreamSet", kb->getInt32(j), output);
    }
    Value * unitCount = kb->simd_popcount(mDeletionFieldWidth, kb->simd_not(delMask));
    kb->storeOutputStreamBlock("unitCounts", kb->getInt32(0), kb->bitCast(unitCount));
}

void DeletionKernel::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & kb, Value * remainingBytes) {
    IntegerType * vecTy = kb->getIntNTy(kb->getBitBlockWidth());
    Value * remaining = kb->CreateZExt(remainingBytes, vecTy);
    Value * EOF_del = kb->bitCast(kb->CreateShl(Constant::getAllOnesValue(vecTy), remaining));
    Value * delMask = kb->CreateOr(EOF_del, kb->loadInputStreamBlock("delMaskSet", kb->getInt32(0)));
    const auto move_masks = parallel_prefix_deletion_masks(kb, mDeletionFieldWidth, delMask);
    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * input = kb->loadInputStreamBlock("inputStreamSet", kb->getInt32(j));
        Value * output = apply_parallel_prefix_deletion(kb, mDeletionFieldWidth, delMask, move_masks, input);
        kb->storeOutputStreamBlock("outputStreamSet", kb->getInt32(j), output);
    }
    Value * const unitCount = kb->simd_popcount(mDeletionFieldWidth, kb->simd_not(delMask));
    kb->storeOutputStreamBlock("unitCounts", kb->getInt32(0), kb->bitCast(unitCount));
}

DeletionKernel::DeletionKernel(const std::unique_ptr<kernel::KernelBuilder> & kb, const unsigned fieldWidth, const unsigned streamCount)
: BlockOrientedKernel("del" + std::to_string(fieldWidth) + "_" + std::to_string(streamCount),
                      {Binding{kb->getStreamSetTy(streamCount), "inputStreamSet"},
                          Binding{kb->getStreamSetTy(), "delMaskSet"}},
                      {Binding{kb->getStreamSetTy(streamCount), "outputStreamSet"},
                          Binding{kb->getStreamSetTy(), "unitCounts", FixedRate(), RoundUpTo(kb->getBitBlockWidth())}},
                      {}, {}, {})
, mDeletionFieldWidth(fieldWidth)
, mStreamCount(streamCount) {
}

void FieldCompressKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfBlocks) {
    BasicBlock * entry = kb->GetInsertBlock();
    BasicBlock * processBlock = kb->CreateBasicBlock("processBlock");
    BasicBlock * done = kb->CreateBasicBlock("done");
    Constant * const ZERO = kb->getSize(0);
    kb->CreateBr(processBlock);
    kb->SetInsertPoint(processBlock);
    PHINode * blockOffsetPhi = kb->CreatePHI(kb->getSizeTy(), 2);
    blockOffsetPhi->addIncoming(ZERO, entry);
    Value * extractionMask = kb->loadInputStreamBlock("extractionMask", ZERO, blockOffsetPhi);
    Value * delMask = kb->simd_not(extractionMask);
    const auto move_masks = parallel_prefix_deletion_masks(kb, mCompressFieldWidth, delMask);
    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * input = kb->loadInputStreamBlock("inputStreamSet", kb->getInt32(j), blockOffsetPhi);
        Value * output = apply_parallel_prefix_deletion(kb, mCompressFieldWidth, delMask, move_masks, input);
        kb->storeOutputStreamBlock("outputStreamSet", kb->getInt32(j), blockOffsetPhi, output);
    }
    Value * unitCount = kb->simd_popcount(mCompressFieldWidth, extractionMask);
    kb->storeOutputStreamBlock("unitCounts", kb->getInt32(0), blockOffsetPhi, kb->bitCast(unitCount));
    Value * nextBlk = kb->CreateAdd(blockOffsetPhi, kb->getSize(1));
    blockOffsetPhi->addIncoming(nextBlk, processBlock);
    Value * moreToDo = kb->CreateICmpNE(nextBlk, numOfBlocks);
    kb->CreateCondBr(moreToDo, processBlock, done);
    kb->SetInsertPoint(done);
}

FieldCompressKernel::FieldCompressKernel(const std::unique_ptr<kernel::KernelBuilder> & kb, const unsigned fieldWidth, const unsigned streamCount)
: MultiBlockKernel("fieldCompress" + std::to_string(fieldWidth) + "_" + std::to_string(streamCount),
                      {Binding{kb->getStreamSetTy(streamCount), "inputStreamSet"},
                          Binding{kb->getStreamSetTy(), "extractionMask"}},
                      {Binding{kb->getStreamSetTy(streamCount), "outputStreamSet"},
                          Binding{kb->getStreamSetTy(), "unitCounts", FixedRate(), RoundUpTo(kb->getBitBlockWidth())}},
                      {}, {}, {})
, mCompressFieldWidth(fieldWidth)
, mStreamCount(streamCount) {
}

void PEXTFieldCompressKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfBlocks) {
    Type * fieldTy = kb->getIntNTy(mPEXTWidth);
    Type * fieldPtrTy = PointerType::get(fieldTy, 0);
    Constant * PEXT_func = nullptr;
    Constant * popc_func = Intrinsic::getDeclaration(getModule(), Intrinsic::ctpop, fieldTy);
    if (mPEXTWidth == 64) {
        PEXT_func = Intrinsic::getDeclaration(kb->getModule(), Intrinsic::x86_bmi_pext_64);
    } else if (mPEXTWidth == 32) {
        PEXT_func = Intrinsic::getDeclaration(kb->getModule(), Intrinsic::x86_bmi_pext_32);
    }
    BasicBlock * entry = kb->GetInsertBlock();
    BasicBlock * processBlock = kb->CreateBasicBlock("processBlock");
    BasicBlock * done = kb->CreateBasicBlock("done");
    Constant * const ZERO = kb->getSize(0);
    const unsigned fieldsPerBlock = kb->getBitBlockWidth()/mPEXTWidth;
    kb->CreateBr(processBlock);
    kb->SetInsertPoint(processBlock);
    PHINode * blockOffsetPhi = kb->CreatePHI(kb->getSizeTy(), 2);
    blockOffsetPhi->addIncoming(ZERO, entry);
    std::vector<Value *> mask(fieldsPerBlock);
    Value * extractionMaskPtr = kb->getInputStreamBlockPtr("extractionMask", ZERO, blockOffsetPhi);
    extractionMaskPtr = kb->CreatePointerCast(extractionMaskPtr, fieldPtrTy);
    Value * unitCountPtr = kb->getOutputStreamBlockPtr("unitCounts", ZERO, blockOffsetPhi);
    unitCountPtr = kb->CreatePointerCast(unitCountPtr, fieldPtrTy);
    for (unsigned i = 0; i < fieldsPerBlock; i++) {
        mask[i] = kb->CreateLoad(kb->CreateGEP(extractionMaskPtr, kb->getInt32(i)));
        Value * popc = kb->CreateCall(popc_func, mask[i]);
        kb->CreateStore(popc, kb->CreateGEP(unitCountPtr, kb->getInt32(i)));
    }
    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * inputPtr = kb->getInputStreamBlockPtr("inputStreamSet", kb->getInt32(j), blockOffsetPhi);
        inputPtr = kb->CreatePointerCast(inputPtr, fieldPtrTy);
        Value * outputPtr = kb->getOutputStreamBlockPtr("outputStreamSet", kb->getInt32(j), blockOffsetPhi);
        outputPtr = kb->CreatePointerCast(outputPtr, fieldPtrTy);
        for (unsigned i = 0; i < fieldsPerBlock; i++) {
            Value * field = kb->CreateLoad(kb->CreateGEP(inputPtr, kb->getInt32(i)));
            Value * compressed = kb->CreateCall(PEXT_func, {field, mask[i]});
            kb->CreateStore(compressed, kb->CreateGEP(outputPtr, kb->getInt32(i)));
        }
    }
    Value * nextBlk = kb->CreateAdd(blockOffsetPhi, kb->getSize(1));
    blockOffsetPhi->addIncoming(nextBlk, processBlock);
    Value * moreToDo = kb->CreateICmpNE(nextBlk, numOfBlocks);
    kb->CreateCondBr(moreToDo, processBlock, done);
    kb->SetInsertPoint(done);
}

PEXTFieldCompressKernel::PEXTFieldCompressKernel(const std::unique_ptr<kernel::KernelBuilder> & kb, const unsigned fieldWidth, const unsigned streamCount)
: MultiBlockKernel("PEXTfieldCompress" + std::to_string(fieldWidth) + "_" + std::to_string(streamCount),
                   {Binding{kb->getStreamSetTy(streamCount), "inputStreamSet"},
                       Binding{kb->getStreamSetTy(), "extractionMask"}},
                   {Binding{kb->getStreamSetTy(streamCount), "outputStreamSet"},
                       Binding{kb->getStreamSetTy(), "unitCounts", FixedRate(), RoundUpTo(kb->getBitBlockWidth())}},
                   {}, {}, {})
, mPEXTWidth(fieldWidth)
, mStreamCount(streamCount) {
    if ((fieldWidth != 32) && (fieldWidth != 64)) llvm::report_fatal_error("Unsupported PEXT width for PEXTFieldCompressKernel");
}
    
StreamCompressKernel::StreamCompressKernel(const std::unique_ptr<kernel::KernelBuilder> & kb, const unsigned fieldWidth, const unsigned streamCount)
: MultiBlockKernel("streamCompress" + std::to_string(fieldWidth) + "_" + std::to_string(streamCount),
                   {Binding{kb->getStreamSetTy(streamCount), "sourceStreamSet"},
                       Binding{kb->getStreamSetTy(), "unitCounts"}},
                   {Binding{kb->getStreamSetTy(streamCount), "compressedOutput", BoundedRate(0, 1)}},
                   {}, {}, {})
, mCompressedFieldWidth(fieldWidth)
, mStreamCount(streamCount) {
    addScalar(kb->getSizeTy(), "pendingItemCount");
    for (unsigned i = 0; i < streamCount; i++) {
        addScalar(kb->getBitBlockType(), "pendingOutputBlock_" + std::to_string(i));
    }

}
    
void StreamCompressKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfBlocks) {
    const unsigned fw = mCompressedFieldWidth;
    Type * fwTy = b->getIntNTy(fw);
    Type * sizeTy = b->getSizeTy();
    const unsigned numFields = b->getBitBlockWidth()/fw;
    Constant * zeroSplat = Constant::getNullValue(b->fwVectorType(fw));
    Constant * oneSplat = ConstantVector::getSplat(numFields, ConstantInt::get(fwTy, 1));
    Constant * fwSplat = ConstantVector::getSplat(numFields, ConstantInt::get(fwTy, fw));
    Constant * numFieldConst = ConstantInt::get(sizeTy, numFields);
    Constant * fwMaskSplat = ConstantVector::getSplat(numFields, ConstantInt::get(fwTy, fw-1));
    Constant * bitBlockWidthConst = ConstantInt::get(sizeTy, b->getBitBlockWidth());
    BasicBlock * entry = b->GetInsertBlock();
    BasicBlock * segmentLoop = b->CreateBasicBlock("segmentLoop");
    BasicBlock * segmentDone = b->CreateBasicBlock("segmentDone");
    BasicBlock * finalWrite = b->CreateBasicBlock("finalWrite");
    BasicBlock * updateProducedCount = b->CreateBasicBlock("updateProducedCount");
    Constant * const ZERO = b->getSize(0);
    
    Value * pendingItemCount = b->getScalarField("pendingItemCount");
    std::vector<Value *> pendingData(mStreamCount);
    for (unsigned i = 0; i < mStreamCount; i++) {
        pendingData[i] = b->getScalarField("pendingOutputBlock_" + std::to_string(i));
    }
    
    b->CreateBr(segmentLoop);
    // Main Loop
    b->SetInsertPoint(segmentLoop);
    PHINode * blockOffsetPhi = b->CreatePHI(b->getSizeTy(), 2);
    PHINode * outputBlockPhi = b->CreatePHI(b->getSizeTy(), 2);
    PHINode * pendingItemsPhi = b->CreatePHI(b->getSizeTy(), 2);
    PHINode * pendingDataPhi[mStreamCount];
    blockOffsetPhi->addIncoming(ZERO, entry);
    outputBlockPhi->addIncoming(ZERO, entry);
    pendingItemsPhi->addIncoming(pendingItemCount, entry);
    for (unsigned i = 0; i < mStreamCount; i++) {
        pendingDataPhi[i] = b->CreatePHI(b->getBitBlockType(), 2);
        pendingDataPhi[i]->addIncoming(pendingData[i], entry);
    }
    Value * fieldPopCounts = b->loadInputStreamBlock("unitCounts", ZERO, blockOffsetPhi);
    // For each field determine the (partial) sum popcount of all fields up to and
    // including the current field.
    Value * partialSum = fieldPopCounts;
    for (unsigned i = 1; i < numFields; i *= 2) {
        partialSum = b->simd_add(fw, partialSum, b->mvmd_slli(fw, partialSum, i));
    }
    Value * blockPopCount = b->CreateZExtOrTrunc(b->CreateExtractElement(partialSum, numFields-1), sizeTy);
    //
    // Now determine for each source field the output offset of the first bit.
    // Note that this depends on the number of pending bits.
    //
    Value * pendingOffset = b->CreateURem(pendingItemsPhi, ConstantInt::get(sizeTy, fw));
    Value * splatPending = b->simd_fill(fw, b->CreateZExtOrTrunc(pendingOffset, fwTy));
    Value * pendingFieldIdx = b->CreateUDiv(pendingItemsPhi, ConstantInt::get(sizeTy, fw));
    Value * offsets = b->simd_add(fw, b->mvmd_slli(fw, partialSum, 1), splatPending);
    offsets = b->simd_and(offsets, fwMaskSplat); // parallel URem fw
   //
    // Determine the relative field number for each output field.   Note that the total
    // number of fields involved is numFields + 1.   However, the first field always
    // be immediately combined into the current pending data field, so we calculate
    // field numbers for all subsequent fields, (the fields that receive overflow bits).
    Value * fieldNo = b->simd_srli(fw, b->simd_add(fw, partialSum, splatPending), std::log2(fw));
  //
    // Now process the input data block of each stream in the input stream set.
    //
    // First load all the stream set blocks and the pending data.
    std::vector<Value *> sourceBlock(mStreamCount);
    for (unsigned i = 0; i < mStreamCount; i++) {
        sourceBlock[i] = b->loadInputStreamBlock("sourceStreamSet", b->getInt32(i), blockOffsetPhi);
    }
    // Now separate the bits of each field into ones that go into the current field
    // and ones that go into the overflow field.   Extract the first field separately,
    // and then shift and combine subsequent fields.
    std::vector<Value *> pendingOutput(mStreamCount);
    std::vector<Value *> outputFields(mStreamCount);
    Value * backShift = b->simd_sub(fw, fwSplat, offsets);
    for (unsigned i = 0; i < mStreamCount; i++) {
        Value * currentFieldBits = b->simd_sllv(fw, sourceBlock[i], offsets);
        Value * nextFieldBits = b->simd_srlv(fw, sourceBlock[i], backShift);
        Value * firstField = b->mvmd_extract(fw, currentFieldBits, 0);
        Value * vec1 = b->CreateInsertElement(zeroSplat, firstField, pendingFieldIdx);
        pendingOutput[i] = b->simd_or(pendingDataPhi[i], vec1);
        // shift back currentFieldBits to combine with nextFieldBits.
        outputFields[i] = b->simd_or(b->mvmd_srli(fw, currentFieldBits, 1), nextFieldBits);
    }
    // Now combine forward all fields with the same field number.  This may require
    // up to log2 numFields steps.
    for (unsigned j = 1; j < numFields; j*=2) {
        Value * select = b->simd_eq(fw, fieldNo, b->mvmd_slli(fw, fieldNo, j));
        for (unsigned i = 0; i < mStreamCount; i++) {
            Value * fields_fwd = b->mvmd_slli(fw, outputFields[i], j);
            outputFields[i] = b->simd_or(outputFields[i], b->simd_and(select, fields_fwd));
       }
    }
    // Now compress the data fields, eliminating all but the last field from
    // each run of consecutive field having the same field number as a subsequent field.
    // But it may be that last field number is 0 which will compare equal to a 0 shifted in.
    // So we add 1 to field numbers first.
    Value * nonZeroFieldNo = b->simd_add(fw, fieldNo, oneSplat);
    Value * eqNext = b->simd_eq(fw, nonZeroFieldNo, b->mvmd_srli(fw, nonZeroFieldNo, 1));
    Value * compressMask = b->hsimd_signmask(fw, b->simd_not(eqNext));
    for (unsigned i = 0; i < mStreamCount; i++) {
        outputFields[i] = b->mvmd_compress(fw, outputFields[i], compressMask);
    }
    //
    // Finally combine the pendingOutput and outputField data.
    // (a) shift forward outputField data to fill the pendingOutput values.
    // (b) shift back outputField data to clear data added to pendingOutput.
    //
    // However, we may need to increment pendingFieldIndex if we previously
    // filled the field with the extracted firstField value.  The first
    // value of the fieldNo vector will be 0 or 1.
    // It is possible that pendingFieldIndex will reach the total number
    // of fields held in register.  mvmd_sll may not handle this if it
    // translates to an LLVM shl.
    Value * increment = b->CreateZExtOrTrunc(b->mvmd_extract(fw, fieldNo, 0), sizeTy);
    pendingFieldIdx = b->CreateAdd(pendingFieldIdx, increment);
    Value * const pendingSpaceFilled = b->CreateICmpEQ(pendingFieldIdx, numFieldConst);
    Value * shftBack = b->CreateSub(numFieldConst, pendingFieldIdx);
    for (unsigned i = 0; i < mStreamCount; i++) {
        Value * outputFwd = b->mvmd_sll(fw, outputFields[i], pendingFieldIdx);
        outputFwd = b->CreateSelect(pendingSpaceFilled, zeroSplat, outputFwd);
        pendingOutput[i] = b->simd_or(pendingOutput[i], outputFwd);
        outputFields[i] = b->mvmd_srl(fw, outputFields[i], shftBack);
    }
    //
    // Write the pendingOutput data to outputStream.
    // Note: this data may be overwritten later, but we avoid branching.
    for (unsigned i = 0; i < mStreamCount; i++) {
        b->storeOutputStreamBlock("compressedOutput", b->getInt32(i), outputBlockPhi, pendingOutput[i]);
    }
    // Now determine the total amount of pending items and whether
    // the pending data all fits within the pendingOutput.
    Value * newPending = b->CreateAdd(pendingItemsPhi, blockPopCount);
    Value * doesFit = b->CreateICmpULT(newPending, bitBlockWidthConst);
    newPending = b->CreateSelect(doesFit, newPending, b->CreateSub(newPending, bitBlockWidthConst));
    //
    // Prepare Phi nodes for the next iteration.
    //
    Value * nextBlk = b->CreateAdd(blockOffsetPhi, b->getSize(1));
    blockOffsetPhi->addIncoming(nextBlk, segmentLoop);
    Value * nextOutputBlk = b->CreateAdd(outputBlockPhi, b->getSize(1));
    // But don't advance the output if all the data does fit into pendingOutput.
    nextOutputBlk = b->CreateSelect(doesFit, outputBlockPhi, nextOutputBlk);
    outputBlockPhi->addIncoming(nextOutputBlk, segmentLoop);
    pendingItemsPhi->addIncoming(newPending, segmentLoop);

    for (unsigned i = 0; i < mStreamCount; i++) {
        pendingOutput[i] = b->CreateSelect(doesFit, b->fwCast(fw, pendingOutput[i]), b->fwCast(fw, outputFields[i]));
        pendingDataPhi[i]->addIncoming(b->bitCast(pendingOutput[i]), segmentLoop);
    }
    //
    // Now continue the loop if there are more blocks to process.
    Value * moreToDo = b->CreateICmpNE(nextBlk, numOfBlocks);
    b->CreateCondBr(moreToDo, segmentLoop, segmentDone);
    
    b->SetInsertPoint(segmentDone);
    // Save kernel state.
    b->setScalarField("pendingItemCount", newPending);
    for (unsigned i = 0; i < mStreamCount; i++) {
        b->setScalarField("pendingOutputBlock_" + std::to_string(i), b->bitCast(pendingOutput[i]));
    }
    b->CreateCondBr(mIsFinal, finalWrite, updateProducedCount);
    b->SetInsertPoint(finalWrite);
    for (unsigned i = 0; i < mStreamCount; i++) {
        //Value * pending = b->getScalarField("pendingOutputBlock_" + std::to_string(i));
        Value * pending = b->bitCast(pendingOutput[i]);
        b->storeOutputStreamBlock("compressedOutput", b->getInt32(i), nextOutputBlk, pending);
    }
    b->CreateBr(updateProducedCount);
    b->SetInsertPoint(updateProducedCount);
     Value * produced = b->getProducedItemCount("compressedOutput");
    produced = b->CreateAdd(produced, b->CreateMul(nextOutputBlk, bitBlockWidthConst));
    produced = b->CreateSelect(mIsFinal, b->CreateAdd(produced, newPending), produced);
    b->setProducedItemCount("compressedOutput", produced);
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

void DeleteByPEXTkernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & kb) {
    Value * delMask = kb->loadInputStreamBlock("delMaskSet", kb->getInt32(0));
    generateProcessingLoop(kb, delMask);
}

void DeleteByPEXTkernel::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> &kb, Value * remainingBytes) {
    IntegerType * vecTy = kb->getIntNTy(kb->getBitBlockWidth());
    Value * remaining = kb->CreateZExt(remainingBytes, vecTy);
    Value * EOF_del = kb->bitCast(kb->CreateShl(Constant::getAllOnesValue(vecTy), remaining));
    Value * delMask = kb->CreateOr(EOF_del, kb->loadInputStreamBlock("delMaskSet", kb->getInt32(0)));
    generateProcessingLoop(kb, delMask);
}

void DeleteByPEXTkernel::generateProcessingLoop(const std::unique_ptr<KernelBuilder> & kb, Value * delMask) {
    Constant * PEXT_func = nullptr;
    if (mPEXTWidth == 64) {
        PEXT_func = Intrinsic::getDeclaration(kb->getModule(), Intrinsic::x86_bmi_pext_64);
    } else if (mPEXTWidth == 32) {
        PEXT_func = Intrinsic::getDeclaration(kb->getModule(), Intrinsic::x86_bmi_pext_32);
    }
    std::vector<Value *> masks(mSwizzleFactor);
    Value * const m = kb->fwCast(mSwizzleFactor, kb->simd_not(delMask));
    for (unsigned i = 0; i < mSwizzleFactor; i++) {
        masks.push_back(kb->CreateExtractElement(m, i));
    }

    for (unsigned i = 0; i < mStreamCount; ++i) {
        Value * input = kb->loadInputStreamBlock("inputStreamSet", kb->getInt32(i));
        Value * value = kb->fwCast(mPEXTWidth, input);
        Value * output = UndefValue::get(value->getType());
        for (unsigned j = 0; j < mSwizzleFactor; j++) {
            Value * field = kb->CreateExtractElement(value, j);
            Value * compressed = kb->CreateCall(PEXT_func, {field, masks[j]});
            output = kb->CreateInsertElement(output, compressed, j);
        }
        kb->storeOutputStreamBlock("outputStreamSet", kb->getInt32(i), output);
    }
    Value * delCount = kb->simd_popcount(mDelCountFieldWidth, kb->simd_not(delMask));
    kb->storeOutputStreamBlock("deletionCounts", kb->getInt32(0), kb->bitCast(delCount));
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

SwizzledBitstreamCompressByCount::SwizzledBitstreamCompressByCount(const std::unique_ptr<kernel::KernelBuilder> & kb, unsigned bitStreamCount, unsigned fieldWidth)
: BlockOrientedKernel("swizzled_compress" + std::to_string(fieldWidth) + "_" + std::to_string(bitStreamCount),
                     {Binding{kb->getStreamSetTy(), "countsPerStride"}}, {}, {}, {}, {})
, mBitStreamCount(bitStreamCount)
    , mFieldWidth(fieldWidth)
    , mSwizzleFactor(kb->getBitBlockWidth() / fieldWidth)
    , mSwizzleSetCount((mBitStreamCount + mSwizzleFactor - 1)/mSwizzleFactor) {
        assert((fieldWidth > 0) && ((fieldWidth & (fieldWidth - 1)) == 0) && "fieldWidth must be a power of 2");
        assert(mSwizzleFactor > 1 && "fieldWidth must be less than the block width");
        mStreamSetInputs.push_back(Binding{kb->getStreamSetTy(mSwizzleFactor, 1), "inputSwizzle0"});
        mStreamSetOutputs.push_back(Binding{kb->getStreamSetTy(mSwizzleFactor, 1), "outputSwizzle0", BoundedRate(0, 1)});
        addScalar(kb->getBitBlockType(), "pendingSwizzleData0");
        for (unsigned i = 1; i < mSwizzleSetCount; i++) {
            mStreamSetInputs.push_back(Binding{kb->getStreamSetTy(mSwizzleFactor, 1), "inputSwizzle" + std::to_string(i)});
            mStreamSetOutputs.push_back(Binding{kb->getStreamSetTy(mSwizzleFactor, 1), "outputSwizzle" + std::to_string(i), RateEqualTo("outputSwizzle0")});
            addScalar(kb->getBitBlockType(), "pendingSwizzleData" + std::to_string(i));
        }
        addScalar(kb->getSizeTy(), "pendingOffset");
}
    
void SwizzledBitstreamCompressByCount::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & kb) {
        
    Value * countsPerStridePtr = kb->getInputStreamBlockPtr("countsPerStride", kb->getInt32(0));
    Value * countStreamPtr = kb->CreatePointerCast(countsPerStridePtr, kb->getIntNTy(mFieldWidth)->getPointerTo());
    
    // Output is written and committed to the output buffer one swizzle at a time.
    //
    Constant * blockOffsetMask = kb->getSize(kb->getBitBlockWidth() - 1);
    Constant * outputIndexShift = kb->getSize(std::log2(mFieldWidth));
    
    Value * outputProduced = kb->getProducedItemCount("outputSwizzle0"); // All output groups have the same count.
    Value * producedOffset = kb->CreateAnd(outputProduced, blockOffsetMask);
    Value * outputIndex = kb->CreateLShr(producedOffset, outputIndexShift);

    // There may be pending data in the kernel state, for up to mFieldWidth-1 bits per stream.
    Value * pendingOffset = kb->getScalarField("pendingOffset");
    // There is a separate vector of pending data for each swizzle group.
    std::vector<Value *> pendingData;
    std::vector<Value *> outputStreamPtr;
    for (unsigned i = 0; i < mSwizzleSetCount; i++) {
        pendingData.push_back(kb->getScalarField("pendingSwizzleData" + std::to_string(i)));
        outputStreamPtr.push_back(kb->getOutputStreamBlockPtr("outputSwizzle" + std::to_string(i), kb->getInt32(0)));
    }
    
    // Generate code for each of the mSwizzleFactor fields making up a block.
    // We load the count for the field and process all swizzle groups accordingly.
    for (unsigned i = 0; i < mSwizzleFactor; i++) {
        Value * newItemCount = kb->CreateLoad(kb->CreateGEP(countStreamPtr, kb->getInt32(i)));
        Value * pendingSpace = kb->CreateSub(kb->getSize(mFieldWidth), pendingOffset);
        Value * pendingSpaceFilled = kb->CreateICmpUGE(newItemCount, pendingSpace);
        
        // Data from the ith swizzle pack of each group is processed
        // according to the same newItemCount, pendingSpace, ...
        for (unsigned j = 0; j < mSwizzleSetCount; j++) {
            Value * newItems = kb->loadInputStreamBlock("inputSwizzle" + std::to_string(j), kb->getInt32(i));
            // Combine as many of the new items as possible into the pending group.
            Value * combinedGroup = kb->CreateOr(pendingData[j], kb->CreateShl(newItems, kb->simd_fill(mFieldWidth, pendingOffset)));
            // To avoid an unpredictable branch, always store the combined group, whether full or not.               
            kb->CreateBlockAlignedStore(combinedGroup, kb->CreateGEP(outputStreamPtr[j], outputIndex));
            // Any items in excess of the space available in the current pending group overflow for the next group.
            Value * overFlowGroup = kb->CreateLShr(newItems, kb->simd_fill(mFieldWidth, pendingSpace));
            // If we filled the space, then the overflow group becomes the new pending group and the index is updated.
            pendingData[j] = kb->CreateSelect(pendingSpaceFilled, overFlowGroup, combinedGroup);
        }
        outputIndex = kb->CreateSelect(pendingSpaceFilled, kb->CreateAdd(outputIndex, kb->getSize(1)), outputIndex);
        pendingOffset = kb->CreateAnd(kb->CreateAdd(newItemCount, pendingOffset), kb->getSize(mFieldWidth-1));
    }
    kb->setScalarField("pendingOffset", pendingOffset);
    Value * newlyProduced = kb->CreateSub(kb->CreateShl(outputIndex, outputIndexShift), producedOffset);
    Value * produced = kb->CreateAdd(outputProduced, newlyProduced);
    for (unsigned j = 0; j < mSwizzleSetCount; j++) {
        kb->setScalarField("pendingSwizzleData" + std::to_string(j), pendingData[j]);
    }
    kb->setProducedItemCount("outputSwizzle0", produced);
}

void SwizzledBitstreamCompressByCount::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & kb, Value * /* remainingBytes */) {
    CreateDoBlockMethodCall(kb);
    Constant * blockOffsetMask = kb->getSize(kb->getBitBlockWidth() - 1);
    Constant * outputIndexShift = kb->getSize(std::log2(mFieldWidth));
    
    Value * outputProduced = kb->getProducedItemCount("outputSwizzle0"); // All output groups have the same count.
    Value * producedOffset = kb->CreateAnd(outputProduced, blockOffsetMask);
    Value * outputIndex = kb->CreateLShr(producedOffset, outputIndexShift);
    Value * pendingOffset = kb->getScalarField("pendingOffset");

    // Write the pending data.
    for (unsigned i = 0; i < mSwizzleSetCount; i++) {
        Value * pendingData = kb->getScalarField("pendingSwizzleData" + std::to_string(i));
        Value * outputStreamPtr = kb->getOutputStreamBlockPtr("outputSwizzle" + std::to_string(i), kb->getInt32(0));
        kb->CreateBlockAlignedStore(pendingData, kb->CreateGEP(outputStreamPtr, outputIndex));
    }
    kb->setProducedItemCount("outputSwizzle0", kb->CreateAdd(pendingOffset, outputProduced));
}
}
