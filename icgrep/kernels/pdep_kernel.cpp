/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "pdep_kernel.h"
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>
#include <toolchain/toolchain.h>
#include <toolchain/driver.h>
#include <toolchain/cpudriver.h>
#include <IR_Gen/idisa_target.h>
#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Module.h>


using namespace llvm;

namespace kernel {

PDEPkernel::PDEPkernel(const std::unique_ptr<kernel::KernelBuilder> & b, const unsigned swizzleFactor, std::string name)
: MultiBlockKernel(b, std::move(name),
// input stream sets
{Binding{b->getStreamSetTy(), "marker", FixedRate(), Principal()},
Binding{b->getStreamSetTy(swizzleFactor), "source", PopcountOf("marker"), BlockSize(b->getBitBlockWidth() / swizzleFactor) }},
// output stream set
{Binding{b->getStreamSetTy(swizzleFactor), "output", FixedRate(), BlockSize(b->getBitBlockWidth() / swizzleFactor)}},
{}, {}, {})
, mSwizzleFactor(swizzleFactor) {

}

void PDEPkernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfBlocks) {
    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const processBlock = b->CreateBasicBlock("processBlock");
    BasicBlock * const finishedStrides = b->CreateBasicBlock("finishedStrides");
    const auto pdepWidth = b->getBitBlockWidth() / mSwizzleFactor;
    ConstantInt * const BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());
    ConstantInt * const PDEP_WIDTH = b->getSize(pdepWidth);

    Constant * const ZERO = b->getSize(0);
    Value * const sourceItemCount = b->getProcessedItemCount("source");

    Value * const initialSourceOffset = b->CreateURem(sourceItemCount, BLOCK_WIDTH);
    b->CreateBr(processBlock);

    b->SetInsertPoint(processBlock);
    PHINode * const strideIndex = b->CreatePHI(b->getSizeTy(), 2);
    strideIndex->addIncoming(ZERO, entry);
    PHINode * const bufferPhi = b->CreatePHI(b->getBitBlockType(), 2);
    bufferPhi->addIncoming(Constant::getNullValue(b->getBitBlockType()), entry);
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
    Value * buffer = bufferPhi;
    for (unsigned i = 0; i < mSwizzleFactor; i++) {

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
        PHINode * const updatedBuffer = b->CreatePHI(buffer->getType(), 2);
        updatedBuffer->addIncoming(buffer, entry);

        // Calculate the block and swizzle index of the current swizzle row
        Value * const blockOffset = b->CreateUDiv(updatedSourceOffset, BLOCK_WIDTH);
        Value * const swizzleIndex = b->CreateUDiv(b->CreateURem(updatedSourceOffset, BLOCK_WIDTH), PDEP_WIDTH);
        Value * const swizzle = b->CreateBlockAlignedLoad(b->getInputStreamBlockPtr("source", swizzleIndex, blockOffset));
        Value * const swizzleOffset = b->CreateURem(updatedSourceOffset, PDEP_WIDTH);

        // Shift the swizzle to the right to clear off any used bits ...
        Value * const swizzleShift = b->simd_fill(pdepWidth, swizzleOffset);
        Value * const unreadBits = b->CreateLShr(swizzle, swizzleShift);

        // ... then to the left to align the bits with the buffer and combine them.
        Value * const bufferShift = b->simd_fill(pdepWidth, updatedBufferSize);
        Value * const pendingBits = b->CreateShl(unreadBits, bufferShift);

        buffer = b->CreateOr(updatedBuffer, pendingBits);
        updatedBuffer->addIncoming(buffer, enqueueBits);

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
        Value* result = b->simd_pdep(pdepWidth, buffer, b->simd_fill(pdepWidth, mask));

        // Store the result
        Value * const outputStreamPtr = b->getOutputStreamBlockPtr("output", b->getSize(i), strideIndex);
        b->CreateBlockAlignedStore(result, outputStreamPtr);

        // Shift away any used bits from the buffer and decrement our buffer size by the number we used
        Value * const usedShift = b->simd_fill(pdepWidth, required);
        buffer = b->CreateLShr(buffer, usedShift);
        bufferSize = b->CreateSub(bufferSize, required);
    }

    BasicBlock * const finishedBlock = b->GetInsertBlock();
    sourceOffsetPhi->addIncoming(sourceOffset, finishedBlock);
    bufferSizePhi->addIncoming(bufferSize, finishedBlock);
    bufferPhi->addIncoming(buffer, finishedBlock);
    Value * const nextStrideIndex = b->CreateAdd(strideIndex, b->getSize(1));
    strideIndex->addIncoming(nextStrideIndex, finishedBlock);
    b->CreateLikelyCondBr(b->CreateICmpNE(nextStrideIndex, numOfBlocks), processBlock, finishedStrides);

    b->SetInsertPoint(finishedStrides);
}

StreamExpandKernel::StreamExpandKernel(const std::unique_ptr<kernel::KernelBuilder> & b
                                       , StreamSet * source, const unsigned base, StreamSet * mask
                                       , StreamSet * expanded
                                       , const unsigned FieldWidth)
: MultiBlockKernel(b, "streamExpand" + std::to_string(FieldWidth)
+ "_" + std::to_string(source->getNumElements())
+ "_" + std::to_string(base) + "_" + std::to_string(expanded->getNumElements()),

{Binding{"marker", mask, FixedRate(), Principal()},
Binding{"source", source, PopcountOf("marker")}},
{Binding{"output", expanded, FixedRate(), BlockSize(1)}},
{}, {}, {})
, mFieldWidth(FieldWidth)
, mSelectedStreamBase(base)
, mSelectedStreamCount(expanded->getNumElements()) {

}

void StreamExpandKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfBlocks) {
    Type * fieldWidthTy = b->getIntNTy(mFieldWidth);
    Type * sizeTy = b->getSizeTy();
    const unsigned numFields = b->getBitBlockWidth() / mFieldWidth;

    Constant * const ZERO = b->getSize(0);
    Constant * bwConst = ConstantInt::get(sizeTy, b->getBitBlockWidth());
    Constant * fwConst = ConstantInt::get(sizeTy, mFieldWidth);
    Constant * fwSplat = ConstantVector::getSplat(numFields, ConstantInt::get(fieldWidthTy, mFieldWidth));
    Constant * fw_sub1Splat = ConstantVector::getSplat(numFields, ConstantInt::get(fieldWidthTy, mFieldWidth - 1));

    BasicBlock * entry = b->GetInsertBlock();
    BasicBlock * expandLoop = b->CreateBasicBlock("expandLoop");
    BasicBlock * expansionDone = b->CreateBasicBlock("expansionDone");
    Value * processedSourceItems = b->getProcessedItemCount("source");
    Value * initialSourceOffset = b->CreateURem(processedSourceItems, bwConst);

    SmallVector<Value *, 16> pendingData(mSelectedStreamCount);
    for (unsigned i = 0; i < mSelectedStreamCount; i++) {
        pendingData[i] = b->loadInputStreamBlock("source", b->getInt32(mSelectedStreamBase + i), ZERO);
    }

    b->CreateBr(expandLoop);
    // Main Loop
    b->SetInsertPoint(expandLoop);
    PHINode * blockNoPhi = b->CreatePHI(b->getSizeTy(), 2);
    PHINode * pendingOffsetPhi = b->CreatePHI(b->getSizeTy(), 2);
    SmallVector<PHINode *, 16> pendingDataPhi(mSelectedStreamCount);
    blockNoPhi->addIncoming(ZERO, entry);
    pendingOffsetPhi->addIncoming(initialSourceOffset, entry);
    for (unsigned i = 0; i < mSelectedStreamCount; i++) {
        pendingDataPhi[i] = b->CreatePHI(b->getBitBlockType(), 2);
        pendingDataPhi[i]->addIncoming(pendingData[i], entry);
    }

    Value * deposit_mask = b->loadInputStreamBlock("marker", ZERO, blockNoPhi);

    // Calculate the field values and offsets we need for assembling a
    // a full block of source bits.  Assembly will use the following operations.
    // A = b->simd_srlv(fw, b->mvmd_dsll(fw, source, pending, field_offset_lo), bit_offset);
    // B = b->simd_sllv(fw, b->mvmd_dsll(fw, source, pending, field_offset_hi), shift_fwd);
    // all_source_bits = simd_or(A, B);
    Value * pendingOffset = b->CreateURem(pendingOffsetPhi, bwConst);
    // Value * pendingItems = b->CreateURem(b->CreateSub(bwConst, pendingOffset), bwConst);
    Value * pendingItems = b->CreateSub(bwConst, pendingOffset);

    Value * field_offset_lo = b->CreateCeilUDiv(pendingItems, fwConst);
    Value * bit_offset = b->simd_fill(mFieldWidth, b->CreateURem(pendingOffset, fwConst));
    // Carefully avoid a shift by the full fieldwith (which gives a poison value).
    // field_offset_lo + 1 unless the bit_offset is 0, in which case it is just field_offset_lo.
    Value * field_offset_hi =  b->CreateUDiv(pendingItems, fwConst);
    // fw - bit_offset, unless bit_offset is 0, in which case, the shift_fwd is 0.
    Value * shift_fwd = b->CreateURem(b->CreateSub(fwSplat, bit_offset), fwSplat);

    // Once all source bits are assembled, they need to be distributed to the
    // output fields in accord with the popcounts of the deposit mask fields.
    // The bits for each output field will typically come from (at most) two
    // source fields, with offsets.  Calculate the field numbers and offsets.

    Value * fieldPopCounts = b->simd_popcount(mFieldWidth, deposit_mask);
    // For each field determine the (partial) sum popcount of all fields prior to
    // the current field.
    Value * partialSum = fieldPopCounts;
    for (unsigned i = 1; i < numFields; i *= 2) {
        partialSum = b->simd_add(mFieldWidth, partialSum, b->mvmd_slli(mFieldWidth, partialSum, i));
    }
    Value * const blockPopCount = b->CreateZExtOrTrunc(b->CreateExtractElement(partialSum, numFields - 1), sizeTy);
    partialSum = b->mvmd_slli(mFieldWidth, partialSum, 1);

    Value * const source_field_lo = b->CreateUDiv(partialSum, fwSplat);
    Value * const source_field_hi = b->CreateUDiv(b->CreateAdd(partialSum, fw_sub1Splat), fwSplat);
    Value * const source_shift_lo = b->CreateAnd(partialSum, fw_sub1Splat);  // parallel URem
    Value * const source_shift_hi = b->CreateAnd(b->CreateSub(fwSplat, source_shift_lo), fw_sub1Splat);

    // The source stream may not be positioned at a block boundary.  Partial data
    // has been saved in the kernel state, determine the next full block number
    // for loading source streams.
    Value * const newPendingOffset = b->CreateAdd(pendingOffsetPhi, blockPopCount);
    Value * const srcBlockNo = b->CreateUDiv(newPendingOffset, bwConst);

    // Now load and process source streams.
    SmallVector<Value *, 16> sourceData(mSelectedStreamCount);
    for (unsigned i = 0; i < mSelectedStreamCount; i++) {
        sourceData[i] = b->loadInputStreamBlock("source", b->getInt32(mSelectedStreamBase + i), srcBlockNo);
        Value * A = b->simd_srlv(mFieldWidth, b->mvmd_dsll(mFieldWidth, sourceData[i], pendingDataPhi[i], field_offset_lo), bit_offset);
        Value * B = b->simd_sllv(mFieldWidth, b->mvmd_dsll(mFieldWidth, sourceData[i], pendingDataPhi[i], field_offset_hi), shift_fwd);
        Value * full_source_block = b->simd_or(A, B);
        Value * C = b->simd_srlv(mFieldWidth, b->mvmd_shuffle(mFieldWidth, full_source_block, source_field_lo), source_shift_lo);
        Value * D = b->simd_sllv(mFieldWidth, b->mvmd_shuffle(mFieldWidth, full_source_block, source_field_hi), source_shift_hi);
        Value * output = b->bitCast(b->simd_or(C, D));
        b->storeOutputStreamBlock("output", b->getInt32(i), blockNoPhi, output);
    }
    //
    // Update loop control Phis for the next iteration.
    //
    Value * nextBlk = b->CreateAdd(blockNoPhi, b->getSize(1));
    blockNoPhi->addIncoming(nextBlk, expandLoop);
    pendingOffsetPhi->addIncoming(newPendingOffset, expandLoop);
    for (unsigned i = 0; i < mSelectedStreamCount; i++) {
        pendingDataPhi[i]->addIncoming(sourceData[i], expandLoop);
    }
    //
    // Now continue the loop if there are more blocks to process.
    Value * moreToDo = b->CreateICmpNE(nextBlk, numOfBlocks);
    b->CreateCondBr(moreToDo, expandLoop, expansionDone);

    b->SetInsertPoint(expansionDone);
}

FieldDepositKernel::FieldDepositKernel(const std::unique_ptr<kernel::KernelBuilder> & b
                                       , StreamSet * mask, StreamSet * input, StreamSet * output
                                       , const unsigned fieldWidth)
: MultiBlockKernel(b, "FieldDeposit" + std::to_string(fieldWidth) + "_" + std::to_string(input->getNumElements()),
{Binding{"depositMask", mask}
, Binding{"inputStreamSet", input}},
{Binding{"outputStreamSet", output}},
{}, {}, {})
, mFieldWidth(fieldWidth)
, mStreamCount(input->getNumElements()) {

}

void FieldDepositKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfBlocks) {
    BasicBlock * entry = kb->GetInsertBlock();
    BasicBlock * processBlock = kb->CreateBasicBlock("processBlock");
    BasicBlock * done = kb->CreateBasicBlock("done");
    Constant * const ZERO = kb->getSize(0);
    kb->CreateBr(processBlock);
    kb->SetInsertPoint(processBlock);
    PHINode * blockOffsetPhi = kb->CreatePHI(kb->getSizeTy(), 2);
    blockOffsetPhi->addIncoming(ZERO, entry);
    Value * depositMask = kb->loadInputStreamBlock("depositMask", ZERO, blockOffsetPhi);
    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * input = kb->loadInputStreamBlock("inputStreamSet", kb->getInt32(j), blockOffsetPhi);
        Value * output = kb->simd_pdep(mFieldWidth, input, depositMask);
        kb->storeOutputStreamBlock("outputStreamSet", kb->getInt32(j), blockOffsetPhi, output);
    }
    Value * nextBlk = kb->CreateAdd(blockOffsetPhi, kb->getSize(1));
    blockOffsetPhi->addIncoming(nextBlk, processBlock);
    Value * moreToDo = kb->CreateICmpNE(nextBlk, numOfBlocks);
    kb->CreateCondBr(moreToDo, processBlock, done);
    kb->SetInsertPoint(done);
}

PDEPFieldDepositKernel::PDEPFieldDepositKernel(const std::unique_ptr<kernel::KernelBuilder> & b
                                               , StreamSet * mask, StreamSet * input, StreamSet * output
                                               , const unsigned fieldWidth)
: MultiBlockKernel(b, "PDEPFieldDeposit" + std::to_string(fieldWidth) + "_" + std::to_string(input->getNumElements()) ,
                   {Binding{"depositMask", mask},
                    Binding{"inputStreamSet", input}},
                   {Binding{"outputStreamSet", output}},
                   {}, {}, {})
, mPDEPWidth(fieldWidth)
, mStreamCount(input->getNumElements()) {
    if ((fieldWidth != 32) && (fieldWidth != 64))
        llvm::report_fatal_error("Unsupported PDEP width for PDEPFieldDepositKernel");
}

void PDEPFieldDepositKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfBlocks) {
    Type * fieldTy = kb->getIntNTy(mPDEPWidth);
    Type * fieldPtrTy = PointerType::get(fieldTy, 0);
    Constant * PDEP_func = nullptr;
    if (mPDEPWidth == 64) {
        PDEP_func = Intrinsic::getDeclaration(kb->getModule(), Intrinsic::x86_bmi_pdep_64);
    } else if (mPDEPWidth == 32) {
        PDEP_func = Intrinsic::getDeclaration(kb->getModule(), Intrinsic::x86_bmi_pdep_32);
    }
    BasicBlock * entry = kb->GetInsertBlock();
    BasicBlock * processBlock = kb->CreateBasicBlock("processBlock");
    BasicBlock * done = kb->CreateBasicBlock("done");
    Constant * const ZERO = kb->getSize(0);
    const unsigned fieldsPerBlock = kb->getBitBlockWidth()/mPDEPWidth;
    kb->CreateBr(processBlock);
    kb->SetInsertPoint(processBlock);
    PHINode * blockOffsetPhi = kb->CreatePHI(kb->getSizeTy(), 2);
    blockOffsetPhi->addIncoming(ZERO, entry);
    std::vector<Value *> mask(fieldsPerBlock);
//  When operating on fields individually, we can use vector load/store with
//  extract/insert element operations, or we can use individual field load
//  and stores.   Individual field operations require fewer total operations,
//  but more memory instructions.   It may be that vector load/extract is better,
//  while field store is better.   Vector insert then store creates long dependence
//  chains.
//
#define PREFER_FIELD_STORES_OVER_INSERT_ELEMENT
#ifdef PREFER_FIELD_LOADS_OVER_EXTRACT_ELEMENT
    Value * depositMaskPtr = kb->getInputStreamBlockPtr("depositMask", ZERO, blockOffsetPhi);
    depositMaskPtr = kb->CreatePointerCast(depositMaskPtr, fieldPtrTy);
    for (unsigned i = 0; i < fieldsPerBlock; i++) {
        mask[i] = kb->CreateLoad(kb->CreateGEP(depositMaskPtr, kb->getInt32(i)));
    }
#else
    Value * depositMask = kb->fwCast(mPDEPWidth, kb->loadInputStreamBlock("depositMask", ZERO, blockOffsetPhi));
    for (unsigned i = 0; i < fieldsPerBlock; i++) {
        mask[i] = kb->CreateExtractElement(depositMask, kb->getInt32(i));
    }
#endif
    for (unsigned j = 0; j < mStreamCount; ++j) {
#ifdef PREFER_FIELD_LOADS_OVER_EXTRACT_ELEMENT
        Value * inputPtr = kb->getInputStreamBlockPtr("inputStreamSet", kb->getInt32(j), blockOffsetPhi);
        inputPtr = kb->CreatePointerCast(inputPtr, fieldPtrTy);
#else
        Value * inputStrm = kb->fwCast(mPDEPWidth, kb->loadInputStreamBlock("inputStreamSet", kb->getInt32(j), blockOffsetPhi));
#endif
#ifdef PREFER_FIELD_STORES_OVER_INSERT_ELEMENT
        Value * outputPtr = kb->getOutputStreamBlockPtr("outputStreamSet", kb->getInt32(j), blockOffsetPhi);
        outputPtr = kb->CreatePointerCast(outputPtr, fieldPtrTy);
#else
        Value * outputStrm = kb->fwCast(mPDEPWidth, kb->allZeroes());
#endif
        for (unsigned i = 0; i < fieldsPerBlock; i++) {
#ifdef PREFER_FIELD_LOADS_OVER_EXTRACT_ELEMENT
            Value * field = kb->CreateLoad(kb->CreateGEP(inputPtr, kb->getInt32(i)));
#else
            Value * field = kb->CreateExtractElement(inputStrm, kb->getInt32(i));
#endif
            Value * compressed = kb->CreateCall(PDEP_func, {field, mask[i]});
#ifdef PREFER_FIELD_STORES_OVER_INSERT_ELEMENT
            kb->CreateStore(compressed, kb->CreateGEP(outputPtr, kb->getInt32(i)));
        }
#else
            outputStrm = kb->CreateInsertElement(outputStrm, compressed, kb->getInt32(i));
        }
        kb->storeOutputStreamBlock("outputStreamSet", kb->getInt32(j), blockOffsetPhi, outputStrm);
#endif
    }
    Value * nextBlk = kb->CreateAdd(blockOffsetPhi, kb->getSize(1));
    blockOffsetPhi->addIncoming(nextBlk, processBlock);
    Value * moreToDo = kb->CreateICmpNE(nextBlk, numOfBlocks);
    kb->CreateCondBr(moreToDo, processBlock, done);
    kb->SetInsertPoint(done);
}

}

