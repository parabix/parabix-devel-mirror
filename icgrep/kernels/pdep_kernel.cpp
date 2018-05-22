/*
 *  Copyright (c) 2017 International Characters.
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
: MultiBlockKernel(std::move(name),
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
    
StreamExpandKernel::StreamExpandKernel(const std::unique_ptr<kernel::KernelBuilder> & kb, const unsigned fieldWidth, const unsigned streamCount)
: MultiBlockKernel("streamExpand" + std::to_string(fieldWidth) + "_" + std::to_string(streamCount),
                   {Binding{kb->getStreamSetTy(), "marker", FixedRate(), Principal()},
                       Binding{kb->getStreamSetTy(streamCount), "source", PopcountOf("marker")}},
                   {Binding{kb->getStreamSetTy(streamCount), "output", FixedRate()}},
                   {}, {}, {})
, mFieldWidth(fieldWidth)
, mStreamCount(streamCount) {
    for (unsigned i = 0; i < streamCount; i++) {
        addScalar(kb->getBitBlockType(), "pendingSourceBlock_" + std::to_string(i));
    }
}

void StreamExpandKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfBlocks) {
    const unsigned fw = mFieldWidth;
    Type * fwTy = b->getIntNTy(fw);
    Type * sizeTy = b->getSizeTy();
    const unsigned numFields = b->getBitBlockWidth()/fw;
    
    Constant * const ZERO = b->getSize(0);
    Constant * bwConst = ConstantInt::get(sizeTy, b->getBitBlockWidth());
    Constant * bw_sub1Const = ConstantInt::get(sizeTy, b->getBitBlockWidth() -1);
    Constant * fwConst = ConstantInt::get(sizeTy, fw);
    Constant * fw_sub1Const = ConstantInt::get(sizeTy, fw-1);
    Constant * fwSplat = ConstantVector::getSplat(numFields, ConstantInt::get(fwTy, fw));
    Constant * fw_sub1Splat = ConstantVector::getSplat(numFields, ConstantInt::get(fwTy, fw-1));
    
    BasicBlock * entry = b->GetInsertBlock();
    BasicBlock * expandLoop = b->CreateBasicBlock("expandLoop");
    BasicBlock * expansionDone = b->CreateBasicBlock("expansionDone");
    
    Value * processedSourceItems = b->getProcessedItemCount("source");
    Value * sourceOffset = b->CreateURem(processedSourceItems, bwConst);
    
    std::vector<Value *> pendingData(mStreamCount);
    for (unsigned i = 0; i < mStreamCount; i++) {
        pendingData[i] = b->getScalarField("pendingSourceBlock_" + std::to_string(i));
    }
    
    b->CreateBr(expandLoop);
    // Main Loop
    b->SetInsertPoint(expandLoop);
    PHINode * blockNoPhi = b->CreatePHI(b->getSizeTy(), 2);
    PHINode * pendingItemsPhi = b->CreatePHI(b->getSizeTy(), 2);
    PHINode * pendingDataPhi[mStreamCount];
    blockNoPhi->addIncoming(ZERO, entry);
    pendingItemsPhi->addIncoming(sourceOffset, entry);
    for (unsigned i = 0; i < mStreamCount; i++) {
        pendingDataPhi[i] = b->CreatePHI(b->getBitBlockType(), 2);
        pendingDataPhi[i]->addIncoming(pendingData[i], entry);
    }
    Value * deposit_mask = b->loadInputStreamBlock("marker", ZERO, blockNoPhi);
    // The source stream may not be positioned at a block boundary.  Partial data
    // has been saved in the kernel state, determine the next full block number
    // for loading source streams.
    Value * pendingBlockEnd = b->CreateAdd(pendingItemsPhi, bw_sub1Const);
    Value * srcBlockNo = b->CreateUDiv(pendingBlockEnd, bwConst);
    
    // Calculate the field values and offsets we need for assembling a
    // a full block of source bits.  Assembly will use the following operations.
    // A = b->simd_srl(fw, b->mvmd_dsll(fw, source, pending, field_offset_lo), bit_offset);
    // B = b->simd_sll(fw, b->mvmd_dsll(fw, source, pending, field_offset_hi), shift_fwd);
    // all_source_bits = simd_or(A, B);
    Value * pendingOffset = b->CreateURem(pendingBlockEnd, bwConst);
    Value * field_offset_lo =  b->CreateUDiv(pendingOffset, fwConst);
    Value * bit_offset = b->simd_fill(fw, b->CreateURem(pendingOffset, fwConst));
    
    // Carefully avoid a shift by the full fieldwith (which gives a poison value).
    // field_offset_lo + 1 unless the bit_offset is 0, in which case it is just field_offset_lo.
    Value * field_offset_hi =  b->CreateUDiv(b->CreateAdd(pendingOffset, fw_sub1Const), fwConst);
    // fw - bit_offset, unless bit_offset is 0, in which case, the shift_fwd is 0.
    Value * shift_fwd = b->CreateURem(b->CreateSub(fwSplat, bit_offset), fwSplat);
    
    // Once all source bits are assembled, they need to be distributed to the
    // output fields in accord with the popcounts of the deposit mask fields.
    // The bits for each output field will typically come from (at most) two
    // source fields, with offsets.  Calculate the field numbers and offsets.
    
    Value * fieldPopCounts = b->simd_popcount(fw, deposit_mask);
    // For each field determine the (partial) sum popcount of all fields prior to
    // the current field.
    Value * partialSum = fieldPopCounts;
    for (unsigned i = 1; i < numFields; i *= 2) {
        partialSum = b->simd_add(fw, partialSum, b->mvmd_slli(fw, partialSum, i));
    }
    Value * blockPopCount = b->CreateZExtOrTrunc(b->CreateExtractElement(partialSum, numFields-1), sizeTy);
    partialSum = b->mvmd_slli(fw, partialSum, 1);
    
    Value * source_field_lo = b->CreateUDiv(partialSum, fwSplat);
    Value * source_field_hi = b->CreateUDiv(b->CreateAdd(partialSum, fw_sub1Splat), fwSplat);
    Value * source_shift_lo = b->CreateAnd(partialSum, fw_sub1Splat);  // parallel URem
    Value * source_shift_hi = b->CreateAnd(b->CreateSub(fwSplat, source_shift_lo), fw_sub1Splat);
    
    // Now load and process source streams.
    for (unsigned i = 0; i < mStreamCount; i++) {
        Value * source = b->loadInputStreamBlock("source", b->getInt32(i), srcBlockNo);
        Value * A = b->simd_srlv(fw, b->mvmd_dsll(fw, source, pendingDataPhi[i], field_offset_lo), bit_offset);
        Value * B = b->simd_sllv(fw, b->mvmd_dsll(fw, source, pendingDataPhi[i], field_offset_hi), shift_fwd);
        Value * full_source_block = b->simd_or(A, B);
        
        Value * C = b->simd_srlv(fw, b->mvmd_shuffle(fw, full_source_block, source_field_lo), source_shift_lo);
        Value * D = b->simd_sllv(fw, b->mvmd_shuffle(fw, full_source_block, source_field_hi), source_shift_hi);
        Value * output = b->bitCast(b->simd_or(C, D));
        b->storeOutputStreamBlock("output", b->getInt32(i), blockNoPhi, output);
        pendingDataPhi[i]->addIncoming(source, expandLoop);
    }
    //
    // Update loop control Phis for the next iteration.
    //
    Value * nextBlk = b->CreateAdd(blockNoPhi, b->getSize(1));
    blockNoPhi->addIncoming(nextBlk, expandLoop);
    Value * newPending = b->CreateAdd(pendingItemsPhi, blockPopCount);
    pendingItemsPhi->addIncoming(newPending, expandLoop);
    //
    // Now continue the loop if there are more blocks to process.
    Value * moreToDo = b->CreateICmpNE(nextBlk, numOfBlocks);
    b->CreateCondBr(moreToDo, expandLoop, expansionDone);
    
    b->SetInsertPoint(expansionDone);
    // Update kernel state.
    for (unsigned i = 0; i < mStreamCount; i++) {
        b->setScalarField("pendingSourceBlock_" + std::to_string(i), b->bitCast(pendingDataPhi[i]));
    }
}

FieldDepositKernel::FieldDepositKernel(const std::unique_ptr<kernel::KernelBuilder> & kb, const unsigned fieldWidth, const unsigned streamCount)
: MultiBlockKernel("FieldDeposit" + std::to_string(fieldWidth) + "_" + std::to_string(streamCount),
                   {Binding{kb->getStreamSetTy(), "depositMask"}, Binding{kb->getStreamSetTy(streamCount), "inputStreamSet"}},
                   {Binding{kb->getStreamSetTy(streamCount), "outputStreamSet"}},
                   {}, {}, {})
, mFieldWidth(fieldWidth)
, mStreamCount(streamCount) {
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

PDEPFieldDepositKernel::PDEPFieldDepositKernel(const std::unique_ptr<kernel::KernelBuilder> & kb, const unsigned fieldWidth, const unsigned streamCount)
: MultiBlockKernel("PDEPFieldDeposit" + std::to_string(fieldWidth) + "_" + std::to_string(streamCount),
                   {Binding{kb->getStreamSetTy(), "depositMask"}, Binding{kb->getStreamSetTy(streamCount), "inputStreamSet"}},
                   {Binding{kb->getStreamSetTy(streamCount), "outputStreamSet"}},
                   {}, {}, {})
, mPDEPWidth(fieldWidth)
, mStreamCount(streamCount) {
    if ((fieldWidth != 32) && (fieldWidth != 64)) llvm::report_fatal_error("Unsupported PDEP width for PDEPFieldCompressKernel");
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
    Value * extractionMaskPtr = kb->getInputStreamBlockPtr("depositMask", ZERO, blockOffsetPhi);
    extractionMaskPtr = kb->CreatePointerCast(extractionMaskPtr, fieldPtrTy);
    for (unsigned i = 0; i < fieldsPerBlock; i++) {
        mask[i] = kb->CreateLoad(kb->CreateGEP(extractionMaskPtr, kb->getInt32(i)));
    }
    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * inputPtr = kb->getInputStreamBlockPtr("inputStreamSet", kb->getInt32(j), blockOffsetPhi);
        inputPtr = kb->CreatePointerCast(inputPtr, fieldPtrTy);
        Value * outputPtr = kb->getOutputStreamBlockPtr("outputStreamSet", kb->getInt32(j), blockOffsetPhi);
        outputPtr = kb->CreatePointerCast(outputPtr, fieldPtrTy);
        for (unsigned i = 0; i < fieldsPerBlock; i++) {
            Value * field = kb->CreateLoad(kb->CreateGEP(inputPtr, kb->getInt32(i)));
            Value * compressed = kb->CreateCall(PDEP_func, {field, mask[i]});
            kb->CreateStore(compressed, kb->CreateGEP(outputPtr, kb->getInt32(i)));
        }
    }
    Value * nextBlk = kb->CreateAdd(blockOffsetPhi, kb->getSize(1));
    blockOffsetPhi->addIncoming(nextBlk, processBlock);
    Value * moreToDo = kb->CreateICmpNE(nextBlk, numOfBlocks);
    kb->CreateCondBr(moreToDo, processBlock, done);
    kb->SetInsertPoint(done);
}

void StreamDepositCompiler::makeCall(parabix::StreamSetBuffer * depositMask, parabix::StreamSetBuffer * inputs, parabix::StreamSetBuffer * outputs) {
    if (mBufferBlocks == 0) {
        llvm::report_fatal_error("StreamDepositCompiler needs a non-zero bufferBlocks parameter (for now).");
    }
    auto & iBuilder = mDriver.getBuilder();
    unsigned N = IDISA::getNumOfStreams(ssType);
    if (IDISA::getStreamFieldWidth(ssType) != 1) {
        llvm::report_fatal_error("StreamDepositCompiler only compresses bit streams (for now)");
    }
    parabix::StreamSetBuffer * expandedStreams = mDriver.addBuffer<parabix::StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(N), mBufferBlocks);
    Kernel * streamK = mDriver.addKernelInstance<StreamExpandKernel>(iBuilder, mFieldWidth, N);
    mDriver.makeKernelCall(streamK, {depositMask, inputs}, {expandedStreams});

    Kernel * depositK = nullptr;
    if (AVX2_available()) {
        depositK = mDriver.addKernelInstance<PDEPFieldDepositKernel>(iBuilder, mFieldWidth, N);
    } else {
        depositK = mDriver.addKernelInstance<FieldDepositKernel>(iBuilder, mFieldWidth, N);
    }
    mDriver.makeKernelCall(depositK, {depositMask, expandedStreams}, {outputs});
}

}
