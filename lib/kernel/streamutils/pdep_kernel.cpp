/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include <kernel/streamutils/pdep_kernel.h>
#include <kernel/streamutils/deletion.h>

#include <llvm/IR/Intrinsics.h>
#include <llvm/IR/Module.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/core/streamset.h>
#include <kernel/core/idisa_target.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <llvm/Support/raw_ostream.h>
#include <kernel/pipeline/driver/driver.h>
#include <kernel/pipeline/driver/cpudriver.h>
#include <pablo/pablo_kernel.h>
#include <toolchain/pablo_toolchain.h>
#include <pablo/bixnum/bixnum.h>

using namespace llvm;

namespace kernel {

using BuilderRef = Kernel::BuilderRef;

void SpreadByMask(const std::unique_ptr<ProgramBuilder> & P,
                  StreamSet * mask, StreamSet * toSpread, StreamSet * outputs,
                  unsigned streamOffset,
                  StreamExpandOptimization opt,
                  unsigned expansionFieldWidth) {
    unsigned streamCount = outputs->getNumElements();
    StreamSet * const expanded = P->CreateStreamSet(streamCount);
    Scalar * base = P->CreateConstant(P->getDriver().getBuilder()->getSize(streamOffset));
    P->CreateKernelCall<StreamExpandKernel>(mask, toSpread, expanded, base, opt, expansionFieldWidth);
    P->CreateKernelCall<FieldDepositKernel>(mask, expanded, outputs, expansionFieldWidth);
}

StreamExpandKernel::StreamExpandKernel(BuilderRef b,
                                       StreamSet * mask,
                                       StreamSet * source,
                                       StreamSet * expanded,
                                       Scalar * base,
                                       const StreamExpandOptimization opt,
                                       const unsigned FieldWidth)
    : MultiBlockKernel(b, "streamExpand" + std::to_string(FieldWidth) + ((opt == StreamExpandOptimization::NullCheck) ? "nullcheck" : "")
+ "_" + std::to_string(source->getNumElements())
+ ":" + std::to_string(expanded->getNumElements()),
// input stream sets
{Binding{"marker", mask, FixedRate()},
Binding{"source", source, PopcountOf("marker"), BlockSize(b->getBitBlockWidth())}},
// output stream set
{Binding{"output", expanded}},
// input scalar
{Binding{"base", base}},
{}, {})
, mFieldWidth(FieldWidth)
, mSelectedStreamCount(expanded->getNumElements()),
    mOptimization(opt) {}

void StreamExpandKernel::generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfBlocks) {
    Type * fieldWidthTy = b->getIntNTy(mFieldWidth);
    Type * sizeTy = b->getSizeTy();
    const unsigned numFields = b->getBitBlockWidth() / mFieldWidth;

    Constant * const ZERO = b->getSize(0);
    Constant * BLOCK_WIDTH = ConstantInt::get(sizeTy, b->getBitBlockWidth());
    Constant * FIELD_WIDTH = ConstantInt::get(sizeTy, mFieldWidth);
    Constant * fwSplat = ConstantVector::getSplat(numFields, ConstantInt::get(fieldWidthTy, mFieldWidth));
    Constant * fw_sub1Splat = ConstantVector::getSplat(numFields, ConstantInt::get(fieldWidthTy, mFieldWidth - 1));

    BasicBlock * entry = b->GetInsertBlock();
    BasicBlock * expandLoop = b->CreateBasicBlock("expandLoop");
    BasicBlock * expansionDone = b->CreateBasicBlock("expansionDone");
    Value * processedSourceItems = b->getProcessedItemCount("source");
    Value * initialSourceOffset = b->CreateURem(processedSourceItems, BLOCK_WIDTH);

    Value * const streamBase = b->getScalarField("base");

    SmallVector<Value *, 16> pendingData(mSelectedStreamCount);
    for (unsigned i = 0; i < mSelectedStreamCount; i++) {
        Constant * const streamIndex = ConstantInt::get(streamBase->getType(), i);
        Value * const streamOffset = b->CreateAdd(streamBase, streamIndex);
        pendingData[i] = b->loadInputStreamBlock("source", streamOffset, ZERO);
    }

    b->CreateBr(expandLoop);
    // Main Loop
    b->SetInsertPoint(expandLoop);
    const unsigned incomingCount = (mOptimization == StreamExpandOptimization::NullCheck) ? 3 : 2;
    PHINode * blockNoPhi = b->CreatePHI(b->getSizeTy(), incomingCount);
    PHINode * pendingOffsetPhi = b->CreatePHI(b->getSizeTy(), incomingCount);
    SmallVector<PHINode *, 16> pendingDataPhi(mSelectedStreamCount);
    blockNoPhi->addIncoming(ZERO, entry);
    pendingOffsetPhi->addIncoming(initialSourceOffset, entry);

    for (unsigned i = 0; i < mSelectedStreamCount; i++) {
        pendingDataPhi[i] = b->CreatePHI(b->getBitBlockType(), incomingCount);
        pendingDataPhi[i]->addIncoming(pendingData[i], entry);
    }

    Value * deposit_mask = b->loadInputStreamBlock("marker", ZERO, blockNoPhi);
    Value * nextBlk = b->CreateAdd(blockNoPhi, b->getSize(1));
    Value * moreToDo = b->CreateICmpNE(nextBlk, numOfBlocks);
    if (mOptimization == StreamExpandOptimization::NullCheck) {
        BasicBlock * expandLoopContinue = b->CreateBasicBlock("expandLoopContinue");
        BasicBlock * nullMarkers = b->CreateBasicBlock("nullMarkers");
        b->CreateCondBr(b->bitblock_any(deposit_mask), expandLoopContinue, nullMarkers);

        b->SetInsertPoint(nullMarkers);
        Constant * zeroes = b->allZeroes();
        for (unsigned i = 0; i < mSelectedStreamCount; i++) {
            b->storeOutputStreamBlock("output", b->getInt32(i), blockNoPhi, zeroes);
        }
        blockNoPhi->addIncoming(nextBlk, nullMarkers);
        pendingOffsetPhi->addIncoming(pendingOffsetPhi, nullMarkers);
        for (unsigned i = 0; i < mSelectedStreamCount; i++) {
            pendingDataPhi[i]->addIncoming(pendingDataPhi[i], nullMarkers);
        }
        b->CreateCondBr(moreToDo, expandLoop, expansionDone);

        b->SetInsertPoint(expandLoopContinue);
    }
    // Calculate the field values and offsets we need for assembling a
    // a full block of source bits.  Assembly will use the following operations.
    // A = b->simd_srlv(fw, b->mvmd_dsll(fw, source, pending, field_offset_lo), bit_offset);
    // B = b->simd_sllv(fw, b->mvmd_dsll(fw, source, pending, field_offset_hi), shift_fwd);
    // all_source_bits = simd_or(A, B);
    Value * pendingOffset = b->CreateURem(pendingOffsetPhi, BLOCK_WIDTH);
    // Value * pendingItems = b->CreateURem(b->CreateSub(bwConst, pendingOffset), bwConst);
    Value * pendingItems = b->CreateSub(BLOCK_WIDTH, pendingOffset);

    Value * field_offset_lo = b->CreateCeilUDiv(pendingItems, FIELD_WIDTH);
    Value * bit_offset = b->simd_fill(mFieldWidth, b->CreateURem(pendingOffset, FIELD_WIDTH));
    // Carefully avoid a shift by the full fieldwith (which gives a poison value).
    // field_offset_lo + 1 unless the bit_offset is 0, in which case it is just field_offset_lo.
    Value * field_offset_hi =  b->CreateUDiv(pendingItems, FIELD_WIDTH);
    // fw - bit_offset, unless bit_offset is 0, in which case, the shift_fwd is 0.
    Value * shift_fwd = b->CreateURem(b->CreateSub(fwSplat, bit_offset), fwSplat);

    // Once all source bits are assembled, they need to be distributed to the
    // output fields in accord with the popcounts of the deposit mask fields.
    // The bits for each output field will typically come from (at most) two
    // source fields, with offsets.  Calculate the field numbers and offsets.

    Value * fieldPopCounts = b->simd_popcount(mFieldWidth, deposit_mask);
    // For each field determine the (partial) sum popcount of all fields prior to
    // the current field.

    Value * partialSum = b->hsimd_partial_sum(mFieldWidth, fieldPopCounts);
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
    Value * const srcBlockNo = b->CreateUDiv(newPendingOffset, BLOCK_WIDTH);

    // Now load and process source streams.
    SmallVector<Value *, 16> sourceData(mSelectedStreamCount);
    for (unsigned i = 0; i < mSelectedStreamCount; i++) {
        Constant * const streamIndex = ConstantInt::get(streamBase->getType(), i);
        Value * const streamOffset = b->CreateAdd(streamBase, streamIndex);
        sourceData[i] = b->loadInputStreamBlock("source", streamOffset, srcBlockNo);
        Value * A = b->simd_srlv(mFieldWidth, b->mvmd_dsll(mFieldWidth, sourceData[i], pendingDataPhi[i], field_offset_lo), bit_offset);
        Value * B = b->simd_sllv(mFieldWidth, b->mvmd_dsll(mFieldWidth, sourceData[i], pendingDataPhi[i], field_offset_hi), shift_fwd);
        Value * full_source_block = b->CreateOr(A, B, "toExpand");
        Value * C = b->simd_srlv(mFieldWidth, b->mvmd_shuffle(mFieldWidth, full_source_block, source_field_lo), source_shift_lo);
        Value * D = b->simd_sllv(mFieldWidth, b->mvmd_shuffle(mFieldWidth, full_source_block, source_field_hi), source_shift_hi);
        Value * output = b->bitCast(b->CreateOr(C, D, "expanded"));
        b->storeOutputStreamBlock("output", b->getInt32(i), blockNoPhi, output);
    }
    //
    // Update loop control Phis for the next iteration.
    //
    blockNoPhi->addIncoming(nextBlk, b->GetInsertBlock());
    pendingOffsetPhi->addIncoming(newPendingOffset, b->GetInsertBlock());
    for (unsigned i = 0; i < mSelectedStreamCount; i++) {
        pendingDataPhi[i]->addIncoming(sourceData[i], b->GetInsertBlock());
    }
    //
    // Now continue the loop if there are more blocks to process.
    b->CreateCondBr(moreToDo, expandLoop, expansionDone);

    b->SetInsertPoint(expansionDone);
}

FieldDepositKernel::FieldDepositKernel(BuilderRef b
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

void PDEPFieldDepositLogic(BuilderRef kb, llvm::Value * const numOfBlocks, unsigned fieldWidth, unsigned streamCount);

void FieldDepositKernel::generateMultiBlockLogic(BuilderRef kb, llvm::Value * const numOfBlocks) {
    if (AVX2_available() && BMI2_available() && ((mFieldWidth == 32) || (mFieldWidth == 64))) {
        PDEPFieldDepositLogic(kb, numOfBlocks, mFieldWidth, mStreamCount);
    } else {
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
}

void PDEPFieldDepositLogic(BuilderRef kb, llvm::Value * const numOfBlocks, unsigned fieldWidth, unsigned streamCount) {
        Type * fieldTy = kb->getIntNTy(fieldWidth);
        Type * fieldPtrTy = PointerType::get(fieldTy, 0);
        Constant * PDEP_func = nullptr;
        if (fieldWidth == 64) {
            PDEP_func = Intrinsic::getDeclaration(kb->getModule(), Intrinsic::x86_bmi_pdep_64);
        } else if (fieldWidth == 32) {
            PDEP_func = Intrinsic::getDeclaration(kb->getModule(), Intrinsic::x86_bmi_pdep_32);
        }
        BasicBlock * entry = kb->GetInsertBlock();
        BasicBlock * processBlock = kb->CreateBasicBlock("processBlock");
        BasicBlock * done = kb->CreateBasicBlock("done");
        Constant * const ZERO = kb->getSize(0);
        const unsigned fieldsPerBlock = kb->getBitBlockWidth()/fieldWidth;
        kb->CreateBr(processBlock);
        kb->SetInsertPoint(processBlock);
        PHINode * blockOffsetPhi = kb->CreatePHI(kb->getSizeTy(), 2);
        blockOffsetPhi->addIncoming(ZERO, entry);

        SmallVector<Value *, 16> mask(fieldsPerBlock);
        //  When operating on fields individually, we can use vector load/store with
        //  extract/insert element operations, or we can use individual field load
        //  and stores.   Individual field operations require fewer total operations,
        //  but more memory instructions.   It may be that vector load/extract is better,
        //  while field store is better.   Vector insert then store creates long dependence
        //  chains.
        //
//#define PREFER_FIELD_STORES_OVER_INSERT_ELEMENT
#ifdef PREFER_FIELD_LOADS_OVER_EXTRACT_ELEMENT
        Value * depositMaskPtr = kb->getInputStreamBlockPtr("depositMask", ZERO, blockOffsetPhi);
        depositMaskPtr = kb->CreatePointerCast(depositMaskPtr, fieldPtrTy);
        for (unsigned i = 0; i < fieldsPerBlock; i++) {
            mask[i] = kb->CreateLoad(kb->CreateGEP(depositMaskPtr, kb->getInt32(i)));
        }
#else

        Value * depositMask = kb->fwCast(fieldWidth, kb->loadInputStreamBlock("depositMask", ZERO, blockOffsetPhi));
        for (unsigned i = 0; i < fieldsPerBlock; i++) {
            mask[i] = kb->CreateExtractElement(depositMask, kb->getInt32(i));
        }
#endif
        for (unsigned j = 0; j < streamCount; ++j) {
#ifdef PREFER_FIELD_LOADS_OVER_EXTRACT_ELEMENT
            Value * inputPtr = kb->getInputStreamBlockPtr("inputStreamSet", kb->getInt32(j), blockOffsetPhi);
            inputPtr = kb->CreatePointerCast(inputPtr, fieldPtrTy);
#else
            Value * const input = kb->loadInputStreamBlock("inputStreamSet", kb->getInt32(j), blockOffsetPhi);
            Value * inputStrm = kb->fwCast(fieldWidth, input);
#endif
#ifdef PREFER_FIELD_STORES_OVER_INSERT_ELEMENT
            Value * outputPtr = kb->getOutputStreamBlockPtr("outputStreamSet", kb->getInt32(j), blockOffsetPhi);
            outputPtr = kb->CreatePointerCast(outputPtr, fieldPtrTy);
#else
           // Value * outputStrm = kb->fwCast(fieldWidth, kb->allZeroes());
            Value * outputStrm = UndefValue::get(kb->fwVectorType(fieldWidth));
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

PDEPFieldDepositKernel::PDEPFieldDepositKernel(BuilderRef b
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

void PDEPFieldDepositKernel::generateMultiBlockLogic(BuilderRef kb, llvm::Value * const numOfBlocks) {
    PDEPFieldDepositLogic(kb, numOfBlocks, mPDEPWidth, mStreamCount);
}


PDEPkernel::PDEPkernel(BuilderRef b, const unsigned swizzleFactor, std::string name)
: MultiBlockKernel(b, std::move(name),
                   // input stream sets
{Binding{b->getStreamSetTy(), "marker", FixedRate(), Principal()},
    Binding{b->getStreamSetTy(swizzleFactor), "source", PopcountOf("marker"), BlockSize(b->getBitBlockWidth() / swizzleFactor) }},
                   // output stream set
{Binding{b->getStreamSetTy(swizzleFactor), "output", FixedRate(), BlockSize(b->getBitBlockWidth() / swizzleFactor)}},
{}, {}, {})
, mSwizzleFactor(swizzleFactor) {
}

void PDEPkernel::generateMultiBlockLogic(BuilderRef b, Value * const numOfBlocks) {
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

std::string InsertString(StreamSet * mask, InsertPosition p) {
    std::string s = std::to_string(mask->getNumElements()) + "x1_";
    return s + (p == InsertPosition::Before ? "Before" : "After");
}

class UnitInsertionExtractionMasks final : public BlockOrientedKernel {
public:
    UnitInsertionExtractionMasks(BuilderRef b,
                                 StreamSet * insertion_mask, StreamSet * stream01, StreamSet * valid01, InsertPosition p = InsertPosition::Before)
    : BlockOrientedKernel(b, "unitInsertionExtractionMasks" + InsertString(insertion_mask, p),
        {Binding{"insertion_mask", insertion_mask}},
        {Binding{"stream01", stream01, FixedRate(2)}, Binding{"valid01", valid01, FixedRate(2)}},
        {}, {},
        {InternalScalar{ScalarType::NonPersistent, b->getBitBlockType(), "EOFmask"}}),
    mInsertPos(p) {}
protected:
    void generateDoBlockMethod(BuilderRef b) override;
    void generateFinalBlockMethod(BuilderRef b, llvm::Value * const remainingBytes) override;
private:
    const InsertPosition mInsertPos;
};

void UnitInsertionExtractionMasks::generateDoBlockMethod(BuilderRef b) {
    Value * fileExtentMask = b->CreateNot(b->getScalarField("EOFmask"));
    Value * insertion_mask = b->loadInputStreamBlock("insertion_mask", b->getSize(0), b->getSize(0));
    const auto n = b->getInputStreamSet("insertion_mask")->getNumElements();
    for (unsigned i = 1; i < n; i++) {
        insertion_mask = b->CreateOr(insertion_mask, b->loadInputStreamBlock("insertion_mask", b->getSize(i), b->getSize(0)));
    }
    Constant * mask01 = nullptr;
    Value * extract_mask_lo = nullptr;
    Value * extract_mask_hi = nullptr;
    if (mInsertPos == InsertPosition::Before) {
        mask01 = b->simd_himask(2);
        extract_mask_lo = b->esimd_mergel(1, insertion_mask, fileExtentMask);
        extract_mask_hi = b->esimd_mergeh(1, insertion_mask, fileExtentMask);
    } else {
        mask01 = b->simd_lomask(2);
        extract_mask_lo = b->esimd_mergel(1, fileExtentMask, insertion_mask);
        extract_mask_hi = b->esimd_mergeh(1, fileExtentMask, insertion_mask);
    }
    b->storeOutputStreamBlock("stream01", b->getSize(0), b->getSize(0), mask01);
    b->storeOutputStreamBlock("stream01", b->getSize(0), b->getSize(1), mask01);
    b->storeOutputStreamBlock("valid01", b->getSize(0), b->getSize(0), extract_mask_lo);
    b->storeOutputStreamBlock("valid01", b->getSize(0), b->getSize(1), extract_mask_hi);
}

void UnitInsertionExtractionMasks::generateFinalBlockMethod(BuilderRef b, Value * const remainingBytes) {
    // Standard Pablo convention for final block processing: set a bit marking
    // the position just past EOF, as well as a mask marking all positions past EOF.
    b->setScalarField("EOFmask", b->bitblock_mask_from(remainingBytes));
    RepeatDoBlockLogic(b);
}

StreamSet * UnitInsertionSpreadMask(const std::unique_ptr<ProgramBuilder> & P, StreamSet * insertion_mask, InsertPosition p) {
    auto stream01 = P->CreateStreamSet(1);
    auto valid01 = P->CreateStreamSet(1);
    P->CreateKernelCall<UnitInsertionExtractionMasks>(insertion_mask, stream01, valid01, p);
    auto spread_mask = P->CreateStreamSet(1);
    FilterByMask(P, valid01, stream01, spread_mask);
    return spread_mask;
}

class UGT_Kernel final : public pablo::PabloKernel {
public:
    UGT_Kernel(BuilderRef b, StreamSet * bixnum, unsigned immediate, StreamSet * result) :
    pablo::PabloKernel(b, "ugt_" + std::to_string(immediate) + "_" + std::to_string(bixnum->getNumElements()),
                {Binding{"bixnum", bixnum}}, {Binding{"result", result}}), mTestVal(immediate) {}
protected:
    void generatePabloMethod() override;
private:
    const unsigned mTestVal;
};

void UGT_Kernel::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    pablo::BixNumCompiler bnc(pb);
    pablo::BixNum bixnum = getInputStreamSet("bixnum");
    pablo::Var * output = getOutputStreamVar("result");
    pb.createAssign(pb.createExtract(output, 0), bnc.UGT(bixnum, mTestVal));
}

class SpreadMaskStep final : public pablo::PabloKernel {
public:
    SpreadMaskStep(BuilderRef b,
                   StreamSet * bixnum, StreamSet * result, InsertPosition p = InsertPosition::Before) :
    pablo::PabloKernel(b, "spreadMaskStep_" + InsertString(bixnum, p),
                {Binding{"bixnum", bixnum, FixedRate(1), LookAhead(1)}},
                {Binding{"result", result}}), mInsertPos(p) {}
protected:
    void generatePabloMethod() override;
private:
    const InsertPosition mInsertPos;
};

void SpreadMaskStep::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    pablo::BixNumCompiler bnc(pb);
    pablo::BixNum insert_counts = getInputStreamSet("bixnum");
    pablo::BixNum has_insert(1);
    has_insert[0] = bnc.UGT(insert_counts, 0);
    // Since we have inserted one position, subtract one from the insert count.
    pablo::BixNum remaining_to_insert = bnc.SubModular(insert_counts, has_insert);
    // Split the original insert counts into 2
    unsigned remaining_bits = insert_counts.size() - 1;
    pablo::BixNum divided_counts = bnc.HighBits(insert_counts, remaining_bits);
    pablo::BixNum divided_remaining = bnc.HighBits(remaining_to_insert, remaining_bits);
    for (unsigned i = 0; i < remaining_bits; i++) {
        std::string vname = "divided_counts[" + std::to_string(i) + "]";
        if (mInsertPos == InsertPosition::Before) {
            divided_counts[i] = pb.createOr(divided_remaining[i], pb.createLookahead(divided_counts[i], 1), vname);
        } else {
            divided_counts[i] = pb.createOr(divided_counts[i], pb.createAdvance(divided_remaining[i], 1), vname);
        }
    }
    pablo::Var * result = getOutputStreamVar("result");
    for (unsigned i = 0; i < remaining_bits; i++) {
        pb.createAssign(pb.createExtract(result, i), divided_counts[i]);
    }
}

StreamSet * InsertionSpreadMask(const std::unique_ptr<ProgramBuilder> & P,
                                StreamSet * bixNumInsertCount, InsertPosition pos) {
    unsigned steps = bixNumInsertCount->getNumElements();
    if (steps == 1) {
        return UnitInsertionSpreadMask(P, bixNumInsertCount, pos);
    }
    /* Create a spread mask that adds one spread position for any position
       at which there is at least one item to insert.  */
    StreamSet * spread1_mask = P->CreateStreamSet(1);
    spread1_mask = UnitInsertionSpreadMask(P, bixNumInsertCount, pos);
    /* Spread out the counts so that there are two positions for each nonzero entry. */
    StreamSet * spread_counts = P->CreateStreamSet(steps);
    SpreadByMask(P, spread1_mask, bixNumInsertCount, spread_counts);
    /* Divide the count at each original position equally into the
       two positions that were created by the unit spread process. */
    StreamSet * reduced_counts = P->CreateStreamSet(steps - 1);
    P->CreateKernelCall<SpreadMaskStep>(spread_counts, reduced_counts, pos);
    StreamSet * submask = InsertionSpreadMask(P, reduced_counts, pos);
    StreamSet * finalmask = P->CreateStreamSet(1);
    SpreadByMask(P, submask, spread1_mask, finalmask);
    return finalmask;
}
}
