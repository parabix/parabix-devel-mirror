/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/streamutils/deletion.h>

#include <llvm/IR/Intrinsics.h>
#include <llvm/Support/raw_ostream.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/core/idisa_target.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/pipeline/driver/driver.h>
#include <kernel/pipeline/driver/cpudriver.h>

using namespace llvm;

inline size_t ceil_udiv(const size_t n, const size_t m) {
    return (n + m - 1) / m;
}

namespace kernel {

using BuilderRef = Kernel::BuilderRef;

void FilterByMask(const std::unique_ptr<ProgramBuilder> & P,
                  StreamSet * mask, StreamSet * inputs, StreamSet * outputs,
                  unsigned streamOffset,
                  unsigned extractionFieldWidth) {
    StreamSet * const compressed = P->CreateStreamSet(outputs->getNumElements());
    std::vector<uint32_t> output_indices = streamutils::Range(streamOffset, streamOffset + outputs->getNumElements());
    P->CreateKernelCall<FieldCompressKernel>(Select(mask, {0}), SelectOperationList { Select(inputs, output_indices)}, compressed, extractionFieldWidth);
    P->CreateKernelCall<StreamCompressKernel>(mask, compressed, outputs, extractionFieldWidth);
}

inline std::vector<Value *> parallel_prefix_deletion_masks(BuilderRef kb, const unsigned fw, Value * del_mask) {
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

inline Value * apply_parallel_prefix_deletion(BuilderRef kb, const unsigned fw, Value * del_mask, const std::vector<Value *> & mv, Value * strm) {
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

void DeletionKernel::generateDoBlockMethod(BuilderRef kb) {
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

void DeletionKernel::generateFinalBlockMethod(BuilderRef kb, Value * remainingBytes) {
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

DeletionKernel::DeletionKernel(BuilderRef b, const unsigned fieldWidth, const unsigned streamCount)
: BlockOrientedKernel(b, "del" + std::to_string(fieldWidth) + "_" + std::to_string(streamCount),
{Binding{b->getStreamSetTy(streamCount), "inputStreamSet"},
  Binding{b->getStreamSetTy(), "delMaskSet"}},
{Binding{b->getStreamSetTy(streamCount), "outputStreamSet"},
  Binding{b->getStreamSetTy(), "unitCounts", FixedRate(), RoundUpTo(b->getBitBlockWidth())}},
{}, {}, {})
, mDeletionFieldWidth(fieldWidth)
, mStreamCount(streamCount) {
}

void FieldCompressKernel::generateMultiBlockLogic(BuilderRef kb, llvm::Value * const numOfBlocks) {
    BasicBlock * entry = kb->GetInsertBlock();
    BasicBlock * processBlock = kb->CreateBasicBlock("processBlock");
    BasicBlock * done = kb->CreateBasicBlock("done");
    Constant * const ZERO = kb->getSize(0);
    kb->CreateBr(processBlock);
    kb->SetInsertPoint(processBlock);
    PHINode * blockOffsetPhi = kb->CreatePHI(kb->getSizeTy(), 2);
    blockOffsetPhi->addIncoming(ZERO, entry);
    std::vector<Value *> maskVec = streamutils::loadInputSelectionsBlock(kb, {mMaskOp}, blockOffsetPhi);
    std::vector<Value *> input = streamutils::loadInputSelectionsBlock(kb, mInputOps, blockOffsetPhi);
    if (BMI2_available() && ((mCompressFieldWidth == 32) || (mCompressFieldWidth == 64))) {
        Type * fieldTy = kb->getIntNTy(mCompressFieldWidth);
        Type * fieldPtrTy = PointerType::get(fieldTy, 0);
        Constant * PEXT_func = nullptr;
        if (mCompressFieldWidth == 64) {
            PEXT_func = Intrinsic::getDeclaration(kb->getModule(), Intrinsic::x86_bmi_pext_64);
        } else if (mCompressFieldWidth == 32) {
            PEXT_func = Intrinsic::getDeclaration(kb->getModule(), Intrinsic::x86_bmi_pext_32);
        }
        const unsigned fieldsPerBlock = kb->getBitBlockWidth()/mCompressFieldWidth;
        Value * extractionMask = kb->fwCast(mCompressFieldWidth, maskVec[0]);
        std::vector<Value *> mask(fieldsPerBlock);
        for (unsigned i = 0; i < fieldsPerBlock; i++) {
            mask[i] = kb->CreateExtractElement(extractionMask, kb->getInt32(i));
        }
        for (unsigned j = 0; j < input.size(); ++j) {
            Value * fieldVec = kb->fwCast(mCompressFieldWidth, input[j]);
            Value * outputPtr = kb->getOutputStreamBlockPtr("outputStreamSet", kb->getInt32(j), blockOffsetPhi);
            outputPtr = kb->CreatePointerCast(outputPtr, fieldPtrTy);
            for (unsigned i = 0; i < fieldsPerBlock; i++) {
                Value * field = kb->CreateExtractElement(fieldVec, kb->getInt32(i));
                Value * compressed = kb->createCall(PEXT_func, {field, mask[i]});
                kb->CreateStore(compressed, kb->CreateGEP(outputPtr, kb->getInt32(i)));
            }
        }
    } else {
        Value * delMask = kb->simd_not(maskVec[0]);
        const auto move_masks = parallel_prefix_deletion_masks(kb, mCompressFieldWidth, delMask);
        for (unsigned j = 0; j < input.size(); ++j) {
            Value * output = apply_parallel_prefix_deletion(kb, mCompressFieldWidth, delMask, move_masks, input[j]);
            kb->storeOutputStreamBlock("outputStreamSet", kb->getInt32(j), blockOffsetPhi, output);
        }
    }
    Value * nextBlk = kb->CreateAdd(blockOffsetPhi, kb->getSize(1));
    blockOffsetPhi->addIncoming(nextBlk, processBlock);
    Value * moreToDo = kb->CreateICmpNE(nextBlk, numOfBlocks);
    kb->CreateCondBr(moreToDo, processBlock, done);
    kb->SetInsertPoint(done);
}

FieldCompressKernel::FieldCompressKernel(BuilderRef b,
                                         SelectOperation const & maskOp,
                                         SelectOperationList const & inputOps,
                                         StreamSet * outputStreamSet,
                                         unsigned fieldWidth)
: MultiBlockKernel(b, "fieldCompress" + std::to_string(fieldWidth) + "_" +
                   streamutils::genSignature(maskOp) +
                   ":" + streamutils::genSignature(inputOps),
{},
{Binding{"outputStreamSet", outputStreamSet}},
{}, {}, {})
, mCompressFieldWidth(fieldWidth) {
    mMaskOp.operation = maskOp.operation;
    mMaskOp.bindings.push_back(std::make_pair("extractionMask", maskOp.bindings[0].second));
    // assert (streamutil::resultStreamCount(maskOp) == 1);
    mInputStreamSets.push_back({"extractionMask", maskOp.bindings[0].first});
    // assert (streamutil::resultStreamCount(inputOps) == outputStreamSet->getNumElements());
    std::unordered_map<StreamSet *, std::string> inputBindings;
    std::tie(mInputOps, inputBindings) = streamutils::mapOperationsToStreamNames(inputOps);
    for (auto const & kv : inputBindings) {
        mInputStreamSets.push_back({kv.second, kv.first, FixedRate(), ZeroExtended()});
    }
}

void PEXTFieldCompressKernel::generateMultiBlockLogic(BuilderRef kb, llvm::Value * const numOfBlocks) {
    Type * fieldTy = kb->getIntNTy(mPEXTWidth);
    Type * fieldPtrTy = PointerType::get(fieldTy, 0);
    Constant * PEXT_func = nullptr;
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
            Value * compressed = kb->createCall(PEXT_func, {field, mask[i]});
            kb->CreateStore(compressed, kb->CreateGEP(outputPtr, kb->getInt32(i)));
        }
    }
    Value * nextBlk = kb->CreateAdd(blockOffsetPhi, kb->getSize(1));
    blockOffsetPhi->addIncoming(nextBlk, processBlock);
    Value * moreToDo = kb->CreateICmpNE(nextBlk, numOfBlocks);
    kb->CreateCondBr(moreToDo, processBlock, done);
    kb->SetInsertPoint(done);
}

PEXTFieldCompressKernel::PEXTFieldCompressKernel(BuilderRef b, const unsigned fieldWidth, const unsigned streamCount)
: MultiBlockKernel(b, "PEXTfieldCompress" + std::to_string(fieldWidth) + "_" + std::to_string(streamCount),
                   {Binding{b->getStreamSetTy(streamCount), "inputStreamSet"},
                       Binding{b->getStreamSetTy(), "extractionMask"}},
                   {Binding{b->getStreamSetTy(streamCount), "outputStreamSet"}},
                   {}, {}, {})
, mPEXTWidth(fieldWidth)
, mStreamCount(streamCount) {
    if ((fieldWidth != 32) && (fieldWidth != 64)) llvm::report_fatal_error("Unsupported PEXT width for PEXTFieldCompressKernel");
}

StreamCompressKernel::StreamCompressKernel(BuilderRef b
                                           , StreamSet * extractionMask
                                           , StreamSet * source
                                           , StreamSet * compressedOutput
                                           , const unsigned FieldWidth)
: MultiBlockKernel(b, "streamCompress" + std::to_string(FieldWidth) + "_" + std::to_string(source->getNumElements()),
{Binding{"extractionMask", extractionMask, FixedRate(), Principal()},
    Binding{"sourceStreamSet", source, FixedRate(), ZeroExtended()}},
{Binding{"compressedOutput", compressedOutput, PopcountOf("extractionMask")}},
{}, {}, {})
, mCompressedFieldWidth(FieldWidth)
, mStreamCount(source->getNumElements()) {
    for (unsigned i = 0; i < mStreamCount; i++) {
        addInternalScalar(b->getBitBlockType(), "pendingOutputBlock_" + std::to_string(i));
    }
}

void StreamCompressKernel::generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfBlocks) {
    IntegerType * const fwTy = b->getIntNTy(mCompressedFieldWidth);
    IntegerType * const sizeTy = b->getSizeTy();
    const unsigned numFields = b->getBitBlockWidth() / mCompressedFieldWidth;
    Constant * zeroSplat = Constant::getNullValue(b->fwVectorType(mCompressedFieldWidth));
    #if LLVM_VERSION_MAJOR < 10
        Constant * oneSplat = ConstantVector::getSplat(numFields, ConstantInt::get(fwTy, 1));
    #else
        Constant * oneSplat = ConstantVector::getSplat({numFields, false}, ConstantInt::get(fwTy, 1));
    #endif
    Constant * CFW = ConstantInt::get(fwTy, mCompressedFieldWidth);
    #if LLVM_VERSION_MAJOR < 10
        Constant * fwSplat = ConstantVector::getSplat(numFields, CFW);
    #else
        Constant * fwSplat = ConstantVector::getSplat({numFields, false}, CFW);
    #endif
    Constant * numFieldConst = ConstantInt::get(fwTy, numFields);
    #if LLVM_VERSION_MAJOR < 10
        Constant * fwMaskSplat = ConstantVector::getSplat(numFields, ConstantInt::get(fwTy, mCompressedFieldWidth - 1));
    #else
        Constant * fwMaskSplat = ConstantVector::getSplat({numFields, false}, ConstantInt::get(fwTy, mCompressedFieldWidth - 1));
    #endif
    Constant * BLOCK_WIDTH = ConstantInt::get(fwTy, b->getBitBlockWidth());
    Constant * BLOCK_MASK = ConstantInt::get(fwTy, b->getBitBlockWidth() - 1);

    BasicBlock * const segmentLoop = b->CreateBasicBlock("segmentLoop");
    BasicBlock * const segmentDone = b->CreateBasicBlock("segmentDone");
    BasicBlock * const writePartialData = b->CreateBasicBlock("writePartialData");
    BasicBlock * const segmentExit = b->CreateBasicBlock("segmentExit");

    Constant * const ZERO = ConstantInt::get(sizeTy, 0);
    Constant * const ONE = ConstantInt::get(sizeTy, 1);

    Value * const produced = b->getProducedItemCount("compressedOutput");



    Value * const pendingItemCount = b->CreateAnd(b->CreateZExtOrTrunc(produced, fwTy), BLOCK_MASK);

    // TODO: we could make this kernel stateless but would have to use "bitblock_mask_to" on the
    // input to account for the fact the pipeline doesn't zero out output stream data when it
    // reuses it.

    SmallVector<Value *, 16> pendingData(mStreamCount);
    for (unsigned i = 0; i < mStreamCount; i++) {
        pendingData[i] = b->getScalarField("pendingOutputBlock_" + std::to_string(i));
    }

    BasicBlock * const entry = b->GetInsertBlock();
    b->CreateBr(segmentLoop);
    // Main Loop
    b->SetInsertPoint(segmentLoop);
    PHINode * const inputOffsetPhi = b->CreatePHI(sizeTy, 2);
    PHINode * const outputOffsetPhi = b->CreatePHI(sizeTy, 2);
    PHINode * const pendingItemsPhi = b->CreatePHI(fwTy, 2);
    SmallVector<PHINode *, 16> pendingDataPhi(mStreamCount);

    inputOffsetPhi->addIncoming(ZERO, entry);
    outputOffsetPhi->addIncoming(ZERO, entry);
    pendingItemsPhi->addIncoming(pendingItemCount, entry);
    for (unsigned i = 0; i < mStreamCount; i++) {
        pendingDataPhi[i] = b->CreatePHI(pendingData[i]->getType(), 2);
        pendingDataPhi[i]->addIncoming(pendingData[i], entry);
    }
    Value * const extractionMask = b->loadInputStreamBlock("extractionMask", ZERO, inputOffsetPhi);
    Value * const fieldPopCounts = b->simd_popcount(mCompressedFieldWidth, extractionMask);
    // For each field determine the (partial) sum popcount of all fields up to and
    // including the current field.
    Value * const partialSum = b->hsimd_partial_sum(mCompressedFieldWidth, fieldPopCounts);
    Value * const newPendingItems = b->mvmd_extract(mCompressedFieldWidth, partialSum, numFields - 1);

    //
    // Now determine for each source field the output offset of the first bit.
    // Note that this depends on the number of pending bits.
    //
    Value * pendingOffset = b->CreateURem(pendingItemsPhi, CFW);
    Value * splatPending = b->simd_fill(mCompressedFieldWidth, b->CreateZExtOrTrunc(pendingOffset, fwTy));
    Value * pendingFieldIdx = b->CreateUDiv(pendingItemsPhi, CFW);
    Value * offsets = b->simd_add(mCompressedFieldWidth, b->mvmd_slli(mCompressedFieldWidth, partialSum, 1), splatPending);
    offsets = b->simd_and(offsets, fwMaskSplat); // parallel URem fw
   //
    // Determine the relative field number for each output field.   Note that the total
    // number of fields involved is numFields + 1.   However, the first field always
    // be immediately combined into the current pending data field, so we calculate
    // field numbers for all subsequent fields, (the fields that receive overflow bits).
    Value * pendingSum = b->simd_add(mCompressedFieldWidth, partialSum, splatPending);
    Value * fieldNo = b->simd_srli(mCompressedFieldWidth, pendingSum, std::log2(mCompressedFieldWidth));
    // Now process the input data block of each stream in the input stream set.
    //
    // First load all the stream set blocks and the pending data.
    SmallVector<Value *, 16> sourceBlock(mStreamCount);
    for (unsigned i = 0; i < mStreamCount; i++) {
        sourceBlock[i] = b->loadInputStreamBlock("sourceStreamSet", b->getInt32(i), inputOffsetPhi);
    }
    // Now separate the bits of each field into ones that go into the current field
    // and ones that go into the overflow field.   Extract the first field separately,
    // and then shift and combine subsequent fields.
    SmallVector<Value *, 16> pendingOutput(mStreamCount);
    SmallVector<Value *, 16> outputFields(mStreamCount);
    Value * backShift = b->simd_sub(mCompressedFieldWidth, fwSplat, offsets);
    for (unsigned i = 0; i < mStreamCount; i++) {
        Value * currentFieldBits = b->simd_sllv(mCompressedFieldWidth, sourceBlock[i], offsets);
        Value * nextFieldBits = b->simd_srlv(mCompressedFieldWidth, sourceBlock[i], backShift);
        Value * firstField = b->mvmd_extract(mCompressedFieldWidth, currentFieldBits, 0);
        Value * vec1 = b->CreateInsertElement(zeroSplat, firstField, pendingFieldIdx);
        pendingOutput[i] = b->simd_or(pendingDataPhi[i], vec1);
        // shift back currentFieldBits to combine with nextFieldBits.
        outputFields[i] = b->simd_or(b->mvmd_srli(mCompressedFieldWidth, currentFieldBits, 1), nextFieldBits);
    }
    // Now combine forward all fields with the same field number.  This may require
    // up to log2 numFields steps.
    for (unsigned j = 1; j < numFields; j*=2) {
        Value * select = b->simd_eq(mCompressedFieldWidth, fieldNo, b->mvmd_slli(mCompressedFieldWidth, fieldNo, j));
        for (unsigned i = 0; i < mStreamCount; i++) {
            Value * fields_fwd = b->mvmd_slli(mCompressedFieldWidth, outputFields[i], j);
            outputFields[i] = b->simd_or(outputFields[i], b->simd_and(select, fields_fwd));
       }
    }
    // Now compress the data fields, eliminating all but the last field from
    // each run of consecutive field having the same field number as a subsequent field.
    // But it may be that last field number is 0 which will compare equal to a 0 shifted in.
    // So we add 1 to field numbers first.
    Value * nonZeroFieldNo = b->simd_add(mCompressedFieldWidth, fieldNo, oneSplat);
    Value * eqNext = b->simd_eq(mCompressedFieldWidth, nonZeroFieldNo, b->mvmd_srli(mCompressedFieldWidth, nonZeroFieldNo, 1));
    Value * compressMask = b->hsimd_signmask(mCompressedFieldWidth, b->simd_not(eqNext));
    for (unsigned i = 0; i < mStreamCount; i++) {
        outputFields[i] = b->mvmd_compress(mCompressedFieldWidth, outputFields[i], compressMask);
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
    Value * increment = b->CreateZExtOrTrunc(b->mvmd_extract(mCompressedFieldWidth, fieldNo, 0), fwTy);
    pendingFieldIdx = b->CreateAdd(pendingFieldIdx, increment);
    Value * const pendingSpaceFilled = b->CreateICmpEQ(pendingFieldIdx, numFieldConst);
    Value * shftBack = b->CreateSub(numFieldConst, pendingFieldIdx);
    for (unsigned i = 0; i < mStreamCount; i++) {
        Value * shiftedField = b->mvmd_sll(mCompressedFieldWidth, outputFields[i], pendingFieldIdx);
        Value * outputFwd = b->fwCast(mCompressedFieldWidth, shiftedField);
        shiftedField = b->CreateSelect(pendingSpaceFilled, zeroSplat, outputFwd);
        pendingOutput[i] = b->simd_or(pendingOutput[i], shiftedField);
        outputFields[i] = b->mvmd_srl(mCompressedFieldWidth, outputFields[i], shftBack);
    }
    //
    // Write the pendingOutput data to outputStream.
    // Note: this data may be overwritten later, but we avoid branching.
    for (unsigned i = 0; i < mStreamCount; i++) {
        b->storeOutputStreamBlock("compressedOutput", b->getInt32(i), outputOffsetPhi, pendingOutput[i]);
    }
    // Now determine the total amount of pending items and whether
    // the pending data all fits within the pendingOutput.
    Value * nextPendingItems = b->CreateAdd(pendingItemsPhi, newPendingItems);
    Value * const doesFit = b->CreateICmpULT(nextPendingItems, BLOCK_WIDTH);
    nextPendingItems = b->CreateSelect(doesFit, nextPendingItems, b->CreateSub(nextPendingItems, BLOCK_WIDTH));
    pendingItemsPhi->addIncoming(nextPendingItems, segmentLoop);

    //
    // Prepare Phi nodes for the next iteration.
    //
    Value * const nextInputOffset = b->CreateAdd(inputOffsetPhi, ONE);
    inputOffsetPhi->addIncoming(nextInputOffset, segmentLoop);

    // But don't advance the output if all the data does fit into pendingOutput.
    Value * nextOutputOffset = b->CreateAdd(outputOffsetPhi, ONE);
    nextOutputOffset = b->CreateSelect(doesFit, outputOffsetPhi, nextOutputOffset);
    outputOffsetPhi->addIncoming(nextOutputOffset, segmentLoop);

    for (unsigned i = 0; i < mStreamCount; i++) {
        pendingOutput[i] = b->bitCast(b->CreateSelect(doesFit, pendingOutput[i], outputFields[i]));
        pendingDataPhi[i]->addIncoming(pendingOutput[i], segmentLoop);
    }
    //
    // Now continue the loop if there are more blocks to process.
    Value * moreToDo = b->CreateICmpNE(nextInputOffset, numOfBlocks);
    b->CreateCondBr(moreToDo, segmentLoop, segmentDone);

    b->SetInsertPoint(segmentDone);
    for (unsigned i = 0; i < mStreamCount; i++) {
        b->setScalarField("pendingOutputBlock_" + std::to_string(i), pendingOutput[i]);
    }

    // It's possible that we'll perfectly fill the last block of data on the
    // last iteration of the loop. If we arbritarily write the pending data,
    // this could end up incorrectly overwriting unprocessed data with 0s.
    Value * const hasMore = b->CreateICmpNE(nextPendingItems, ZERO);
    b->CreateLikelyCondBr(hasMore, writePartialData, segmentExit);

    b->SetInsertPoint(writePartialData);
    for (unsigned i = 0; i < mStreamCount; i++) {
        b->storeOutputStreamBlock("compressedOutput", b->getInt32(i), nextOutputOffset, pendingOutput[i]);
    }
    b->CreateBr(segmentExit);

    b->SetInsertPoint(segmentExit);
}

Bindings makeSwizzledDeleteByPEXTOutputBindings(const std::vector<StreamSet *> & outputStreamSets, const unsigned PEXTWidth) {
    const auto n = outputStreamSets.size();
    Bindings outputs;
    outputs.reserve(n);
    outputs.emplace_back("outputSwizzle0", outputStreamSets[0], PopcountOf("selectors"), BlockSize(PEXTWidth)); // PopcountOfNot("delMaskSet")
    for (unsigned i = 1; i < n; ++i) {
        outputs.emplace_back("outputSwizzle" + std::to_string(i), outputStreamSets[i], RateEqualTo("outputSwizzle0"), BlockSize(PEXTWidth));
    }
    return outputs;
}

SwizzledDeleteByPEXTkernel::SwizzledDeleteByPEXTkernel(BuilderRef b,
                                                       StreamSet * selectors, StreamSet * inputStreamSet,
                                                       const std::vector<StreamSet *> & outputStreamSets,
                                                       const unsigned PEXTWidth)

: MultiBlockKernel(b, "PEXTdel" + std::to_string(PEXTWidth) + "_" + std::to_string(inputStreamSet->getNumElements()),
{Binding{"selectors", selectors}, Binding{"inputStreamSet", inputStreamSet}},
makeSwizzledDeleteByPEXTOutputBindings(outputStreamSets, PEXTWidth),
{}, {}, {})
, mStreamCount(inputStreamSet->getNumElements())
, mSwizzleFactor(b->getBitBlockWidth() / PEXTWidth)
, mSwizzleSetCount(ceil_udiv(mStreamCount, mSwizzleFactor))
, mPEXTWidth(PEXTWidth) {

    assert((mPEXTWidth > 0) && ((mPEXTWidth & (mPEXTWidth - 1)) == 0) && "mDelCountFieldWidth must be a power of 2");
    assert(mSwizzleFactor > 1 && "mDelCountFieldWidth must be less than the block width");
    assert((mPEXTWidth == 64 || mPEXTWidth == 32) && "PEXT width must be 32 or 64");
    assert (mSwizzleSetCount);
    assert (outputStreamSets.size() == mSwizzleSetCount);
    assert (outputStreamSets[0]->getNumElements() == mSwizzleFactor);

    addInternalScalar(b->getBitBlockType(), "pendingSwizzleData0");
    for (unsigned i = 1; i < outputStreamSets.size(); ++i) {
        assert (outputStreamSets[i]->getNumElements() == mSwizzleFactor);
        addInternalScalar(b->getBitBlockType(), "pendingSwizzleData" + std::to_string(i));
    }
}

void SwizzledDeleteByPEXTkernel::generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfBlocks) {
    // We use delMask to apply the same PEXT delete operation to each stream in the input stream set

    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const beginLoop = b->CreateBasicBlock("beginLoop");

    ConstantInt * const ZERO = b->getSize(0);
    ConstantInt * const BLOCK_WIDTH_MASK = b->getSize(b->getBitBlockWidth() - 1);
    ConstantInt * const PEXT_WIDTH = b->getSize(mPEXTWidth);
    ConstantInt * const LOG_2_PEXT_WIDTH = b->getSize(std::log2(mPEXTWidth));
    ConstantInt * const LOG_2_SWIZZLE_FACTOR = b->getSize(std::log2(mSwizzleFactor));
    ConstantInt * const PEXT_WIDTH_MASK = b->getSize(mPEXTWidth - 1);

    // All output groups have the same count.
    Value * const baseOutputProduced = b->getProducedItemCount("outputSwizzle0");
    Value * const baseProducedOffset = b->CreateAnd(baseOutputProduced, BLOCK_WIDTH_MASK);

    // There is a separate vector of pending data for each swizzle group.
    std::vector<Value *> pendingData(mSwizzleSetCount);
    for (unsigned i = 0; i < mSwizzleSetCount; i++) {
        pendingData[i] = b->getScalarField("pendingSwizzleData" + std::to_string(i));
    }
    b->CreateBr(beginLoop);

    b->SetInsertPoint(beginLoop);
    PHINode * const strideIndex = b->CreatePHI(numOfBlocks->getType(), 2);
    strideIndex->addIncoming(ZERO, entry);
    PHINode * const producedOffsetPhi = b->CreatePHI(numOfBlocks->getType(), 2);
    producedOffsetPhi->addIncoming(baseProducedOffset, entry);
    std::vector<PHINode *> pendingDataPhi(mSwizzleSetCount);
    for (unsigned i = 0; i < mSwizzleSetCount; i++) {
        pendingDataPhi[i] = b->CreatePHI(pendingData[i]->getType(), 2);
        pendingDataPhi[i]->addIncoming(pendingData[i], entry);
        pendingData[i] = pendingDataPhi[i];
    }

    Value * const selectors = b->loadInputStreamBlock("selectors", strideIndex);

    const auto swizzleSets = makeSwizzleSets(b, selectors, strideIndex);

    Value * const newItemCounts = b->simd_popcount(mPEXTWidth, selectors);

    // Compress the PEXTedSwizzleSets
    // Output is written and committed to the output buffer one swizzle at a time.
    Value * producedOffset = producedOffsetPhi;

    // For each row i
    for (unsigned i = 0; i < mSwizzleFactor; i++) {

        // Generate code for each of the mSwizzleFactor fields making up a block.
        // We load the count for the field and process all swizzle groups accordingly.
        Value * const pendingOffset = b->CreateAnd(producedOffset, PEXT_WIDTH_MASK);
        Value * const newItemCount = b->CreateExtractElement(newItemCounts, i);
        Value * const pendingSpace = b->CreateSub(PEXT_WIDTH, pendingOffset);
        Value * const pendingSpaceFilled = b->CreateICmpUGE(newItemCount, pendingSpace);

        Value * const shiftVector = b->simd_fill(mPEXTWidth, pendingOffset);
        Value * const spaceVector = b->simd_fill(mPEXTWidth, pendingSpace);

        Value * const outputIndex = b->CreateLShr(producedOffset, LOG_2_PEXT_WIDTH);
        Value * const swizzleIndex = b->CreateAnd(outputIndex, mSwizzleFactor - 1);
        Value * const blockOffset = b->CreateLShr(outputIndex, LOG_2_SWIZZLE_FACTOR);

        // Data from the ith swizzle pack of each group is processed
        // according to the same newItemCount, pendingSpace, ...
        for (unsigned j = 0; j < mSwizzleSetCount; j++) {
            Value * const newItems = swizzleSets[j][i];
            // Combine as many of the new items as possible into the pending group.
            Value * const shiftedItems = b->CreateShl(newItems, shiftVector);
            Value * const combinedGroup = b->CreateOr(pendingData[j], shiftedItems);
            // To avoid an unpredictable branch, always store the combined group, whether full or not.
            b->storeOutputStreamBlock("outputSwizzle" + std::to_string(j), swizzleIndex, blockOffset, combinedGroup);
            // Any items in excess of the space available in the current pending group overflow for the next group.
            Value * overFlowGroup = b->CreateLShr(newItems, spaceVector);
            // If we filled the space, then the overflow group becomes the new pending group and the index is updated.
            pendingData[j] = b->CreateSelect(pendingSpaceFilled, overFlowGroup, combinedGroup);
        }
        producedOffset = b->CreateAdd(producedOffset, newItemCount);
    }

    BasicBlock * const finishedLoop = b->CreateBasicBlock("finishedLoop");
    Value * const nextStrideIndex = b->CreateAdd(strideIndex, b->getSize(1));
    BasicBlock * const loopEndBlock = b->GetInsertBlock();
    strideIndex->addIncoming(nextStrideIndex, loopEndBlock);
    for (unsigned i = 0; i < mSwizzleSetCount; i++) {
        pendingDataPhi[i]->addIncoming(pendingData[i], loopEndBlock);
    }
    producedOffsetPhi->addIncoming(producedOffset, loopEndBlock);
    Value * const doneLoop = b->CreateICmpEQ(nextStrideIndex, numOfBlocks);

    b->CreateUnlikelyCondBr(doneLoop, finishedLoop, beginLoop);

    b->SetInsertPoint(finishedLoop);
    for (unsigned i = 0; i < mSwizzleSetCount; i++) {
        b->setScalarField("pendingSwizzleData" + std::to_string(i), pendingData[i]);
    }
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

SwizzledDeleteByPEXTkernel::SwizzleSets SwizzledDeleteByPEXTkernel::makeSwizzleSets(BuilderRef b, llvm::Value * const selectors, Value * const strideIndex) {

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

    SwizzleSets swizzleSets;
    swizzleSets.reserve(mSwizzleSetCount);

    VectorType * const vecTy = b->fwVectorType(mPEXTWidth);

    UndefValue * const outputInitializer = UndefValue::get(vecTy);

    std::vector<Value *> input(mSwizzleFactor);
    // For each of the k swizzle sets required to apply PEXT to all input streams
    for (unsigned i = 0; i < mSwizzleSetCount; ++i) {

        for (unsigned j = 0; j < mSwizzleFactor; ++j) {
            const unsigned k = (i * mSwizzleFactor) + j;
            if (k < mStreamCount) {
                input[j] = b->CreateBitCast(b->loadInputStreamBlock("inputStreamSet", b->getInt32(k), strideIndex), vecTy);
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
                Value * const selected = b->createCall(pext, {field, masks[k]});
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

void DeleteByPEXTkernel::generateDoBlockMethod(BuilderRef kb) {
    Value * delMask = kb->loadInputStreamBlock("delMaskSet", kb->getInt32(0));
    generateProcessingLoop(kb, delMask);
}

void DeleteByPEXTkernel::generateFinalBlockMethod(BuilderRef kb, Value * remainingBytes) {
    IntegerType * vecTy = kb->getIntNTy(kb->getBitBlockWidth());
    Value * remaining = kb->CreateZExt(remainingBytes, vecTy);
    Value * EOF_del = kb->bitCast(kb->CreateShl(Constant::getAllOnesValue(vecTy), remaining));
    Value * delMask = kb->CreateOr(EOF_del, kb->loadInputStreamBlock("delMaskSet", kb->getInt32(0)));
    generateProcessingLoop(kb, delMask);
}

void DeleteByPEXTkernel::generateProcessingLoop(BuilderRef kb, Value * delMask) {
    Constant * PEXT_func = nullptr;
    if (mPEXTWidth == 64) {
        PEXT_func = Intrinsic::getDeclaration(kb->getModule(), Intrinsic::x86_bmi_pext_64);
    } else if (mPEXTWidth == 32) {
        PEXT_func = Intrinsic::getDeclaration(kb->getModule(), Intrinsic::x86_bmi_pext_32);
    }
    std::vector<Value *> masks(mSwizzleFactor);
    Value * const m = kb->fwCast(mPEXTWidth, kb->simd_not(delMask));
    for (unsigned i = 0; i < mSwizzleFactor; i++) {
        masks[i] = kb->CreateExtractElement(m, i);
    }

    for (unsigned i = 0; i < mStreamCount; ++i) {
        Value * input = kb->loadInputStreamBlock("inputStreamSet", kb->getInt32(i));
        Value * value = kb->fwCast(mPEXTWidth, input);
        Value * output = UndefValue::get(value->getType());
        for (unsigned j = 0; j < mSwizzleFactor; j++) {
            Value * field = kb->CreateExtractElement(value, j);
            Value * compressed = kb->createCall(PEXT_func, {field, masks[j]});
            output = kb->CreateInsertElement(output, compressed, j);
        }
        kb->storeOutputStreamBlock("outputStreamSet", kb->getInt32(i), output);
    }
    Value * delCount = kb->simd_popcount(mDelCountFieldWidth, kb->simd_not(delMask));
    kb->storeOutputStreamBlock("deletionCounts", kb->getInt32(0), kb->bitCast(delCount));
}

DeleteByPEXTkernel::DeleteByPEXTkernel(BuilderRef b, unsigned fw, unsigned streamCount, unsigned PEXT_width)
: BlockOrientedKernel(b, "PEXTdel" + std::to_string(fw) + "_" + std::to_string(streamCount) + "_" + std::to_string(PEXT_width),
              {Binding{b->getStreamSetTy(streamCount), "inputStreamSet"},
                  Binding{b->getStreamSetTy(), "delMaskSet"}},
              {}, {}, {}, {})
, mDelCountFieldWidth(fw)
, mStreamCount(streamCount)
, mSwizzleFactor(b->getBitBlockWidth() / PEXT_width)
, mPEXTWidth(PEXT_width) {
    mOutputStreamSets.emplace_back(b->getStreamSetTy(mStreamCount), "outputStreamSet", PopcountOfNot("delMaskSet"));
    mOutputStreamSets.emplace_back(b->getStreamSetTy(), "deletionCounts");
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

SwizzledBitstreamCompressByCount::SwizzledBitstreamCompressByCount(BuilderRef b, unsigned bitStreamCount, unsigned fieldWidth)
: BlockOrientedKernel(b, "swizzled_compress" + std::to_string(fieldWidth) + "_" + std::to_string(bitStreamCount),
                     {Binding{b->getStreamSetTy(), "countsPerStride"}}, {}, {}, {}, {})
, mBitStreamCount(bitStreamCount)
, mFieldWidth(fieldWidth)
, mSwizzleFactor(b->getBitBlockWidth() / fieldWidth)
, mSwizzleSetCount((mBitStreamCount + mSwizzleFactor - 1)/mSwizzleFactor) {
    assert((fieldWidth > 0) && ((fieldWidth & (fieldWidth - 1)) == 0) && "fieldWidth must be a power of 2");
    assert(mSwizzleFactor > 1 && "fieldWidth must be less than the block width");
    mInputStreamSets.push_back(Binding{b->getStreamSetTy(mSwizzleFactor, 1), "inputSwizzle0"});
    mOutputStreamSets.push_back(Binding{b->getStreamSetTy(mSwizzleFactor, 1), "outputSwizzle0", BoundedRate(0, 1)});
    addInternalScalar(b->getBitBlockType(), "pendingSwizzleData0");
    for (unsigned i = 1; i < mSwizzleSetCount; i++) {
        mInputStreamSets.push_back(Binding{b->getStreamSetTy(mSwizzleFactor, 1), "inputSwizzle" + std::to_string(i)});
        mOutputStreamSets.push_back(Binding{b->getStreamSetTy(mSwizzleFactor, 1), "outputSwizzle" + std::to_string(i), RateEqualTo("outputSwizzle0")});
        addInternalScalar(b->getBitBlockType(), "pendingSwizzleData" + std::to_string(i));
    }
    addInternalScalar(b->getSizeTy(), "pendingOffset");
}

void SwizzledBitstreamCompressByCount::generateDoBlockMethod(BuilderRef kb) {

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

        Value * const fieldWidths = kb->simd_fill(mFieldWidth, pendingOffset);

        // Data from the ith swizzle pack of each group is processed
        // according to the same newItemCount, pendingSpace, ...
        for (unsigned j = 0; j < mSwizzleSetCount; j++) {
            Value * newItems = kb->loadInputStreamBlock("inputSwizzle" + std::to_string(j), kb->getInt32(i));
            // Combine as many of the new items as possible into the pending group.
            Value * combinedGroup = kb->CreateOr(pendingData[j], kb->CreateShl(newItems, fieldWidths));
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
//    Value * newlyProduced = kb->CreateSub(kb->CreateShl(outputIndex, outputIndexShift), producedOffset);
//    Value * produced = kb->CreateAdd(outputProduced, newlyProduced);
    for (unsigned j = 0; j < mSwizzleSetCount; j++) {
        kb->setScalarField("pendingSwizzleData" + std::to_string(j), pendingData[j]);
    }
//    kb->setProducedItemCount("outputSwizzle0", produced);
}

void SwizzledBitstreamCompressByCount::generateFinalBlockMethod(BuilderRef kb, Value * /* remainingBytes */) {
    RepeatDoBlockLogic(kb);
    Constant * blockOffsetMask = kb->getSize(kb->getBitBlockWidth() - 1);
    Constant * outputIndexShift = kb->getSize(std::log2(mFieldWidth));

    Value * outputProduced = kb->getProducedItemCount("outputSwizzle0"); // All output groups have the same count.
    Value * producedOffset = kb->CreateAnd(outputProduced, blockOffsetMask);
    Value * outputIndex = kb->CreateLShr(producedOffset, outputIndexShift);
//    Value * pendingOffset = kb->getScalarField("pendingOffset");

    // Write the pending data.
    for (unsigned i = 0; i < mSwizzleSetCount; i++) {
        Value * pendingData = kb->getScalarField("pendingSwizzleData" + std::to_string(i));
        Value * outputStreamPtr = kb->getOutputStreamBlockPtr("outputSwizzle" + std::to_string(i), kb->getInt32(0));
        kb->CreateBlockAlignedStore(pendingData, kb->CreateGEP(outputStreamPtr, outputIndex));
    }
//    kb->setProducedItemCount("outputSwizzle0", kb->CreateAdd(pendingOffset, outputProduced));
}

}
