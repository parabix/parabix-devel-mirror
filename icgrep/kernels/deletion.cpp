/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernels/kernel.h>
#include <kernels/deletion.h>
#include <IDISA/idisa_builder.h>
#include <llvm/IR/Value.h>

std::vector<Value *> parallel_prefix_deletion_masks(IDISA::IDISA_Builder * iBuilder, unsigned fw, Value * del_mask) {
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

Value * apply_parallel_prefix_deletion(IDISA::IDISA_Builder * iBuilder, unsigned fw, Value * del_mask, std::vector<Value *> mv, Value * strm) {
    Value * s = iBuilder->simd_and(strm, iBuilder->simd_not(del_mask));
    for (unsigned i = 0; i < mv.size(); i++) {
        unsigned shift = 1 << i;
        Value * t = iBuilder->simd_and(s, mv[i]);
        s = iBuilder->simd_or(iBuilder->simd_xor(s, t), iBuilder->simd_srli(fw, t, shift));
    }
    return s;
}

Value * partial_sum_popcount(IDISA::IDISA_Builder * iBuilder, unsigned fw, Value * mask) {
    Value * per_field = iBuilder->simd_popcount(fw, mask);
    for (unsigned move = 1; move < iBuilder->getBitBlockWidth()/fw; move *= 2) {
        per_field = iBuilder->simd_add(fw, per_field, iBuilder->mvmd_slli(fw, per_field, move));
    }
    return per_field;
}

// Apply deletion to a set of stream_count input streams to produce a set of output streams.
// Kernel inputs: stream_count data streams plus one del_mask stream
// Outputs: the deleted streams, plus a partial sum popcount

namespace kernel {

void DeletionKernel::generateDoBlockMethod() {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();

    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);

    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));

    Value * self = getParameter(doBlockFunction, "self");

    Value * blockNo = getScalarField(self, blockNoScalar);

    Value * inputStreamPtr = getStreamSetBlockPtr(self, "inputStreamSet", blockNo);

    Value * delMaskPtr = getStreamSetBlockPtr(self, "delMaskSet", blockNo);

    Value * outputStreamPtr = getStreamSetBlockPtr(self, "outputStreamSet", blockNo);

    Value * delCountPtr = getStreamSetBlockPtr(self, "deletionCounts", blockNo);

    Value * delMask = iBuilder->CreateBlockAlignedLoad(delMaskPtr, {iBuilder->getInt32(0), iBuilder->getInt32(0)});

    std::vector<Value *> move_masks = parallel_prefix_deletion_masks(iBuilder, mDeletionFieldWidth, delMask);

    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * input = iBuilder->CreateBlockAlignedLoad(inputStreamPtr, {iBuilder->getInt32(0), iBuilder->getInt32(j)});
        Value * output = apply_parallel_prefix_deletion(iBuilder, mDeletionFieldWidth, delMask, move_masks, input);
        iBuilder->CreateBlockAlignedStore(output, outputStreamPtr, {iBuilder->getInt32(0), iBuilder->getInt32(j)});
    }
    Value * counts = partial_sum_popcount(iBuilder, mDeletionFieldWidth, iBuilder->simd_not(delMask));
    iBuilder->CreateBlockAlignedStore(iBuilder->bitCast(counts), delCountPtr, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    /* Stream deletion has only been applied within fields; the actual number of data items
     * has not yet changed.   */
    Value * produced = getProducedItemCount(self);
    produced = iBuilder->CreateAdd(produced, ConstantInt::get(iBuilder->getSizeTy(), iBuilder->getStride()));
    setProducedItemCount(self, produced);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

void DeletionKernel::generateFinalBlockMethod() {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();

    unsigned blockSize = iBuilder->getBitBlockWidth();
    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    Function * finalBlockFunction = m->getFunction(mKernelName + finalBlock_suffix);

    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", finalBlockFunction, 0));
    Value * remainingBytes = getParameter(finalBlockFunction, "remainingBytes");
    Value * self = getParameter(finalBlockFunction, "self");
    Value * blockNo = getScalarField(self, blockNoScalar);
    Value * delMaskBlock = getStreamSetBlockPtr(self, "delMaskSet", blockNo);
    Value * remaining = iBuilder->CreateZExt(remainingBytes, iBuilder->getIntNTy(blockSize));
    Value * EOF_del = iBuilder->bitCast(iBuilder->CreateShl(Constant::getAllOnesValue(iBuilder->getIntNTy(blockSize)), remaining));
    Value * const delmaskPtr = iBuilder->CreateGEP(delMaskBlock, {iBuilder->getInt32(0), iBuilder->getInt32(0)});
    Value * const delmaskVal = iBuilder->CreateBlockAlignedLoad(delmaskPtr);
    iBuilder->CreateBlockAlignedStore(iBuilder->CreateOr(EOF_del, delmaskVal), delmaskPtr);
    iBuilder->CreateCall(doBlockFunction, {self});
    /* Adjust the produced item count */
    Value * produced = getProducedItemCount(self);
    produced = iBuilder->CreateSub(produced, ConstantInt::get(iBuilder->getSizeTy(), iBuilder->getStride()));
    setProducedItemCount(self, iBuilder->CreateAdd(produced, remainingBytes));

    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

}
