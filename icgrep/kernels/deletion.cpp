/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "deletion.h"
#include <IR_Gen/idisa_builder.h>
#include <llvm/IR/Value.h>
#include <llvm/IR/Module.h>

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

void DeletionKernel::generateDoBlockMethod() const {

    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();

    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);

    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));

    Value * self = getParameter(doBlockFunction, "self");

    Value * blockNo = getScalarField(self, blockNoScalar);

    Value * delMaskPtr = getStream(self, "delMaskSet", blockNo, iBuilder->getInt32(0));
    Value * delMask = iBuilder->CreateBlockAlignedLoad(delMaskPtr);

    std::vector<Value *> move_masks = parallel_prefix_deletion_masks(iBuilder, mDeletionFieldWidth, delMask);

    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * inputStreamPtr = getStream(self, "inputStreamSet", blockNo, iBuilder->getInt32(j));
        Value * input = iBuilder->CreateBlockAlignedLoad(inputStreamPtr);

        Value * output = apply_parallel_prefix_deletion(iBuilder, mDeletionFieldWidth, delMask, move_masks, input);

        Value * outputStreamPtr = getStream(self, "outputStreamSet", blockNo, iBuilder->getInt32(j));
        iBuilder->CreateBlockAlignedStore(output, outputStreamPtr);
    }
    Value * delCount = partial_sum_popcount(iBuilder, mDeletionFieldWidth, iBuilder->simd_not(delMask));
    Value * delCountPtr = getStream(self, "deletionCounts", blockNo, iBuilder->getInt32(0));
    iBuilder->CreateBlockAlignedStore(iBuilder->bitCast(delCount), delCountPtr);
    /* Stream deletion has only been applied within fields; the actual number of data items
     * has not yet changed.   */
    Value * produced = getProducedItemCount(self, "outputStreamSet");
    produced = iBuilder->CreateAdd(produced, iBuilder->getSize(iBuilder->getStride()));
    setProducedItemCount(self, "outputStreamSet", produced);
    setProducedItemCount(self, "deletionCounts", produced);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

void DeletionKernel::generateFinalBlockMethod() const {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();

    unsigned blockSize = iBuilder->getBitBlockWidth();
    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    Function * finalBlockFunction = m->getFunction(mKernelName + finalBlock_suffix);

    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", finalBlockFunction, 0));
    Value * remainingBytes = getParameter(finalBlockFunction, "remainingBytes");
    Value * self = getParameter(finalBlockFunction, "self");
    Value * blockNo = getScalarField(self, blockNoScalar);
    Value * remaining = iBuilder->CreateZExt(remainingBytes, iBuilder->getIntNTy(blockSize));
    Value * EOF_del = iBuilder->bitCast(iBuilder->CreateShl(Constant::getAllOnesValue(iBuilder->getIntNTy(blockSize)), remaining));
    Value * const delmaskPtr = getStream(self, "delMaskSet", blockNo, iBuilder->getInt32(0));
    Value * const delmaskVal = iBuilder->CreateBlockAlignedLoad(delmaskPtr);
    iBuilder->CreateBlockAlignedStore(iBuilder->CreateOr(EOF_del, delmaskVal), delmaskPtr);
    iBuilder->CreateCall(doBlockFunction, {self});
    /* Adjust the produced item count */
    Value * produced = getProducedItemCount(self, "outputStreamSet");
    produced = iBuilder->CreateSub(produced, iBuilder->getSize(iBuilder->getStride()));
    produced =  iBuilder->CreateAdd(produced, remainingBytes);
    setProducedItemCount(self, "outputStreamSet", produced);
    setProducedItemCount(self, "deletionCounts", produced);

    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

DeletionKernel::DeletionKernel(IDISA::IDISA_Builder * iBuilder, unsigned fw, unsigned streamCount) :
KernelBuilder(iBuilder, "del",
              {Binding{iBuilder->getStreamSetTy(streamCount), "inputStreamSet"},
               Binding{iBuilder->getStreamSetTy(), "delMaskSet"}},
              {Binding{iBuilder->getStreamSetTy(streamCount), "outputStreamSet"},
               Binding{iBuilder->getStreamSetTy(), "deletionCounts"}},
              {}, {}, {}),
mDeletionFieldWidth(fw),
mStreamCount(streamCount) {
    mDoBlockUpdatesProducedItemCountsAttribute = true;
}

}
