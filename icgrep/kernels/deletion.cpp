/*
 *  Copyright (c) 2016 International Characters.
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
    Value * delMaskPtr = getInputStream("delMaskSet", iBuilder->getInt32(0));
    Value * delMask = iBuilder->CreateBlockAlignedLoad(delMaskPtr);
    const auto move_masks = parallel_prefix_deletion_masks(iBuilder, mDeletionFieldWidth, delMask);
    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * inputStreamPtr = getInputStream("inputStreamSet", iBuilder->getInt32(j));
        Value * input = iBuilder->CreateBlockAlignedLoad(inputStreamPtr);
        Value * output = apply_parallel_prefix_deletion(iBuilder, mDeletionFieldWidth, delMask, move_masks, input);
        Value * outputStreamPtr = getOutputStream("outputStreamSet", iBuilder->getInt32(j));
        iBuilder->CreateBlockAlignedStore(output, outputStreamPtr);
    }
    Value * delCount = partial_sum_popcount(iBuilder, mDeletionFieldWidth, iBuilder->simd_not(delMask));
    Value * delCountPtr = getOutputStream("deletionCounts", iBuilder->getInt32(0));
    iBuilder->CreateBlockAlignedStore(iBuilder->bitCast(delCount), delCountPtr);
}

void DeletionKernel::generateFinalBlockMethod(Value * remainingBytes) {
    IntegerType * vecTy = iBuilder->getIntNTy(iBuilder->getBitBlockWidth());
    Value * remaining = iBuilder->CreateZExt(remainingBytes, vecTy);
    Value * EOF_del = iBuilder->bitCast(iBuilder->CreateShl(Constant::getAllOnesValue(vecTy), remaining));
    Value * const delmaskPtr = getInputStream("delMaskSet", iBuilder->getInt32(0));
    Value * delMask = iBuilder->CreateOr(EOF_del, iBuilder->CreateBlockAlignedLoad(delmaskPtr));
    const auto move_masks = parallel_prefix_deletion_masks(iBuilder, mDeletionFieldWidth, delMask);
    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * inputStreamPtr = getInputStream("inputStreamSet", iBuilder->getInt32(j));
        Value * input = iBuilder->CreateBlockAlignedLoad(inputStreamPtr);
        Value * output = apply_parallel_prefix_deletion(iBuilder, mDeletionFieldWidth, delMask, move_masks, input);
        Value * outputStreamPtr = getOutputStream("outputStreamSet", iBuilder->getInt32(j));
        iBuilder->CreateBlockAlignedStore(output, outputStreamPtr);
    }
    Value * delCount = partial_sum_popcount(iBuilder, mDeletionFieldWidth, iBuilder->simd_not(delMask));
    Value * delCountPtr = getOutputStream("deletionCounts", iBuilder->getInt32(0));
    iBuilder->CreateBlockAlignedStore(iBuilder->bitCast(delCount), delCountPtr);
}

DeletionKernel::DeletionKernel(IDISA::IDISA_Builder * iBuilder, unsigned fw, unsigned streamCount)
: BlockOrientedKernel(iBuilder, "del",
              {Binding{iBuilder->getStreamSetTy(streamCount), "inputStreamSet"},
               Binding{iBuilder->getStreamSetTy(), "delMaskSet"}},
              {Binding{iBuilder->getStreamSetTy(streamCount), "outputStreamSet"},
               Binding{iBuilder->getStreamSetTy(), "deletionCounts"}},
              {}, {}, {})
, mDeletionFieldWidth(fw)
, mStreamCount(streamCount) {
    mDoBlockUpdatesProducedItemCountsAttribute = false;
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

inline Value * apply_PEXT_deletion(IDISA::IDISA_Builder * iBuilder, const std::vector<Value *> & masks, Value * strm) {
    Value * PEXT_func = nullptr;
    if (PEXT_width == 64) PEXT_func = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::x86_bmi_pext_64);
    else if (PEXT_width == 32) PEXT_func = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::x86_bmi_pext_32);
    Value * v = iBuilder->fwCast(PEXT_width, strm);
    Value * output = Constant::getNullValue(v->getType());
    for (unsigned i = 0; i < iBuilder->getBitBlockWidth()/PEXT_width; i++) {
        Value * field = iBuilder->CreateExtractElement(v, i);
        Value * compressed = iBuilder->CreateCall(PEXT_func, {field, masks[i]});
        output = iBuilder->CreateInsertElement(output, compressed, i);
    }
    return output;
}

// Apply deletion to a set of stream_count input streams to produce a set of output streams.
// Kernel inputs: stream_count data streams plus one del_mask stream
// Outputs: the deleted streams, plus a partial sum popcount

void DeleteByPEXTkernel::generateDoBlockMethod() {
    Value * delMaskPtr = getInputStream("delMaskSet", iBuilder->getInt32(0));
    Value * delMask = iBuilder->CreateBlockAlignedLoad(delMaskPtr);
    const auto masks = get_PEXT_masks(iBuilder, delMask);
    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * inputStreamPtr = getInputStream("inputStreamSet", iBuilder->getInt32(j));
        Value * input = iBuilder->CreateBlockAlignedLoad(inputStreamPtr);
        Value * output = apply_PEXT_deletion(iBuilder, masks, input);
        Value * outputStreamPtr = getOutputStream("outputStreamSet", iBuilder->getInt32(j));
        iBuilder->CreateBlockAlignedStore(iBuilder->bitCast(output), outputStreamPtr);
    }
    Value * delCount = partial_sum_popcount(iBuilder, mDelCountFieldWidth, apply_PEXT_deletion(iBuilder, masks, iBuilder->simd_not(delMask)));
    Value * delCountPtr = getOutputStream("deletionCounts", iBuilder->getInt32(0));
    iBuilder->CreateBlockAlignedStore(iBuilder->bitCast(delCount), delCountPtr);
}

void DeleteByPEXTkernel::generateFinalBlockMethod(Value * remainingBytes) {
    IntegerType * vecTy = iBuilder->getIntNTy(iBuilder->getBitBlockWidth());
    Value * remaining = iBuilder->CreateZExt(remainingBytes, vecTy);
    Value * EOF_del = iBuilder->bitCast(iBuilder->CreateShl(Constant::getAllOnesValue(vecTy), remaining));
    Value * const delmaskPtr = getInputStream("delMaskSet", iBuilder->getInt32(0));
    Value * delMask = iBuilder->CreateOr(EOF_del, iBuilder->CreateBlockAlignedLoad(delmaskPtr));
    const auto masks = get_PEXT_masks(iBuilder, delMask);
    for (unsigned j = 0; j < mStreamCount; ++j) {
        Value * inputStreamPtr = getInputStream("inputStreamSet", iBuilder->getInt32(j));
        Value * input = iBuilder->CreateBlockAlignedLoad(inputStreamPtr);
        Value * output = apply_PEXT_deletion(iBuilder, masks, input);
        Value * outputStreamPtr = getOutputStream("outputStreamSet", iBuilder->getInt32(j));
        iBuilder->CreateBlockAlignedStore(iBuilder->bitCast(output), outputStreamPtr);
    }
    Value * delCount = partial_sum_popcount(iBuilder, mDelCountFieldWidth, apply_PEXT_deletion(iBuilder, masks, iBuilder->simd_not(delMask)));
    Value * delCountPtr = getOutputStream("deletionCounts", iBuilder->getInt32(0));
    iBuilder->CreateBlockAlignedStore(iBuilder->bitCast(delCount), delCountPtr);
}

DeleteByPEXTkernel::DeleteByPEXTkernel(IDISA::IDISA_Builder * iBuilder, unsigned fw, unsigned streamCount)
: BlockOrientedKernel(iBuilder, "PEXTdel",
                      {Binding{iBuilder->getStreamSetTy(streamCount), "inputStreamSet"},
                          Binding{iBuilder->getStreamSetTy(), "delMaskSet"}},
                      {Binding{iBuilder->getStreamSetTy(streamCount), "outputStreamSet"},
                          Binding{iBuilder->getStreamSetTy(), "deletionCounts"}},
                      {}, {}, {})
, mDelCountFieldWidth(fw)
, mStreamCount(streamCount) {
    mDoBlockUpdatesProducedItemCountsAttribute = false;
}

}
