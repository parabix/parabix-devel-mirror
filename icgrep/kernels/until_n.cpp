/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "until_n.h"
#include <llvm/IR/Module.h>
#include <kernels/kernel_builder.h>
#include <kernels/streamset.h>
#include <toolchain/toolchain.h>

namespace llvm { class Type; }

using namespace llvm;
using namespace parabix;

namespace kernel {

void UntilNkernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfStrides) {

/*  
   Strategy:  first form an index consisting of one bit per packsize input positions,
   with a 1 bit signifying that the corresponding pack has at least one 1 bit.
   Build the index one pack at a time, i.e, packsize * packsize positions at a time.
   After an index pack is constructed, scan the index pack for 1 bits.  Each 1 bit
   found identifies an input pack with a nonzero popcount.  Take the actual popcount
   of the corresponding input pack and update the total number of bits seen.   If
   the number of bits seen reaches N with any pack, determine the position of the 
   Nth bit and signal termination at that point.
 
   For normal processing, we process whole blocks only, always advanced processed
   and produced item counts by an integral number of blocks.   For final block
   processing, we treat the final partial block as a whole block for the purpose
   of finding the Nth bit.   However, if the located bit position is past the
   EOF position, then this is treated as if the Nth bit does not exist in the
   input stream.
*/

    IntegerType * const sizeTy = b->getSizeTy();
    const unsigned packSize = sizeTy->getBitWidth();
    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);
    const auto packsPerBlock = b->getBitBlockWidth() / packSize;
    Constant * const PACK_SIZE = b->getSize(packSize);
    Constant * const PACKS_PER_BLOCK = b->getSize(packsPerBlock);
    VectorType * const vTy = VectorType::get(sizeTy, packsPerBlock);
    Value * const ZEROES = Constant::getNullValue(vTy);

    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const strideLoop = b->CreateBasicBlock("strideLoop");

    Value * const allAvailableItems = b->getAvailableItemCount("bits");

    b->CreateBr(strideLoop);
    b->SetInsertPoint(strideLoop);
    PHINode * const strideIndex = b->CreatePHI(sizeTy, 2);
    strideIndex->addIncoming(ZERO, entry);

    const auto n = (packSize * packSize) / b->getBitBlockWidth();
    Value * groupMask = nullptr;
    Value * const baseOffset = b->CreateMul(strideIndex, b->getSize(n));
    for (unsigned i = 0; i < n; ++i) {
        Value * offset = b->CreateNUWAdd(baseOffset, b->getSize(i));
        Value * inputPtr = b->getInputStreamBlockPtr("bits", ZERO, offset);
        Value * inputValue = b->CreateBlockAlignedLoad(inputPtr);
        Value * outputPtr = b->getOutputStreamBlockPtr("uptoN", ZERO, offset);
        b->CreateBlockAlignedStore(inputValue, outputPtr);
        Value * markers = b->CreateNot(b->simd_eq(packSize, inputValue, ZEROES));
        Value * blockMask = b->CreateZExtOrTrunc(b->hsimd_signmask(packSize, markers), sizeTy);
        if (i) {
            blockMask = b->CreateShl(blockMask, i * packsPerBlock);
            groupMask = b->CreateOr(groupMask, blockMask);
        } else {
            groupMask = blockMask;
        }
    }

    BasicBlock * const processGroups = b->CreateBasicBlock("processGroups");
    BasicBlock * const nextStride = b->CreateBasicBlock("nextStride");

    b->CreateLikelyCondBr(b->CreateIsNull(groupMask), nextStride, processGroups);

    b->SetInsertPoint(processGroups);
    Value * const N = b->getScalarField("N");
    Value * const initiallyObserved = b->getScalarField("observed");
    BasicBlock * const processGroup = b->CreateBasicBlock("processGroup");
    b->CreateBr(processGroup);

    b->SetInsertPoint(processGroup);
    PHINode * const observed = b->CreatePHI(initiallyObserved->getType(), 2);
    observed->addIncoming(initiallyObserved, processGroups);
    PHINode * const groupMarkers = b->CreatePHI(groupMask->getType(), 2);
    groupMarkers->addIncoming(groupMask, processGroups);

    Value * const groupIndex = b->CreateZExtOrTrunc(b->CreateCountForwardZeroes(groupMarkers), sizeTy);
    Value * const blockIndex = b->CreateNUWAdd(baseOffset, b->CreateUDiv(groupIndex, PACKS_PER_BLOCK));
    Value * const packOffset = b->CreateURem(groupIndex, PACKS_PER_BLOCK);
    Value * const groupPtr = b->getInputStreamBlockPtr("bits", ZERO, blockIndex);
    Value * const groupValue = b->CreateBlockAlignedLoad(groupPtr);
    Value * const packBits = b->CreateExtractElement(b->CreateBitCast(groupValue, vTy), packOffset);
    //Type * packPtrTy = packTy->getPointerTo();
    //Value * const packPtr = b->CreateGEP(b->CreatePointerCast(groupPtr, packPtrTy), packOffset);
    //Value * const packBits = b->CreateLoad(packPtr);
    Value * const packCount = b->CreateZExtOrTrunc(b->CreatePopcount(packBits), sizeTy);
    Value * const observedUpTo = b->CreateNUWAdd(observed, packCount);

    BasicBlock * const haveNotSeenEnough = b->CreateBasicBlock("haveNotSeenEnough");
    BasicBlock * const seenNOrMore = b->CreateBasicBlock("seenNOrMore");
    b->CreateLikelyCondBr(b->CreateICmpULT(observedUpTo, N), haveNotSeenEnough, seenNOrMore);

    // update our kernel state and check whether we have any other groups to process
    b->SetInsertPoint(haveNotSeenEnough);
    observed->addIncoming(observedUpTo, haveNotSeenEnough);
    b->setScalarField("observed", observedUpTo);
    Value * const remainingGroupMarkers = b->CreateResetLowestBit(groupMarkers);
    groupMarkers->addIncoming(remainingGroupMarkers, haveNotSeenEnough);
    b->CreateLikelyCondBr(b->CreateIsNull(remainingGroupMarkers), nextStride, processGroup);

    // we've seen N non-zero items; determine the position of our items and clear any subsequent markers
    b->SetInsertPoint(seenNOrMore);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        b->CreateAssert(b->CreateICmpUGT(N, observed), "N must be greater than observed count!");
    }
    Value * const bitsToFind = b->CreateNUWSub(N, observed);
    BasicBlock * const findNthBit = b->CreateBasicBlock("findNthBit");
    BasicBlock * const foundNthBit = b->CreateBasicBlock("foundNthBit");
    b->CreateBr(findNthBit);

    b->SetInsertPoint(findNthBit);
    PHINode * const remainingBitsToFind = b->CreatePHI(bitsToFind->getType(), 2);
    remainingBitsToFind->addIncoming(bitsToFind, seenNOrMore);
    PHINode * const remainingBits = b->CreatePHI(packBits->getType(), 2);
    remainingBits->addIncoming(packBits, seenNOrMore);
    Value * const nextRemainingBits = b->CreateResetLowestBit(remainingBits);
    remainingBits->addIncoming(nextRemainingBits, findNthBit);
    Value * const nextRemainingBitsToFind = b->CreateNUWSub(remainingBitsToFind, ONE);
    remainingBitsToFind->addIncoming(nextRemainingBitsToFind, findNthBit);
    b->CreateLikelyCondBr(b->CreateIsNull(nextRemainingBitsToFind), foundNthBit, findNthBit);

    // If we've found the n-th bit, end the segment after clearing the markers
    b->SetInsertPoint(foundNthBit);
    Value * const inputPtr = b->getInputStreamBlockPtr("bits", ZERO, blockIndex);
    Value * const inputValue = b->CreateBlockAlignedLoad(inputPtr);
    Value * const packPosition = b->CreateZExtOrTrunc(b->CreateCountForwardZeroes(remainingBits), sizeTy);
    Value * const basePosition = b->CreateNUWMul(packOffset, PACK_SIZE);
    Value * const blockOffset = b->CreateNUWAdd(b->CreateOr(basePosition, packPosition), ONE);
    Value * const mask = b->CreateNot(b->bitblock_mask_from(blockOffset));
    Value * const maskedInputValue = b->CreateAnd(inputValue, mask);
    Value * const outputPtr = b->getOutputStreamBlockPtr("uptoN", ZERO, blockIndex);
    b->CreateBlockAlignedStore(maskedInputValue, outputPtr);
    Value * const positionOfNthItem = b->CreateNUWAdd(b->CreateMul(blockIndex, b->getSize(b->getBitBlockWidth())), blockOffset);
    b->setTerminationSignal();
    BasicBlock * const segmentDone = b->CreateBasicBlock("segmentDone");
    b->CreateBr(segmentDone);

    nextStride->moveAfter(foundNthBit);

    b->SetInsertPoint(nextStride);
    Value * const nextStrideIndex = b->CreateNUWAdd(strideIndex, ONE);
    strideIndex->addIncoming(nextStrideIndex, nextStride);
    b->CreateLikelyCondBr(b->CreateICmpEQ(nextStrideIndex, numOfStrides), segmentDone, strideLoop);

    b->SetInsertPoint(segmentDone);
    PHINode * const produced = b->CreatePHI(sizeTy, 2);
    produced->addIncoming(positionOfNthItem, foundNthBit);
    produced->addIncoming(allAvailableItems, nextStride);
    Value * producedCount = b->getProducedItemCount("uptoN");
    producedCount = b->CreateNUWAdd(producedCount, produced);
    b->setProducedItemCount("uptoN", producedCount);

}

unsigned LLVM_READNONE calculateRate(const std::unique_ptr<kernel::KernelBuilder> & b) {
    const unsigned packSize = b->getSizeTy()->getBitWidth();
    return (packSize * packSize) / b->getBitBlockWidth();
}

UntilNkernel::UntilNkernel(const std::unique_ptr<kernel::KernelBuilder> & b)
: MultiBlockKernel("UntilN_" + std::to_string(calculateRate(b)),
// inputs
{Binding{b->getStreamSetTy(), "bits", FixedRate(calculateRate(b))}},
// outputs
{Binding{b->getStreamSetTy(), "uptoN", BoundedRate(0, calculateRate(b))}},
// input scalar
{Binding{b->getSizeTy(), "N"}}, {},
// internal state
{Binding{b->getSizeTy(), "observed"}}) {

}

}
