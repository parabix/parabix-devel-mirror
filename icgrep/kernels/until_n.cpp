/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "until_n.h"
#include <llvm/IR/Module.h>
#include <kernels/kernel_builder.h>
#include <kernels/streamset.h>

namespace llvm { class Type; }

using namespace llvm;
using namespace parabix;

namespace kernel {

const unsigned packSize = 64;
    
void UntilNkernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb) {
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
    
    BasicBlock * entry = kb->GetInsertBlock();
    
    BasicBlock * processGroups = kb->CreateBasicBlock("processGroups");
    BasicBlock * processBlockGroup = kb->CreateBasicBlock("processBlockGroup");
    BasicBlock * doScan = kb->CreateBasicBlock("doScan");
    BasicBlock * scanLoop = kb->CreateBasicBlock("scanLoop");
    BasicBlock * continueScanLoop = kb->CreateBasicBlock("continueScanLoop");
    BasicBlock * scanDone = kb->CreateBasicBlock("scanDone");
    BasicBlock * notFoundYet = kb->CreateBasicBlock("notFoundYet");
    BasicBlock * findNth = kb->CreateBasicBlock("findNth");
    BasicBlock * getPosnAfterNth = kb->CreateBasicBlock("getPosnAfterNth");
    BasicBlock * nthPosFound = kb->CreateBasicBlock("nthPosFound");
    BasicBlock * doSegmentReturn = kb->CreateBasicBlock("doSegmentReturn");
    Constant * blockSize = kb->getSize(kb->getBitBlockWidth());
    Constant * blockSizeLess1 = kb->getSize(kb->getBitBlockWidth() - 1);
    Constant * packsPerBlock = kb->getSize(kb->getBitBlockWidth()/packSize);
    
    Value * N = kb->getScalarField("N");
    
    // Set up the types for processing by pack.
    Type * iPackTy = kb->getIntNTy(packSize);
    Type * iPackPtrTy = iPackTy->getPointerTo();
    
    Function::arg_iterator args = mCurrentMethod->arg_begin();
    /* self = */ args++;
    Value * itemsToDo = &*(args++);
    Value * sourceBitstream = &*(args++);
    Value * uptoN_bitstream = &*(args);
    
    // Compute the ceiling of the number of blocks to do.  If we have a final
    // partial block, it is treated as a full block initially.   
    Value * blocksToDo = kb->CreateUDiv(kb->CreateAdd(itemsToDo, blockSizeLess1), blockSize);
    
    // We will create a bitmask of size packSize with one bit for every packSize positions.
    // The index can accommodate blocksPerGroup blocks.
    Constant * blocksPerGroup = kb->getSize(packSize/((kb->getBitBlockWidth()/packSize)));
    kb->CreateCondBr(kb->CreateICmpUGT(blocksToDo, kb->getSize(0)), processGroups, notFoundYet);
    
    // Each iteration of the outerloop processes one blockGroup of at most blocksPerGroup.
    kb->SetInsertPoint(processGroups);
    PHINode * blockGroupBase = kb->CreatePHI(kb->getSizeTy(), 2);
    blockGroupBase->addIncoming(kb->getSize(0), entry);
    Value * groupPackPtr = kb->CreatePointerCast(kb->CreateGEP(sourceBitstream, blockGroupBase), iPackPtrTy);
    Value * blockGroupLimit = kb->CreateAdd(blockGroupBase, blocksPerGroup);
    blockGroupLimit = kb->CreateSelect(kb->CreateICmpULT(blockGroupLimit, blocksToDo), blockGroupLimit, blocksToDo);
    kb->CreateBr(processBlockGroup);

    // Outer loop processes the blocksToDo in groups of up to blocksPerGroup at a time.
    // The bitmask for this group is assembled.
    kb->SetInsertPoint(processBlockGroup);
    PHINode * blockNo = kb->CreatePHI(kb->getSizeTy(), 2);
    PHINode * groupMask = kb->CreatePHI(iPackTy, 2);
    blockNo->addIncoming(blockGroupBase, processGroups);
    groupMask->addIncoming(ConstantInt::getNullValue(iPackTy), processGroups);

    Value * blk = kb->CreateBlockAlignedLoad(kb->CreateGEP(sourceBitstream, {blockNo, kb->getInt32(0)}));
    kb->CreateBlockAlignedStore(blk, kb->CreateGEP(uptoN_bitstream, {blockNo, kb->getInt32(0)}));
    Value * hasbit = kb->simd_ugt(packSize, blk, kb->allZeroes());
    Value * blockMask = kb->CreateZExtOrTrunc(kb->hsimd_signmask(packSize, hasbit), iPackTy);
    Value * nextBlockNo = kb->CreateAdd(blockNo, kb->getSize(1));
    Value * blockMaskPosition = kb->CreateMul(kb->CreateSub(blockNo, blockGroupBase), packsPerBlock);
    Value * nextgroupMask = kb->CreateOr(groupMask, kb->CreateShl(blockMask, blockMaskPosition));
    blockNo->addIncoming(nextBlockNo, processBlockGroup);
    groupMask->addIncoming(nextgroupMask, processBlockGroup);
    kb->CreateCondBr(kb->CreateICmpULT(nextBlockNo, blockGroupLimit), processBlockGroup, doScan);

    // The index pack has been assembled - process the corresponding blocks.
    kb->SetInsertPoint(doScan);
    Value * seenSoFar = kb->getScalarField("seenSoFar");
    kb->CreateCondBr(kb->CreateICmpUGT(nextgroupMask, ConstantInt::getNullValue(iPackTy)), scanLoop, scanDone);
    
    kb->SetInsertPoint(scanLoop);
    PHINode * groupMaskPhi = kb->CreatePHI(iPackTy, 2);
    groupMaskPhi->addIncoming(nextgroupMask, doScan);
    PHINode * seenSoFarPhi = kb->CreatePHI(kb->getSizeTy(), 2);
    seenSoFarPhi->addIncoming(seenSoFar, doScan);
    Value * nonZeroPack = kb->CreateZExtOrTrunc(kb->CreateCountForwardZeroes(groupMaskPhi), kb->getSizeTy());
    Value * scanMask = kb->CreateLoad(kb->CreateGEP(groupPackPtr, nonZeroPack));
    Value * packCount = kb->CreateZExtOrTrunc(kb->CreatePopcount(scanMask), kb->getSizeTy());
    Value * newTotalSeen = kb->CreateAdd(packCount, seenSoFarPhi);
    Value * seenLessThanN = kb->CreateICmpULT(newTotalSeen, N);
    kb->CreateCondBr(seenLessThanN, continueScanLoop, findNth);

    kb->SetInsertPoint(continueScanLoop);
    Value * reducedGroupMask = kb->CreateResetLowestBit(groupMaskPhi);
    groupMaskPhi->addIncoming(reducedGroupMask, continueScanLoop);
    seenSoFarPhi->addIncoming(newTotalSeen, continueScanLoop);
    kb->CreateCondBr(kb->CreateICmpUGT(reducedGroupMask, ConstantInt::getNullValue(iPackTy)), scanLoop, scanDone);

    // Now we have processed the group of blocks and updated the number of positions
    // seenSoFar without finding the Nth bit.  
    kb->SetInsertPoint(scanDone);
    PHINode * newTotalSeenPhi = kb->CreatePHI(kb->getSizeTy(), 2);
    newTotalSeenPhi->addIncoming(seenSoFar, doScan);
    newTotalSeenPhi->addIncoming(newTotalSeen, continueScanLoop);
    kb->setScalarField("seenSoFar", newTotalSeenPhi);
    blockGroupBase->addIncoming(nextBlockNo, scanDone);
    kb->CreateCondBr(kb->CreateICmpULT(nextBlockNo, blocksToDo), processGroups, notFoundYet);

    kb->SetInsertPoint(notFoundYet);
    // Now we have determined that the Nth bit has not been found in the entire
    // set of itemsToDo.
    
    Value * finalCount = kb->CreateAdd(kb->getProducedItemCount("uptoN"), itemsToDo);
    kb->setProducedItemCount("uptoN", finalCount);
    kb->CreateBr(doSegmentReturn);

    //
    // With the last input scanMask loaded, the count of one bits seen reaches or
    // exceeds N.  Determine the position immediately after the Nth one bit.
    // 
    kb->SetInsertPoint(findNth);
    
    PHINode * seen1 = kb->CreatePHI(kb->getSizeTy(), 2);
    seen1->addIncoming(seenSoFarPhi, scanLoop);
    PHINode * remainingBits = kb->CreatePHI(iPackTy, 2);
    remainingBits->addIncoming(scanMask, scanLoop);
    Value * clearLowest = kb->CreateResetLowestBit(remainingBits);
    Value * oneMoreSeen = kb->CreateAdd(seen1, kb->getSize(1));
    seen1->addIncoming(oneMoreSeen, findNth);
    remainingBits->addIncoming(clearLowest, findNth);
    kb->CreateCondBr(kb->CreateICmpULT(oneMoreSeen, N), findNth, getPosnAfterNth);

    //
    // We have cleared the low bits of scanMask up to and including the Nth in the stream.
    kb->SetInsertPoint(getPosnAfterNth);
    Value * scanMaskUpToN = kb->CreateXor(scanMask, clearLowest);
    Value * posnInPack = kb->CreateSub(ConstantInt::get(iPackTy, packSize), kb->CreateCountReverseZeroes(scanMaskUpToN));
    Value * posnInGroup = kb->CreateAdd(kb->CreateMul(nonZeroPack, kb->getSize(packSize)), posnInPack);
    Value * posnInItemsToDo = kb->CreateAdd(kb->CreateMul(blockGroupBase, blockSize), posnInGroup);
    // It is conceivable that we found a bit at a position beyond the given itemsToDo,
    // when we have a partial pack at the end of input.  In this case, the Nth bit does
    // not exist in the valid range of itemsToDo.
    kb->CreateCondBr(kb->CreateICmpUGE(posnInItemsToDo, itemsToDo), notFoundYet, nthPosFound);
    
    kb->SetInsertPoint(nthPosFound);
    finalCount = kb->CreateAdd(kb->getProcessedItemCount("bits"), posnInItemsToDo);
    Value * finalBlock = kb->CreateUDiv(posnInItemsToDo, blockSize);
    blk = kb->CreateBlockAlignedLoad(kb->CreateGEP(sourceBitstream, {finalBlock, kb->getInt32(0)}));
    blk = kb->CreateAnd(blk, kb->CreateNot(kb->bitblock_mask_from(kb->CreateURem(posnInItemsToDo, blockSize))));
    Value * outputPtr = kb->CreateGEP(uptoN_bitstream, {finalBlock, kb->getInt32(0)});
    kb->CreateBlockAlignedStore(blk, outputPtr);
    kb->setProcessedItemCount("bits", finalCount);
    kb->setProducedItemCount("uptoN", finalCount);
    kb->setTerminationSignal();
    kb->CreateBr(doSegmentReturn);
    
    kb->SetInsertPoint(doSegmentReturn);
}

UntilNkernel::UntilNkernel(const std::unique_ptr<kernel::KernelBuilder> & kb)
: MultiBlockKernel("UntilN", {Binding{kb->getStreamSetTy(1, 1), "bits"}},
                             {Binding{kb->getStreamSetTy(1, 1), "uptoN", MaxRatio(1)}},
                             {Binding{kb->getSizeTy(), "N"}}, {},
                             {Binding{kb->getSizeTy(), "seenSoFar"}}) {
}

}
