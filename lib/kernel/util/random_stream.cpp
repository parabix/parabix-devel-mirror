/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include <kernel/util/random_stream.h>
#include <llvm/IR/Module.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/core/streamset.h>
#include <toolchain/toolchain.h>

using namespace kernel;
using namespace llvm;

void RandomStreamKernel::generateInitializeMethod(BuilderRef b) {
    b->CreateSRandCall(b->getInt32(mSeed));
}

/* Generate the next segment of random values.   Input requirement:
   the output buffer has a full segment of space available. */
void RandomStreamKernel::generateDoSegmentMethod(BuilderRef b) {
    const size_t randIntSize = 32;  // from standard C rand function
    const size_t segmentItems = codegen::SegmentSize * codegen::BlockSize;
    //
    // The item width (mValueWidth) for the desired random value stream may
    // be a single bit or any power of 2.   Determine the number of 32-bit values
    // returned by rand that are necessary for a full segment of the random
    // value stream.
    const size_t segmentRandInts = segmentItems * mValueWidth/randIntSize;

    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const genNextRVsegment = b->CreateBasicBlock("genNextRVsegment");
    BasicBlock * const genFinalRVsegment = b->CreateBasicBlock("genFinalRVsegment");
    BasicBlock * const exit = b->CreateBasicBlock("exit");

    Constant * strmLgthConst = ConstantInt::get(b->getSizeTy(), mStreamLength);
    Constant * segmentItemConst = ConstantInt::get(b->getSizeTy(), segmentItems);
    Constant * segmentRandIntConst = ConstantInt::get(b->getSizeTy(), segmentRandInts);
    Value * produced = b->getProducedItemCount("randomValues");
    Value * rvBuffer = b->getRawOutputPointer("randomValues", b->getInt32(0));
    rvBuffer = b->CreateBitCast(rvBuffer, b->getInt32Ty()->getPointerTo());
    Value * addFullSegment = b->CreateAdd(produced, segmentItemConst);
    Value * moreToDoLater = b->CreateICmpULT(addFullSegment, strmLgthConst);
    b->CreateCondBr(moreToDoLater, genNextRVsegment, genFinalRVsegment);

    b->SetInsertPoint(genNextRVsegment);
    PHINode* rvNumPHI = b->CreatePHI(b->getSizeTy(), 2);
    rvNumPHI->addIncoming(b->getSize(0), entry);
    // Use b->getBitBlockWidth()/randIntSize as an unroll factor
    Value * rvNo = rvNumPHI;
    for (unsigned i = 0; i < b->getBitBlockWidth()/randIntSize; i++) {
        Value * randVal = b->CreateRandCall();
        b->CreateStore(randVal, b->CreateGEP(rvBuffer, rvNo));
        rvNo = b->CreateAdd(rvNo, b->getSize(1));
    }
    rvNumPHI->addIncoming(rvNo, genNextRVsegment);
    b->CreateCondBr(b->CreateICmpULT(rvNo, segmentRandIntConst), genNextRVsegment, exit);

    b->SetInsertPoint(genFinalRVsegment);
    const size_t remainingItems = mStreamLength % segmentItems;
    const size_t remainingBits = remainingItems * mValueWidth;
    const size_t remainingFullRandInts = remainingBits/randIntSize;
    const size_t partialRandInt = remainingBits % randIntSize;
    const size_t zeroFill = (segmentItems - remainingItems) * mValueWidth/randIntSize;
    if (remainingFullRandInts > 0) {
        rvNumPHI = b->CreatePHI(b->getSizeTy(), 2);
        rvNumPHI->addIncoming(b->getSize(0), entry);
        rvNo = rvNumPHI;
        Value * randVal = b->CreateRandCall();
        b->CreateStore(randVal, b->CreateGEP(rvBuffer, rvNo));
        rvNo = b->CreateAdd(rvNo, b->getSize(1));
        rvNumPHI->addIncoming(rvNo, b->GetInsertBlock());
        Value * moreToDo = b->CreateICmpULT(rvNo, ConstantInt::get(b->getSizeTy(), remainingFullRandInts));
        BasicBlock * const finalSegmentExit = b->CreateBasicBlock("finalSegmentExit");
        b->CreateCondBr(moreToDo, genFinalRVsegment, finalSegmentExit);
        b->SetInsertPoint(finalSegmentExit);
    }
    if (partialRandInt > 0) {
        Constant * mask = ConstantInt::get(b->getInt32Ty(), APInt::getLowBitsSet(randIntSize, partialRandInt));
        Value * partialRV = b->CreateAnd(mask, b->CreateRandCall());
        b->CreateStore(partialRV, b->CreateGEP(rvBuffer, rvNo));
    }
    if (zeroFill > 0) {
        if (partialRandInt > 0) rvNo = b->CreateAdd(rvNo, b->getSize(1));
        b->CreateMemZero(b->CreateGEP(rvBuffer, rvNo), b->getSize(zeroFill/8));
    }
    b->setTerminationSignal();
    BasicBlock * finalBB = b->GetInsertBlock();
    b->CreateBr(exit);
    // The required random values have all been written, update the
    // produced item count.
    b->SetInsertPoint(exit);
    PHINode* finalProducedPHI = b->CreatePHI(b->getSizeTy(), 2);
    finalProducedPHI->addIncoming(addFullSegment, genNextRVsegment);
    finalProducedPHI->addIncoming(strmLgthConst, finalBB);
    b->setProducedItemCount("randomValues", finalProducedPHI);
}

RandomStreamKernel::RandomStreamKernel(BuilderRef b, unsigned seed, unsigned valueWidth, size_t streamLength)
: SegmentOrientedKernel(b, "rand" + std::to_string(valueWidth) + "_" + std::to_string(seed) + "_" + std::to_string(streamLength),
// input
{},
// output
{Binding{b->getStreamSetTy(1, valueWidth), "randomValues"}},
// scalars
{}, {}, {})
, mSeed(seed)
, mValueWidth(valueWidth)
, mStreamLength(streamLength) {
}

