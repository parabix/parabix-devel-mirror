/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "expand3_4.h"
#include <kernels/kernel.h>
#include <IDISA/idisa_builder.h>
#include <llvm/Support/raw_ostream.h>

namespace kernel {
using namespace llvm;

// This kernel produces an expanded input stream by duplicating every third byte.
// It is implemented using SIMD shufflevector operations.  With 16-byte registers,
// a single shufflevector operation produces 16 bytes of output data from the
// 12 bytes of input data.   With 32-byte registers, 32 bytes of output data are
// produced from 24 bytes of input data.
//
// Using aligned SIMD loads, an inner loop processes three registers full of input
// data (i.e., three BytePacks) to produce four registers full of output.   This is 
// a 3 step process.
// Step 1:  Load input_pack0, apply the shuffle operation to produce output_pack0.
//          At this point 3/4 of the data in input_pack0 has been processed.
// Step 2:  Load input_pack1, apply a shuffle operation to use the remaining
//          1/4 of input_pack0 and 1/2 of input_pack1 to produce output_pack1.
//          At this point 1/2 of the data in input_pack1 has been processed.
// Step 3:  Load input_pack2, apply a shuffle operation to use the remaining 1/2
//          of input_pack1 and 1/4 of input_pack2 to produce output_pack2.
//          Then apply a further shuffle opertaion to use the remaining 3/4 of
//          input_pack2 to produce output_pack3.

// The doSegment method processes input in terms of tripleBlocks, 3 blocks of input,
// producing 4 blocks of output.   Unless less than one tripleBlock remains, the
// doSegment method always processes an integral number of tripleBlocks as a logical
// segment.  Both input and output buffers are hence maintained at block boundaries,
// with the input data completely processed for each tripleBlock.
//
// The pipeline must guarantee that the doSegment method is called with the 
// a continous buffer for the full segment (number of blocks).

void expand3_4Kernel::generateDoSegmentMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doSegmentFunction = m->getFunction(mKernelName + doSegment_suffix);
    BasicBlock * expand2_3entry = BasicBlock::Create(iBuilder->getContext(), "expand2_3entry", doSegmentFunction, 0);
    iBuilder->SetInsertPoint(expand2_3entry);
    BasicBlock * expand_3_4_loop = BasicBlock::Create(iBuilder->getContext(), "expand_3_4_loop", doSegmentFunction, 0);
    BasicBlock * expand3_4_loop_exit = BasicBlock::Create(iBuilder->getContext(), "expand3_4_loop_exit", doSegmentFunction, 0);
    BasicBlock * finalStep1 = BasicBlock::Create(iBuilder->getContext(), "finalStep1", doSegmentFunction, 0);
    BasicBlock * finalStep2 = BasicBlock::Create(iBuilder->getContext(), "finalStep2", doSegmentFunction, 0);
    BasicBlock * step2load = BasicBlock::Create(iBuilder->getContext(), "step2load", doSegmentFunction, 0);
    BasicBlock * step2store = BasicBlock::Create(iBuilder->getContext(), "step2store", doSegmentFunction, 0);
    BasicBlock * finalStep3 = BasicBlock::Create(iBuilder->getContext(), "finalStep3", doSegmentFunction, 0);
    BasicBlock * step3load = BasicBlock::Create(iBuilder->getContext(), "step3load", doSegmentFunction, 0);
    BasicBlock * step3store = BasicBlock::Create(iBuilder->getContext(), "step3store", doSegmentFunction, 0);
    BasicBlock * step3store2 = BasicBlock::Create(iBuilder->getContext(), "step3store2", doSegmentFunction, 0);
    BasicBlock * itemsDone = BasicBlock::Create(iBuilder->getContext(), "itemsDone", doSegmentFunction, 0);
    BasicBlock * setTermination = BasicBlock::Create(iBuilder->getContext(), "setTermination", doSegmentFunction, 0);
    BasicBlock * expand3_4_exit = BasicBlock::Create(iBuilder->getContext(), "expand3_4_exit", doSegmentFunction, 0);
    BasicBlock * finalExit = BasicBlock::Create(iBuilder->getContext(), "finalExit", doSegmentFunction, 0);
    
    // Determine the require shufflevector constants.
    const unsigned PACK_SIZE = iBuilder->getStride()/8;
    
    // Construct a list of indexes in  the form
    // 0, 1, 2, 2, 3, 4, 5, 5, 6, 7, 8, 8, ...
    unsigned sourceByteIndex = 0;
    unsigned expand3_4_index[PACK_SIZE];
    for (unsigned i = 0; i < PACK_SIZE; i++) {
        expand3_4_index[i] = sourceByteIndex;
        if (i % 4 != 2) sourceByteIndex++;
    }
    unsigned const expand3_4_offset[4] = {PACK_SIZE, 3*PACK_SIZE/4, PACK_SIZE/2, PACK_SIZE/4};
    Value * expand_3_4_shuffle[4];
    for (unsigned j = 0; j < 4; j++) {
        std::vector<Constant *> Idxs;
        for (unsigned i = 0; i < PACK_SIZE; i++) {
            Idxs.push_back(ConstantInt::get(iBuilder->getInt32Ty(), expand3_4_offset[j] + expand3_4_index[i]));
        }
        expand_3_4_shuffle[j] = ConstantVector::get(Idxs);
    }    
    Constant * Const3 = ConstantInt::get(iBuilder->getSizeTy(), 3);
    Constant * Const4 = ConstantInt::get(iBuilder->getSizeTy(), 4);
    Constant * tripleBlockSize = ConstantInt::get(iBuilder->getSizeTy(), iBuilder->getBitBlockWidth() * 3);
    Constant * stride = ConstantInt::get(iBuilder->getSizeTy(), iBuilder->getStride());
    Constant * packSize = ConstantInt::get(iBuilder->getSizeTy(), PACK_SIZE);
    Constant * loopItemCount = ConstantInt::get(iBuilder->getSizeTy(), 3 * PACK_SIZE); // 3 packs per loop.
    UndefValue * undefPack = UndefValue::get(iBuilder->fwVectorType(parabix::i8));
    
    const unsigned packAlign = iBuilder->getBitBlockWidth()/8;
    Function::arg_iterator args = doSegmentFunction->arg_begin();
    Value * self = &*(args++);
    Value * blocksToDo = &*(args);
    //iBuilder->CallPrintInt("blocksToDo", blocksToDo);
    Value * segmentNo = getLogicalSegmentNo(self);
    Value * streamStructPtr = getStreamSetStructPtr(self, "sourceStream");
    //iBuilder->CallPrintInt("streamStructPtr", iBuilder->CreatePtrToInt(streamStructPtr, iBuilder->getInt64Ty()));
    
    LoadInst * producerPos = iBuilder->CreateAtomicLoadAcquire(mStreamSetInputBuffers[0]->getProducerPosPtr(streamStructPtr));
    //iBuilder->CallPrintInt("producerPos", producerPos);
    Value * processed = getProcessedItemCount(self);
    Value * itemsAvail = iBuilder->CreateSub(producerPos, processed);
    
    // Except for the final segment, we always process an integral number of triple blocks.
    Value * tripleBlocksToDo = iBuilder->CreateUDiv(blocksToDo, ConstantInt::get(iBuilder->getSizeTy(), 3));
    Value * tripleBlocksAvail = iBuilder->CreateUDiv(itemsAvail, tripleBlockSize);
    Value * lessThanFullSegment = iBuilder->CreateICmpULT(tripleBlocksAvail, tripleBlocksToDo);
    Value * tripleBlockItems = iBuilder->CreateMul(iBuilder->CreateSelect(lessThanFullSegment, tripleBlocksAvail, tripleBlocksToDo), tripleBlockSize);
    Value * endSignal = iBuilder->CreateLoad(mStreamSetInputBuffers[0]->hasEndOfInputPtr(streamStructPtr));
    Value * inFinalSegment = iBuilder->CreateAnd(endSignal, lessThanFullSegment);
    Value * itemsToDo = iBuilder->CreateSelect(inFinalSegment, itemsAvail, tripleBlockItems);
    //iBuilder->CallPrintInt("itemsToDo", itemsToDo);

    Value * blockNo = getScalarField(self, blockNoScalar);
    Value * sourceBlockPtr = getStreamSetBlockPtr(self, "sourceStream", blockNo);
    
    Value * outputGenerated = getProducedItemCount(self); // bytes previously generated to output
    Value * outputBlockNo = iBuilder->CreateUDiv(outputGenerated, stride);
    Value * outputBlockPtr = getStreamSetBlockPtr(self, "expandedStream", outputBlockNo);
    
    // A block is made up of 8 packs.  Get the pointer to the first pack (changes the type of the pointer only).
    Value * sourcePackPtr = iBuilder->CreateGEP(sourceBlockPtr, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(0)});
    Value * outputPackPtr = iBuilder->CreateGEP(outputBlockPtr, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(0)});
    Value * hasFullLoop = iBuilder->CreateICmpUGE(itemsToDo, loopItemCount);
    iBuilder->CreateCondBr(hasFullLoop, expand_3_4_loop, expand3_4_loop_exit);
    iBuilder->SetInsertPoint(expand_3_4_loop);
    PHINode * loopInput_ptr = iBuilder->CreatePHI(sourcePackPtr->getType(), 2);
    PHINode * loopOutput_ptr = iBuilder->CreatePHI(outputPackPtr->getType(), 2);
    PHINode * loopItemsRemain = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
    loopInput_ptr->addIncoming(sourcePackPtr, expand2_3entry);
    loopOutput_ptr->addIncoming(outputPackPtr, expand2_3entry);
    loopItemsRemain->addIncoming(itemsToDo, expand2_3entry);
    // Step 1 of the main loop.
    Value * pack0 = iBuilder->fwCast(8, iBuilder->CreateAlignedLoad(loopInput_ptr, packAlign));
    Value * expand0 = iBuilder->bitCast(iBuilder->CreateShuffleVector(undefPack, pack0, expand_3_4_shuffle[0]));
    iBuilder->CreateAlignedStore(expand0, loopOutput_ptr, packAlign);
    // Step 2 of the main loop.
    Value * inPack1_ptr = iBuilder->CreateGEP(loopInput_ptr, {iBuilder->getInt32(1)});
    Value * outPack1_ptr = iBuilder->CreateGEP(loopOutput_ptr, {iBuilder->getInt32(1)});
    Value * pack1 = iBuilder->fwCast(8, iBuilder->CreateAlignedLoad(inPack1_ptr, packAlign));
    Value * expand1 = iBuilder->bitCast(iBuilder->CreateShuffleVector(pack0, pack1, expand_3_4_shuffle[1]));
    iBuilder->CreateAlignedStore(expand1, outPack1_ptr, packAlign);
    // Step 3 of the main loop.
    Value * inPack2_ptr = iBuilder->CreateGEP(loopInput_ptr, {iBuilder->getInt32(2)});
    Value * outPack2_ptr = iBuilder->CreateGEP(loopOutput_ptr, {iBuilder->getInt32(2)});
    Value * pack2 = iBuilder->fwCast(8, iBuilder->CreateAlignedLoad(inPack2_ptr, packAlign));
    Value * expand2 = iBuilder->bitCast(iBuilder->CreateShuffleVector(pack1, pack2, expand_3_4_shuffle[2]));
    iBuilder->CreateAlignedStore(expand2, outPack2_ptr, packAlign);
    Value * outPack3_ptr = iBuilder->CreateGEP(loopOutput_ptr, {iBuilder->getInt32(3)});
    Value * expand3 = iBuilder->bitCast(iBuilder->CreateShuffleVector(pack2, undefPack, expand_3_4_shuffle[3]));
    iBuilder->CreateAlignedStore(expand3, outPack3_ptr, packAlign);
    
    Value * loopNextInputPack = iBuilder->CreateGEP(loopInput_ptr, {iBuilder->getInt32(3)});
    Value * loopNextOutputPack = iBuilder->CreateGEP(loopOutput_ptr, {iBuilder->getInt32(4)});
    Value * remainingItems = iBuilder->CreateSub(loopItemsRemain, loopItemCount);
    loopInput_ptr->addIncoming(loopNextInputPack, expand_3_4_loop);
    loopOutput_ptr->addIncoming(loopNextOutputPack, expand_3_4_loop);
    loopItemsRemain->addIncoming(remainingItems, expand_3_4_loop);
    //iBuilder->CallPrintInt("loopItemsRemain", remainingItems);
    Value * continueLoop = iBuilder->CreateICmpUGE(remainingItems, loopItemCount);
    iBuilder->CreateCondBr(continueLoop, expand_3_4_loop, expand3_4_loop_exit);

    // Except for the final segment, the number of items remaining is now 0.
    // For the final segment, less than loopItemCount items remain.
    iBuilder->SetInsertPoint(expand3_4_loop_exit);
    PHINode * loopExitInput_ptr = iBuilder->CreatePHI(sourcePackPtr->getType(), 2);
    PHINode * loopExitOutput_ptr = iBuilder->CreatePHI(outputPackPtr->getType(), 2);
    PHINode * loopExitItemsRemain = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
    loopExitInput_ptr->addIncoming(sourcePackPtr, expand2_3entry);
    loopExitOutput_ptr->addIncoming(outputPackPtr, expand2_3entry);
    loopExitItemsRemain->addIncoming(itemsToDo, expand2_3entry);
    loopExitInput_ptr->addIncoming(loopNextInputPack, expand_3_4_loop);
    loopExitOutput_ptr->addIncoming(loopNextOutputPack, expand_3_4_loop);
    loopExitItemsRemain->addIncoming(remainingItems, expand_3_4_loop);

    // There may be one or two remaining full packs and/or a partial pack.  
    //
    // We have several cases depending on the number of reumaing items.  Let N = packSize
    // (a) 0 remaining items: all done
    // (b) 1..3N/4 remaining items:  do Step1 only, no items or pending data will remain
    // (c) 3N/4+1 .. N remaining items:  do Step 1, do Step 2 for pending data from Step 1 only, there is no more input.
    // (d) N+1 .. 6N/4 remaining items:  do Step 1 and Step 2, no items or pending data will remain.
    // (e) 6N/4+1 .. 2N remaining items: do Steps 1 and 2, do Step 3 for pending data only, there is no more input.
    // (f) 2N+1 .. 9N/4 remaining items: do Steps 1 and 2, do Step 3 up to the first write only.
    // (g) 9N/4+1 .. 3N - 1 remaining items: do Steps 1, 2 and 3.
    Value * condition_a = iBuilder->CreateICmpEQ(loopExitItemsRemain, ConstantInt::getNullValue(iBuilder->getSizeTy()));
    iBuilder->CreateCondBr(condition_a, itemsDone, finalStep1);
    // Final Step1 processing
    iBuilder->SetInsertPoint(finalStep1);
    pack0 = iBuilder->fwCast(8, iBuilder->CreateAlignedLoad(loopExitInput_ptr, packAlign));
    expand0 = iBuilder->bitCast(iBuilder->CreateShuffleVector(undefPack, pack0, expand_3_4_shuffle[0]));
    iBuilder->CreateAlignedStore(expand0, loopExitOutput_ptr, packAlign);
    Value * condition_b = iBuilder->CreateICmpULE(loopExitItemsRemain, ConstantInt::get(iBuilder->getSizeTy(), 3 * PACK_SIZE/4));
    iBuilder->CreateCondBr(condition_b, itemsDone, finalStep2);
    // Final Step 2 processing
    iBuilder->SetInsertPoint(finalStep2);
    Value * condition_c = iBuilder->CreateICmpULE(loopExitItemsRemain, packSize);
    iBuilder->CreateCondBr(condition_c, step2store, step2load);
    iBuilder->SetInsertPoint(step2load);
    inPack1_ptr = iBuilder->CreateGEP(loopExitInput_ptr, {iBuilder->getInt32(1)});
    pack1 = iBuilder->fwCast(8, iBuilder->CreateAlignedLoad(inPack1_ptr, packAlign));
    iBuilder->CreateBr(step2store);
    iBuilder->SetInsertPoint(step2store);
    PHINode * pack1phi = iBuilder->CreatePHI(iBuilder->fwVectorType(8), 2);
    pack1phi->addIncoming(undefPack, finalStep2);
    pack1phi->addIncoming(pack1, step2load);
    outPack1_ptr = iBuilder->CreateGEP(loopExitOutput_ptr, {iBuilder->getInt32(1)});
    expand1 = iBuilder->bitCast(iBuilder->CreateShuffleVector(pack0, pack1phi, expand_3_4_shuffle[1]));
    iBuilder->CreateAlignedStore(expand1, outPack1_ptr, packAlign);
    Value * condition_d = iBuilder->CreateICmpULE(loopExitItemsRemain, ConstantInt::get(iBuilder->getSizeTy(), 6 * PACK_SIZE/4));
    iBuilder->CreateCondBr(condition_d, itemsDone, finalStep3);
    // Final Step 3
    iBuilder->SetInsertPoint(finalStep3);
    Value * condition_e = iBuilder->CreateICmpULE(loopExitItemsRemain, ConstantInt::get(iBuilder->getSizeTy(), 2 * PACK_SIZE));
    iBuilder->CreateCondBr(condition_e, step3store, step3load);
    iBuilder->SetInsertPoint(step3load);
    inPack2_ptr = iBuilder->CreateGEP(loopExitInput_ptr, {iBuilder->getInt32(2)});
    pack2 = iBuilder->fwCast(8, iBuilder->CreateAlignedLoad(inPack2_ptr, packAlign));
    iBuilder->CreateBr(step3store);
    iBuilder->SetInsertPoint(step3store);
    PHINode * pack2phi = iBuilder->CreatePHI(iBuilder->fwVectorType(8), 2);
    pack2phi->addIncoming(undefPack, finalStep3);
    pack2phi->addIncoming(pack2, step3load);
    outPack2_ptr = iBuilder->CreateGEP(loopExitOutput_ptr, {iBuilder->getInt32(2)});
    expand2 = iBuilder->bitCast(iBuilder->CreateShuffleVector(pack1phi, pack2phi, expand_3_4_shuffle[2]));
    iBuilder->CreateAlignedStore(expand2, outPack2_ptr, packAlign);
    Value * condition_f = iBuilder->CreateICmpULE(loopExitItemsRemain, ConstantInt::get(iBuilder->getSizeTy(), 9 * PACK_SIZE/4));
    iBuilder->CreateCondBr(condition_f, itemsDone, step3store2);
    iBuilder->SetInsertPoint(step3store2);
    outPack3_ptr = iBuilder->CreateGEP(loopExitOutput_ptr, {iBuilder->getInt32(3)});
    expand3 = iBuilder->bitCast(iBuilder->CreateShuffleVector(pack2phi, undefPack, expand_3_4_shuffle[3]));
    iBuilder->CreateAlignedStore(expand3, outPack3_ptr, packAlign);
    iBuilder->CreateBr(itemsDone);
    //
    iBuilder->SetInsertPoint(itemsDone);
    
    processed = iBuilder->CreateAdd(processed, itemsToDo);
    setProcessedItemCount(self, processed);
    setScalarField(self, blockNoScalar, iBuilder->CreateUDiv(processed, stride));
    // We have produced 4 output bytes for every 3 input bytes.  If the number of input
    // bytes is not a multiple of 3, then we have one more output byte for each excess
    // input byte.
    Value * totalProduced = iBuilder->CreateAdd(iBuilder->CreateMul(iBuilder->CreateUDiv(processed, Const3), Const4), iBuilder->CreateURem(processed, Const3));
    setProducedItemCount(self, totalProduced);
    Value * ssStructPtr = getStreamSetStructPtr(self, "expandedStream");
    
    Value * producerPosPtr = mStreamSetOutputBuffers[0]->getProducerPosPtr(ssStructPtr);
    iBuilder->CreateAtomicStoreRelease(totalProduced, producerPosPtr);
    
    iBuilder->CreateCondBr(inFinalSegment, setTermination, expand3_4_exit);
    iBuilder->SetInsertPoint(setTermination);
#ifndef NDEBUG
    iBuilder->CallPrintInt(mKernelName + " termination in segment ", segmentNo);
#endif
    setTerminationSignal(self);
    mStreamSetOutputBuffers[0]->setEndOfInput(ssStructPtr);
    iBuilder->CreateBr(expand3_4_exit);
    iBuilder->SetInsertPoint(expand3_4_exit);
    // Must be the last action, for synchronization.
    setLogicalSegmentNo(self, iBuilder->CreateAdd(segmentNo, ConstantInt::get(iBuilder->getSizeTy(), 1)));
    iBuilder->CreateBr(finalExit);
    
    iBuilder->SetInsertPoint(finalExit);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}


// The doBlock method is deprecated.   But in case it is used, just call doSegment with
// 1 as the number of blocks to do.
void expand3_4Kernel::generateDoBlockMethod() {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    Function * doSegmentFunction = m->getFunction(mKernelName + doSegment_suffix);
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));
    Value * self = getParameter(doBlockFunction, "self");
    iBuilder->CreateCall(doSegmentFunction, {self, ConstantInt::get(iBuilder->getSizeTy(), 1)});
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

}