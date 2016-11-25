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

    
// Radix 64 determination, converting 3 bytes to 4 6-bit values.
//
//  00000000|zyxwvuts|rqpmnlkj|hgfedcba    Original 3 bytes of binary data in a 32-bit field
//                        nlkj|hgfedcba    bits to move 0 positions initially
//           zyxwvuts|rqpm    |            bits to move 4 positions
//      zyxw|vutsrqpm|        |            shift forward 4
//      zyxw|vutsrqpm|    nlkj|hgfedcba    combine with bits moving 0
//          |  tsrqpm|        |  fedcba    bits to move 0 positions in second step
//      zyxw|vu      |    nlkj|hg          bits to move 2 positions in second stap
//    zyxwvu|        |  nlkjhg|            shift forward 2
//    zyxwvu|  tsrqpm|  nlkjhg|  fedcba    The 4 radix64 values have been computed.

void radix64Kernel::generateDoBlockLogic(Value * self, Value * blockNo) {
    Value * expandedStream = getStreamSetBlockPtr(self, "expandedStream", blockNo);
    Value * radix64stream = getStreamSetBlockPtr(self, "radix64stream", blockNo);
    Value * step1_bits_to_move4 = iBuilder->simd_fill(32, ConstantInt::get(iBuilder->getInt32Ty(), 0x0003FFC0)); 
    Value * step1_bits_to_stay = iBuilder->simd_fill(32, ConstantInt::get(iBuilder->getInt32Ty(), 0x00000FFF)); 
    Value * step2_bits_to_move2 = iBuilder->simd_fill(32, ConstantInt::get(iBuilder->getInt32Ty(), 0x006F006F)); 
    Value * step2_bits_to_stay = iBuilder->simd_fill(32, ConstantInt::get(iBuilder->getInt32Ty(), 0x0FC00FC0));
    
    for (unsigned i = 0; i < 8; i++) {
        Value * bytepack = iBuilder->CreateBlockAlignedLoad(expandedStream, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(i)});
        Value * move4 = iBuilder->simd_slli(32, iBuilder->simd_and(bytepack, step1_bits_to_move4), 4);
        Value * step1 = iBuilder->simd_or(move4, iBuilder->simd_and(bytepack, step1_bits_to_stay));
        Value * move2 = iBuilder->simd_slli(32, iBuilder->simd_and(step1, step2_bits_to_move2), 2);
        Value * radix64pack = iBuilder->bitCast(iBuilder->simd_or(move2, iBuilder->simd_and(step1, step2_bits_to_stay)));
        iBuilder->CreateBlockAlignedStore(radix64pack, radix64stream, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(i)});
    }
    Value * produced = getProducedItemCount(self);
    produced = iBuilder->CreateAdd(produced, ConstantInt::get(iBuilder->getSizeTy(), iBuilder->getStride()));
    setProducedItemCount(self, produced);    
}


void base64Kernel::generateDoBlockLogic(Value * self, Value * blockNo) {
    Value * radix64stream_ptr = getStreamSetBlockPtr(self, "radix64stream", blockNo);
    Value * base64stream_ptr = getStreamSetBlockPtr(self, "base64stream", blockNo);
    Type * i8_t = iBuilder->getInt8Ty();
    
    for (unsigned i = 0; i < 8; i++) {
        Value * bytepack = iBuilder->CreateBlockAlignedLoad(radix64stream_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(i)});
        Value * mask_gt_25 = iBuilder->simd_ugt(8, bytepack, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 25)));
        Value * mask_gt_51 = iBuilder->simd_ugt(8, bytepack, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 51)));
        Value * mask_eq_62 = iBuilder->simd_eq(8, bytepack, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 62)));
        Value * mask_eq_63 = iBuilder->simd_eq(8, bytepack, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 63)));
        // Strategy:
        // 1. add ord('A') = 65 to all radix64 values, this sets the correct values for entries 0 to 25.
        // 2. add ord('a') - ord('A') = 32 to all values >25, this sets the correct values for entries 0 to 51
        // 3. subtract ord('a') - ord('0') = 49 to all values > 51, this sets the correct values for entries 0 to 61
        // 4. subtract ord('0') - ord('+') = 5 for all values = 62
        // 4. subtract ord('0') - ord('/') = 1 for all values = 63
        Value * t0_25 = iBuilder->simd_add(8, bytepack, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 'A')));
        Value * t0_51 = iBuilder->simd_add(8, t0_25, iBuilder->simd_and(mask_gt_25, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 32))));
        Value * t0_61 = iBuilder->simd_sub(8, t0_51, iBuilder->simd_and(mask_gt_51, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 49))));
        Value * t0_62 = iBuilder->simd_sub(8, t0_61, iBuilder->simd_and(mask_eq_62, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 5))));
        Value * base64pack = iBuilder->simd_sub(8, t0_62, iBuilder->simd_and(mask_eq_63, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 1))));
        iBuilder->CreateBlockAlignedStore(base64pack, base64stream_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(i)});
    }
    Value * produced = getProducedItemCount(self);
    produced = iBuilder->CreateAdd(produced, ConstantInt::get(iBuilder->getSizeTy(), iBuilder->getStride()));
    setProducedItemCount(self, produced);    
}


// Special processing for the base 64 format.   The output must always contain a multiple
// of 4 bytes.   When the number of radix 64 values is not a multiple of 4
// number of radix 64 values
void base64Kernel::generateFinalBlockMethod() {
    auto savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * finalBlockFunction = m->getFunction(mKernelName + finalBlock_suffix);
    BasicBlock * base64_fb_entry = BasicBlock::Create(iBuilder->getContext(), "base64_fb_entry", finalBlockFunction, 0);
    iBuilder->SetInsertPoint(base64_fb_entry);
    BasicBlock * base64_loop = BasicBlock::Create(iBuilder->getContext(), "base64_loop", finalBlockFunction, 0);
    BasicBlock * loopExit = BasicBlock::Create(iBuilder->getContext(), "loopExit", finalBlockFunction, 0);
    BasicBlock * doPadding = BasicBlock::Create(iBuilder->getContext(), "doPadding", finalBlockFunction, 0);
    BasicBlock * doPadding2 = BasicBlock::Create(iBuilder->getContext(), "doPadding2", finalBlockFunction, 0);
    BasicBlock * fbExit = BasicBlock::Create(iBuilder->getContext(), "fbExit", finalBlockFunction, 0);
    // Final Block arguments: self, remaining.
    Function::arg_iterator args = finalBlockFunction->arg_begin();
    Value * self = &*(args++);
    Value * remainingBytes = &*(args++);
    Value * remainMod4 = iBuilder->CreateAnd(remainingBytes, ConstantInt::get(iBuilder->getSizeTy(), 3));
    Value * padBytes = iBuilder->CreateSub(ConstantInt::get(iBuilder->getSizeTy(), 4), remainMod4);
    padBytes = iBuilder->CreateAnd(padBytes, ConstantInt::get(iBuilder->getSizeTy(), 3));

    const unsigned PACK_SIZE = iBuilder->getStride()/8;
    Constant * packSize = ConstantInt::get(iBuilder->getSizeTy(), PACK_SIZE);
    Value * blockNo = getScalarField(self, blockNoScalar);
    Value * radix64stream_ptr = getStreamSetBlockPtr(self, "radix64stream", blockNo);
    Value * base64stream_ptr = getStreamSetBlockPtr(self, "base64stream", blockNo);
    Type * i8_t = iBuilder->getInt8Ty();
    
    // Enter the loop only if there is at least one byte remaining to process.
    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(remainingBytes, ConstantInt::get(iBuilder->getSizeTy(), 0)), fbExit, base64_loop);
    
    iBuilder->SetInsertPoint(base64_loop);
    PHINode * idx = iBuilder->CreatePHI(iBuilder->getInt32Ty(), 2);
    PHINode * loopRemain = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
    idx->addIncoming(ConstantInt::getNullValue(iBuilder->getInt32Ty()), base64_fb_entry);
    loopRemain->addIncoming(remainingBytes, base64_fb_entry);
    Value * bytepack = iBuilder->CreateBlockAlignedLoad(radix64stream_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(0), idx});
    Value * mask_gt_25 = iBuilder->simd_ugt(8, bytepack, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 25)));
    Value * mask_gt_51 = iBuilder->simd_ugt(8, bytepack, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 51)));
    Value * mask_eq_62 = iBuilder->simd_eq(8, bytepack, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 62)));
    Value * mask_eq_63 = iBuilder->simd_eq(8, bytepack, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 63)));
    Value * t0_25 = iBuilder->simd_add(8, bytepack, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 'A')));
    Value * t0_51 = iBuilder->simd_add(8, t0_25, iBuilder->simd_and(mask_gt_25, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 32))));
    Value * t0_61 = iBuilder->simd_sub(8, t0_51, iBuilder->simd_and(mask_gt_51, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 49))));
    Value * t0_62 = iBuilder->simd_sub(8, t0_61, iBuilder->simd_and(mask_eq_62, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 5))));
    Value * base64pack = iBuilder->simd_sub(8, t0_62, iBuilder->simd_and(mask_eq_63, iBuilder->simd_fill(8, ConstantInt::get(i8_t, 1))));
    iBuilder->CreateBlockAlignedStore(base64pack, base64stream_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(0), idx});
    idx->addIncoming(iBuilder->CreateAdd(idx, ConstantInt::get(iBuilder->getInt32Ty(), 1)), base64_loop);
    loopRemain->addIncoming(iBuilder->CreateSub(loopRemain, packSize), base64_loop);
    iBuilder->SetInsertPoint(loopExit);
    // All base64 data has been computed, but we may need to set one or two '=' padding bytes.
    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(padBytes, ConstantInt::get(iBuilder->getSizeTy(), 0)), fbExit, doPadding);
    iBuilder->SetInsertPoint(doPadding);
    // At least one padding byte required.
    Value * i8output_ptr = iBuilder->CreatePointerCast(base64stream_ptr, iBuilder->getInt8PtrTy());
    iBuilder->CreateStore(iBuilder->CreateGEP(i8output_ptr, {remainingBytes}), ConstantInt::get(iBuilder->getInt8Ty(), '='));
    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(remainMod4, ConstantInt::get(iBuilder->getSizeTy(), 3)), fbExit, doPadding2);
    iBuilder->SetInsertPoint(doPadding2);
    // One more padding byte required.
    Value * finalPadPos = iBuilder->CreateAdd(remainingBytes, ConstantInt::get(iBuilder->getSizeTy(), 1));
    iBuilder->CreateStore(iBuilder->CreateGEP(i8output_ptr, {finalPadPos}), ConstantInt::get(iBuilder->getInt8Ty(), '='));
    iBuilder->CreateBr(fbExit);
    iBuilder->SetInsertPoint(fbExit);
    Value * produced = iBuilder->CreateAdd(getProducedItemCount(self), iBuilder->CreateAdd(remainingBytes, padBytes));
    setProducedItemCount(self, produced);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}
}
