/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "radix64.h"
#include <kernels/streamset.h>
#include <IR_Gen/idisa_builder.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace kernel {

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

void expand3_4Kernel::generateDoSegmentMethod(Value *doFinal, const std::vector<Value *> &producerPos) {

    BasicBlock * expand2_3entry = iBuilder->GetInsertBlock();
    BasicBlock * expand_3_4_loop = CreateBasicBlock("expand_3_4_loop");
    BasicBlock * expand3_4_loop_exit = CreateBasicBlock("expand3_4_loop_exit");
    BasicBlock * finalStep1 = CreateBasicBlock("finalStep1");
    BasicBlock * finalStep2 = CreateBasicBlock("finalStep2");
    BasicBlock * step2load = CreateBasicBlock("step2load");
    BasicBlock * step2store = CreateBasicBlock("step2store");
    BasicBlock * finalStep3 = CreateBasicBlock("finalStep3");
    BasicBlock * step3load = CreateBasicBlock("step3load");
    BasicBlock * step3store = CreateBasicBlock("step3store");
    BasicBlock * step3store2 = CreateBasicBlock("step3store2");
    BasicBlock * itemsDone = CreateBasicBlock("itemsDone");
    BasicBlock * expand3_4_final = CreateBasicBlock("expand3_4_final");
    BasicBlock * expand3_4_exit = CreateBasicBlock("expand3_4_exit");
    
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
    Constant * Const3 = iBuilder->getSize(3);
    Constant * Const4 = iBuilder->getSize(4);
    Constant * Const6 = iBuilder->getSize(6);
    Constant * Const5 = iBuilder->getSize(5);

    Constant * tripleBlockSize = iBuilder->getSize(3 * iBuilder->getStride());
    Constant * packSize = iBuilder->getSize(PACK_SIZE);
    Constant * triplePackSize = iBuilder->getSize(3 * PACK_SIZE); // 3 packs per loop.
    UndefValue * undefPack = UndefValue::get(iBuilder->fwVectorType(8));
    
    const unsigned packAlign = iBuilder->getBitBlockWidth()/8;

    Value * processed = getProcessedItemCount("sourceStream");
    Value * itemsAvail = iBuilder->CreateSub(producerPos[0], processed);
    
    //
    // The main loop processes 3 packs of data at a time.  For doFinal
    // processing, process all the remaining sets of 3 packs, otherwise
    // process in multiples of 3 full blocks of data.
    //
    Value * loopDivisor = iBuilder->CreateSelect(doFinal, triplePackSize, tripleBlockSize);
    Value * excessItems = iBuilder->CreateURem(itemsAvail, loopDivisor);
    Value * loopItemsToDo = iBuilder->CreateSub(itemsAvail, excessItems);

    // A block is made up of 8 packs.  Get the pointer to the first pack (changes the type of the pointer only).
    Value * sourcePackPtr = getInputStreamPackPtr("sourceStream", iBuilder->getInt32(0), iBuilder->getInt32(0));
    Value * outputPackPtr = getOutputStreamPackPtr("expandedStream", iBuilder->getInt32(0), iBuilder->getInt32(0));

    Value * hasFullLoop = iBuilder->CreateICmpUGE(loopItemsToDo, triplePackSize);

    iBuilder->CreateCondBr(hasFullLoop, expand_3_4_loop, expand3_4_loop_exit);
    iBuilder->SetInsertPoint(expand_3_4_loop);
    PHINode * loopInput_ptr = iBuilder->CreatePHI(sourcePackPtr->getType(), 2);
    PHINode * loopOutput_ptr = iBuilder->CreatePHI(outputPackPtr->getType(), 2);
    PHINode * loopItemsRemain = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);

    loopInput_ptr->addIncoming(sourcePackPtr, expand2_3entry);
    loopOutput_ptr->addIncoming(outputPackPtr, expand2_3entry);
    loopItemsRemain->addIncoming(loopItemsToDo, expand2_3entry);

    // Step 1 of the main loop.
    Value * pack0 = iBuilder->fwCast(8, iBuilder->CreateAlignedLoad(loopInput_ptr, packAlign));
    Value * expand0 = iBuilder->bitCast(iBuilder->CreateShuffleVector(undefPack, pack0, expand_3_4_shuffle[0]));
    iBuilder->CreateBlockAlignedStore(expand0, loopOutput_ptr);
    // Step 2 of the main loop.
    Value * inPack1_ptr = iBuilder->CreateGEP(loopInput_ptr, iBuilder->getInt32(1));
    Value * outPack1_ptr = iBuilder->CreateGEP(loopOutput_ptr, iBuilder->getInt32(1));
    Value * pack1 = iBuilder->fwCast(8, iBuilder->CreateAlignedLoad(inPack1_ptr, packAlign));
    Value * expand1 = iBuilder->bitCast(iBuilder->CreateShuffleVector(pack0, pack1, expand_3_4_shuffle[1]));
    iBuilder->CreateBlockAlignedStore(expand1, outPack1_ptr);
    // Step 3 of the main loop.
    Value * inPack2_ptr = iBuilder->CreateGEP(loopInput_ptr, iBuilder->getInt32(2));
    Value * outPack2_ptr = iBuilder->CreateGEP(loopOutput_ptr, iBuilder->getInt32(2));
    Value * pack2 = iBuilder->fwCast(8, iBuilder->CreateAlignedLoad(inPack2_ptr, packAlign));
    Value * expand2 = iBuilder->bitCast(iBuilder->CreateShuffleVector(pack1, pack2, expand_3_4_shuffle[2]));
    iBuilder->CreateBlockAlignedStore(expand2, outPack2_ptr);
    Value * outPack3_ptr = iBuilder->CreateGEP(loopOutput_ptr, iBuilder->getInt32(3));
    Value * expand3 = iBuilder->bitCast(iBuilder->CreateShuffleVector(pack2, undefPack, expand_3_4_shuffle[3]));
    iBuilder->CreateBlockAlignedStore(expand3, outPack3_ptr);

    Value * loopNextInputPack = iBuilder->CreateGEP(loopInput_ptr, iBuilder->getInt32(3));
    Value * remainingItems = iBuilder->CreateSub(loopItemsRemain, triplePackSize);

    Value * loopNextOutputPack;
    loopNextOutputPack = iBuilder->CreateGEP(loopOutput_ptr, iBuilder->getInt32(4));

    loopInput_ptr->addIncoming(loopNextInputPack, expand_3_4_loop);
    loopOutput_ptr->addIncoming(loopNextOutputPack, expand_3_4_loop);
    loopItemsRemain->addIncoming(remainingItems, expand_3_4_loop);

    Value * continueLoop = iBuilder->CreateICmpUGE(remainingItems, triplePackSize);
    iBuilder->CreateCondBr(continueLoop, expand_3_4_loop, expand3_4_loop_exit);
    
    iBuilder->SetInsertPoint(expand3_4_loop_exit);
    PHINode * loopExitInput_ptr = iBuilder->CreatePHI(sourcePackPtr->getType(), 2);
    PHINode * loopExitOutput_ptr = iBuilder->CreatePHI(outputPackPtr->getType(), 2);
    loopExitInput_ptr->addIncoming(sourcePackPtr, expand2_3entry);
    loopExitOutput_ptr->addIncoming(outputPackPtr, expand2_3entry);
    loopExitInput_ptr->addIncoming(loopNextInputPack, expand_3_4_loop);
    loopExitOutput_ptr->addIncoming(loopNextOutputPack, expand_3_4_loop);

    // Update the produced and processed items count based on the loopItemsToDo value.
    processed = iBuilder->CreateAdd(processed, loopItemsToDo);
    setProcessedItemCount("sourceStream", processed);

    // We have produced 4 output bytes for every 3 input bytes.
    Value * totalProduced = iBuilder->CreateMul(iBuilder->CreateUDiv(processed, Const3), Const4);
    setProducedItemCount("expandedStream", totalProduced);

    // Except for final segment processing, we are done.
    iBuilder->CreateCondBr(doFinal, expand3_4_final, expand3_4_exit);

    // Final segment processing.   Less than a triplePack remains.
    iBuilder->SetInsertPoint(expand3_4_final);
    
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
    Value * condition_a = iBuilder->CreateICmpEQ(excessItems, ConstantInt::getNullValue(iBuilder->getSizeTy()));
    iBuilder->CreateCondBr(condition_a, itemsDone, finalStep1);
    // Final Step1 processing
    iBuilder->SetInsertPoint(finalStep1);
    pack0 = iBuilder->fwCast(8, iBuilder->CreateAlignedLoad(loopExitInput_ptr, packAlign));
    expand0 = iBuilder->bitCast(iBuilder->CreateShuffleVector(undefPack, pack0, expand_3_4_shuffle[0]));
    iBuilder->CreateAlignedStore(expand0, loopExitOutput_ptr, packAlign);
    Value * condition_b = iBuilder->CreateICmpULE(excessItems, iBuilder->getSize(3 * PACK_SIZE/4));
    iBuilder->CreateCondBr(condition_b, itemsDone, finalStep2);
    // Final Step 2 processing
    iBuilder->SetInsertPoint(finalStep2);
    Value * condition_c = iBuilder->CreateICmpULE(excessItems, packSize);
    iBuilder->CreateCondBr(condition_c, step2store, step2load);
    iBuilder->SetInsertPoint(step2load);
    inPack1_ptr = iBuilder->CreateGEP(loopExitInput_ptr, iBuilder->getInt32(1));
    pack1 = iBuilder->fwCast(8, iBuilder->CreateAlignedLoad(inPack1_ptr, packAlign));
    iBuilder->CreateBr(step2store);
    iBuilder->SetInsertPoint(step2store);
    PHINode * pack1phi = iBuilder->CreatePHI(iBuilder->fwVectorType(8), 2);
    pack1phi->addIncoming(undefPack, finalStep2);
    pack1phi->addIncoming(pack1, step2load);
    outPack1_ptr = iBuilder->CreateGEP(loopExitOutput_ptr, iBuilder->getInt32(1));
    expand1 = iBuilder->bitCast(iBuilder->CreateShuffleVector(pack0, pack1phi, expand_3_4_shuffle[1]));
    iBuilder->CreateAlignedStore(expand1, outPack1_ptr, packAlign);
    Value * condition_d = iBuilder->CreateICmpULE(excessItems, iBuilder->getSize(6 * PACK_SIZE/4));
    iBuilder->CreateCondBr(condition_d, itemsDone, finalStep3);
    // Final Step 3
    iBuilder->SetInsertPoint(finalStep3);
    Value * condition_e = iBuilder->CreateICmpULE(excessItems, iBuilder->getSize(2 * PACK_SIZE));
    iBuilder->CreateCondBr(condition_e, step3store, step3load);
    iBuilder->SetInsertPoint(step3load);
    inPack2_ptr = iBuilder->CreateGEP(loopExitInput_ptr, iBuilder->getInt32(2));
    pack2 = iBuilder->fwCast(8, iBuilder->CreateAlignedLoad(inPack2_ptr, packAlign));
    iBuilder->CreateBr(step3store);
    iBuilder->SetInsertPoint(step3store);
    PHINode * pack2phi = iBuilder->CreatePHI(iBuilder->fwVectorType(8), 2);
    pack2phi->addIncoming(undefPack, finalStep3);
    pack2phi->addIncoming(pack2, step3load);
    outPack2_ptr = iBuilder->CreateGEP(loopExitOutput_ptr, iBuilder->getInt32(2));
    expand2 = iBuilder->bitCast(iBuilder->CreateShuffleVector(pack1phi, pack2phi, expand_3_4_shuffle[2]));
    iBuilder->CreateAlignedStore(expand2, outPack2_ptr, packAlign);
    Value * condition_f = iBuilder->CreateICmpULE(excessItems, iBuilder->getSize(9 * PACK_SIZE/4));
    iBuilder->CreateCondBr(condition_f, itemsDone, step3store2);
    iBuilder->SetInsertPoint(step3store2);
    outPack3_ptr = iBuilder->CreateGEP(loopExitOutput_ptr, iBuilder->getInt32(3));
    expand3 = iBuilder->bitCast(iBuilder->CreateShuffleVector(pack2phi, undefPack, expand_3_4_shuffle[3]));
    iBuilder->CreateAlignedStore(expand3, outPack3_ptr, packAlign);
    iBuilder->CreateBr(itemsDone);
    //
    iBuilder->SetInsertPoint(itemsDone);
    processed = iBuilder->CreateAdd(processed, excessItems);
    setProcessedItemCount("sourceStream", processed);

    // We have produced 4 output bytes for every 3 input bytes.  For radix64 applications,
    // each output byte will have six significant bits.   If the number of bytes is not
    // a multiple of 3, then there will be 8 or 16 additional bits to encode.  This will
    // require 2 or 3 additional output bytes given that each output byte only encodes 6 bits.
    
    Value * totalBits = iBuilder->CreateShl(processed, Const3);
    totalProduced = iBuilder->CreateUDiv(iBuilder->CreateAdd(totalBits, Const5), Const6);
    setProducedItemCount("expandedStream", totalProduced);
    
    iBuilder->CreateBr(expand3_4_exit);
    iBuilder->SetInsertPoint(expand3_4_exit);
}


// Radix 64 determination, converting 3 bytes to 4 6-bit values.
//
//  00000000|zyxwvuts|rqpmnlkj|hgfedcba    Original
//           zy                            bits to move 6 positions right
//             xwvuts                      bits to move 8 positions left
//                    rqpm                 bits to move 4 positions right
//                        nlkj             bits to move 10 positions left
//                             hqfedc      bits to move 2 positions right
//                                   ba    bits to move 12 positions left
//    xwvuts|  nlkjzy|  barqpm|  hgfedc    Target
inline Value * radix64Kernel::processPackData(llvm::Value * bytepack) const {

    Value * step_right_6 = iBuilder->simd_fill(32, ConstantInt::get(iBuilder->getInt32Ty(), 0x00C00000));
    Value * right_6_result = iBuilder->simd_srli(32, iBuilder->simd_and(bytepack, step_right_6), 6);

    Value * step_left_8 = iBuilder->simd_fill(32, ConstantInt::get(iBuilder->getInt32Ty(), 0x003F0000));
    Value * left_8_result = iBuilder->simd_slli(32, iBuilder->simd_and(bytepack, step_left_8), 8);
    Value * mid = iBuilder->simd_or(right_6_result, left_8_result);

    Value * step_right_4 = iBuilder->simd_fill(32, ConstantInt::get(iBuilder->getInt32Ty(), 0x0000F000));
    Value * right_4_result = iBuilder->simd_srli(32, iBuilder->simd_and(bytepack, step_right_4), 4);
    mid = iBuilder->simd_or(mid, right_4_result);

    Value * step_left_10 = iBuilder->simd_fill(32, ConstantInt::get(iBuilder->getInt32Ty(), 0x00000F00));
    Value * left_10_result = iBuilder->simd_slli(32, iBuilder->simd_and(bytepack, step_left_10), 10);
    mid = iBuilder->simd_or(mid, left_10_result);

    Value * step_right_2 = iBuilder->simd_fill(32, ConstantInt::get(iBuilder->getInt32Ty(), 0x000000FC));
    Value * right_2_result = iBuilder->simd_srli(32, iBuilder->simd_and(bytepack, step_right_2), 2);
    mid = iBuilder->simd_or(mid, right_2_result);

    Value * step_left_12 = iBuilder->simd_fill(32, ConstantInt::get(iBuilder->getInt32Ty(), 0x00000003));
    Value * left_12_result = iBuilder->simd_slli(32, iBuilder->simd_and(bytepack, step_left_12), 12);
    mid = iBuilder->simd_or(mid, left_12_result);

    return iBuilder->bitCast(mid);
}

void radix64Kernel::generateDoBlockMethod() {
    for (unsigned i = 0; i < 8; i++) {
        Value * bytepack = loadInputStreamPack("expandedStream", iBuilder->getInt32(0), iBuilder->getInt32(i));
        Value * radix64pack = processPackData(bytepack);
        storeOutputStreamPack("radix64stream", iBuilder->getInt32(0), iBuilder->getInt32(i), radix64pack);
    }
}

void radix64Kernel::generateFinalBlockMethod(Value * remainingBytes) {

    BasicBlock * entry = iBuilder->GetInsertBlock();
    BasicBlock * radix64_loop = CreateBasicBlock("radix64_loop");
    BasicBlock * fbExit = CreateBasicBlock("fbExit");
    
    const unsigned PACK_SIZE = iBuilder->getStride()/8;
    Constant * packSize = iBuilder->getSize(PACK_SIZE);

    // Enter the loop only if there is at least one byte remaining to process.
    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(remainingBytes, iBuilder->getSize(0)), fbExit, radix64_loop);

    iBuilder->SetInsertPoint(radix64_loop);
    PHINode * idx = iBuilder->CreatePHI(iBuilder->getInt32Ty(), 2);
    PHINode * loopRemain = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
    idx->addIncoming(ConstantInt::getNullValue(iBuilder->getInt32Ty()), entry);
    loopRemain->addIncoming(remainingBytes, entry);

    Value * bytepack = loadInputStreamPack("expandedStream", iBuilder->getInt32(0), idx);
    Value * radix64pack = processPackData(bytepack);
    storeOutputStreamPack("radix64stream", iBuilder->getInt32(0), idx, radix64pack);

    Value* nextIdx = iBuilder->CreateAdd(idx, ConstantInt::get(iBuilder->getInt32Ty(), 1));
    idx->addIncoming(nextIdx, radix64_loop);
    Value* remainAfterLoop = iBuilder->CreateSub(loopRemain, packSize);
    loopRemain->addIncoming(remainAfterLoop, radix64_loop);

    Value* continueLoop = iBuilder->CreateICmpSGT(remainAfterLoop, iBuilder->getSize(0));

    iBuilder->CreateCondBr(continueLoop, radix64_loop, fbExit);

    iBuilder->SetInsertPoint(fbExit);
}

inline llvm::Value* base64Kernel::processPackData(llvm::Value* bytepack) const {
    Value * mask_gt_25 = iBuilder->simd_ugt(8, bytepack, iBuilder->simd_fill(8, iBuilder->getInt8(25)));
    Value * mask_gt_51 = iBuilder->simd_ugt(8, bytepack, iBuilder->simd_fill(8, iBuilder->getInt8(51)));
    Value * mask_eq_62 = iBuilder->simd_eq(8, bytepack, iBuilder->simd_fill(8, iBuilder->getInt8(62)));
    Value * mask_eq_63 = iBuilder->simd_eq(8, bytepack, iBuilder->simd_fill(8, iBuilder->getInt8(63)));
    // Strategy:
    // 1. add ord('A') = 65 to all radix64 values, this sets the correct values for entries 0 to 25.
    // 2. add ord('a') - ord('A') - (26 - 0) = 6 to all values >25, this sets the correct values for entries 0 to 51
    // 3. subtract ord('a') - ord('0') + (52 - 26) = 75 to all values > 51, this sets the correct values for entries 0 to 61
    // 4. subtract ord('0') - ord('+') + (62 - 52) = 15 for all values = 62
    // 4. add ord('/') - ord('0') - (63 - 52) = 3 for all values = 63
    Value * t0_25 = iBuilder->simd_add(8, bytepack, iBuilder->simd_fill(8, iBuilder->getInt8('A')));
    Value * t0_51 = iBuilder->simd_add(8, t0_25, iBuilder->simd_and(mask_gt_25, iBuilder->simd_fill(8, iBuilder->getInt8(6))));
    Value * t0_61 = iBuilder->simd_sub(8, t0_51, iBuilder->simd_and(mask_gt_51, iBuilder->simd_fill(8, iBuilder->getInt8(75))));
    Value * t0_62 = iBuilder->simd_sub(8, t0_61, iBuilder->simd_and(mask_eq_62, iBuilder->simd_fill(8, iBuilder->getInt8(15))));
    return iBuilder->bitCast(iBuilder->simd_sub(8, t0_62, iBuilder->simd_and(mask_eq_63, iBuilder->simd_fill(8, iBuilder->getInt8(12)))));
}

void base64Kernel::generateDoBlockMethod() {
    for (unsigned i = 0; i < 8; i++) {
        Value * bytepack = loadInputStreamPack("radix64stream", iBuilder->getInt32(0), iBuilder->getInt32(i));
        Value * base64pack = processPackData(bytepack);
        storeOutputStreamPack("base64stream", iBuilder->getInt32(0), iBuilder->getInt32(i), base64pack);
    }
    Value * produced = getProducedItemCount("base64stream");
    produced = iBuilder->CreateAdd(produced, iBuilder->getSize(iBuilder->getStride()));
    setProducedItemCount("base64stream", produced);
}

//// Special processing for the base 64 format.   The output must always contain a multiple
//// of 4 bytes.   When the number of radix 64 values is not a multiple of 4
//// number of radix 64 values
//void base64Kernel::generateFinalBlockMethod(Value * remainingBytes) {

//    BasicBlock * entry = iBuilder->GetInsertBlock();
//    BasicBlock * base64_loop = CreateBasicBlock("base64_loop");
//    BasicBlock * loopExit = CreateBasicBlock("loopExit");
//    BasicBlock * doPadding = CreateBasicBlock("doPadding");
//    BasicBlock * doPadding2 = CreateBasicBlock("doPadding2");
//    BasicBlock * fbExit = CreateBasicBlock("fbExit");

//    Value * remainMod4 = iBuilder->CreateAnd(remainingBytes, iBuilder->getSize(3));
//    Value * padBytes = iBuilder->CreateSub(iBuilder->getSize(4), remainMod4);
//    padBytes = iBuilder->CreateAnd(padBytes, iBuilder->getSize(3));

//    Constant * packSize = iBuilder->getSize(iBuilder->getStride() / 8);

//    // Enter the loop only if there is at least one byte remaining to process.
//    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(remainingBytes, iBuilder->getSize(0)), fbExit, base64_loop);

//    iBuilder->SetInsertPoint(base64_loop);
//    PHINode * idx = iBuilder->CreatePHI(iBuilder->getInt32Ty(), 2);
//    PHINode * loopRemain = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
//    idx->addIncoming(ConstantInt::getNullValue(iBuilder->getInt32Ty()), entry);
//    loopRemain->addIncoming(remainingBytes, entry);
//    Value * radix64streamPtr = getInputStream("radix64stream", iBuilder->getInt32(0), idx);
//    Value * bytepack = iBuilder->CreateBlockAlignedLoad(radix64streamPtr);
//    Value * base64pack = processPackData(bytepack);
//    Value * base64streamPtr = getOutputStream("base64stream", iBuilder->getInt32(0), idx);

//    iBuilder->CreateBlockAlignedStore(iBuilder->bitCast(base64pack), base64streamPtr);
//    idx->addIncoming(iBuilder->CreateAdd(idx, ConstantInt::get(iBuilder->getInt32Ty(), 1)), base64_loop);
//    Value* remainAfterLoop = iBuilder->CreateSub(loopRemain, packSize);
//    loopRemain->addIncoming(remainAfterLoop, base64_loop);

//    Value* continueLoop = iBuilder->CreateICmpSGT(remainAfterLoop, iBuilder->getSize(0));
//    iBuilder->CreateCondBr(continueLoop, base64_loop, loopExit);

//    iBuilder->SetInsertPoint(loopExit);
//    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(padBytes, iBuilder->getSize(0)), fbExit, doPadding);

//    iBuilder->SetInsertPoint(doPadding);

//    base64streamPtr = getOutputStream("base64stream", iBuilder->getInt32(0), idx);
//    Value * i8streamPtr = iBuilder->CreatePointerCast(base64streamPtr, iBuilder->getInt8PtrTy());
//    iBuilder->CreateStore(ConstantInt::get(iBuilder->getInt8Ty(), '='), iBuilder->CreateGEP(i8streamPtr, remainingBytes));
//    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(remainMod4, iBuilder->getSize(3)), fbExit, doPadding2);
//    iBuilder->SetInsertPoint(doPadding2);
//    Value * finalPadPos = iBuilder->CreateAdd(remainingBytes, iBuilder->getSize(1));
//    iBuilder->CreateStore(ConstantInt::get(iBuilder->getInt8Ty(), '='), iBuilder->CreateGEP(i8streamPtr, finalPadPos));
//    iBuilder->CreateBr(fbExit);
//    iBuilder->SetInsertPoint(fbExit);
//    Value * produced = iBuilder->CreateAdd(getProducedItemCount("base64stream"), iBuilder->CreateAdd(remainingBytes, padBytes));
//    setProducedItemCount("base64stream", produced);
//}

// Special processing for the base 64 format.   The output must always contain a multiple
// of 4 bytes.   When the number of radix 64 values is not a multiple of 4
// number of radix 64 values
void base64Kernel::generateFinalBlockMethod(Value * remainingBytes) {

    BasicBlock * entry = iBuilder->GetInsertBlock();
    BasicBlock * base64_loop = CreateBasicBlock("base64_loop");
    BasicBlock * loopExit = CreateBasicBlock("loopExit");
    BasicBlock * doPadding = CreateBasicBlock("doPadding");
    BasicBlock * doPadding2 = CreateBasicBlock("doPadding2");
    BasicBlock * fbExit = CreateBasicBlock("fbExit");

    Value * remainMod4 = iBuilder->CreateAnd(remainingBytes, iBuilder->getSize(3));
    Value * padBytes = iBuilder->CreateSub(iBuilder->getSize(4), remainMod4);
    padBytes = iBuilder->CreateAnd(padBytes, iBuilder->getSize(3));

    Constant * packSize = iBuilder->getSize(iBuilder->getStride() / 8);

    // Enter the loop only if there is at least one byte remaining to process.
    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(remainingBytes, iBuilder->getSize(0)), fbExit, base64_loop);

    iBuilder->SetInsertPoint(base64_loop);
    PHINode * idx = iBuilder->CreatePHI(iBuilder->getInt32Ty(), 2);
    PHINode * loopRemain = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
    idx->addIncoming(ConstantInt::getNullValue(iBuilder->getInt32Ty()), entry);
    loopRemain->addIncoming(remainingBytes, entry);
    Value * bytepack = loadInputStreamPack("radix64stream", iBuilder->getInt32(0), idx);
    Value * base64pack = processPackData(bytepack);
    storeOutputStreamPack("base64stream", iBuilder->getInt32(0), idx, base64pack);
    idx->addIncoming(iBuilder->CreateAdd(idx, ConstantInt::get(iBuilder->getInt32Ty(), 1)), base64_loop);
    Value* remainAfterLoop = iBuilder->CreateSub(loopRemain, packSize);
    loopRemain->addIncoming(remainAfterLoop, base64_loop);

    Value* continueLoop = iBuilder->CreateICmpSGT(remainAfterLoop, iBuilder->getSize(0));
    iBuilder->CreateCondBr(continueLoop, base64_loop, loopExit);

    iBuilder->SetInsertPoint(loopExit);
    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(padBytes, iBuilder->getSize(0)), fbExit, doPadding);

    iBuilder->SetInsertPoint(doPadding);
    Value * i8output_ptr = getOutputStreamBlockPtr("base64stream", iBuilder->getInt32(0));
    i8output_ptr = iBuilder->CreatePointerCast(i8output_ptr, iBuilder->getInt8PtrTy());
    iBuilder->CreateStore(ConstantInt::get(iBuilder->getInt8Ty(), '='), iBuilder->CreateGEP(i8output_ptr, remainingBytes));
    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(remainMod4, iBuilder->getSize(3)), fbExit, doPadding2);
    iBuilder->SetInsertPoint(doPadding2);
    Value * finalPadPos = iBuilder->CreateAdd(remainingBytes, iBuilder->getSize(1));
    iBuilder->CreateStore(ConstantInt::get(iBuilder->getInt8Ty(), '='), iBuilder->CreateGEP(i8output_ptr, finalPadPos));
    iBuilder->CreateBr(fbExit);
    iBuilder->SetInsertPoint(fbExit);
    Value * produced = iBuilder->CreateAdd(getProducedItemCount("base64stream"), iBuilder->CreateAdd(remainingBytes, padBytes));
    setProducedItemCount("base64stream", produced);
}

expand3_4Kernel::expand3_4Kernel(IDISA::IDISA_Builder * iBuilder)
: SegmentOrientedKernel(iBuilder, "expand3_4",
            {Binding{iBuilder->getStreamSetTy(1, 8), "sourceStream"}},
            {Binding{iBuilder->getStreamSetTy(1, 8), "expandedStream", new FixedRatio(4,3)}},
            {}, {}, {}) {
}

radix64Kernel::radix64Kernel(IDISA::IDISA_Builder * iBuilder)
: BlockOrientedKernel(iBuilder, "radix64",
            {Binding{iBuilder->getStreamSetTy(1, 8), "expandedStream"}},
            {Binding{iBuilder->getStreamSetTy(1, 8), "radix64stream"}},
            {}, {}, {}) {
}

base64Kernel::base64Kernel(IDISA::IDISA_Builder * iBuilder)
: BlockOrientedKernel(iBuilder, "base64",
            {Binding{iBuilder->getStreamSetTy(1, 8), "radix64stream"}},
            {Binding{iBuilder->getStreamSetTy(1, 8), "base64stream"}},
            {}, {}, {}) {
    setDoBlockUpdatesProducedItemCountsAttribute(true);
}

}
