/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#include "radix64.h"
#include <kernels/streamset.h>
#include <kernels/kernel_builder.h>

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

// The MultiBlockLogic is based on a natural stride taking 3 packs at a time.
// In this case, the output produced is exactly 4 packs or 4 blocks, with no pending
// data maintained in the kernel state.
//
// When processing the final partial stride of data, the kernel performs full
// triple-pack processing for each full or partial triple-pack remaining,
// relying on the MultiBlockKernel builder to only copy the correct number
// of bytes to the actual output stream.

void expand3_4Kernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & iBuilder, Value * const numOfStrides) {

    BasicBlock * expand2_3entry = iBuilder->GetInsertBlock();
    BasicBlock * expand_3_4_loop = iBuilder->CreateBasicBlock("expand_3_4_loop");
    BasicBlock * expand3_4_exit = iBuilder->CreateBasicBlock("expand3_4_exit");
    
    // Determine the require shufflevector constants.
    const unsigned PACK_SIZE = iBuilder->getBitBlockWidth()/8;
    
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

    Constant * triplePackSize = iBuilder->getSize(3 * PACK_SIZE); // 3 packs per loop.
    UndefValue * undefPack = UndefValue::get(iBuilder->fwVectorType(8));
    
    const unsigned packAlign = iBuilder->getBitBlockWidth()/8;

    Value * itemsToDo = mAvailableItemCount[0];

    Value * sourceStream = iBuilder->getInputStreamBlockPtr("sourceStream", iBuilder->getInt32(0));
    Value * expandedStream = iBuilder->getOutputStreamBlockPtr("expand34Stream", iBuilder->getInt32(0));

    // The main loop processes 3 packs of data at a time.
    // The initial pack offsets may be nonzero.
    sourceStream = iBuilder->CreatePointerCast(sourceStream, iBuilder->getInt8PtrTy());
    expandedStream = iBuilder->CreatePointerCast(expandedStream, iBuilder->getInt8PtrTy());
    Value * offset = iBuilder->CreateURem(iBuilder->getProcessedItemCount("sourceStream"), iBuilder->getSize(iBuilder->getBitBlockWidth()));
    Value * sourcePackPtr = iBuilder->CreatePointerCast(iBuilder->CreateGEP(sourceStream, offset), iBuilder->getBitBlockType()->getPointerTo());
    offset = iBuilder->CreateURem(iBuilder->getProducedItemCount("expand34Stream"), iBuilder->getSize(iBuilder->getBitBlockWidth()));
    Value * outputPackPtr = iBuilder->CreatePointerCast(iBuilder->CreateGEP(expandedStream, offset), iBuilder->getBitBlockType()->getPointerTo());
    iBuilder->CreateCondBr(iBuilder->CreateICmpSGT(itemsToDo, iBuilder->getSize(0)), expand_3_4_loop, expand3_4_exit);
    
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

    Value * continueLoop = iBuilder->CreateICmpSGT(remainingItems, iBuilder->getSize(0));
    iBuilder->CreateCondBr(continueLoop, expand_3_4_loop, expand3_4_exit);
    
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
inline Value * radix64Kernel::processPackData(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value * bytepack) const {

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

void radix64Kernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    for (unsigned i = 0; i < 8; i++) {
        Value * bytepack = iBuilder->loadInputStreamPack("expandedStream", iBuilder->getInt32(0), iBuilder->getInt32(i));
        Value * radix64pack = processPackData(iBuilder, bytepack);
        iBuilder->storeOutputStreamPack("radix64stream", iBuilder->getInt32(0), iBuilder->getInt32(i), radix64pack);
    }
}

void radix64Kernel::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder, Value * remainingBytes) {

    BasicBlock * entry = iBuilder->GetInsertBlock();
    BasicBlock * radix64_loop = iBuilder->CreateBasicBlock("radix64_loop");
    BasicBlock * fbExit = iBuilder->CreateBasicBlock("fbExit");
    
    const unsigned PACK_SIZE = iBuilder->getStride()/8;
    Constant * packSize = iBuilder->getSize(PACK_SIZE);

    // Enter the loop only if there is at least one byte remaining to process.
    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(remainingBytes, iBuilder->getSize(0)), fbExit, radix64_loop);

    iBuilder->SetInsertPoint(radix64_loop);
    PHINode * idx = iBuilder->CreatePHI(iBuilder->getInt32Ty(), 2);
    PHINode * loopRemain = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
    idx->addIncoming(ConstantInt::getNullValue(iBuilder->getInt32Ty()), entry);
    loopRemain->addIncoming(remainingBytes, entry);

    Value * bytepack = iBuilder->loadInputStreamPack("expandedStream", iBuilder->getInt32(0), idx);
    Value * radix64pack = processPackData(iBuilder, bytepack);
    iBuilder->storeOutputStreamPack("radix64stream", iBuilder->getInt32(0), idx, radix64pack);

    Value* nextIdx = iBuilder->CreateAdd(idx, ConstantInt::get(iBuilder->getInt32Ty(), 1));
    idx->addIncoming(nextIdx, radix64_loop);
    Value* remainAfterLoop = iBuilder->CreateSub(loopRemain, packSize);
    loopRemain->addIncoming(remainAfterLoop, radix64_loop);

    Value* continueLoop = iBuilder->CreateICmpSGT(remainAfterLoop, iBuilder->getSize(0));

    iBuilder->CreateCondBr(continueLoop, radix64_loop, fbExit);

    iBuilder->SetInsertPoint(fbExit);
}

inline llvm::Value* base64Kernel::processPackData(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value* bytepack) const {
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

void base64Kernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    for (unsigned i = 0; i < 8; i++) {
        Value * bytepack = iBuilder->loadInputStreamPack("radix64stream", iBuilder->getInt32(0), iBuilder->getInt32(i));
        Value * base64pack = processPackData(iBuilder, bytepack);
        iBuilder->storeOutputStreamPack("base64stream", iBuilder->getInt32(0), iBuilder->getInt32(i), base64pack);
    }
}

// Special processing for the base 64 format.   The output must always contain a multiple
// of 4 bytes.   When the number of radix 64 values is not a multiple of 4
// number of radix 64 values
void base64Kernel::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder, Value * remainingBytes) {

    BasicBlock * entry = iBuilder->GetInsertBlock();
    BasicBlock * base64_loop = iBuilder->CreateBasicBlock("base64_loop");
    BasicBlock * loopExit = iBuilder->CreateBasicBlock("loopExit");
    BasicBlock * doPadding = iBuilder->CreateBasicBlock("doPadding");
    BasicBlock * doPadding2 = iBuilder->CreateBasicBlock("doPadding2");
    BasicBlock * fbExit = iBuilder->CreateBasicBlock("fbExit");

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
    Value * bytepack = iBuilder->loadInputStreamPack("radix64stream", iBuilder->getInt32(0), idx);
    Value * base64pack = processPackData(iBuilder, bytepack);
    iBuilder->storeOutputStreamPack("base64stream", iBuilder->getInt32(0), idx, base64pack);
    idx->addIncoming(iBuilder->CreateAdd(idx, ConstantInt::get(iBuilder->getInt32Ty(), 1)), base64_loop);
    Value* remainAfterLoop = iBuilder->CreateSub(loopRemain, packSize);
    loopRemain->addIncoming(remainAfterLoop, base64_loop);

    Value* continueLoop = iBuilder->CreateICmpSGT(remainAfterLoop, iBuilder->getSize(0));
    iBuilder->CreateCondBr(continueLoop, base64_loop, loopExit);

    iBuilder->SetInsertPoint(loopExit);
    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(padBytes, iBuilder->getSize(0)), fbExit, doPadding);

    iBuilder->SetInsertPoint(doPadding);
    Value * i8output_ptr = iBuilder->getOutputStreamBlockPtr("base64stream", iBuilder->getInt32(0));
    i8output_ptr = iBuilder->CreatePointerCast(i8output_ptr, iBuilder->getInt8PtrTy());
    iBuilder->CreateStore(ConstantInt::get(iBuilder->getInt8Ty(), '='), iBuilder->CreateGEP(i8output_ptr, remainingBytes));
    iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(remainMod4, iBuilder->getSize(3)), fbExit, doPadding2);
    iBuilder->SetInsertPoint(doPadding2);
    Value * finalPadPos = iBuilder->CreateAdd(remainingBytes, iBuilder->getSize(1));
    iBuilder->CreateStore(ConstantInt::get(iBuilder->getInt8Ty(), '='), iBuilder->CreateGEP(i8output_ptr, finalPadPos));
    iBuilder->CreateBr(fbExit);
    iBuilder->SetInsertPoint(fbExit);
}

expand3_4Kernel::expand3_4Kernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder)
: MultiBlockKernel("expand3_4",
            {Binding{iBuilder->getStreamSetTy(1, 8), "sourceStream", FixedRate(3)}},
            {Binding{iBuilder->getStreamSetTy(1, 8), "expand34Stream", FixedRate(4)}},
            {}, {}, {}) {

}

radix64Kernel::radix64Kernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder)
: BlockOrientedKernel("radix64",
            {Binding{iBuilder->getStreamSetTy(1, 8), "expandedStream"}},
            {Binding{iBuilder->getStreamSetTy(1, 8), "radix64stream"}},
            {}, {}, {}) {
}

base64Kernel::base64Kernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder)
: BlockOrientedKernel("base64",
            {Binding{iBuilder->getStreamSetTy(1, 8), "radix64stream"}},
            {Binding{iBuilder->getStreamSetTy(1, 8), "base64stream", FixedRate(1), RoundUpTo(4)}},
            {}, {}, {}) {
}

}
