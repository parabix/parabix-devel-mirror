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

void expand3_4Kernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &b, Value * const numOfStrides) {

    BasicBlock * expand2_3entry = b->GetInsertBlock();
    BasicBlock * expand_3_4_loop = b->CreateBasicBlock("expand_3_4_loop");
    BasicBlock * expand3_4_exit = b->CreateBasicBlock("expand3_4_exit");

    // Determine the require shufflevector constants.
    const unsigned PACK_SIZE = b->getBitBlockWidth()/8;

    ConstantInt * const ZERO = b->getSize(0);
    ConstantInt * const ONE = b->getSize(1);
    ConstantInt * const THREE = b->getSize(3);
    ConstantInt * const FOUR = b->getSize(4);
    ConstantInt * const SEVEN = b->getSize(7);

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
            Idxs.push_back(ConstantInt::get(b->getInt32Ty(), expand3_4_offset[j] + expand3_4_index[i]));
        }
        expand_3_4_shuffle[j] = ConstantVector::get(Idxs);
    }

    UndefValue * undefPack = UndefValue::get(b->fwVectorType(8));
    Value * const numOfBlocks = b->CreateMul(numOfStrides, b->getSize(8));
    // The main loop processes 3 packs of data at a time.
    b->CreateBr(expand_3_4_loop);

    b->SetInsertPoint(expand_3_4_loop);
    PHINode * strideOffset = b->CreatePHI(b->getSizeTy(), 2);
    strideOffset->addIncoming(ZERO, expand2_3entry);

    Value * const baseInputOffset = b->CreateMul(strideOffset, THREE);
    Value * const baseOutputOffset = b->CreateMul(strideOffset, FOUR);
    Value * carryOver = undefPack;
    for (unsigned i = 0; i < 3; ++i) {
        ConstantInt * const index = b->getSize(i);
        Value * const inputOffset = b->CreateAdd(baseInputOffset, index);
        Value * const inputPackIndex = b->CreateAnd(inputOffset, SEVEN);
        Value * const inputBlockOffset = b->CreateLShr(inputOffset, THREE);
        Value * const input = b->fwCast(8, b->loadInputStreamPack("sourceStream", ZERO, inputPackIndex, inputBlockOffset));
        Value * const expanded = b->CreateShuffleVector(carryOver, input, expand_3_4_shuffle[i]);
        Value * const outputOffset = b->CreateAdd(baseOutputOffset, index);
        Value * const outputPackIndex = b->CreateAnd(outputOffset, SEVEN);
        Value * const outputBlockOffset = b->CreateLShr(outputOffset, THREE);
        b->storeOutputStreamPack("expand34Stream", ZERO, outputPackIndex, outputBlockOffset, b->bitCast(expanded));
        carryOver = input;
    }
    Value * expanded = b->CreateShuffleVector(carryOver, undefPack, expand_3_4_shuffle[3]);
    Value * outputOffset = b->CreateAdd(baseOutputOffset, THREE);
    Value * const outputPackIndex = b->CreateAnd(outputOffset, SEVEN);
    Value * const outputBlockOffset = b->CreateLShr(outputOffset, THREE);
    b->storeOutputStreamPack("expand34Stream", ZERO, outputPackIndex, outputBlockOffset, b->bitCast(expanded));

    Value * const nextStrideOffset = b->CreateAdd(strideOffset, ONE);
    strideOffset->addIncoming(nextStrideOffset, expand_3_4_loop);
    Value * continueLoop = b->CreateICmpULT(nextStrideOffset, numOfBlocks);
    b->CreateCondBr(continueLoop, expand_3_4_loop, expand3_4_exit);

    b->SetInsertPoint(expand3_4_exit);
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
void base64Kernel::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & b, Value * remainingBytes) {

    BasicBlock * entry = b->GetInsertBlock();
    BasicBlock * base64_loop = b->CreateBasicBlock("base64_loop");
    BasicBlock * loopExit = b->CreateBasicBlock("loopExit");
    BasicBlock * doPadding = b->CreateBasicBlock("doPadding");
    BasicBlock * doPadding2 = b->CreateBasicBlock("doPadding2");
    BasicBlock * fbExit = b->CreateBasicBlock("fbExit");

    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);
    Constant * const THREE = b->getSize(3);
    Constant * const PADDING = b->getInt8('=');

    Value * remainMod4 = b->CreateAnd(remainingBytes, THREE);
    Value * padBytes = b->CreateAnd(b->CreateSub(b->getSize(4), remainMod4), THREE);

    Constant * const PACK_SIZE = b->getSize(b->getStride() / 8);

    // Enter the loop only if there is at least one byte remaining to process.
    b->CreateCondBr(b->CreateICmpEQ(remainingBytes, ZERO), fbExit, base64_loop);

    b->SetInsertPoint(base64_loop);
    PHINode * idx = b->CreatePHI(b->getSizeTy(), 2);
    PHINode * loopRemain = b->CreatePHI(b->getSizeTy(), 2);
    idx->addIncoming(ZERO, entry);
    loopRemain->addIncoming(remainingBytes, entry);
    Value * bytepack = b->loadInputStreamPack("radix64stream", ZERO, idx);
    Value * base64pack = processPackData(b, bytepack);
    b->storeOutputStreamPack("base64stream", ZERO, idx, base64pack);
    idx->addIncoming(b->CreateAdd(idx, ONE), base64_loop);
    Value * remainAfterLoop = b->CreateSub(loopRemain, PACK_SIZE);
    loopRemain->addIncoming(remainAfterLoop, base64_loop);
    Value * continueLoop = b->CreateICmpUGT(loopRemain, PACK_SIZE);
    b->CreateCondBr(continueLoop, base64_loop, loopExit);

    b->SetInsertPoint(loopExit);
    b->CreateCondBr(b->CreateICmpEQ(padBytes, ZERO), fbExit, doPadding);

    b->SetInsertPoint(doPadding);
    Value * i8output_ptr = b->getOutputStreamBlockPtr("base64stream", ZERO);
    i8output_ptr = b->CreatePointerCast(i8output_ptr, b->getInt8PtrTy());
    b->CreateStore(PADDING, b->CreateGEP(i8output_ptr, remainingBytes));
    b->CreateCondBr(b->CreateICmpEQ(remainMod4, THREE), fbExit, doPadding2);

    b->SetInsertPoint(doPadding2);
    Value * finalPadPos = b->CreateAdd(remainingBytes, ONE);
    b->CreateStore(PADDING, b->CreateGEP(i8output_ptr, finalPadPos));
    b->CreateBr(fbExit);
    b->SetInsertPoint(fbExit);
}

expand3_4Kernel::expand3_4Kernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet *input, StreamSet *expandedOutput)
: MultiBlockKernel(b, "expand3_4",
{Binding{"sourceStream", input, FixedRate(3)}},
{Binding{"expand34Stream", expandedOutput, FixedRate(4)}},
{}, {}, {}) {

}

radix64Kernel::radix64Kernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * input, StreamSet * output)
: BlockOrientedKernel(b, "radix64",
            {Binding{"expandedStream", input}},
            {Binding{"radix64stream", output}},
            {}, {}, {}) {
}

base64Kernel::base64Kernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * input, StreamSet * output)
: BlockOrientedKernel(b, "base64",
{Binding{"radix64stream", input}},
{Binding{"base64stream", output, FixedRate(1), RoundUpTo(4)}},
{}, {}, {}) {

}

}
