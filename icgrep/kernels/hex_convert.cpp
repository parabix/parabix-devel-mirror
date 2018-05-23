/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "hex_convert.h"
#include <kernels/kernel_builder.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>

using namespace kernel;
using namespace llvm;

HexToBinary::HexToBinary(const std::unique_ptr<kernel::KernelBuilder> & b)
: BlockOrientedKernel("HexToBinary",
                   {Binding{b->getStreamSetTy(1, 8), "hexdata", FixedRate()}},
                   {Binding{b->getStreamSetTy(1, 1), "binary_data", FixedRate(4)}},
{}, {}, {}) {}

void HexToBinary::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) {
    Type * i8Ty = b->getInt8Ty();
    Constant * const ZERO = b->getSize(0);
    const unsigned bytesPerBlock = b->getBitBlockWidth()/8;
    Constant * splat_0 = ConstantVector::getSplat(bytesPerBlock, ConstantInt::get(i8Ty, '0'));
    Constant * splat_9 = ConstantVector::getSplat(bytesPerBlock, ConstantInt::get(i8Ty, '9'));
    Constant * case_shift = ConstantVector::getSplat(bytesPerBlock, ConstantInt::get(i8Ty, 'a' - 'A'));
    Constant * splat_a = ConstantVector::getSplat(bytesPerBlock, ConstantInt::get(i8Ty, 'a'));
    Constant * splat_f = ConstantVector::getSplat(bytesPerBlock, ConstantInt::get(i8Ty, 'f'));
    Constant * af_cvt = ConstantVector::getSplat(bytesPerBlock, ConstantInt::get(i8Ty, 'a' - 10));

    for (unsigned i = 0; i < 4; i++) {
        Value * hexPack[2];
        hexPack[0] = b->loadInputStreamPack("hexdata", ZERO, b->getInt32(2*i));
        hexPack[1] = b->loadInputStreamPack("hexdata", ZERO, b->getInt32(2*i + 1));
        Value * base_val[2];
        for (unsigned j = 0; j < 2; j++) {
            Value * lc_hex = b->simd_or(hexPack[j], case_shift); // valid only for [A-Za-z]=>[a-z]
            Value * range_09 = b->simd_and(b->simd_ule(8, splat_0, hexPack[j]), b->simd_ule(8, hexPack[j], splat_9));
            Value * range_af = b->simd_and(b->simd_ule(8, splat_a, lc_hex), b->simd_ule(8, lc_hex, splat_f));
            base_val[j] = b->simd_or(b->simd_and(range_09, b->simd_sub(8, hexPack[j], splat_0)),
                                    b->simd_and(range_af, b->simd_sub(8, lc_hex, af_cvt)));
        }
        //b->CallPrintInt("binary_pack ptr", b->CreateGEP(outputStreamBasePtr, b->CreateUDiv(packNumPhi, TWO)));
        Value * binary_pack = b->bitCast(b->hsimd_packl(8, base_val[0], base_val[1]));
        b->storeOutputStreamBlock("binary_data", ZERO, b->getSize(i), binary_pack); 
    }
}

BinaryToHex::BinaryToHex(const std::unique_ptr<kernel::KernelBuilder> & b)
: BlockOrientedKernel("BinaryToHex",
                   {Binding{b->getStreamSetTy(1, 1), "binary_data", FixedRate(4)}},
                   {Binding{b->getStreamSetTy(1, 8), "hexdata", FixedRate(), RoundUpTo(1)}},
                   {}, {}, {}) {}


void BinaryToHex::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) {
    Type * i8Ty = b->getInt8Ty();
    Constant * const ZERO = b->getSize(0);
    const unsigned bytesPerBlock = b->getBitBlockWidth()/8;
    Constant * splatCh_0 = ConstantVector::getSplat(bytesPerBlock, ConstantInt::get(i8Ty, '0'));
    Constant * splat_9 = ConstantVector::getSplat(bytesPerBlock, ConstantInt::get(i8Ty, 9));
    Constant * splatCh_a = ConstantVector::getSplat(bytesPerBlock, ConstantInt::get(i8Ty, 'a'-10));
    for (unsigned i = 0; i < 4; i++) {
        Value * bit_data = b->loadInputStreamBlock("binary_data", ZERO, b->getInt32(i));
        Value * nybble_0 = b->fwCast(8, b->esimd_mergel(4, bit_data, b->allZeroes()));
        Value * nybble_1 = b->fwCast(8, b->esimd_mergeh(4, bit_data, b->allZeroes()));
        Value * data_09 = b->simd_add(8, nybble_0, splatCh_0);
        Value * data_af = b->simd_add(8, nybble_0, splatCh_a);
        Value * hex_data = b->bitCast(b->CreateSelect(b->CreateICmpULE(nybble_0, splat_9), data_09, data_af));
        b->storeOutputStreamPack("hexdata", ZERO, b->getInt32(2*i), hex_data);
        data_09 = b->simd_add(8, nybble_1, splatCh_0);
        data_af = b->simd_add(8, nybble_1, splatCh_a);
        hex_data = b->bitCast(b->CreateSelect(b->CreateICmpULE(nybble_1, splat_9), data_09, data_af));
        b->storeOutputStreamPack("hexdata", ZERO, b->getInt32(2*i + 1), hex_data);
    }
}

void BinaryToHex::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & b, Value * const remainingBits) {
    Value * priorProduced = b->getProducedItemCount("hexdata");
    CreateDoBlockMethodCall(b);
    Value * remainingHexDigits = b->CreateUDiv(b->CreateAdd(remainingBits, b->getSize(3)), b->getSize(4));
    Value * toZero = b->CreateSub(b->getSize(b->getBitBlockWidth()), remainingHexDigits);
    b->CreateMemZero(b->getRawOutputPointer("hexdata", b->CreateAdd(priorProduced, remainingHexDigits)), toZero);
}
