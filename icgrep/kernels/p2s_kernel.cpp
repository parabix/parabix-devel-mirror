#include "p2s_kernel.h"
#include <kernels/core/streamset.h>
#include <kernels/kernel_builder.h>
#include <toolchain/toolchain.h>
#include <llvm/Support/Compiler.h>

namespace llvm { class Value; }

using namespace llvm;

namespace kernel{

void p2s_step(const std::unique_ptr<KernelBuilder> & iBuilder, Value * p0, Value * p1, Value * hi_mask, unsigned shift, Value * &s1, Value * &s0) {
    Value * t0 = iBuilder->simd_if(1, hi_mask, p0, iBuilder->simd_srli(16, p1, shift));
    Value * t1 = iBuilder->simd_if(1, hi_mask, iBuilder->simd_slli(16, p0, shift), p1);
    s1 = iBuilder->esimd_mergeh(8, t1, t0);
    s0 = iBuilder->esimd_mergel(8, t1, t0);
}

inline void p2s(const std::unique_ptr<KernelBuilder> & iBuilder, Value * p[], Value * s[]) {
    Value * bit00004444[2];
    Value * bit22226666[2];
    Value * bit11115555[2];
    Value * bit33337777[2];
    p2s_step(iBuilder, p[7], p[3], iBuilder->simd_himask(8), 4, bit00004444[1], bit00004444[0]);
    p2s_step(iBuilder, p[6], p[2], iBuilder->simd_himask(8), 4, bit11115555[1], bit11115555[0]);
    p2s_step(iBuilder, p[5], p[1], iBuilder->simd_himask(8), 4, bit22226666[1], bit22226666[0]);
    p2s_step(iBuilder, p[4], p[0], iBuilder->simd_himask(8), 4, bit33337777[1], bit33337777[0]);
    Value * bit00224466[4];
    Value * bit11335577[4];
    for (unsigned j = 0; j<2; j++) {
        p2s_step(iBuilder, bit00004444[j], bit22226666[j],iBuilder->simd_himask(4), 2, bit00224466[2*j+1], bit00224466[2*j]);
        p2s_step(iBuilder, bit11115555[j], bit33337777[j],iBuilder->simd_himask(4), 2, bit11335577[2*j+1], bit11335577[2*j]);
    }
    for (unsigned j = 0; j<4; j++) {
        p2s_step(iBuilder, bit00224466[j], bit11335577[j], iBuilder->simd_himask(2), 1, s[2*j+1], s[2*j]);
    }
}

void P2SKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) {
    const auto numOfStreams = getInputStreamSet("basisBits")->getNumElements();
    Value * p_bitblock[8];
    // todo: generalize this to the nearest pow 2?
    for (unsigned i = 0; i < 8; i++) {
        if (i < numOfStreams) {
            p_bitblock[i] = b->loadInputStreamBlock("basisBits", b->getInt32(i));
        } else {
            p_bitblock[i] = ConstantVector::getNullValue(b->getBitBlockType());
        }

    }
    Value * s_bytepack[8];
    p2s(b, p_bitblock, s_bytepack);
    for (unsigned j = 0; j < 8; ++j) {
        b->storeOutputStreamPack("byteStream", b->getInt32(0), b->getInt32(j), s_bytepack[j]);
    }
}


void P2SMultipleStreamsKernel::generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> &b) {
    Value * input[8];
    unsigned k = 0;
    for (unsigned i = 0; i < getNumOfStreamInputs(); ++i) {
        const auto m = getInputStreamSet(i)->getNumElements();
        for (unsigned j = 0; j < m; j++) {
            input[k++] = b->loadInputStreamBlock("basisBits_" + std::to_string(i), b->getInt32(j));
        }
    }
    assert (k <= 8);
    while (k < 8) {
        input[k++] = ConstantVector::getNullValue(b->getBitBlockType());
    }

    Value * output[8];
    p2s(b, input, output);
    for (unsigned j = 0; j < 8; ++j) {
        b->storeOutputStreamPack("byteStream", b->getInt32(0), b->getInt32(j), output[j]);
    }
}


inline Value * partial_sum_popcounts(const std::unique_ptr<KernelBuilder> & iBuilder, const unsigned fw, Value * popcounts) {
    Value * summed_counts = popcounts;
    const auto count = iBuilder->getBitBlockWidth() / fw;
    for (unsigned move = 1; move < count; move *= 2) {
        summed_counts = iBuilder->simd_add(fw, summed_counts, iBuilder->mvmd_slli(fw, summed_counts, move));
    }
    return summed_counts;
}

void P2SKernelWithCompressedOutput::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) {
    IntegerType * i32 = b->getInt32Ty();
    PointerType * bitBlockPtrTy = PointerType::get(b->getBitBlockType(), 0);
    unsigned const unitsPerRegister = b->getBitBlockWidth()/8;

    Value * basisBits[8];
    for (unsigned i = 0; i < 8; i++) {
        basisBits[i] = b->loadInputStreamBlock("basisBits", b->getInt32(i));
    }
    Value * bytePack[8];
    p2s(b, basisBits, bytePack);

    Value * const fieldCounts = b->loadInputStreamBlock("fieldCounts", b->getInt32(0));
    Value * unitCounts = partial_sum_popcounts(b, unitsPerRegister, fieldCounts);

    Value * output_ptr = b->getOutputStreamBlockPtr("byteStream", b->getInt32(0));
    output_ptr = b->CreatePointerCast(output_ptr, b->getInt8PtrTy());
    Value * offset = b->getInt32(0);
    for (unsigned j = 0; j < 8; ++j) {
        b->CreateStore(bytePack[j], b->CreateBitCast(b->CreateGEP(output_ptr, offset), bitBlockPtrTy));
        offset = b->CreateZExt(b->CreateExtractElement(unitCounts, b->getInt32(j)), i32);
    }

    Value * unitsGenerated = b->getProducedItemCount("byteStream"); // units generated to buffer
    unitsGenerated = b->CreateAdd(unitsGenerated, b->CreateZExt(offset, b->getSizeTy()));
    b->setProducedItemCount("byteStream", unitsGenerated);
}

void P2S16Kernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) {
    Value * hi_input[8];
    for (unsigned j = 0; j < 8; ++j) {
        const unsigned idx = j + 8;
        hi_input[j] = b->loadInputStreamBlock("basisBits", b->getInt32(idx));
    }
    Value * hi_bytes[8];
    p2s(b, hi_input, hi_bytes);
    Value * lo_input[8];
    for (unsigned j = 0; j < 8; ++j) {
        lo_input[j] = b->loadInputStreamBlock("basisBits", b->getInt32(j));
    }
    Value * lo_bytes[8];
    p2s(b, lo_input, lo_bytes);
    for (unsigned j = 0; j < 8; ++j) {
        Value * merge0 = b->bitCast(b->esimd_mergel(8, hi_bytes[j], lo_bytes[j]));
        Value * merge1 = b->bitCast(b->esimd_mergeh(8, hi_bytes[j], lo_bytes[j]));
        b->storeOutputStreamPack("i16Stream", b->getInt32(0), b->getInt32(2 * j), merge0);
        b->storeOutputStreamPack("i16Stream", b->getInt32(0), b->getInt32(2 * j + 1), merge1);
    }
}

void P2S16KernelWithCompressedOutput::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) {
    IntegerType * i32Ty = b->getInt32Ty();
    PointerType * int16PtrTy = b->getInt16Ty()->getPointerTo();
    PointerType * bitBlockPtrTy = b->getBitBlockType()->getPointerTo();
    ConstantInt * blockMask = b->getSize(b->getBitBlockWidth() - 1);
    ConstantInt * ZERO = b->getInt32(0);
    unsigned const unitsPerRegister = b->getBitBlockWidth()/16;

    Value * hi_input[8];
    for (unsigned j = 0; j < 8; ++j) {
        const unsigned idx = j + 8;
        hi_input[j] = b->loadInputStreamBlock("basisBits", b->getInt32(idx));
    }
    Value * hi_bytes[8];
    p2s(b, hi_input, hi_bytes);

    Value * lo_input[8];
    for (unsigned j = 0; j < 8; ++j) {
        lo_input[j] = b->loadInputStreamBlock("basisBits", b->getInt32(j));
    }
    Value * lo_bytes[8];
    p2s(b, lo_input, lo_bytes);
    Value * const extractionMask = b->loadInputStreamBlock("extractionMask", ZERO);
    Value * const fieldCounts = b->simd_popcount(unitsPerRegister, extractionMask);
    Value * unitCounts = partial_sum_popcounts(b, unitsPerRegister, fieldCounts);
    Value * outputPtr = b->getOutputStreamBlockPtr("i16Stream", ZERO);
    outputPtr = b->CreatePointerCast(outputPtr, int16PtrTy);
    Value * const i16UnitsGenerated = b->getProducedItemCount("i16Stream"); // units generated to buffer
    outputPtr = b->CreateGEP(outputPtr, b->CreateAnd(i16UnitsGenerated, blockMask));

    Value * offset = ZERO;
    for (unsigned j = 0; j < 8; ++j) {
        Value * const merge0 = b->bitCast(b->esimd_mergel(8, hi_bytes[j], lo_bytes[j]));
        b->CreateAlignedStore(merge0, b->CreateBitCast(b->CreateGEP(outputPtr, offset), bitBlockPtrTy), 1);
        Value * const nextOffset1 = b->CreateZExt(b->CreateExtractElement(unitCounts, b->getInt32(2 * j)), i32Ty);
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            b->CreateAssert(b->CreateICmpULE(offset, nextOffset1), "deletion offset is not monotonically non-decreasing");
        }
        Value * const merge1 = b->bitCast(b->esimd_mergeh(8, hi_bytes[j], lo_bytes[j]));
        b->CreateAlignedStore(merge1, b->CreateBitCast(b->CreateGEP(outputPtr, nextOffset1), bitBlockPtrTy), 1);
        Value * const nextOffset2 = b->CreateZExt(b->CreateExtractElement(unitCounts, b->getInt32(2 * j + 1)), i32Ty);
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            b->CreateAssert(b->CreateICmpULE(nextOffset1, nextOffset2), "deletion offset is not monotonically non-decreasing");
        }
        offset = nextOffset2;
    }
    Value * const i16UnitsFinal = b->CreateAdd(i16UnitsGenerated, b->CreateZExt(offset, b->getSizeTy()));
    b->setProducedItemCount("i16Stream", i16UnitsFinal);
}




P2SKernel::P2SKernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * basisBits, StreamSet * byteStream)
: BlockOrientedKernel(b, "p2s",
{Binding{"basisBits", basisBits}},
{Binding{"byteStream", byteStream}},
{}, {}, {}) {

}

P2SMultipleStreamsKernel::P2SMultipleStreamsKernel(const std::unique_ptr<kernel::KernelBuilder> &b,
                                                   const StreamSets & inputStreams,
                                                   StreamSet * const outputStream)
: BlockOrientedKernel(b, "p2sMultipleStreams",
{},
{Binding{"byteStream", outputStream}},
{}, {}, {}) {
    for (unsigned i = 0; i < inputStreams.size(); i++) {
        mInputStreamSets.emplace_back("basisBits_" + std::to_string(i), inputStreams[i]);
    }
}

P2SKernelWithCompressedOutput::P2SKernelWithCompressedOutput(const std::unique_ptr<kernel::KernelBuilder> & b)
: BlockOrientedKernel(b, "p2s_compress",
{Binding{b->getStreamSetTy(8, 1), "basisBits"}, Binding{b->getStreamSetTy(1, 1), "extractionMask"}},
{Binding{b->getStreamSetTy(1, 8), "byteStream", BoundedRate(0, 1)}},
{}, {}, {}) {

}

P2S16Kernel::P2S16Kernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet *u16bits, StreamSet *u16bytes)
: BlockOrientedKernel(b, "p2s_16",
{Binding{"basisBits", u16bits}},
{Binding{"i16Stream", u16bytes}},
{}, {}, {}) {

}

P2S16KernelWithCompressedOutput::P2S16KernelWithCompressedOutput(const std::unique_ptr<kernel::KernelBuilder> & b,
                                                                 StreamSet * basisBits, StreamSet * extractionMask, StreamSet * i16Stream)
: BlockOrientedKernel(b, "p2s_16_compress",
{Binding{"basisBits", basisBits},
Binding{"extractionMask", extractionMask}},
{Binding{"i16Stream", i16Stream, BoundedRate(0, 1)}},
{}, {}, {}) {

}


}
