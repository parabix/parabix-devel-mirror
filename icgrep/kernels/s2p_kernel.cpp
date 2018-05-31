/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "s2p_kernel.h"
#include <kernels/kernel_builder.h>
#include <pablo/pabloAST.h>
#include <pablo/builder.hpp>
#include <pablo/pe_pack.h>

#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace kernel {

const int PACK_LANES = 2;

void s2p_step(const std::unique_ptr<KernelBuilder> & iBuilder, Value * s0, Value * s1, Value * hi_mask, unsigned shift, Value * &p0, Value * &p1) {
    Value * t0 = nullptr;
    Value * t1 = nullptr;
    if ((iBuilder->getBitBlockWidth() == 256) && (PACK_LANES == 2)) {
        Value * x0 = iBuilder->esimd_mergel(128, s0, s1);
        Value * x1 = iBuilder->esimd_mergeh(128, s0, s1);
        t0 = iBuilder->hsimd_packh_in_lanes(PACK_LANES, 16, x0, x1);
        t1 = iBuilder->hsimd_packl_in_lanes(PACK_LANES, 16, x0, x1);
    } else {
        t0 = iBuilder->hsimd_packh(16, s0, s1);
        t1 = iBuilder->hsimd_packl(16, s0, s1);
    }
    p0 = iBuilder->simd_if(1, hi_mask, t0, iBuilder->simd_srli(16, t1, shift));
    p1 = iBuilder->simd_if(1, hi_mask, iBuilder->simd_slli(16, t0, shift), t1);
}

void s2p(const std::unique_ptr<KernelBuilder> & iBuilder, Value * input[], Value * output[]) {
    Value * bit00224466[4];
    Value * bit11335577[4];

    for (unsigned i = 0; i < 4; i++) {
        Value * s0 = input[2 * i];
        Value * s1 = input[2 * i + 1];
        s2p_step(iBuilder, s0, s1, iBuilder->simd_himask(2), 1, bit00224466[i], bit11335577[i]);
    }
    Value * bit00004444[2];
    Value * bit22226666[2];
    Value * bit11115555[2];
    Value * bit33337777[2];
    for (unsigned j = 0; j<2; j++) {
        s2p_step(iBuilder, bit00224466[2*j], bit00224466[2*j+1],
                 iBuilder->simd_himask(4), 2, bit00004444[j], bit22226666[j]);
        s2p_step(iBuilder, bit11335577[2*j], bit11335577[2*j+1],
                 iBuilder->simd_himask(4), 2, bit11115555[j], bit33337777[j]);
    }
    s2p_step(iBuilder, bit00004444[0], bit00004444[1], iBuilder->simd_himask(8), 4, output[0], output[4]);
    s2p_step(iBuilder, bit11115555[0], bit11115555[1], iBuilder->simd_himask(8), 4, output[1], output[5]);
    s2p_step(iBuilder, bit22226666[0], bit22226666[1], iBuilder->simd_himask(8), 4, output[2], output[6]);
    s2p_step(iBuilder, bit33337777[0], bit33337777[1], iBuilder->simd_himask(8), 4, output[3], output[7]);
}

/* Alternative transposition model, but small field width packs are problematic. */
#if 0
void s2p_ideal(const std::unique_ptr<KernelBuilder> & iBuilder, Value * input[], Value * output[]) {
    Value * hi_nybble[4];
    Value * lo_nybble[4];
    for (unsigned i = 0; i<4; i++) {
        Value * s0 = input[2*i];
        Value * s1 = input[2*i+1];
        hi_nybble[i] = iBuilder->hsimd_packh(8, s0, s1);
        lo_nybble[i] = iBuilder->hsimd_packl(8, s0, s1);
    }
    Value * pair01[2];
    Value * pair23[2];
    Value * pair45[2];
    Value * pair67[2];
    for (unsigned i = 0; i<2; i++) {
        pair01[i] = iBuilder->hsimd_packh(4, hi_nybble[2*i], hi_nybble[2*i+1]);
        pair23[i] = iBuilder->hsimd_packl(4, hi_nybble[2*i], hi_nybble[2*i+1]);
        pair45[i] = iBuilder->hsimd_packh(4, lo_nybble[2*i], lo_nybble[2*i+1]);
        pair67[i] = iBuilder->hsimd_packl(4, lo_nybble[2*i], lo_nybble[2*i+1]);
    }
    output[0] = iBuilder->hsimd_packh(2, pair01[0], pair01[1]);
    output[1] = iBuilder->hsimd_packl(2, pair01[0], pair01[1]);
    output[2] = iBuilder->hsimd_packh(2, pair23[0], pair23[1]);
    output[3] = iBuilder->hsimd_packl(2, pair23[0], pair23[1]);
    output[4] = iBuilder->hsimd_packh(2, pair45[0], pair45[1]);
    output[5] = iBuilder->hsimd_packl(2, pair45[0], pair45[1]);
    output[6] = iBuilder->hsimd_packh(2, pair67[0], pair67[1]);
    output[7] = iBuilder->hsimd_packl(2, pair67[0], pair67[1]);
}
#endif
    
#if 0
void generateS2P_16Kernel(const std::unique_ptr<KernelBuilder> & iBuilder, Kernel * kBuilder) {
    kBuilder->addInputStream(16, "unit_pack");
    for(unsigned i = 0; i < 16; i++) {
	    kBuilder->addOutputStream(1);
    }
    Value * ptr = kBuilder->getInputStream(0);

    Value * lo[8];
    Value * hi[8];
    for (unsigned i = 0; i < 8; i++) {
        Value * s0 = iBuilder->CreateBlockAlignedLoad(ptr, {iBuilder->getInt32(0), iBuilder->getInt32(2 * i)});
        Value * s1 = iBuilder->CreateBlockAlignedLoad(ptr, {iBuilder->getInt32(0), iBuilder->getInt32(2 * i + 1)});
        lo[i] = iBuilder->hsimd_packl(16, s0, s1);
        hi[i] = iBuilder->hsimd_packh(16, s0, s1);
    }

    Value * output[16];
    s2p(iBuilder, lo, output);
    s2p(iBuilder, hi, output + 8);
    for (unsigned j = 0; j < 16; j++) {
        iBuilder->CreateBlockAlignedStore(output[j], kBuilder->getOutputStream(j));
    }
}    
#endif

void S2PKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, Value * const numOfBlocks) {
    BasicBlock * entry = kb->GetInsertBlock();
    BasicBlock * processBlock = kb->CreateBasicBlock("processBlock");
    BasicBlock * s2pDone = kb->CreateBasicBlock("s2pDone");
    Constant * const ZERO = kb->getSize(0);

    kb->CreateBr(processBlock);
    
    kb->SetInsertPoint(processBlock);
    PHINode * blockOffsetPhi = kb->CreatePHI(kb->getSizeTy(), 2); // block offset from the base block, e.g. 0, 1, 2, ...
    blockOffsetPhi->addIncoming(ZERO, entry);

    Value * bytepack[8];
    for (unsigned i = 0; i < 8; i++) {
        if (mAligned) {
            bytepack[i] = kb->loadInputStreamPack("byteStream", ZERO, kb->getInt32(i), blockOffsetPhi);
        } else {
            Value * ptr = kb->getInputStreamPackPtr("byteStream", ZERO, kb->getInt32(i), blockOffsetPhi);
            // CreateLoad defaults to aligned here, so we need to force the alignment to 1 byte.
            bytepack[i] = kb->CreateAlignedLoad(ptr, 1);
        }
    }
    Value * basisbits[8];
    s2p(kb, bytepack, basisbits);
    for (unsigned i = 0; i < 8; ++i) {
        kb->storeOutputStreamBlock("basisBits", kb->getInt32(i), blockOffsetPhi, basisbits[i]);
    }
    Value * nextBlk = kb->CreateAdd(blockOffsetPhi, kb->getSize(1));
    blockOffsetPhi->addIncoming(nextBlk, processBlock);
    Value * moreToDo = kb->CreateICmpNE(nextBlk, numOfBlocks);
    kb->CreateCondBr(moreToDo, processBlock, s2pDone);
    kb->SetInsertPoint(s2pDone);
}
S2PKernel::S2PKernel(const std::unique_ptr<KernelBuilder> & b, bool aligned, std::string prefix)
: MultiBlockKernel(aligned ? prefix + "s2p" : prefix + "s2p_unaligned",
    {Binding{b->getStreamSetTy(1, 8), "byteStream", FixedRate(), Principal()}},
    {Binding{b->getStreamSetTy(8, 1), "basisBits"}}, {}, {}, {}),
  mAligned(aligned) {
    if (!aligned) {
        mStreamSetInputs[0].addAttribute(Misaligned());
    }
}
void S2P_PabloKernel::generatePabloMethod() {
    pablo::PabloBlock * const pb = getEntryScope();
    const unsigned steps = std::log2(mCodeUnitWidth);
    std::vector<PabloAST *> streamSet[steps + 1];
    for (unsigned i = 0; i <= steps; i++) {
        streamSet[i].resize(1<<i);
    }
    streamSet[0][0] = pb->createExtract(getInputStreamVar("codeUnitStream"), pb->getInteger(0));
    unsigned streamWidth = mCodeUnitWidth;
    for (unsigned i = 1; i <= steps; i++) {
        for (unsigned j = 0; j < streamSet[i-1].size(); j++) {
            auto strm = streamSet[i-1][j];
            streamSet[i][2*j] = pb->createPackL(pb->getInteger(streamWidth), strm);
            streamSet[i][2*j+1] = pb->createPackH(pb->getInteger(streamWidth), strm);
        }
        streamWidth = streamWidth/2;
    }
    for (unsigned bit = 0; bit < mCodeUnitWidth; bit++) {
        pb->createAssign(pb->createExtract(getOutputStreamVar("basisBits"), pb->getInteger(bit)), streamSet[steps][mCodeUnitWidth-1-bit]);
    }
}

S2P_PabloKernel::S2P_PabloKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const unsigned codeUnitWidth)
: PabloKernel(b, "s2p_pablo" + std::to_string(codeUnitWidth),
    {Binding{b->getStreamSetTy(1, codeUnitWidth), "codeUnitStream"}},
    {Binding{b->getStreamSetTy(codeUnitWidth, 1), "basisBits"}}),
  mCodeUnitWidth(codeUnitWidth) {
}


S2PByPextKernel::S2PByPextKernel(const std::unique_ptr<kernel::KernelBuilder> &b, std::string prefix)
        : BlockOrientedKernel(prefix + "s2pByPext",
                           {Binding{b->getStreamSetTy(1, 8), "byteStream", FixedRate(), Principal()}},
                           {Binding{b->getStreamSetTy(8, 1), "basisBits"}}, {}, {}, {})
{

}

void S2PByPextKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> &b) {
    Value* inputBasePtr = b->CreatePointerCast(b->getInputStreamBlockPtr("byteStream", b->getSize(0)), b->getInt64Ty()->getPointerTo());
    std::vector<Value*> outputPtrs(8, nullptr);
    for (unsigned i = 0; i < 8; i++) {
        outputPtrs[i] = b->CreatePointerCast(b->getOutputStreamBlockPtr("basisBits", b->getSize(i)), b->getInt64Ty()->getPointerTo());
    }
    uint64_t base_mask = 0x0101010101010101;

    Constant * pext = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pext_64);

    for (unsigned iBlockIndex = 0; iBlockIndex < 4; iBlockIndex++) {
        std::vector<Value*> tempValues(8, b->getInt64(0));
        for (size_t iDataIndex = 0; iDataIndex < 8; iDataIndex++) {
            Value* inputData = b->CreateLoad(b->CreateGEP(inputBasePtr, b->getSize(iDataIndex + iBlockIndex * 8)));
            for (int iStreamIndex = 0; iStreamIndex < 8; iStreamIndex++) {
                Value* targetMask = b->getInt64(base_mask << iStreamIndex);
                Value * const outputValue = b->CreateCall(pext, {inputData, targetMask});
                tempValues[iStreamIndex] = b->CreateOr(tempValues[iStreamIndex], b->CreateShl(outputValue, b->getInt64(iDataIndex * 8)));
            }
        }
        for (int iStreamIndex = 0; iStreamIndex < 8; iStreamIndex++) {
            b->CreateStore(tempValues[iStreamIndex], b->CreateGEP(outputPtrs[7 - iStreamIndex], b->getSize(iBlockIndex)));
        }
    }
}
}
