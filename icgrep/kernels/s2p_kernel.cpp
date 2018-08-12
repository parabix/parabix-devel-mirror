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

        t0 = iBuilder->hsimd_packh_in_lanes(PACK_LANES, 16, x0, x1); // TODO 4个bit streams时这里的16改为8?
        t1 = iBuilder->hsimd_packl_in_lanes(PACK_LANES, 16, x0, x1);

    } else {
        t0 = iBuilder->hsimd_packh(16, s0, s1);
        t1 = iBuilder->hsimd_packl(16, s0, s1);
    }

    p0 = iBuilder->simd_if(1, hi_mask, t0, iBuilder->simd_srli(16, t1, shift));
    p1 = iBuilder->simd_if(1, hi_mask, iBuilder->simd_slli(16, t0, shift), t1);
}

void s2p(const std::unique_ptr<KernelBuilder> & iBuilder, Value * input[], Value * output[], cc::BitNumbering basisNumbering) {
    {
        //input[0 - 3]
        Value* bit3311[2];
        Value* bit2200[2];
        for (unsigned i = 0; i < 2; i++) {
            s2p_step(iBuilder, input[2 * i], input[2 * i + 1], iBuilder->simd_himask(2), 1, bit3311[i], bit2200[i]);
        }

        Value* out[4];
        s2p_step(iBuilder, bit3311[0], bit3311[1],
                 iBuilder->simd_himask(4), 2, out[3], out[1]);

        s2p_step(iBuilder, bit2200[0], bit2200[1],
                 iBuilder->simd_himask(4), 2, out[2], out[0]);
    }


    // Little-endian bit number is used for variables.
    Value * bit66442200[4];
    Value * bit77553311[4];

    for (unsigned i = 0; i < 4; i++) {
        Value * s0 = input[2 * i];
        Value * s1 = input[2 * i + 1];
        s2p_step(iBuilder, s0, s1, iBuilder->simd_himask(2), 1, bit77553311[i], bit66442200[i]);
    }
    Value * bit44440000[2];
    Value * bit66662222[2];
    Value * bit55551111[2];
    Value * bit77773333[2];
    for (unsigned j = 0; j<2; j++) {
        s2p_step(iBuilder, bit66442200[2*j], bit66442200[2*j+1],
                 iBuilder->simd_himask(4), 2, bit66662222[j], bit44440000[j]);
        s2p_step(iBuilder, bit77553311[2*j], bit77553311[2*j+1],
                 iBuilder->simd_himask(4), 2, bit77773333[j], bit55551111[j]);
    }
    if (basisNumbering == cc::BitNumbering::LittleEndian) {
        s2p_step(iBuilder, bit44440000[0], bit44440000[1], iBuilder->simd_himask(8), 4, output[4], output[0]);
        s2p_step(iBuilder, bit55551111[0], bit55551111[1], iBuilder->simd_himask(8), 4, output[5], output[1]);
        s2p_step(iBuilder, bit66662222[0], bit66662222[1], iBuilder->simd_himask(8), 4, output[6], output[2]);
        s2p_step(iBuilder, bit77773333[0], bit77773333[1], iBuilder->simd_himask(8), 4, output[7], output[3]);
    }
    else {
        s2p_step(iBuilder, bit44440000[0], bit44440000[1], iBuilder->simd_himask(8), 4, output[3], output[7]);
        s2p_step(iBuilder, bit55551111[0], bit55551111[1], iBuilder->simd_himask(8), 4, output[2], output[6]);
        s2p_step(iBuilder, bit66662222[0], bit66662222[1], iBuilder->simd_himask(8), 4, output[1], output[5]);
        s2p_step(iBuilder, bit77773333[0], bit77773333[1], iBuilder->simd_himask(8), 4, output[0], output[4]);
    }
}

/* Alternative transposition model, but small field width packs are problematic. */
#if 0
void s2p_ideal(const std::unique_ptr<KernelBuilder> & iBuilder, Value * input[], Value * output[], cc::BitNumbering basisNumbering) {
    Value * hi_nybble[4];
    Value * lo_nybble[4];
    for (unsigned i = 0; i<4; i++) {
        Value * s0 = input[2*i];
        Value * s1 = input[2*i+1];
        hi_nybble[i] = iBuilder->hsimd_packh(8, s0, s1);
        lo_nybble[i] = iBuilder->hsimd_packl(8, s0, s1);
    }
    Value * pair76[2];
    Value * pair54[2];
    Value * pair32[2];
    Value * pair10[2];
    for (unsigned i = 0; i<2; i++) {
        pair76[i] = iBuilder->hsimd_packh(4, hi_nybble[2*i], hi_nybble[2*i+1]);
        pair54[i] = iBuilder->hsimd_packl(4, hi_nybble[2*i], hi_nybble[2*i+1]);
        pair32[i] = iBuilder->hsimd_packh(4, lo_nybble[2*i], lo_nybble[2*i+1]);
        pair10[i] = iBuilder->hsimd_packl(4, lo_nybble[2*i], lo_nybble[2*i+1]);
    }
    if (basisNumbering == cc::BitNumbering::LittleEndian) {
        output[7] = iBuilder->hsimd_packh(2, pair76[0], pair76[1]);
        output[6] = iBuilder->hsimd_packl(2, pair76[0], pair76[1]);
        output[5] = iBuilder->hsimd_packh(2, pair54[0], pair54[1]);
        output[4] = iBuilder->hsimd_packl(2, pair54[0], pair54[1]);
        output[3] = iBuilder->hsimd_packh(2, pair32[0], pair32[1]);
        output[2] = iBuilder->hsimd_packl(2, pair32[0], pair32[1]);
        output[1] = iBuilder->hsimd_packh(2, pair10[0], pair10[1]);
        output[0] = iBuilder->hsimd_packl(2, pair10[0], pair10[1]);
    } else {
        output[0] = iBuilder->hsimd_packh(2, pair76[0], pair76[1]);
        output[1] = iBuilder->hsimd_packl(2, pair76[0], pair76[1]);
        output[2] = iBuilder->hsimd_packh(2, pair54[0], pair54[1]);
        output[3] = iBuilder->hsimd_packl(2, pair54[0], pair54[1]);
        output[4] = iBuilder->hsimd_packh(2, pair32[0], pair32[1]);
        output[5] = iBuilder->hsimd_packl(2, pair32[0], pair32[1]);
        output[6] = iBuilder->hsimd_packh(2, pair10[0], pair10[1]);
        output[7] = iBuilder->hsimd_packl(2, pair10[0], pair10[1]);
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
    s2p(kb, bytepack, basisbits, mBasisSetNumbering);
    for (unsigned i = 0; i < mNumOfStreams; ++i) {
        kb->storeOutputStreamBlock("basisBits", kb->getInt32(i), blockOffsetPhi, basisbits[i]);
    }
    Value * nextBlk = kb->CreateAdd(blockOffsetPhi, kb->getSize(1));
    blockOffsetPhi->addIncoming(nextBlk, processBlock);
    Value * moreToDo = kb->CreateICmpNE(nextBlk, numOfBlocks);
    kb->CreateCondBr(moreToDo, processBlock, s2pDone);
    kb->SetInsertPoint(s2pDone);
}

S2PKernel::S2PKernel(const std::unique_ptr<KernelBuilder> & b, cc::BitNumbering numbering, bool aligned, std::string prefix, unsigned numOfStreams)
    : MultiBlockKernel(aligned ? prefix + "s2p" + cc::numberingSuffix(numbering): prefix + "s2p_unaligned" + cc::numberingSuffix(numbering),
    {Binding{b->getStreamSetTy(1, 8), "byteStream", FixedRate(), Principal()}},
    {Binding{b->getStreamSetTy(numOfStreams, 1), "basisBits"}}, {}, {}, {}),
  mBasisSetNumbering(numbering),
  mAligned(aligned),
  mNumOfStreams(numOfStreams)
{
    if (!aligned) {
        mStreamSetInputs[0].addAttribute(Misaligned());
    }
}

S2PMultipleStreamsKernel::S2PMultipleStreamsKernel(
        const std::unique_ptr<kernel::KernelBuilder> & b,
        cc::BitNumbering basisNumbering,
        bool aligned,
        std::vector<unsigned> numsOfStreams)
: MultiBlockKernel(aligned ? "s2pMultipleStreams" + cc::numberingSuffix(basisNumbering): "s2p_unaligned" + cc::numberingSuffix(basisNumbering),
                   {Binding{b->getStreamSetTy(1, 8), "byteStream", FixedRate(), Principal()}},
                   {}, {}, {}, {}),
  mBasisSetNumbering(basisNumbering),
  mAligned(aligned),
  mNumsOfStreams(numsOfStreams)
{
    for (unsigned i = 0; i < numsOfStreams.size(); i++) {
        mStreamSetOutputs.push_back(Binding{b->getStreamSetTy(numsOfStreams[i], 1), "basisBits_" + std::to_string(i)});
    }
}

void S2PMultipleStreamsKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &kb,
                                                       llvm::Value *const numOfBlocks) {
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
    s2p(kb, bytepack, basisbits, mBasisSetNumbering);
    unsigned iStreamIndex = 0;
    for (unsigned i = 0; i < mNumsOfStreams.size(); i++) {
        for (unsigned j = 0; j < mNumsOfStreams[i]; j++) {
            kb->storeOutputStreamBlock("basisBits_" + std::to_string(i), kb->getInt32(j), blockOffsetPhi, basisbits[iStreamIndex]);
            iStreamIndex++;
        }
    }

    Value * nextBlk = kb->CreateAdd(blockOffsetPhi, kb->getSize(1));
    blockOffsetPhi->addIncoming(nextBlk, processBlock);
    Value * moreToDo = kb->CreateICmpNE(nextBlk, numOfBlocks);
    kb->CreateCondBr(moreToDo, processBlock, s2pDone);
    kb->SetInsertPoint(s2pDone);
}


S2P_21Kernel::S2P_21Kernel(const std::unique_ptr<KernelBuilder> & b, cc::BitNumbering numbering)
: MultiBlockKernel("s2p_21" + cc::numberingSuffix(numbering),
                   {Binding{b->getStreamSetTy(1, 32), "codeUnitStream", FixedRate(), Principal()}},
                   {Binding{b->getStreamSetTy(21, 1), "basisBits"}}, {}, {}, {}),
    mBasisSetNumbering(numbering) {
}

void S2P_21Kernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, Value * const numOfBlocks) {
    BasicBlock * entry = kb->GetInsertBlock();
    BasicBlock * processBlock = kb->CreateBasicBlock("s2p21_loop");
    BasicBlock * s2pDone = kb->CreateBasicBlock("s2p21_done");
    Constant * const ZERO = kb->getSize(0);
    
    kb->CreateBr(processBlock);
    kb->SetInsertPoint(processBlock);
    PHINode * blockOffsetPhi = kb->CreatePHI(kb->getSizeTy(), 2); // block offset from the base block, e.g. 0, 1, 2, ...
    blockOffsetPhi->addIncoming(ZERO, entry);
    Value * u32byte0[8];
    Value * u32byte1[8];
    Value * u32byte2[8];
    for (unsigned i = 0; i < 8; i++) {
        Value * UTF32units[4];
        for (unsigned j = 0; j < 4; j++) {
            UTF32units[j] = kb->loadInputStreamPack("codeUnitStream", ZERO, kb->getInt32(4*i + j), blockOffsetPhi);
        }
        Value * u32lo16_0 = kb->hsimd_packl(32, UTF32units[0], UTF32units[1]);
        Value * u32lo16_1 = kb->hsimd_packl(32, UTF32units[2], UTF32units[3]);
        Value * u32hi16_0 = kb->hsimd_packh(32, UTF32units[0], UTF32units[1]);
        Value * u32hi16_1 = kb->hsimd_packh(32, UTF32units[2], UTF32units[3]);
        u32byte0[i] = kb->hsimd_packl(16, u32lo16_0, u32lo16_1);
        u32byte1[i] = kb->hsimd_packh(16, u32lo16_0, u32lo16_1);
        u32byte2[i] = kb->hsimd_packl(16, u32hi16_0, u32hi16_1);
    #ifdef VALIDATE_U32
        //  Validation should ensure that none of the high 11 bits are
        //  set for any UTF-32 code unit.   We simply combine the bits
        //  of code units together with bitwise-or, and then perform a
        //  single check at the end.
        u32_check = simd_or(u32_check, simd_or(u32hi16_0, u32hi16_1));
    #endif
    }
    Value * basisbits[24];
    s2p(kb, u32byte0, basisbits, cc::BitNumbering::LittleEndian);
    s2p(kb, u32byte1, &basisbits[8], cc::BitNumbering::LittleEndian);
    s2p(kb, u32byte2, &basisbits[16], cc::BitNumbering::LittleEndian);
    for (unsigned i = 0; i < 21; ++i) {
        const unsigned bitIdx = mBasisSetNumbering == cc::BitNumbering::LittleEndian ? i : 21 - i;
        kb->storeOutputStreamBlock("basisBits", kb->getInt32(i), blockOffsetPhi, basisbits[bitIdx]);
    }
    Value * nextBlk = kb->CreateAdd(blockOffsetPhi, kb->getSize(1));
    blockOffsetPhi->addIncoming(nextBlk, processBlock);
    Value * moreToDo = kb->CreateICmpNE(nextBlk, numOfBlocks);
    kb->CreateCondBr(moreToDo, processBlock, s2pDone);
    kb->SetInsertPoint(s2pDone);
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
        const unsigned bitIndex = mBasisSetNumbering == cc::BitNumbering::LittleEndian ? bit : mCodeUnitWidth-1-bit;
        pb->createAssign(pb->createExtract(getOutputStreamVar("basisBits"), pb->getInteger(bitIndex)), streamSet[steps][bit]);
    }
}

S2P_PabloKernel::S2P_PabloKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const unsigned codeUnitWidth, cc::BitNumbering numbering)
: PabloKernel(b, "s2p_pablo" + std::to_string(codeUnitWidth) + cc::numberingSuffix(numbering),
    {Binding{b->getStreamSetTy(1, codeUnitWidth), "codeUnitStream"}},
    {Binding{b->getStreamSetTy(codeUnitWidth, 1), "basisBits"}}),
  mCodeUnitWidth(codeUnitWidth),
  mBasisSetNumbering(numbering) {
}


}
