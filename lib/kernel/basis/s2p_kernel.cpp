/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/basis/s2p_kernel.h>
#include <kernel/core/callback.h>
#include <kernel/core/kernel_builder.h>
#include <pablo/pabloAST.h>
#include <pablo/builder.hpp>
#include <pablo/pe_pack.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace kernel {

using BuilderRef = Kernel::BuilderRef;

const int PACK_LANES = 2;
void s2p_step(BuilderRef iBuilder, Value * s0, Value * s1, Value * hi_mask, unsigned shift, Value * &p0, Value * &p1) {
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

void s2p(BuilderRef iBuilder, Value * input[], Value * output[]) {
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
    s2p_step(iBuilder, bit44440000[0], bit44440000[1], iBuilder->simd_himask(8), 4, output[4], output[0]);
    s2p_step(iBuilder, bit55551111[0], bit55551111[1], iBuilder->simd_himask(8), 4, output[5], output[1]);
    s2p_step(iBuilder, bit66662222[0], bit66662222[1], iBuilder->simd_himask(8), 4, output[6], output[2]);
    s2p_step(iBuilder, bit77773333[0], bit77773333[1], iBuilder->simd_himask(8), 4, output[7], output[3]);
}

/* Alternative transposition model, but small field width packs are problematic. */
#if 0
void s2p_ideal(BuilderRef iBuilder, Value * input[], Value * output[]) {
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
    output[7] = iBuilder->hsimd_packh(2, pair76[0], pair76[1]);
    output[6] = iBuilder->hsimd_packl(2, pair76[0], pair76[1]);
    output[5] = iBuilder->hsimd_packh(2, pair54[0], pair54[1]);
    output[4] = iBuilder->hsimd_packl(2, pair54[0], pair54[1]);
    output[3] = iBuilder->hsimd_packh(2, pair32[0], pair32[1]);
    output[2] = iBuilder->hsimd_packl(2, pair32[0], pair32[1]);
    output[1] = iBuilder->hsimd_packh(2, pair10[0], pair10[1]);
    output[0] = iBuilder->hsimd_packl(2, pair10[0], pair10[1]);
}
#endif

void S2PKernel::generateMultiBlockLogic(BuilderRef kb, Value * const numOfBlocks) {
    Module * m = kb->getModule();
    DataLayout DL(m);
    IntegerType * const intPtrTy = DL.getIntPtrType(kb->getContext());
    PointerType * const voidPtrTy = kb->getVoidPtrTy();
    BasicBlock * entry = kb->GetInsertBlock();
    BasicBlock * s2pLoop = kb->CreateBasicBlock("s2pLoop");
    BasicBlock * s2pFinalize = kb->CreateBasicBlock("s2pFinalize");
    Constant * const ZERO = kb->getSize(0);
    // Declarations for AbortOnNull mode:
    PHINode * nullCheckPhi = nullptr;
    Value * nonNullSoFar = nullptr;

    kb->CreateBr(s2pLoop);

    kb->SetInsertPoint(s2pLoop);
    PHINode * blockOffsetPhi = kb->CreatePHI(kb->getSizeTy(), 2); // block offset from the base block, e.g. 0, 1, 2, ...
    blockOffsetPhi->addIncoming(ZERO, entry);
    if (mAbortOnNull) {
        nullCheckPhi = kb->CreatePHI(kb->getBitBlockType(), 2);
        nullCheckPhi->addIncoming(kb->allOnes(), entry);
    }
    Value * bytepack[8];
    for (unsigned i = 0; i < 8; i++) {
        bytepack[i] = kb->loadInputStreamPack("byteStream", ZERO, kb->getInt32(i), blockOffsetPhi);
    }
    Value * basisbits[8];
    s2p(kb, bytepack, basisbits);
    for (unsigned i = 0; i < mNumOfStreams; ++i) {
        kb->storeOutputStreamBlock("basisBits", kb->getInt32(i), blockOffsetPhi, basisbits[i]);
    }
    if (mAbortOnNull) {
        Value * nonNull = kb->simd_or(kb->simd_or(kb->simd_or(basisbits[0], basisbits[1]),
                                                  kb->simd_or(basisbits[2], basisbits[3])),
                                      kb->simd_or(kb->simd_or(basisbits[4], basisbits[5]),
                                                  kb->simd_or(basisbits[6], basisbits[7])));
        nonNullSoFar = kb->simd_and(nonNull, nullCheckPhi);
        nullCheckPhi->addIncoming(nonNullSoFar, s2pLoop);
    }
    Value * nextBlk = kb->CreateAdd(blockOffsetPhi, kb->getSize(1));
    blockOffsetPhi->addIncoming(nextBlk, s2pLoop);
    Value * moreToDo = kb->CreateICmpNE(nextBlk, numOfBlocks);

    kb->CreateCondBr(moreToDo, s2pLoop, s2pFinalize);

    kb->SetInsertPoint(s2pFinalize);
    //  s2p is complete, except for null byte check.
    if (mAbortOnNull) {
        BasicBlock * nullByteDetected = kb->CreateBasicBlock("nullByteDetected");
        BasicBlock * nullInFileDetected = kb->CreateBasicBlock("nullInFileDetected");
        BasicBlock * s2pExit = kb->CreateBasicBlock("s2pExit");
        Value * itemsToDo = kb->getAccessibleItemCount("byteStream");
        Value * anyNull = kb->bitblock_any(kb->simd_not(nonNullSoFar));
        kb->CreateCondBr(anyNull, nullByteDetected, s2pExit);

        kb->SetInsertPoint(nullByteDetected);
        // A null byte has been detected, determine its position and whether it is past EOF.
        Value * byteStreamBasePtr = kb->getInputStreamBlockPtr("byteStream", ZERO, ZERO);
        Value * ptrToNull = kb->CreateMemChr(kb->CreatePointerCast(byteStreamBasePtr, voidPtrTy), kb->getInt32(0), itemsToDo);
        Value * nullInFile = kb->CreateICmpNE(ptrToNull, ConstantPointerNull::get(cast<PointerType>(ptrToNull->getType())));
        kb->CreateCondBr(nullInFile, nullInFileDetected, s2pExit);

        kb->SetInsertPoint(nullInFileDetected);
        // A null byte has been located within the file; set the termination code and call the signal handler.
        Value * firstNull = kb->CreatePtrToInt(ptrToNull, intPtrTy);
        Value * startOfStride = kb->CreatePtrToInt(byteStreamBasePtr, intPtrTy);
        Value * itemsBeforeNull = kb->CreateZExtOrTrunc(kb->CreateSub(firstNull, startOfStride), itemsToDo->getType());
        Value * producedCount = kb->CreateAdd(kb->getProducedItemCount("basisBits"), itemsBeforeNull);
        kb->setProducedItemCount("basisBits", producedCount);
        kb->setFatalTerminationSignal();
        Function * const dispatcher = m->getFunction("signal_dispatcher"); assert (dispatcher);
        Value * handler = kb->getScalarField("handler_address");
        kb->CreateCall(dispatcher, {handler, ConstantInt::get(kb->getInt32Ty(), NULL_SIGNAL)});
        kb->CreateBr(s2pExit);

        kb->SetInsertPoint(s2pExit);
    }
}

Bindings S2PKernel::makeOutputBindings(StreamSet * const BasisBits, bool abortOnNull) {
    if (abortOnNull) {
        return {Binding("basisBits", BasisBits)};
    } else {
        return {Binding("basisBits", BasisBits)};
    }
}

Bindings S2PKernel::makeInputScalarBindings(Scalar * signalNullObject) {
    if (signalNullObject) {
        return {Binding{"handler_address", signalNullObject}};
    } else {
        return {};
    }
}

S2PKernel::S2PKernel(BuilderRef b,
                     StreamSet * const codeUnitStream,
                     StreamSet * const BasisBits,
                     Scalar * signalNullObject)
: MultiBlockKernel(b, (signalNullObject ? "s2pa" : "s2p") + std::to_string(BasisBits->getNumElements())
, {Binding{"byteStream", codeUnitStream, FixedRate(), Principal()}}
, makeOutputBindings(BasisBits, signalNullObject)
, makeInputScalarBindings(signalNullObject), {}, {})
, mAbortOnNull(signalNullObject != nullptr)
, mNumOfStreams(BasisBits->getNumElements()) {
    assert (codeUnitStream->getFieldWidth() == BasisBits->getNumElements());
    if (mAbortOnNull) {
        addAttribute(CanTerminateEarly());
        addAttribute(MayFatallyTerminate());
    }
}

inline std::string makeMultiS2PName(const StreamSets & outputStreams, const bool aligned) {
    std::string buffer;
    raw_string_ostream out(buffer);
    out << "s2p";
    for (unsigned i = 0; i < outputStreams.size(); ++i) {
        if (i) out << ".";
        out << outputStreams[i]->getNumElements();
    }
    out << (aligned ? "a" : "u");
    out.flush();
    return buffer;
}

S2PMultipleStreamsKernel::S2PMultipleStreamsKernel(BuilderRef b,
        StreamSet * codeUnitStream,
        const StreamSets & outputStreams,
        const bool aligned)
: MultiBlockKernel(b, makeMultiS2PName(outputStreams, aligned),
// input
{Binding{"byteStream", codeUnitStream}},
{}, {}, {}, {}),
mAligned(aligned) {
    for (unsigned i = 0; i < outputStreams.size(); i++) {
        mOutputStreamSets.emplace_back("basisBits_" + std::to_string(i), outputStreams[i]);
    }
}

void S2PMultipleStreamsKernel::generateMultiBlockLogic(BuilderRef b, Value *const numOfBlocks) {
    BasicBlock * entry = b->GetInsertBlock();
    BasicBlock * processBlock = b->CreateBasicBlock("processBlock");
    BasicBlock * s2pDone = b->CreateBasicBlock("s2pDone");
    Constant * const ZERO = b->getSize(0);

    b->CreateBr(processBlock);

    b->SetInsertPoint(processBlock);
    PHINode * blockOffsetPhi = b->CreatePHI(b->getSizeTy(), 2); // block offset from the base block, e.g. 0, 1, 2, ...
    blockOffsetPhi->addIncoming(ZERO, entry);

    Value * bytepack[8];
    for (unsigned i = 0; i < 8; i++) {
        if (mAligned) {
            bytepack[i] = b->loadInputStreamPack("byteStream", ZERO, b->getInt32(i), blockOffsetPhi);
        } else {
            Value * ptr = b->getInputStreamPackPtr("byteStream", ZERO, b->getInt32(i), blockOffsetPhi);
            // CreateLoad defaults to aligned here, so we need to force the alignment to 1 byte.
            bytepack[i] = b->CreateAlignedLoad(ptr, 1);
        }
    }
    Value * basisbits[8];
    s2p(b, bytepack, basisbits);

    unsigned k = 0;
    for (unsigned i = 0; i < getNumOfStreamOutputs(); ++i) {
        const auto m = getOutputStreamSet(i)->getNumElements();
        for (unsigned j = 0; j < m; j++) {
            b->storeOutputStreamBlock("basisBits_" + std::to_string(i), b->getInt32(j), blockOffsetPhi, basisbits[k++]);
        }
    }

    Value * nextBlk = b->CreateAdd(blockOffsetPhi, b->getSize(1));
    blockOffsetPhi->addIncoming(nextBlk, processBlock);
    Value * moreToDo = b->CreateICmpNE(nextBlk, numOfBlocks);
    b->CreateCondBr(moreToDo, processBlock, s2pDone);
    b->SetInsertPoint(s2pDone);
}


S2P_21Kernel::S2P_21Kernel(BuilderRef b, StreamSet * const codeUnitStream, StreamSet * const BasisBits)
: MultiBlockKernel(b, "s2p_21",
{Binding{"codeUnitStream", codeUnitStream, FixedRate(), Principal()}},
    {Binding{"basisBits", BasisBits}}, {}, {}, {})  {}

void S2P_21Kernel::generateMultiBlockLogic(BuilderRef kb, Value * const numOfBlocks) {
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
            UTF32units[j] = kb->loadInputStreamPack("codeUnitStream", ZERO, kb->getInt32(4 * i + j), blockOffsetPhi);
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
    s2p(kb, u32byte0, basisbits);
    s2p(kb, u32byte1, &basisbits[8]);
    s2p(kb, u32byte2, &basisbits[16]);
    for (unsigned i = 0; i < 21; ++i) {
        kb->storeOutputStreamBlock("basisBits", kb->getInt32(i), blockOffsetPhi, basisbits[i]);
    }
    Value * nextBlk = kb->CreateAdd(blockOffsetPhi, kb->getSize(1));
    BasicBlock * const processBlockExit = kb->GetInsertBlock();
    blockOffsetPhi->addIncoming(nextBlk, processBlockExit);
    Value * moreToDo = kb->CreateICmpNE(nextBlk, numOfBlocks);
    kb->CreateCondBr(moreToDo, processBlock, s2pDone);
    kb->SetInsertPoint(s2pDone);
}

void S2P_PabloKernel::generatePabloMethod() {
    pablo::PabloBlock * const pb = getEntryScope();
    const unsigned steps = std::log2(mCodeUnitWidth);
    SmallVector<std::vector<PabloAST *>, 8> streamSet(steps + 1);
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
        pb->createAssign(pb->createExtract(getOutputStreamVar("basisBits"), pb->getInteger(bit)), streamSet[steps][bit]);
    }
}

S2P_PabloKernel::S2P_PabloKernel(BuilderRef b, StreamSet * const codeUnitStream, StreamSet * const BasisBits)
: PabloKernel(b, "s2p_pablo" + std::to_string(codeUnitStream->getFieldWidth()),
// input
{Binding{"codeUnitStream", codeUnitStream}},
// output
{Binding{"basisBits", BasisBits}}),
mCodeUnitWidth(codeUnitStream->getFieldWidth()) {
    assert (codeUnitStream->getFieldWidth() == BasisBits->getNumElements());
}


}

