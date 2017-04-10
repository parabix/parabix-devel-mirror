/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <IR_Gen/idisa_builder.h>                  // for IDISA_Builder
#include <IR_Gen/idisa_target.h>                   // for GetIDISA_Builder
#include <cc/cc_compiler.h>                        // for CC_Compiler
#include <kernels/deletion.h>                      // for DeletionKernel
#include <kernels/swizzle.h>                      // for DeletionKernel
#include <kernels/mmap_kernel.h>                   // for MMapSourceKernel
#include <kernels/p2s_kernel.h>                    // for P2S16KernelWithCom...
#include <kernels/s2p_kernel.h>                    // for S2PKernel
#include <kernels/stdout_kernel.h>                 // for StdOutKernel
#include <llvm/ExecutionEngine/ExecutionEngine.h>  // for ExecutionEngine
#include <llvm/IR/Function.h>                      // for Function, Function...
#include <llvm/IR/Module.h>                        // for Module
#include <llvm/IR/Verifier.h>                      // for verifyModule
#include <llvm/Support/CommandLine.h>              // for ParseCommandLineOp...
#include <llvm/Support/Debug.h>                    // for dbgs
#include <pablo/pablo_kernel.h>                    // for PabloKernel
#include <pablo/pablo_toolchain.h>                 // for pablo_function_passes
#include <pablo/pe_zeroes.h>
#include <kernels/toolchain.h>
#include <boost/iostreams/device/mapped_file.hpp>  // for mapped_file_source
#include <boost/filesystem.hpp>
#include <boost/interprocess/anonymous_shared_memory.hpp>
#include "kernels/streamset.h"                     // for CircularBuffer
#include <kernels/pipeline.h>
#include "llvm/ADT/StringRef.h"                    // for StringRef
#include "llvm/IR/CallingConv.h"                   // for ::C
#include "llvm/IR/DerivedTypes.h"                  // for ArrayType, Pointer...
#include "llvm/IR/LLVMContext.h"                   // for LLVMContext
#include "llvm/IR/Value.h"                         // for Value
#include "llvm/Support/Compiler.h"                 // for LLVM_UNLIKELY
#include <pablo/builder.hpp>                       // for PabloBuilder
#include <iostream>

using namespace pablo;
using namespace kernel;
using namespace parabix;
using namespace llvm;

static cl::OptionCategory u8u16Options("u8u16 Options", "Transcoding control options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(u8u16Options));
static cl::opt<std::string> outputFile(cl::Positional, cl::desc("<output file>"),  cl::Required, cl::cat(u8u16Options));
static cl::opt<bool> enableAVXdel("enable-AVX-deletion", cl::desc("Enable AVX2 deletion algorithms."), cl::cat(u8u16Options));
static cl::opt<bool> mMapBuffering("mmap-buffering", cl::desc("Enable mmap buffering."), cl::cat(u8u16Options));
static cl::opt<bool> memAlignBuffering("memalign-buffering", cl::desc("Enable posix_memalign buffering."), cl::cat(u8u16Options));


void u8u16_pablo(PabloKernel * kernel) {
    //  input: 8 basis bit streams
    
    const auto u8bitSet = kernel->getInputStreamVar("u8bit");
    
    //  output: 16 u8-indexed streams, + delmask stream + error stream
    
    cc::CC_Compiler ccc(kernel, u8bitSet);
    
    PabloBuilder & main = ccc.getBuilder();
    const auto u8_bits = ccc.getBasisBits();
    
    Zeroes * zeroes = main.createZeroes();

    // Outputs
    Var * u16_hi[8];
    for (int i = 0; i < 8; ++i) {
        u16_hi[i] = main.createVar("u16_hi" + std::to_string(i), zeroes);
    }
    Var * u16_lo[8];
    for (int i = 0; i < 8; ++i) {
        u16_lo[i] = main.createVar("u16_lo" + std::to_string(i), zeroes);
    }
    Var * delmask = main.createVar("delmask", zeroes);
    Var * error_mask = main.createVar("error_mask", zeroes);

    // The logic for processing non-ASCII bytes will be embedded within an if-hierarchy.
    PabloAST * nonASCII = ccc.compileCC(re::makeCC(0x80, 0xFF));
    
    // Builder for the if statement handling all non-ASCII logic
    PabloBuilder nAb = PabloBuilder::Create(main);
    // Bits 3 through 7 of a 2-byte prefix are data bits, needed to
    // produce the UTF-16 code unit data ..., 
    PabloAST * bit3a1 = nAb.createAdvance(u8_bits[3], 1);
    PabloAST * bit4a1 = nAb.createAdvance(u8_bits[4], 1);
    PabloAST * bit5a1 = nAb.createAdvance(u8_bits[5], 1);
    PabloAST * bit6a1 = nAb.createAdvance(u8_bits[6], 1);
    PabloAST * bit7a1 = nAb.createAdvance(u8_bits[7], 1);
    
    // Entry condition for 3 or 4 byte sequences: we have a prefix byte in the range 0xE0-0xFF.
    PabloAST * pfx34 = ccc.compileCC(re::makeCC(0xE0, 0xFF), nAb);
    // Builder for the if statement handling all logic for 3- and 4-byte sequences.
    PabloBuilder p34b = PabloBuilder::Create(nAb);
    // Bits 4 through 7 of a 3-byte prefix are data bits.  They must be moved
    // to the final position of the 3-byte sequence.
    PabloAST * bit2a1 = p34b.createAdvance(u8_bits[2], 1);
    PabloAST * bit4a2 = p34b.createAdvance(bit4a1, 1);
    PabloAST * bit5a2 = p34b.createAdvance(bit5a1, 1);
    PabloAST * bit6a2 = p34b.createAdvance(bit6a1, 1);
    PabloAST * bit7a2 = p34b.createAdvance(bit7a1, 1);


    Var * const u8scope32 = nAb.createVar("u8scope32", zeroes);
    Var * const u8scope33 = nAb.createVar("u8scope33", zeroes);
    Var * const u8scope44 = nAb.createVar("u8scope44", zeroes);

    //
    // Logic for 4-byte UTF-8 sequences
    //
    // Entry condition  or 4 byte sequences: we have a prefix byte in the range 0xF0-0xFF.
    PabloAST * pfx4 = ccc.compileCC(re::makeCC(0xF0, 0xFF), p34b);
    // Builder for the if statement handling all logic for 4-byte sequences only.
    PabloBuilder p4b = PabloBuilder::Create(p34b);
    // Illegal 4-byte sequences
    PabloAST * F0 = ccc.compileCC(re::makeCC(0xF0), p4b);
    PabloAST * F4 = ccc.compileCC(re::makeCC(0xF4), p4b);
    PabloAST * F0_err = p4b.createAnd(p4b.createAdvance(F0, 1), ccc.compileCC(re::makeCC(0x80, 0x8F), p4b));
    PabloAST * F4_err = p4b.createAnd(p4b.createAdvance(F4, 1), ccc.compileCC(re::makeCC(0x90, 0xBF), p4b));
    PabloAST * F5_FF = ccc.compileCC(re::makeCC(0xF5, 0xFF), p4b);

    Var * FX_err = p34b.createVar("FX_err", zeroes);
    p4b.createAssign(FX_err, p4b.createOr(F5_FF, p4b.createOr(F0_err, F4_err)));
    //
    // 4-byte prefixes have a scope that extends over the next 3 bytes.

    Var * u8scope42 = p34b.createVar("u8scope42", zeroes);
    Var * u8scope43 = p34b.createVar("u8scope43", zeroes);

    p4b.createAssign(u8scope42, p4b.createAdvance(pfx4, 1));
    p4b.createAssign(u8scope43, p4b.createAdvance(u8scope42, 1));
    p4b.createAssign(u8scope44, p4b.createAdvance(u8scope43, 1));
    //
    
    //  From the 4-byte sequence 11110abc 10defghi 10jklmno 10pqrstu,
    //  we must calculate the value abcde - 1 to produce the bit values
    //  for u16_hi6, hi7, lo0, lo1 at the scope43 position.
    Var * s43_lo0 = nAb.createVar("scope43_lo0", zeroes);
    Var * s43_lo1 = nAb.createVar("scope43_lo1", zeroes);
    Var * s43_hi6 = nAb.createVar("scope43_hi6", zeroes);
    Var * s43_hi7 = nAb.createVar("scope43_hi7", zeroes);

    Var * s43_lo2 = main.createVar("scope43_lo2", zeroes);
    Var * s43_lo3 = main.createVar("scope43_lo3", zeroes);
    Var * s43_lo4 = main.createVar("scope43_lo4", zeroes);
    Var * s43_lo5 = main.createVar("scope43_lo5", zeroes);
    Var * s43_lo6 = main.createVar("scope43_lo6", zeroes);
    Var * s43_lo7 = main.createVar("scope43_lo7", zeroes);

    p4b.createAssign(s43_lo1, p4b.createAnd(u8scope43, p4b.createNot(bit3a1)));           // e - 1
    p4b.createAssign(s43_lo0, p4b.createAnd(u8scope43, p4b.createXor(bit2a1, s43_lo1)));  // d - borrow
    PabloAST * brw1 = p4b.createAnd(s43_lo1, p4b.createNot(bit2a1));
    p4b.createAssign(s43_hi7, p4b.createAnd(u8scope43, p4b.createXor(bit7a2, brw1)));     // c - borrow
    PabloAST * brw2 = p4b.createAnd(brw1, p4b.createNot(bit7a2));
    p4b.createAssign(s43_hi6, p4b.createAnd(u8scope43, p4b.createXor(bit6a2, brw2)));     // b - borrow
    //
    p4b.createAssign(s43_lo2, p4b.createAnd(u8scope43, bit4a1));
    p4b.createAssign(s43_lo3, p4b.createAnd(u8scope43, bit5a1));
    p4b.createAssign(s43_lo4, p4b.createAnd(u8scope43, bit6a1));
    p4b.createAssign(s43_lo5, p4b.createAnd(u8scope43, bit7a1));
    p4b.createAssign(s43_lo6, p4b.createAnd(u8scope43, u8_bits[2]));
    p4b.createAssign(s43_lo7, p4b.createAnd(u8scope43, u8_bits[3]));
    //
    //
    p34b.createIf(pfx4, p4b);
    //
    // Combined logic for 3 and 4 byte sequences
    //
    PabloAST * pfx3 = ccc.compileCC(re::makeCC(0xE0, 0xEF), p34b);

    p34b.createAssign(u8scope32, p34b.createAdvance(pfx3, 1));
    p34b.createAssign(u8scope33, p34b.createAdvance(u8scope32, 1));

    // Illegal 3-byte sequences
    PabloAST * E0 = ccc.compileCC(re::makeCC(0xE0), p34b);
    PabloAST * ED = ccc.compileCC(re::makeCC(0xED), p34b);
    PabloAST * E0_err = p34b.createAnd(p34b.createAdvance(E0, 1), ccc.compileCC(re::makeCC(0x80, 0x9F), p34b));
    PabloAST * ED_err = p34b.createAnd(p34b.createAdvance(ED, 1), ccc.compileCC(re::makeCC(0xA0, 0xBF), p34b));
    Var * EX_FX_err = nAb.createVar("EX_FX_err", zeroes);

    p34b.createAssign(EX_FX_err, p34b.createOr(p34b.createOr(E0_err, ED_err), FX_err));
    // Two surrogate UTF-16 units are computed at the 3rd and 4th positions of 4-byte sequences.
    PabloAST * surrogate = p34b.createOr(u8scope43, u8scope44);
    
    Var * p34del = nAb.createVar("p34del", zeroes);
    p34b.createAssign(p34del, p34b.createOr(u8scope32, u8scope42));


    // The high 5 bits of the UTF-16 code unit are only nonzero for 3 and 4-byte
    // UTF-8 sequences.
    p34b.createAssign(u16_hi[0], p34b.createOr(p34b.createAnd(u8scope33, bit4a2), surrogate));
    p34b.createAssign(u16_hi[1], p34b.createOr(p34b.createAnd(u8scope33, bit5a2), surrogate));
    p34b.createAssign(u16_hi[2], p34b.createAnd(u8scope33, bit6a2));
    p34b.createAssign(u16_hi[3], p34b.createOr(p34b.createAnd(u8scope33, bit7a2), surrogate));
    p34b.createAssign(u16_hi[4], p34b.createOr(p34b.createAnd(u8scope33, bit2a1), surrogate));
    
    //
    nAb.createIf(pfx34, p34b);
    //
    // Combined logic for 2, 3 and 4 byte sequences
    //

    Var * u8lastscope = main.createVar("u8lastscope", zeroes);

    PabloAST * pfx2 = ccc.compileCC(re::makeCC(0xC0, 0xDF), nAb);
    PabloAST * u8scope22 = nAb.createAdvance(pfx2, 1);
    nAb.createAssign(u8lastscope, nAb.createOr(u8scope22, nAb.createOr(u8scope33, u8scope44)));
    PabloAST * u8anyscope = nAb.createOr(u8lastscope, p34del);

    PabloAST * C0_C1_err = ccc.compileCC(re::makeCC(0xC0, 0xC1), nAb);
    PabloAST * scope_suffix_mismatch = nAb.createXor(u8anyscope, ccc.compileCC(re::makeCC(0x80, 0xBF), nAb));
    nAb.createAssign(error_mask, nAb.createOr(scope_suffix_mismatch, nAb.createOr(C0_C1_err, EX_FX_err)));
    nAb.createAssign(delmask, nAb.createOr(p34del, ccc.compileCC(re::makeCC(0xC0, 0xFF), nAb)));
    
    // The low 3 bits of the high byte of the UTF-16 code unit as well as the high bit of the
    // low byte are only nonzero for 2, 3 and 4 byte sequences.
    nAb.createAssign(u16_hi[5], nAb.createOr(nAb.createAnd(u8lastscope, bit3a1), u8scope44));
    nAb.createAssign(u16_hi[6], nAb.createOr(nAb.createAnd(u8lastscope, bit4a1), s43_hi6));
    nAb.createAssign(u16_hi[7], nAb.createOr(nAb.createAnd(u8lastscope, bit5a1), s43_hi7));
    nAb.createAssign(u16_lo[0], nAb.createOr(nAb.createAnd(u8lastscope, bit6a1), s43_lo0));

    Var * p234_lo1 = main.createVar("p234_lo1", zeroes);

    nAb.createAssign(p234_lo1, nAb.createOr(nAb.createAnd(u8lastscope, bit7a1), s43_lo1));

    main.createIf(nonASCII, nAb);
    //
    //
    PabloAST * ASCII = ccc.compileCC(re::makeCC(0x0, 0x7F));
    PabloAST * last_byte = main.createOr(ASCII, u8lastscope);
    main.createAssign(u16_lo[1], main.createOr(main.createAnd(ASCII, u8_bits[1]), p234_lo1));
    main.createAssign(u16_lo[2], main.createOr(main.createAnd(last_byte, u8_bits[2]), s43_lo2));
    main.createAssign(u16_lo[3], main.createOr(main.createAnd(last_byte, u8_bits[3]), s43_lo3));
    main.createAssign(u16_lo[4], main.createOr(main.createAnd(last_byte, u8_bits[4]), s43_lo4));
    main.createAssign(u16_lo[5], main.createOr(main.createAnd(last_byte, u8_bits[5]), s43_lo5));
    main.createAssign(u16_lo[6], main.createOr(main.createAnd(last_byte, u8_bits[6]), s43_lo6));
    main.createAssign(u16_lo[7], main.createOr(main.createAnd(last_byte, u8_bits[7]), s43_lo7));
    
    Var * output = kernel->getOutputStreamVar("u16bit");
    Var * delmask_out = kernel->getOutputStreamVar("delMask");
    Var * error_mask_out = kernel->getOutputStreamVar("errMask");
    
    for (unsigned i = 0; i < 8; i++) {
        main.createAssign(main.createExtract(output, i), u16_hi[i]);
    }
    for (unsigned i = 0; i < 8; i++) {
        main.createAssign(main.createExtract(output, i + 8), u16_lo[i]);
    }
    main.createAssign(main.createExtract(delmask_out, main.getInteger(0)), delmask);
    main.createAssign(main.createExtract(error_mask_out,  main.getInteger(0)), error_mask);

    pablo_function_passes(kernel);
}

void u8u16PipelineAVX2Gen(ParabixDriver & pxDriver) {

    IDISA::IDISA_Builder * iBuilder = pxDriver.getIDISA_Builder();
    Module * mod = iBuilder->getModule();
    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::ThreadNum+1;

    assert (iBuilder);

    Type * const size_ty = iBuilder->getSizeTy();
    Type * const voidTy = iBuilder->getVoidTy();
    Type * const bitBlockType = iBuilder->getBitBlockType();
    Type * const inputType = ArrayType::get(ArrayType::get(bitBlockType, 8), 1)->getPointerTo();
    Type * const outputType = ArrayType::get(ArrayType::get(bitBlockType, 16), 1)->getPointerTo();
    
    Function * const main = cast<Function>(mod->getOrInsertFunction("Main", voidTy, inputType, outputType, size_ty, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();
    
    Value * const inputStream = &*(args++);
    inputStream->setName("inputStream");
    Value * const outputStream = &*(args++);
    outputStream->setName("outputStream");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");

    // File data from mmap
    ExternalFileBuffer ByteStream(iBuilder, iBuilder->getStreamSetTy(1, 8));

    MMapSourceKernel mmapK(iBuilder, segmentSize); 
    mmapK.setInitialArguments({fileSize});
    pxDriver.addKernelCall(mmapK, {}, {&ByteStream});

    // Transposed bits from s2p
    CircularBuffer BasisBits(iBuilder, iBuilder->getStreamSetTy(8), segmentSize * bufferSegments);

    S2PKernel s2pk(iBuilder);
    pxDriver.addKernelCall(s2pk, {&ByteStream}, {&BasisBits});
    
    // Calculate UTF-16 data bits through bitwise logic on u8-indexed streams.
    CircularBuffer U8u16Bits(iBuilder, iBuilder->getStreamSetTy(16), segmentSize * bufferSegments);
    CircularBuffer DelMask(iBuilder, iBuilder->getStreamSetTy(), segmentSize * bufferSegments);
    CircularBuffer ErrorMask(iBuilder, iBuilder->getStreamSetTy(), segmentSize * bufferSegments);

    PabloKernel u8u16k(iBuilder, "u8u16",
                       {Binding{iBuilder->getStreamSetTy(8, 1), "u8bit"}},
                       {Binding{iBuilder->getStreamSetTy(16, 1), "u16bit"},
                           Binding{iBuilder->getStreamSetTy(1, 1), "delMask"},
                           Binding{iBuilder->getStreamSetTy(1, 1), "errMask"}}, {});
    
    u8u16_pablo(&u8u16k);
    pxDriver.addKernelCall(u8u16k, {&BasisBits}, {&U8u16Bits, &DelMask, &ErrorMask});

    // Apply a deletion algorithm to discard all but the final position of the UTF-8
    // sequences for each UTF-16 code unit. Swizzle the results.
    CircularBuffer SwizzleFields0(iBuilder, iBuilder->getStreamSetTy(4), segmentSize * bufferSegments);
    CircularBuffer SwizzleFields1(iBuilder, iBuilder->getStreamSetTy(4), segmentSize * bufferSegments);
    CircularBuffer SwizzleFields2(iBuilder, iBuilder->getStreamSetTy(4), segmentSize * bufferSegments);
    CircularBuffer SwizzleFields3(iBuilder, iBuilder->getStreamSetTy(4), segmentSize * bufferSegments);
    CircularBuffer DeletionCounts(iBuilder, iBuilder->getStreamSetTy(), segmentSize * bufferSegments);

    DeleteByPEXTkernel delK(iBuilder, 64, 16, true);
    pxDriver.addKernelCall(delK, {&U8u16Bits, &DelMask}, {&SwizzleFields0, &SwizzleFields1, &SwizzleFields2, &SwizzleFields3, &DeletionCounts});

    //  Produce fully compressed swizzled UTF-16 bit streams
    SwizzledCopybackBuffer u16Swizzle0(iBuilder, iBuilder->getStreamSetTy(4), segmentSize * (bufferSegments+2), 1);
    SwizzledCopybackBuffer u16Swizzle1(iBuilder, iBuilder->getStreamSetTy(4), segmentSize * (bufferSegments+2), 1);
    SwizzledCopybackBuffer u16Swizzle2(iBuilder, iBuilder->getStreamSetTy(4), segmentSize * (bufferSegments+2), 1);
    SwizzledCopybackBuffer u16Swizzle3(iBuilder, iBuilder->getStreamSetTy(4), segmentSize * (bufferSegments+2), 1);

    SwizzledBitstreamCompressByCount compressK(iBuilder, 16);
    pxDriver.addKernelCall(compressK, {&DeletionCounts, &SwizzleFields0, &SwizzleFields1, &SwizzleFields2, &SwizzleFields3},
                             {&u16Swizzle0, &u16Swizzle1, &u16Swizzle2, &u16Swizzle3});
 
    // Produce unswizzled UTF-16 bit streams
    CircularBuffer u16bits(iBuilder, iBuilder->getStreamSetTy(16), segmentSize * bufferSegments);
    SwizzleGenerator unSwizzleK(iBuilder, 16, 1, 4);
    unSwizzleK.setName("unswizzle");
    pxDriver.addKernelCall(unSwizzleK, {&u16Swizzle0, &u16Swizzle1, &u16Swizzle2, &u16Swizzle3}, {&u16bits});
    
    // Different choices for the output buffer depending on chosen option.
    ExternalFileBuffer U16external(iBuilder, iBuilder->getStreamSetTy(1, 16));
    CircularBuffer U16out(iBuilder, iBuilder->getStreamSetTy(1, 16), segmentSize * bufferSegments);

    P2S16Kernel p2sk(iBuilder);

    //P2S16KernelWithCompressedOutput p2sk(iBuilder);

    FileSink outK(iBuilder, 16);
    if (mMapBuffering || memAlignBuffering) {
        pxDriver.addKernelCall(p2sk, {&u16bits}, {&U16external});
        pxDriver.addKernelCall(outK, {&U16external}, {});
    } else {
        pxDriver.addKernelCall(p2sk, {&u16bits}, {&U16out});
        pxDriver.addKernelCall(outK, {&U16out}, {});
    }
    
    iBuilder->SetInsertPoint(BasicBlock::Create(mod->getContext(), "entry", main,0));

    ByteStream.setStreamSetBuffer(inputStream);
    BasisBits.allocateBuffer();
    U8u16Bits.allocateBuffer();
    DelMask.allocateBuffer();
    ErrorMask.allocateBuffer();
    DeletionCounts.allocateBuffer();
    SwizzleFields0.allocateBuffer();
    SwizzleFields1.allocateBuffer();
    SwizzleFields2.allocateBuffer();
    SwizzleFields3.allocateBuffer();
    u16Swizzle0.allocateBuffer();
    u16Swizzle1.allocateBuffer();
    u16Swizzle2.allocateBuffer();
    u16Swizzle3.allocateBuffer();
    u16bits.allocateBuffer();

    if (mMapBuffering || memAlignBuffering) {
        U16external.setStreamSetBuffer(outputStream);
    } else {
        U16out.allocateBuffer();
    }
    Value * fName = iBuilder->CreatePointerCast(iBuilder->CreateGlobalString(outputFile.c_str()), iBuilder->getInt8PtrTy());
    outK.setInitialArguments({fName});

    pxDriver.generatePipelineIR();

    iBuilder->CreateRetVoid();

    pxDriver.linkAndFinalize();
}

void u8u16PipelineGen(ParabixDriver & pxDriver) {
    
    IDISA::IDISA_Builder * iBuilder = pxDriver.getIDISA_Builder();
    Module * mod = iBuilder->getModule();
    
    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::ThreadNum+1;
    
    assert (iBuilder);
    
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const voidTy = iBuilder->getVoidTy();
    Type * const bitBlockType = iBuilder->getBitBlockType();
    Type * const inputType = ArrayType::get(ArrayType::get(bitBlockType, 8), 1)->getPointerTo();
    Type * const outputType = ArrayType::get(ArrayType::get(bitBlockType, 16), 1)->getPointerTo();
    
    Function * const main = cast<Function>(mod->getOrInsertFunction("Main", voidTy, inputType, outputType, size_ty, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();
    
    Value * const inputStream = &*(args++);
    inputStream->setName("inputStream");
    Value * const outputStream = &*(args++);
    outputStream->setName("outputStream");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");
    
    ExternalFileBuffer ByteStream(iBuilder, iBuilder->getStreamSetTy(1, 8));
    
    CircularBuffer BasisBits(iBuilder, iBuilder->getStreamSetTy(8), segmentSize * bufferSegments);
    
    CircularBuffer U8u16Bits(iBuilder, iBuilder->getStreamSetTy(16), segmentSize * bufferSegments);
    CircularBuffer DelMask(iBuilder, iBuilder->getStreamSetTy(), segmentSize * bufferSegments);
    CircularBuffer ErrorMask(iBuilder, iBuilder->getStreamSetTy(), segmentSize * bufferSegments);
    
    CircularBuffer U16Bits(iBuilder, iBuilder->getStreamSetTy(16), segmentSize * bufferSegments);
    
    CircularBuffer DeletionCounts(iBuilder, iBuilder->getStreamSetTy(), segmentSize * bufferSegments);
    
    // Different choices for the output buffer depending on chosen option.
    ExternalFileBuffer U16external(iBuilder, iBuilder->getStreamSetTy(1, 16));
    CircularCopybackBuffer U16out(iBuilder, iBuilder->getStreamSetTy(1, 16), segmentSize * bufferSegments, 1 /*overflow block*/);
    
    MMapSourceKernel mmapK(iBuilder, segmentSize); 
    mmapK.setInitialArguments({fileSize});
    pxDriver.addKernelCall(mmapK, {}, {&ByteStream});
    
    S2PKernel s2pk(iBuilder);
    pxDriver.addKernelCall(s2pk, {&ByteStream}, {&BasisBits});
    
    PabloKernel u8u16k(iBuilder, "u8u16",
                       {Binding{iBuilder->getStreamSetTy(8, 1), "u8bit"}},
                       {Binding{iBuilder->getStreamSetTy(16, 1), "u16bit"},
                           Binding{iBuilder->getStreamSetTy(1, 1), "delMask"},
                           Binding{iBuilder->getStreamSetTy(1, 1), "errMask"}}, {});
    
    u8u16_pablo(&u8u16k);
    pxDriver.addKernelCall(u8u16k, {&BasisBits}, {&U8u16Bits, &DelMask, &ErrorMask});
    
    DeletionKernel delK(iBuilder, iBuilder->getBitBlockWidth()/16, 16);
    pxDriver.addKernelCall(delK, {&U8u16Bits, &DelMask}, {&U16Bits, &DeletionCounts});
    
    P2S16KernelWithCompressedOutput p2sk(iBuilder);
    
    FileSink outK(iBuilder, 16);
    if (mMapBuffering || memAlignBuffering) {
        pxDriver.addKernelCall(p2sk, {&U16Bits, &DeletionCounts}, {&U16external});
        pxDriver.addKernelCall(outK, {&U16external}, {});
    } else {
        pxDriver.addKernelCall(p2sk, {&U16Bits, &DeletionCounts}, {&U16out});
        pxDriver.addKernelCall(outK, {&U16out}, {});
    }
    iBuilder->SetInsertPoint(BasicBlock::Create(mod->getContext(), "entry", main,0));
    
    ByteStream.setStreamSetBuffer(inputStream);
    BasisBits.allocateBuffer();
    U8u16Bits.allocateBuffer();
    DelMask.allocateBuffer();
    ErrorMask.allocateBuffer();
    U16Bits.allocateBuffer();
    DeletionCounts.allocateBuffer();
    if (mMapBuffering || memAlignBuffering) {
        U16external.setStreamSetBuffer(outputStream);
    } else {
        U16out.allocateBuffer();
    }
    Value * fName = iBuilder->CreatePointerCast(iBuilder->CreateGlobalString(outputFile.c_str()), iBuilder->getInt8PtrTy());
    outK.setInitialArguments({fName});
    
    pxDriver.generatePipelineIR();

    
    iBuilder->CreateRetVoid();

    pxDriver.linkAndFinalize();
}



typedef void (*u8u16FunctionType)(char * byte_data, char * output_data, size_t filesize);

u8u16FunctionType u8u16CodeGen(void) {
    LLVMContext TheContext;                            
    Module * M = new Module("u8u16", TheContext);
    IDISA::IDISA_Builder * idb = IDISA::GetIDISA_Builder(M);
    ParabixDriver pxDriver(idb);
    
    if (enableAVXdel && AVX2_available() && codegen::BlockSize==256) {
        //u8u16PipelineAVX2(M, idb)
        u8u16PipelineAVX2Gen(pxDriver);
    }
    else{
        //u8u16Pipeline(M, idb);
        u8u16PipelineGen(pxDriver);
    }
    u8u16FunctionType main = reinterpret_cast<u8u16FunctionType>(pxDriver.getPointerToMain());

    delete idb;
    return main;
}

void u8u16(u8u16FunctionType fn_ptr, const std::string & fileName) {

    const boost::filesystem::path file(fileName);
    if (exists(file)) {
        if (is_directory(file)) {
            return;
        }
    } else {
        std::cerr << "Error: cannot open " << fileName << " for processing. Skipped.\n";
        return;
    }
    
    size_t fileSize = file_size(file);
    boost::iostreams::mapped_file_source input;

    char * fileBuffer = nullptr;
    if (fileSize) {
        try {
            input.open(fileName);
            fileBuffer = const_cast<char *>(input.data());
        } catch (std::exception & e) {
            throw std::runtime_error("Boost mmap error: " + fileName + ": " + e.what());
        }        
    }

    if (mMapBuffering) {
        boost::interprocess::mapped_region outputBuffer(boost::interprocess::anonymous_shared_memory(2*fileSize));
        outputBuffer.advise(boost::interprocess::mapped_region::advice_willneed);
        outputBuffer.advise(boost::interprocess::mapped_region::advice_sequential);
        fn_ptr(fileBuffer, static_cast<char*>(outputBuffer.get_address()), fileSize);
    } else if (memAlignBuffering) {
        char * outputBuffer;
        const auto r = posix_memalign(reinterpret_cast<void **>(&outputBuffer), 32, 2*fileSize);
        if (LLVM_UNLIKELY(r != 0)) {
            throw std::runtime_error("posix_memalign failed with return code " + std::to_string(r));
        }
        fn_ptr(fileBuffer, outputBuffer, fileSize);
        free(reinterpret_cast<void *>(outputBuffer));
    } else {
        /* No external output buffer */
        fn_ptr(fileBuffer, nullptr, fileSize);
    }
    input.close();
    
}

int main(int argc, char *argv[]) {
    AddParabixVersionPrinter();
    cl::HideUnrelatedOptions(ArrayRef<const cl::OptionCategory *>{&u8u16Options, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    cl::ParseCommandLineOptions(argc, argv);
    u8u16(u8u16CodeGen(), inputFile);
    return 0;
}

                       
