/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>

#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/MCJIT.h>
#include <llvm/IRReader/IRReader.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Support/Debug.h>

#include <llvm/Support/CommandLine.h>

#include <toolchain.h>
#include <re/re_cc.h>
#include <cc/cc_compiler.h>
#include <pablo/pablo_toolchain.h>
#include <pablo/pablo_kernel.h>
#include <IR_Gen/idisa_builder.h>
#include <IR_Gen/idisa_target.h>
#include <kernels/pipeline.h>
#include <kernels/mmap_kernel.h>
#include <kernels/interface.h>
#include <kernels/kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/p2s_kernel.h>
#include <kernels/deletion.h>
#include <kernels/stdout_kernel.h>

// mmap system
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
#include <boost/interprocess/anonymous_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <fcntl.h>
static cl::OptionCategory u8u16Options("u8u16 Options",
                                            "Transcoding control options.");

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore, cl::cat(u8u16Options));

static cl::opt<bool> segmentPipelineParallel("enable-segment-pipeline-parallel", cl::desc("Enable multithreading with segment pipeline parallelism."), cl::cat(u8u16Options));
static cl::opt<bool> mMapBuffering("mmap-buffering", cl::desc("Enable mmap buffering."), cl::cat(u8u16Options));
static cl::opt<bool> memAlignBuffering("memalign-buffering", cl::desc("Enable posix_memalign buffering."), cl::cat(u8u16Options));


using namespace pablo;
using namespace kernel;
using namespace parabix;

void u8u16_pablo(PabloKernel * kernel) {
    //  input: 8 basis bit streams
    //  output: 16 u8-indexed streams, + delmask stream + error stream

    cc::CC_Compiler ccc(kernel);
    
    PabloBuilder & main = ccc.getBuilder();
    const auto u8_bits = ccc.getBasisBits();

    PabloAST * zeroes = main.createZeroes();

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
    
    Var * output = kernel->addOutput("output", kernel->getStreamSetTy(16));
    Var * delmask_out = kernel->addOutput("delmask_out", kernel->getStreamSetTy());
    Var * error_mask_out = kernel->addOutput("error_mask_out", kernel->getStreamSetTy());
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

Function * u8u16Pipeline(Module * mod, IDISA::IDISA_Builder * iBuilder) {

    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments;
    
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
    LinearCopybackBuffer U16out(iBuilder, iBuilder->getStreamSetTy(16, 16), segmentSize * bufferSegments + 2);

    MMapSourceKernel mmapK(iBuilder, iBuilder->getStride()); 
    mmapK.generateKernel({}, {&ByteStream});
    mmapK.setInitialArguments({fileSize});
    
    S2PKernel s2pk(iBuilder);

    s2pk.generateKernel({&ByteStream}, {&BasisBits});

    PabloKernel u8u16k(iBuilder, "u8u16");

    u8u16_pablo(&u8u16k);

    u8u16k.generateKernel({&BasisBits}, {&U8u16Bits, &DelMask, &ErrorMask});

    DeletionKernel delK(iBuilder, iBuilder->getBitBlockWidth()/16, 16);
    delK.generateKernel({&U8u16Bits, &DelMask}, {&U16Bits, &DeletionCounts});

    p2s_16Kernel_withCompressedOutput p2sk(iBuilder);

    StdOutKernel stdoutK(iBuilder, 16);

    if (mMapBuffering || memAlignBuffering) {
        p2sk.generateKernel({&U16Bits, &DeletionCounts}, {&U16external});
        stdoutK.generateKernel({&U16external}, {});
    } else {
        p2sk.generateKernel({&U16Bits, &DeletionCounts}, {&U16out});
        stdoutK.generateKernel({&U16out}, {});
    }


    iBuilder->SetInsertPoint(BasicBlock::Create(mod->getContext(), "entry", main,0));

    ByteStream.setStreamSetBuffer(inputStream, fileSize);
    BasisBits.allocateBuffer();
    U8u16Bits.allocateBuffer();
    DelMask.allocateBuffer();
    ErrorMask.allocateBuffer();
    U16Bits.allocateBuffer();
    DeletionCounts.allocateBuffer();
    if (mMapBuffering || memAlignBuffering) {
        U16external.setEmptyBuffer(outputStream);
    } else {
        U16out.allocateBuffer();
    }

    if (segmentPipelineParallel){
        generateSegmentParallelPipeline(iBuilder, {&mmapK, &s2pk, &u8u16k, &delK, &p2sk, &stdoutK});
    } else {
        generatePipelineLoop(iBuilder, {&mmapK, &s2pk, &u8u16k, &delK, &p2sk, &stdoutK});
    }

    iBuilder->CreateRetVoid();
    return main;
}





typedef void (*u8u16FunctionType)(char * byte_data, char * output_data, size_t filesize);

static ExecutionEngine * u8u16Engine = nullptr;

u8u16FunctionType u8u16CodeGen(void) {
    LLVMContext TheContext;                            
    Module * M = new Module("u8u16", TheContext);
    IDISA::IDISA_Builder * idb = IDISA::GetIDISA_Builder(M);

    llvm::Function * main_IR = u8u16Pipeline(M, idb);
    
    verifyModule(*M, &dbgs());
    u8u16Engine = JIT_to_ExecutionEngine(M);   
    u8u16Engine->finalizeObject();

    delete idb;
    return reinterpret_cast<u8u16FunctionType>(u8u16Engine->getPointerToFunction(main_IR));
}

void u8u16(u8u16FunctionType fn_ptr, const std::string & fileName) {
    std::string mFileName = fileName;
    size_t fileSize;
    char * fileBuffer;
    
    const boost::filesystem::path file(mFileName);
    if (exists(file)) {
        if (is_directory(file)) {
            return;
        }
    } else {
        std::cerr << "Error: cannot open " << mFileName << " for processing. Skipped.\n";
        return;
    }
    
    fileSize = file_size(file);
    boost::iostreams::mapped_file_source mFile;
    if (fileSize == 0) {
        fileBuffer = nullptr;
    }
    else {
        try {
            mFile.open(mFileName);
        } catch (std::exception &e) {
            std::cerr << "Error: Boost mmap of " << mFileName << ": " << e.what() << std::endl;
            return;
        }
        fileBuffer = const_cast<char *>(mFile.data());
    }

    if (mMapBuffering) {
        boost::interprocess::mapped_region outputBuffer(boost::interprocess::anonymous_shared_memory(2*fileSize));
        outputBuffer.advise(boost::interprocess::mapped_region::advice_willneed);
        outputBuffer.advise(boost::interprocess::mapped_region::advice_sequential);
        fn_ptr(fileBuffer, static_cast<char*>(outputBuffer.get_address()), fileSize);
    }
    else if (memAlignBuffering) {
        char * outputBuffer;
        const auto r = posix_memalign(reinterpret_cast<void **>(&outputBuffer), 32, 2*fileSize);
        if (LLVM_UNLIKELY(r != 0)) {
            throw std::runtime_error("posix_memalign failed with return code " + std::to_string(r));
        }
        fn_ptr(fileBuffer, outputBuffer, fileSize);
        free(reinterpret_cast<void *>(outputBuffer));
    }
    else {
        /* No external output buffer */
        fn_ptr(fileBuffer, nullptr, fileSize);
    }
    mFile.close();
    
}


int main(int argc, char *argv[]) {
    cl::HideUnrelatedOptions(ArrayRef<const cl::OptionCategory *>{&u8u16Options, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    cl::ParseCommandLineOptions(argc, argv);

    u8u16FunctionType fn_ptr = u8u16CodeGen();

    for (unsigned i = 0; i != inputFiles.size(); ++i) {
        u8u16(fn_ptr, inputFiles[i]);
    }

    return 0;
}

                       
