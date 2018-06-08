
/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <llvm/IR/Module.h>
#include <llvm/IR/Function.h>
#include <llvm/Linker/Linker.h>
#include <llvm/Support/PrettyStackTrace.h>
#include <llvm/Support/Signals.h>
#include <llvm/Support/ManagedStatic.h>
#include <toolchain/toolchain.h>

#include <IR_Gen/idisa_target.h>
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

#include <cc/cc_compiler.h>
#include <kernels/cc_kernel.h>
#include <kernels/streamset.h>
#include <kernels/s2p_kernel.h>
#include <kernels/source_kernel.h>
#include <kernels/stdout_kernel.h>
#include <kernels/swizzle.h>

#include <kernels/kernel_builder.h>
#include <kernels/p2s_kernel.h>
#include <toolchain/cpudriver.h>
#include <iostream>
#include <fstream>
#include <kernels/deletion.h>
#include <kernels/pdep_kernel.h>
#include <kernels/bitstream_pdep_kernel.h>

namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;

static cl::OptionCategory characterDepositFlags("Command Flags", "deletion options");
static cl::opt<char> characterToBeDeposit(cl::Positional, cl::desc("<character to be depositted>"), cl::Required, cl::cat(characterDepositFlags));
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(characterDepositFlags));
static cl::opt<std::string> outputFile(cl::Positional, cl::desc("<output file>"), cl::Required, cl::cat(characterDepositFlags));
static cl::opt<bool> useSwizzledDeposit("swizzle-deposit", cl::desc("Use swizzle deletion"), cl::init(false), cl::cat(characterDepositFlags));
static cl::opt<bool> UseStreamDepositCompiler("UseStreamDepositCompiler", cl::desc("Use the StreamDepositCompiler deletion"), cl::init(false), cl::cat(characterDepositFlags));


typedef void (*MainFunctionType)(char * byte_data, size_t filesize);

/*
 * Usage:
 *     character_deposit <character to be deposit> <input file name> <output file name>
 *
 * Assume we have a input file with 6 character:
 *     qbwbeb
 * Then, the pipeline will first deleted all of character 'b' from input:
 *     qbwbeb => qwe
 * And then it will deposit the deleted result (qwe) to the original position of b:
 *     qwe => \0 q \0 w \0 e (spaces are not included)
 * and store the result to output file
 * */

StreamSetBuffer * loadBasisBits(ParabixDriver & pxDriver, Value* inputStream, Value* fileSize, int bufferBlocks) {
    auto & iBuilder = pxDriver.getBuilder();

    StreamSetBuffer * ByteStream = pxDriver.addBuffer<ExternalBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8));
    StreamSetBuffer * BasisBits = pxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8, 1), bufferBlocks);

    kernel::Kernel * sourceK = pxDriver.addKernelInstance<MemorySourceKernel>(iBuilder);
    sourceK->setInitialArguments({inputStream, fileSize});
    pxDriver.makeKernelCall(sourceK, {}, {ByteStream});
    Kernel * s2pk = pxDriver.addKernelInstance<S2PKernel>(iBuilder);
    pxDriver.makeKernelCall(s2pk, {ByteStream}, {BasisBits});

    return BasisBits;
}

StreamSetBuffer * generateSwizzledDeposit(ParabixDriver & pxDriver, StreamSetBuffer * BasisBits, int bufferBlocks) {
    auto & iBuilder = pxDriver.getBuilder();

    StreamSetBuffer * const CharacterMarkerBuffer = pxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), bufferBlocks);
    Kernel * ccK = pxDriver.addKernelInstance<ParabixCharacterClassKernelBuilder>(iBuilder, "extenders", std::vector<re::CC *>{re::makeCC(characterToBeDeposit)}, 8);
    pxDriver.makeKernelCall(ccK, {BasisBits}, {CharacterMarkerBuffer});


    StreamSetBuffer * u16Swizzle0 = pxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), bufferBlocks, 1);
    StreamSetBuffer * u16Swizzle1 = pxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(4), bufferBlocks, 1);
    Kernel * delK = pxDriver.addKernelInstance<SwizzledDeleteByPEXTkernel>(iBuilder, 8);
    pxDriver.makeKernelCall(delK, {CharacterMarkerBuffer, BasisBits}, {u16Swizzle0, u16Swizzle1});

    StreamSetBuffer * depositedSwizzle0 = pxDriver.addBuffer<DynamicBuffer>(iBuilder, iBuilder->getStreamSetTy(4), bufferBlocks, 1);
    Kernel * pdep0K = pxDriver.addKernelInstance<PDEPkernel>(iBuilder, 4, "pdep0");
    pxDriver.makeKernelCall(pdep0K, {CharacterMarkerBuffer, u16Swizzle0}, {depositedSwizzle0});


    StreamSetBuffer * depositedSwizzle1 = pxDriver.addBuffer<DynamicBuffer>(iBuilder, iBuilder->getStreamSetTy(4), bufferBlocks, 1);
    Kernel * pdep1K = pxDriver.addKernelInstance<PDEPkernel>(iBuilder, 4, "pdep1");
    pxDriver.makeKernelCall(pdep1K, {CharacterMarkerBuffer, u16Swizzle1}, {depositedSwizzle1});

    // Produce unswizzled bit streams
    StreamSetBuffer * resultbits = pxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), bufferBlocks);
    Kernel * unSwizzleK = pxDriver.addKernelInstance<SwizzleGenerator>(iBuilder, 8, 1, 2);

    pxDriver.makeKernelCall(unSwizzleK, {depositedSwizzle0, depositedSwizzle1}, {resultbits});
    return resultbits;
}

StreamSetBuffer * generateBitStreamDeposit(ParabixDriver & pxDriver, StreamSetBuffer * BasisBits, int bufferBlocks) {
    auto & iBuilder = pxDriver.getBuilder();

    StreamSetBuffer * const deletionMarker = pxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), bufferBlocks);
    Kernel * ccK1 = pxDriver.addKernelInstance<ParabixCharacterClassKernelBuilder>(iBuilder, "deletionMarker", std::vector<re::CC *>{re::subtractCC(re::makeByte(0, 255), re::makeCC(characterToBeDeposit))}, 8);
    pxDriver.makeKernelCall(ccK1, {BasisBits}, {deletionMarker});

    StreamSetBuffer * const depositMarker = pxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), bufferBlocks);
    Kernel * ccK2 = pxDriver.addKernelInstance<ParabixCharacterClassKernelBuilder>(iBuilder, "extenders", std::vector<re::CC *>{re::makeCC(characterToBeDeposit)}, 8);
    pxDriver.makeKernelCall(ccK2, {BasisBits}, {depositMarker});

    // Deletion
    StreamSetBuffer * deletedBits = pxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), bufferBlocks);
    StreamSetBuffer * deletionCounts = pxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), bufferBlocks);

    Kernel * delK = pxDriver.addKernelInstance<PEXTFieldCompressKernel>(iBuilder, 64, 8);
    pxDriver.makeKernelCall(delK, {BasisBits, deletionMarker}, {deletedBits, deletionCounts});

    StreamSetBuffer * compressedBits = pxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(8), bufferBlocks);
    Kernel * streamCompressionK = pxDriver.addKernelInstance<StreamCompressKernel>(iBuilder, 64, 8);
    pxDriver.makeKernelCall(streamCompressionK, {deletedBits, deletionCounts}, {compressedBits});

    // Deposit
    StreamSetBuffer * depositedBits = pxDriver.addBuffer<DynamicBuffer>(iBuilder, iBuilder->getStreamSetTy(8), bufferBlocks, 1);
    if (UseStreamDepositCompiler) {
        StreamDepositCompiler depositCompiler(pxDriver, 8, 0, 8, bufferBlocks);
        depositCompiler.makeCall(depositMarker, compressedBits, depositedBits);
    } else {
        Kernel * pdepK = pxDriver.addKernelInstance<BitStreamPDEPKernel>(iBuilder, 8);
        pxDriver.makeKernelCall(pdepK, {depositMarker, compressedBits}, {depositedBits});
    }
    return depositedBits;
}

int main(int argc, char *argv[]) {
    // This boilerplate provides convenient stack traces and clean LLVM exit
    // handling. It also initializes the built in support for convenient
    // command line option handling.
    sys::PrintStackTraceOnErrorSignal(argv[0]);
    llvm::PrettyStackTraceProgram X(argc, argv);
    llvm_shutdown_obj shutdown;
    codegen::ParseCommandLineOptions(argc, argv, {&characterDepositFlags, codegen::codegen_flags()});

    std::string fileName = inputFile;

    std::ifstream f(fileName, std::ios::binary | std::ios::ate);
    if (f.fail()) {
        return -1;
    }
    size_t mFilesize = f.tellg();

    boost::iostreams::mapped_file_source mappedFile;
    // Since mmap offset has to be multiples of pages, we can't use it to skip headers.
    mappedFile.open(fileName , mFilesize);

    char *fileBuffer = const_cast<char *>(mappedFile.data());

    const auto bufferBlocks = codegen::ThreadNum * codegen::SegmentSize;

    ParabixDriver pxDriver("character_deletion");
    auto & iBuilder = pxDriver.getBuilder();
    Module * M = iBuilder->getModule();
    Type * const sizeTy = iBuilder->getSizeTy();
    Type * const boolTy = iBuilder->getIntNTy(sizeof(bool) * 8);
    Type * const voidTy = iBuilder->getVoidTy();
    Type * const inputType = iBuilder->getInt8PtrTy();

    Function * const main = cast<Function>(M->getOrInsertFunction("Main", voidTy, inputType, sizeTy, sizeTy, boolTy, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();
    Value* inputStream = &*(args++);
    inputStream->setName("input");

    Value* fileSize = &*(args++);
    fileSize->setName("fileSize");

    iBuilder->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", main, 0));

    StreamSetBuffer * BasisBits = loadBasisBits(pxDriver, inputStream, fileSize, bufferBlocks);
    StreamSetBuffer * resultbits = NULL;
    if (useSwizzledDeposit) {
        resultbits = generateSwizzledDeposit(pxDriver, BasisBits, bufferBlocks);
    } else {
        resultbits = generateBitStreamDeposit(pxDriver, BasisBits, bufferBlocks);
    }

    StreamSetBuffer * const ResultBytes = pxDriver.addBuffer<StaticBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), bufferBlocks);
    Kernel * p2sK = pxDriver.addKernelInstance<P2SKernel>(iBuilder);
    pxDriver.makeKernelCall(p2sK, {resultbits}, {ResultBytes});

    Kernel * outK = pxDriver.addKernelInstance<FileSink>(iBuilder, 8);
    outK->setInitialArguments({iBuilder->GetString(outputFile)});
    pxDriver.makeKernelCall(outK, {ResultBytes}, {});

    pxDriver.generatePipelineIR();

    pxDriver.deallocateBuffers();

    iBuilder->CreateRetVoid();

    pxDriver.finalizeObject();


    auto mainFunc = reinterpret_cast<MainFunctionType>(pxDriver.getMain());


    mainFunc(fileBuffer, mFilesize);

    mappedFile.close();
    return 0;
}
