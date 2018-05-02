
/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <llvm/IR/Module.h>
#include <llvm/IR/Function.h>
#include <llvm/Linker/Linker.h>
#include <llvm/Support/CommandLine.h>
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

namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;

static cl::OptionCategory lz4dFlags("Command Flags", "deletion options");
static cl::opt<char> characterToBeDeleted(cl::Positional, cl::desc("<character to be deletion>"), cl::Required, cl::cat(lz4dFlags));
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(lz4dFlags));


typedef void (*MainFunctionType)(char * byte_data, size_t filesize);

/*
 * Usage:
 *   character_deletion <character to be deleted> <input file name>
 * It will delete the character from the input file and then print the output to stdout
 * */

int main(int argc, char *argv[]) {
    // This boilerplate provides convenient stack traces and clean LLVM exit
    // handling. It also initializes the built in support for convenient
    // command line option handling.
    sys::PrintStackTraceOnErrorSignal(argv[0]);
    llvm::PrettyStackTraceProgram X(argc, argv);
    llvm_shutdown_obj shutdown;
    codegen::ParseCommandLineOptions(argc, argv, {&lz4dFlags, codegen::codegen_flags()});

    const std::string fileName = inputFile;

    std::ifstream f(fileName, std::ios::binary | std::ios::ate);
    if (f.fail()) {
        return -1;
    }
    size_t mFilesize = f.tellg();

    boost::iostreams::mapped_file_source mappedFile;
    // Since mmap offset has to be multiples of pages, we can't use it to skip headers.
    mappedFile.open(fileName , mFilesize);

    char *fileBuffer = const_cast<char *>(mappedFile.data());

    const int inputBufferBlocks = codegen::BufferSegments * codegen::ThreadNum * 16;

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

    // GeneratePipeline
    StreamSetBuffer * ByteStream = pxDriver.addBuffer<SourceBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8));
    StreamSetBuffer * BasisBits = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(8, 1), inputBufferBlocks);

    kernel::Kernel * sourceK = pxDriver.addKernelInstance<MemorySourceKernel>(iBuilder);
    sourceK->setInitialArguments({inputStream, fileSize});
    pxDriver.makeKernelCall(sourceK, {}, {ByteStream});

    Kernel * s2pk = pxDriver.addKernelInstance<S2PKernel>(iBuilder, /*aligned = */ true);
    pxDriver.makeKernelCall(s2pk, {ByteStream}, {BasisBits});

    StreamSetBuffer * const CharacterMarkerBuffer = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), inputBufferBlocks);
    Kernel * ccK = pxDriver.addKernelInstance<ParabixCharacterClassKernelBuilder>(iBuilder, "extenders", std::vector<re::CC *>{re::makeCC(characterToBeDeleted)}, 8);
    pxDriver.makeKernelCall(ccK, {BasisBits}, {CharacterMarkerBuffer});


    StreamSetBuffer * u16Swizzle0 = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(4), inputBufferBlocks, 1);
    StreamSetBuffer * u16Swizzle1 = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(4), inputBufferBlocks, 1);
    Kernel * delK = pxDriver.addKernelInstance<SwizzledDeleteByPEXTkernel>(iBuilder, 8);
    pxDriver.makeKernelCall(delK, {CharacterMarkerBuffer, BasisBits}, {u16Swizzle0, u16Swizzle1});

    // Produce unswizzled bit streams
    StreamSetBuffer * extractedbits = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(8), inputBufferBlocks);
    Kernel * unSwizzleK = pxDriver.addKernelInstance<SwizzleGenerator>(iBuilder, 8, 1, 2);

    pxDriver.makeKernelCall(unSwizzleK, {u16Swizzle0, u16Swizzle1}, {extractedbits});

    StreamSetBuffer * const DecompressedByteStream = pxDriver.addBuffer<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), inputBufferBlocks);
    Kernel * p2sK = pxDriver.addKernelInstance<P2SKernel>(iBuilder);
    pxDriver.makeKernelCall(p2sK, {extractedbits}, {DecompressedByteStream});

    // --------------------------------------------------------
    // End

    Kernel * outK = pxDriver.addKernelInstance<StdOutKernel>(iBuilder, 8);
    pxDriver.makeKernelCall(outK, {DecompressedByteStream}, {});

    pxDriver.generatePipelineIR();

    pxDriver.deallocateBuffers();

    iBuilder->CreateRetVoid();

    pxDriver.finalizeObject();


    auto mainFunc = reinterpret_cast<MainFunctionType>(pxDriver.getMain());

    mainFunc(fileBuffer, mFilesize);

    mappedFile.close();
    return 0;
}
