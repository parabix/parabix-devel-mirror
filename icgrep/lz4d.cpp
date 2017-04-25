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
#include <IR_Gen/idisa_builder.h>
#include <IR_Gen/idisa_target.h>
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

#include <lz4FrameDecoder.h>
#include <cc/cc_compiler.h>
#include <toolchain/toolchain.h>
#include <kernels/cc_kernel.h>
#include <kernels/streamset.h>
#include <kernels/s2p_kernel.h>
#include <kernels/stdin_kernel.h>
#include <kernels/stdout_kernel.h>
#include <kernels/mmap_kernel.h>
#include <kernels/lz4_index_decoder.h>
#include <kernels/lz4_bytestream_decoder.h>

#include <string>
#include <iostream>
namespace re { class CC; }

using namespace llvm;
using namespace parabix;
using namespace kernel;

static cl::OptionCategory lz4dFlags("Command Flags", "lz4d options");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(lz4dFlags));
static cl::opt<std::string> outputFile(cl::Positional, cl::desc("<output file>"), cl::Required, cl::cat(lz4dFlags));
static cl::opt<bool> overwriteOutput("f", cl::desc("Overwrite existing output file."), cl::init(false), cl::cat(lz4dFlags));


typedef void (*MainFunctionType)(char * byte_data, size_t filesize, bool hasBlockChecksum);

void generatePipeline(ParabixDriver & pxDriver) {
    IDISA::IDISA_Builder * iBuilder = pxDriver.getIDISA_Builder();
    Module * M = iBuilder->getModule();

    Type * const size_ty = iBuilder->getSizeTy();
    Type * const bool_ty = iBuilder->getIntNTy(sizeof(bool) * 8);
    Type * const voidTy = iBuilder->getVoidTy();
    Type * const inputType = iBuilder->getInt8PtrTy();
    
    Function * const main = cast<Function>(M->getOrInsertFunction("Main", voidTy, inputType, size_ty, bool_ty, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();
    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");
    Value * const hasBlockChecksum = &*(args++);
    hasBlockChecksum->setName("hasBlockChecksum");

    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments * codegen::ThreadNum;
    // Output buffer should be at least one whole LZ4 block (4MB) large in case of uncompressed blocks.
    // And the size (in bytes) also needs to be a power of two.
    const unsigned decompressBufBlocks = 4U * 1024 * 1024 / codegen::BlockSize;

    iBuilder->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", main, 0));

    StreamSetBuffer * const ByteStream = pxDriver.addBuffer(make_unique<SourceFileBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8)));
    StreamSetBuffer * const BasisBits = pxDriver.addBuffer(make_unique<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(8, 1), segmentSize * bufferSegments));
    StreamSetBuffer * const Extenders = pxDriver.addBuffer(make_unique<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 1), segmentSize * bufferSegments));
    StreamSetBuffer * const LiteralIndexes = pxDriver.addBuffer(make_unique<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(2, 32), segmentSize * bufferSegments));
    StreamSetBuffer * const MatchIndexes = pxDriver.addBuffer(make_unique<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(2, 32), segmentSize * bufferSegments));
    StreamSetBuffer * const DecompressedByteStream = pxDriver.addBuffer(make_unique<CircularBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), decompressBufBlocks));

    
    kernel::KernelBuilder * sourceK = pxDriver.addKernelInstance(make_unique<kernel::FileSourceKernel>(iBuilder, iBuilder->getInt8PtrTy(), segmentSize));
    sourceK->setInitialArguments({inputStream, fileSize});
    pxDriver.makeKernelCall(sourceK, {}, {ByteStream});

    // Input stream is not aligned due to the offset.
    KernelBuilder * s2pk = pxDriver.addKernelInstance(make_unique<S2PKernel>(iBuilder, /*aligned = */ false));
    pxDriver.makeKernelCall(s2pk, {ByteStream}, {BasisBits});
    
    KernelBuilder * extenderK = pxDriver.addKernelInstance(make_unique<ParabixCharacterClassKernelBuilder>(iBuilder, "extenders", std::vector<re::CC *>{re::makeCC(0xFF)}, 8));
    pxDriver.makeKernelCall(extenderK, {BasisBits}, {Extenders});

    KernelBuilder * lz4iK = pxDriver.addKernelInstance(make_unique<LZ4IndexDecoderKernel>(iBuilder));
    lz4iK->setInitialArguments({iBuilder->CreateTrunc(hasBlockChecksum, iBuilder->getInt1Ty())});
    pxDriver.makeKernelCall(lz4iK, {ByteStream, Extenders}, {LiteralIndexes, MatchIndexes});

    KernelBuilder * lz4bK = pxDriver.addKernelInstance(make_unique<LZ4ByteStreamDecoderKernel>(iBuilder, decompressBufBlocks * codegen::BlockSize));
    pxDriver.makeKernelCall(lz4bK, {LiteralIndexes, MatchIndexes, ByteStream}, {DecompressedByteStream});

    KernelBuilder * outK = pxDriver.addKernelInstance(make_unique<FileSink>(iBuilder, 8));
    outK->setInitialArguments({iBuilder->CreatePointerCast(iBuilder->CreateGlobalString(outputFile), iBuilder->getInt8PtrTy())});
    pxDriver.makeKernelCall(outK, {DecompressedByteStream}, {});
 
    pxDriver.generatePipelineIR();

    iBuilder->CreateRetVoid();
 
    pxDriver.linkAndFinalize();
}


MainFunctionType codeGen() {
    ParabixDriver pxDriver("lz4d");
    generatePipeline(pxDriver);
    return reinterpret_cast<MainFunctionType>(pxDriver.getPointerToMain());
}


int main(int argc, char *argv[]) {
    // This boilerplate provides convenient stack traces and clean LLVM exit
    // handling. It also initializes the built in support for convenient
    // command line option handling.
    sys::PrintStackTraceOnErrorSignal(argv[0]);
    llvm::PrettyStackTraceProgram X(argc, argv);
    llvm_shutdown_obj shutdown;
    cl::HideUnrelatedOptions(ArrayRef<const cl::OptionCategory *>{&lz4dFlags, codegen::codegen_flags()});
    cl::ParseCommandLineOptions(argc, argv);
    std::string fileName = inputFile;
    LZ4FrameDecoder lz4Frame(fileName);
    if (!lz4Frame.isValid()) {
        errs() << "Invalid LZ4 file.\n";
        return -1;
    }

    if (boost::filesystem::exists(outputFile)) {
        if (overwriteOutput) {
            boost::filesystem::remove(outputFile);
        } else {
            errs() << outputFile + " existed. Use -f argument to overwrite.\n";
            return -1;
        }
    }

    boost::iostreams::mapped_file_source mappedFile;
    // Since mmap offset has to be multiples of pages, we can't use it to skip headers.
    mappedFile.open(fileName, lz4Frame.getBlocksLength() + lz4Frame.getBlocksStart());
    char *fileBuffer = const_cast<char *>(mappedFile.data()) + lz4Frame.getBlocksStart();

    MainFunctionType fn_ptr = codeGen();
    fn_ptr(fileBuffer, lz4Frame.getBlocksLength(), lz4Frame.hasBlockChecksum());

    mappedFile.close();
    return 0;
}
