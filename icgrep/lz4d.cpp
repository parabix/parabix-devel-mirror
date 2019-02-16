/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */


#include <llvm/IR/Module.h>
#include <llvm/Linker/Linker.h>
#include <llvm/Support/PrettyStackTrace.h>
#include <llvm/Support/Signals.h>
#include <llvm/Support/ManagedStatic.h>
#include <toolchain/toolchain.h>

#include <IR_Gen/idisa_target.h>
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

#include <lz4/lz4_frame_decoder.h>
#include <cc/alphabet.h>
#include <cc/cc_compiler.h>
#include <kernels/cc_kernel.h>
#include <kernels/streamset.h>
#include <kernels/s2p_kernel.h>
#include <kernels/source_kernel.h>
#include <kernels/stdout_kernel.h>
#include <kernels/lz4/lz4_index_decoder.h>
#include <kernels/lz4/lz4_bytestream_decoder.h>

#include <kernels/kernel_builder.h>
#include <kernels/pipeline_builder.h>
#include <toolchain/cpudriver.h>
#include <iostream>
#include <llvm/Support/raw_ostream.h>
namespace re { class CC; }

using namespace llvm;
using namespace kernel;

static cl::OptionCategory lz4dFlags("Command Flags", "lz4d options");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(lz4dFlags));
static cl::opt<std::string> outputFile(cl::Positional, cl::desc("<output file>"), cl::Required, cl::cat(lz4dFlags));
static cl::opt<bool> overwriteOutput("f", cl::desc("Overwrite existing output file."), cl::init(false), cl::cat(lz4dFlags));

typedef void (*MainFunctionType)(char * byte_data, size_t filesize, bool hasBlockChecksum, const char * outputFileName);

MainFunctionType generatePipeline(CPUDriver & pxDriver) {
    auto & b = pxDriver.getBuilder();

    Type * const sizeTy = b->getSizeTy();
    Type * const boolTy = b->getIntNTy(sizeof(bool) * 8);
    Type * const int8PtrTy = b->getInt8PtrTy();
    

    auto P = pxDriver.makePipeline({Binding{int8PtrTy, "input"}, Binding{sizeTy, "fileSize"}, Binding{boolTy, "hasBlockChecksum"}, Binding{int8PtrTy, "outputFileName"}});

    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);
    StreamSet * const BasisBits = P->CreateStreamSet(8, 1);
    StreamSet * const Extenders = P->CreateStreamSet(1, 1);
    StreamSet * const LiteralIndexes = P->CreateStreamSet(2, 32);
    StreamSet * const MatchIndexes = P->CreateStreamSet(2, 32);
    StreamSet * const DecompressedByteStream = P->CreateStreamSet(1, 8);


    Scalar * const inputStream = P->getInputScalar("input");
    Scalar * const fileSize = P->getInputScalar("fileSize");

    P->CreateKernelCall<MemorySourceKernel>(inputStream, fileSize, ByteStream);

    // Input stream is not aligned due to the offset.
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);

    P->CreateKernelCall<ParabixCharacterClassKernelBuilder>("extenders", std::vector<re::CC *>{re::makeCC(0xFF)}, BasisBits, Extenders);

    Scalar * const hasBlockChecksum = P->getInputScalar("hasBlockChecksum");

    P->CreateKernelCall<LZ4IndexDecoderKernel>(hasBlockChecksum, ByteStream, Extenders, LiteralIndexes, MatchIndexes);

    P->CreateKernelCall<LZ4ByteStreamDecoderKernel>(LiteralIndexes, MatchIndexes, ByteStream, DecompressedByteStream);

    Scalar * outputFileName = P->getInputScalar("outputFileName");
    P->CreateKernelCall<FileSink>(outputFileName, DecompressedByteStream);

    return reinterpret_cast<MainFunctionType>(P->compile());
}

int main(int argc, char *argv[]) {
    // This boilerplate provides convenient stack traces and clean LLVM exit
    // handling. It also initializes the built in support for convenient
    // command line option handling.
    sys::PrintStackTraceOnErrorSignal(argv[0]);
    llvm::PrettyStackTraceProgram X(argc, argv);
    llvm_shutdown_obj shutdown;
    codegen::ParseCommandLineOptions(argc, argv, {&lz4dFlags, codegen::codegen_flags()});
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
    CPUDriver pxDriver("lz4d");
    auto main = generatePipeline(pxDriver);

    main(fileBuffer, lz4Frame.getBlocksLength(), lz4Frame.hasBlockChecksum(), outputFile.c_str());

    mappedFile.close();
    return 0;
}
