/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */


#include <kernel/core/idisa_target.h>

#include <re/alphabet/alphabet.h>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>
#include <re/cc/cc_kernel.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/streamutils/deletion.h>
#include <kernel/basis/p2s_kernel.h>
#include <kernel/basis/s2p_kernel.h>
#include <kernel/io/source_kernel.h>
#include <kernel/io/stdout_kernel.h>
#include <kernel/streamutils/swizzle.h>
#include <kernel/streamutils/zeroextend.h>
#include <kernel/streamutils/stream_select.h>
#include <pablo/builder.hpp>
#include <pablo/boolean.h>
#include <pablo/pablo_kernel.h>
#include <toolchain/pablo_toolchain.h>
#include <pablo/bixnum/bixnum.h>
#include <pablo/pe_zeroes.h>
#include <kernel/pipeline/driver/cpudriver.h>
#include <toolchain/toolchain.h>

#include <pablo/bixnum/utf8gen.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>

using namespace pablo;
using namespace kernel;
using namespace llvm;
using namespace codegen;
using namespace re;

static cl::OptionCategory u8u16Options("u8u16 Options", "Transcoding control options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(u8u16Options));
static cl::opt<std::string> outputFile(cl::Positional, cl::desc("<output file>"), cl::cat(u8u16Options));
static cl::opt<std::string> OutputEncoding("encoding", cl::desc("Output encoding (default: UTF-16BE)"), cl::init("UTF-16BE"), cl::cat(u8u16Options));
static cl::opt<bool> enableAVXdel("enable-AVX-deletion", cl::desc("Enable AVX2 deletion algorithms."), cl::cat(u8u16Options));

static cl::opt<bool> BranchingMode("branch", cl::desc("Use Experimental branching pipeline mode"), cl::cat(u8u16Options));

inline bool useAVX2() {
    return enableAVXdel && AVX2_available() && codegen::BlockSize == 256;
}

typedef void (*u8u16FunctionType)(uint32_t fd, const char *);

// ------------------------------------------------------

u8u16FunctionType generatePipeline(CPUDriver & pxDriver, cc::ByteNumbering byteNumbering) {

    auto & b = pxDriver.getBuilder();
    auto P = pxDriver.makePipeline({Binding{b->getInt32Ty(), "inputFileDecriptor"}, Binding{b->getInt8PtrTy(), "outputFileName"}}, {});
    Scalar * fileDescriptor = P->getInputScalar("inputFileDecriptor");
    // File data from mmap
    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);

    // Transposed bits from s2p
    StreamSet * BasisBits = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);

    // Calculate UTF-16 data bits through bitwise logic on u8-indexed streams.
    StreamSet * u8bits = P->CreateStreamSet(16);
    StreamSet * selectors = P->CreateStreamSet();
    P->CreateKernelCall<U8U16Kernel>(BasisBits, u8bits, selectors);

    StreamSet * u16bits = P->CreateStreamSet(16);
    StreamSet * u16bytes = P->CreateStreamSet(1, 16);
    if (useAVX2()) {
        // Allocate space for fully compressed swizzled UTF-16 bit streams
        std::vector<StreamSet *> u16Swizzles(4);
        u16Swizzles[0] = P->CreateStreamSet(4);
        u16Swizzles[1] = P->CreateStreamSet(4);
        u16Swizzles[2] = P->CreateStreamSet(4);
        u16Swizzles[3] = P->CreateStreamSet(4);

        // Apply a deletion algorithm to discard all but the final position of the UTF-8
        // sequences (bit streams) for each UTF-16 code unit. Also compresses and swizzles the result.
        P->CreateKernelCall<SwizzledDeleteByPEXTkernel>(selectors, u8bits, u16Swizzles);
        // Produce unswizzled UTF-16 bit streams
        P->CreateKernelCall<SwizzleGenerator>(u16Swizzles, std::vector<StreamSet *>{u16bits});
        P->CreateKernelCall<P2S16Kernel>(u16bits, u16bytes);
    } else {
        const auto fieldWidth = b->getBitBlockWidth() / 16;
        P->CreateKernelCall<FieldCompressKernel>(Select(selectors, {0}),
                                                 SelectOperationList{Select(u8bits, streamutils::Range(0, 16))},
                                                 u16bits,
                                                 fieldWidth);
        P->CreateKernelCall<P2S16KernelWithCompressedOutput>(u16bits, selectors, u16bytes, byteNumbering);
    }

    Scalar * outputFileName = P->getInputScalar("outputFileName");
    P->CreateKernelCall<FileSink>(outputFileName, u16bytes);

    return reinterpret_cast<u8u16FunctionType>(P->compile());
}

// ------------------------------------------------------

void makeNonAsciiBranch(Kernel::BuilderRef b,
                        const std::unique_ptr<PipelineBuilder> & P,
                        StreamSet * const ByteStream, StreamSet * const u16bytes, cc::ByteNumbering byteNumbering) {

    // Transposed bits from s2p
    StreamSet * BasisBits = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);

    // Calculate UTF-16 data bits through bitwise logic on u8-indexed streams.
    StreamSet * u8bits = P->CreateStreamSet(16);
    StreamSet * selectors = P->CreateStreamSet();
    P->CreateKernelCall<U8U16Kernel>(BasisBits, u8bits, selectors);

    StreamSet * u16bits = P->CreateStreamSet(16);
    if (useAVX2()) {
        // Allocate space for fully compressed swizzled UTF-16 bit streams
        std::vector<StreamSet *> u16Swizzles(4);
        u16Swizzles[0] = P->CreateStreamSet(4);
        u16Swizzles[1] = P->CreateStreamSet(4);
        u16Swizzles[2] = P->CreateStreamSet(4);
        u16Swizzles[3] = P->CreateStreamSet(4);
        // Apply a deletion algorithm to discard all but the final position of the UTF-8
        // sequences (bit streams) for each UTF-16 code unit. Also compresses and swizzles the result.
        P->CreateKernelCall<SwizzledDeleteByPEXTkernel>(selectors, u8bits, u16Swizzles);
        // Produce unswizzled UTF-16 bit streams
        P->CreateKernelCall<SwizzleGenerator>(u16Swizzles, std::vector<StreamSet *>{u16bits});
        P->CreateKernelCall<P2S16Kernel>(u16bits, u16bytes);
    } else {
        const auto fieldWidth = b->getBitBlockWidth() / 16;
        P->CreateKernelCall<FieldCompressKernel>(Select(selectors, {0}),
                                                 SelectOperationList{Select(u8bits, streamutils::Range(0, 16))},
                                                 u16bits,
                                                 fieldWidth);
        P->CreateKernelCall<P2S16KernelWithCompressedOutput>(u16bits, selectors, u16bytes, byteNumbering);
    }
}

void makeAllAsciiBranch(const std::unique_ptr<PipelineBuilder> & P, StreamSet * const ByteStream, StreamSet * const u16bytes, cc::ByteNumbering byteNumbering) {
    if (byteNumbering == cc::ByteNumbering::BigEndian) {
        P->CreateKernelCall<ZeroExtend>(ByteStream, u16bytes);
    } else {
        llvm::report_fatal_error("Little endian not supported with ASCII optimization yet.");
    }
}

u8u16FunctionType generatePipeline2(CPUDriver & pxDriver, cc::ByteNumbering byteNumbering) {

    auto & b = pxDriver.getBuilder();
    auto P = pxDriver.makePipeline({Binding{b->getInt32Ty(), "inputFileDecriptor"}, Binding{b->getInt8PtrTy(), "outputFileName"}}, {});
    Scalar * fileDescriptor = P->getInputScalar("inputFileDecriptor");
    // File data from mmap
    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);
    StreamSet * const u16bytes = P->CreateStreamSet(1, 16);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);

    StreamSet * const nonAscii =  P->CreateStreamSet();

    CC * const nonAsciiCC = makeByte(0x80, 0xFF);
    P->CreateKernelCall<CharacterClassKernelBuilder>(
        std::vector<CC *>{nonAsciiCC}, ByteStream, nonAscii);

    auto B = P->CreateOptimizationBranch(nonAscii,
        {Binding{"ByteStream", ByteStream}, Binding{"condition", nonAscii}},
        {Binding{"u16bytes", u16bytes, BoundedRate(0, 1)}});

    makeAllAsciiBranch(B->getAllZeroBranch(), ByteStream, u16bytes, byteNumbering);

    makeNonAsciiBranch(b, B->getNonZeroBranch(), ByteStream, u16bytes, byteNumbering);

    Scalar * outputFileName = P->getInputScalar("outputFileName");
    P->CreateKernelCall<FileSink>(outputFileName, u16bytes);

    return reinterpret_cast<u8u16FunctionType>(P->compile());
}

size_t file_size(const int fd) {
    struct stat st;
    if (LLVM_UNLIKELY(fstat(fd, &st) != 0)) {
        st.st_size = 0;
    }
    return st.st_size;
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&u8u16Options, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    cc::ByteNumbering byteNumbering;
    if ((OutputEncoding == "UTF16LE") || (OutputEncoding == "UTF-16LE")) {
        byteNumbering = cc::ByteNumbering::LittleEndian;
    } else if ((OutputEncoding == "UTF16BE") || (OutputEncoding == "UTF-16BE")) {
        byteNumbering = cc::ByteNumbering::BigEndian;
    } else if ((OutputEncoding == "UTF16") || (OutputEncoding == "UTF-16")) {
        byteNumbering = cc::ByteNumbering::BigEndian;
    } else {
        llvm::report_fatal_error("Unrecognized encoding.");
    }
    CPUDriver pxDriver("u8u16");
    u8u16FunctionType u8u16Function = nullptr;
    if (BranchingMode) {
        u8u16Function = generatePipeline2(pxDriver, byteNumbering);
    } else {
        u8u16Function = generatePipeline(pxDriver, byteNumbering);
    }
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        std::cerr << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        u8u16Function(fd, outputFile.c_str());
        close(fd);
    }
    return 0;
}



