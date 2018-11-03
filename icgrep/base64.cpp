/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <iostream>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Support/CommandLine.h>
#include <toolchain/toolchain.h>
#include <toolchain/cpudriver.h>
#include <IR_Gen/idisa_target.h>
#include <kernels/source_kernel.h>
#include <kernels/streamset.h>
#include <kernels/radix64.h>
#include <kernels/stdout_kernel.h>
#include <kernels/kernel_builder.h>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/anonymous_shared_memory.hpp>
#include <boost/math/common_factor_rt.hpp>
#include <sys/stat.h>
#include <fcntl.h>
#include <mutex>
#include <kernels/pipeline_builder.h>

using namespace llvm;
using namespace codegen;

static cl::OptionCategory base64Options("base64 Options",
                                            "Transcoding control options.");

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore, cl::cat(base64Options));

static cl::opt<bool> mMapBuffering("mmap-buffering", cl::desc("Enable mmap buffering."), cl::cat(base64Options));
static cl::opt<bool> memAlignBuffering("memalign-buffering", cl::desc("Enable posix_memalign buffering."), cl::cat(base64Options));
static cl::opt<int> Threads("threads", cl::desc("Total number of threads."), cl::init(1));

using namespace kernel;

typedef void (*base64FunctionType)(const uint32_t fd);

base64FunctionType base64PipelineGen(CPUDriver & pxDriver) {
    auto & iBuilder = pxDriver.getBuilder();
    Type * const int32Ty = iBuilder->getInt32Ty();
    auto P = pxDriver.makePipeline({Binding{int32Ty, "fd"}});
    Scalar * const fileDescriptor = P->getInputScalar("fd");
    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);
    StreamSet * const Expanded3_4Out = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<expand3_4Kernel>(ByteStream, Expanded3_4Out);
    StreamSet * const Radix64out = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<radix64Kernel>(Expanded3_4Out, Radix64out);
    StreamSet * const base64 = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<base64Kernel>(Radix64out, base64);
    P->CreateKernelCall<StdOutKernel>(base64);
    return reinterpret_cast<base64FunctionType>(P->compile());
}

size_t file_size(const int fd) {
    struct stat st;
    if (LLVM_UNLIKELY(fstat(fd, &st) != 0)) {
        st.st_size = 0;
    }
    return st.st_size;
}

void base64(base64FunctionType fn_ptr, const std::string & fileName) {
    const int fd = open(fileName.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        std::cerr << "Error: cannot open " << fileName << " for processing. Skipped.\n";
        return;
    }
    fn_ptr(fd);
    close(fd);    
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&base64Options, codegen::codegen_flags()});

    CPUDriver pxDriver("base64");
    auto fn_ptr = base64PipelineGen(pxDriver);
    for (unsigned i = 0; i != inputFiles.size(); ++i) {
        base64(fn_ptr, inputFiles[i]);
    }
    return 0;
}

