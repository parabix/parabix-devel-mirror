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
#include <IR_Gen/idisa_builder.h>
#include <IR_Gen/idisa_target.h>
#include <kernels/source_kernel.h>
#include <kernels/streamset.h>
#include <kernels/radix64.h>
#include <kernels/stdout_kernel.h>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/anonymous_shared_memory.hpp>
#include <sys/stat.h>
#include <fcntl.h>

using namespace llvm;

static cl::OptionCategory base64Options("base64 Options",
                                            "Transcoding control options.");

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore, cl::cat(base64Options));

static cl::opt<bool> mMapBuffering("mmap-buffering", cl::desc("Enable mmap buffering."), cl::cat(base64Options));
static cl::opt<bool> memAlignBuffering("memalign-buffering", cl::desc("Enable posix_memalign buffering."), cl::cat(base64Options));


using namespace kernel;
using namespace parabix;

void base64PipelineGen(ParabixDriver & pxDriver) {
        
    auto & iBuilder = pxDriver.getBuilder();
    Module * mod = iBuilder->getModule();
    Type * bitBlockType = iBuilder->getBitBlockType();

    Type * const voidTy = iBuilder->getVoidTy();
    Type * const int32Ty = iBuilder->getInt32Ty();
    Type * const outputType = PointerType::get(ArrayType::get(ArrayType::get(bitBlockType, 8), 1), 0);
    
    
    Function * const main = cast<Function>(mod->getOrInsertFunction("Main", voidTy, int32Ty, outputType, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();
    
    Value * const fileDescriptor = &*(args++);
    fileDescriptor->setName("fileDescriptor");
    Value * const outputStream = &*(args++);
    outputStream->setName("outputStream");

    //Round up to a multiple of 3.
    const unsigned segmentSize = ((codegen::SegmentSize + 2)/3) * 3;
    
    const unsigned bufferSegments = codegen::BufferSegments;
    
    SourceBuffer ByteStream(iBuilder, iBuilder->getStreamSetTy(1, 8));

    CircularBuffer Expanded3_4Out(iBuilder, iBuilder->getStreamSetTy(1, 8), segmentSize * 4/3 * bufferSegments);
    CircularBuffer Radix64out(iBuilder, iBuilder->getStreamSetTy(1, 8), segmentSize * 4/3 * bufferSegments);
    CircularCopybackBuffer Base64out(iBuilder, iBuilder->getStreamSetTy(1, 8), segmentSize * 4/3 * bufferSegments, 1);
    
    MMapSourceKernel mmapK(iBuilder, segmentSize);
    mmapK.setInitialArguments({fileDescriptor});
    pxDriver.addKernelCall(mmapK, {}, {&ByteStream});
    
    expand3_4Kernel expandK(iBuilder);
    pxDriver.addKernelCall(expandK, {&ByteStream}, {&Expanded3_4Out});

    radix64Kernel radix64K(iBuilder);
    pxDriver.addKernelCall(radix64K, {&Expanded3_4Out}, {&Radix64out});

    base64Kernel base64K(iBuilder);
    pxDriver.addKernelCall(base64K, {&Radix64out}, {&Base64out});
    
    StdOutKernel stdoutK(iBuilder, 8);
    pxDriver.addKernelCall(stdoutK, {&Base64out}, {});
    
    iBuilder->SetInsertPoint(BasicBlock::Create(mod->getContext(), "entry", main,0));

    ByteStream.allocateBuffer(iBuilder);
    Expanded3_4Out.allocateBuffer(iBuilder);
    Radix64out.allocateBuffer(iBuilder);
    Base64out.allocateBuffer(iBuilder);

    pxDriver.generatePipelineIR();

    iBuilder->CreateRetVoid();

    pxDriver.linkAndFinalize();
}


typedef void (*base64FunctionType)(const uint32_t fd, char * outputBuffer);

base64FunctionType base64CodeGen(void) {
    ParabixDriver pxDriver("base64");
    base64PipelineGen(pxDriver);
    return reinterpret_cast<base64FunctionType>(pxDriver.getPointerToMain());
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
    if (mMapBuffering) {
        boost::interprocess::mapped_region outputBuffer(boost::interprocess::anonymous_shared_memory(2 * file_size(fd)));
        outputBuffer.advise(boost::interprocess::mapped_region::advice_willneed);
        outputBuffer.advise(boost::interprocess::mapped_region::advice_sequential);
        fn_ptr(fd, static_cast<char*>(outputBuffer.get_address()));
    } else if (memAlignBuffering) {
        char * outputBuffer;
        if (posix_memalign(reinterpret_cast<void **>(&outputBuffer), 32, 2 * file_size(fd))) {
            throw std::bad_alloc();
        }
        fn_ptr(fd, outputBuffer);
        free(reinterpret_cast<void *>(outputBuffer));
    } else { /* No external output buffer */
        fn_ptr(fd, nullptr);
    }
    close(fd);
    
}


int main(int argc, char *argv[]) {
    AddParabixVersionPrinter();
    cl::HideUnrelatedOptions(ArrayRef<const cl::OptionCategory *>{&base64Options, codegen::codegen_flags()});
    cl::ParseCommandLineOptions(argc, argv);

    base64FunctionType fn_ptr = base64CodeGen();

    for (unsigned i = 0; i != inputFiles.size(); ++i) {
        base64(fn_ptr, inputFiles[i]);
    }

    return 0;
}

