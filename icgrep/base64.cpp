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
#include <sys/stat.h>
#include <fcntl.h>
#include <mutex>

using namespace llvm;

static cl::OptionCategory base64Options("base64 Options",
                                            "Transcoding control options.");

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore, cl::cat(base64Options));

static cl::opt<bool> mMapBuffering("mmap-buffering", cl::desc("Enable mmap buffering."), cl::cat(base64Options));
static cl::opt<bool> memAlignBuffering("memalign-buffering", cl::desc("Enable posix_memalign buffering."), cl::cat(base64Options));
static cl::opt<int> Threads("threads", cl::desc("Total number of threads."), cl::init(1));


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
    iBuilder->SetInsertPoint(BasicBlock::Create(mod->getContext(), "entry", main,0));

    //Round up to a multiple of 3.
    const unsigned initSegSize = ((codegen::SegmentSize + 2)/3) * 3;
    const unsigned bufferSize = (4 * initSegSize * codegen::BufferSegments) / 3;

    StreamSetBuffer * ByteStream = pxDriver.addBuffer<SourceBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8));

    Kernel * mmapK = pxDriver.addKernelInstance<MMapSourceKernel>(iBuilder, initSegSize);
    mmapK->setInitialArguments({fileDescriptor});
    pxDriver.makeKernelCall(mmapK, {}, {ByteStream});
    
    StreamSetBuffer * Expanded3_4Out = pxDriver.addBuffer<DynamicBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), bufferSize);
    Kernel * expandK = pxDriver.addKernelInstance<expand3_4Kernel>(iBuilder);
    pxDriver.makeKernelCall(expandK, {ByteStream}, {Expanded3_4Out});
    
    StreamSetBuffer * Radix64out = pxDriver.addBuffer<DynamicBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), bufferSize);
    Kernel * radix64K = pxDriver.addKernelInstance<radix64Kernel>(iBuilder);
    pxDriver.makeKernelCall(radix64K, {Expanded3_4Out}, {Radix64out});
    
    if (memAlignBuffering){
        auto Base64out = pxDriver.addBuffer<ExternalBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), outputStream);
        Kernel * base64K = pxDriver.addKernelInstance<base64Kernel>(iBuilder);
        pxDriver.makeKernelCall(base64K, {Radix64out}, {Base64out});
    }
    else {
        StreamSetBuffer * Base64out = pxDriver.addBuffer<DynamicBuffer>(iBuilder, iBuilder->getStreamSetTy(1, 8), bufferSize);
        Kernel * base64K = pxDriver.addKernelInstance<base64Kernel>(iBuilder);
        pxDriver.makeKernelCall(base64K, {Radix64out}, {Base64out});
        
        Kernel * outK = pxDriver.addKernelInstance<StdOutKernel>(iBuilder, 8);
        pxDriver.makeKernelCall(outK, {Base64out}, {});
    }
    
    pxDriver.generatePipelineIR();
    pxDriver.deallocateBuffers();
    iBuilder->CreateRetVoid();

    pxDriver.finalizeObject();
}

typedef void (*base64FunctionType)(const uint32_t fd, char * outputBuffer);

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
        unsigned inputSize = file_size(fd);
        unsigned paddingSize = (inputSize % 3) ? (4 - (inputSize % 3)) : 0;
        unsigned outputSize = inputSize * 4/3 + paddingSize;

        char * outputBuffer;
        if (posix_memalign(reinterpret_cast<void **>(&outputBuffer), 32, inputSize * 2)) {
            throw std::bad_alloc();
        }
        fn_ptr(fd, outputBuffer);
        fwrite(outputBuffer, outputSize, 1, stdout);
        free(reinterpret_cast<void *>(outputBuffer));
    } else { /* No external output buffer */
        fn_ptr(fd, nullptr);
    }
    close(fd);
    
}

std::mutex count_mutex;
size_t fileCount;
base64FunctionType fn_ptr;

std::vector<char *> resultStrs;
std::vector<int> filesizes;

void *Base64ThreadFunction(void *args)
{
    size_t fileIdx;

    count_mutex.lock();
    fileIdx = fileCount;
    fileCount++;
    count_mutex.unlock();

    while (fileIdx < inputFiles.size()) {
        const int fd = open(inputFiles[fileIdx].c_str(), O_RDONLY);
        if (LLVM_UNLIKELY(fd == -1)) {
            std::cerr << "Error: cannot open " << inputFiles[fileIdx] << " for processing. Skipped.\n";
            exit(-1);
        }

        char * outputBuffer;
        if (posix_memalign(reinterpret_cast<void **>(&outputBuffer), 32, 2 * file_size(fd))) {
            throw std::bad_alloc();
        }
    
        fn_ptr(fd, outputBuffer);
        resultStrs[fileIdx] = outputBuffer;
        filesizes[fileIdx] = file_size(fd);

        count_mutex.lock();
        fileIdx = fileCount;
        fileCount++;
        count_mutex.unlock();
    }

    pthread_exit(nullptr);
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&base64Options, codegen::codegen_flags()});

    if (Threads == 1) {
        ParabixDriver pxDriver("base64");
        base64PipelineGen(pxDriver);
        fn_ptr = reinterpret_cast<base64FunctionType>(pxDriver.getMain());     
        for (unsigned i = 0; i != inputFiles.size(); ++i) {
            base64(fn_ptr, inputFiles[i]);
        }
    }
    else{
        memAlignBuffering = true;
        ParabixDriver pxDriver("base64");
        base64PipelineGen(pxDriver);
        fn_ptr = reinterpret_cast<base64FunctionType>(pxDriver.getMain());

        fileCount = 0;
        const unsigned n = inputFiles.size();
        resultStrs.resize(n);
        filesizes.resize(n);

        const unsigned numOfThreads = Threads;
        pthread_t threads[numOfThreads];

        for(unsigned long i = 0; i < numOfThreads; ++i){
            const int rc = pthread_create(&threads[i], NULL, Base64ThreadFunction, (void *)i);
            if (rc) {
                llvm::report_fatal_error("Failed to create thread: code " + std::to_string(rc));
            }
        }

        for(unsigned i = 0; i < numOfThreads; ++i) {
            void * status = nullptr;
            const int rc = pthread_join(threads[i], &status);
            if (rc) {
                llvm::report_fatal_error("Failed to join thread: code " + std::to_string(rc));
            }
        }

        for (unsigned i=0; i<resultStrs.size(); i++){
            unsigned paddingSize = (filesizes[i] % 3) ? (4 - (filesizes[i] % 3)) : 0;
            fwrite(resultStrs[i], filesizes[i] * 4/3 + paddingSize, 1, stdout);
        }
    }   

    return 0;
}

