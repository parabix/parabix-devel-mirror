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
#include <IR_Gen/idisa_builder.h>
#include <IR_Gen/idisa_target.h>
#include <kernels/pipeline.h>
#include <kernels/interface.h>
#include <kernels/kernel.h>
#include <kernels/radix64.h>
#include <kernels/stdout_kernel.h>


// mmap system
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
#include <boost/interprocess/anonymous_shared_memory.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <fcntl.h>
static cl::OptionCategory base64Options("base64 Options",
                                            "Transcoding control options.");

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore, cl::cat(base64Options));

static cl::opt<bool> segmentPipelineParallel("enable-segment-pipeline-parallel", cl::desc("Enable multithreading with segment pipeline parallelism."), cl::cat(base64Options));
static cl::opt<bool> mMapBuffering("mmap-buffering", cl::desc("Enable mmap buffering."), cl::cat(base64Options));
static cl::opt<bool> memAlignBuffering("memalign-buffering", cl::desc("Enable posix_memalign buffering."), cl::cat(base64Options));


using namespace kernel;
using namespace parabix;

Function * base64Pipeline(Module * mMod, IDISA::IDISA_Builder * iBuilder) {
    Type * mBitBlockType = iBuilder->getBitBlockType();

    //Round up to a multiple of 4. 
    const unsigned segmentSize = ((codegen::SegmentSize + 3)/4) * 4;
    
    const unsigned bufferSegments = codegen::BufferSegments;
    
    ExternalFileBuffer ByteStream(iBuilder, iBuilder->getStreamSetTy(1, 8));

    CircularBuffer Expanded3_4Out(iBuilder, iBuilder->getStreamSetTy(1, 8), segmentSize * bufferSegments * 16);
    CircularBuffer Radix64out(iBuilder, iBuilder->getStreamSetTy(1, 8), segmentSize * bufferSegments * 16);
    LinearCopybackBuffer Base64out(iBuilder, iBuilder->getStreamSetTy(1, 8), segmentSize * bufferSegments * 16 + 2);

    expand3_4Kernel expandK(iBuilder);
    expandK.generateKernel({&ByteStream}, {&Expanded3_4Out});

    radix64Kernel radix64K(iBuilder);
    radix64K.generateKernel({&Expanded3_4Out}, {&Radix64out});

    base64Kernel base64K(iBuilder);
    base64K.generateKernel({&Radix64out}, {&Base64out});
    
    StdOutKernel stdoutK(iBuilder, 8);
    stdoutK.generateKernel({&Base64out}, {});

    Type * const size_ty = iBuilder->getSizeTy();
    Type * const voidTy = Type::getVoidTy(mMod->getContext());
    Type * const inputType = PointerType::get(ArrayType::get(ArrayType::get(mBitBlockType, 8), 1), 0);
    Type * const outputType = PointerType::get(ArrayType::get(ArrayType::get(mBitBlockType, 8), 1), 0);

    
    Function * const main = cast<Function>(mMod->getOrInsertFunction("Main", voidTy, inputType, outputType, size_ty, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();
    
    Value * const inputStream = &*(args++);
    inputStream->setName("inputStream");
    Value * const outputStream = &*(args++);
    outputStream->setName("outputStream");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");
    
    iBuilder->SetInsertPoint(BasicBlock::Create(mMod->getContext(), "entry", main,0));

    ByteStream.setStreamSetBuffer(inputStream, fileSize);
//    Radix64out.setEmptyBuffer(iBuilder->CreatePointerCast(outputStream, outputType));
    mMod->dump();
    Expanded3_4Out.allocateBuffer();
    Radix64out.allocateBuffer();
    Base64out.allocateBuffer();


    if (segmentPipelineParallel){
        generateSegmentParallelPipeline(iBuilder, {&expandK, &radix64K, &base64K, &stdoutK});
    }
    else{
        generatePipelineLoop(iBuilder, {&expandK, &radix64K, &base64K, &stdoutK});
    }

    iBuilder->CreateRetVoid();
    return main;
}


typedef void (*base64FunctionType)(char * byte_data, char * output_data, size_t filesize);

static ExecutionEngine * base64Engine = nullptr;

base64FunctionType base64CodeGen(void) {
    LLVMContext TheContext;                            
    Module * M = new Module("base64", TheContext);
    IDISA::IDISA_Builder * idb = IDISA::GetIDISA_Builder(M);

    
    llvm::Function * main_IR = base64Pipeline(M, idb);
    
    verifyModule(*M, &dbgs());
    //std::cerr << "ExecuteKernels(); done\n";
    base64Engine = JIT_to_ExecutionEngine(M);
    
    base64Engine->finalizeObject();
    //std::cerr << "finalizeObject(); done\n";

    delete idb;
    return reinterpret_cast<base64FunctionType>(base64Engine->getPointerToFunction(main_IR));
}

void base64(base64FunctionType fn_ptr, const std::string & fileName) {
    std::string mFileName = fileName;
    size_t mFileSize;
    char * mFileBuffer;

    const boost::filesystem::path file(mFileName);
    if (exists(file)) {
        if (is_directory(file)) {
            return;
        }
    } else {
        std::cerr << "Error: cannot open " << mFileName << " for processing. Skipped.\n";
        return;
    }
    
    mFileSize = file_size(file);
    boost::iostreams::mapped_file_source mFile;
    if (mFileSize == 0) {
        mFileBuffer = nullptr;
    }
    else {
        try {
            mFile.open(mFileName);
        } catch (std::exception &e) {
            std::cerr << "Error: Boost mmap of " << mFileName << ": " << e.what() << std::endl;
            return;
        }
        mFileBuffer = const_cast<char *>(mFile.data());
    }

    if (mMapBuffering) {
        boost::interprocess::mapped_region outputBuffer(boost::interprocess::anonymous_shared_memory(2*mFileSize));
        outputBuffer.advise(boost::interprocess::mapped_region::advice_willneed);
        outputBuffer.advise(boost::interprocess::mapped_region::advice_sequential);
        std::cerr << "outputBuffer " << std::hex << (intptr_t)(outputBuffer.get_address()) << std::dec << "\n";
        fn_ptr(mFileBuffer, static_cast<char*>(outputBuffer.get_address()), mFileSize);
    }
    else if (memAlignBuffering) {
        char * outputBuffer;
        if (posix_memalign(reinterpret_cast<void **>(&outputBuffer), 32, 2*mFileSize)) {
            throw std::bad_alloc();
        }
        fn_ptr(mFileBuffer, outputBuffer, mFileSize);
        free(reinterpret_cast<void *>(outputBuffer));
    }
    else {
        /* No external output buffer */
        fn_ptr(mFileBuffer, nullptr, mFileSize);
    }
    mFile.close();
    
}


int main(int argc, char *argv[]) {
    cl::HideUnrelatedOptions(ArrayRef<const cl::OptionCategory *>{&base64Options, codegen::codegen_flags()});
    cl::ParseCommandLineOptions(argc, argv);

    base64FunctionType fn_ptr = base64CodeGen();

    for (unsigned i = 0; i != inputFiles.size(); ++i) {
        base64(fn_ptr, inputFiles[i]);
    }

    return 0;
}

