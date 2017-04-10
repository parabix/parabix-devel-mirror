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
#include <kernels/toolchain.h>
#include <IR_Gen/idisa_builder.h>
#include <IR_Gen/idisa_target.h>
#include <kernels/pipeline.h>
#include <kernels/mmap_kernel.h>
#include <kernels/streamset.h>
#include <kernels/radix64.h>
#include <kernels/stdout_kernel.h>
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
#include <boost/interprocess/anonymous_shared_memory.hpp>

using namespace llvm;

static cl::OptionCategory base64Options("base64 Options",
                                            "Transcoding control options.");

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore, cl::cat(base64Options));

static cl::opt<bool> mMapBuffering("mmap-buffering", cl::desc("Enable mmap buffering."), cl::cat(base64Options));
static cl::opt<bool> memAlignBuffering("memalign-buffering", cl::desc("Enable posix_memalign buffering."), cl::cat(base64Options));


using namespace kernel;
using namespace parabix;

void base64PipelineGen(ParabixDriver & pxDriver) {
        
    IDISA::IDISA_Builder * iBuilder = pxDriver.getIDISA_Builder();
    Module * mod = iBuilder->getModule();
    Type * mBitBlockType = iBuilder->getBitBlockType();

    Type * const size_ty = iBuilder->getSizeTy();
    Type * const voidTy = Type::getVoidTy(mod->getContext());
    Type * const inputType = PointerType::get(ArrayType::get(ArrayType::get(mBitBlockType, 8), 1), 0);
    Type * const outputType = PointerType::get(ArrayType::get(ArrayType::get(mBitBlockType, 8), 1), 0);
    
    
    Function * const main = cast<Function>(mod->getOrInsertFunction("Main", voidTy, inputType, outputType, size_ty, nullptr));
    main->setCallingConv(CallingConv::C);
    Function::arg_iterator args = main->arg_begin();
    
    Value * const inputStream = &*(args++);
    inputStream->setName("inputStream");
    Value * const outputStream = &*(args++);
    outputStream->setName("outputStream");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");

    //Round up to a multiple of 3.
    const unsigned segmentSize = ((codegen::SegmentSize + 2)/3) * 3;
    
    const unsigned bufferSegments = codegen::BufferSegments;
    
    ExternalFileBuffer ByteStream(iBuilder, iBuilder->getStreamSetTy(1, 8));

    CircularBuffer Expanded3_4Out(iBuilder, iBuilder->getStreamSetTy(1, 8), segmentSize * 4/3 * bufferSegments);
    CircularBuffer Radix64out(iBuilder, iBuilder->getStreamSetTy(1, 8), segmentSize * 4/3 * bufferSegments);
    CircularCopybackBuffer Base64out(iBuilder, iBuilder->getStreamSetTy(1, 8), segmentSize * 4/3 * bufferSegments, 1);
    
    MMapSourceKernel mmapK(iBuilder, segmentSize);
    mmapK.setInitialArguments({fileSize});
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

    ByteStream.setStreamSetBuffer(inputStream);
    Expanded3_4Out.allocateBuffer();
    Radix64out.allocateBuffer();
    Base64out.allocateBuffer();

    pxDriver.generatePipelineIR();

    iBuilder->CreateRetVoid();

    pxDriver.linkAndFinalize();
}


typedef void (*base64FunctionType)(char * byte_data, char * output_data, size_t filesize);

base64FunctionType base64CodeGen(void) {
    LLVMContext TheContext;                            
    Module * M = new Module("base64", TheContext);
    IDISA::IDISA_Builder * idb = IDISA::GetIDISA_Builder(M);
    ParabixDriver pxDriver(idb);
    
    base64PipelineGen(pxDriver);
    base64FunctionType main = reinterpret_cast<base64FunctionType>(pxDriver.getPointerToMain());
    
    delete idb;
    return main;
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
    AddParabixVersionPrinter();
    cl::HideUnrelatedOptions(ArrayRef<const cl::OptionCategory *>{&base64Options, codegen::codegen_flags()});
    cl::ParseCommandLineOptions(argc, argv);

    base64FunctionType fn_ptr = base64CodeGen();

    for (unsigned i = 0; i != inputFiles.size(); ++i) {
        base64(fn_ptr, inputFiles[i]);
    }

    return 0;
}

