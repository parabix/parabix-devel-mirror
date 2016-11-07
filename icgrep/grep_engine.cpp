/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <grep_engine.h>
#include <IDISA/idisa_builder.h>
#include <IDISA/idisa_target.h>
#include <llvm/Support/CommandLine.h>
#include <re/re_toolchain.h>
#include <re/re_cc.h>

#include <pablo/pablo_toolchain.h>
#include <toolchain.h>
#include <pablo/pablo_compiler.h>
#include <kernels/pipeline.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/MCJIT.h>
#include <llvm/IRReader/IRReader.h>
#include <llvm/Support/Debug.h>
#include <llvm/IR/Verifier.h>
#include <llvm/IR/TypeBuilder.h>
#include <UCD/UnicodeNameData.h>


#include <kernels/streamset.h>
#include <kernels/scanmatchgen.h>
#include <kernels/s2p_kernel.h>
#include <kernels/cc_kernel.h>
#include <kernels/pipeline.h>

#include <pablo/function.h>
#include <pablo/pablo_kernel.h>
#include <pablo/pablo_toolchain.h>

#include <llvm/IR/Intrinsics.h>
#include "llvm/Support/SourceMgr.h"
#include "llvm/IRReader/IRReader.h"
#include "llvm/Linker/Linker.h"
#include "llvm-c/Core.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <stdint.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdexcept>
#include <cctype>


#include <llvm/Support/raw_os_ostream.h>

// mmap system
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>

#include <fcntl.h>

#include <kernels/kernel.h>

#ifdef CUDA_ENABLED 
#include <IDISA/CudaDriver.h>
#endif

static cl::OptionCategory bGrepOutputOptions("Output Options",
                                             "These options control the output.");
static cl::opt<bool> SilenceFileErrors("s", cl::desc("Suppress messages for file errors."), cl::init(false),  cl::cat(bGrepOutputOptions));

static cl::opt<bool> SuppressOutput("q", cl::desc("Suppress normal output; set return code only."), cl::init(false),  cl::cat(bGrepOutputOptions));

static cl::opt<bool> NormalizeLineBreaks("normalize-line-breaks", cl::desc("Normalize line breaks to std::endl."), cl::init(false),  cl::cat(bGrepOutputOptions));

static cl::opt<bool> ShowFileNames("H", cl::desc("Show the file name with each matching line."), cl::cat(bGrepOutputOptions));
static cl::alias ShowFileNamesLong("with-filename", cl::desc("Alias for -H"), cl::aliasopt(ShowFileNames));

static cl::opt<bool> ShowLineNumbers("n", cl::desc("Show the line number with each matching line."), cl::cat(bGrepOutputOptions));
static cl::alias ShowLineNumbersLong("line-number", cl::desc("Alias for -n"), cl::aliasopt(ShowLineNumbers));

static cl::opt<bool> pipelineParallel("enable-pipeline-parallel", cl::desc("Enable multithreading with pipeline parallelism."), cl::cat(bGrepOutputOptions));

static cl::opt<bool> segmentPipelineParallel("enable-segment-pipeline-parallel", cl::desc("Enable multithreading with segment pipeline parallelism."), cl::cat(bGrepOutputOptions));

bool isUTF_16 = false;
std::string IRFilename = "icgrep.ll";
std::string PTXFilename = "icgrep.ptx";

void GrepEngine::doGrep(const std::string & fileName, const int fileIdx, bool CountOnly, std::vector<size_t> & total_CountOnly, bool UTF_16) {
    boost::filesystem::path file(fileName);
    if (exists(file)) {
        if (is_directory(file)) {
            return;
        }
    } else {
        if (!SilenceFileErrors) {
            std::cerr << "Error: cannot open " << fileName << " for processing. Skipped.\n";
            return;
        }
    }

    const auto fileSize = file_size(file);
    if (fileSize > 0) {
        try {
            boost::iostreams::mapped_file_source source(fileName, fileSize, 0);
            char * fileBuffer = const_cast<char *>(source.data());
#ifdef CUDA_ENABLED  
            if(codegen::NVPTX){
                ulong * rslt = RunPTX(PTXFilename, fileBuffer, fileSize, CountOnly);
                if (CountOnly){
                    exit(0);
                }
                else{
                    mGrepFunction_CPU((char *)rslt, fileBuffer, fileSize, fileIdx);
                    return;
                }
                
            } 
#endif
            if (CountOnly) {
                total_CountOnly[fileIdx] = mGrepFunction_CountOnly(fileBuffer, fileSize, fileIdx);
            } else {
                mGrepFunction(fileBuffer, fileSize, fileIdx);
            }
            source.close();
        } catch (std::exception & e) {
            if (!SilenceFileErrors) {
                std::cerr << "Boost mmap error: " + fileName + ": " + e.what() + " Skipped.\n";
                return;
            }
        }
    } else {
#ifdef CUDA_ENABLED 
        if (codegen::NVPTX){
            std::cout << 0 << std::endl;
            exit(0);
        }
#endif
        if (CountOnly) {
            total_CountOnly[fileIdx] = mGrepFunction_CountOnly(nullptr, 0, fileIdx);
        } else {
            mGrepFunction(nullptr, 0, fileIdx);
        }
    }
}

using namespace parabix;

Function * generateGPUKernel(Module * m, IDISA::IDISA_Builder * iBuilder, bool CountOnly){
    Type * const int64ty = iBuilder->getInt64Ty();
    Type * const inputType = PointerType::get(int64ty, 1);
    Type * const resultTy = iBuilder->getVoidTy();
    Function * kernelFunc = cast<Function>(m->getOrInsertFunction("GPU_Main", resultTy, inputType, inputType, inputType, nullptr));
    kernelFunc->setCallingConv(CallingConv::C);
    Function::arg_iterator args = kernelFunc->arg_begin();

    Value * const inputPtr = &*(args++);
    inputPtr->setName("inputPtr");
    Value * const bufferSizePtr = &*(args++);
    bufferSizePtr->setName("bufferSizePtr");
    Value * const outputPtr = &*(args++);
    outputPtr->setName("resultPtr");

    BasicBlock * entryBlock = BasicBlock::Create(m->getContext(), "entry", kernelFunc, 0);
    iBuilder->SetInsertPoint(entryBlock);

    Function * tidFunc = m->getFunction("llvm.nvvm.read.ptx.sreg.tid.x");
    Value * id = iBuilder->CreateCall(tidFunc);

    Function * mainFunc = m->getFunction("Main");
    Type * const inputStreamType = PointerType::get(ArrayType::get(ArrayType::get(iBuilder->getBitBlockType(), 8), 1), 1);    
    Value * inputStreamPtr = iBuilder->CreateBitCast(inputPtr, inputStreamType); 
    Value * inputStream = iBuilder->CreateGEP(inputStreamPtr, id);

    Value * bufferSize = iBuilder->CreateLoad(bufferSizePtr);
    if (CountOnly){
        Value * outputThreadPtr = iBuilder->CreateGEP(outputPtr, id);
        Value * result = iBuilder->CreateCall(mainFunc, {inputStream, bufferSize});
        iBuilder->CreateStore(result, outputThreadPtr);
    }
    else {
        Type * const outputStremType = PointerType::get(ArrayType::get(iBuilder->getBitBlockType(), 2), 1);
        Value * outputStreamPtr = iBuilder->CreateBitCast(outputPtr, outputStremType);
        Value * outputStream = iBuilder->CreateGEP(outputStreamPtr, id);
        iBuilder->CreateCall(mainFunc, {inputStream, bufferSize, outputStream});
    }    

    iBuilder->CreateRetVoid();
    return kernelFunc;
}

Function * generateCPUKernel(Module * m, IDISA::IDISA_Builder * iBuilder, bool isNameExpression){
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();
    Type * const rsltType = PointerType::get(ArrayType::get(iBuilder->getBitBlockType(), 2), 0);
    Function * const mainCPUFn = cast<Function>(m->getOrInsertFunction("CPU_Main", iBuilder->getVoidTy(), rsltType, int8PtrTy, size_ty, size_ty, nullptr));
    mainCPUFn->setCallingConv(CallingConv::C);
    iBuilder->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", mainCPUFn, 0));
    Function::arg_iterator args = mainCPUFn->arg_begin();
    
    Value * const rsltStream = &*(args++);
    rsltStream->setName("rslt");
    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");
    Value * const fileIdx = &*(args++);
    fileIdx->setName("fileIdx");

    ExternalFileBuffer MatchResults(iBuilder, StreamSetType(iBuilder, 2, 1));
    MatchResults.setStreamSetBuffer(rsltStream, fileSize);

    kernel::scanMatchKernel scanMatchK(iBuilder, isNameExpression);
    scanMatchK.generateKernel({&MatchResults}, {});
            
    Value * scanMatchInstance = scanMatchK.createInstance({inputStream, fileSize, fileIdx});
    
    generatePipelineLoop(iBuilder, {&scanMatchK}, {scanMatchInstance}, fileSize);
    iBuilder->CreateRetVoid();

    return mainCPUFn;
}

void GrepEngine::grepCodeGen(std::string moduleName, re::RE * re_ast, bool CountOnly, bool UTF_16, bool isNameExpression) {
    isUTF_16 = UTF_16;
    int addrSpace = 0;
    bool CPU_Only = true;
    Module * M = nullptr;  
    IDISA::IDISA_Builder * iBuilder = nullptr; 

#ifdef CUDA_ENABLED 
    setNVPTXOption(); 
    if (codegen::NVPTX) {
        Module * gpuM = new Module(moduleName+":gpu", getGlobalContext());
        IDISA::IDISA_Builder * GPUBuilder = IDISA::GetIDISA_GPU_Builder(gpuM);
        M = gpuM;
        iBuilder = GPUBuilder;
        M->setDataLayout("e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v16:16:16-v32:32:32-v64:64:64-v128:128:128-n16:32:64");
        M->setTargetTriple("nvptx64-nvidia-cuda");
        addrSpace = 1;
        CPU_Only = false;
        codegen::BlockSize = 64;
    }
#endif

    Module * cpuM = new Module(moduleName+":cpu", getGlobalContext());
    IDISA::IDISA_Builder * CPUBuilder = IDISA::GetIDISA_Builder(cpuM);

    if(CPU_Only) {
        M = cpuM;
        iBuilder = CPUBuilder;
    }

    const unsigned segmentSize = codegen::SegmentSize;
    if (segmentPipelineParallel && codegen::BufferSegments < 2) {
        codegen::BufferSegments = 2;
    }
    const unsigned bufferSegments = codegen::BufferSegments;


    unsigned encodingBits = UTF_16 ? 16 : 8;

    mIsNameExpression = isNameExpression;

    Type * const int32ty = iBuilder->getInt32Ty();
    Type * const int64ty = iBuilder->getInt64Ty();
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();
    Type * const voidTy = Type::getVoidTy(M->getContext());    
    Type * const voidPtrTy = TypeBuilder<void *, false>::get(M->getContext());
    Type * const inputType = PointerType::get(ArrayType::get(ArrayType::get(iBuilder->getBitBlockType(), (UTF_16 ? 16 : 8)), 1), addrSpace);
    Type * const resultTy = CountOnly ? int64ty : iBuilder->getVoidTy();

    Function * mainFn = nullptr;
    Value * inputStream = nullptr;
    Value * fileSize = nullptr;
    Value * fileIdx = nullptr;

#ifdef CUDA_ENABLED   
    Value * outputStream = nullptr;
    Type * const outputType = PointerType::get(ArrayType::get(iBuilder->getBitBlockType(), 2), addrSpace);
    if (codegen::NVPTX){
        if (CountOnly){
            mainFn = cast<Function>(M->getOrInsertFunction("Main", resultTy, inputType, size_ty, nullptr));
            mainFn->setCallingConv(CallingConv::C);
            iBuilder->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", mainFn, 0));
            Function::arg_iterator args = mainFn->arg_begin();
            
            inputStream = &*(args++);
            inputStream->setName("input");
            fileSize = &*(args++);
            fileSize->setName("fileSize");
        } else {
            mainFn = cast<Function>(M->getOrInsertFunction("Main", resultTy, inputType, size_ty, outputType, nullptr));
            mainFn->setCallingConv(CallingConv::C);
            iBuilder->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", mainFn, 0));
            Function::arg_iterator args = mainFn->arg_begin();
            
            inputStream = &*(args++);
            inputStream->setName("input");
            fileSize = &*(args++);
            fileSize->setName("fileSize");
            outputStream = &*(args++);
            outputStream->setName("output");
        }
    } 
#endif
    if (CPU_Only){
        mainFn = cast<Function>(M->getOrInsertFunction("Main", resultTy, inputType, int64ty, int64ty, nullptr));
        mainFn->setCallingConv(CallingConv::C);
        iBuilder->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", mainFn, 0));
        Function::arg_iterator args = mainFn->arg_begin();
        
        inputStream = &*(args++);
        inputStream->setName("input");
        fileSize = &*(args++);
        fileSize->setName("fileSize");
        fileIdx = &*(args++);
        fileIdx->setName("fileIdx");
    }
       
    ExternalFileBuffer ByteStream(iBuilder, StreamSetType(iBuilder, 1, 8));
    CircularBuffer BasisBits(iBuilder, StreamSetType(iBuilder, 8, 1), segmentSize * bufferSegments);

    kernel::s2pKernel  s2pk(iBuilder);
    s2pk.generateKernel({&ByteStream}, {&BasisBits});
    
    re_ast = re::regular_expression_passes(re_ast);
    pablo::PabloFunction * function = re::re2pablo_compiler(encodingBits, re_ast, CountOnly);
    pablo_function_passes(function);

    ByteStream.setStreamSetBuffer(inputStream, fileSize);
    BasisBits.allocateBuffer();

    Value * s2pInstance = s2pk.createInstance({});
 
    Type * pthreadTy = int64ty;
    FunctionType * funVoidPtrVoidTy = FunctionType::get(voidTy, int8PtrTy, false);   
    
    Function * pthreadCreateFunc = cast<Function>(M->getOrInsertFunction("pthread_create",
                                        int32ty, 
                                        pthreadTy->getPointerTo(), 
                                        voidPtrTy, 
                                        static_cast<Type *>(funVoidPtrVoidTy)->getPointerTo(),
                                        voidPtrTy, nullptr));
    pthreadCreateFunc->setCallingConv(llvm::CallingConv::C);
    Function * pthreadJoinFunc = cast<Function>(M->getOrInsertFunction("pthread_join", 
                                        int32ty, 
                                        pthreadTy, 
                                        PointerType::get(int8PtrTy, 0), nullptr));
    pthreadJoinFunc->setCallingConv(llvm::CallingConv::C);

    Function * pthreadExitFunc = cast<Function>(M->getOrInsertFunction("pthread_exit", 
                                        voidTy, 
                                        voidPtrTy, nullptr));
    pthreadExitFunc->addFnAttr(llvm::Attribute::NoReturn);
    pthreadExitFunc->setCallingConv(llvm::CallingConv::C);

    if (CountOnly) {
        pablo::PabloKernel icgrepK(iBuilder, "icgrep", function);
        icgrepK.generateKernel({&BasisBits}, {});       
        Value * icgrepInstance = icgrepK.createInstance({});
        if (pipelineParallel){
            generatePipelineParallel(iBuilder, {&s2pk, &icgrepK}, {s2pInstance, icgrepInstance});
        } else if (segmentPipelineParallel){
            generateSegmentParallelPipeline(iBuilder, {&s2pk, &icgrepK}, {s2pInstance, icgrepInstance}, fileSize);
        } else {
            generatePipelineLoop(iBuilder, {&s2pk, &icgrepK}, {s2pInstance, icgrepInstance}, fileSize);
        }

        Value * matchCount = icgrepK.createGetAccumulatorCall(icgrepInstance, "matchedLineCount");

        iBuilder->CreateRet(matchCount);

    } else {
#ifdef CUDA_ENABLED 
        if (codegen::NVPTX){
            ExternalFileBuffer MatchResults(iBuilder, StreamSetType(iBuilder,2, i1), addrSpace);
            MatchResults.setStreamSetBuffer(outputStream, fileSize);

            pablo::PabloKernel  icgrepK(iBuilder, "icgrep", function, {});
            icgrepK.generateKernel({&BasisBits},  {&MatchResults});
            Value * icgrepInstance = icgrepK.createInstance({});

            generatePipelineLoop(iBuilder, {&s2pk, &icgrepK}, {s2pInstance, icgrepInstance}, fileSize);

        }
#endif
        if (CPU_Only) {
            CircularBuffer MatchResults(iBuilder, StreamSetType(iBuilder, 2, 1), segmentSize * bufferSegments);
            MatchResults.allocateBuffer();

            pablo::PabloKernel  icgrepK(iBuilder, "icgrep", function);
            icgrepK.generateKernel({&BasisBits}, {&MatchResults});
            Value * icgrepInstance = icgrepK.createInstance({});

            kernel::scanMatchKernel scanMatchK(iBuilder, mIsNameExpression);
            scanMatchK.generateKernel({&MatchResults}, {});                
            Value * scanMatchInstance = scanMatchK.createInstance({iBuilder->CreateBitCast(inputStream, int8PtrTy), fileSize, fileIdx});

            if (pipelineParallel){
                generatePipelineParallel(iBuilder, {&s2pk, &icgrepK, &scanMatchK}, {s2pInstance, icgrepInstance, scanMatchInstance});
            } else if (segmentPipelineParallel){
                generateSegmentParallelPipeline(iBuilder, {&s2pk, &icgrepK, &scanMatchK}, {s2pInstance, icgrepInstance, scanMatchInstance}, fileSize);
            }  else{
                generatePipelineLoop(iBuilder, {&s2pk, &icgrepK, &scanMatchK}, {s2pInstance, icgrepInstance, scanMatchInstance}, fileSize);
            }
        }

        iBuilder->CreateRetVoid();

    }

#ifdef CUDA_ENABLED 
    Function * mainCPUFn = nullptr;
    if(codegen::NVPTX){
        Function * kernelFunction = generateGPUKernel(M, iBuilder, CountOnly);
        MDNode * Node = MDNode::get(M->getContext(),
                                    {llvm::ValueAsMetadata::get(kernelFunction),
                                     MDString::get(M->getContext(), "kernel"), 
                                     ConstantAsMetadata::get(ConstantInt::get(iBuilder->getInt32Ty(), 1))});
        NamedMDNode *NMD = M->getOrInsertNamedMetadata("nvvm.annotations");
        NMD->addOperand(Node);
   
        Compile2PTX(M, IRFilename, PTXFilename);
        mainCPUFn = generateCPUKernel(cpuM, CPUBuilder, mIsNameExpression);
        if (CountOnly) return;
    }
#endif
    
     
    mEngine = JIT_to_ExecutionEngine(cpuM);
    ApplyObjectCache(mEngine);
    icgrep_Linking(cpuM, mEngine);

#ifndef NDEBUG
    verifyModule(*M, &dbgs());
#endif

    mEngine->finalizeObject();
    delete iBuilder;
    
    if (CountOnly) {
        mGrepFunction_CountOnly = reinterpret_cast<GrepFunctionType_CountOnly>(mEngine->getPointerToFunction(mainFn));
    } else {
#ifdef CUDA_ENABLED 
        if(codegen::NVPTX){
            mGrepFunction_CPU = reinterpret_cast<GrepFunctionType_CPU>(mEngine->getPointerToFunction(mainCPUFn));
        }
#endif
        if (CPU_Only) {
            mGrepFunction = reinterpret_cast<GrepFunctionType>(mEngine->getPointerToFunction(mainFn));
        }
    }

}

re::CC *  GrepEngine::grepCodepoints() {

    setParsedCodePointSet();
    char * mFileBuffer = getUnicodeNameDataPtr();
    size_t mFileSize = getUnicodeNameDataSize();

    mGrepFunction(mFileBuffer, mFileSize, 0);

    return getParsedCodePointSet();
}

GrepEngine::~GrepEngine() {
//    delete mEngine;
}

static int * total_count;
static std::stringstream * resultStrs = nullptr;
static std::vector<std::string> inputFiles;

void initResult(std::vector<std::string> filenames){
    const int n = filenames.size();
    if (n > 1) {
        ShowFileNames = true;
    }
    inputFiles = filenames;
    resultStrs = new std::stringstream[n];
    total_count = new int[n];
    for (unsigned i = 0; i < inputFiles.size(); ++i){
        total_count[i] = 0;
    }
    
}

extern "C" {
    void wrapped_report_match(size_t lineNum, size_t line_start, size_t line_end, const char * buffer, size_t filesize, int fileIdx) {
        int index = isUTF_16 ? 2 : 1;
        int idx = fileIdx;
          
        if (ShowFileNames) {
            resultStrs[idx] << inputFiles[idx] << ':';
        }
        if (ShowLineNumbers) {
            resultStrs[idx] << lineNum << ":";
        }
        
        if ((!isUTF_16 && buffer[line_start] == 0xA) && (line_start != line_end)) {
            // The line "starts" on the LF of a CRLF.  Really the end of the last line.
            line_start++;
        }
        if (((isUTF_16 && buffer[line_start] == 0x0) && buffer[line_start + 1] == 0xA) && (line_start != line_end)) {
            // The line "starts" on the LF of a CRLF.  Really the end of the last line.
            line_start += 2;
        }
        if (line_end == filesize) {
            // The match position is at end-of-file.   We have a final unterminated line.
            resultStrs[idx].write(&buffer[line_start * index], (line_end - line_start) * index);
            if (NormalizeLineBreaks) {
                resultStrs[idx] << '\n';  // terminate it
            }
            return;
        }
        unsigned char end_byte = (unsigned char)buffer[line_end]; 
        unsigned char penult_byte = (unsigned char)(buffer[line_end - 1]);
        if (NormalizeLineBreaks) {
            if (end_byte == 0x85) {
                // Line terminated with NEL, on the second byte.  Back up 1.
                line_end--;
            } else if (end_byte > 0xD) {
                // Line terminated with PS or LS, on the third byte.  Back up 2.
                isUTF_16 ? line_end-- : line_end -= 2;
            }
            resultStrs[idx].write(&buffer[line_start * index], (line_end - line_start) * index);
            resultStrs[idx] << '\n';
        }
        else {   
            if ((!isUTF_16 && end_byte == 0x0D) || (isUTF_16 && (end_byte == 0x0D && penult_byte == 0x0))) {
                // Check for line_end on first byte of CRLF;  note that we don't
                // want to access past the end of buffer.
                if (line_end + 1 < filesize) {
                    if (!isUTF_16 && buffer[line_end + 1] == 0x0A) {
                        // Found CRLF; preserve both bytes.
                        line_end++;
                    }
                    if (isUTF_16 && buffer[line_end + 1] == 0x0 && buffer[line_end + 2] == 0x0A) {
                        // Found CRLF; preserve both bytes.
                        line_end += 2;
                    }
                }
            }
            resultStrs[idx].write(&buffer[line_start * index], (line_end - line_start + 1) * index);
        }
    }
}

void PrintResult(bool CountOnly, std::vector<size_t> & total_CountOnly){
    
    if(CountOnly){
        if (!ShowFileNames) {
            for (unsigned i = 0; i < inputFiles.size(); ++i){
                std::cout << total_CountOnly[i] << std::endl;
            }
        }
        else {
            for (unsigned i = 0; i < inputFiles.size(); ++i){
                std::cout << inputFiles[i] << ':' << total_CountOnly[i] << std::endl;
            };
        }
        return;
    }
    
    for (unsigned i = 0; i < inputFiles.size(); ++i){
        std::cout << resultStrs[i].str();
    }
}

re::CC * parsedCodePointSet;

extern "C" {
    void insert_codepoints(size_t lineNum, size_t line_start, size_t line_end, const char * buffer) {
        re::codepoint_t c = 0;
        ssize_t line_pos = line_start;
        while (isxdigit(buffer[line_pos])) {
            if (isdigit(buffer[line_pos])) {
                c = (c << 4) | (buffer[line_pos] - '0');
            }
            else {
                c = (c << 4) | (tolower(buffer[line_pos]) - 'a' + 10);
            }
            line_pos++;
        }
        assert(((line_pos - line_start) >= 4) && ((line_pos - line_start) <= 6)); // UCD format 4 to 6 hex digits.       
        parsedCodePointSet->insert(c);
    }
}

void setParsedCodePointSet(){
    parsedCodePointSet = re::makeCC();
}

re::CC * getParsedCodePointSet(){
    return parsedCodePointSet;
}


void icgrep_Linking(Module * m, ExecutionEngine * e) {
    Module::FunctionListType & fns = m->getFunctionList();
    for (Module::FunctionListType::iterator it = fns.begin(), it_end = fns.end(); it != it_end; ++it) {
        std::string fnName = it->getName().str();
        if (fnName == "s2p_block") continue;
        if (fnName == "process_block") continue;
        if (fnName == "process_block_initialize_carries") continue;
        
        if (fnName == "wrapped_report_match") {
            e->addGlobalMapping(cast<GlobalValue>(it), (void *)&wrapped_report_match);
        }
        if (fnName == "insert_codepoints") {
            e->addGlobalMapping(cast<GlobalValue>(it), (void *)&insert_codepoints);
        }
#ifndef DISABLE_PREGENERATED_UCD_FUNCTIONS
        else {
            const UCD::ExternalProperty & ep = UCD::resolveExternalProperty(fnName);
            e->addGlobalMapping(cast<GlobalValue>(it), std::get<0>(ep));
        }
#endif
    }
}

