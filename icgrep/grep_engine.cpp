/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "grep_engine.h"
#include <llvm/IR/Module.h>
#include <llvm/ExecutionEngine/MCJIT.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Support/CommandLine.h>
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
#include <IR_Gen/idisa_builder.h>
#include <IR_Gen/idisa_target.h>
#include <UCD/UnicodeNameData.h>
#include <UCD/resolve_properties.h>
#include <kernels/cc_kernel.h>
#include <kernels/linebreak_kernel.h>
#include <kernels/streams_merge.h>
#include <kernels/match_count.h>
#include <kernels/pipeline.h>
#include <kernels/mmap_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/scanmatchgen.h>
#include <kernels/streamset.h>
#include <kernels/interface.h>
#include <pablo/pablo_compiler.h>
#include <pablo/pablo_kernel.h>
#include <pablo/pablo_toolchain.h>
#include <re/re_cc.h>
#include <re/re_toolchain.h>
#include <toolchain.h>
#include <iostream>
#include <sstream>
#ifdef CUDA_ENABLED
#include <IR_Gen/CudaDriver.h>
#include "preprocess.cpp"
#endif
#include <util/aligned_allocator.h>


using namespace parabix;
using namespace llvm;

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

static re::CC * parsedCodePointSet = nullptr;
static std::vector<std::string> parsedPropertyValues;

#ifdef CUDA_ENABLED 
int blockNo = 0;
size_t * startPoints = nullptr;
size_t * accumBytes = nullptr;
#endif

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
                codegen::BlockSize = 128;
		char * LineBreak;
                if (posix_memalign((void**)&LineBreak, 32, fileSize)) {
                    std::cerr << "Cannot allocate memory for linebreak.\n";
                    exit(-1);
                }
                std::vector<size_t> LFPositions = preprocess(fileBuffer, fileSize, LineBreak);

                const unsigned numOfGroups = codegen::GroupNum;
                if (posix_memalign((void**)&startPoints, 8, (numOfGroups+1)*sizeof(size_t)) ||
                    posix_memalign((void**)&accumBytes, 8, (numOfGroups+1)*sizeof(size_t))) {
                    std::cerr << "Cannot allocate memory for startPoints or accumBytes.\n";
                    exit(-1);
                }

                ulong * rslt = RunPTX(PTXFilename, fileBuffer, fileSize, CountOnly, LFPositions, startPoints, accumBytes);
                if (CountOnly){
                    exit(0);
                }
                else{
                    size_t intputSize = startPoints[numOfGroups]-accumBytes[numOfGroups]+accumBytes[numOfGroups-1];
                    mGrepFunction_CPU((char *)rslt, LineBreak, fileBuffer, intputSize, fileIdx);
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


Function * generateGPUKernel(Module * m, IDISA::IDISA_Builder * iBuilder, bool CountOnly){
    Type * const int64ty = iBuilder->getInt64Ty();
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const int32ty = iBuilder->getInt32Ty();
    Type * const sizeTyPtr = PointerType::get(size_ty, 1);
    Type * const int64tyPtr = PointerType::get(int64ty, 1);
    Type * const inputType = PointerType::get(iBuilder->getInt8Ty(), 1);
    Type * const resultTy = iBuilder->getVoidTy();
    Function * kernelFunc = cast<Function>(m->getOrInsertFunction("GPU_Main", resultTy, inputType, sizeTyPtr, sizeTyPtr, int64tyPtr, nullptr));
    kernelFunc->setCallingConv(CallingConv::C);
    Function::arg_iterator args = kernelFunc->arg_begin();

    Value * const inputPtr = &*(args++);
    inputPtr->setName("inputPtr");
    Value * const startPointsPtr = &*(args++);
    startPointsPtr->setName("startPointsPtr");
    Value * const bufferSizesPtr = &*(args++);
    bufferSizesPtr->setName("bufferSizesPtr");
    Value * const outputPtr = &*(args++);
    outputPtr->setName("resultPtr");

    BasicBlock * entryBlock = BasicBlock::Create(m->getContext(), "entry", kernelFunc, 0);
    iBuilder->SetInsertPoint(entryBlock);

    Function * tidFunc = m->getFunction("llvm.nvvm.read.ptx.sreg.tid.x");
    Value * tid = iBuilder->CreateCall(tidFunc);
    Function * bidFunc = cast<Function>(m->getOrInsertFunction("llvm.nvvm.read.ptx.sreg.ctaid.x", int32ty, nullptr));
    Value * bid = iBuilder->CreateCall(bidFunc);

    Value * startPoint = iBuilder->CreateLoad(iBuilder->CreateGEP(startPointsPtr, bid));

    Function * mainFunc = m->getFunction("Main");
    Value * startBlock = iBuilder->CreateUDiv(startPoint, ConstantInt::get(int64ty, iBuilder->getBitBlockWidth()));
    Type * const inputStreamType = PointerType::get(ArrayType::get(ArrayType::get(iBuilder->getBitBlockType(), 8), 1), 1);    
    Value * inputStreamPtr = iBuilder->CreateGEP(iBuilder->CreateBitCast(inputPtr, inputStreamType), startBlock);
    Value * inputStream = iBuilder->CreateGEP(inputStreamPtr, tid);
    Value * bufferSize = iBuilder->CreateLoad(iBuilder->CreateGEP(bufferSizesPtr, bid));

    if (CountOnly) {
        Value * strideBlocks = ConstantInt::get(int32ty, iBuilder->getStride() / iBuilder->getBitBlockWidth());
        Value * outputThreadPtr = iBuilder->CreateGEP(outputPtr, iBuilder->CreateAdd(iBuilder->CreateMul(bid, strideBlocks), tid));
        Value * result = iBuilder->CreateCall(mainFunc, {inputStream, bufferSize});
        iBuilder->CreateStore(result, outputThreadPtr);
    } else {
        Type * const outputStremType = PointerType::get(ArrayType::get(iBuilder->getBitBlockType(), 1), 1);
        Value * outputStreamPtr = iBuilder->CreateGEP(iBuilder->CreateBitCast(outputPtr, outputStremType), startBlock);
        Value * outputStream = iBuilder->CreateGEP(outputStreamPtr, tid);
        iBuilder->CreateCall(mainFunc, {inputStream, bufferSize, outputStream});
    }    

    iBuilder->CreateRetVoid();
    return kernelFunc;
}

Function * generateCPUKernel(Module * m, IDISA::IDISA_Builder * iBuilder, GrepType grepType){
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();
    Type * const rsltType = PointerType::get(ArrayType::get(iBuilder->getBitBlockType(), 1), 0);
    Function * const mainCPUFn = cast<Function>(m->getOrInsertFunction("CPU_Main", iBuilder->getVoidTy(), rsltType, rsltType, int8PtrTy, size_ty, size_ty, nullptr));
    mainCPUFn->setCallingConv(CallingConv::C);
    iBuilder->SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", mainCPUFn, 0));
    Function::arg_iterator args = mainCPUFn->arg_begin();
    
    Value * const rsltStream = &*(args++);
    rsltStream->setName("rslt");
    Value * const lbStream = &*(args++);
    lbStream->setName("lb");
    Value * const inputStream = &*(args++);
    inputStream->setName("input");
    Value * const fileSize = &*(args++);
    fileSize->setName("fileSize");
    Value * const fileIdx = &*(args++);
    fileIdx->setName("fileIdx");

    const unsigned segmentSize = codegen::SegmentSize;
    
    ExternalFileBuffer MatchResults(iBuilder, iBuilder->getStreamSetTy(1, 1));
    MatchResults.setStreamSetBuffer(rsltStream, fileSize);

    kernel::MMapSourceKernel mmapK1(iBuilder, segmentSize); 
    mmapK1.setName("mmap1");
    mmapK1.generateKernel({}, {&MatchResults});
    mmapK1.setInitialArguments({fileSize});

    ExternalFileBuffer LineBreak(iBuilder, iBuilder->getStreamSetTy(1, 1));
    LineBreak.setStreamSetBuffer(lbStream, fileSize);
    
    kernel::MMapSourceKernel mmapK2(iBuilder, segmentSize); 
    mmapK2.setName("mmap2");
    mmapK2.generateKernel({}, {&LineBreak});
    mmapK2.setInitialArguments({fileSize});

    kernel::ScanMatchKernel scanMatchK(iBuilder, grepType);
    scanMatchK.generateKernel({&MatchResults, &LineBreak}, {});
    scanMatchK.setInitialArguments({iBuilder->CreateBitCast(inputStream, int8PtrTy), fileSize, fileIdx});
    
    generatePipelineLoop(iBuilder, {&mmapK1, &mmapK2, &scanMatchK});
    iBuilder->CreateRetVoid();

    return mainCPUFn;
}

void GrepEngine::multiGrepCodeGen(std::string moduleName, std::vector<re::RE *> REs, bool CountOnly, bool UTF_16, GrepType grepType) {

    isUTF_16 = UTF_16;
    Module * M = new Module(moduleName + ":icgrep", getGlobalContext());;  
    IDISA::IDISA_Builder * iBuilder = IDISA::GetIDISA_Builder(M);; 

    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = segmentPipelineParallel ? (codegen::BufferSegments * codegen::ThreadNum) : codegen::BufferSegments;
    const unsigned encodingBits = UTF_16 ? 16 : 8;

    mGrepType = grepType;

    Type * const size_ty = iBuilder->getSizeTy();
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();
    Type * const inputType = PointerType::get(ArrayType::get(ArrayType::get(iBuilder->getBitBlockType(), encodingBits), 1), 0);
    Type * const resultTy = CountOnly ? size_ty : iBuilder->getVoidTy();

    Function * mainFn = cast<Function>(M->getOrInsertFunction("Main", resultTy, inputType, size_ty, size_ty, nullptr));
    mainFn->setCallingConv(CallingConv::C);
    iBuilder->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", mainFn, 0));
    Function::arg_iterator args = mainFn->arg_begin();
    
    Value * inputStream = &*(args++);
    inputStream->setName("input");
    Value * fileSize = &*(args++);
    fileSize->setName("fileSize");
    Value * fileIdx = &*(args++);
    fileIdx->setName("fileIdx");

    ExternalFileBuffer ByteStream(iBuilder, iBuilder->getStreamSetTy(1, 8));    
    CircularBuffer BasisBits(iBuilder, iBuilder->getStreamSetTy(8), segmentSize * bufferSegments);
    ByteStream.setStreamSetBuffer(inputStream, fileSize);
    BasisBits.allocateBuffer();
    
    kernel::MMapSourceKernel mmapK(iBuilder, segmentSize); 
    mmapK.generateKernel({}, {&ByteStream});
    mmapK.setInitialArguments({fileSize});

    kernel::S2PKernel  s2pk(iBuilder);
    s2pk.generateKernel({&ByteStream}, {&BasisBits});
   
    std::vector<pablo::PabloKernel *> icgrepKs;
    std::vector<StreamSetBuffer *> MatchResultsBufs;

    for(unsigned i=0; i<REs.size(); i++){   
        pablo::PabloKernel * icgrepK = new pablo::PabloKernel(iBuilder, "icgrep"+std::to_string(i), {Binding{iBuilder->getStreamSetTy(8), "basis"}, Binding{iBuilder->getStreamSetTy(1, 1), "linebreak"}});
        re::re2pablo_compiler(icgrepK, re::regular_expression_passes(REs[i]), false);
        pablo_function_passes(icgrepK);
        icgrepKs.push_back(icgrepK);
        CircularBuffer * MatchResults = new CircularBuffer(iBuilder, iBuilder->getStreamSetTy(2, 1), segmentSize * bufferSegments);       
        MatchResults->allocateBuffer();
        MatchResultsBufs.push_back(MatchResults);
    }   

    std::vector<kernel::KernelBuilder *> KernelList;
    KernelList.push_back(&mmapK);
    KernelList.push_back(&s2pk);

    CircularBuffer mergedResults(iBuilder, iBuilder->getStreamSetTy(1, 1), segmentSize * bufferSegments);
    mergedResults.allocateBuffer();

    kernel::StreamsMerge streamsMergeK(iBuilder, 1, REs.size());
    streamsMergeK.generateKernel(MatchResultsBufs, {&mergedResults});

    kernel::LineBreakKernelBuilder linebreakK(iBuilder, "lb", encodingBits);
    CircularBuffer LineBreakStream(iBuilder, iBuilder->getStreamSetTy(1, 1), segmentSize * bufferSegments);
    LineBreakStream.allocateBuffer();
    linebreakK.generateKernel({&BasisBits}, {&LineBreakStream});
    
    KernelList.push_back(&linebreakK);
    for(unsigned i=0; i<REs.size(); i++){
        icgrepKs[i]->generateKernel({&BasisBits, &LineBreakStream}, {MatchResultsBufs[i]});
        KernelList.push_back(icgrepKs[i]);
    }
    KernelList.push_back(&streamsMergeK);

    if (CountOnly) {
        kernel::MatchCount matchCountK(iBuilder);
        matchCountK.generateKernel({&mergedResults}, {});  

        KernelList.push_back(&matchCountK);  

        if (pipelineParallel){
            generatePipelineParallel(iBuilder, KernelList);
        } else if (segmentPipelineParallel){
            generateSegmentParallelPipeline(iBuilder, KernelList);
        }  else{
            generatePipelineLoop(iBuilder, KernelList);
        }
        iBuilder->CreateRet(matchCountK.getScalarField(matchCountK.getInstance(), "matchedLineCount"));

    } else {
        kernel::ScanMatchKernel scanMatchK(iBuilder, mGrepType);
        scanMatchK.generateKernel({&mergedResults, &LineBreakStream}, {});                
        scanMatchK.setInitialArguments({iBuilder->CreateBitCast(inputStream, int8PtrTy), fileSize, fileIdx});

        KernelList.push_back(&scanMatchK);

        if (pipelineParallel) {
            generatePipelineParallel(iBuilder, KernelList);
        } else if (segmentPipelineParallel) {
            generateSegmentParallelPipeline(iBuilder, KernelList);
        } else {
            generatePipelineLoop(iBuilder, KernelList);
        }
        
        iBuilder->CreateRetVoid();
    }
    
    mEngine = JIT_to_ExecutionEngine(M);
    ApplyObjectCache(mEngine);
    icgrep_Linking(M, mEngine);

#ifndef NDEBUG
    verifyModule(*M, &dbgs());
#endif

    mEngine->finalizeObject();
    delete iBuilder;
    
    if (CountOnly) {
        mGrepFunction_CountOnly = reinterpret_cast<GrepFunctionType_CountOnly>(mEngine->getPointerToFunction(mainFn));
    } else {
        mGrepFunction = reinterpret_cast<GrepFunctionType>(mEngine->getPointerToFunction(mainFn));
    }

}

void GrepEngine::grepCodeGen(std::string moduleName, re::RE * re_ast, bool CountOnly, bool UTF_16, GrepType grepType) {
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

    Module * cpuM = new Module(moduleName + ":cpu", getGlobalContext());
    IDISA::IDISA_Builder * CPUBuilder = IDISA::GetIDISA_Builder(cpuM);

    if (CPU_Only) {
        M = cpuM;
        iBuilder = CPUBuilder;
    }

    // segment size made availabe for each call to the mmap source kernel
    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = segmentPipelineParallel ? (codegen::BufferSegments * codegen::ThreadNum) : codegen::BufferSegments;
    const unsigned encodingBits = UTF_16 ? 16 : 8;

    mGrepType = grepType;

    Type * const size_ty = iBuilder->getSizeTy();
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();
    Type * const inputType = PointerType::get(ArrayType::get(ArrayType::get(iBuilder->getBitBlockType(), encodingBits), 1), addrSpace);
    Type * const resultTy = CountOnly ? size_ty : iBuilder->getVoidTy();

    Function * mainFn = nullptr;
    Value * inputStream = nullptr;
    Value * fileSize = nullptr;
    Value * fileIdx = nullptr;

#ifdef CUDA_ENABLED   
    Value * outputStream = nullptr;
    Type * const outputType = PointerType::get(ArrayType::get(iBuilder->getBitBlockType(), 1), addrSpace);
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
    if (CPU_Only) {
        mainFn = cast<Function>(M->getOrInsertFunction("Main", resultTy, inputType, size_ty, size_ty, nullptr));
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
       
    ExternalFileBuffer ByteStream(iBuilder, iBuilder->getStreamSetTy(1, 8));
    
    kernel::MMapSourceKernel mmapK(iBuilder, segmentSize); 
    mmapK.generateKernel({}, {&ByteStream});
    mmapK.setInitialArguments({fileSize});
    
    CircularBuffer BasisBits(iBuilder, iBuilder->getStreamSetTy(8), segmentSize * bufferSegments);

    kernel::S2PKernel  s2pk(iBuilder);
    s2pk.generateKernel({&ByteStream}, {&BasisBits});
    
    kernel::LineBreakKernelBuilder linebreakK(iBuilder, "lb", encodingBits);
    CircularBuffer LineBreakStream(iBuilder, iBuilder->getStreamSetTy(1, 1), segmentSize * bufferSegments);

    linebreakK.generateKernel({&BasisBits}, {&LineBreakStream});
    LineBreakStream.allocateBuffer();

    pablo::PabloKernel icgrepK(iBuilder, "icgrep", {Binding{iBuilder->getStreamSetTy(8), "basis"}, Binding{iBuilder->getStreamSetTy(1, 1), "linebreak"}});
    re::re2pablo_compiler(&icgrepK, re::regular_expression_passes(re_ast), CountOnly);
    pablo_function_passes(&icgrepK);

    ByteStream.setStreamSetBuffer(inputStream, fileSize);
    BasisBits.allocateBuffer();

    if (CountOnly) {
        icgrepK.generateKernel({&BasisBits, &LineBreakStream}, {});
        if (pipelineParallel) {
            generatePipelineParallel(iBuilder, {&mmapK, &s2pk, &linebreakK, &icgrepK});
        } else if (segmentPipelineParallel) {
            generateSegmentParallelPipeline(iBuilder, {&mmapK, &s2pk, &linebreakK, &icgrepK});
        } else {
            generatePipelineLoop(iBuilder, {&mmapK, &s2pk, &linebreakK, &icgrepK});
        }
        iBuilder->CreateRet(icgrepK.createGetAccumulatorCall(icgrepK.getInstance(), "matchedLineCount"));
    } else {
#ifdef CUDA_ENABLED 
        if (codegen::NVPTX){
            ExternalFileBuffer MatchResults(iBuilder, iBuilder->getStreamSetTy(1, 1), addrSpace);
            MatchResults.setStreamSetBuffer(outputStream, fileSize);

            icgrepK.generateKernel({&BasisBits, &LineBreakStream},  {&MatchResults});
            generatePipelineLoop(iBuilder, {&mmapK, &s2pk, &linebreakK, &icgrepK});

        }
#endif
        if (CPU_Only) {
            CircularBuffer MatchResults(iBuilder, iBuilder->getStreamSetTy(1, 1), segmentSize * bufferSegments);
            MatchResults.allocateBuffer();

            icgrepK.generateKernel({&BasisBits, &LineBreakStream}, {&MatchResults});

            kernel::ScanMatchKernel scanMatchK(iBuilder, mGrepType);
            scanMatchK.generateKernel({&MatchResults, &LineBreakStream}, {});                
            scanMatchK.setInitialArguments({iBuilder->CreateBitCast(inputStream, int8PtrTy), fileSize, fileIdx});
	    
            if (pipelineParallel) {
                generatePipelineParallel(iBuilder, {&mmapK, &s2pk, &linebreakK, &icgrepK, &scanMatchK});
            } else if (segmentPipelineParallel) {
                generateSegmentParallelPipeline(iBuilder, {&mmapK, &s2pk, &linebreakK, &icgrepK, &scanMatchK});
            } else {
                generatePipelineLoop(iBuilder, {&mmapK, &s2pk, &linebreakK, &icgrepK, &scanMatchK});
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
        mainCPUFn = generateCPUKernel(cpuM, CPUBuilder, mGrepType);
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

re::CC * GrepEngine::grepCodepoints() {
    parsedCodePointSet = re::makeCC();
    char * mFileBuffer = getUnicodeNameDataPtr();
    size_t mFileSize = getUnicodeNameDataSize();
    mGrepFunction(mFileBuffer, mFileSize, 0);
    return parsedCodePointSet;
}

const std::vector<std::string> & GrepEngine::grepPropertyValues(const std::string& propertyName) {
    enum { MaxSupportedVectorWidthInBytes = 32 };
    AlignedAllocator<char, MaxSupportedVectorWidthInBytes> alloc;
    parsedPropertyValues.clear();
    const std::string & str = UCD::getPropertyValueGrepString(propertyName);
    const auto n = str.length();
    // NOTE: MaxSupportedVectorWidthInBytes of trailing 0s are needed to prevent the grep function from
    // erroneously matching garbage data when loading the final partial block.
    char * aligned = alloc.allocate(n + MaxSupportedVectorWidthInBytes, 0);
    std::memcpy(aligned, str.data(), n);
    std::memset(aligned + n, 0, MaxSupportedVectorWidthInBytes);
    mGrepFunction(aligned, n, 0);
    alloc.deallocate(aligned, 0);
    return parsedPropertyValues;
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
        assert (buffer);
#ifdef CUDA_ENABLED 
    if (codegen::NVPTX){
        while(line_start>startPoints[blockNo]) blockNo++;
        line_start -= accumBytes[blockNo-1];
        line_end -= accumBytes[blockNo-1];
    }
#endif
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
        } else {
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

extern "C" {
    void insert_codepoints(size_t lineNum, size_t line_start, size_t line_end, const char * buffer) {
        assert (buffer);
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

extern "C" {
    void insert_property_values(size_t lineNum, size_t line_start, size_t line_end, const char * buffer) {
        parsedPropertyValues.emplace_back(buffer + line_start, buffer + line_end);
    }
}

void icgrep_Linking(Module * m, ExecutionEngine * e) {
    Module::FunctionListType & fns = m->getFunctionList();
    for (auto it = fns.begin(), it_end = fns.end(); it != it_end; ++it) {
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
        if (fnName == "insert_property_values") {
            e->addGlobalMapping(cast<GlobalValue>(it), (void *)&insert_property_values);
        }
    }
}

GrepEngine::GrepEngine()
: mGrepFunction(nullptr)
, mGrepFunction_CountOnly(nullptr)
, mGrepFunction_CPU(nullptr)
, mGrepType(GrepType::Normal)
, mEngine(nullptr) {

}

GrepEngine::~GrepEngine() {
    delete mEngine;
}
