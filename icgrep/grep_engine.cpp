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
#include <kernels/grep_kernel.h>
#include <kernels/linebreak_kernel.h>
#include <kernels/streams_merge.h>
#include <kernels/match_count.h>
#include <kernels/mmap_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/scanmatchgen.h>
#include <kernels/streamset.h>
#include <kernels/stdin_kernel.h>
#include <pablo/pablo_kernel.h>
#include <re/re_cc.h>
#include <re/re_toolchain.h>
#include <kernels/toolchain.h>
#include <iostream>
#include <sstream>
#include <cc/multiplex_CCs.h>

#include <llvm/Support/raw_ostream.h>
#include <sys/stat.h>


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

#ifdef CUDA_ENABLED
const auto IRFilename = "icgrep.ll";
const auto PTXFilename = "icgrep.ptx";
#endif

static re::CC * parsedCodePointSet = nullptr;

static std::vector<std::string> parsedPropertyValues;

#ifdef CUDA_ENABLED 
int blockNo = 0;
size_t * startPoints = nullptr;
size_t * accumBytes = nullptr;
#endif

void GrepEngine::doGrep(const std::string & fileName, const int fileIdx, bool CountOnly, std::vector<size_t> & total_CountOnly) {
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

void GrepEngine::doGrep(const int fileIdx, bool CountOnly, std::vector<size_t> & total_CountOnly) {
    if (CountOnly) {
        total_CountOnly[fileIdx] = mGrepFunction_CountOnly(nullptr, 0, fileIdx);
    } else {
        mGrepFunction(nullptr, 0, fileIdx);
    }
}

#ifdef CUDA_ENABLED
Function * generateGPUKernel(ParabixDriver & nvptxDriver, bool CountOnly){
    IDISA::IDISA_Builder * iBuilder = nvptxDriver.getIDISA_Builder();
    Module * m = iBuilder->getModule();
    Type * const int64ty = iBuilder->getInt64Ty();
    Type * const size_ty = iBuilder->getSizeTy();
    Type * const int32ty = iBuilder->getInt32Ty();
    Type * const sizeTyPtr = PointerType::get(size_ty, 1);
    Type * const int64tyPtr = PointerType::get(int64ty, 1);
    Type * const inputType = PointerType::get(iBuilder->getInt8Ty(), 1);
    Type * const resultTy = iBuilder->getVoidTy();
    Function * kernelFunc = cast<Function>(m->getOrInsertFunction("Main", resultTy, inputType, sizeTyPtr, sizeTyPtr, int64tyPtr, nullptr));
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

void generateCPUKernel(ParabixDriver & pxDriver, GrepType grepType){
    IDISA::IDISA_Builder * iBuilder = pxDriver.getIDISA_Builder();
    Module * m = iBuilder->getModule();

    Type * const size_ty = iBuilder->getSizeTy();
    Type * const int8PtrTy = iBuilder->getInt8PtrTy();
    Type * const rsltType = PointerType::get(ArrayType::get(iBuilder->getBitBlockType(), 1), 0);
    Function * const mainCPUFn = cast<Function>(m->getOrInsertFunction("Main", iBuilder->getVoidTy(), rsltType, rsltType, int8PtrTy, size_ty, size_ty, nullptr));
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
    
    ExternalFileBuffer InputStream(iBuilder, iBuilder->getStreamSetTy(1, 8));
    InputStream.setStreamSetBuffer(inputStream);

    ExternalFileBuffer MatchResults(iBuilder, iBuilder->getStreamSetTy(1, 1));
    MatchResults.setStreamSetBuffer(rsltStream);

    kernel::MMapSourceKernel mmapK0(iBuilder, segmentSize); 
    mmapK0.setName("mmap0");
    mmapK0.setInitialArguments({fileSize});
    pxDriver.addKernelCall(mmapK0, {}, {&InputStream});


    kernel::MMapSourceKernel mmapK1(iBuilder, segmentSize); 
    mmapK1.setName("mmap1");
    mmapK1.setInitialArguments({fileSize});
    pxDriver.addKernelCall(mmapK1, {}, {&MatchResults});

    ExternalFileBuffer LineBreak(iBuilder, iBuilder->getStreamSetTy(1, 1));
    LineBreak.setStreamSetBuffer(lbStream);
    
    kernel::MMapSourceKernel mmapK2(iBuilder, segmentSize); 
    mmapK2.setName("mmap2");
    mmapK2.setInitialArguments({fileSize});
    pxDriver.addKernelCall(mmapK2, {}, {&LineBreak});

    kernel::ScanMatchKernel scanMatchK(iBuilder, grepType, 8);
    scanMatchK.setInitialArguments({fileIdx});
    pxDriver.addKernelCall(scanMatchK, {&InputStream, &MatchResults, &LineBreak}, {});
    pxDriver.generatePipelineIR();
    iBuilder->CreateRetVoid();

    pxDriver.linkAndFinalize();
}
#endif

static int * total_count;
static std::stringstream * resultStrs = nullptr;
static std::vector<std::string> inputFiles;

void initFileResult(std::vector<std::string> filenames){
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

template<typename CodeUnit>
void wrapped_report_match(const size_t lineNum, size_t line_start, size_t line_end, const CodeUnit * const buffer, const size_t filesize, const size_t fileIdx) {
    assert (buffer);
    assert (line_start <= line_end);
    assert (line_end <= filesize);

    #ifdef CUDA_ENABLED
    if (codegen::NVPTX){
        while(line_start>startPoints[blockNo]) blockNo++;
        line_start -= accumBytes[blockNo-1];
        line_end -= accumBytes[blockNo-1];
    }
    #endif

    if (ShowFileNames) {
        resultStrs[fileIdx] << inputFiles[fileIdx] << ':';
    }
    if (ShowLineNumbers) {
        resultStrs[fileIdx] << lineNum << ":";
    }

    // If the line "starts" on the LF of a CRLF, it is actually the end of the last line.
    if ((buffer[line_start] == 0xA) && (line_start != line_end)) {
        ++line_start;
    }

    if (LLVM_UNLIKELY(line_end == filesize)) {
        // The match position is at end-of-file.   We have a final unterminated line.
        resultStrs[fileIdx].write((char *)&buffer[line_start], (line_end - line_start) * sizeof(CodeUnit));
        if (NormalizeLineBreaks) {
            resultStrs[fileIdx] << '\n';  // terminate it
        }
    } else {
        const auto end_byte = buffer[line_end];
        if (NormalizeLineBreaks) {
            if (LLVM_UNLIKELY(end_byte == 0x85)) {
                // Line terminated with NEL, on the second byte.  Back up 1.
                line_end -= 1;
            } else if (LLVM_UNLIKELY(end_byte > 0xD)) {
                // Line terminated with PS or LS, on the third byte.  Back up 2.
                line_end -= 2;
            }
            resultStrs[fileIdx].write((char *)&buffer[line_start], (line_end - line_start) * sizeof(CodeUnit));
            resultStrs[fileIdx] << '\n';
        } else {
            if (end_byte == 0x0D) {
                // Check for line_end on first byte of CRLF; we don't want to access past the end of buffer.
                if ((line_end + 1) < filesize) {
                    if (buffer[line_end + 1] == 0x0A) {
                        // Found CRLF; preserve both bytes.
                        ++line_end;
                    }
                }
            }
            resultStrs[fileIdx].write((char *)&buffer[line_start], (line_end - line_start + 1) * sizeof(CodeUnit));
        }
    }
}

void PrintResult(bool CountOnly, std::vector<size_t> & total_CountOnly){
    if (CountOnly) {
        if (!ShowFileNames) {
            for (unsigned i = 0; i < inputFiles.size(); ++i){
                std::cout << total_CountOnly[i] << std::endl;
            }
        } else {
            for (unsigned i = 0; i < inputFiles.size(); ++i){
                std::cout << inputFiles[i] << ':' << total_CountOnly[i] << std::endl;
            };
        }
    } else {
        for (unsigned i = 0; i < inputFiles.size(); ++i){
            std::cout << resultStrs[i].str();
        }
    }
}

void insert_codepoints(const size_t lineNum, const size_t line_start, const size_t line_end, const char * const buffer) {
    assert (buffer);
    assert (line_start <= line_end);
    re::codepoint_t c = 0;
    size_t line_pos = line_start;
    while (isxdigit(buffer[line_pos])) {
        assert (line_pos < line_end);
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

void insert_property_values(size_t lineNum, size_t line_start, size_t line_end, const char * buffer) {
    assert (line_start <= line_end);
    parsedPropertyValues.emplace_back(buffer + line_start, buffer + line_end);
}

inline void linkGrepFunction(ParabixDriver & pxDriver, const GrepType grepType, const bool UTF_16, kernel::KernelBuilder & kernel) {
    switch (grepType) {
        case GrepType::Normal:
            if (UTF_16) {
                pxDriver.addExternalLink(kernel, "matcher", &wrapped_report_match<uint16_t>);
            } else {
                pxDriver.addExternalLink(kernel, "matcher", &wrapped_report_match<uint8_t>);
            }
            break;
        case GrepType::NameExpression:
            pxDriver.addExternalLink(kernel, "matcher", &insert_codepoints);
            break;
        case GrepType::PropertyValue:
            pxDriver.addExternalLink(kernel, "matcher", &insert_property_values);
            break;
    }
}

void GrepEngine::grepCodeGen(std::string moduleName, re::RE * re_ast, const bool CountOnly, const bool UTF_16, const GrepType grepType, const bool usingStdIn) {
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
    ParabixDriver pxDriver(iBuilder);

    // segment size made availabe for each call to the mmap source kernel
    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments * codegen::ThreadNum;
    const unsigned encodingBits = UTF_16 ? 16 : 8;

    Type * const size_ty = iBuilder->getSizeTy();
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

    StreamSetBuffer * byteStream = nullptr;
    kernel::KernelBuilder * sourceK = nullptr;
    if (usingStdIn) {
        // TODO: use fstat(STDIN_FILENO) to see if we can mmap the stdin safely and avoid the calls to read
        byteStream = new ExtensibleBuffer(iBuilder, iBuilder->getStreamSetTy(1, 8), segmentSize);
        sourceK = new kernel::StdInKernel(iBuilder, segmentSize);
    } else {
        byteStream = new SourceFileBuffer(iBuilder, iBuilder->getStreamSetTy(1, 8));
        sourceK = new kernel::FileSourceKernel(iBuilder, inputStream->getType(), segmentSize);
        sourceK->setInitialArguments({inputStream, fileSize});
    }
    byteStream->allocateBuffer();
    pxDriver.addKernelCall(*sourceK, {}, {byteStream});

    CircularBuffer BasisBits(iBuilder, iBuilder->getStreamSetTy(8), segmentSize * bufferSegments);
    BasisBits.allocateBuffer();

    kernel::S2PKernel s2pk(iBuilder);
    pxDriver.addKernelCall(s2pk, {byteStream}, {&BasisBits});

    kernel::LineBreakKernelBuilder linebreakK(iBuilder, encodingBits);
    CircularBuffer LineBreakStream(iBuilder, iBuilder->getStreamSetTy(1, 1), segmentSize * bufferSegments);
    LineBreakStream.allocateBuffer();

    pxDriver.addKernelCall(linebreakK, {&BasisBits}, {&LineBreakStream});
   
    kernel::ICgrepKernelBuilder icgrepK(iBuilder, re_ast, CountOnly);

    if (CountOnly) {
       
        pxDriver.addKernelCall(icgrepK, {&BasisBits, &LineBreakStream}, {});

        pxDriver.generatePipelineIR();

        iBuilder->CreateRet(icgrepK.createGetAccumulatorCall(icgrepK.getInstance(), "matchedLineCount"));

        pxDriver.linkAndFinalize();

    } else {

        #ifdef CUDA_ENABLED
        if (codegen::NVPTX){
            ExternalFileBuffer MatchResults(iBuilder, iBuilder->getStreamSetTy(1, 1), addrSpace);
            MatchResults.setStreamSetBuffer(outputStream);

            pxDriver.addKernelCall(icgrepK, {&BasisBits, &LineBreakStream}, {&MatchResults});

            pxDriver.generatePipelineIR();

            iBuilder->CreateRetVoid();

            pxDriver.linkAndFinalize();
        }
        #endif

        if (CPU_Only) {

            CircularBuffer MatchResults(iBuilder, iBuilder->getStreamSetTy(1, 1), segmentSize * bufferSegments);
            MatchResults.allocateBuffer();

            pxDriver.addKernelCall(icgrepK, {&BasisBits, &LineBreakStream}, {&MatchResults});

            kernel::ScanMatchKernel scanMatchK(iBuilder, grepType, encodingBits);
            scanMatchK.setInitialArguments({fileIdx});

            pxDriver.addKernelCall(scanMatchK, {&MatchResults, &LineBreakStream, byteStream}, {});

            linkGrepFunction(pxDriver, grepType, UTF_16, scanMatchK);

            pxDriver.generatePipelineIR();

            iBuilder->CreateRetVoid();

            pxDriver.linkAndFinalize();
        }
    }

    #ifdef CUDA_ENABLED
    if(codegen::NVPTX){
        ParabixDriver nvptxDriver(iBuilder);
        Function * kernelFunction = generateGPUKernel(nvptxDriver, CountOnly);
        
        MDNode * Node = MDNode::get(M->getContext(),
                                    {llvm::ValueAsMetadata::get(kernelFunction),
                                     MDString::get(M->getContext(), "kernel"),
                                     ConstantAsMetadata::get(ConstantInt::get(iBuilder->getInt32Ty(), 1))});
        NamedMDNode *NMD = M->getOrInsertNamedMetadata("nvvm.annotations");
        NMD->addOperand(Node);

        Compile2PTX(M, IRFilename, PTXFilename);
        
        ParabixDriver pxDriver(CPUBuilder);
        generateCPUKernel(pxDriver, grepType);
        
        mGrepFunction_CPU = reinterpret_cast<GrepFunctionType_CPU>(pxDriver.getPointerToMain());
        if (CountOnly) return;
    }
    #endif

    delete iBuilder;
    delete sourceK;
    delete byteStream;

    if (CountOnly) {
        mGrepFunction_CountOnly = reinterpret_cast<GrepFunctionType_CountOnly>(pxDriver.getPointerToMain());
    } else {
        if (CPU_Only) {
            mGrepFunction = reinterpret_cast<GrepFunctionType>(pxDriver.getPointerToMain());
        }
    }
}



void GrepEngine::grepCodeGen(std::string moduleName, std::vector<re::RE *> REs, const bool CountOnly, const bool UTF_16, const GrepType grepType, const bool usingStdIn) {

    Module * M = new Module(moduleName + ":icgrep", getGlobalContext());;
    IDISA::IDISA_Builder * iBuilder = IDISA::GetIDISA_Builder(M);;
    ParabixDriver pxDriver(iBuilder);

    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments * codegen::ThreadNum;
    const unsigned encodingBits = UTF_16 ? 16 : 8;

    Type * const sizeTy = iBuilder->getSizeTy();
    Type * const inputType = PointerType::get(ArrayType::get(ArrayType::get(iBuilder->getBitBlockType(), encodingBits), 1), 0);
    Type * const resultTy = CountOnly ? sizeTy : iBuilder->getVoidTy();

    Function * mainFn = cast<Function>(M->getOrInsertFunction("Main", resultTy, inputType, sizeTy, sizeTy, nullptr));
    mainFn->setCallingConv(CallingConv::C);
    iBuilder->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", mainFn, 0));
    Function::arg_iterator args = mainFn->arg_begin();

    Value * inputStream = &*(args++);
    inputStream->setName("input");
    Value * fileSize = &*(args++);
    fileSize->setName("fileSize");
    Value * fileIdx = &*(args++);
    fileIdx->setName("fileIdx");

    StreamSetBuffer * byteStream = nullptr;
    kernel::KernelBuilder * sourceK = nullptr;
    if (usingStdIn) {
        // TODO: use fstat(STDIN_FILENO) to see if we can mmap the stdin safely and avoid the calls to read
        byteStream = new ExtensibleBuffer(iBuilder, iBuilder->getStreamSetTy(1, 8), segmentSize);
        sourceK = new kernel::StdInKernel(iBuilder, segmentSize);
    } else {
        byteStream = new SourceFileBuffer(iBuilder, iBuilder->getStreamSetTy(1, 8));
        sourceK = new kernel::FileSourceKernel(iBuilder, inputStream->getType(), segmentSize);
        sourceK->setInitialArguments({inputStream, fileSize});
    }
    byteStream->allocateBuffer();
    pxDriver.addKernelCall(*sourceK, {}, {byteStream});

    CircularBuffer BasisBits(iBuilder, iBuilder->getStreamSetTy(8), segmentSize * bufferSegments);
    BasisBits.allocateBuffer();

    kernel::S2PKernel s2pk(iBuilder);
    pxDriver.addKernelCall(s2pk, {byteStream}, {&BasisBits});

    kernel::LineBreakKernelBuilder linebreakK(iBuilder, encodingBits);
    CircularBuffer LineBreakStream(iBuilder, iBuilder->getStreamSetTy(1, 1), segmentSize * bufferSegments);
    LineBreakStream.allocateBuffer();
    pxDriver.addKernelCall(linebreakK, {&BasisBits}, {&LineBreakStream});

    std::vector<pablo::PabloKernel *> icgrepKs;
    std::vector<StreamSetBuffer *> MatchResultsBufs;

    for(unsigned i = 0; i < REs.size(); ++i){
        pablo::PabloKernel * const icgrepK = new kernel::ICgrepKernelBuilder(iBuilder, REs[i], false);
        CircularBuffer * const matchResults = new CircularBuffer(iBuilder, iBuilder->getStreamSetTy(1, 1), segmentSize * bufferSegments);
        matchResults->allocateBuffer();

        pxDriver.addKernelCall(*icgrepK, {&BasisBits, &LineBreakStream}, {matchResults});
        icgrepKs.push_back(icgrepK);
        MatchResultsBufs.push_back(matchResults);
    }

    CircularBuffer mergedResults(iBuilder, iBuilder->getStreamSetTy(1, 1), segmentSize * bufferSegments);
    mergedResults.allocateBuffer();

    kernel::StreamsMerge streamsMergeK(iBuilder, 1, REs.size());
    pxDriver.addKernelCall(streamsMergeK, MatchResultsBufs, {&mergedResults});

    if (CountOnly) {
        kernel::MatchCount matchCountK(iBuilder);
        pxDriver.addKernelCall(matchCountK, {&mergedResults}, {});
        pxDriver.generatePipelineIR();
        iBuilder->CreateRet(matchCountK.getScalarField("matchedLineCount"));
        pxDriver.linkAndFinalize();
    } else {
        kernel::ScanMatchKernel scanMatchK(iBuilder, grepType, encodingBits);
        scanMatchK.setInitialArguments({fileIdx});
        pxDriver.addKernelCall(scanMatchK, {&mergedResults, &LineBreakStream, byteStream}, {});
        linkGrepFunction(pxDriver, grepType, UTF_16, scanMatchK);
        pxDriver.generatePipelineIR();
        iBuilder->CreateRetVoid();
        pxDriver.linkAndFinalize();
    }

    delete iBuilder;
    delete sourceK;
    delete byteStream;
    for (StreamSetBuffer * buf : MatchResultsBufs) {
        delete buf;
    }

    if (CountOnly) {
        mGrepFunction_CountOnly = reinterpret_cast<GrepFunctionType_CountOnly>(pxDriver.getPointerToMain());
    } else {
        mGrepFunction = reinterpret_cast<GrepFunctionType>(pxDriver.getPointerToMain());
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

GrepEngine::GrepEngine()
: mGrepFunction(nullptr)
, mGrepFunction_CountOnly(nullptr)
#ifdef CUDA_ENABLED
, mGrepFunction_CPU(nullptr)
#endif
{

}
