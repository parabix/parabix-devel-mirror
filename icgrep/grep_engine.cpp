/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "grep_engine.h"
#include "grep_interface.h"
#include <llvm/IR/Module.h>
#include <boost/filesystem.hpp>
#include <UCD/UnicodeNameData.h>
#include <UCD/resolve_properties.h>
#include <kernels/charclasses.h>
#include <kernels/cc_kernel.h>
#include <kernels/grep_kernel.h>
#include <kernels/linebreak_kernel.h>
#include <kernels/streams_merge.h>
#include <kernels/source_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/scanmatchgen.h>
#include <kernels/streamset.h>
#include <kernels/until_n.h>
#include <kernels/kernel_builder.h>
#include <pablo/pablo_kernel.h>
#include <re/re_cc.h>
#include <re/re_toolchain.h>
#include <toolchain/toolchain.h>
#include <re/re_name_resolve.h>    
#include <re/re_collect_unicodesets.h>
#include <re/re_multiplex.h>
#include <toolchain/cpudriver.h>
#include <toolchain/NVPTXDriver.h>
#include <iostream>
#include <sstream>
#include <cc/multiplex_CCs.h>
#include <llvm/Support/raw_ostream.h>
#include <util/aligned_allocator.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <mutex>
#ifdef CUDA_ENABLED
#include <preprocess.cpp>
#include <IR_Gen/CudaDriver.h>
#endif

using namespace parabix;
using namespace llvm;

namespace grep {

static std::stringstream * resultStrs = nullptr;
static std::vector<std::string> inputFiles;
static std::vector<std::string> linePrefix;
static bool grepMatchFound;

size_t * startPoints = nullptr;
size_t * accumBytes = nullptr;


std::mutex count_mutex;
size_t fileCount;

// DoGrep thread function.
void *DoGrepThreadFunction(void *args)
{
    size_t fileIdx;
    grep::GrepEngine * grepEngine = (grep::GrepEngine *)args;

    count_mutex.lock();
    fileIdx = fileCount;
    fileCount++;
    count_mutex.unlock();

    while (fileIdx < inputFiles.size()) {
        size_t grepResult = grepEngine->doGrep(inputFiles[fileIdx], fileIdx);
        
        count_mutex.lock();
        if (grepResult > 0) grepMatchFound = true;
        fileIdx = fileCount;
        fileCount++;
        count_mutex.unlock();
        if (QuietMode && grepMatchFound) pthread_exit(nullptr);
    }

    pthread_exit(nullptr);
}

bool matchesNeedToBeMovedToEOL() {
    if ((Mode == QuietMode) | (Mode == FilesWithMatch) | (Mode == FilesWithoutMatch)) {
        return false;
    }
    else if (LineRegexpFlag) {
        return false;
    }
    // TODO: return false for other cases based on regexp analysis, e.g., regexp ends with $.
    return true;
}
    
void GrepEngine::doGrep(const std::string & fileName, std::string & PTXFilename) const{
#ifdef CUDA_ENABLED
    const bool CountOnly = true;
    boost::filesystem::path file(fileName);
    if (exists(file)) {
        if (is_directory(file)) {
            return;
        }
    } else {
        if (!NoMessagesFlag) {
            std::cerr << "Error: cannot open " << fileName << " for processing. Skipped.\n";
            return;
        }
    }

    const auto fileSize = file_size(file);
    
    if (fileSize > 0) {
        try {
            boost::iostreams::mapped_file_source source(fileName, fileSize, 0);
            char * fileBuffer = const_cast<char *>(source.data());
            
            codegen::BlockSize = 128;
            std::vector<size_t> LFPositions = preprocess(fileBuffer, fileSize);
            
            const unsigned numOfGroups = codegen::GroupNum;
            if (posix_memalign((void**)&startPoints, 8, (numOfGroups+1)*sizeof(size_t)) ||
                posix_memalign((void**)&accumBytes, 8, (numOfGroups+1)*sizeof(size_t))) {
                std::cerr << "Cannot allocate memory for startPoints or accumBytes.\n";
                exit(-1);
            }
            if(PTXFilename=="")
                PTXFilename = mGrepDriver->getBuilder()->getModule()->getModuleIdentifier() + ".ptx";
            RunPTX(PTXFilename, fileBuffer, fileSize, CountOnly, LFPositions, startPoints, accumBytes);
            source.close();
        } catch (std::exception & e) {
            if (!NoMessagesFlag) {
                std::cerr << "Boost mmap error: " + fileName + ": " + e.what() + " Skipped.\n";
                return;
            }
        }
    } else {
        std::cout << 0 << std::endl;
    }
#endif
}

uint64_t GrepEngine::doGrep(const std::string & fileName, const uint32_t fileIdx) const {
    struct stat sb;
    const int32_t fd = open(fileName.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        if (!NoMessagesFlag  && !(Mode == QuietMode)) {
            if (errno == EACCES) {
                resultStrs[fileIdx] << "icgrep: " << fileName << ": Permission denied.\n";
            }
            else if (errno == ENOENT) {
                resultStrs[fileIdx] << "icgrep: " << fileName << ": No such file.\n";
            }
            else {
                resultStrs[fileIdx] << "icgrep: " << fileName << ": Failed.\n";
            }
        }
        return 0;
    }
    if (stat(fileName.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
        if (!NoMessagesFlag  && !(Mode == QuietMode)) {
            resultStrs[fileIdx] << "icgrep: " << fileName << ": Is a directory.\n";
        }
        close(fd);
        return 0;
    }
    const auto result = doGrep(fd, fileIdx);
    close(fd);
    return result;
}

uint64_t GrepEngine::doGrep(const int32_t fileDescriptor, const uint32_t fileIdx) const {
    assert (mGrepDriver);
    typedef uint64_t (*GrepFunctionType)(int32_t fileDescriptor, const uint32_t fileIdx);
    auto f = reinterpret_cast<GrepFunctionType>(mGrepDriver->getMain());
    
    uint64_t grepResult = f(fileDescriptor, fileIdx);
    if (grepResult > 0) grepMatchFound = true;
    else if ((Mode == NormalMode) && !resultStrs[fileIdx].str().empty()) grepMatchFound = true;
    
    if (Mode == CountOnly) {
        resultStrs[fileIdx] << linePrefix[fileIdx] << grepResult << "\n";
    }
    else if (Mode == FilesWithMatch || Mode == FilesWithoutMatch ) {
        size_t requiredCount = Mode == FilesWithMatch ? 1 : 0;
        if (grepResult == requiredCount) {
            resultStrs[fileIdx] << linePrefix[fileIdx];
        }
    }
    else if (Mode == QuietMode) {
        if (grepMatchFound) exit(MatchFoundExitCode);
    }
    return grepResult;
}

void initFileResult(std::vector<std::string> filenames){
    grepMatchFound = false;
    const int n = filenames.size();
    linePrefix.resize(n);
    if ((n > 1) && !NoFilenameFlag) {
        WithFilenameFlag = true;
    }
    std::string fileSuffix = "";
    bool setLinePrefix = WithFilenameFlag || (Mode == FilesWithMatch) || (Mode == FilesWithoutMatch);
    if (setLinePrefix) {
        if (NullFlag) {
            fileSuffix = std::string("\0", 1);
        }
        else if ((Mode == NormalMode) && InitialTabFlag && !(LineNumberFlag || ByteOffsetFlag)) {
            fileSuffix = "\t:";
        }
        else if ((Mode == NormalMode) || (Mode == CountOnly)) {
            fileSuffix = ":";
        }
        else if ((Mode == FilesWithMatch) || (Mode == FilesWithoutMatch)) {
            fileSuffix = "\n";
        }
    }
    inputFiles = filenames;
    resultStrs = new std::stringstream[n];
    for (unsigned i = 0; i < inputFiles.size(); ++i) {
        if (setLinePrefix) {
            if (inputFiles[i] == "-") {
                linePrefix[i] = LabelFlag + fileSuffix;
            }
            else {
                linePrefix[i] = inputFiles[i] + fileSuffix;
            }
        }
    }
}

template<typename CodeUnit>
void wrapped_report_match(const size_t lineNum, size_t line_start, size_t line_end, const CodeUnit * const buffer, const size_t filesize, const size_t fileIdx) {

//    errs().write_hex((size_t)buffer) << " : " << lineNum << " (" << line_start << ", " << line_end << ", " << filesize << ")\n";

    assert (buffer);
    assert (line_start <= line_end);
    assert (line_end <= filesize);

    if (WithFilenameFlag) {
        resultStrs[fileIdx] << linePrefix[fileIdx];
    }
    if (LineNumberFlag) {
        // Internally line numbers are counted from 0.  For display, adjust
        // the line number so that lines are numbered from 1.
        if (InitialTabFlag) {
            resultStrs[fileIdx] << lineNum+1 << "\t:";
        }
        else {
            resultStrs[fileIdx] << lineNum+1 << ":";
        }
    }

    // If the line "starts" on the LF of a CRLF, it is actually the end of the last line.
    if ((buffer[line_start] == 0xA) && (line_start != line_end)) {
        ++line_start;
    }

    if (LLVM_UNLIKELY(line_end == filesize)) {
        // The match position is at end-of-file.   We have a final unterminated line.
        resultStrs[fileIdx].write((char *)&buffer[line_start], (line_end - line_start) * sizeof(CodeUnit));
        if (NormalizeLineBreaksFlag) {
            resultStrs[fileIdx] << '\n';  // terminate it
        }
    } else {
        const auto end_byte = buffer[line_end];
        if (grep::NormalizeLineBreaksFlag) {
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

void PrintResults(){
    
    for (unsigned i = 0; i < inputFiles.size(); ++i){
        std::cout << resultStrs[i].str();
    }
    exit(grepMatchFound ? MatchFoundExitCode : MatchNotFoundExitCode);
}

    
std::pair<StreamSetBuffer *, StreamSetBuffer *> grepPipeline(Driver * grepDriver, std::vector<re::RE *> & REs, const GrepModeType grepMode, unsigned encodingBits, StreamSetBuffer * ByteStream) {
    auto & idb = grepDriver->getBuilder();
    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments * codegen::ThreadNum;
    size_t MatchLimit = ((grepMode == QuietMode) | (grepMode == FilesWithMatch) | (grepMode == FilesWithoutMatch)) ? 1 : MaxCountFlag;
    

    StreamSetBuffer * BasisBits = grepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(encodingBits, 1), segmentSize * bufferSegments));
    kernel::Kernel * s2pk = grepDriver->addKernelInstance(make_unique<kernel::S2PKernel>(idb));
    grepDriver->makeKernelCall(s2pk, {ByteStream}, {BasisBits});
    
    StreamSetBuffer * LineBreakStream = grepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
    kernel::Kernel * linebreakK = grepDriver->addKernelInstance(make_unique<kernel::LineBreakKernelBuilder>(idb, encodingBits));
    grepDriver->makeKernelCall(linebreakK, {BasisBits}, {LineBreakStream});
    
    kernel::Kernel * requiredStreamsK = grepDriver->addKernelInstance(make_unique<kernel::RequiredStreams_UTF8>(idb));
    StreamSetBuffer * RequiredStreams = grepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(4, 1), segmentSize * bufferSegments));
    grepDriver->makeKernelCall(requiredStreamsK, {BasisBits}, {RequiredStreams});
    
    const auto n = REs.size();
    
    std::vector<std::vector<UCD::UnicodeSet>> charclasses(n);

    for (unsigned i = 0; i < n; i++) {
        REs[i] = re::resolveNames(REs[i]);
        std::vector<UCD::UnicodeSet> UnicodeSets = re::collect_UnicodeSets(REs[i]);
        std::vector<std::vector<unsigned>> exclusiveSetIDs;
        doMultiplexCCs(UnicodeSets, exclusiveSetIDs, charclasses[i]);
        REs[i] = multiplex(REs[i], UnicodeSets, exclusiveSetIDs);
    } 

    std::vector<StreamSetBuffer *> MatchResultsBufs(n);

    for(unsigned i = 0; i < n; ++i){
        const auto numOfCharacterClasses = charclasses[i].size();
        StreamSetBuffer * CharClasses = grepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(numOfCharacterClasses), segmentSize * bufferSegments));
        kernel::Kernel * ccK = grepDriver->addKernelInstance(make_unique<kernel::CharClassesKernel>(idb, std::move(charclasses[i])));
        grepDriver->makeKernelCall(ccK, {BasisBits}, {CharClasses});
        StreamSetBuffer * MatchResults = grepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
        kernel::Kernel * icgrepK = grepDriver->addKernelInstance(make_unique<kernel::ICGrepKernel>(idb, REs[i], numOfCharacterClasses));
        grepDriver->makeKernelCall(icgrepK, {CharClasses, LineBreakStream, RequiredStreams}, {MatchResults});
        MatchResultsBufs[i] = MatchResults;
    }
    StreamSetBuffer * MergedResults = MatchResultsBufs[0];
    if (REs.size() > 1) {
        MergedResults = grepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
        kernel::Kernel * streamsMergeK = grepDriver->addKernelInstance(make_unique<kernel::StreamsMerge>(idb, 1, REs.size()));
        grepDriver->makeKernelCall(streamsMergeK, MatchResultsBufs, {MergedResults});
    }
    StreamSetBuffer * Matches = MergedResults;
    
    if (matchesNeedToBeMovedToEOL()) {
        StreamSetBuffer * OriginalMatches = Matches;
        kernel::Kernel * matchedLinesK = grepDriver->addKernelInstance(make_unique<kernel::MatchedLinesKernel>(idb));
        Matches = grepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
        grepDriver->makeKernelCall(matchedLinesK, {OriginalMatches, LineBreakStream}, {Matches});
    }
    
    if (InvertMatchFlag) {
        kernel::Kernel * invertK = grepDriver->addKernelInstance(make_unique<kernel::InvertMatchesKernel>(idb));
        StreamSetBuffer * OriginalMatches = Matches;
        Matches = grepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
        grepDriver->makeKernelCall(invertK, {OriginalMatches, LineBreakStream}, {Matches});
    }
    if (MatchLimit > 0) {
        kernel::Kernel * untilK = grepDriver->addKernelInstance(make_unique<kernel::UntilNkernel>(idb));
        untilK->setInitialArguments({idb->getSize(MatchLimit)});
        StreamSetBuffer * AllMatches = Matches;
        Matches = grepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
        grepDriver->makeKernelCall(untilK, {AllMatches}, {Matches});
    }
    return std::pair<StreamSetBuffer *, StreamSetBuffer *>(LineBreakStream, Matches);
}


    
void GrepEngine::grepCodeGen_nvptx(std::vector<re::RE *> REs, const GrepModeType grepMode, const bool UTF_16) {

    assert (mGrepDriver == nullptr);

    mGrepDriver = new NVPTXDriver("engine");
    auto & idb = mGrepDriver->getBuilder();
    Module * M = idb->getModule();

    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned encodingBits = UTF_16 ? 16 : 8;

    Type * const int64Ty = idb->getInt64Ty();
    Type * const int32Ty = idb->getInt32Ty();
    Type * const size_ty = idb->getSizeTy();
    Type * const sizeTyPtr = PointerType::get(size_ty, 1);
    Type * const int64tyPtr = PointerType::get(int64Ty, 1);
    Type * const voidTy = idb->getVoidTy();
    
    Function * mainFunc = cast<Function>(M->getOrInsertFunction("Main", voidTy, int64tyPtr, sizeTyPtr, sizeTyPtr, int64tyPtr, nullptr));
    mainFunc->setCallingConv(CallingConv::C);
    idb->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", mainFunc, 0));
    auto args = mainFunc->arg_begin();

    Value * const inputPtr = &*(args++);
    inputPtr->setName("inputPtr");
    Value * const startPointsPtr = &*(args++);
    startPointsPtr->setName("startPointsPtr");
    Value * const bufferSizesPtr = &*(args++);
    bufferSizesPtr->setName("bufferSizesPtr");
    Value * const outputPtr = &*(args++);
    outputPtr->setName("outputPtr");

    Function * tidFunc = M->getFunction("llvm.nvvm.read.ptx.sreg.tid.x");
    Value * tid = idb->CreateCall(tidFunc);
    Function * bidFunc = cast<Function>(M->getOrInsertFunction("llvm.nvvm.read.ptx.sreg.ctaid.x", int32Ty, nullptr));
    Value * bid = idb->CreateCall(bidFunc);

    Value * startPoint = idb->CreateLoad(idb->CreateGEP(startPointsPtr, bid));
    Value * startBlock = idb->CreateUDiv(startPoint, ConstantInt::get(int64Ty, idb->getBitBlockWidth()));
    Type * const inputStreamType = PointerType::get(ArrayType::get(ArrayType::get(idb->getBitBlockType(), 8), 1), 1);    
    Value * inputStreamPtr = idb->CreateGEP(idb->CreateBitCast(inputPtr, inputStreamType), startBlock);
    Value * inputStream = idb->CreateGEP(inputStreamPtr, tid);
    Value * bufferSize = idb->CreateLoad(idb->CreateGEP(bufferSizesPtr, bid));

    StreamSetBuffer * ByteStream = mGrepDriver->addBuffer(make_unique<SourceBuffer>(idb, idb->getStreamSetTy(1, 8), 1));
    kernel::Kernel * sourceK = mGrepDriver->addKernelInstance(make_unique<kernel::MemorySourceKernel>(idb, inputStreamType, segmentSize));
    sourceK->setInitialArguments({inputStream, bufferSize});
    mGrepDriver->makeKernelCall(sourceK, {}, {ByteStream});

    StreamSetBuffer * Matches = std::get<1>(grepPipeline(mGrepDriver, REs, grepMode, encodingBits, ByteStream));
    
    kernel::Kernel * matchCountK = mGrepDriver->addKernelInstance(make_unique<kernel::PopcountKernel>(idb));
    mGrepDriver->makeKernelCall(matchCountK, {Matches}, {});
    mGrepDriver->generatePipelineIR();
    idb->setKernel(matchCountK);
    Value * matchedLineCount = idb->getAccumulator("countResult");
    matchedLineCount = idb->CreateZExt(matchedLineCount, int64Ty);
    
    Value * strideBlocks = ConstantInt::get(int32Ty, idb->getStride() / idb->getBitBlockWidth());
    Value * outputThreadPtr = idb->CreateGEP(outputPtr, idb->CreateAdd(idb->CreateMul(bid, strideBlocks), tid));
    idb->CreateStore(matchedLineCount, outputThreadPtr);
    mGrepDriver->deallocateBuffers();
    idb->CreateRetVoid();

    mGrepDriver->finalizeObject();
}

void GrepEngine::grepCodeGen(std::vector<re::RE *> REs, const GrepModeType grepMode, const bool UTF_16, GrepSource grepSource) {

    assert (mGrepDriver == nullptr);
    mGrepDriver = new ParabixDriver("engine");
    auto & idb = mGrepDriver->getBuilder();
    Module * M = idb->getModule();

    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned encodingBits = UTF_16 ? 16 : 8;

    Type * const int64Ty = idb->getInt64Ty();
    Type * const int32Ty = idb->getInt32Ty();

    kernel::Kernel * sourceK = nullptr;
    
    Function * mainFunc = cast<Function>(M->getOrInsertFunction("Main", int64Ty, idb->getInt32Ty(), int32Ty, nullptr));
    mainFunc->setCallingConv(CallingConv::C);
    idb->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", mainFunc, 0));
    auto args = mainFunc->arg_begin();

    Value * const fileDescriptor = &*(args++);
    fileDescriptor->setName("fileDescriptor");
    Value * fileIdx = &*(args++);
    fileIdx->setName("fileIdx");

    StreamSetBuffer * ByteStream = mGrepDriver->addBuffer(make_unique<SourceBuffer>(idb, idb->getStreamSetTy(1, 8)));

    if (grepSource == GrepSource::File) {
        sourceK = mGrepDriver->addKernelInstance(make_unique<kernel::MMapSourceKernel>(idb, segmentSize));
    } else {
        sourceK = mGrepDriver->addKernelInstance(make_unique<kernel::ReadSourceKernel>(idb, segmentSize));
    }
    sourceK->setInitialArguments({fileDescriptor});

    mGrepDriver->makeKernelCall(sourceK, {}, {ByteStream});
    
    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::tie(LineBreakStream, Matches) = grepPipeline(mGrepDriver, REs, grepMode, encodingBits, ByteStream);
    
    if (grepMode == NormalMode) {
        kernel::Kernel * scanMatchK = mGrepDriver->addKernelInstance(make_unique<kernel::ScanMatchKernel>(idb, GrepType::Normal, encodingBits));
        scanMatchK->setInitialArguments({fileIdx});
        mGrepDriver->makeKernelCall(scanMatchK, {Matches, LineBreakStream, ByteStream}, {});
        if (UTF_16) {
            mGrepDriver->LinkFunction(*scanMatchK, "matcher", &wrapped_report_match<uint16_t>);
        } else {
            mGrepDriver->LinkFunction(*scanMatchK, "matcher", &wrapped_report_match<uint8_t>);
        }
        mGrepDriver->generatePipelineIR();
        mGrepDriver->deallocateBuffers();

        idb->CreateRet(idb->getInt64(0));
    } else {
        kernel::Kernel * matchCountK = mGrepDriver->addKernelInstance(make_unique<kernel::PopcountKernel>(idb));
        mGrepDriver->makeKernelCall(matchCountK, {Matches}, {});
        mGrepDriver->generatePipelineIR();
        idb->setKernel(matchCountK);
        Value * matchedLineCount = idb->getAccumulator("countResult");
        matchedLineCount = idb->CreateZExt(matchedLineCount, int64Ty);
        mGrepDriver->deallocateBuffers();
        idb->CreateRet(matchedLineCount);
    }
    mGrepDriver->finalizeObject();
}

GrepEngine::GrepEngine()
: mGrepDriver(nullptr) {

}

GrepEngine::~GrepEngine() {
    delete mGrepDriver;
}

    
void accumulate_match_wrapper(intptr_t accum_addr, const size_t lineNum, size_t line_start, size_t line_end) {
    reinterpret_cast<MatchAccumulator *>(accum_addr)->accumulate_match(lineNum, line_start, line_end);
}

    


void grepBuffer(re::RE * pattern, char * UnicodeDataBuffer, size_t bufferLength, MatchAccumulator * accum) {
    const unsigned segmentSize = 8;

    ParabixDriver pxDriver("codepointEngine");
    auto & idb = pxDriver.getBuilder();
    Module * M = idb->getModule();
    
    Function * mainFunc = cast<Function>(M->getOrInsertFunction("Main", idb->getVoidTy(), idb->getInt8PtrTy(), idb->getSizeTy(), nullptr));
    mainFunc->setCallingConv(CallingConv::C);
    auto args = mainFunc->arg_begin();
    Value * const buffer = &*(args++);
    buffer->setName("buffer");
    Value * length = &*(args++);
    length->setName("length");
    
    idb->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", mainFunc, 0));
    
    StreamSetBuffer * ByteStream = pxDriver.addBuffer(make_unique<SourceBuffer>(idb, idb->getStreamSetTy(1, 8)));
    kernel::Kernel * sourceK = pxDriver.addKernelInstance(make_unique<kernel::MemorySourceKernel>(idb, idb->getInt8PtrTy(), segmentSize));
    sourceK->setInitialArguments({buffer, length});
    pxDriver.makeKernelCall(sourceK, {}, {ByteStream});
    
    StreamSetBuffer * BasisBits = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(8, 1), segmentSize));
    
    kernel::Kernel * s2pk = pxDriver.addKernelInstance(make_unique<kernel::S2PKernel>(idb));
    pxDriver.makeKernelCall(s2pk, {ByteStream}, {BasisBits});
    
    kernel::Kernel * linebreakK = pxDriver.addKernelInstance(make_unique<kernel::LineBreakKernelBuilder>(idb, 8));
    StreamSetBuffer * LineBreakStream = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize));
    pxDriver.makeKernelCall(linebreakK, {BasisBits}, {LineBreakStream});
    
    kernel::Kernel * requiredStreamsK = pxDriver.addKernelInstance(make_unique<kernel::RequiredStreams_UTF8>(idb));
    StreamSetBuffer * RequiredStreams = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(4, 1), segmentSize));
    pxDriver.makeKernelCall(requiredStreamsK, {BasisBits}, {RequiredStreams});
    
    StreamSetBuffer * MatchResults = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize));
    kernel::Kernel * icgrepK = pxDriver.addKernelInstance(make_unique<kernel::ICGrepKernel>(idb, pattern));
    pxDriver.makeKernelCall(icgrepK, {BasisBits, LineBreakStream, RequiredStreams}, {MatchResults});
    
    StreamSetBuffer * MatchedLines = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize));
    kernel::Kernel * matchedLinesK = pxDriver.addKernelInstance(make_unique<kernel::MatchedLinesKernel>(idb));
    pxDriver.makeKernelCall(matchedLinesK, {MatchResults, LineBreakStream}, {MatchedLines});
    
    kernel::Kernel * scanMatchK = pxDriver.addKernelInstance(make_unique<kernel::ScanMatchKernel>(idb, GrepType::CallBack, 8));
    scanMatchK->setInitialArguments({ConstantInt::get(idb->getIntAddrTy(), reinterpret_cast<intptr_t>(accum))});
    pxDriver.makeKernelCall(scanMatchK, {MatchedLines, LineBreakStream, ByteStream}, {});
    pxDriver.LinkFunction(*scanMatchK, "accumulate_match_wrapper", &accumulate_match_wrapper);
    pxDriver.generatePipelineIR();
    pxDriver.deallocateBuffers();
    idb->CreateRetVoid();
    pxDriver.finalizeObject();
    
    typedef void (*GrepFunctionType)(const char * buffer, const size_t length);
    auto f = reinterpret_cast<GrepFunctionType>(pxDriver.getMain());
    f(UnicodeDataBuffer, bufferLength);
}

class CodepointAccumulator : public MatchAccumulator {
public:
    
    CodepointAccumulator(const char * searchBuffer) : mSearchBuffer(searchBuffer), mParsedCodePointSet(re::makeCC()) {}
    
    void accumulate_match(const size_t lineNum, size_t line_start, size_t line_end) override;
    re::CC * getCodePoints() {return mParsedCodePointSet;}
private:
    const char * mSearchBuffer;
    re::CC * mParsedCodePointSet;
};

void CodepointAccumulator::accumulate_match(const size_t lineNum, size_t line_start, size_t line_end) {
    assert (line_start <= line_end);
    re::codepoint_t c = 0;
    size_t line_pos = line_start;
    while (isxdigit(mSearchBuffer[line_pos])) {
        assert (line_pos < line_end);
        if (isdigit(mSearchBuffer[line_pos])) {
            c = (c << 4) | (mSearchBuffer[line_pos] - '0');
        }
        else {
            c = (c << 4) | (tolower(mSearchBuffer[line_pos]) - 'a' + 10);
        }
        line_pos++;
    }
    assert(((line_pos - line_start) >= 4) && ((line_pos - line_start) <= 6)); // UCD format 4 to 6 hex digits.
    mParsedCodePointSet->insert(c);
}
re::CC * grepCodepoints(re::RE * pattern, char * UnicodeDataBuffer, size_t bufferLength) {
    
    CodepointAccumulator accum(UnicodeDataBuffer);
    
    grepBuffer(pattern, UnicodeDataBuffer, bufferLength, & accum);
    return accum.getCodePoints();
}


class PropertyValueAccumulator : public MatchAccumulator {
public:
    
    PropertyValueAccumulator(const char * searchBuffer, std::vector<std::string> & accumulatedPropertyValues)
       : mSearchBuffer(searchBuffer), mParsedPropertyValueSet(accumulatedPropertyValues) {}
    
    void accumulate_match(const size_t lineNum, size_t line_start, size_t line_end) override;
private:
    const char * mSearchBuffer;
    std::vector<std::string> & mParsedPropertyValueSet;
};
void PropertyValueAccumulator::accumulate_match(const size_t lineNum, size_t line_start, size_t line_end) {
    assert (line_start <= line_end);
    mParsedPropertyValueSet.emplace_back(mSearchBuffer + line_start, mSearchBuffer + line_end);
}


const std::vector<std::string> grepPropertyValues(const std::string& propertyName, re::RE * propertyValuePattern) {
    ParabixDriver pxDriver("propertyValueEngine");
    AlignedAllocator<char, 32> alloc;
    std::vector<std::string> accumulatedValues;

    const std::string & str = UCD::getPropertyValueGrepString(propertyName);

    auto & idb = pxDriver.getBuilder();

    const unsigned segmentSize = 8;
    const auto n = str.length();
    const auto w = idb->getBitBlockWidth() * segmentSize;
    const auto m = w - (n % w);

    char * aligned = alloc.allocate(n + m, 0);
    std::memcpy(aligned, str.data(), n);
    std::memset(aligned + n, 0, m);

    PropertyValueAccumulator accum(aligned, accumulatedValues);
    grepBuffer(propertyValuePattern, aligned, n, & accum);
    alloc.deallocate(aligned, 0);
    return accumulatedValues;
}
}
