/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "grep_engine.h"
#include "grep_interface.h"
#include <llvm/IR/Module.h>
#include <boost/filesystem.hpp>
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
#include <cc/multiplex_CCs.h>
#include <llvm/Support/raw_ostream.h>
#include <util/aligned_allocator.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <llvm/ADT/STLExtras.h> // for make_unique

using namespace parabix;
using namespace llvm;

namespace grep {


// DoGrep thread function.
void *DoGrepThreadFunction(void *args)
{
    size_t fileIdx;
    grep::GrepEngine * grepEngine = (grep::GrepEngine *)args;

    grepEngine->count_mutex.lock();
    fileIdx = grepEngine->fileCount;
    grepEngine->fileCount++;
    grepEngine->count_mutex.unlock();

    while (fileIdx < grepEngine->inputFiles.size()) {
        size_t grepResult = grepEngine->doGrep(grepEngine->inputFiles[fileIdx], fileIdx);
        
        grepEngine->count_mutex.lock();
        if (grepResult > 0) grepEngine->grepMatchFound = true;
        fileIdx = grepEngine->fileCount;
        grepEngine->fileCount++;
        grepEngine->count_mutex.unlock();
        if (QuietMode && grepEngine->grepMatchFound) pthread_exit(nullptr);
    }

    pthread_exit(nullptr);
}
    
    
    void NonNormalizingReportMatch::accumulate_match (const size_t lineNum, char * line_start, char * line_end) {
        if (!(WithFilenameFlag | LineNumberFlag) && (line_start == mPrevious_line_end + 1)) {
            // Consecutive matches: only one write call needed.
            mResultStr.write(mPrevious_line_end, line_end - mPrevious_line_end);
        }
        else {
            if (mLineCount > 0) {
                // deal with the final byte of the previous line.
                mResultStr.write(mPrevious_line_end, 1);
            }
            if (WithFilenameFlag) {
                mResultStr << mLinePrefix;
            }
            if (LineNumberFlag) {
                // Internally line numbers are counted from 0.  For display, adjust
                // the line number so that lines are numbered from 1.
                if (InitialTabFlag) {
                    mResultStr << lineNum+1 << "\t:";
                }
                else {
                    mResultStr << lineNum+1 << ":";
                }
            }
            mResultStr.write(line_start, line_end - line_start);
        }
        mPrevious_line_end = line_end;
        mLineCount++;
    }
    
    void NonNormalizingReportMatch::finalize_match(char * buffer_end) {
        if (mLineCount == 0) return;  // No matches.
        if (mPrevious_line_end < buffer_end) {
            mResultStr.write(mPrevious_line_end, 1);
        }
        else {
            // Likely unterminated final line.
            char last_byte = mPrevious_line_end[-1];
            if (last_byte == 0x0D) {
                // The final CR is acceptable as a line_end.
                return;
            }
            // Terminate the line with an LF
            // (Even if we had an incomplete UTF-8 sequence.)
            mResultStr << "\n";
        }
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
    
uint64_t GrepEngine::doGrep(const std::string & fileName, const uint32_t fileIdx) {
    if (fileName == "-") {
        return doGrep(STDIN_FILENO, fileIdx);
    }
    struct stat sb;
    const int32_t fd = open(fileName.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        if (!NoMessagesFlag  && !(Mode == QuietMode)) {
            if (errno == EACCES) {
                resultAccums[fileIdx]->mResultStr << "icgrep: " << fileName << ": Permission denied.\n";
            }
            else if (errno == ENOENT) {
                resultAccums[fileIdx]->mResultStr << "icgrep: " << fileName << ": No such file.\n";
            }
            else {
                resultAccums[fileIdx]->mResultStr << "icgrep: " << fileName << ": Failed.\n";
            }
        }
        return 0;
    }
    if (stat(fileName.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
        if (!NoMessagesFlag  && !(Mode == QuietMode)) {
            resultAccums[fileIdx]->mResultStr << "icgrep: " << fileName << ": Is a directory.\n";
        }
        close(fd);
        return 0;
    }
    const auto result = doGrep(fd, fileIdx);
    close(fd);
    return result;
}

uint64_t GrepEngine::doGrep(const int32_t fileDescriptor, const uint32_t fileIdx) {
    typedef uint64_t (*GrepFunctionType)(int32_t fileDescriptor, intptr_t accum_addr);
    auto f = reinterpret_cast<GrepFunctionType>(mGrepDriver->getMain());
    
    uint64_t grepResult = f(fileDescriptor, reinterpret_cast<intptr_t>(resultAccums[fileIdx].get()));
    if (grepResult > 0) grepMatchFound = true;
    else if ((Mode == NormalMode) && !resultAccums[fileIdx]->mResultStr.str().empty()) grepMatchFound = true;
    
    if (Mode == CountOnly) {
        resultAccums[fileIdx]->mResultStr << resultAccums[fileIdx]->mLinePrefix << grepResult << "\n";
    }
    else if (Mode == FilesWithMatch || Mode == FilesWithoutMatch ) {
        size_t requiredCount = Mode == FilesWithMatch ? 1 : 0;
        if (grepResult == requiredCount) {
            resultAccums[fileIdx]->mResultStr << resultAccums[fileIdx]->mLinePrefix;
        }
    }
    else if (Mode == QuietMode) {
        if (grepMatchFound) exit(MatchFoundExitCode);
    }
    return grepResult;
}

void GrepEngine::initFileResult(std::vector<std::string> filenames){
    grepMatchFound = false;
    const int n = filenames.size();
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
    for (unsigned i = 0; i < inputFiles.size(); ++i) {
        std::string linePrefix;
        if (setLinePrefix) {
            if (inputFiles[i] == "-") {
                linePrefix = LabelFlag + fileSuffix;
            }
            else {
                linePrefix = inputFiles[i] + fileSuffix;
            }
        }
        resultAccums.push_back(make_unique<NonNormalizingReportMatch>(linePrefix));
    }
}


void GrepEngine::PrintResults(){
    
    for (unsigned i = 0; i < inputFiles.size(); ++i){
        std::cout << resultAccums[i]->mResultStr.str();
    }
    exit(grepMatchFound ? MatchFoundExitCode : MatchNotFoundExitCode);
}

    
std::pair<StreamSetBuffer *, StreamSetBuffer *> grepPipeline(Driver * grepDriver, std::vector<re::RE *> & REs, const GrepModeType grepMode, StreamSetBuffer * ByteStream) {
    auto & idb = grepDriver->getBuilder();
    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments * codegen::ThreadNum;
    const unsigned encodingBits = 8;

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
    if (MaxCountFlag > 0) {
        kernel::Kernel * untilK = grepDriver->addKernelInstance(make_unique<kernel::UntilNkernel>(idb));
        untilK->setInitialArguments({idb->getSize(MaxCountFlag)});
        StreamSetBuffer * AllMatches = Matches;
        Matches = grepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
        grepDriver->makeKernelCall(untilK, {AllMatches}, {Matches});
    }
    return std::pair<StreamSetBuffer *, StreamSetBuffer *>(LineBreakStream, Matches);
}

void GrepEngine::grepCodeGen(std::vector<re::RE *> REs, const GrepModeType grepMode) {

    assert (mGrepDriver == nullptr);
    mGrepDriver = new ParabixDriver("engine");
    auto & idb = mGrepDriver->getBuilder();
    Module * M = idb->getModule();

    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned encodingBits = 8;

    Function * mainFunc = cast<Function>(M->getOrInsertFunction("Main", idb->getInt64Ty(), idb->getInt32Ty(), idb->getIntAddrTy(), nullptr));
    mainFunc->setCallingConv(CallingConv::C);
    idb->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", mainFunc, 0));
    auto args = mainFunc->arg_begin();

    Value * const fileDescriptor = &*(args++);
    fileDescriptor->setName("fileDescriptor");
    Value * match_accumulator = &*(args++);
    match_accumulator->setName("match_accumulator");

    StreamSetBuffer * ByteStream = mGrepDriver->addBuffer(make_unique<SourceBuffer>(idb, idb->getStreamSetTy(1, encodingBits)));
    kernel::Kernel * sourceK = mGrepDriver->addKernelInstance(make_unique<kernel::FDSourceKernel>(idb, segmentSize));
    sourceK->setInitialArguments({fileDescriptor});
    mGrepDriver->makeKernelCall(sourceK, {}, {ByteStream});
    
    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::tie(LineBreakStream, Matches) = grepPipeline(mGrepDriver, REs, grepMode, ByteStream);
    
    if (grepMode == NormalMode) {
        kernel::Kernel * scanMatchK = mGrepDriver->addKernelInstance(make_unique<kernel::ScanMatchKernel>(idb));
        scanMatchK->setInitialArguments({match_accumulator});
        mGrepDriver->makeKernelCall(scanMatchK, {Matches, LineBreakStream, ByteStream}, {});
        mGrepDriver->LinkFunction(*scanMatchK, "accumulate_match_wrapper", &accumulate_match_wrapper);
        mGrepDriver->LinkFunction(*scanMatchK, "finalize_match_wrapper", &finalize_match_wrapper);

        
        mGrepDriver->generatePipelineIR();
        mGrepDriver->deallocateBuffers();

        idb->CreateRet(idb->getInt64(0));
    } else {
        kernel::Kernel * matchCountK = mGrepDriver->addKernelInstance(make_unique<kernel::PopcountKernel>(idb));
        mGrepDriver->makeKernelCall(matchCountK, {Matches}, {});
        mGrepDriver->generatePipelineIR();
        idb->setKernel(matchCountK);
        Value * matchedLineCount = idb->getAccumulator("countResult");
        matchedLineCount = idb->CreateZExt(matchedLineCount, idb->getInt64Ty());
        mGrepDriver->deallocateBuffers();
        idb->CreateRet(matchedLineCount);
    }
    mGrepDriver->finalizeObject();
}


GrepEngine::~GrepEngine() {
    delete mGrepDriver;
}

}
