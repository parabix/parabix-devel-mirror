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
#include <re/casing.h>
#include <re/exclude_CC.h>
#include <re/re_toolchain.h>
#include <toolchain/toolchain.h>
#include <re/re_name_resolve.h>
#include <re/re_collect_unicodesets.h>
#include <re/re_multiplex.h>
#include <re/grapheme_clusters.h>
#include <re/printer_re.h>
#include <toolchain/toolchain.h>
#include <toolchain/cpudriver.h>
#include <iostream>
#include <cc/multiplex_CCs.h>
#include <llvm/Support/raw_ostream.h>
#include <util/aligned_allocator.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <llvm/ADT/STLExtras.h> // for make_unique
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/Debug.h>
#include <sched.h>

using namespace parabix;
using namespace llvm;
using namespace cc;

static cl::opt<int> Threads("t", cl::desc("Total number of threads."), cl::init(2));

namespace grep {

// Grep Engine construction and initialization.

GrepEngine::GrepEngine() :
    mGrepDriver(nullptr),
    mNextFileToGrep(0),
    mNextFileToPrint(0),
    grepMatchFound(false),
    mMoveMatchesToEOL(true),
    mEngineThread(pthread_self()) {}

GrepEngine::~GrepEngine() {
    delete mGrepDriver;
}

QuietModeEngine::QuietModeEngine() : GrepEngine() {
    mMoveMatchesToEOL = false;
}

MatchOnlyEngine::MatchOnlyEngine(bool showFilesWithoutMatch) :
    GrepEngine(), mRequiredCount(showFilesWithoutMatch) {
    mFileSuffix = NullFlag ? std::string("\0", 1) : "\n";
    mMoveMatchesToEOL = false;
}

CountOnlyEngine::CountOnlyEngine() : GrepEngine() {
    mFileSuffix = ":";
}

EmitMatchesEngine::EmitMatchesEngine() : GrepEngine() {
    mFileSuffix = InitialTabFlag ? "\t:" : ":";
    if (LineRegexpFlag) mMoveMatchesToEOL = false;
}

void GrepEngine::initFileResult(std::vector<std::string> & filenames) {
    const unsigned n = filenames.size();
    mResultStrs.resize(n);
    mFileStatus.resize(n, FileStatus::Pending);
    inputFiles = filenames;
}

// Code Generation
//
// All engines share a common pipeline to compute a stream of Matches from a given input Bytestream.

std::pair<StreamSetBuffer *, StreamSetBuffer *> GrepEngine::grepPipeline(std::vector<re::RE *> & REs, StreamSetBuffer * ByteStream) {
    auto & idb = mGrepDriver->getBuilder();
    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments * codegen::ThreadNum;
    const unsigned encodingBits = 8;

    StreamSetBuffer * BasisBits = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(encodingBits, 1), segmentSize * bufferSegments);
    kernel::Kernel * s2pk = mGrepDriver->addKernelInstance<kernel::S2PKernel>(idb);
    mGrepDriver->makeKernelCall(s2pk, {ByteStream}, {BasisBits});

    StreamSetBuffer * LineFeedStream = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments);
    kernel::Kernel * linefeedK = mGrepDriver->addKernelInstance<kernel::LineFeedKernelBuilder>(idb, encodingBits);
    mGrepDriver->makeKernelCall(linefeedK, {BasisBits}, {LineFeedStream});

    StreamSetBuffer * LineBreakStream = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments);
    StreamSetBuffer * CRLFStream = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments);
    kernel::Kernel * linebreakK = mGrepDriver->addKernelInstance<kernel::LineBreakKernelBuilder>(idb, encodingBits);
    mGrepDriver->makeKernelCall(linebreakK, {BasisBits, LineFeedStream}, {LineBreakStream, CRLFStream});

    kernel::Kernel * requiredStreamsK = mGrepDriver->addKernelInstance<kernel::RequiredStreams_UTF8>(idb);
    StreamSetBuffer * RequiredStreams = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(3, 1), segmentSize * bufferSegments);
    mGrepDriver->makeKernelCall(requiredStreamsK, {BasisBits}, {RequiredStreams});

    const auto n = REs.size();
    std::vector<std::vector<re::CC *>> charclasses(n);
    std::vector<StreamSetBuffer *> MatchResultsBufs(n);

    for(unsigned i = 0; i < n; ++i) {
        REs[i] = resolveModesAndExternalSymbols(REs[i]);
        REs[i] = excludeUnicodeLineBreak(REs[i]);
#define USE_MULTIPLEX_CC
#ifdef USE_MULTIPLEX_CC
        
        REs[i] = multiplexing_prepasses(REs[i]);
        const std::vector<const re::CC *> UnicodeSets = re::collectUnicodeSets(REs[i]);
        mpx = make_unique<MultiplexedAlphabet>("mpx", UnicodeSets);
        REs[i] = transformCCs(mpx.get(), REs[i]);
        //llvm::errs() << Printer_RE::PrintRE(REs[i]) << '\n';
        std::vector<re::CC *> mpx_basis = mpx->getMultiplexedCCs();
        auto numOfCharacterClasses = mpx_basis.size();
        StreamSetBuffer * CharClasses = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(numOfCharacterClasses), segmentSize * bufferSegments);
        kernel::Kernel * ccK = mGrepDriver->addKernelInstance<kernel::CharClassesKernel>(idb, std::move(mpx_basis));
        mGrepDriver->makeKernelCall(ccK, {BasisBits}, {CharClasses});
        StreamSetBuffer * MatchResults = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments);
        kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, REs[i], numOfCharacterClasses);
        mGrepDriver->makeKernelCall(icgrepK, {CharClasses, LineBreakStream, CRLFStream, RequiredStreams}, {MatchResults});
#else
        REs[i] = regular_expression_passes(REs[i]);
        StreamSetBuffer * MatchResults = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments);
        kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance<kernel::ICGrepKernel>(idb, REs[i]);
        mGrepDriver->makeKernelCall(icgrepK, {BasisBits, LineBreakStream, CRLFStream, RequiredStreams}, {MatchResults});
#endif
        MatchResultsBufs[i] = MatchResults;
    }
    StreamSetBuffer * MergedResults = MatchResultsBufs[0];
    if (REs.size() > 1) {
        MergedResults = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments);
        kernel::Kernel * streamsMergeK = mGrepDriver->addKernelInstance<kernel::StreamsMerge>(idb, 1, REs.size());
        mGrepDriver->makeKernelCall(streamsMergeK, MatchResultsBufs, {MergedResults});
    }
    StreamSetBuffer * Matches = MergedResults;

    if (mMoveMatchesToEOL) {
        StreamSetBuffer * OriginalMatches = Matches;
        kernel::Kernel * matchedLinesK = mGrepDriver->addKernelInstance<kernel::MatchedLinesKernel>(idb);
        Matches = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments);
        mGrepDriver->makeKernelCall(matchedLinesK, {OriginalMatches, LineBreakStream}, {Matches});
    }

    if (InvertMatchFlag) {
        kernel::Kernel * invertK = mGrepDriver->addKernelInstance<kernel::InvertMatchesKernel>(idb);
        StreamSetBuffer * OriginalMatches = Matches;
        Matches = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments);
        mGrepDriver->makeKernelCall(invertK, {OriginalMatches, LineBreakStream}, {Matches});
    }
    if (MaxCountFlag > 0) {
        kernel::Kernel * untilK = mGrepDriver->addKernelInstance<kernel::UntilNkernel>(idb);
        untilK->setInitialArguments({idb->getSize(MaxCountFlag)});
        StreamSetBuffer * AllMatches = Matches;
        Matches = mGrepDriver->addBuffer<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments);
        mGrepDriver->makeKernelCall(untilK, {AllMatches}, {Matches});
    }
    return std::pair<StreamSetBuffer *, StreamSetBuffer *>(LineBreakStream, Matches);
}

// The QuietMode, MatchOnly and CountOnly engines share a common code generation main function,
// which returns a count of the matches found (possibly subject to a MaxCount).
//

void GrepEngine::grepCodeGen(std::vector<re::RE *> REs) {

    assert (mGrepDriver == nullptr);
    mGrepDriver = new ParabixDriver("engine");
    auto & idb = mGrepDriver->getBuilder();
    Module * M = idb->getModule();

    const auto segmentSize = codegen::SegmentSize;
    const auto bufferSegments = codegen::BufferSegments * codegen::ThreadNum;

    const unsigned encodingBits = 8;

    Function * mainFunc = cast<Function>(M->getOrInsertFunction("Main", idb->getInt64Ty(), idb->getInt32Ty(), nullptr));
    mainFunc->setCallingConv(CallingConv::C);
    idb->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", mainFunc, 0));
    auto args = mainFunc->arg_begin();

    Value * const fileDescriptor = &*(args++);
    fileDescriptor->setName("fileDescriptor");

    StreamSetBuffer * ByteStream = mGrepDriver->addBuffer<SourceBuffer>(idb, idb->getStreamSetTy(1, encodingBits));
    kernel::Kernel * sourceK = mGrepDriver->addKernelInstance<kernel::FDSourceKernel>(idb, segmentSize * bufferSegments);
    sourceK->setInitialArguments({fileDescriptor});
    mGrepDriver->makeKernelCall(sourceK, {}, {ByteStream});

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::tie(LineBreakStream, Matches) = grepPipeline(REs, ByteStream);

    kernel::Kernel * matchCountK = mGrepDriver->addKernelInstance<kernel::PopcountKernel>(idb);
    mGrepDriver->makeKernelCall(matchCountK, {Matches}, {});
    mGrepDriver->generatePipelineIR();
    idb->setKernel(matchCountK);
    Value * matchedLineCount = idb->getAccumulator("countResult");
    matchedLineCount = idb->CreateZExt(matchedLineCount, idb->getInt64Ty());
    mGrepDriver->deallocateBuffers();
    idb->CreateRet(matchedLineCount);
    mGrepDriver->finalizeObject();
}

//
// The EmitMatches engine uses an EmitMatchesAccumulator object to concatenate together
// matched lines.

class EmitMatch : public MatchAccumulator {
    friend class EmitMatchesEngine;
public:
    EmitMatch(std::string linePrefix, std::ostringstream & strm) : mLinePrefix(linePrefix), mLineCount(0), mTerminated(true), mResultStr(strm) {}
    void accumulate_match(const size_t lineNum, char * line_start, char * line_end) override;
    void finalize_match(char * buffer_end) override;
protected:
    std::string mLinePrefix;
    size_t mLineCount;
    bool mTerminated;
    std::ostringstream & mResultStr;
};

//
//  Default Report Match:  lines are emitted with whatever line terminators are found in the
//  input.  However, if the final line is not terminated, a new line is appended.
//
void EmitMatch::accumulate_match (const size_t lineNum, char * line_start, char * line_end) {
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
    size_t bytes = line_end - line_start + 1;
    mResultStr.write(line_start, bytes);
    mLineCount++;
    unsigned last_byte = *line_end;
    mTerminated = (last_byte >= 0x0A) && (last_byte <= 0x0D);
    if (LLVM_UNLIKELY(!mTerminated)) {
        if (last_byte == 0x85) {  //  Possible NEL terminator.
            mTerminated = (bytes >= 2) && (static_cast<unsigned>(line_end[-1]) == 0xC2);
        }
        else {
            // Possible LS or PS terminators.
            mTerminated = (bytes >= 3) && (static_cast<unsigned>(line_end[-2]) == 0xE2)
                                       && (static_cast<unsigned>(line_end[-1]) == 0x80)
                                       && ((last_byte == 0xA8) || (last_byte == 0xA9));
        }
    }
}

void EmitMatch::finalize_match(char * buffer_end) {
    if (!mTerminated) mResultStr << "\n";
}

void EmitMatchesEngine::grepCodeGen(std::vector<re::RE *> REs) {
    assert (mGrepDriver == nullptr);
    mGrepDriver = new ParabixDriver("engine");
    auto & idb = mGrepDriver->getBuilder();
    Module * M = idb->getModule();

    const auto segmentSize = codegen::SegmentSize;
    const auto bufferSegments = codegen::BufferSegments * codegen::ThreadNum;
    const unsigned encodingBits = 8;

    Function * mainFunc = cast<Function>(M->getOrInsertFunction("Main", idb->getInt64Ty(), idb->getInt32Ty(), idb->getIntAddrTy(), nullptr));
    mainFunc->setCallingConv(CallingConv::C);
    idb->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", mainFunc, 0));
    auto args = mainFunc->arg_begin();

    Value * const fileDescriptor = &*(args++);
    fileDescriptor->setName("fileDescriptor");
    Value * match_accumulator = &*(args++);
    match_accumulator->setName("match_accumulator");

    StreamSetBuffer * ByteStream = mGrepDriver->addBuffer<SourceBuffer>(idb, idb->getStreamSetTy(1, encodingBits));
    kernel::Kernel * sourceK = mGrepDriver->addKernelInstance<kernel::FDSourceKernel>(idb, segmentSize * bufferSegments);
    sourceK->setInitialArguments({fileDescriptor});
    mGrepDriver->makeKernelCall(sourceK, {}, {ByteStream});

    StreamSetBuffer * LineBreakStream;
    StreamSetBuffer * Matches;
    std::tie(LineBreakStream, Matches) = grepPipeline(REs, ByteStream);

    kernel::Kernel * scanMatchK = mGrepDriver->addKernelInstance<kernel::ScanMatchKernel>(idb);
    scanMatchK->setInitialArguments({match_accumulator});
    mGrepDriver->makeKernelCall(scanMatchK, {Matches, LineBreakStream, ByteStream}, {});
    mGrepDriver->LinkFunction(*scanMatchK, "accumulate_match_wrapper", &accumulate_match_wrapper);
    mGrepDriver->LinkFunction(*scanMatchK, "finalize_match_wrapper", &finalize_match_wrapper);

    mGrepDriver->generatePipelineIR();
    mGrepDriver->deallocateBuffers();
    idb->CreateRet(idb->getInt64(0));
    mGrepDriver->finalizeObject();
}


//
//  The doGrep methods apply a GrepEngine to a single file, processing the results
//  differently based on the engine type.

uint64_t GrepEngine::doGrep(const std::string & fileName, const uint32_t fileIdx) {
    typedef uint64_t (*GrepFunctionType)(int32_t fileDescriptor);
    auto f = reinterpret_cast<GrepFunctionType>(mGrepDriver->getMain());

    int32_t fileDescriptor = openFile(fileName, mResultStrs[fileIdx]);
    if (fileDescriptor == -1) return 0;

    uint64_t grepResult = f(fileDescriptor);
    close(fileDescriptor);
    return grepResult;
}

uint64_t CountOnlyEngine::doGrep(const std::string & fileName, const uint32_t fileIdx) {
    uint64_t grepResult = GrepEngine::doGrep(fileName, fileIdx);
    if (WithFilenameFlag) mResultStrs[fileIdx] << linePrefix(fileName);
    mResultStrs[fileIdx] << grepResult << "\n";
    return grepResult;
}

std::string GrepEngine::linePrefix(std::string fileName) {
    if (fileName == "-") {
        return LabelFlag + mFileSuffix;
    }
    else {
        return fileName + mFileSuffix;
    }
}

uint64_t MatchOnlyEngine::doGrep(const std::string & fileName, const uint32_t fileIdx) {
    uint64_t grepResult = GrepEngine::doGrep(fileName, fileIdx);
    if (grepResult == mRequiredCount) {
       mResultStrs[fileIdx] << linePrefix(fileName);
    }
    return grepResult;
}

uint64_t EmitMatchesEngine::doGrep(const std::string & fileName, const uint32_t fileIdx) {
    typedef uint64_t (*GrepFunctionType)(int32_t fileDescriptor, intptr_t accum_addr);
    auto f = reinterpret_cast<GrepFunctionType>(mGrepDriver->getMain());

    int32_t fileDescriptor = openFile(fileName, mResultStrs[fileIdx]);
    if (fileDescriptor == -1) return 0;
    EmitMatch accum(linePrefix(fileName), mResultStrs[fileIdx]);
    f(fileDescriptor, reinterpret_cast<intptr_t>(&accum));
    close(fileDescriptor);
    if (accum.mLineCount > 0) grepMatchFound = true;
    return accum.mLineCount;
}

// Open a file and return its file desciptor.
int32_t GrepEngine::openFile(const std::string & fileName, std::ostringstream & msgstrm) {
    if (fileName == "-") {
        return STDIN_FILENO;
    }
    else {
        struct stat sb;
        int32_t fileDescriptor = open(fileName.c_str(), O_RDONLY);
        if (LLVM_UNLIKELY(fileDescriptor == -1)) {
            if (!NoMessagesFlag) {
                if (errno == EACCES) {
                    msgstrm << "icgrep: " << fileName << ": Permission denied.\n";
                }
                else if (errno == ENOENT) {
                    msgstrm << "icgrep: " << fileName << ": No such file.\n";
                }
                else {
                    msgstrm << "icgrep: " << fileName << ": Failed.\n";
                }
            }
            return fileDescriptor;
        }
        if (stat(fileName.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
            if (!NoMessagesFlag) {
                msgstrm << "icgrep: " << fileName << ": Is a directory.\n";
            }
            close(fileDescriptor);
            return -1;
        }
        return fileDescriptor;
    }
}

// The process of searching a group of files may use a sequential or a task
// parallel approach.

void * DoGrepThreadFunction(void *args) {
    return reinterpret_cast<GrepEngine *>(args)->DoGrepThreadMethod();
}

bool GrepEngine::searchAllFiles() {
    const unsigned numOfThreads = std::min(static_cast<unsigned>(Threads), static_cast<unsigned>(inputFiles.size())); 
    std::vector<pthread_t> threads(numOfThreads);

    for(unsigned long i = 1; i < numOfThreads; ++i) {
        const int rc = pthread_create(&threads[i], nullptr, DoGrepThreadFunction, (void *)this);
        if (rc) {
            llvm::report_fatal_error("Failed to create thread: code " + std::to_string(rc));
        }
    }
    // Main thread also does the work;

    DoGrepThreadMethod();
    for(unsigned i = 1; i < numOfThreads; ++i) {
        void * status = nullptr;
        const int rc = pthread_join(threads[i], &status);
        if (rc) {
            llvm::report_fatal_error("Failed to join thread: code " + std::to_string(rc));
        }
    }
    return grepMatchFound;
}


// DoGrep thread function.
void * GrepEngine::DoGrepThreadMethod() {

    unsigned fileIdx = mNextFileToGrep++;
    while (fileIdx < inputFiles.size()) {
        const auto grepResult = doGrep(inputFiles[fileIdx], fileIdx);
        mFileStatus[fileIdx] = FileStatus::GrepComplete;
        if (grepResult > 0) {
            grepMatchFound = true;
        }
        if (QuietMode && grepMatchFound) {
            if (pthread_self() != mEngineThread) {
                pthread_exit(nullptr);
            }
            return nullptr;
        }
        fileIdx = mNextFileToGrep++;
    }

    unsigned printIdx = mNextFileToPrint++;
    while (printIdx < inputFiles.size()) {
        const bool readyToPrint = ((printIdx == 0) || (mFileStatus[printIdx - 1] == FileStatus::PrintComplete)) && (mFileStatus[printIdx] == FileStatus::GrepComplete);
        if (readyToPrint) {
            const auto output = mResultStrs[printIdx].str();
            if (!output.empty()) {
                llvm::outs() << output;
            }
            mFileStatus[printIdx] = FileStatus::PrintComplete;
            printIdx = mNextFileToPrint++;
        } else {
            mGrepDriver->performIncrementalCacheCleanupStep();
        }
        sched_yield();
    }

    if (pthread_self() != mEngineThread) {
        pthread_exit(nullptr);
    } else {
        // Always perform one final cache cleanup step.
        mGrepDriver->performIncrementalCacheCleanupStep();
    }
}

}
