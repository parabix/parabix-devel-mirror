/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "grep_engine.h"
#include <llvm/IR/Module.h>
//#include <llvm/ExecutionEngine/MCJIT.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Support/CommandLine.h>
#include <boost/filesystem.hpp>
#include <UCD/UnicodeNameData.h>
#include <UCD/resolve_properties.h>
#include <kernels/cc_kernel.h>
#include <kernels/grep_kernel.h>
#include <kernels/linebreak_kernel.h>
#include <kernels/streams_merge.h>
#include <kernels/match_count.h>
#include <kernels/source_kernel.h>
#include <kernels/s2p_kernel.h>
#include <kernels/scanmatchgen.h>
#include <kernels/streamset.h>
#include <kernels/kernel_builder.h>
#include <pablo/pablo_kernel.h>
#include <re/re_cc.h>
#include <re/re_toolchain.h>
#include <toolchain/toolchain.h>
#include <iostream>
#include <sstream>
#include <cc/multiplex_CCs.h>
#include <llvm/Support/raw_ostream.h>
#include <util/aligned_allocator.h>
#include <sys/stat.h>
#include <fcntl.h>

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

static re::CC * parsedCodePointSet = nullptr;

static std::vector<std::string> parsedPropertyValues;

uint64_t GrepEngine::doGrep(const std::string & fileName, const uint32_t fileIdx) const {
    const int32_t fd = open(fileName.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        return 0;
    }
    const auto result = doGrep(fd, fileIdx);
    close(fd);
    return result;
}

uint64_t GrepEngine::doGrep(const int32_t fileDescriptor, const uint32_t fileIdx) const {
    assert (mGrepFunction);
    typedef uint64_t (*GrepFunctionType)(int32_t fileDescriptor, const uint32_t fileIdx);
    return reinterpret_cast<GrepFunctionType>(mGrepFunction)(fileDescriptor, fileIdx);
}

void GrepEngine::doGrep(const char * buffer, const uint64_t length, const uint32_t fileIdx) const {
    assert (mGrepFunction);
    typedef uint64_t (*GrepFunctionType)(const char * buffer, const uint64_t length, const uint32_t fileIdx);
    reinterpret_cast<GrepFunctionType>(mGrepFunction)(buffer, length, fileIdx);
}

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

//    errs().write_hex((size_t)buffer) << " : " << lineNum << " (" << line_start << ", " << line_end << ", " << filesize << ")\n";

    assert (buffer);
    assert (line_start <= line_end);
    assert (line_end <= filesize);

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

void GrepEngine::grepCodeGen(const std::string & moduleName, std::vector<re::RE *> REs, const bool CountOnly, const bool UTF_16, GrepSource grepSource, const GrepType grepType) {

    ParabixDriver pxDriver(moduleName + ":icgrep");
    auto & idb = pxDriver.getBuilder();
    Module * M = idb->getModule();

    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments * codegen::ThreadNum;
    const unsigned encodingBits = UTF_16 ? 16 : 8;

    Type * const int64Ty = idb->getInt64Ty();
    Type * const int32Ty = idb->getInt32Ty();

    Function * mainFunc = nullptr;
    Value * fileIdx = nullptr;
    StreamSetBuffer * ByteStream = nullptr;
    kernel::Kernel * sourceK = nullptr;

    if (grepSource == GrepSource::Internal) {

        mainFunc = cast<Function>(M->getOrInsertFunction("Main", int64Ty, idb->getInt8PtrTy(), int64Ty, int32Ty, nullptr));
        mainFunc->setCallingConv(CallingConv::C);
        idb->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", mainFunc, 0));
        auto args = mainFunc->arg_begin();

        Value * const buffer = &*(args++);
        buffer->setName("buffer");

        Value * length = &*(args++);
        length->setName("length");
        length = idb->CreateZExtOrTrunc(length, idb->getSizeTy());

        fileIdx = &*(args++);
        fileIdx->setName("fileIdx");

        ByteStream = pxDriver.addBuffer(make_unique<SourceBuffer>(idb, idb->getStreamSetTy(1, 8)));

        sourceK = pxDriver.addKernelInstance(make_unique<kernel::MemorySourceKernel>(idb, idb->getInt8PtrTy(), segmentSize));
        sourceK->setInitialArguments({buffer, length});

    } else {

        mainFunc = cast<Function>(M->getOrInsertFunction("Main", int64Ty, idb->getInt32Ty(), int32Ty, nullptr));
        mainFunc->setCallingConv(CallingConv::C);
        idb->SetInsertPoint(BasicBlock::Create(M->getContext(), "entry", mainFunc, 0));
        auto args = mainFunc->arg_begin();

        Value * const fileDescriptor = &*(args++);
        fileDescriptor->setName("fileDescriptor");
        fileIdx = &*(args++);
        fileIdx->setName("fileIdx");

        ByteStream = pxDriver.addBuffer(make_unique<SourceBuffer>(idb, idb->getStreamSetTy(1, 8)));

        if (grepSource == GrepSource::File) {
            sourceK = pxDriver.addKernelInstance(make_unique<kernel::MMapSourceKernel>(idb, segmentSize));
            sourceK->setInitialArguments({fileDescriptor});
        } else { // if (grepSource == GrepSource::StdIn) {
            sourceK = pxDriver.addKernelInstance(make_unique<kernel::ReadSourceKernel>(idb, segmentSize));
            sourceK->setInitialArguments({idb->getInt32(STDIN_FILENO)});
        }
    }

    pxDriver.makeKernelCall(sourceK, {}, {ByteStream});
    StreamSetBuffer * BasisBits = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(8, 1), segmentSize * bufferSegments));
    
    kernel::Kernel * s2pk = pxDriver.addKernelInstance(make_unique<kernel::S2PKernel>(idb));
    pxDriver.makeKernelCall(s2pk, {ByteStream}, {BasisBits});
    
    kernel::Kernel * linebreakK = pxDriver.addKernelInstance(make_unique<kernel::LineBreakKernelBuilder>(idb, encodingBits));
    StreamSetBuffer * LineBreakStream = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
    pxDriver.makeKernelCall(linebreakK, {BasisBits}, {LineBreakStream});
    
    const auto n = REs.size();

    std::vector<StreamSetBuffer *> MatchResultsBufs(n);

    for(unsigned i = 0; i < n; ++i){
        StreamSetBuffer * MatchResults = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
        kernel::Kernel * icgrepK = pxDriver.addKernelInstance(make_unique<kernel::ICgrepKernelBuilder>(idb, REs[i]));
        pxDriver.makeKernelCall(icgrepK, {BasisBits, LineBreakStream}, {MatchResults});
        MatchResultsBufs[i] = MatchResults;
    }
    StreamSetBuffer * MergedResults = MatchResultsBufs[0];
    if (REs.size() > 1) {
        MergedResults = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
        kernel::Kernel * streamsMergeK = pxDriver.addKernelInstance(make_unique<kernel::StreamsMerge>(idb, 1, REs.size()));
        pxDriver.makeKernelCall(streamsMergeK, MatchResultsBufs, {MergedResults});
    }
    
    if (AlgorithmOptionIsSet(re::InvertMatches)) {
        kernel::Kernel * invertK = pxDriver.addKernelInstance(make_unique<kernel::InvertMatchesKernel>(idb));
        StreamSetBuffer * OriginalMatches = MergedResults;
        MergedResults = pxDriver.addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
        pxDriver.makeKernelCall(invertK, {OriginalMatches, LineBreakStream}, {MergedResults});
    }
    if (CountOnly) {
        kernel::MatchCount matchCountK(idb);
        pxDriver.addKernelCall(matchCountK, {MergedResults}, {});
        pxDriver.generatePipelineIR();
        Value * matchedLineCount = matchCountK.getScalarField("matchedLineCount");
        matchedLineCount = idb->CreateZExt(matchedLineCount, int64Ty);
        idb->CreateRet(matchedLineCount);
        pxDriver.linkAndFinalize();
    } else {
        kernel::ScanMatchKernel scanMatchK(idb, grepType, encodingBits);
        scanMatchK.setInitialArguments({fileIdx});
        pxDriver.addKernelCall(scanMatchK, {MergedResults, LineBreakStream, ByteStream}, {});
        switch (grepType) {
            case GrepType::Normal:
                if (UTF_16) {
                    pxDriver.LinkFunction(scanMatchK, "matcher", &wrapped_report_match<uint16_t>);
                } else {
                    pxDriver.LinkFunction(scanMatchK, "matcher", &wrapped_report_match<uint8_t>);
                }
                break;
            case GrepType::NameExpression:
                pxDriver.LinkFunction(scanMatchK, "matcher", &insert_codepoints);
                break;
            case GrepType::PropertyValue:
                pxDriver.LinkFunction(scanMatchK, "matcher", &insert_property_values);
                break;
        }
        pxDriver.generatePipelineIR();
        idb->CreateRet(idb->getInt64(0));
        pxDriver.linkAndFinalize();
    }

    mGrepFunction = pxDriver.getPointerToMain();
}

re::CC * GrepEngine::grepCodepoints() {
    parsedCodePointSet = re::makeCC();
    char * mFileBuffer = getUnicodeNameDataPtr();
    size_t mFileSize = getUnicodeNameDataSize();
    doGrep(mFileBuffer, mFileSize, 0);
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
    doGrep(aligned, n, 0);
    alloc.deallocate(aligned, 0);
    return parsedPropertyValues;
}

GrepEngine::GrepEngine()
: mGrepFunction(nullptr) {

}
