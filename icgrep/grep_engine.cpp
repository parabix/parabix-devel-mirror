/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "grep_engine.h"
#include <llvm/IR/Module.h>
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
#include <kernels/until_n.h>
#include <kernels/kernel_builder.h>
#include <pablo/pablo_kernel.h>
#include <re/re_cc.h>
#include <re/re_toolchain.h>
#include <toolchain/toolchain.h>
#include <toolchain/cpudriver.h>
#include <toolchain/NVPTXDriver.h>
#include <iostream>
#include <sstream>
#include <cc/multiplex_CCs.h>
#include <llvm/Support/raw_ostream.h>
#include <util/aligned_allocator.h>
#include <sys/stat.h>
#include <fcntl.h>

#ifdef CUDA_ENABLED
#include <preprocess.cpp>
#include <IR_Gen/CudaDriver.h>
#endif

using namespace parabix;
using namespace llvm;

namespace grep {

static cl::OptionCategory RE_Options("A. Regular Expression Interpretation", "These options control regular expression parsing and interpretation");

re::RE_Syntax RegexpSyntax;
static cl::opt<re::RE_Syntax, true> RegexpSyntaxOption(cl::desc("Regular expression syntax: (default PCRE)"),
    cl::values(
        clEnumValN(re::RE_Syntax::ERE, "E", "Posix extended regular expression (ERE) syntax"),
        clEnumValN(re::RE_Syntax::FixedStrings, "F", "Fixed strings, separated by newlines"),
        clEnumValN(re::RE_Syntax::BRE, "G", "Posix basic regular expression (BRE) syntax"),
        clEnumValN(re::RE_Syntax::PCRE, "P", "Perl-compatible regular expression (PCRE) syntax"),
        clEnumValN(re::RE_Syntax::ERE, "extended-regexp", "Alias for -E"),
        clEnumValN(re::RE_Syntax::FixedStrings, "fixed-strings", "Alias for -F"),
        clEnumValN(re::RE_Syntax::BRE, "basic-regexp", "Alias for -G"),
        clEnumValN(re::RE_Syntax::PCRE, "perl-regexp", "Alias for -P"),
        clEnumValN(re::RE_Syntax::PROSITE, "PROSITE", "PROSITE protein patterns syntax"),
        clEnumValEnd), cl::cat(RE_Options), cl::Grouping, cl::location(RegexpSyntax), cl::init(re::RE_Syntax::PCRE));

bool IgnoreCaseFlag;
static cl::opt<bool, true> IgnoreCase("i", cl::desc("Ignore case distinctions in the pattern and the file (alias: -ignore-case)."), 
                                      cl::cat(RE_Options), cl::location(IgnoreCaseFlag), cl::Grouping);
static cl::alias IgnoreCaseAlias("ignore-case", cl::desc("Alias for -i"), cl::aliasopt(IgnoreCase), cl::NotHidden);

bool InvertMatchFlag;
static cl::opt<bool, true> InvertMatch("v", cl::desc("Invert match results: select non-matching lines (alias: -invert-match)."), 
                                       cl::cat(RE_Options), cl::location(InvertMatchFlag), cl::Grouping);
static cl::alias InvertMatchAlias("invert-match", cl::desc("Alias for -v"), cl::aliasopt(InvertMatch), cl::NotHidden);

bool LineRegexpFlag;
static cl::opt<bool, true> LineRegexp("x", cl::desc("Require that entire lines be matched (alias: -line-regexp)."), cl::cat(RE_Options), 
                                      cl::location(LineRegexpFlag), cl::Grouping);
static cl::alias LineRegexpAlias("line-regexp", cl::desc("Alias for -x"), cl::aliasopt(LineRegexp), cl::NotHidden);

bool WordRegexpFlag;
static cl::opt<bool, true> WordRegexp("w", cl::desc("Require that that whole words be matched (alias: -word-regexp)."), cl::cat(RE_Options),
                                      cl::location(WordRegexpFlag), cl::Grouping);
static cl::alias WordRegexpAlias("word-regexp", cl::desc("Alias for -w"), cl::aliasopt(WordRegexp), cl::NotHidden);

const cl::OptionCategory * grep_regexp_flags() {
    return &RE_Options;
}

static cl::OptionCategory GrepInputOptions("B. Input Options",
                                             "These options control the input.");

static cl::opt<bool> NullData("z", cl::desc("Use the NUL character (codepoint 00) as the line-break character for input."), cl::cat(GrepInputOptions), cl::Grouping);
static cl::alias NullDataAlias("null-data", cl::desc("Alias for -z"), cl::aliasopt(NullData));

bool RecursiveFlag;
static cl::opt<bool, true> Recursive("r", cl::desc("Recursively process files within directories, (but follow only top-level symlinks unless -R)."), 
                               cl::location(RecursiveFlag), cl::cat(GrepInputOptions), cl::Grouping);
static cl::alias RecursiveAlias("recursive", cl::desc("Alias for -r"), cl::aliasopt(Recursive));

bool DereferenceRecursiveFlag;
static cl::opt<bool, true> DereferenceRecursive("R", cl::desc("Recursively process files within directories, following symlinks at all levels."),
                                          cl::location(DereferenceRecursiveFlag), cl::cat(GrepInputOptions), cl::Grouping);
static cl::alias DereferenceRecursiveAlias("dereference-recursive", cl::desc("Alias for -R"), cl::aliasopt(DereferenceRecursive));




static cl::OptionCategory bGrepOutputOptions("C. Output Options",
                                             "These options control the output.");

GrepModeType Mode;
static cl::opt<GrepModeType, true> GrepModeOption(cl::desc("Abbreviated output mode options:"),
    cl::values(
        clEnumValN(CountOnly, "c", "Display only the count of matching lines per file."),
        clEnumValN(FilesWithMatch, "l", "Display only the names of files that have at least one match to the pattern."),
        clEnumValN(FilesWithoutMatch, "L", "Display only the names of files that do not match the pattern."),
        clEnumValN(QuietMode, "q", "Do not generate any output and ignore errors; set the return to zero status if a match is found."),
        clEnumValN(CountOnly, "count", "Alias for -c"),
        clEnumValN(FilesWithMatch, "files-with-match", "Alias for -l"),
        clEnumValN(FilesWithoutMatch, "files-without-match", "Alias for -L"),
        clEnumValN(QuietMode, "quiet", "Alias for -q"),
        clEnumValN(QuietMode, "silent", "Alias for -q"),
        clEnumValEnd), cl::cat(bGrepOutputOptions), cl::Grouping, cl::location(Mode), cl::init(NormalMode));


static cl::opt<bool> SilenceFileErrors("s", cl::desc("Suppress messages for file errors."), cl::init(false),  cl::cat(bGrepOutputOptions));

static cl::opt<bool> NormalizeLineBreaks("normalize-line-breaks", cl::desc("Normalize line breaks to std::endl."), cl::init(false),  cl::cat(bGrepOutputOptions));

static cl::opt<bool> ShowFileNames("H", cl::desc("Show the file name with each matching line."), cl::cat(bGrepOutputOptions));
static cl::alias ShowFileNamesLong("with-filename", cl::desc("Alias for -H"), cl::aliasopt(ShowFileNames));

static cl::opt<bool> ShowLineNumbers("n", cl::desc("Show the line number with each matching line."), cl::cat(bGrepOutputOptions));
static cl::alias ShowLineNumbersLong("line-number", cl::desc("Alias for -n"), cl::aliasopt(ShowLineNumbers));

static cl::opt<int> MaxCount("m", cl::desc("Limit the number of matches per file."), cl::cat(bGrepOutputOptions), cl::init((size_t) 0), cl::Prefix);
static cl::alias MaxCountLong("max-count", cl::desc("Alias for -m"), cl::aliasopt(MaxCount));

static cl::opt<int> AfterContext("A", cl::desc("Print <num> lines of context after each matching line."), cl::cat(bGrepOutputOptions), cl::Prefix);
static cl::alias AfterContextAlias("after-context", cl::desc("Alias for -A"), cl::aliasopt(AfterContext));

static cl::opt<int> BeforeContext("B", cl::desc("Print <num>lines of context before each matching line."), cl::cat(bGrepOutputOptions), cl::Prefix);
static cl::alias BeforeContextAlias("before-context", cl::desc("Alias for -B"), cl::aliasopt(BeforeContext));

static cl::opt<int> Context("C", cl::desc("Print <num> lines of context before and after each matching line."), cl::cat(bGrepOutputOptions), cl::Prefix);
static cl::alias ContextAlias("context", cl::desc("Alias for -C"), cl::aliasopt(Context));

static cl::opt<bool> OnlyMatching("o", cl::desc("Display only the exact strings that match the pattern, with possibly multiple matches per line."), cl::cat(bGrepOutputOptions), cl::Grouping);
static cl::alias OnlyMatchingAlias("only-matching", cl::desc("Alias for -o"), cl::aliasopt(OnlyMatching));

static cl::opt<bool> Null("Z", cl::desc("Write NUL characters after filenames generated to output."), cl::cat(bGrepOutputOptions), cl::Grouping);
static cl::alias NullAlias("null", cl::desc("Alias for -Z"), cl::aliasopt(Null));

static cl::opt<bool> ByteOffset("b", cl::desc("Show the byte offset within the file for each matching line."), cl::cat(bGrepOutputOptions), cl::Grouping);
static cl::alias ByteOffsetAlias("byte-offset", cl::desc("Alias for -b"), cl::aliasopt(ByteOffset));

static cl::opt<bool> UnixByteOffsets("u", cl::desc("If byte offsets are displayed, report offsets as if all lines are terminated with a single LF."), cl::cat(bGrepOutputOptions), cl::Grouping);
static cl::alias UnixByteOffsetsAlias("unix-byte-offsets", cl::desc("Alias for -u"), cl::aliasopt(UnixByteOffsets));

static cl::opt<bool> InitialTab("T", cl::desc("Line up matched line content using an inital tab character."), cl::cat(bGrepOutputOptions), cl::Grouping);
static cl::alias InitialTabAlias("initial-tab", cl::desc("Alias for -T"), cl::aliasopt(InitialTab));


const cl::OptionCategory * grep_output_flags() {
    return &bGrepOutputOptions;
}

const cl::OptionCategory * grep_input_flags() {
    return &GrepInputOptions;
}


static re::CC * parsedCodePointSet = nullptr;

static std::vector<std::string> parsedPropertyValues;

size_t * startPoints = nullptr;
size_t * accumBytes = nullptr;


void GrepEngine::doGrep(const std::string & fileName) const{
#ifdef CUDA_ENABLED
    const bool CountOnly = true;
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
            
            codegen::BlockSize = 128;
            std::vector<size_t> LFPositions = preprocess(fileBuffer, fileSize);
            
            const unsigned numOfGroups = codegen::GroupNum;
            if (posix_memalign((void**)&startPoints, 8, (numOfGroups+1)*sizeof(size_t)) ||
                posix_memalign((void**)&accumBytes, 8, (numOfGroups+1)*sizeof(size_t))) {
                std::cerr << "Cannot allocate memory for startPoints or accumBytes.\n";
                exit(-1);
            }

            ulong * rslt = RunPTX(PTXFilename, fileBuffer, fileSize, CountOnly, LFPositions, startPoints, accumBytes);
            source.close();
        } catch (std::exception & e) {
            if (!SilenceFileErrors) {
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
    const int32_t fd = open(fileName.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
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
    return f(fileDescriptor, fileIdx);
}

void GrepEngine::doGrep(const char * buffer, const uint64_t length, const uint32_t fileIdx) const {
    assert (mGrepDriver);
    typedef uint64_t (*GrepFunctionType)(const char * buffer, const uint64_t length, const uint32_t fileIdx);
    auto f = reinterpret_cast<GrepFunctionType>(mGrepDriver->getMain());
    f(buffer, length, fileIdx);
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
        // Internally line numbers are counted from 0.  For display, adjust
        // the line number so that lines are numbered from 1.
        resultStrs[fileIdx] << lineNum+1 << ":";
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

const int MatchFoundReturnCode = 0;
const int MatchNotFoundReturnCode = 1;
void PrintResult(GrepModeType grepMode, std::vector<size_t> & total_CountOnly){
    if (grepMode == NormalMode) {
        int returnCode = MatchNotFoundReturnCode;
        for (unsigned i = 0; i < inputFiles.size(); ++i){
            std::cout << resultStrs[i].str();
            if (!resultStrs[i].str().empty()) returnCode = MatchFoundReturnCode;
        }
        exit(returnCode);
    }
    if (grepMode == CountOnly) {
        size_t total = 0;
        if (!ShowFileNames) {
            for (unsigned i = 0; i < inputFiles.size(); ++i) {
                std::cout << total_CountOnly[i] << std::endl;
                total += total_CountOnly[i];
            }
        } else {
            for (unsigned i = 0; i < inputFiles.size(); ++i){
                std::cout << inputFiles[i] << ':' << total_CountOnly[i] << std::endl;
                total += total_CountOnly[i];
            };
        }
        exit(total == 0 ? MatchNotFoundReturnCode : MatchFoundReturnCode);
    }
    else if (grepMode == FilesWithMatch || grepMode == FilesWithoutMatch ) {
        size_t total = 0;
        size_t requiredCount = grepMode == FilesWithMatch ? 1 : 0;
        for (unsigned i = 0; i < inputFiles.size(); ++i) {
            if (total_CountOnly[i] == requiredCount) {
                std::cout << inputFiles[i] << std::endl;
            }
            total += total_CountOnly[i];
        }
        exit(total == 0 ? MatchNotFoundReturnCode : MatchFoundReturnCode);
    } else /* QuietMode */ {
        for (unsigned i = 0; i < inputFiles.size(); ++i){
            if (total_CountOnly[i] > 0) exit(MatchFoundReturnCode);
        }
        exit(MatchNotFoundReturnCode);
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

void GrepEngine::grepCodeGen_nvptx(std::vector<re::RE *> REs, const GrepModeType grepMode, const bool UTF_16) {

    assert (mGrepDriver == nullptr);

    mGrepDriver = new NVPTXDriver("engine");
    auto & idb = mGrepDriver->getBuilder();
    Module * M = idb->getModule();

    const unsigned segmentSize = codegen::SegmentSize;
    const unsigned bufferSegments = codegen::BufferSegments * codegen::ThreadNum;
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

    StreamSetBuffer * BasisBits = mGrepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(8, 1), segmentSize * bufferSegments));
    kernel::Kernel * s2pk = mGrepDriver->addKernelInstance(make_unique<kernel::S2PKernel>(idb));
    mGrepDriver->makeKernelCall(s2pk, {ByteStream}, {BasisBits});
 
    StreamSetBuffer * LineBreakStream = mGrepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
    kernel::Kernel * linebreakK = mGrepDriver->addKernelInstance(make_unique<kernel::LineBreakKernelBuilder>(idb, encodingBits));
    mGrepDriver->makeKernelCall(linebreakK, {BasisBits}, {LineBreakStream});
    
    const auto n = REs.size();

    std::vector<StreamSetBuffer *> MatchResultsBufs(n);

    for(unsigned i = 0; i < n; ++i){
        StreamSetBuffer * MatchResults = mGrepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
        kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance(make_unique<kernel::ICGrepKernel>(idb, REs[i]));
        mGrepDriver->makeKernelCall(icgrepK, {BasisBits, LineBreakStream}, {MatchResults});
        MatchResultsBufs[i] = MatchResults;
    }
    StreamSetBuffer * MergedResults = MatchResultsBufs[0];
    if (REs.size() > 1) {
        MergedResults = mGrepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
        kernel::Kernel * streamsMergeK = mGrepDriver->addKernelInstance(make_unique<kernel::StreamsMerge>(idb, 1, REs.size()));
        mGrepDriver->makeKernelCall(streamsMergeK, MatchResultsBufs, {MergedResults});
    }

    kernel::MatchCount matchCountK(idb);
    mGrepDriver->addKernelCall(matchCountK, {MergedResults}, {});
    mGrepDriver->generatePipelineIR();

    idb->setKernel(&matchCountK);
    Value * matchedLineCount = idb->getScalarField("matchedLineCount");
    matchedLineCount = idb->CreateZExt(matchedLineCount, int64Ty);
    
    Value * strideBlocks = ConstantInt::get(int32Ty, idb->getStride() / idb->getBitBlockWidth());
    Value * outputThreadPtr = idb->CreateGEP(outputPtr, idb->CreateAdd(idb->CreateMul(bid, strideBlocks), tid));
    idb->CreateStore(matchedLineCount, outputThreadPtr);
    idb->CreateRetVoid();

    mGrepDriver->finalizeObject();
}

void GrepEngine::grepCodeGen(std::vector<re::RE *> REs, const GrepModeType grepMode, const bool UTF_16, GrepSource grepSource, const GrepType grepType) {

    assert (mGrepDriver == nullptr);
    mGrepDriver = new ParabixDriver("engine");
    auto & idb = mGrepDriver->getBuilder();
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
    
    size_t MatchLimit = ((grepMode == QuietMode) | (grepMode == FilesWithMatch) | (grepMode == FilesWithoutMatch)) ? 1 : MaxCount;

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

        ByteStream = mGrepDriver->addBuffer(make_unique<SourceBuffer>(idb, idb->getStreamSetTy(1, 8)));

        sourceK = mGrepDriver->addKernelInstance(make_unique<kernel::MemorySourceKernel>(idb, idb->getInt8PtrTy(), segmentSize));
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

        ByteStream = mGrepDriver->addBuffer(make_unique<SourceBuffer>(idb, idb->getStreamSetTy(1, 8)));

        if (grepSource == GrepSource::File) {
            sourceK = mGrepDriver->addKernelInstance(make_unique<kernel::MMapSourceKernel>(idb, segmentSize));
            sourceK->setInitialArguments({fileDescriptor});
        } else { // if (grepSource == GrepSource::StdIn) {
            sourceK = mGrepDriver->addKernelInstance(make_unique<kernel::ReadSourceKernel>(idb, segmentSize));
            sourceK->setInitialArguments({idb->getInt32(STDIN_FILENO)});
        }
    }

    mGrepDriver->makeKernelCall(sourceK, {}, {ByteStream});
    StreamSetBuffer * BasisBits = mGrepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(8, 1), segmentSize * bufferSegments));
    
    kernel::Kernel * s2pk = mGrepDriver->addKernelInstance(make_unique<kernel::S2PKernel>(idb));
    mGrepDriver->makeKernelCall(s2pk, {ByteStream}, {BasisBits});
    
    kernel::Kernel * linebreakK = mGrepDriver->addKernelInstance(make_unique<kernel::LineBreakKernelBuilder>(idb, encodingBits));
    StreamSetBuffer * LineBreakStream = mGrepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
    mGrepDriver->makeKernelCall(linebreakK, {BasisBits}, {LineBreakStream});
    
    const auto n = REs.size();

    std::vector<StreamSetBuffer *> MatchResultsBufs(n);

    for(unsigned i = 0; i < n; ++i){
        StreamSetBuffer * MatchResults = mGrepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
        kernel::Kernel * icgrepK = mGrepDriver->addKernelInstance(make_unique<kernel::ICGrepKernel>(idb, REs[i]));
        mGrepDriver->makeKernelCall(icgrepK, {BasisBits, LineBreakStream}, {MatchResults});
        MatchResultsBufs[i] = MatchResults;
    }
    StreamSetBuffer * MergedResults = MatchResultsBufs[0];
    if (REs.size() > 1) {
        MergedResults = mGrepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
        kernel::Kernel * streamsMergeK = mGrepDriver->addKernelInstance(make_unique<kernel::StreamsMerge>(idb, 1, REs.size()));
        mGrepDriver->makeKernelCall(streamsMergeK, MatchResultsBufs, {MergedResults});
    }
    
    if (InvertMatch) {
        kernel::Kernel * invertK = mGrepDriver->addKernelInstance(make_unique<kernel::InvertMatchesKernel>(idb));
        StreamSetBuffer * OriginalMatches = MergedResults;
        MergedResults = mGrepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
        mGrepDriver->makeKernelCall(invertK, {OriginalMatches, LineBreakStream}, {MergedResults});
    }
    if (MatchLimit > 0) {
        kernel::Kernel * untilK = mGrepDriver->addKernelInstance(make_unique<kernel::UntilNkernel>(idb));
        untilK->setInitialArguments({idb->getSize(MatchLimit)});
        StreamSetBuffer * AllMatches = MergedResults;
        MergedResults = mGrepDriver->addBuffer(make_unique<CircularBuffer>(idb, idb->getStreamSetTy(1, 1), segmentSize * bufferSegments));
        mGrepDriver->makeKernelCall(untilK, {AllMatches}, {MergedResults});
    }
    if (grepMode != NormalMode) {
        kernel::Kernel * matchCountK = mGrepDriver->addKernelInstance(make_unique<kernel::MatchCount>(idb));
        mGrepDriver->makeKernelCall(matchCountK, {MergedResults}, {});
        mGrepDriver->generatePipelineIR();
        idb->setKernel(matchCountK);
        Value * matchedLineCount = idb->getScalarField("matchedLineCount");
        matchedLineCount = idb->CreateZExt(matchedLineCount, int64Ty);
        idb->CreateRet(matchedLineCount);
    } else {
        kernel::Kernel * scanMatchK = mGrepDriver->addKernelInstance(make_unique<kernel::ScanMatchKernel>(idb, grepType, encodingBits));
        scanMatchK->setInitialArguments({fileIdx});
        mGrepDriver->makeKernelCall(scanMatchK, {MergedResults, LineBreakStream, ByteStream}, {});
        switch (grepType) {
            case GrepType::Normal:
                if (UTF_16) {
                    mGrepDriver->LinkFunction(*scanMatchK, "matcher", &wrapped_report_match<uint16_t>);
                } else {
                    mGrepDriver->LinkFunction(*scanMatchK, "matcher", &wrapped_report_match<uint8_t>);
                }
                break;
            case GrepType::NameExpression:
                mGrepDriver->LinkFunction(*scanMatchK, "matcher", &insert_codepoints);
                break;
            case GrepType::PropertyValue:
                mGrepDriver->LinkFunction(*scanMatchK, "matcher", &insert_property_values);
                break;
        }
        mGrepDriver->generatePipelineIR();
        idb->CreateRet(idb->getInt64(0));
    }
    mGrepDriver->finalizeObject();
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
: mGrepDriver(nullptr) {

}

GrepEngine::~GrepEngine() {
    delete mGrepDriver;
}

}
