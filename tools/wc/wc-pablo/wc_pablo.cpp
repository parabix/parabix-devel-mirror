/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <IR_Gen/idisa_target.h>
#include <boost/filesystem.hpp>
#include <cc/cc_compiler.h>
#include <cc/cc_compiler_target.h>
#include <kernels/kernel_builder.h>
#include <kernels/pipeline_builder.h>
#include <kernels/s2p_kernel.h>
#include <kernels/source_kernel.h>
#include <kernels/core/streamset.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/Path.h>
#include <llvm/Support/raw_ostream.h>
#include <pablo/pablo_compiler.h>
#include <pablo/pablo_source_kernel.h>
#include <toolchain/pablo_toolchain.h>
#include <pablo/parser/error.h>
#include <pablo/parser/rd_parser.h>
#include <pablo/parser/simple_lexer.h>
#include <toolchain/cpudriver.h>
#include <toolchain/toolchain.h>
#include <util/file_select.h>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace fs = boost::filesystem;

using namespace llvm;
using namespace codegen;
using namespace pablo;
using namespace pablo::parse;

static cl::OptionCategory wcFlags("Command Flags", "wc options");

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore, cl::cat(wcFlags));

std::vector<fs::path> allFiles;

enum CountOptions {
    LineOption, WordOption, CharOption, ByteOption
};

static cl::list<CountOptions> wcOptions(
  cl::values(clEnumValN(LineOption, "l", "Report the number of lines in each input file."),
             clEnumValN(WordOption, "w", "Report the number of words in each input file."),
             clEnumValN(CharOption, "m", "Report the number of characters in each input file (override -c)."),
             clEnumValN(ByteOption, "c", "Report the number of bytes in each input file (override -m).")
             CL_ENUM_VAL_SENTINEL), cl::cat(wcFlags), cl::Grouping);
                                                 
static std::string wc_modes = "";

static int defaultDisplayColumnWidth = 7;  // default field width



bool CountLines = false;
bool CountWords = false;
bool CountChars = false;
bool CountBytes = false;

std::vector<uint64_t> lineCount;
std::vector<uint64_t> wordCount;
std::vector<uint64_t> charCount;
std::vector<uint64_t> byteCount;

uint64_t TotalLines = 0;
uint64_t TotalWords = 0;
uint64_t TotalChars = 0;
uint64_t TotalBytes = 0;

using namespace pablo;
using namespace kernel;
using namespace cc;

//  The callback routine that records counts in progress.
//
extern "C" {
    void record_counts(uint64_t lines, uint64_t words, uint64_t chars, uint64_t bytes, uint64_t fileIdx) {
        lineCount[fileIdx] = lines;
        wordCount[fileIdx] = words;
        charCount[fileIdx] = chars;
        byteCount[fileIdx] = bytes;
        TotalLines += lines;
        TotalWords += words;
        TotalChars += chars;
        TotalBytes += bytes;
    }
}

typedef void (*WordCountFunctionType)(uint32_t fd, uint32_t fileIdx);

WordCountFunctionType wcPipelineGen(CPUDriver & pxDriver, std::shared_ptr<PabloParser> parser, std::shared_ptr<SourceFile> source) {

    auto & iBuilder = pxDriver.getBuilder();

    Type * const int32Ty = iBuilder->getInt32Ty();

    auto P = pxDriver.makePipeline({Binding{int32Ty, "fd"}, Binding{int32Ty, "fileIdx"}});

    Scalar * const fileDescriptor = P->getInputScalar("fd");
    Scalar * const fileIdx = P->getInputScalar("fileIdx");

    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);
    Kernel * mmapK = P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);

    StreamSet * const BasisBits = P->CreateStreamSet(8, 1);
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);

    Kernel * const lck = P->CreateKernelCall<PabloSourceKernel>(
        parser,
        source,
        "LineCount",
        kernel::Bindings{Binding{"bb", BasisBits}},             // input stream bindings
        kernel::Bindings{},                                     // output stream bindings
        kernel::Bindings{},                                     // input scalar bindings
        kernel::Bindings{Binding{iBuilder->getInt64Ty(), "lc"}} // output scalar bindings
    );

    Kernel * const wck = P->CreateKernelCall<PabloSourceKernel>(
        parser,
        source,
        "WordCount",
        kernel::Bindings{Binding{"bb", BasisBits}},             // input stream bindings
        kernel::Bindings{},                                     // output stream bindings
        kernel::Bindings{},                                     // input scalar bindings
        kernel::Bindings{Binding{iBuilder->getInt64Ty(), "wc"}} // output scalar bindings
    );

    Kernel * const cck = P->CreateKernelCall<PabloSourceKernel>(
        parser,
        source,
        "CharCount",
        kernel::Bindings{Binding{"bb", BasisBits}},             // input stream bindings
        kernel::Bindings{},                                     // output stream bindings
        kernel::Bindings{},                                     // input scalar bindings
        kernel::Bindings{Binding{iBuilder->getInt64Ty(), "cc"}} // output scalar bindings
    );

    Scalar * const lineCount = lck->getOutputScalar("lc");
    Scalar * const wordCount = wck->getOutputScalar("wc");
    Scalar * const charCount = cck->getOutputScalar("cc");
    Scalar * const fileSize = mmapK->getOutputScalar("fileItems");
    P->CreateCall("record_counts", record_counts, {lineCount, wordCount, charCount, fileSize, fileIdx});
    return reinterpret_cast<WordCountFunctionType>(P->compile());
}

void wc(WordCountFunctionType fn_ptr, const uint32_t fileIdx) {
    std::string fileName = allFiles[fileIdx].string();
    struct stat sb;
    const int fd = open(fileName.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        if (errno == EACCES) {
            std::cerr << "wc: " << fileName << ": Permission denied.\n";
        }
        else if (errno == ENOENT) {
            std::cerr << "wc: " << fileName << ": No such file.\n";
        }
        else {
            std::cerr << "wc: " << fileName << ": Failed.\n";
        }
        return;
    }
    if (stat(fileName.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
        std::cerr << "wc: " << fileName << ": Is a directory.\n";
        close(fd);
        return;
    }
    fn_ptr(fd, fileIdx);
    close(fd);
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&wcFlags, pablo_toolchain_flags(), codegen::codegen_flags()});
    if (argv::RecursiveFlag || argv::DereferenceRecursiveFlag) {
        argv::DirectoriesFlag = argv::Recurse;
    }

    CPUDriver pxDriver("wc-pablo");
    allFiles = argv::getFullFileList(pxDriver, inputFiles);

    const auto fileCount = allFiles.size();
    if (wcOptions.size() == 0) {
        CountLines = true;
        CountWords = true;
        CountBytes = true;
    } else {
        CountLines = false;
        CountWords = false;
        CountBytes = false;
        CountChars = false;
        for (unsigned i = 0; i < wcOptions.size(); i++) {
            switch (wcOptions[i]) {
                case WordOption: CountWords = true; break;
                case LineOption: CountLines = true; break;
                case CharOption: CountChars = true; CountBytes = false; break;
                case ByteOption: CountBytes = true; CountChars = false; break;
            }
        }
    }
    if (CountLines) wc_modes += "l";
    if (CountWords) wc_modes += "w";
    if (CountChars) wc_modes += "m";
    if (CountBytes) wc_modes += "c";

    auto em = ErrorManager::Create();
    auto lexer = SimpleLexer::Create(em);
    auto parser = RecursiveParser::Create(std::move(lexer), em);
    auto source = SourceFile::Relative("wc.pablo");
    if (source == nullptr) {
        std::cerr << "pablo-parser: error loading pablo source file\n";
        return -1;
    }
    auto wordCountFunctionPtr = wcPipelineGen(pxDriver, parser, source);

    lineCount.resize(fileCount);
    wordCount.resize(fileCount);
    charCount.resize(fileCount);
    byteCount.resize(fileCount);

    for (unsigned i = 0; i < fileCount; ++i) {
        wc(wordCountFunctionPtr, i);
    }
    
    size_t maxCount = 0;
    if (CountLines) maxCount = TotalLines;
    if (CountWords) maxCount = TotalWords;
    if (CountChars) maxCount = TotalChars;
    if (CountBytes) maxCount = TotalBytes;
    
    int displayColumnWidth = std::to_string(maxCount).size() + 1;
    if (displayColumnWidth < defaultDisplayColumnWidth) displayColumnWidth = defaultDisplayColumnWidth;

    for (unsigned i = 0; i < fileCount; ++i) {
        std::cout << std::setw(displayColumnWidth-1);
        if (CountLines) {
            std::cout << lineCount[i] << std::setw(displayColumnWidth);
        }
        if (CountWords) {
            std::cout << wordCount[i] << std::setw(displayColumnWidth);
        }
        if (CountChars) {
            std::cout << charCount[i] << std::setw(displayColumnWidth);
        }
        if (CountBytes) {
            std::cout << byteCount[i];
        }
        std::cout << " " << allFiles[i].string() << std::endl;
    }
    if (inputFiles.size() > 1) {
        std::cout << std::setw(displayColumnWidth-1);
        if (CountLines) {
            std::cout << TotalLines << std::setw(displayColumnWidth);
        }
        if (CountWords) {
            std::cout << TotalWords << std::setw(displayColumnWidth);
        }
        if (CountChars) {
            std::cout << TotalChars << std::setw(displayColumnWidth);
        }
        if (CountBytes) {
            std::cout << TotalBytes;
        }
        std::cout << " total" << std::endl;
    }

    return 0;
}

