/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <kernel/core/idisa_target.h>
#include <boost/filesystem.hpp>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>
#include <re/adt/adt.h>
#include <re/unicode/resolve_properties.h>
#include <re/ucd/ucd_compiler.hpp>
#include <kernel/core/kernel_builder.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/basis/s2p_kernel.h>
#include <kernel/io/source_kernel.h>
#include <kernel/core/streamset.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/raw_ostream.h>
#include <pablo/pablo_kernel.h>
#include <pablo/builder.hpp>
#include <pablo/pe_zeroes.h>
#include <toolchain/pablo_toolchain.h>
#include <kernel/pipeline/driver/cpudriver.h>
#include <toolchain/toolchain.h>
#include <fileselect/file_select.h>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace fs = boost::filesystem;

using namespace llvm;
using namespace codegen;

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
using namespace re;

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

class WordCountKernel final: public pablo::PabloKernel {
public:
    WordCountKernel(BuilderRef b, StreamSet * const countable);
protected:
    void generatePabloMethod() override;
};

WordCountKernel::WordCountKernel (BuilderRef b, StreamSet * const countable)
: PabloKernel(b, "wc_" + wc_modes,
    {Binding{"countable", countable}},
    {},
    {},
    {Binding{b->getSizeTy(), "lineCount"}, Binding{b->getSizeTy(), "wordCount"}, Binding{b->getSizeTy(), "charCount"}}) {

}

void WordCountKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::unique_ptr<cc::CC_Compiler> ccc;
    if (CountWords || CountChars) {
        ccc = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), getInputStreamSet("countable"));
    } else {
        ccc = make_unique<cc::Direct_CC_Compiler>(getEntryScope(), pb.createExtract(getInput(0), pb.getInteger(0)));
    }
    
    PabloAST * u8final = nullptr;
    if (CountWords || CountChars) {
        Zeroes * const ZEROES = pb.createZeroes();
        PabloAST * const u8pfx = ccc->compileCC(re::makeByte(0xC0, 0xFF));

        Var * const nonFinal = pb.createVar("nonFinal", u8pfx);
        Var * const u8invalid = pb.createVar("u8invalid", ZEROES);
        Var * const valid_pfx = pb.createVar("valid_pfx", u8pfx);

        auto it = pb.createScope();
        pb.createIf(u8pfx, it);
        PabloAST * const u8pfx2 = ccc->compileCC(re::makeByte(0xC2, 0xDF), it);
        PabloAST * const u8pfx3 = ccc->compileCC(re::makeByte(0xE0, 0xEF), it);
        PabloAST * const u8pfx4 = ccc->compileCC(re::makeByte(0xF0, 0xF4), it);

        //
        // Two-byte sequences
        Var * const anyscope = it.createVar("anyscope", ZEROES);
        auto it2 = it.createScope();
        it.createIf(u8pfx2, it2);
        it2.createAssign(anyscope, it2.createAdvance(u8pfx2, 1));


        //
        // Three-byte sequences
        Var * const EF_invalid = it.createVar("EF_invalid", ZEROES);
        auto it3 = it.createScope();
        it.createIf(u8pfx3, it3);
        PabloAST * const u8scope32 = it3.createAdvance(u8pfx3, 1);
        it3.createAssign(nonFinal, it3.createOr(nonFinal, u8scope32));
        PabloAST * const u8scope33 = it3.createAdvance(u8pfx3, 2);
        PabloAST * const u8scope3X = it3.createOr(u8scope32, u8scope33);
        it3.createAssign(anyscope, it3.createOr(anyscope, u8scope3X));

        PabloAST * const advE0 = it3.createAdvance(ccc->compileCC(re::makeByte(0xE0), it3), 1, "advEO");
        PabloAST * const range80_9F = ccc->compileCC(re::makeByte(0x80, 0x9F), it3);
        PabloAST * const E0_invalid = it3.createAnd(advE0, range80_9F, "E0_invalid");

        PabloAST * const advED = it3.createAdvance(ccc->compileCC(re::makeByte(0xED), it3), 1, "advED");
        PabloAST * const rangeA0_BF = ccc->compileCC(re::makeByte(0xA0, 0xBF), it3);
        PabloAST * const ED_invalid = it3.createAnd(advED, rangeA0_BF, "ED_invalid");

        PabloAST * const EX_invalid = it3.createOr(E0_invalid, ED_invalid);
        it3.createAssign(EF_invalid, EX_invalid);

        //
        // Four-byte sequences
        auto it4 = it.createScope();
        it.createIf(u8pfx4, it4);
        PabloAST * const u8scope42 = it4.createAdvance(u8pfx4, 1, "u8scope42");
        PabloAST * const u8scope43 = it4.createAdvance(u8scope42, 1, "u8scope43");
        PabloAST * const u8scope44 = it4.createAdvance(u8scope43, 1, "u8scope44");
        PabloAST * const u8scope4nonfinal = it4.createOr(u8scope42, u8scope43);
        it4.createAssign(nonFinal, it4.createOr(nonFinal, u8scope4nonfinal));
        PabloAST * const u8scope4X = it4.createOr(u8scope4nonfinal, u8scope44);
        it4.createAssign(anyscope, it4.createOr(anyscope, u8scope4X));
        PabloAST * const F0_invalid = it4.createAnd(it4.createAdvance(ccc->compileCC(re::makeByte(0xF0), it4), 1), ccc->compileCC(re::makeByte(0x80, 0x8F), it4));
        PabloAST * const F4_invalid = it4.createAnd(it4.createAdvance(ccc->compileCC(re::makeByte(0xF4), it4), 1), ccc->compileCC(re::makeByte(0x90, 0xBF), it4));
        PabloAST * const FX_invalid = it4.createOr(F0_invalid, F4_invalid);
        it4.createAssign(EF_invalid, it4.createOr(EF_invalid, FX_invalid));

        //
        // Invalid cases
        PabloAST * const legalpfx = it.createOr(it.createOr(u8pfx2, u8pfx3), u8pfx4);
        //  Any scope that does not have a suffix byte, and any suffix byte that is not in
        //  a scope is a mismatch, i.e., invalid UTF-8.
        PabloAST * const u8suffix = ccc->compileCC("u8suffix", re::makeByte(0x80, 0xBF), it);
        PabloAST * const mismatch = it.createXor(anyscope, u8suffix);
        //
        PabloAST * const pfx_invalid = it.createXor(valid_pfx, legalpfx);
        it.createAssign(u8invalid, it.createOr(pfx_invalid, it.createOr(mismatch, EF_invalid)));
        PabloAST * const u8valid = it.createNot(u8invalid, "u8valid");
        //
        //
        it.createAssign(nonFinal, it.createAnd(nonFinal, u8valid));

        u8final = pb.createInFile(pb.createNot(nonFinal));
    }

    //  output: 3 counters
    Var * lc = getOutputScalarVar("lineCount");
    Var * wc = getOutputScalarVar("wordCount");
    Var * cc = getOutputScalarVar("charCount");

    if (CountLines) {
        PabloAST * LF = ccc->compileCC(re::makeByte(0x0A));
        pb.createAssign(lc, pb.createCount(LF));
    }
    if (CountWords) {
        re::RE * WS_prop = re::makePropertyExpression("space");
        WS_prop = UCD::linkAndResolve(WS_prop);
        re::CC * WS_CC = cast<re::CC>(cast<PropertyExpression>(WS_prop)->getResolvedRE());
        Var * WS = pb.createVar("space");
        UCD::UCDCompiler unicodeCompiler(*ccc.get(), pb);
        unicodeCompiler.addTarget(WS, WS_CC);
        unicodeCompiler.compile();
        //PabloAST * WS = ccc->compileCC(re::makeCC(re::re::makeByte(0x09, 0x0D), re::re::makeByte(0x20)));
        PabloAST * wordChar = pb.createAnd(pb.createNot(WS), u8final, "wordChar");
        // WS_follow_or_start = 1 past WS or at start of file
        PabloAST * WS_follow_or_start = pb.createNot(pb.createIndexedAdvance(wordChar, u8final, 1), "WS_follow_or_start");
        PabloAST * wordStart = pb.createInFile(pb.createAnd(wordChar, WS_follow_or_start), "wordStart");
        pb.createAssign(wc, pb.createCount(wordStart));
    }
    if (CountChars) {
        pb.createAssign(cc, pb.createCount(u8final));
    }
}

typedef void (*WordCountFunctionType)(uint32_t fd, uint32_t fileIdx);

WordCountFunctionType wcPipelineGen(CPUDriver & pxDriver) {

    auto & iBuilder = pxDriver.getBuilder();

    Type * const int32Ty = iBuilder->getInt32Ty();

    auto P = pxDriver.makePipeline({Binding{int32Ty, "fd"}, Binding{int32Ty, "fileIdx"}});

    Scalar * const fileDescriptor = P->getInputScalar("fd");
    Scalar * const fileIdx = P->getInputScalar("fileIdx");

    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);

    Kernel * mmapK = P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);

    auto CountableStream = ByteStream;
    if (CountWords || CountChars) {
        auto BasisBits = P->CreateStreamSet(8, 1);
        P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);
        CountableStream = BasisBits;
    }

    Kernel * const wck = P->CreateKernelCall<WordCountKernel>(CountableStream);

    Scalar * const lineCount = wck->getOutputScalarAt(0);
    Scalar * const wordCount = wck->getOutputScalarAt(1);
    Scalar * const charCount = wck->getOutputScalarAt(2);
    Scalar * const fileSize = mmapK->getOutputScalarAt(0);

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
    CPUDriver pxDriver("wc");

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

    auto wordCountFunctionPtr = wcPipelineGen(pxDriver);

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
    if (allFiles.size() > 1) {
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
