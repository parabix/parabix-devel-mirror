#include <cstdio>
#include <vector>
#include <llvm/ADT/STLExtras.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/PrettyStackTrace.h>
#include <llvm/Support/Signals.h>
#include <llvm/Support/ManagedStatic.h>
#include <llvm/Support/raw_ostream.h>
#include <pablo/pablo_kernel.h>
#include <pablo/builder.hpp>
#include <pablo/pe_zeroes.h>
#include <re/parse/parser.h>
#include <re/toolchain/toolchain.h>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>
#include <re/adt/adt.h>
#include <re/unicode/re_name_resolve.h>
#include <re/cc/cc_kernel.h>
#include <re/ucd/ucd_compiler.hpp>
#include <grep/grep_engine.h>
#include <grep/grep_kernel.h>
#include <fstream>
#include <string>
#include <map>
#include <toolchain/toolchain.h>
#include <toolchain/pablo_toolchain.h>
#include <boost/filesystem.hpp>
#include <fileselect/file_select.h>
#include <sys/stat.h>
#include <fcntl.h>
//#include <llvm/ADT/STLExtras.h> // for make_unique
#include <kernel/pipeline/driver/cpudriver.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/basis/s2p_kernel.h>
#include <kernel/io/source_kernel.h>
#include <kernel/core/streamset.h>
#include <kernel/unicode/UCD_property_kernel.h>
#include <kernel/core/idisa_target.h>
#include <boost/filesystem.hpp>
#include <util/aligned_allocator.h>
#include "../icgrep/grep_interface.h"
#include "pinyin.h"

namespace fs = boost::filesystem;

using namespace llvm;
using namespace codegen;
using namespace kernel;

static cl::OptionCategory pygrepFlags("Command Flags", "pinyingrep options");

static cl::opt<std::string> PinyinLinePattern(cl::Positional, cl::desc("Pinyin Syllables"), cl::Required, cl::cat(pygrepFlags));

static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore, cl::cat(pygrepFlags));

std::vector<fs::path> allFiles;

typedef uint64_t (*UCountFunctionType)(uint32_t fd);

//step4
std::vector<re::RE*> generateREs(std::vector<std::string> KangXiLinePattern){
    std::vector<re::RE*> PinyinCC;
    //re::RE* PinyinCC_final;
    for(size_t i= 0;i<KangXiLinePattern.size();i++)
    {
        re::RE * PinyinRe = re::RE_Parser::parse(KangXiLinePattern[i], 0U, re::PCRE, false);
       //how to find the unicode? 
        PinyinCC.push_back(PinyinRe);
    }
    return PinyinCC;
    //PinyinCC_final = re::makeSeq(PinyinCC.begin(),PinyinCC.end());
    //return PinyinCC_final;
}

UCountFunctionType pipelineGen(CPUDriver & pxDriver, re::RE * pinyinCC) {

    auto & B = pxDriver.getBuilder();

    auto P = pxDriver.makePipeline(
                {Binding{B->getInt32Ty(), "fileDescriptor"}},
                {Binding{B->getInt64Ty(), "countResult"}});

    Scalar * const fileDescriptor = P->getInputScalar("fileDescriptor");

    //  Create a stream set consisting of a single stream of 8-bit units (bytes).
    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);

    //  Read the file into the ByteStream.
    P->CreateKernelCall<ReadSourceKernel>(fileDescriptor, ByteStream);

    //  Create a set of 8 parallel streams of 1-bit units (bits).
    StreamSet * const BasisBits = P->CreateStreamSet(8, 1);

    //  Transpose the ByteSteam into parallel bit stream form.
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);

    //  Create a character class bit stream.
    StreamSet * CCstream = P->CreateStreamSet(1, 1);
    
    std::map<std::string, StreamSet *> propertyStreamMap;
    auto nameString = "kHanyuPinyin:" + PinyinLinePattern;
    propertyStreamMap.emplace(nameString, CCstream);
    P->CreateKernelCall<UnicodePropertyKernelBuilder>(makeName(nameString, pinyinCC), BasisBits, CCstream);

    P->CreateKernelCall<PopcountKernel>(CCstream, P->getOutputScalar("countResult"));

    return reinterpret_cast<UCountFunctionType>(P->compile());
}

uint64_t ucount1(UCountFunctionType fn_ptr, const uint32_t fileIdx) {
    std::string fileName = allFiles[fileIdx].string();
    struct stat sb;
    const int fd = open(fileName.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        if (errno == EACCES) {
            std::cerr << "pinyincount: " << fileName << ": Permission denied.\n";
        }
        else if (errno == ENOENT) {
            std::cerr << "pinyincount: " << fileName << ": No such file.\n";
        }
        else {
            std::cerr << "pinyincount: " << fileName << ": Failed.\n";
        }
        return 0;
    }
    if (stat(fileName.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
        std::cerr << "pinyincount: " << fileName << ": Is a directory.\n";
        close(fd);
        return 0;
    }
    uint64_t theCount = fn_ptr(fd);
    close(fd);
    return theCount;
}

int main(int argc, char* argv[]){
    PinyinPattern::Buffer buf;
    AlignedAllocator <char, 32> alloc;
    char * UnihanBuf;
    
    codegen::ParseCommandLineOptions(argc, argv, {&pygrepFlags, codegen::codegen_flags()});
    if (argv::RecursiveFlag || argv::DereferenceRecursiveFlag) {
        argv::DirectoriesFlag = argv::Recurse;
    }
    
    CPUDriver pxDriver("pygrep");
    allFiles = argv::getFullFileList(pxDriver, inputFiles);
    const auto fileCount = allFiles.size();
    //step2
    std::vector <std::string> KangXiLinePattern;
    //string Search_Prefix = "kHanyuPinyin.*";
    KangXiLinePattern = PinyinPattern::Before_Search(PinyinLinePattern);
    //here needs step3
    UnihanBuf = alloc.allocate(buf.R_size32(), 0);
    std::memcpy(UnihanBuf, buf.R_fstring().data(),buf.R_size());
    std::memset(UnihanBuf + buf.R_size(), 0, buf.R_diff());
    //step4
    auto KangXilineREs = generateREs(KangXiLinePattern);
    //auto PinyinCC = re::makeSeq(KangXilineREs.begin(),KangXilineREs.end());
    PinyinPattern::PinyinSetAccumulator accum;
    std::vector <re::RE*> VectorRE;
    for (auto word: KangXilineREs){
        auto PinyinRE = word;
        grep::InternalSearchEngine engine(pxDriver);
        engine.setRecordBreak(grep::GrepRecordBreakKind::LF);
        engine.grepCodeGen(PinyinRE);
        engine.doGrep(UnihanBuf, buf.R_size32(), accum); 
        resolveUnicodeNames(PinyinRE);
    }
    
    alloc.deallocate(UnihanBuf, 0);
    re::CC * CC_ast = re::makeCC(accum.getAccumulatedSet());
    VectorRE.push_back(CC_ast);

    UCountFunctionType uCountFunctionPtr = pipelineGen(pxDriver, CC_ast);
    /*
    auto PinyinRE = KangXilineREs[0];
    //step5 for each RE,use parabix internal search Engine to search
    PinyinPattern::PinyinSetAccumulator accum;
    
    // what is this driver it cannot be sepcified by the IDE
    grep::InternalSearchEngine engine(pxDriver);
    //

    engine.setRecordBreak(grep::GrepRecordBreakKind::LF);
    //cannot using this KangXiLinePattern, must use RE int the Vector KangXiLineREs, 
    //do not kown what this function used to do
    engine.grepCodeGen(PinyinRE);

    engine.doGrep(UnihanBuf, buf.R_size32(), accum);
    alloc.deallocate(UnihanBuf, 0);
    
    resolveUnicodeNames(PinyinRE);
    std::vector <re::RE*> VectorRE;

    re::CC * CC_ast = re::makeCC(accum.getAccumulatedSet());
    VectorRE.push_back(CC_ast);

    UCountFunctionType uCountFunctionPtr = pipelineGen(pxDriver, CC_ast); */
    /*
    std::vector<uint64_t> theCounts;
    theCounts.resize(fileCount);
    uint64_t totalCount = 0;
    for (unsigned i = 0; i < fileCount; ++i) {
        theCounts[i] = ucount1(uCountFunctionPtr, i);
        totalCount += theCounts[i];
    }
    
    const int defaultDisplayColumnWidth = 7;
    int displayColumnWidth = std::to_string(totalCount).size() + 1;
    if (displayColumnWidth < defaultDisplayColumnWidth) displayColumnWidth = defaultDisplayColumnWidth;

    for (unsigned i = 0; i < fileCount; ++i) {
        std::cout << std::setw(displayColumnWidth);
        std::cout << theCounts[i] << std::setw(displayColumnWidth);
        std::cout << " " << allFiles[i].string() << std::endl;
    }
    if (inputFiles.size() > 1) {
        std::cout << std::setw(displayColumnWidth-1);
        std::cout << totalCount;
        std::cout << " total" << std::endl;
    }*/

    std::unique_ptr<grep::GrepEngine> grep = make_unique<grep::EmitMatchesEngine>(pxDriver);
    grep->initFileResult(allFiles);
    grep->initREs(VectorRE);
    grep->grepCodeGen();
    const bool matchFound = grep->searchAllFiles();

    return matchFound ? argv::MatchFoundExitCode : argv::MatchNotFoundExitCode;
}
