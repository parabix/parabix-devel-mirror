#include <kernel/core/idisa_target.h>
#include <boost/filesystem.hpp>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>
#include <re/adt/adt.h>
#include <re/parse/parser.h>
#include <re/unicode/re_name_resolve.h>
#include <re/cc/cc_kernel.h>
#include <re/ucd/ucd_compiler.hpp>
#include <kernel/core/kernel_builder.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/basis/s2p_kernel.h>
#include <kernel/io/source_kernel.h>
#include <kernel/core/streamset.h>
#include <kernel/unicode/UCD_property_kernel.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/raw_ostream.h>
#include <pablo/pablo_kernel.h>
#include <pablo/builder.hpp>
#include <pablo/pe_zeroes.h>
#include <toolchain/pablo_toolchain.h>
#include <kernel/pipeline/driver/cpudriver.h>
#include <grep/grep_kernel.h>
#include <toolchain/toolchain.h>
#include <fileselect/file_select.h>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <vector>
#include <map>
#include <regex>
#include "radical_interface.h"
#include <unicode/data/kRSKangXi.h>

namespace fs = boost::filesystem;

using namespace llvm;
using namespace codegen;
using namespace kernel;
using namespace BS;

//  Given a Unicode character class (set of Unicode characters), ucount
//  counts the number of occurrences of characters in that class within
//  given input files.

static cl::OptionCategory ucFlags("Command Flags", "ucount options");

static cl::opt<std::string> CC_expr(cl::Positional, cl::desc("<Radical Expression>"), cl::Required, cl::cat(ucFlags));

//  Multiple input files are allowed on the command line; counts are produced
//  for each file.
static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<Input File>"), cl::OneOrMore, cl::cat(ucFlags));

std::vector<fs::path> allFiles;

typedef uint64_t (*UCountFunctionType)(uint32_t fd);

//
//  This is the function that generates and compiles a Parabix pipeline to
//  perform the character counting task on a single input file.   The program
//  takes a re::Name object whose definition includes the UnicodeSet defining
//  the character class.    The compiled pipeline program is returned.
//
//  The compiled pipeline may then be executed.   When executed, it must be given
//  an integer "file descriptor" as its input, and will produce the count of
//  the number of characters of the given character class as a result.
//

UCountFunctionType pipelineGen(CPUDriver & pxDriver, re::Name * CC_name) {

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
    auto nameString = CC_name->getFullName();
    propertyStreamMap.emplace(nameString, CCstream);
    P->CreateKernelCall<UnicodePropertyKernelBuilder>(CC_name, BasisBits, CCstream);

    P->CreateKernelCall<PopcountKernel>(CCstream, P->getOutputScalar("countResult"));

    return reinterpret_cast<UCountFunctionType>(P->compile());
}

//
//  Given a compiled pipeline program for counting  the characters of a class,
//  as well as an index into the global vector of inputFiles,  open the
//  given file and execute the compiled program to produce the count result.
uint64_t ucount1(UCountFunctionType fn_ptr, const uint32_t fileIdx) {
    std::string fileName = allFiles[fileIdx].string();
    struct stat sb;
    const int fd = open(fileName.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        if (errno == EACCES) {
            std::cerr << "ucount: " << fileName << ": Permission denied.\n";
        }
        else if (errno == ENOENT) {
            std::cerr << "ucount: " << fileName << ": No such file.\n";
        }
        else {
            std::cerr << "ucount: " << fileName << ": Failed.\n";
        }
        return 0;
    }
    if (stat(fileName.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)) {
        std::cerr << "ucount: " << fileName << ": Is a directory.\n";
        close(fd);
        return 0;
    }
    uint64_t theCount = fn_ptr(fd);
    close(fd);
    return theCount;
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&ucFlags, codegen::codegen_flags()});
    if (argv::RecursiveFlag || argv::DereferenceRecursiveFlag) {
        argv::DirectoriesFlag = argv::Recurse;
    }
    CPUDriver pxDriver("wc");

    allFiles = argv::getFullFileList(pxDriver, inputFiles);
    const auto fileCount = allFiles.size();

    UCountFunctionType uCountFunctionPtr = nullptr;
    
    //BS::UnicodeSetTable ucd_radical;
    //UCD::UnicodeSet setNum = ucd_radical.get_uset(CC_expr)
    //re::CC* CC_ast = re::makeCC(std::move(UCD::UnicodeSet(UCD::KRS_ns::ucd_radical.get_uset(CC_expr))));
    //uCountFunctionPtr = pipelineGen(pxDriver, makeName(CC_ast));
    
    std::regex regex_pattern("-?[0-9]+.?[0-9]+");
    if ((std::regex_match(CC_expr, regex_pattern))) { 
        UCD::KRS_ns::radSet = BS::ucd_radical.get_uset(CC_expr);
        //re::CC* CC_ast = re::makeCC(std::move(UCD::UnicodeSet(UCD::KRS_ns::_85_Set)));
        re::CC* CC_ast = re::makeCC(std::move(UCD::UnicodeSet(UCD::KRS_ns::radSet)));
        uCountFunctionPtr = pipelineGen(pxDriver, makeName(CC_ast));
    } else {
        llvm::report_fatal_error("Input Error: Enter a integer from 1 to 214.");
    }

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
    }

    return 0;
}
