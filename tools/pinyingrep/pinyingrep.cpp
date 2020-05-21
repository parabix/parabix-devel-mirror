
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
#include <grep/grep_engine.h>
#include <toolchain/toolchain.h>
#include <fileselect/file_select.h>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <vector>
#include <map>
#include "pinyin_interface.h"

// Exit codes for grep
const int MatchFoundExitCode = 0;
const int MatchNotFoundExitCode = 1;

namespace fs = boost::filesystem;

using namespace llvm;
using namespace codegen;
using namespace kernel;

// Given regular-expression-like pinyin input, pinyingrep will grep the lines
// containing corresponding Chinese characters or Chinese phrases.

static cl::OptionCategory pygrepFlags("Command Flags", "pinyingrep options");

// Regex-like pinyin syllables input
// For more details about the support, please refer to README-pinyingrep.md in the root directory
static cl::opt<std::string> pyregex(cl::Positional, cl::desc("<Regex-like Pinyin Syllables>"), cl::Required, cl::cat(pygrepFlags));

//  Multiple input files are allowed on the command line; counts are produced
//  for each file.
static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore, cl::cat(pygrepFlags));

// the source files to grep from
std::vector<fs::path> allFiles;

std::vector<re::RE*> generateREs(std::string pyregex){
    // Separate the input string containing multiple pinyin syllables 
    // into the sequence of individual syllable strings. 
    // And for each individual syllable, it parse it into `<{Syllables}, {Tones}>` pairs.
    PY::PinyinValuesParser parser;
    parser.parse(pyregex);
    
    // enumerate the pair of vector mentioned above 
    // into `{<syllable, tone>,...}` a vector of pairs.
    // for every individual syllable
    PY::PinyinValuesEnumerator enumerator;
    enumerator.enumerate(parser);

    // Create re::REs from the enumerated result
    // to initialize the grep engine of Parabix
    return enumerator.createREs();
}
// Reference: icgrep
int main(int argc, char* argv[]){
    // Built-in CommandLine Parser
    codegen::ParseCommandLineOptions(argc, argv, {&pygrepFlags, codegen::codegen_flags()});
    if (argv::RecursiveFlag || argv::DereferenceRecursiveFlag) {
        argv::DirectoriesFlag = argv::Recurse;
    }
    CPUDriver pxDriver("pygrep");
    // Parsed input files
    allFiles = argv::getFullFileList(pxDriver, inputFiles);

    // Parabix grep engine
    std::unique_ptr<grep::GrepEngine> grep =  make_unique<grep::EmitMatchesEngine>(pxDriver); 
    // generate REs to initialize the grep engine
    auto pinyinREs = generateREs(pyregex);
    //grep->setColoring();
    grep->initREs(pinyinREs); // initialize REs
    grep->grepCodeGen(); // generate pipeline
    grep->initFileResult(allFiles); // initialize source files
    const bool matchFound = grep->searchAllFiles(); // grep

    return matchFound? MatchFoundExitCode : MatchNotFoundExitCode;
}