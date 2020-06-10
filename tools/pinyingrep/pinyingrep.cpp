
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
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/Signals.h>
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

/**
 * Command Line Flags
 */
static cl::OptionCategory pygrepFlags("Command Flags", "pinyingrep options");

// Regex-like pinyin syllables input
// For more details about the support, please refer to README-pinyingrep.md in the root directory
static cl::opt<std::string> pyregex(cl::Positional, cl::desc("<Regex-like Pinyin Syllables>"), cl::Required, cl::cat(pygrepFlags));

//  Multiple input files are allowed on the command line; counts are produced
//  for each file.
static cl::list<std::string> inputFiles(cl::Positional, cl::desc("<input file ...>"), cl::OneOrMore, cl::cat(pygrepFlags));

//enable or disable coloring, disabled by default
static cl::opt<bool,false> pyColorOption("c",cl::desc("set coloring of output"),cl::cat(pygrepFlags));
static cl::alias pyColorOptionAlias0("colour",cl::desc("alias for coloring -c"),cl::aliasopt(pyColorOption));

// enable exact match together
static cl::opt<bool, false> pyExactOption("e", cl::desc("enable exact match mode"), cl::cat(pygrepFlags));
static cl::alias pyExactOptionAlias0("exact", cl::desc("alias for exact match mode -e"), cl::aliasopt(pyExactOption));

//switch database to kpy, xhc database is enabled by default
static cl::opt<bool,false> pyKPY("kpy",cl::desc("set database to kpy"),cl::cat(pygrepFlags));
static cl::alias pyKPYAlias0("KPY_Database",cl::desc("alias for kpy database -kpy"),cl::aliasopt(pyKPY));

cl::opt<OptTraditional, All> ChineseCharacterType(cl::desc("Choose Chinese Characters Type(Traditional or Simplified):"),
  cl::values(
    clEnumValN(All,         "all", "Default: grep all Chinese characters, both traditional and simplified."),
    clEnumValN(Traditional, "trd", "Grep Chinese characters used in traditional Chinese."),
    clEnumValN(Simplified,  "sim", "Grep Chinese characters used in simplified Chinese."),
    clEnumValN(TraditionalOnly, "tonly", "Grep Chinese characters if they are only used in traditional Chinese."),
    clEnumValN(SimplifiedOnly,  "sonly", "Grep Chinese characters if they are only used in simplified Chinese.")));

// Option for showing match lines numbers
static cl::opt<bool, false> LineNumberOption("n", cl::desc("Show the line number with each matching line."), cl::cat(pygrepFlags));
static cl::alias LineNumberAlias("line-number", cl::desc("Alias for -n"), cl::aliasopt(LineNumberOption));

// Option for showing file names
static cl::opt<bool, false> WithFilenameOption("h", cl::desc("Show the file name with each matching line."), cl::cat(pygrepFlags));
static cl::alias WithFilenameAlias("with-filename", cl::desc("Alias for -h"), cl::aliasopt(WithFilenameOption));

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
    std::vector<re::RE*> REs(enumerator.createREs((pyKPY)? 1 : 0, ChineseCharacterType));
    if(pyExactOption){
        re::ModeFlagSet InitialFlag = re::MULTILINE_MODE_FLAG;
        re::RE_Syntax RegexpSyntax = re::RE_Syntax::PCRE;
        bool ByteMode = false;
        REs.push_back(re::RE_Parser::parse(pyregex, InitialFlag, RegexpSyntax, ByteMode));
    }
    return std::move(REs);
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
    grep->initFileResult(allFiles); // initialize source files
    // generate REs to initialize the grep engine
    auto pinyinREs = generateREs(pyregex);
    if (pyColorOption) grep->setColoring();
    if (WithFilenameOption) grep->showFileNames();
    if (LineNumberOption) grep->showLineNumbers();
    grep->initREs(pinyinREs); // initialize REs
    grep->grepCodeGen(); // generate pipeline
    
    const bool matchFound = grep->searchAllFiles(); // grep

    return matchFound? MatchFoundExitCode : MatchNotFoundExitCode;
}