//
//  radicalgrep.cpp
//  radicalgrep
//
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
#include <llvm/ADT/STLExtras.h> //for make_unique
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <vector>
#include <map>
#include "radical_interface.h"

namespace fs = boost::filesystem;

const int MatchFoundExitCode=0;
const int MatchNotFoundExitCode=1;

using namespace std;
using namespace llvm;
using namespace pablo;
using namespace kernel;
using namespace codegen;

static cl::OptionCategory radicalgrepFlags("Command Flags", "radicalgrep options");

static cl::opt<std::string> input_radical(cl::Positional, cl::desc("<Radical Index>"), cl::Required, cl::cat(radicalgrepFlags));
static cl::list<std::string> inputfiles(cl::Positional, cl::desc("<Input File>"), cl::OneOrMore, cl::cat(radicalgrepFlags));

std::vector<fs::path> allfiles;

std::vector<re::RE*> generateREs(std::string input_radical);

int main(int argc, char* argv[])
{
   codegen::ParseCommandLineOptions(argc,argv,{&radicalgrepFlags,codegen::codegen_flags()});
    if (argv::RecursiveFlag||argv::DereferenceRecursiveFlag)
    {
        argv::DirectoriesFlag=argv::Recurse;
    }
    CPUDriver pxDriver("radicalgrep");
    allfiles=argv::getFullFileList(pxDriver, inputfiles);
    const auto filecount=allfiles.size();
    
    std::unique_ptr<grep::GrepEngine> grep;
    grep = make_unique<grep::EmitMatchesEngine>(pxDriver);
    auto radicalREs=generateREs(input_radical);
    grep->setColoring();
    grep->initFileResult(allfiles);
    grep->initREs(radicalREs);
    grep->grepCodeGen();
    const bool matchFound=grep->searchAllFiles();
    if(matchFound==false)
        cout<<"Can not find the results!"<<endl;
    return matchFound? MatchFoundExitCode : MatchNotFoundExitCode;
}

std::vector<re::RE*> generateREs(std::string input_radical)
{
    BS::RadicalValuesEnumerator en_rad;
    en_rad.parse_input(input_radical);
    return en_rad.createREs();
}

