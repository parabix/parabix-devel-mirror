//
//  radicalgrep.cpp
//  radicalgrep
//  group delta
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
#include <llvm/Support/ErrorHandling.h> 
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
#include <sstream>
#include <time.h>
#include "radical_interface.h"

namespace fs = boost::filesystem;

using namespace std;
using namespace llvm;
using namespace pablo;
using namespace kernel;
using namespace codegen;

static cl::OptionCategory radicalgrepFlags("Command Flags", "Options for Radical Grep"); //The command line
static cl::opt<std::string> input_radical(cl::Positional, cl::desc("<Radical Index>"), cl::Required, cl::cat(radicalgrepFlags));    //The input  radical(s)
static cl::list<std::string> inputfiles(cl::Positional, cl::desc("<Input File>"), cl::OneOrMore, cl::cat(radicalgrepFlags));    //search for multiple input files is supported
static cl::opt<bool> indexMode("i", cl::desc("Use radical index instead of the radical character to perform search.\n Link to Radical Indices: https://www.yellowbridge.com/chinese/radicals.php"), cl::init(false), cl::cat(radicalgrepFlags)); 

static cl::opt<bool> mixMode("m", cl::desc("Use both radical character and radical index to perform search."), cl::init(false), cl::cat(radicalgrepFlags));
static cl::opt<bool> altMode("alt", cl::desc("Use regular expressions to search for multiple phrases."), cl::init(false), cl::cat(radicalgrepFlags));
//Adpated from grep_interface.cpp
ColoringType ColorFlag;
static cl::opt<ColoringType, true> Color("c", cl::desc("Set the colorization of the output."),
                                 cl::values(clEnumValN(alwaysColor, "always", "Turn on colorization when outputting to a file and terminal"),
                                            clEnumValN(autoColor,   "auto", "Turn on colorization only when outputting to terminal"),
                                            clEnumValN(neverColor,  "never", "Turn off output colorization")
                                            CL_ENUM_VAL_SENTINEL), cl::cat(radicalgrepFlags), cl::location(ColorFlag), cl::init(neverColor));
bool LineNumberFlag, WithFilenameFlag;
static cl::opt<bool, true> LineNumberOption("n", cl::location(LineNumberFlag), cl::desc("Show the line number with each matching line."), cl::cat(radicalgrepFlags));
static cl::opt<bool, true> WithFilenameOption("h", cl::location(WithFilenameFlag), cl::desc("Show the file name with each matching line."), cl::cat(radicalgrepFlags));
bool CLKCountingFlag;
static cl::opt<bool, true> CLKCountingOption("clk", cl::location(CLKCountingFlag), cl::desc("Show the runtime of the function."), cl::cat(radicalgrepFlags));
std::vector<fs::path> allfiles; //Store all path of files

std::vector<re::RE*> generateREs(std::string input_radical, bool altMode);    //This function parse the input and get the results

int main(int argc, char* argv[])
{
   codegen::ParseCommandLineOptions(argc,argv,{&radicalgrepFlags,codegen::codegen_flags()});

    if (argv::RecursiveFlag||argv::DereferenceRecursiveFlag)
    {
        argv::DirectoriesFlag=argv::Recurse;
    }

    CPUDriver pxDriver("radicalgrep");
    allfiles=argv::getFullFileList(pxDriver, inputfiles);

    if ((allfiles.size() > 1)) WithFilenameFlag = true;
    
    std::unique_ptr<grep::GrepEngine> grep;
    grep = make_unique<grep::EmitMatchesEngine>(pxDriver);
    
    long begintime;
     if(CLKCountingFlag==true)
         begintime=clock();
    auto radicalREs=generateREs(input_radical, altMode); //get the results

    if (WithFilenameFlag) grep->showFileNames();
    if (LineNumberFlag) grep->showLineNumbers();

    //turn on colorizartion if specified by user
    if ((ColorFlag == alwaysColor) || ((ColorFlag == autoColor) && isatty(STDOUT_FILENO))) grep->setColoring();
    

    grep->initFileResult(allfiles); //Defined in file grep_engine, Initialize results of each file
    grep->initREs(radicalREs);  //Defined in file grep_engine, Initialize the output
    grep->grepCodeGen();    //Return the number of the result
    const bool matchFound=grep->searchAllFiles();   //Return if there have found any result, if yes, return true, else return false

    //if there does not exist any results
    if(matchFound==false) cout<<"Can not find the results!"<<endl;

    if(CLKCountingFlag==true)
    {
        long endtime=clock();
        cout<<"the runtime is:"<<(endtime-begintime)*1.0/CLOCKS_PER_SEC<<"s."<<endl;
    }
    
    return matchFound? MatchFoundExitCode : MatchNotFoundExitCode;
}

std::vector<re::RE*> generateREs(std::string input_radical, bool altMode)
{
    BS::RadicalValuesEnumerator en_rad;
    en_rad.parse_input(input_radical, altMode);  //parse the input 
    return en_rad.createREs(indexMode, mixMode, altMode);
}

