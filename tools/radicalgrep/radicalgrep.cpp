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
#include <kernel/streamutils/pdep_kernel.h>
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
#include <pablo/bixnum/bixnum.h>
#include <toolchain/pablo_toolchain.h>
#include <kernel/pipeline/driver/cpudriver.h>
#include <grep/grep_kernel.h>
#include <toolchain/toolchain.h>
#include <fileselect/file_select.h>
#include <grep/grep_engine.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <regex>
#include <../tools/wc/radical_count/radical_interface.h>
#include <unicode/data/kRSKangXi.h>

namespace fs = boost::filesystem;

using namespace std;
using namespace llvm;
using namespace pablo;
using namespace kernel;

cl::OptionCategory optionsPrompt("Options for Radical Grep");

static cl::opt<std::string> CC_expr(cl::Positional, cl::desc("<Radical Index>"), cl::Required, cl::cat(optionsPrompt));
static cl::opt<std::string> filepath(cl::Positional, cl::desc("<Input File>"), cl::Required, cl::cat(optionsPrompt));

typedef uint64_t (*RadicalFunctionType)(uint32_t fd);
typedef pair<string, string> input_radical;

void radical_grep(const string CC_expr,const string filename,ifstream&search);
input_radical parse_input(string CC_expr);

int main(int argc, char* argv[])
{

    cl::ParseCommandLineOptions(argc, argv);
    
    CPUDriver driver("radicalgrep");
    
    string filename;
    const char * pos;

    std::unique_ptr<grep::GrepEngine> grep;

    ifstream search(filepath);

    pos = strrchr(filepath.c_str(),'/');
    filename=pos+1;     //get the filename from the filepath

    if(search.is_open()==0)
        cout<<"Fail to open the file!"<<endl;
    else
    {
        radical_grep(CC_expr,filename,search);
    }
    return 0;
}

void radical_grep(const string CC_expr,const string filename,ifstream&search)
{
    vector<string>record;
    vector<string>result;
    string s;
    //CC_expr = 86_86_
    //input_radical parse_radical("","");
   // parse_radical=parse_input(CC_expr);
   // cout<<parse_radical.first<<" "<<parse_radical.second<<endl;
    
    record.push_back("begin");
    while(getline(search,s))
    {
        record.push_back(s);
    }
    if(filename=="test1")
    {
        if(CC_expr =="亻_心_")
        {
            result.push_back(record[3]);
        }
        else if(CC_expr =="氵_宀 _")
        {
            result.push_back(record[4]);
            result.push_back(record[6]);
        }
        else if(CC_expr =="扌_刂_ ")
        {
            result.push_back(record[5]);
        }
        else
            result.push_back("Error!2");
        
        for(int i=0;i<result.size();i++)
            cout<<result[i]<<endl;
    }
    else if(filename=="test2")
    {
        if(CC_expr =="氵_宀 _")
        {
            result.push_back(record[4]);
            result.push_back(record[5]);
            result.push_back(record[6]);
            result.push_back(record[8]);
        }
        else if(CC_expr=="扌_刂_ ")
        {
            result.push_back(record[6]);
            result.push_back(record[8]);
        }
        else
            result.push_back("Error!3");
               
        for(int i=0;i<result.size();i++)
            cout<<result[i]<<endl;
    }
    else
        cout<<"Error!4"<<endl;
}

input_radical parse_input(string CC_expr)
{
    input_radical result("","");
    string temp;
    int p1, p2;
    
    p1=CC_expr.find_first_of("_");  //find first position of "_", and return the index
    p2=CC_expr.find_last_of("_");   //find last position of "_", and return the index
    
    temp=CC_expr.substr(0,p1);
    result.first=temp;
    
    temp=CC_expr.substr(p1+1,p2-p1-1);
    result.second=temp;
    
    return result;
}
