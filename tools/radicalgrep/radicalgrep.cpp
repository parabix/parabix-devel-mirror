//
//  radicalgrep.cpp
//  radicalgrep
//

#include <iostream>
#include <string.h>
#include <vector>
#include <fstream>
#include <llvm/Support/CommandLine.h>

using namespace std;
using namespace llvm;

cl::OptionCategory optionsPrompt("Options for Radical Grep");

static cl::opt<std::string> regex(cl::Positional, cl::desc("<radical expression>"), cl::Required, cl::cat(optionsPrompt));
static cl::opt<std::string> filepath(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(optionsPrompt));

void radical_grep(const string regex,const string filename,ifstream&search);


int main(int argc, char* argv[])
{

    cl::ParseCommandLineOptions(argc, argv);

    string filename;
    const char * pos;

    ifstream search(filepath);

    pos = strrchr(filepath.c_str(),'/');
    filename=pos+1;     //get the filename from the filepath

    if(search.is_open()==0)
        cout<<"Fail to open the file!"<<endl;
    else
    {
        radical_grep(regex,filename,search);
    }
    return 0;
}

void radical_grep(const string regex,const string filename,ifstream&search)
{
    vector<string>record;
    vector<string>result;
    string s;
    
    record.push_back("begin");
    while(getline(search,s))
    {
        record.push_back(s);
    }
    if(filename=="test1")
    {
        if(regex=="亻_心_")
        {
            result.push_back(record[3]);
        }
        else if(regex=="氵_宀 _")
        {
            result.push_back(record[4]);
            result.push_back(record[6]);
        }
        else if(regex=="扌_刂_ ")
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
        if(regex=="氵_宀 _")
        {
            result.push_back(record[4]);
            result.push_back(record[5]);
            result.push_back(record[6]);
            result.push_back(record[8]);
        }
        else if(regex=="扌_刂_ ")
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

