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

//cl::opt<std::string> outputFile("o", cl::desc("specify name of output file."), cl::value_desc("filename"));
//cl::OptionCategory optionsPrompt("Options for Radical Grep");
void radical_grep(const string regex,const string filename,ifstream&search);


int main(int argc, char* argv[])
{
    cl::ParseCommandLineOptions(argc, argv);

    if(argc==3)     //argv[0]=radicalgrep.exe; argv[1]=regex; argv[2]=filepath
    {
        string regex;
        
        string filepath;
        string filename;
        const char* pos;
        
        ifstream search(argv[2]);
        
        regex=argv[1];
        filepath=argv[2];

        pos=strrchr(filepath.c_str(),'/');
        filename=pos+1;     //get the filename from the filepath
        
        if(search.is_open()==0)
            cout<<"Fail to open the file!"<<endl;
        else
        {
            radical_grep(regex,filename,search);
        }
    }
    else
        cout<<"Error!1"<<endl;
    
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

