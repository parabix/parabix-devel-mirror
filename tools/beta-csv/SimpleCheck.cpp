//Simple test driver
//executable target program as argv[1]              (program)
//folder path for source of testcases as argv[2]    (sourcepath)
//folder path for program outputs as argv[3]        (destpath)
//folder path for expected outputs as argv[4]       (expectpath)
//
//This program reads the names of all the testcases in sourcepath folder
//then executes the target program with every testcase
//lastly compares outputs from the destpath folder with
//expected outputs from the expectpath folder
//
//prints pass or fail
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include<sys/stat.h>
#include <vector>
#include<stdlib.h>
#include<dirent.h>
using namespace std;

#define _MAX_FNAME 100
vector<string> getAllFiles(const string &path);
void ExecuteFiles(const string &spath,const string& dpath, const string &program,const vector<string>& files);
void CompareFiles(const string &epath, const string &dpath,const vector<string>&files);

int main(int args, char* argv[])
{
    //string spath="/home/vincent/Desktop/parabix-devel/QA/csv2json/testcases/";
    //string dpath="/home/vincent/Desktop/parabix-devel/QA/csv2json/outs/";
    //string program="/home/vincent/Desktop/parabix-devel/build/bin/csv2json";
    //string epath="/home/vincent/Desktop/parabix-devel/QA/csv2json/expect/";
    string program=string(argv[1]);
    string sourcepath=string(argv[2]);
    string destpath=string(argv[3]);
    string expectpath=string(argv[4]);
    int count=0;
    vector<string>files;
    string line;
    files = getAllFiles(sourcepath);
    ExecuteFiles(sourcepath,destpath,program,files);
    cout<<endl<<endl<<"Initiating csv2jsontest"<<endl<<endl;
    CompareFiles(expectpath,destpath,files);
    cout<<endl<<"csv2jsontest completed"<<endl;
    return 0;
}

vector<string> getAllFiles(const string &path)
{
    vector<string> files;
    DIR* folder=opendir(path.c_str());
    if(!folder)
    {
        cout<<"Empty folder!"<<endl;
        return files;
    }
    struct dirent* file = NULL;
    size_t index;
    while(file=readdir(folder))
    {
        string filename=file->d_name;
        if((index=filename.find(".csv"))!=filename.npos)
        files.push_back(filename.substr(0,index));
    }
    closedir(folder);
    return files;
}

void ExecuteFiles(const string &spath,const string &dpath, const string &program,const vector<string>& files)
{
    DIR* folder=opendir(dpath.c_str());
    if(!folder)mkdir(dpath.c_str(),0777);
    for(int i=0;i<files.size();i++)
    {
        string dfile=dpath+files[i]+".json";
        string sfile=spath+files[i]+".csv";
        string cmd=program+" "+sfile+" "+dfile;
        system(cmd.c_str());
    }
    closedir(folder);
}

void CompareFiles(const string &epath, const string &dpath,const vector<string>&files)
{
    ifstream expect,out;
    string line1,line2;
    bool pass;
    for(int i=0;i<files.size();i++)
    {
        pass=true;
        expect.open((epath+files[i]+".json").c_str());
        out.open((dpath+files[i]+".json").c_str());
        while(getline(expect,line1))
        {
            getline(out,line2);
            if(line1!=line2)
            {
                pass=false;
                break;
            } 
        }
        expect.close();
        out.close();
        cout<<"testcase: "<<files[i];
        if(pass)cout<<" passed"<<endl;
        else cout<<" failed"<<endl;
    }
}