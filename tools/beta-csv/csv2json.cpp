//compile to make .exe program
//then drag .csv file to .exe program
//associated json program will be created in the same directory

//or remove arguments from main function, and comment out the
//ifelse statement in main function.
//Then un comment function call below the ifelse
//statement and type in desire input file name as the second argumet 
#include <iostream>
#include <string>
#include <fstream>
#include <map>
#include <queue>
#include <sys/stat.h>
using namespace std;
#define DEFAULT_SEP "," //default seperator

//input a default seperator and a file name
//function convers a .csv file and creates a .json file.
void convert(const string&,const string&,const string&);

//put every field seperated by "seperator" from string "s" in "fields"map and return map
map<int,string> getFields(const string&,const string&);

int main(int args, char* argv[])
{
    cout<<argv[1]<<endl;
    if(args<2)
        cout<<"please specific input file\n"<<endl;
    else 
    {
        string dir="",filename,file=string(argv[1]);
        int start = file.find_last_of("/")+1;
        if(start==file.npos)start=0;
        else dir=file.substr(0,start)+"output/";
        filename=file.substr(start,file.find(".csv"));
        mkdir((dir.substr(0,dir.length()-1)).c_str(),0777);
        for(int i=1;i<args;i++)convert(DEFAULT_SEP,string(argv[i]),dir);
    }
    //uncomment below to manually type in file name and test the program
    //convert(DEFAULT_SEP,"testcase_group.csv");
    return 0;
}

void convert(const string& seperator, const string& file, const string& dir)
{
    ifstream ifile;
    ifile.open(file.c_str());
    string line;
    map<int,string> fields; //to store each field of the header of the .csv file
    map<int,string> record; //to store every field of every line  
    if(!getline(ifile,line))
    {
        cout<<"empty file"<<endl;
        return;
    }
    //string dir="";
    int start = file.find_last_of("/")+1;
    if(start==file.npos)start=0;
    string filename = file.substr(start,file.find(".csv")); //get "file" without extension
    filename += ".json";
    ofstream ofile;
    ofile.open(dir+filename.c_str());
    fields=getFields(seperator,line); //fields of header
    cout<<"["<<endl;
    ofile<<"["<<endl;
    int n= fields.size();
    while(getline(ifile,line))
    {
        if(!record.empty())
        {
            cout<<","<<endl;
            ofile<<","<<endl;
        }
        record=getFields(seperator,line);
        ofile<<"  {"<<endl;
        cout<<"  {"<<endl;
        for(int i=0;i<n;i++)// output field values after field titles  
        {
            cout<<"    \""<<fields[i]<<"\": \""<<record[i]<<"\"";
            ofile<<"    \""<<fields[i]<<"\": \""<<record[i]<<"\"";
            if(i==n-1)
            {
                cout<<endl;
                ofile<<endl;
            }
            else 
            {
                cout<<","<<endl;
                ofile<<","<<endl;
            }
        }
        cout<<"  }";
        ofile<<"  }";
    }
    cout<<endl<<"]"<<endl;
    ofile<<endl<<"]";
    ifile.close();
    ofile.close();
}


map<int,string> getFields(const string& seperator, const string& s)
{
    map<int,string>fields;
    int right,left=0,n=0;
    //continuously find "seperator" index from "left" to the end of "s"
    while((right=s.find_first_of(seperator,left))!=s.npos)  
    {
        //store field from "left" to right before the "seperator"
        fields[n++]=s.substr(left,right-left);
        left=right+1;//
    }
    //store the final field after the fine "seperator" 
    right=s.find_first_of("\r");
    fields[n]=s.substr(left,right-left);
    return fields;
}

