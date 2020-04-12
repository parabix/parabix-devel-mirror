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
using namespace std;
#define DEFAULT_SEP "," //default seperator

//input a default seperator and a file name
//function convers a .csv file and creates a .json file.
void convert(const string&,const string&);

//put every field seperated by "seperator" from string "s" in "fields"map and return map
map<int,string> getFields(const string&,const string&);

int main(int args, char* argv[])
{
    if(args<2)cout<<"please specific input file\n"<<endl;
    else for(int i=1;i<args;i++)convert(DEFAULT_SEP,string(argv[i]));

    //uncomment below to manually type in file name and test the program
    //convert(DEFAULT_SEP,"testcase_group.csv");
    return 0;
}

void convert(const string& seperator, const string& file)
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
    string filename = file.substr(0,file.find(".")); //get "file" without extension
    filename += ".json";
    ofstream ofile;
    ofile.open(filename.c_str());
    fields=getFields(seperator,line); //fields of header
    ofile<<"["<<endl;
    int n= fields.size();
    while(getline(ifile,line))
    {
        if(!record.empty())ofile<<","<<endl;
        record=getFields(seperator,line);
        ofile<<"  {"<<endl;
        for(int i=0;i<n;i++)// output field values after field titles  
        {
            ofile<<"    \""<<fields[i]<<"\": \""<<record[i]<<"\"";
            if(i==n-1)ofile<<endl;
            else ofile<<","<<endl;
        }
        ofile<<"  }";
    }
    ofile<<endl<<"]";
    ifile.close();
    ofile.close();
}


map<int,string> getFields(const string& seperator, const string& s)
{
    map<int,string>fields;
    int right,left=0,len,n=0;
    //continuously find "seperator" index from "left" to the end of "s"
    while((right=s.find_first_of(seperator,left))<s.length())  
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

