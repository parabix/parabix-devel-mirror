//compile to make .exe program
//then drag .csv file to .exe program
//associated json program will be created in the same directory

//or remove arguments from main function, and comment out the if-else statement in main function.
//Then uncomment the convert function call below the if-else statement and type in desired input file name as the second argumet 
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
    if(args<2)  cout<<"please specify input file\n"<<endl;
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
    map<int,string> fields; //to map each field of the header of the .csv file to integers 0,1, 2,...
    map<int,string> record; //to store records

    if(!getline(ifile,line))
    {
        cout<<"empty file"<<endl;
        return;
        //checks to see if the .csv file is empty
    }

    string filename = file.substr(0,file.find(".")); //gets the name of the first file to be converted, excluding the file extension
    filename += ".json";
    ofstream ofile;     //creates a file stream to write the .json file
    ofile.open(filename.c_str());   //opens(or creates and opens) a .json with the same name as the .csv file to be converted

    fields=getFields(seperator,line); //fields of header

    ofile<<"["<<endl;   //the .json file created is represented as an array of objects, each object corresponding to a record in the .csv file. this bracket is the opening bracket of the array

    int numberOfFields=fields.size();
    
    while(getline(ifile,line))
    {
        //in each iteration of the loop, a record in the .csv file is converted and written to the .json file
        //the loop terminates when the final line of the .csv file has been read

        if(!record.empty())ofile<<","<<endl;    //checks to see if this is the first object written to the .json file, otherwise a comma and carriage return are written to seperate the previous object from the current object
        
        record=getFields(seperator,line);   
        ofile<<"  {"<<endl; //marks the beginning of a new object in the .json file

        for(int i=0;i<numberOfFields;i++)
        {
            //the loop iterates over the fields of the .csv file, outputting each element of the current record
            ofile<<"    \""<<fields[i]<<"\": \""<<record[i]<<"\"";
            if(i==numberOfFields-1) ofile<<endl;    //checks if this is the last field, if so writes a carriage return to the .json file
            else ofile<<","<<endl;  //otherwise, writes a comma to the .json file to seperate successive fields
        }
        ofile<<"  }";   //marks the end of the object
    }
    ofile<<endl<<"]";   //closes the array that holds the objects in the file

    ifile.close();
    ofile.close();
    return;
}


map<int,string> getFields(const string& seperator, const string& line)  
{
    //takes as arguements a seperator string and a line to be seperated into fields
    //returns a map from the integers 0, 1, 2, ... to the fields in the line

    map<int,string>fields;
    int right, left=0, n=0;
    int seperatorLength=seperator.length()

    //the while-loop below finds the seperator string in the substring of line from left to the end of the line
    //the index of the beginning of the substring is stored in right
    //the loop exits when no instance of the seperator string is found in the remaining substring and, in turn, find_first_of() returns string::npos()
    while((right=line.find_first_of(seperator,left))!=string::npos)
    {
        fields[n++]=sline.substr(left,right-left);
        //stores the field value that is located between left and the index directly preceding the next instance of the seperator string
        left=right+seperatorLength;
        //advances the left index to the position immediately after the end of the current seperator string
    }

    fields[n++]=line.substr(left);
    //after exiting the while loop, the final field (located after the final seperator) is stored to the map

    return fields;
}

