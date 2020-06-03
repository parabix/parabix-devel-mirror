/*	This csv2json file_zero can pass the examples meeting the following requirements.
/*  1.The field values can include numbers, letters, characters and space.
/*	2.The field values can include symbols except for the separator of this file and NewLine.
/*  3.Some field values can be empty but not the field name.Otherwise this field will be ignored. 
*/

#include<iostream>
#include<fstream>
#include<sstream>
#include<string>
#include<vector>
using namespace std;

int main(int argc, char* argv[]) {
	string ipath, opath;
	if (argc != 3) {
		cout << "arguement error";
		system("pause");
		exit(-1);
	}
	//argv[0] is the path of the path of the .exe file.
	//argv[1] and argv[2] are the paths of the input file and the output file respectively.
	ipath = string(argv[1]);
	opath = string(argv[2]);
	const char separator = ',';			//specify the separator type, now it is comma.
	const char JSON_separator = ',';    //specify the JSON_separator type, now it is comma.
	ifstream csvInput;
	ofstream jsonOutput;
	csvInput.open(ipath, ios::in);
	jsonOutput.open(opath, ios::out | ios::trunc);

	string Oneline;						//Oneline is to store each single line of the input.csv file.
	string str;							//str is to store each separated field value in lines.
	vector<string> FieldNameArray;		//FieldNameArray is to an array to store field names.
	int LineCounter = 0;
	int FieldNumber = 0;            
	int ValueCounter = 0;
	jsonOutput << "[\n";				//print JSON_prefix , now it is Left Square Bracket.

	while (getline(csvInput, Oneline))	//If not to the end of the input.csv file, get one line and store it in Oneline.
	{									//In Windows, \r\n means carriage return and newline; while in Linux, \n does the same job.
#ifdef _WIN32 							// _WIN32 stands for both Windows 32-bit and 64-bit
#else									// If the operating system is not Windows, then \r should be discarded.
Oneline = Oneline.replace(Oneline.find("\r"), 1, "");
#endif
		vector<string> FieldArray;      //FieldArray is to store the field values in the current line.
		stringstream ss(Oneline);      
		ValueCounter = 0;
		LineCounter++;				    //LineCounter is the number of the current line.

		if (LineCounter >= 3) {
			jsonOutput << "},\n";
		}
		if (LineCounter >= 2) {
			jsonOutput << "{";
		}
		while (getline(ss, str, separator)) { 
			//To find str(the field value) separated by separator in ss(the current line).
			if (LineCounter == 1) {     //If it is the first line, then there are field names.
				FieldNumber++;
				FieldNameArray.push_back(str);
			}
			else {                     //Else there are field values.
				ValueCounter++;
				FieldArray.push_back(str);
			}
		}
		if (LineCounter > 1) {         //Start to print the output.
			int FieldCounter = 0;
			while (FieldCounter < FieldNumber) {
				if (FieldCounter > 0) {
					jsonOutput << JSON_separator;
				}
				//change the jsonOutput format if we want to change the output.json format.
				jsonOutput << "\"" << FieldNameArray[FieldCounter] << "\":\"";
				if (ValueCounter > FieldCounter) {
				jsonOutput << FieldArray[FieldCounter];
				}
				jsonOutput<< "\"";
				FieldCounter++;			
			}
		}
	}
	if(LineCounter>=2)jsonOutput << "}\n";
	jsonOutput << "]\n";				//print JSON_suffix, now it is  Reft Square Bracket
	csvInput.close();
	jsonOutput.close();
	return 0;
}