//This csv2json file_zero can pass the examples with no separators and backflash in them. And some field values can be empty but the field names.

#include<iostream>
#include<fstream>
#include<sstream>
#include<string>
#include<vector>
using namespace std;

int main() {
	//change ipath(input file path) and opath(output file path) to where your .csv and .json file are. 
	//And please notice that you should use double backslashes to specify a separator in file paths.   
	const char* ipath = "D:\\ZJU\\sophomore\\Spring_Summer\\SoftwareEngneering\\project\\codes\\example\\eg0.csv";
	const char* opath = "D:\\ZJU\\sophomore\\Spring_Summer\\SoftwareEngneering\\project\\codes\\example\\eg0.json";
	const char separater = ',';
	ifstream csvInput;
	ofstream jsonOutput;
	csvInput.open(ipath, ios::in);
	jsonOutput.open(opath, ios::out | ios::trunc);


	string Oneline;
	string str;
	vector<string> fieldnameArray;
	vector<string>::iterator it1;
	int LineCounter = 0;
	int FieldNumber = 0;
	jsonOutput << "[\n";

	while (getline(csvInput, Oneline))
	{
		vector<string> fieldArray;
		stringstream ss(Oneline);
		LineCounter++;

		if (LineCounter >= 3) {
			jsonOutput << "},\n";
		}
		if (LineCounter >= 2) {
			jsonOutput << "{";
		}
		while (getline(ss, str, separater)) {
			if (LineCounter == 1) {
				FieldNumber++;
				fieldnameArray.push_back(str);
			}
			else {
				fieldArray.push_back(str);
			}
		}
		if (LineCounter > 1) {
			int FieldCounter = 0;
			while (FieldCounter < FieldNumber) {
				if (FieldCounter > 0) {
					jsonOutput << separater;
				}
//change the jsonOutput format if we want to change the .json format.
				jsonOutput << "\"" << fieldnameArray[FieldCounter] << "\":\"" << fieldArray[FieldCounter] << "\"";
				FieldCounter++;			
			}

		}
	}
	jsonOutput << "}\n" << "]\n";
	csvInput.close();
	jsonOutput.close();
	return 0;
}