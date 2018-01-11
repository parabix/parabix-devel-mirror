#include <unicode/regex.h>
#include <iostream>
#include <fstream>
#include <cstring>

using namespace std;

bool hasFlag(int argc, char *argv[], string flag){
	for (int i = 0;i < argc; i++){
		string e = argv[i];
		if (flag == e){
			return true;
		}
	}
	return false;
}

int main(int argc, char *argv[]) {
	string re;
	UnicodeString str2match;
	string fileName;
	uint32_t caseInsensitive = hasFlag(argc, argv, "-i")? 2 : 0;
	bool countOnly = hasFlag(argc, argv, "-c");
	bool invertedMatch = hasFlag(argc, argv, "-v");
	bool lineMatch = hasFlag(argc, argv, "-x");
	bool wordMatch = hasFlag(argc, argv, "-w");

	re = argv[argc-2];
	fileName = argv[argc-1];
	
	if (lineMatch){
		re = "^" + re + "$";
	}
	if (wordMatch){
		re = "\\b" + re + "\\b";
	}
	
	UErrorCode status = U_ZERO_ERROR;
	UnicodeString uRE;
	uRE.setTo(re.c_str());
	RegexMatcher *matcher = new RegexMatcher(uRE, caseInsensitive, status);
	if (U_FAILURE(status)) {
        cerr << u_errorName(status) << endl;
	    return 0;
	}

	ifstream inputFile;
	inputFile.open(fileName);
	if (!inputFile.is_open()){
		cerr << "input file not found\n";
	    return 0;
	}
	string line;
	int count = 0;
	while (getline(inputFile, line)){
		UnicodeString str2match;
		str2match.setTo(line.c_str());
		matcher->reset(str2match);
		if (matcher->find()) {
			if (!invertedMatch){
				if (countOnly){
					count ++;
				}
				else{
					std::string ustr;
					str2match.toUTF8String(ustr);
					cout << ustr << endl;
				}
			}
		}
		else {
			if (invertedMatch){
				if (countOnly){
					count ++;
				}
				else{
					std::string ustr;
					str2match.toUTF8String(ustr);
					cout << ustr << endl;
				}
			}
		}
	}
	if (countOnly){
		cout << count << endl;
	}
	inputFile.close();
	return 0;
}
