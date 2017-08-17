#include <unicode/regex.h>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char *argv[]) {
	string re;
	UnicodeString str2match;
	string fileName;
	uint32_t caseInsensitive = 0;
	if (argc == 3 || argc == 4){
		re = argv[argc-2];
		fileName = argv[argc-1];
		if (argc == 4 && strcmp(argv[1], "-i") == 0) {
			caseInsensitive = 2;
		}
	}
	else {
		cerr << "Usage: ugrep [regex] [-i] [file name]\n";
		return 0;
	}
	
	UErrorCode status = U_ZERO_ERROR;
	UnicodeString uRE;
	uRE.setTo(re.c_str());
	RegexMatcher *matcher = new RegexMatcher(uRE, caseInsensitive, status);
	if (U_FAILURE(status)) {
	    cerr << "syntax error for ugrep\n";
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
			count++;
		}
	}

	
	
	cout << count << endl;
	return 0;
}