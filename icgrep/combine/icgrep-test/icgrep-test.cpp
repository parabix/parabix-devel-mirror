#include "icgrep-test.h"
#include "../stringGen.h"
#include "process.h"
#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <stdexcept>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>


using namespace std;


void IcgrepTest::writeToFile(string content, string dir){
	ofstream file;
	file.open(dir, std::ofstream::out | std::ofstream::app);
	if (!file.is_open()){
		cerr << "Could not open input file: " << dir << endl;
		return;
	}
	file << content << endl;
	file.close();
}

void IcgrepTest::generateStringFile(string re, std::vector<string> flags, re::RE_Syntax syntax,
									RegexGen::FileType fTy, string dir){


	int lineNum = 0;
	if (fTy == RegexGen::FileType::SMALL){
		lineNum = 1;
	}
	else if (fTy == RegexGen::FileType::MEDIUM){
		lineNum = 10;
	}
	else {
		lineNum = 100;
	}
	for (int i = 0; i < lineNum; i++){
		StringGenerator strGen(re, flags, syntax);
		writeToFile(strGen.generate() + "\n", dir);
	}

}

void IcgrepTest::copyFile(string src, string dst){

	std::ifstream sFile (src.c_str());
    std::ofstream dFile(dst.c_str());
    dFile << sFile.rdbuf();
    dFile.close();
    sFile.close();
}

string IcgrepTest::UniversalizePropertyName(string re){
	std::string::size_type pos = re.find("\\N{^");
	while (pos != std::string::npos){
		re.erase(pos+3, 1);
		pos = re.find("$}");
		if (pos != std::string::npos){
			re.erase(pos,1);
		}
		pos = re.find("\\N{^");
	}
	return re;
}

bool IcgrepTest::hasFlag(string flag, std::vector<string> flags){
	for (auto f : flags){
		if (strncmp(f.c_str(), flag.c_str(), flag.size()) == 0){
			return true;
		}
	}
	return false;
}

std::vector<string> IcgrepTest::removeFlag(string flag, std::vector<string> flags){
	int c = 0;
	for (auto f : flags){
		if (f.find(flag) == 0){
			flags.erase(flags.begin()+c);
			return flags;
		}
		c++;
	}
	return flags;
}

template<typename InputIterator1, typename InputIterator2>
bool
range_equal(InputIterator1 first1, InputIterator1 last1,
        InputIterator2 first2, InputIterator2 last2)
{
    while(first1 != last1 && first2 != last2)
    {
        if(*first1 != *first2) return false;
        ++first1;
        ++first2;
    }
    return (first1 == last1) && (first2 == last2);
}

bool IcgrepTest::identicalFiles(const std::string& filename1, const std::string& filename2)
{
    std::ifstream file1(filename1);
    std::ifstream file2(filename2);

    std::istreambuf_iterator<char> begin1(file1);
    std::istreambuf_iterator<char> begin2(file2);

    std::istreambuf_iterator<char> end;
    bool identical = range_equal(begin1, end, begin2, end);

    file1.close();
    file2.close();

    return identical;
}

void IcgrepTest::reportBug(vector<string> &icgrepArgs, vector<string> &grepArgs, int testNum){
	cout << "\033[1;31mFAIL\033[0m\n";
	// ++mBugCount;
	string bugList = "core-out/bugs/bugList";
	string bugTestLine = "bug #" + to_string(testNum) + "= ";
	for (auto arg : icgrepArgs){
		bugTestLine += arg + " ";
	}
	bugTestLine += " === ";
	for (auto arg : grepArgs){
		bugTestLine += arg + " ";
	}
	bugTestLine += "\n\n";
	writeToFile(bugTestLine, bugList);
	string srcReDir = "core-out/reg";
	string srcStrDir = "core-out/file";
	string dstReDir = "core-out/bugs/regs/reg" + to_string(testNum);
	string dstStrDir = "core-out/bugs/files/file" + to_string(testNum);
	copyFile(srcReDir, dstReDir);
	copyFile(srcStrDir, dstStrDir);

}

void IcgrepTest::clearTest(){
	remove("core-out/reg");
	remove("core-out/file");
	remove("core-out/icgrep-out");
	remove("core-out/grep-out");
}

unsigned IcgrepTest::buildTest(string re, vector<string> flags, re::RE_Syntax syntax, RegexGen::FileType fTy, int testNum){

	clearTest();
	unsigned errorNo = 0;
	cout << "==========test: " << testNum << "==========" << endl;
	cout << "RE: \'" << re << "\'" << endl;

	string reDir = "core-out/reg";
	string strDir = "core-out/file";

	string icgrepOutputName = "core-out/icgrep-out";
	string grepOutputName = "core-out/grep-out";

	generateStringFile(re, flags, syntax, fTy, strDir);

	cout << "file generated" << endl;
	//remove icgrep specific flags and syntax for comparison.
	vector<string> gflags = removeFlag("-t", flags);
	gflags = removeFlag("-BlockSize", gflags);
	gflags = removeFlag("-P", gflags);
	gflags = removeFlag("-f", gflags);
	gflags = removeFlag("-e", gflags);

	vector<string> icgrepArgs = {"./icgrep"};
	vector<string> grepArgs;
	string baseGrep;
	bool REfromFile = hasFlag("-f", flags);
	bool multipleRE = hasFlag("-e", flags);
	flags = removeFlag("-f", flags);
 	flags = removeFlag("-e", flags);

 	for (auto f : flags){
 		icgrepArgs.push_back(f);
 	}
	if (REfromFile){
 		writeToFile(re, reDir);
 		icgrepArgs.push_back("-f");
 		icgrepArgs.push_back(reDir);
 	}
 	else if (multipleRE){
 		icgrepArgs.push_back("-e");
 		icgrepArgs.push_back(re);
 	}
 	else{
 		icgrepArgs.push_back(re);
	}
	icgrepArgs.push_back(strDir);

	if (syntax == re::RE_Syntax::ERE || syntax == re::RE_Syntax::BRE){
	 	grepArgs.push_back("grep");
	}
	else {
		grepArgs.push_back("./ugrep");
		re = UniversalizePropertyName(re);
	}
	for (auto f : gflags){
		grepArgs.push_back(f);
	}
	grepArgs.push_back(re);
	grepArgs.push_back(strDir);


	cout << "running " << grepArgs[0] << endl;
	int r2 = run_test(grepArgs, grepOutputName);

	cout << "running icgrep:\n";
	int r1 = run_test(icgrepArgs, icgrepOutputName);


	cout << "Differential..." << endl;

	if (!r1 && !r2){

		ifstream file;
		file.open(grepOutputName.c_str());
		if (!file.is_open()){
			cerr << "baseGrep file could not be open." << endl;
		}
		string line;
		if (getline(file, line)){
			cout << line << endl;
		}
		file.close();

		file.open(icgrepOutputName.c_str());
		if (!file.is_open()){
			cerr << "icgrep file could not be open." << endl;
		}
		if (getline(file, line)){
			cout << line << endl;
		}
		file.close();

		if (identicalFiles(icgrepOutputName, grepOutputName)){
			// reportBug(icgrepArgs, grepArgs, testNum);
			cout << "\033[1;32mPASS\033[0m\n";
		}
		else{
			// string line;
			file.open(grepOutputName.c_str());
			if (!file.is_open()){
				cerr << "ugrep file could not be open." << endl;
			}
			if (getline(file, line)){
				if (line.find("U_") ==0)
					cout << "\033[1;34mSKIPPED\033[0m\n";
				else{
					errorNo = 1;
					reportBug(icgrepArgs, grepArgs, testNum);
				}

			}
			else {
				errorNo = 1;
				reportBug(icgrepArgs, grepArgs, testNum);
			}
			file.close();
		}
	}
	else {
		errorNo = 1;
		reportBug(icgrepArgs, grepArgs, testNum);
	}

return errorNo;
}

// void IcgrepTest::getResult(int testNum){
// 	float bugRate = 100 * (float)(mBugCount) / testNum;
// 	cout << "Number of bugs found: " << to_string(mBugCount) << endl;
// 	cout << "Number of tests performed: " << to_string(testNum) << endl;
// 	cout << "Fail Rate: " << to_string(bugRate) << endl;
// }
