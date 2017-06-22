#include "icgrep-test.h"
#include "../stringGen.h"
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>


using namespace std;
namespace io = boost::iostreams;

IcgrepTest::IcgrepTest(){}

void IcgrepTest::resetBash(string fileName){
	ofstream file;
	file.open(fileName);
	file << "#!/bin/bash\n\n";
	file << "echo \"Starting bash file: " << fileName << "\"\n";
	file.close();
}

void IcgrepTest::clearDir(string dir){
	namespace fs=boost::filesystem;
  	fs::path path_to_remove(dir);
  	for (fs::directory_iterator end_dir_it, it(path_to_remove); it!=end_dir_it; ++it) {
   		fs::remove_all(it->path());
  	}
}
void IcgrepTest::writeToBash(string fileName, string value){
	ofstream file;
	file.open(fileName, ios::app);
	file << value << endl;
	file.close();
}
void IcgrepTest::writetoFile(string content, string dir, int fileNo){
	string fileName = dir + to_string(fileNo);
	ofstream file;
	file.open(fileName);
	if (!file.is_open()){
		cout << "Could not open input file\n";
	}
	file << content << endl;
	file.close();
}

void IcgrepTest::prepare(){
	clearDir("../icgrep/combine/regs");
	clearDir("../icgrep/combine/files");
	resetBash("../icgrep/combine/icgrep-test/grep/icgrep-bash.sh");
	resetBash("../icgrep/combine/icgrep-test/grep/grep-bash.sh");
	resetBash("../icgrep/combine/icgrep-test/pcre/icgrep-bash.sh");
	resetBash("../icgrep/combine/icgrep-test/pcre/pcre-bash.sh");
	remove("..//icgrep/combine/icgrep-test/grep/icgrep-result");
	remove("..//icgrep/combine/icgrep-test/grep/grep-result");
	remove("..//icgrep/combine/icgrep-test/pcre/icgrep-result");
	remove("..//icgrep/combine/icgrep-test/pcre/pcre-result");
}

bool IcgrepTest::hasFlag(string flag, std::vector<string> flags){
	return (std::find(flags.begin(), flags.end(), flag) != flags.end()) ? true : false;
}

std::vector<string> IcgrepTest::removeFlag(string flag, std::vector<string> flags){
	flags.erase(std::remove(flags.begin(), flags.end(), flag), flags.end());
	return flags;
}

void IcgrepTest::buildTest(string re, vector<string> flags, re::RE_Syntax syntax, int testNum){
	cout << "test: " << testNum << endl;
	cout << "RE: \'" << re << "\'" << endl;

	string reDir = "../icgrep/combine/regs/reg";
	string strDir = "../icgrep/combine/files/file";

	StringGenerator strGen;
	writetoFile(strGen.generate(re, flags, syntax) + "\n", strDir, testNum);

	if (syntax == re::RE_Syntax::ERE){
		string icgrepScript = "../icgrep/combine/icgrep-test/grep/icgrep-bash.sh";
		string grepScript = "../icgrep/combine/icgrep-test/grep/grep-bash.sh";
		string icgrepResult = "../icgrep/combine/icgrep-test/grep/icgrep-result";
		string grepResult = "../icgrep/combine/icgrep-test/grep/grep-result";

		writeToBash(icgrepScript, "echo \"" + re + "\" >> " + icgrepResult);
		writeToBash(grepScript, "echo \"" + re + "\" >> " + grepResult);

		if (hasFlag("-f", flags)){
			flags = removeFlag("-f", flags);
	 		flags = removeFlag("-e", flags);
	 		writetoFile(re, reDir, testNum);
	 	
 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " -f " + reDir + to_string(testNum)+
 				 " " + strDir + to_string(testNum) + " >> " + icgrepResult + "\n");

 			writeToBash(grepScript, 
 				"grep " + strGen.stringifyVec(flags, " ") + " -f " + reDir + to_string(testNum) + 
 				" " + strDir + to_string(testNum) + " >> " + grepResult + "\n");
	 		
	 	}
	 	else if (hasFlag("-e", flags)){
	 		flags = removeFlag("-f", flags);
	 		flags = removeFlag("-e", flags);
 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " -e \'" + re + "\' " +
 				" " + strDir + to_string(testNum) + " >> " + icgrepResult + "\n");

 			writeToBash(grepScript, 
 				"grep " + strGen.stringifyVec(flags, " ") + " -e \'" + re + "\' " +
 				" " + strDir + to_string(testNum) + " >> " + grepResult + "\n");
	 	}
	 	else{
	 		
 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " \'" + re + "\' " +
 				" " + strDir + to_string(testNum) + " >> " + icgrepResult + "\n");

 			writeToBash(grepScript, 
 				"grep " + strGen.stringifyVec(flags, " ") + " \'" + re + "\' " +
 				" " + strDir + to_string(testNum) + " >> " + grepResult + "\n");

	 	}
	}
	else if (syntax == re::RE_Syntax::BRE){

		string icgrepScript = "../icgrep/combine/icgrep-test/grep/icgrep-bash.sh";
		string grepScript = "../icgrep/combine/icgrep-test/grep/grep-bash.sh";
		string icgrepResult = "../icgrep/combine/icgrep-test/grep/icgrep-result";
		string grepResult = "../icgrep/combine/icgrep-test/grep/grep-result";
		vector<string> gflags;

		writeToBash(icgrepScript, "echo \"" + re + "\" >> " + icgrepResult);
		writeToBash(grepScript, "echo \"" + re + "\" >> " + grepResult);

	 	if (hasFlag("-f", flags)){
	 		flags = removeFlag("-f", flags);
	 		flags = removeFlag("-e", flags);
	 		gflags = removeFlag("-G", flags);
	 		writetoFile(re, reDir, testNum);
	 	
 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " -f " + reDir + to_string(testNum) + 
 				" " + strDir + to_string(testNum) + " >> " + icgrepResult + "\n");


 			writeToBash(grepScript, 
 				"grep " + strGen.stringifyVec(gflags, " ") + " -f " + reDir + to_string(testNum) + 
 				" " + strDir + to_string(testNum) + " >> " + grepResult + "\n");

	 		
	 	}
	 	else if (hasFlag("-e", flags)){
	 		flags = removeFlag("-f", flags);
	 		flags = removeFlag("-e", flags);
	 		gflags = removeFlag("-G", flags);
 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " -e \'" + re + "\' " +
 				strDir + to_string(testNum) + " >> " + icgrepResult + "\n");

 			writeToBash(grepScript, 
 				"grep " + strGen.stringifyVec(gflags, " ") + " -e \'" + re + "\' " +
 				strDir + to_string(testNum) + " >> " + grepResult + "\n");

	 	}
	 	else{
	 		gflags = removeFlag("-G", flags);
 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " \'" + re + "\' " +
 				strDir + to_string(testNum) + " >> " + icgrepResult + "\n");

 			writeToBash(grepScript, 
 				"grep " + strGen.stringifyVec(gflags, " ") + " \'" + re + "\' " +
 				strDir + to_string(testNum) + " >> " + icgrepResult + "\n");

	 	}
	}
	else {

		string icgrepScript = "../icgrep/combine/icgrep-test/pcre/icgrep-bash.sh";
		string pcreScript = "../icgrep/combine/icgrep-test/pcre/pcre-bash.sh";
		string icgrepResult = "../icgrep/combine/icgrep-test/pcre/icgrep-result";
		string pcreResult = "../icgrep/combine/icgrep-test/pcre/pcre-result";
		vector<string> pflags;

		writeToBash(icgrepScript, "echo \"" + re + "\" >> " + icgrepResult);
		writeToBash(pcreScript, "echo \"" + re + "\" >> " + pcreResult);

	 	if (hasFlag("-f", flags)){
	 		flags = removeFlag("-f", flags);
	 		flags = removeFlag("-e", flags);
	 		pflags = removeFlag("-P", flags);
	 		writetoFile(re, reDir, testNum);
	 	
 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " -f " + reDir + to_string(testNum) + 
 				" " + strDir + to_string(testNum) + " >> " + icgrepResult + "\n");


 			writeToBash(pcreScript, 
 				"pcregrep " + strGen.stringifyVec(pflags, " ") + " -f " + reDir + to_string(testNum) + 
 				" " + strDir + to_string(testNum) + " >> " + pcreResult + "\n");
	 		
	 	}
	 	else if (hasFlag("-e", flags)){
	 		flags = removeFlag("-f", flags);
	 		flags = removeFlag("-e", flags);
	 		pflags = removeFlag("-P", flags);
 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " -e \'" + re + "\' " +
 				strDir + to_string(testNum) + " >> " + icgrepResult + "\n");

 			writeToBash(pcreScript, 
 				"pcregrep " + strGen.stringifyVec(pflags, " ") + " -e \'" + re + "\' " +
 				strDir + to_string(testNum) + " >> " + pcreResult + "\n");
 			
	 	}
	 	else{
	 		pflags = removeFlag("-P", flags);
 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " \'" + re + "\' " +
 				strDir + to_string(testNum) + " >> " + icgrepResult + "\n");

 			writeToBash(pcreScript, 
 				"pcregrep " + strGen.stringifyVec(pflags, " ") + " \'" + re + "\' " +
 				strDir + to_string(testNum) + " >> " + pcreResult + "\n");

	 	}
	}
}

void IcgrepTest::execute(){
	system("../icgrep/combine/icgrep-test/grep/icgrep-bash.sh");
	system("../icgrep/combine/icgrep-test/grep/grep-bash.sh");
	system("../icgrep/combine/icgrep-test/pcre/icgrep-bash.sh");
	system("../icgrep/combine/icgrep-test/pcre/pcre-bash.sh");
}

void compareResults(string dir1, string dir2, string testName){
	io::mapped_file_source f1(dir1);
    io::mapped_file_source f2(dir2);

    if(    f1.size() == f2.size()
        && std::equal(f1.data(), f1.data() + f1.size(), f2.data())
       )
        std::cout << testName << "test suceeded!\n";
    else
        std::cout << testName << "test FAILED!\n";
}

void IcgrepTest::getResult(){

	compareResults("..//icgrep/combine/icgrep-test/grep/icgrep-result", "..//icgrep/combine/icgrep-test/grep/grep-result", "GNU grep");
    compareResults("..//icgrep/combine/icgrep-test/pcre/icgrep-result", "..//icgrep/combine/icgrep-test/pcre/pcre-result", "PCRE grep");

    cout << "Done\n";
}

