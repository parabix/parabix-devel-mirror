#include "icgrep-test.h"
#include "../stringGen.h"
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/mapped_file.hpp>
#include <boost/algorithm/string/replace.hpp>


using namespace std;
namespace io = boost::iostreams;
namespace fs=boost::filesystem;

IcgrepTest::IcgrepTest(){}

std::size_t directorySize(fs::path path)
{
	return std::count_if(fs::directory_iterator(path),
                         fs::directory_iterator(), 
                         [](const fs::directory_entry& e) { 
                          return true;  });
}

void IcgrepTest::backup(){
	int count = directorySize("../icgrep/combine/Archive");

	fs::path src("../icgrep/combine/icgrep-test");
	fs::path dst("../icgrep/combine/Archive/test" + to_string(count+1));
	if (!fs::create_directory(dst))
    {
        throw std::runtime_error("Cannot create destination directory " + dst.string());
    }

	for (const auto& dirEnt : fs::recursive_directory_iterator{src})
    {
        const auto& path = dirEnt.path();
        auto relativePathStr = path.string();
        boost::replace_first(relativePathStr, src.string(), "");
        fs::copy(path, dst / relativePathStr);
    }
}
void IcgrepTest::resetBash(string fileName){
	ofstream file;
	file.open(fileName);
	file << "#!/bin/bash\n\n";
	file << "echo \"Starting bash file: " << fileName << "\"\n";
	file.close();
	//give permission.
	string cmd = "chmod +x " + fileName;
	system(cmd.c_str());
}

void IcgrepTest::clearDir(string dir){
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
		cerr << "Could not open input file\n";
	}
	file << content << endl;
	file.close();
}

void IcgrepTest::prepare(){
	backup();
	clearDir("../icgrep/combine/icgrep-test/regs");
	clearDir("../icgrep/combine/icgrep-test/files");
	resetBash("../icgrep/combine/icgrep-test/grep/icgrep-bash.sh");
	resetBash("../icgrep/combine/icgrep-test/grep/grep-bash.sh");
	resetBash("../icgrep/combine/icgrep-test/ugrep/icgrep-bash.sh");
	resetBash("../icgrep/combine/icgrep-test/ugrep/ugrep-bash.sh");
	remove("..//icgrep/combine/icgrep-test/grep/icgrep-result");
	remove("..//icgrep/combine/icgrep-test/grep/grep-result");
	remove("..//icgrep/combine/icgrep-test/ugrep/icgrep-result");
	remove("..//icgrep/combine/icgrep-test/ugrep/ugrep-result");
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
		if (strncmp(f.c_str(), flag.c_str(), flag.size()) == 0){
			flags.erase(flags.begin()+c);
		}
		c++;
	}
	return flags;
}

void IcgrepTest::buildTest(string re, vector<string> flags, re::RE_Syntax syntax, int testNum){
	cout << "test: " << testNum << endl;
	cout << "RE: \'" << re << "\'" << endl;

	string reDir = "../icgrep/combine/icgrep-test/regs/reg";
	string strDir = "../icgrep/combine/icgrep-test/files/file";

	StringGenerator strGen;
	writetoFile(strGen.generate(re, flags, syntax) + "\n", strDir, testNum);
	//remove icgrep specific flags for comparison.
	vector<string> gflags = removeFlag("-t", flags);
	gflags = removeFlag("-BlockSize", gflags);
	gflags = removeFlag("-P", gflags);
	gflags = removeFlag("-f", gflags);
	gflags = removeFlag("-e", gflags);

	if (syntax == re::RE_Syntax::ERE){
		string icgrepScript = "../icgrep/combine/icgrep-test/grep/icgrep-bash.sh";
		string grepScript = "../icgrep/combine/icgrep-test/grep/grep-bash.sh";
		string icgrepResult = "../icgrep/combine/icgrep-test/grep/icgrep-result";
		string grepResult = "../icgrep/combine/icgrep-test/grep/grep-result";

		writeToBash(icgrepScript, "echo \"" + re + "\" | tee -a " + icgrepResult);
		writeToBash(grepScript, "echo \"" + re + "\" | tee -a " + grepResult);

		if (hasFlag("-f", flags)){
			flags = removeFlag("-f", flags);
	 		flags = removeFlag("-e", flags);
	 		
	 		writetoFile(re, reDir, testNum);
	 	
 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " -f " + reDir + to_string(testNum)+
 				 " " + strDir + to_string(testNum) + " >> " + icgrepResult + "\n");	
	 	}
	 	else if (hasFlag("-e", flags)){
	 		flags = removeFlag("-f", flags);
	 		flags = removeFlag("-e", flags);

 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " -e \'" + re + "\' " +
 				" " + strDir + to_string(testNum) + " >> " + icgrepResult + "\n");
	 	}
	 	else{
 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " \'" + re + "\' " +
 				" " + strDir + to_string(testNum) + " >> " + icgrepResult + "\n");
	 	}

	 	writeToBash(grepScript, 
 			"grep " + strGen.stringifyVec(gflags, " ") + " \'" + re + "\' " +
 			" " + strDir + to_string(testNum) + " >> " + grepResult + "\n");
	}
	else if (syntax == re::RE_Syntax::BRE){

		string icgrepScript = "../icgrep/combine/icgrep-test/grep/icgrep-bash.sh";
		string grepScript = "../icgrep/combine/icgrep-test/grep/grep-bash.sh";
		string icgrepResult = "../icgrep/combine/icgrep-test/grep/icgrep-result";
		string grepResult = "../icgrep/combine/icgrep-test/grep/grep-result";

		writeToBash(icgrepScript, "echo \"" + re + "\" | tee -a " + icgrepResult);
		writeToBash(grepScript, "echo \"" + re + "\" | tee -a " + grepResult);

	 	if (hasFlag("-f", flags)){
	 		flags = removeFlag("-f", flags);
	 		flags = removeFlag("-e", flags);

	 		writetoFile(re, reDir, testNum);
 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " -f " + reDir + to_string(testNum) + 
 				" " + strDir + to_string(testNum) + " >> " + icgrepResult + "\n");

	 	}
	 	else if (hasFlag("-e", flags)){
	 		flags = removeFlag("-f", flags);
	 		flags = removeFlag("-e", flags);
	 		
 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " -e \'" + re + "\' " +
 				strDir + to_string(testNum) + " >> " + icgrepResult + "\n");
	 	}
	 	else{
 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " \'" + re + "\' " +
 				strDir + to_string(testNum) + " >> " + icgrepResult + "\n");
	 	}

	 	writeToBash(grepScript, 
 			"grep " + strGen.stringifyVec(gflags, " ") + " \'" + re + "\' " +
 			strDir + to_string(testNum) + " >> " + icgrepResult + "\n");
	}
	else {
		gflags = removeFlag("-c", gflags);

		string icgrepScript = "../icgrep/combine/icgrep-test/ugrep/icgrep-bash.sh";
		string ugrepScript = "../icgrep/combine/icgrep-test/ugrep/ugrep-bash.sh";
		string icgrepResult = "../icgrep/combine/icgrep-test/ugrep/icgrep-result";
		string ugrepResult = "../icgrep/combine/icgrep-test/ugrep/ugrep-result";

		writeToBash(icgrepScript, "echo \"" + re + "\" | tee -a " + icgrepResult);
		writeToBash(ugrepScript, "echo \"" + re + "\" | tee -a " + ugrepResult);

	 	if (hasFlag("-f", flags)){
	 		flags = removeFlag("-f", flags);
	 		flags = removeFlag("-e", flags);
	 
	 		writetoFile(re, reDir, testNum);
 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " -f " + reDir + to_string(testNum) + 
 				" " + strDir + to_string(testNum) + " >> " + icgrepResult + "\n");
	 	}
	 	else if (hasFlag("-e", flags)){
	 		flags = removeFlag("-f", flags);
	 		flags = removeFlag("-e", flags);

 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " -e \'" + re + "\' " +
 				strDir + to_string(testNum) + " >> " + icgrepResult + "\n");
	 	}
	 	else{
 			writeToBash(icgrepScript, 
 				"./icgrep " + strGen.stringifyVec(flags, " ") + " \'" + re + "\' " +
 				strDir + to_string(testNum) + " >> " + icgrepResult + "\n");

	 	}

	 	writeToBash(ugrepScript, 
 			"./ugrep " + strGen.stringifyVec(gflags, " ") + " \'" + re + "\' " +
 			strDir + to_string(testNum) + " >> " + ugrepResult + "\n");

	}
}

void IcgrepTest::execute(){
	system("../icgrep/combine/icgrep-test/grep/icgrep-bash.sh");
	system("../icgrep/combine/icgrep-test/grep/grep-bash.sh");
	system("../icgrep/combine/icgrep-test/ugrep/icgrep-bash.sh");
	system("../icgrep/combine/icgrep-test/ugrep/ugrep-bash.sh");
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

	compareResults("../icgrep/combine/icgrep-test/grep/icgrep-result", "../icgrep/combine/icgrep-test/grep/grep-result", "GNU grep");
    compareResults("../icgrep/combine/icgrep-test/ugrep/icgrep-result", "../icgrep/combine/icgrep-test/ugrep/ugrep-result", "ICU grep");

    cout << "End of test\n";
}

