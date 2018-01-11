#include <string>
#include <vector>
#include <fstream>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <algorithm>
#include <chrono>
#include "regexGen.h"
#include "stringGen.h"
#include "icgrep-test/icgrep-test.h"

#include <getopt.h>

#define likely(x)     __builtin_expect((x),1)

using namespace std;
namespace fs=boost::filesystem;


std::vector<string> vectorizeLine(string line){
	std::vector<string> elements;
	boost::split(elements, line, boost::is_any_of(","), boost::token_compress_on);
	return elements;
}

std::size_t directorySize(fs::path path)
{
	return std::count_if(fs::directory_iterator(path),
                         fs::directory_iterator(),
                         [](const fs::directory_entry& e) {
                          return true;  });
}

void clearDir(string dir){
  	fs::path path_to_remove(dir);
  	for (fs::directory_iterator end_dir_it, it(path_to_remove); it!=end_dir_it; ++it) {
   		fs::remove_all(it->path());
  	}
}

void backup(){
	int count = directorySize("core-out/Archive");

	fs::path src("core-out/bugs");
	fs::path dst("core-out/Archive/test" + to_string(count+1));
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

    clearDir("core-out/bugs/regs");
    clearDir("core-out/bugs/files");
    remove("core-out/bugs/bugList");
}

void printElapsedTime(chrono::system_clock::time_point startTime){
		chrono::duration<int> timeSpan =
		chrono::duration_cast<chrono::duration<int>>(chrono::system_clock::now() - startTime);
		auto elapsed = chrono::duration_cast<chrono::minutes>(timeSpan).count();
		cout << "Time elapsed: " << elapsed << endl;

}

int main (int argc, char *argv[]) {
	srand (time(0));

	string csvInput = "../icgrep/combine/icGrep-output.csv";

	bool exhaustive = false;

	int opt;

	while ((opt = getopt(argc, argv, "eI:")) != -1) {
	   	switch (opt) {
	   	case 'e':
	   		exhaustive = true;
	       	break;
	   	case 'I':
	    	csvInput = optarg;
	      	break;
	   	default: /* '?' */
	       	cerr << "Usage: " << argv[0] << " [-e] [-I csv file]\n";
	       	exit(EXIT_FAILURE);
	   	}
	}


	cout << "CSV: " << csvInput << endl;
	ifstream file;
	file.open(csvInput);
	if (!file.is_open()){
		cerr << "Could not open input file: " << csvInput << endl;
		return 0;
	}

	string line;
	vector<string> row;
	vector<string> header;
	int rownum = 1;
	bool firstLine = true;
	int bugCount = 0;

	backup();

	chrono::system_clock::time_point startTime =
		chrono::system_clock::now();

	if(exhaustive){

		cout << "exhaustive mode selected..." << endl;

		chrono::system_clock::time_point endTime =
    	chrono::system_clock::now() + chrono::hours(24);

    	while (endTime > chrono::system_clock::now()) {

    		if(!getline(file, line)){
				file.clear();
				file.seekg(0, ios::beg);
				getline(file, line);
				firstLine = true;
    		}
    		if (likely(!line.empty() && line[0] != '#')){
    			if (firstLine){
    				header = vectorizeLine(line);
    				firstLine = false;
    			}
    			else{
						row = vectorizeLine(line);
						RegexGen reGen(header, row);
						IcgrepTest icTest;
						bugCount += icTest.buildTest(reGen.RE, reGen.flags, reGen.syntax, reGen.fileType, rownum);
						if (rownum % 50 == 0){
							printElapsedTime(startTime);

						}
						++rownum;
					}
				}
			}

	}
	else{
		cout << "combinatorial mode selected..." << endl;
		while(getline(file, line)){
			if (likely(!line.empty() && line[0] != '#')){
				if (firstLine){
    				header = vectorizeLine(line);
    				firstLine = false;
  			}
  			else{
					row = vectorizeLine(line);
					RegexGen reGen(header, row);
					IcgrepTest icTest;
					bugCount += icTest.buildTest(reGen.RE, reGen.flags, reGen.syntax, reGen.fileType, rownum);
					if (rownum % 50 == 0){
						printElapsedTime(startTime);
					}
					++rownum;
				}
			}
		}
	}
	// ictest.getResult(rownum -1);
	file.close();

	return 0;
}
