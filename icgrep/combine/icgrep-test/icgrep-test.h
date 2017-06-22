#ifndef ICGREPTEST_H
#define ICGREPTEST_H
#include <vector>
#include <string>
#include <re/re_parser.h>


class IcgrepTest{
public:

	void prepare();
	void buildTest(std::string re, std::vector<std::string> flags, re::RE_Syntax syntax , int testNum);
	void execute();
	void getResult();
	IcgrepTest();	

private:
	void resetBash(std::string fileName);
	void clearDir(std::string dir);
	void writeToBash(std::string fileName, std::string value);
	void writetoFile(std::string content, std::string dir, int fileNo);
	bool hasFlag(std::string flag, std::vector<std::string> flags);
	std::vector<std::string> removeFlag(std::string flag, std::vector<std::string> flags);
};


#endif