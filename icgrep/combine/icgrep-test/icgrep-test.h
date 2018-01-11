#ifndef ICGREPTEST_H
#define ICGREPTEST_H
#include <vector>
#include <string>
#include <re/re_parser.h>
#include "../regexGen.h"


class IcgrepTest{
public:

	unsigned buildTest(std::string re, std::vector<std::string> flags,
										 re::RE_Syntax syntax , RegexGen::FileType fTy,
										 int testNum);


private:


	std::string UniversalizePropertyName(std::string re);
	void clearTest();
	void writeToFile(std::string content, std::string dir);
	void generateStringFile(string re, std::vector<string> flags, re::RE_Syntax syntax,
									RegexGen::FileType fTy, string dir);
	void copyFile(std::string src, std::string dst);
	std::string exec(std::string cmd);
	bool identicalFiles(const std::string& filename1, const std::string& filename2);
	void reportBug(std::vector<std::string> &icgrepArgs, std::vector<std::string> &grepArgs, int testNum);
	bool hasFlag(std::string flag, std::vector<std::string> flags);
	std::vector<std::string> removeFlag(std::string flag, std::vector<std::string> flags);
};


#endif
