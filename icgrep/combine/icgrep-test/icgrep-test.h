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
};


#endif