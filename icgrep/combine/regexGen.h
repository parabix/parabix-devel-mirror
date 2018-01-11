#ifndef REGEXGEN_H
#define REGEXGEN_H

#include <vector>
#include <string>
#include <re/re_parser.h>

using namespace std;



class RegexGen {


public:

	enum class FileType{
		SMALL,
		MEDIUM,
		LARGE
	};

	string 				RE;
	vector<string>		flags;
	re::RE_Syntax 		syntax;
	FileType 			fileType;

	RegexGen(std::vector<string> mParameters, std::vector<string> mValues);

	// std::vector<string> parseCC();


private:

	std::vector<string> mParameters;
	std::vector<string> mValues;


	bool usesCC(std::string op);
	re::RE_Syntax getSyntax();
	string parseRE();
	std::vector<string> parseFlags();
	FileType getFileTy();
	string stringifyLine(vector<string> elements, string separator="");

	string getBoundary();
	string getNotBoundary();
	string getWordBegin();
	string getWordEnd();
	string getList(std::string cc1, std::string cc2, std::string cc3);
	string getNList(std::string cc1, std::string cc2, std::string cc3);
	string getZeroOrOne(string cc);
	string getZeroOrMore(string cc);
	string getOneOrMore(string cc);
	string getRep(string cc, string size);
	string getRepNM(string cc, string size);
	string getRepMore(string cc, string size);
	string getJoin(string LS, string RS);
	string getBackRef(string cc);

	string getAssertionCoating(string cc);
	string getNegativeAssertionCoating(string cc);

	string getLookAhead(string cc);
	string getNegativeLookAhead(string cc);
	string getLookBehind(string cc);
	string getNegativeLookBehind(string cc);


};


#endif
