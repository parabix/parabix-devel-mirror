#ifndef REGEXGEN_H
#define REGEXGEN_H
 
#include <vector>
#include <string>
#include <re/re_parser.h>

using namespace std;

class RegexGen {


public:

	string 				RE;
	vector<string>		flags;
	re::RE_Syntax 		syntax;

	RegexGen(std::vector<string> header, std::vector<string> row);
	RegexGen(re::RE_Syntax syntax) : syntax(syntax){}

	std::vector<string> parseCC(std::vector<string> header, std::vector<string> row);
	

private:
	
	bool usesCC(std::string op);
	re::RE_Syntax setSyntax(std::vector<string> header, std::vector<string> row);
	string parseRE(std::vector<string> header, std::vector<string> row);
	std::vector<string> parseFlags(std::vector<string> header, std::vector<string> row);
	string stringifyLine(vector<string> elements, string separator="");

	string getBoundary();
	string getNotBoundary();
	string getWord();
	string getNotWord();
	string getWhitespace();
	string getNotWhitespace();
	string getTab();
	string getDigit();
	string getNotDigit();
	string getAny();
	string getPosix(string value);
	string getUnicode();
	string getList();
	string getNList();
	string getRange();
	string getPropertyValue();
	std::string getCharacterName();
	string getProperty();
	string getNotProperty();
	string getName();
	string getZeroOrOne(string cc);
	string getZeroOrMore(string cc);
	string getOneOrMore(string cc);
	string getRep(string cc, int rep);
	string getRep(string cc, int lb, int ub);
	string getRepMore(string cc, int rep);
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