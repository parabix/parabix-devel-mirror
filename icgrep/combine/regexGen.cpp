#include "regexGen.h"
#include "stringGen.h"

#include <string>
#include <vector>
#include <fstream>
#include <boost/algorithm/string/split.hpp>
#include <iostream>
#include <algorithm>

#include <stdlib.h> 
#include <time.h>  
#include <stdio.h>
#include <re/re_diff.h>
#include <re/re_any.h>	
#include <re/re_parser.h>


using namespace std;

class CC {
	std::vector<string> ccList;
	std::vector<string> usedCC;
public:
	CC(std::vector<string> header, std::vector<string> row, re::RE_Syntax syntax){
		RegexGen reGen(syntax);
		ccList = reGen.parseCC(header, row);

	}

	std::string getCC(){

		int random;
		std::string cc;
		if (!ccList.empty()){
			random = rand() % ccList.size();
			cc = ccList[random];
			ccList.erase(ccList.begin()+random);
			usedCC.push_back(cc);
			return cc;
		}
		else {
			random = rand() % usedCC.size();
			cc = usedCC[random];
			return cc;
		}
	}
	std::string changeCC(std::string cc){
		std::string newCC;
		int random;
		if (!ccList.empty()){
			random = rand() % ccList.size();
			newCC = ccList[random];
			ccList.erase(ccList.begin()+random);
			usedCC.push_back(newCC);
		}
		else {
			random = rand() % usedCC.size();
			newCC = usedCC[random];
		}
		ccList.push_back(cc);
		return newCC;
	}
	std::vector<string> getRemainingCC(){
		return ccList;
	}
	bool isEmpty(){
		return ccList.empty();
	}
};

RegexGen::RegexGen(std::vector<string> header, std::vector<string> row){
	syntax = setSyntax(header, row);
	RE = parseRE(header, row);
	flags = parseFlags(header, row);
}

string RegexGen::stringifyLine(vector<string> elements, string separator){
	string line = "";
	bool sep = false;
	for (auto e : elements){
		line += sep? separator + e : e;
		sep = true;
	}
	return line;
}

string RegexGen::getBoundary(){
	return "\\b";
}
string RegexGen::getNotBoundary(){
	return "\\B";
}
string RegexGen::getWord(){
	if (syntax == re::RE_Syntax::BRE){
		return "w";
	}
	else return "\\w";
}
string RegexGen::getNotWord(){
	if (syntax == re::RE_Syntax::BRE){
		return "notWord";
	}
	else return "\\W";
}
string RegexGen::getWhitespace(){
	return "\\s";
}
string RegexGen::getNotWhitespace(){
	return "\\S";
}
string RegexGen::getTab(){
	if (syntax == re::RE_Syntax::BRE){
		return "t";
	}
	else return "\\t";
}
string RegexGen::getDigit(){
	if (syntax == re::RE_Syntax::BRE){
		return "d";
	}
	else return "\\d";
}
string RegexGen::getNotDigit(){
	if (syntax == re::RE_Syntax::BRE){
		return "D";
	}
	else return "\\D";
}
string RegexGen::getAny(){
	return ".";
}
string RegexGen::getPosix(string value){
	return "[[:" + value + ":]]";
}
std::string RegexGen::getUnicode(){
	if (syntax == re::RE_Syntax::ERE) {
		std::vector<string> ucd;
		ifstream file;
		file.open("../icgrep/combine/Unicode.txt");
		string line;
		while(getline(file, line)){
			ucd.push_back(line);
		}
		file.close();
		int random = rand() % ucd.size();
		return "\\u" + ucd[random];
	}
	else {
		return "u";
	}
}
std::string RegexGen::getList(){
	if (syntax == re::RE_Syntax::BRE) {
		return "l";
	}
	else {
		const char *l[] = {"[abc]","[XYZ]","[123]","[হ্যালো]"};
		std::vector<string> lists (l, l + sizeof(l) / sizeof(l[0]));
		int random = rand() % lists.size();
		return lists[random];
	}
	
}
std::string RegexGen::getNList(){
	if (syntax == re::RE_Syntax::BRE) {
		return "L";
	}
	else {
		std::vector<string> lists = {"[^abc]","[^XYZ]","[^123]","[^হ্যালো]"};
		int random = rand() % lists.size();
		return lists[random];
	}
}
std::string RegexGen::getRange(){
	if (syntax == re::RE_Syntax::BRE) {
		return "r";
	}
	else {
		std::vector<string> lists = {"[a-zA-Z0-9]","[A-Za-z]","[0-9]","[ا-ي]"};
		int random = rand() % lists.size();
		return lists[random];
	}
}
std::string RegexGen::getPropertyValue(){
	std::vector<string> property = {"Common", "Latin", "Greek", "Cyrillic",
             						"Armenian", "Hebrew", "Arabic", "Syriac", 
             						"Thaana", "Devanagari", "Bengali", "Gurmukhi", 
             						"Gujarati", "Oriya", "Tamil", "Telugu", "Kannada"};
	int random = rand() % property.size();
	return property[random];
}
std::string RegexGen::getProperty(){
	if (syntax == re::RE_Syntax::BRE) {
		return "p";
	}
	else {
		return "\\p{" + getPropertyValue() + "}" ;
	}
}
std::string RegexGen::getNotProperty(){
	if (syntax == re::RE_Syntax::BRE) {
		return "P";
	}
	else {
		return "\\P{" + getPropertyValue() + "}" ;
	}
}
std::string RegexGen::getName(){
	if (syntax == re::RE_Syntax::ERE) {
		return "\\N{" + getPropertyValue() + "}" ;
	}
	else {
		return "N";
	}
}
string RegexGen::getZeroOrOne(string cc){
	if (syntax == re::RE_Syntax::BRE) {
		return cc+ "\\?";
	}
	else {
		return cc + "?" ;
	}
}
string RegexGen::getZeroOrMore(string cc){
		return cc + "*" ;
}
string RegexGen::getOneOrMore(string cc){
	if (syntax == re::RE_Syntax::BRE) {
		return cc+ "\\+";
	}
	else {
		return cc + "+" ;
	}
}
string RegexGen::getRep(string cc, int rep){
	if (syntax == re::RE_Syntax::BRE) {
		return cc + "\\{" + to_string(rep) + "\\}";
	}
	else {
		return cc + "{" + to_string(rep) + "}";
	}
}
string RegexGen::getRep(string cc, int lb, int ub){
	if (syntax == re::RE_Syntax::BRE) {
		return cc + "\\{" + to_string(lb) + ',' + to_string(ub) + "\\}";
	}
	else {
		return cc + "{" + to_string(lb) + ',' + to_string(ub) + "}";
	}
}
string RegexGen::getRepMore(string cc, int rep){
	if (syntax == re::RE_Syntax::BRE) {
		return cc + "\\{" + to_string(rep) + ",\\}";
	}
	else {
		return cc + "{" + to_string(rep) + ",}";
	}
}
string RegexGen::getJoin(string LS, string RS){
	if (syntax == re::RE_Syntax::BRE) {
		return "\\(" + LS + "\\|" + RS + "\\)";
	}
	else {
		return "(" + LS + "|" + RS + ")";
	}
}
string RegexGen::getBackRef(string cc){
	if (syntax == re::RE_Syntax::BRE) {
		return "\\(" + cc + "\\)";
	}
	else {
		return "(" + cc + ")";
	}
	
}
string RegexGen::getAssertionCoating(string cc){
	if (cc.find("\\p{") == 0
		|| cc.find("\\P{") == 0
		|| cc.find("\\N{") == 0){
		re::RE * re_ast = re::RE_Parser::parse(cc, 0);
		StringGenerator strGen;
		std::vector<string> set = strGen.generate(re_ast);
		int random = rand() % set.size();
		return set[random];
	}
	else {
		return cc;
	}
}

string RegexGen::getNegativeAssertionCoating(string cc){
	re::RE * re_ast = re::RE_Parser::parse(cc, 0);
	StringGenerator strGen;
	std::vector<string> set = strGen.generate(re::makeDiff(re::makeAny(),re_ast));
	int random = rand() % set.size();
	return set[random];
}
string RegexGen::getLookAhead(string cc){
	if (syntax == re::RE_Syntax::PCRE) {
		std::string tail = getAssertionCoating(cc);
		return "(?=" + cc + ")" + tail;
	}
	return "";
}
string RegexGen::getNegativeLookAhead(string cc){
	if (syntax == re::RE_Syntax::PCRE) {
		std::string tail = getNegativeAssertionCoating(cc);
		return "(?!" + cc + ")" + tail;
	}
	return "";
}
string RegexGen::getLookBehind(string cc){
	if (syntax == re::RE_Syntax::PCRE) {
		std::string front = getAssertionCoating(cc);
		return front + "(?<=" + cc + ")";
	}
	return "";
}
string RegexGen::getNegativeLookBehind(string cc){
	if (syntax == re::RE_Syntax::PCRE) {
		std::string front = getNegativeAssertionCoating(cc);
		return front + "(?<!" + cc + ")";
	}
	return "";
}


bool RegexGen::usesCC(std::string op){
	std::vector<string> set = {"zeroOrOne","zeroOrMore","oneOrMore","repeat_n","repeat_nm","repeat_n_more","repeat_m_less", 
							"backref", "join", "look_ahead", "mlook_ahead", "look_behind", "nlook_behind" "look_ahead", 
							"mlook_ahead", "look_behind", "nlook_behind"};

	if(std::find(set.begin(), set.end(), op) != set.end()){
		return true;
	}
	else return false;
}

re::RE_Syntax RegexGen::setSyntax(std::vector<string> header, std::vector<string> row){
	int colnum = 0;
	for(auto col : row){
		if (header[colnum] == "syntax") {
			if (col == "-G"){
				return re::RE_Syntax::BRE;
			}
			else if (col == "-E"){
				return re::RE_Syntax::ERE;
			}
		}
		colnum++;
	}
	return re::RE_Syntax::PCRE;
}



std::vector<string> RegexGen::parseCC(std::vector<string> header, std::vector<string> row){
	std::vector<string> ccList;

	int colnum = 0;
	for(auto col : row){
		if (col != "false"){
			string cc;

			if (header[colnum] == "wordC") {
				cc = getWord();
				ccList.push_back(getWord());
			}
			else if (header[colnum] == "notWordC") {
				cc = getNotWord();
				ccList.push_back(getNotWord());
			}
			else if (header[colnum] == "whitespace") {
				cc = getWhitespace();
				ccList.push_back(getWhitespace());
			}
			else if (header[colnum] == "notWhitespace") {
				cc = getNotWhitespace();
				ccList.push_back(getNotWhitespace());
			}
			else if (header[colnum] == "tab") {
				cc = getTab();
				ccList.push_back(getTab());
			}
			else if (header[colnum] == "digit") {
				cc = getDigit();
				ccList.push_back(getDigit());
			}
			else if (header[colnum] == "notDigit") {
				cc = getNotDigit();
				ccList.push_back(getNotDigit());
			}
			else if (header[colnum] == "any") {
				cc = getWord();
				ccList.push_back(getAny());
			}
			else if (header[colnum] == "unicodeC"){
				cc = getUnicode();
				ccList.push_back(getUnicode());
			}
			else if (header[colnum] == "list"){
				cc = getList();
				ccList.push_back(getList());
			}
			else if (header[colnum] == "nList"){
				cc = getNList();
				ccList.push_back(getNList());
			}
			else if (header[colnum] == "range"){
				cc = getRange();
				ccList.push_back(getRange());
			}
			else if (header[colnum] == "posix"){
				// if (col != "off"){
				// 	c = getPosix(col);
				// 	if (!c.empty())
				// 	ccList.push_back(getPosix(col));
				// }
			}
			else if (header[colnum] == "property") {
				cc = getProperty();
				ccList.push_back(getProperty());
			}
			else if (header[colnum] == "notProperty") {
				cc = getProperty();
				ccList.push_back(getNotProperty());
			}
			else if (header[colnum] == "nameProperty") {
				cc = getName();
				ccList.push_back(getName());
			}
		}
		++colnum;
	}
	return ccList;
}

std::string RegexGen::parseRE(std::vector<string> header, std::vector<string> row){
	srand (time(NULL));
	std::vector<string> fullRE;
	std::vector<string> assertions;
	CC ccHandler(header,row, syntax);
	int random;
	bool bref = false;
	string first = "";
	string last = "";
	int colnum = 0;
	std::string re;
	for (auto col : row){
		if (col != "false" && !ccHandler.isEmpty()){
			
			string cc;
			if (usesCC(header[colnum])){

				cc = ccHandler.getCC();
				if (header[colnum] == "zeroOrOne"  ){
					re = getZeroOrOne(cc);
					if (!re.empty())
					fullRE.push_back(re);
				}
				else if (header[colnum] == "zeroOrMore"){
					re = getZeroOrMore(cc);
					if (!re.empty())
					fullRE.push_back(re);
				} 
				else if (header[colnum] == "oneOrMore"){
					re = getOneOrMore(cc);
					// std::string nestDepth = row[colnum+1];
					// int depth = std::stoi(nestDepth);
					// while(depth>0){
					// 	std::string cc2;
					// 	cc2 = ccHandler.getCC();
					// 	re = '(' + re + cc2 + ')' + header[colnum];
					// 	--depth;
					// }
					if (!re.empty())
					fullRE.push_back(re);
				}
				else if (header[colnum] == "repeat_n"){
					random = rand() % 200 + 1;
					re = getRep(cc, random);
					// std::string nestDepth = row[colnum+1];
					// int depth = std::stoi(nestDepth);
					// while(depth>0){
					// 	std::string cc2;
					// 	cc2 = ccHandler.getCC();
					// 	random = rand() % 200;
					// 	re = '(?:' + re + cc2 + ')' + '{' + to_string(random) + '}';
					// 	--depth;
					// }
					if (!re.empty())
					fullRE.push_back(re);
				}
				else if (header[colnum] == "repeat_nm" ){
					int r1 = rand() % 200;
					int r2 = rand() % 200;
					while ((r1 == 0) && (r2 == 0)){
						r2 = rand() % 200;
					}
					if (r1 > r2) {
						std::swap(r1, r2);
					}
					re = getRep(cc, r1, r2);
					// std::string nestDepth = row[colnum+1];
					// int depth = std::stoi(nestDepth);
					// while(depth>0){
					// 	std::string cc2 = ccHandler.getCC();
					// 	std::vector<int> randoms;
					// 	randoms.push_back(rand() % 200);
					// 	randoms.push_back(rand() % 200);
					// 	sort(randoms.begin(), randoms.end());
					// 	re = '(?:' + re + cc2 + ')' + '{' + to_string(randoms[0]) + ',' + to_string(randoms[1]) + '}';
					// 	--depth;
					// }
					if (!re.empty())
					fullRE.push_back(re);
				}
				else if (header[colnum] == "repeat_n_more" ){
					random = rand() % 200;
					re = getRepMore(cc, random);
					if (!re.empty())
					fullRE.push_back(re);
				}
				else if (header[colnum] == "repeat_m_less" ){
					random = rand() % 200 + 1;
					re = getRep(cc, 0, random);
					if (!re.empty())
					fullRE.push_back(re);
				}
				else if (header[colnum] == "backref"){
					bref = true;
				}
				else if (header[colnum] == "join" ){
					std::string cc2 = ccHandler.getCC();
					re = getJoin(cc, cc2);
					if (!re.empty())
					fullRE.push_back(re);
				}
				else if (header[colnum] == "look_ahead"){
					re = getLookAhead(cc);
					if (!re.empty())
					assertions.push_back(re);
				}
				else if (header[colnum] == "nlook_ahead"){
					if (cc == ".") {
						continue;
					}
					else {
						re = getNegativeLookAhead(cc);
						if (!re.empty())
						assertions.push_back(re);
					}
				}
				else if (header[colnum] == "look_behind"){
					re = getLookBehind(cc);
					if (!re.empty())
					assertions.push_back(re);
				}
				else if (header[colnum] == "nlook_behind"){
					if (cc == ".") {
						continue;
					}
					else {
						re = getNegativeLookBehind(cc);
						if (!re.empty())
						assertions.push_back(re);
					}
				}
			}
			else {

				// if (header[colnum] == "boundary") {
				// 	re = getBoundary();
				// 	if (!re.empty())
				// 	assertions.push_back(re +"\\s");
				// }
				// else if (header[colnum] == "notBoundary") {
				// 	std::string front = getAssertionCoating("[^\\s\\t\\n]");
				// 	re = getNotBoundary();
				// 	if (!re.empty())
				// 	assertions.push_back(re + front);
				// }
				if (header[colnum] == "start"){
						first = "^";
				}
				else if (header[colnum] == "end"){
						last = "$";
				}
			}

		}
		colnum++;
	}

	vector<string> ccList = ccHandler.getRemainingCC();
	while(!ccList.empty()){
		fullRE.push_back(ccList.back());
		ccList.pop_back();
	}
	if(!fullRE.empty()){

		if (bref){
			random = rand() % fullRE.size();
			fullRE[random] = getBackRef(fullRE[random]);
			std::copy (assertions.begin(), assertions.end(), std::back_inserter(fullRE));
			std::random_shuffle(fullRE.begin(), fullRE.end());
			fullRE.push_back("\\1");
		}
		else {
			std::copy (assertions.begin(), assertions.end(), std::back_inserter(fullRE)); //to avoid nesting assertions.
			std::random_shuffle(fullRE.begin(), fullRE.end());
		}
		string sre= first + stringifyLine(fullRE) + last;
		return sre;
	}
	else {
		return "";
	}
}

std::vector<string> RegexGen::parseFlags(std::vector<string> header, std::vector<string> row){
	std::vector<string> flags;
	int colnum = 0;
	for (auto col : row){
		if (col != "false"){
			if (header[colnum] == "-c"
				|| header[colnum] == "-i"
				// || header[colnum] == "-w"
				// || header[colnum] == "-x"
				|| header[colnum] == "-e"
				|| header[colnum] == "-f"){
				flags.push_back(header[colnum]);
			}
			else if (header[colnum] == "syntax"){
				flags.push_back(col);
			}
		}
		colnum++;
	}
	return flags;
}

