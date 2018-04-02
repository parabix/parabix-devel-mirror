#include "regexGen.h"
#include "stringGen.h"
#include "propGen.h"


#include <string>
#include <vector>
#include <fstream>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <iostream>
#include <algorithm>

#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <re/re_diff.h>
#include <re/re_any.h>
#include <re/parsers/parser.h>



using namespace std;

class CC {
	std::vector<string> ccList;
	std::vector<string> usedCC;
	re::RE_Syntax syntax;
public:
	CC(std::vector<string> &params, std::vector<string> &values, re::RE_Syntax syntax){
		this->syntax = syntax;
		parseCC(params, values);
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
		else if (!usedCC.empty()){
			random = rand() % usedCC.size();
			cc = usedCC[random];
			return cc;
		}
		else return "C";
	}

	std::vector<string> getRemainingCC(){
		return ccList;
	}
private:

	void parseCC(std::vector<string> params, std::vector<string> values){
		int colnum = 0;
		for(auto col : values){
			if ((col != "false") && (col!= "off")){
				string cc;
				if (params[colnum] == "wordC") {
					ccList.push_back(getWord());
				}
				else if (params[colnum] == "notWordC") {
					ccList.push_back(getNotWord());
				}
				else if (params[colnum] == "whitespace") {
					ccList.push_back(getWhitespace());
				}
				else if (params[colnum] == "notWhitespace") {
					ccList.push_back(getNotWhitespace());
				}
				else if (params[colnum] == "tab") {
					ccList.push_back(getTab());
				}
				else if (params[colnum] == "digit") {
					ccList.push_back(getDigit());
				}
				else if (params[colnum] == "notDigit") {
					ccList.push_back(getNotDigit());
				}
				else if (params[colnum] == "any") {
					ccList.push_back(getAny());
				}
				else if (params[colnum] == "unicodeC"){
					ccList.push_back(getUnicodeCodepoint());
				}
				else if (params[colnum] == "range"){
					ccList.push_back(getRange());
				}
				else if (params[colnum] == "posix"){
					ccList.push_back(getPosix(col));
				}
				else if (params[colnum] == "property") {
					ccList.push_back(getProperty(col));
				}
				else if (params[colnum] == "notProperty") {
					ccList.push_back(getNotProperty(col));
				}
				else if (params[colnum] == "nameProperty") {
					ccList.push_back(getUnicodeName());
				}

			}
			++colnum;
		}
	}

	string getWord(){
		if (syntax == re::RE_Syntax::BRE){
			return "w";
		}
		else return "\\w";
	}
	string getNotWord(){
		if (syntax == re::RE_Syntax::BRE){
			return "notWord";
		}
		else return "\\W";
	}
	string getWhitespace(){
		return "\\s";
	}
	string getNotWhitespace(){
		return "\\S";
	}
	string getTab(){
		if (syntax != re::RE_Syntax::PCRE){
			return "t";
		}
		else return "\\t";
	}
	string getDigit(){
		if (syntax != re::RE_Syntax::PCRE){
			return "d";
		}
		else return "\\d";
	}
	string getNotDigit(){
		if (syntax != re::RE_Syntax::PCRE){
			return "D";
		}
		else return "\\D";
	}
	string getAny(){
		return ".";
	}
	string getPosix(string value){
			return "[[:" + value + ":]]";
	}

	std::string getCharacterName(){
		int random = rand() % 28179;
		ifstream file;
		file.open("../icgrep/combine/UnicodeNames.txt");
		string line;
		for (int i = 0; i <= random ; i++){
			if (!getline(file, line)) {
				cerr << "Error in extracting Unicode property names!\n";
			}
		}
		file.close();
		return line;
	}
	std::string getProperty(string type){

		if (syntax == re::RE_Syntax::PCRE) {
			// vector<string> prop = {"L", "Ll", "Lu", "Lt", "Lm", "Lo", "M", "Mn",
			// 											 "Mc", "Me", "Z", "Zs", "Zl", "Zp", "S", "Sm", "Sc",
			// 											 "Sk", "So", "N", "Nd", "Nl", "No", "P", "Pd", "Ps",
			// 										 	 "Pe", "Pi", "Pf", "Pc", "Po", "C", "Cc", "Cf", "Co",
			// 										   "Cs", "Cn"};
			PropGen p;
			string regex = "";
			if (type == "string"){
				regex = "abc";
			}
			return "\\p{" + p.getPropertyValue(type, regex) + "}" ;
		}
		else {
			return "p";
		}
	}
	std::string getNotProperty(string type){
		if (syntax == re::RE_Syntax::PCRE) {
			// vector<string> prop = {"L", "Ll", "Lu", "Lt", "Lm", "Lo", "M", "Mn",
			// 											 "Mc", "Me", "Z", "Zs", "Zl", "Zp", "S", "Sm", "Sc",
			// 											 "Sk", "So", "N", "Nd", "Nl", "No", "P", "Pd", "Ps",
			// 										 	 "Pe", "Pi", "Pf", "Pc", "Po", "C", "Cc", "Cf", "Co",
			// 										   "Cs", "Cn"};
			// int random = rand() % prop.size();
			// return "\\p{" + prop[random] + "}" ;
			string regex = "";
			if (type == "string"){
				regex = "abc";
			}
			PropGen p;
			return "\\P{" + p.getPropertyValue(type, regex) + "}" ;
		}
		else {
			return "P";
		}
	}
	std::string getUnicodeName(){
		if (syntax == re::RE_Syntax::PCRE) {
			return "\\N{^" + getCharacterName() + "$}" ;
		}
		else {
			return "N";
		}
	}
	std::string getUnicodeCodepoint(){
		if (syntax != re::RE_Syntax::PCRE) {
			return "u";
		}
		else {
			int random = rand() % 16544;
			ifstream file;
			file.open("../icgrep/combine/Unicode.txt");
			string line;
			for (int i = 0; i <= random ; i++){
				if (!getline(file, line)) {
					cerr << "Error in extracting Unicode codepoints!\n";
				}
			}
			file.close();
			return "\\u" + line;
		}
	}
	std::string getUnicodeRange(){

		int random1 = rand() % 16544;
		int random2 = rand() % 16544;
		if (random1 > random2){
			std::swap(random1, random2);
		}
		ifstream file;
		file.open("../icgrep/combine/Unicode.txt");
		string line1;
		string line2;
		for (int i = 0; i < random2 ; i++){
			if (!getline(file, line2)) {
				cerr << "Error in extracting Unicode codepoints!\n";
			}
			if (random1 == i){
				line1 = line2;
			}
		}
		file.close();
		return "[\\u" + line1 + "-\\u" + line2 + "]";

	}
	std::string getRange(){

		if (syntax != re::RE_Syntax::PCRE){
			std::vector<string> lists = {"[a-zA-Z0-9]","[A-Za-z]","[0-9]"};
			int random = rand() % lists.size();
			return lists[random];
		}
		else {
			return getUnicodeRange();
		}
	}
};

RegexGen::RegexGen(std::vector<string> param, std::vector<string> val): mParameters(param), mValues(val) {
	try{
		syntax = getSyntax();
		RE = parseRE();
		flags = parseFlags();
		fileType = getFileTy();
	}
	catch (...){
		syntax = re::RE_Syntax::PCRE;
		RE = "Could Not Generate RE";
	}
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


std::string RegexGen::getList(string cc1, string cc2, string cc3){
	if (syntax == re::RE_Syntax::BRE) {
		return "l";
	}
	else if (syntax == re::RE_Syntax::ERE){
		if(cc1.find("[")!= string::npos) cc1 = "a";
		if(cc2.find("[")!= string::npos) cc2 = "b";
		if(cc3.find("[")!= string::npos) cc3 = "c";
		return "[" + cc1 + cc2 + cc3 + "]";
	}
	else {
		return "[" + cc1 + cc2 + cc3 + "]";
	}

}
std::string RegexGen::getNList(string cc1, string cc2, string cc3){
	if (syntax == re::RE_Syntax::BRE) {
		return "L";
	}
	else if (syntax == re::RE_Syntax::ERE){
		if(cc1.find("[")!= string::npos) cc1 = "1";
		if(cc2.find("[")!= string::npos) cc2 = "2";
		if(cc3.find("[")!= string::npos) cc3 = "3";

	}
	string ret = "[^" + cc1 + cc2 + cc3 + "]";
	if (getAssertionCoating(ret) != ret){
		return ret;
	}
	return ret;
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
string RegexGen::getRep(string cc, string size){
	int rep;
	if (size == "small"){
		rep = rand() % 10 + 1;
	}
	else if (size == "medium"){
		rep = rand() % 100 + 1;
	}
	else if (syntax != re::RE_Syntax::PCRE){
		rep = rand() % 255 + 1;
	}
	else {
		rep = rand() % 1000 + 1;
	}

	if (syntax == re::RE_Syntax::BRE) {
		return cc + "\\{" + to_string(rep) + "\\}";
	}
	else {
		return cc + "{" + to_string(rep) + "}";
	}
}
string RegexGen::getRepNM(string cc, string size){
	int lb;
	int ub;
	if (size == "small-small"){
		lb = rand() % 10;
		ub = rand() % 10 + 1;
	}
	else if (size == "small- medium"){
		lb = rand() % 10;
		ub = rand() % 100 + 1;
	}
	else if (size == "small-large"){
		lb = rand() % 10;
		ub = rand() % 255 + 1;
	}
	else if (size == "medium-meduim"){
		lb = rand() % 100;
		ub = rand() % 100 + 1;
	}
	else if (size == "medium-large"){
		lb = rand() % 100;
		if (syntax != re::RE_Syntax::PCRE){
			ub = rand() % 255 + 1;
		}
		else {
			ub = rand() % 1000 + 1;
		}
	}
	else {
		if (syntax != re::RE_Syntax::PCRE){
			lb = rand() % 255;
			ub = rand() % 255 + 1;
		}
		else {
			lb = rand() % 1000;
			ub = rand() % 1000 + 1;
		}

	}
	if (lb > ub) {
		std::swap(lb, ub);
	}
	if (syntax == re::RE_Syntax::BRE) {
		return cc + "\\{" + to_string(lb) + ',' + to_string(ub) + "\\}";
	}
	else {
		return cc + "{" + to_string(lb) + ',' + to_string(ub) + "}";
	}
}
string RegexGen::getRepMore(string cc, string size){
	int lb;
	if (size == "small"){
		lb = rand() % 10 + 1;
	}
	else if (size == "medium"){
		lb = rand() % 100 + 1;
	}
	else {
		if (syntax != re::RE_Syntax::PCRE){
			lb = rand() % 255 + 1;
		}
		else {
			lb = rand() % 1000 + 1;
		}
	}

	if (syntax == re::RE_Syntax::BRE) {
		return cc + "\\{" + to_string(lb) + ",\\}";
	}
	else {
		return cc + "{" + to_string(lb) + ",}";
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
	return cc;
}

string RegexGen::getNegativeAssertionCoating(string cc){
	if (cc == "\\s"){
		return "\\S";
	}
	else if (cc == "\\S"){
		return "\\s";
	}
	else if (cc == "\\t"){
		return "\\S";
	}
	else if (cc == "\\w"){
		return "\\W";
	}
	else if (cc == "\\W"){
		return "\\w";
	}
	else if (cc == "\\d"){
		return "\\D";
	}
	else if (cc == "\\D"){
		return "\\d";
	}
	else if (cc.find("\\p") == 0){
		return "\\P" + cc.substr(2);
	}
	else {
		return "";
	}
}

string RegexGen::getLookAhead(string cc){
	if (syntax == re::RE_Syntax::PCRE) {
		std::string tail = getAssertionCoating(cc);
		return "(?=" + cc + ")" + tail;
	}
	return "la";
}
string RegexGen::getNegativeLookAhead(string cc){
	if (syntax == re::RE_Syntax::PCRE) {
		std::string tail = getNegativeAssertionCoating(cc);
		return "(?!" + cc + ")" + tail;
	}
	return "nla";
}
string RegexGen::getLookBehind(string cc){
	if (syntax == re::RE_Syntax::PCRE) {
		std::string front = getAssertionCoating(cc);
		return front + "(?<=" + cc + ")";
	}
	return "lb";
}
string RegexGen::getNegativeLookBehind(string cc){
	if (syntax == re::RE_Syntax::PCRE) {
		std::string front = getNegativeAssertionCoating(cc);
		return front + "(?<!" + cc + ")";
	}
	return "nlb";
}

string RegexGen::getWordBegin(){
	if (syntax == re::RE_Syntax::PCRE){
		return "\\W\\<\\w";
	}
	else return "wb";
}
string RegexGen::getWordEnd(){
	if (syntax == re::RE_Syntax::PCRE){
		return "\\w\\>\\W";
	}
	else return "we";
}


bool RegexGen::usesCC(std::string op){
	std::vector<string> set = {"zeroOrOne","zeroOrMore","oneOrMore","repeat_n","repeat_nm","repeat_n_more","repeat_m_less",
							"list", "nList", "backref", "join", "look_ahead", "mlook_ahead", "look_behind", "nlook_behind" "look_ahead",
							"mlook_ahead", "look_behind", "nlook_behind"};

	if(std::find(set.begin(), set.end(), op) != set.end()){
		return true;
	}
	else return false;
}

re::RE_Syntax RegexGen::getSyntax(){
	int colnum = 0;
	for(auto col : mValues){
		if (mParameters[colnum] == "syntax") {
			if (col == "-G"){
				return re::RE_Syntax::BRE;
			}
			else if (col == "-E"){
				return re::RE_Syntax::ERE;
			}
			else if (col == "-P"){
				return re::RE_Syntax::PCRE;
			}
			else if (col == "-F"){
				return re::RE_Syntax::FixedStrings;
			}
		}
		colnum++;
	}
	return re::RE_Syntax::PCRE;
}
RegexGen::FileType RegexGen::getFileTy(){
	int colnum = 0;
	for(auto col : mValues){
		if (mParameters[colnum] == "fileType") {
			if (col == "s"){
				return FileType::SMALL;
			}
			else if (col == "m"){
				return FileType::MEDIUM;
			}
			else if (col == "l"){
				return FileType::LARGE;
			}
		}
		colnum++;
	}
	//default
	return FileType::SMALL;
}


std::string RegexGen::parseRE(){

	std::vector<string> fullRE;
	std::vector<string> assertions;
	CC ccHandler(mParameters,mValues, syntax);
	int random;
	bool bref = false;
	string first = "";
	string last = "";
	std::string re;
	for (int colnum = 0; colnum < mValues.size(); colnum++){
		auto col = mValues[colnum];
		if (col != "false" && col != "off"){
			string cc;
			if (usesCC(mParameters[colnum])){

				cc = ccHandler.getCC();
				if (mParameters[colnum] == "zeroOrOne"  ){
					re = getZeroOrOne(cc);
					if (!re.empty())
					fullRE.push_back(re);
				}
				else if (mParameters[colnum] == "zeroOrMore"){
					re = getZeroOrMore(cc);
					if (!re.empty())
					fullRE.push_back(re);
				}
				else if (mParameters[colnum] == "oneOrMore"){
					re = getOneOrMore(cc);
					if (!re.empty())
					fullRE.push_back(re);
				}
				else if (mParameters[colnum] == "repeat_n"){
					re = getRep(cc, col);
					if (!re.empty())
					fullRE.push_back(re);
				}
				else if (mParameters[colnum] == "repeat_nm" ){
					re = getRepNM(cc, col);
					if (!re.empty())
					fullRE.push_back(re);
				}
				else if (mParameters[colnum] == "repeat_n_more" ){
					re = getRepMore(cc, col);
					if (!re.empty())
					fullRE.push_back(re);
				}
				else if (mParameters[colnum] == "list"){
					string cc1 = ccHandler.getCC();
					string cc2 = ccHandler.getCC();
					fullRE.push_back(getList(cc, cc1, cc2));
				}
				else if (mParameters[colnum] == "nList"){
					string cc1 = ccHandler.getCC();
					string cc2 = ccHandler.getCC();
					auto ret = getNList(cc, cc1, cc2);
					if (ret!= "")
						fullRE.push_back(ret);
				}
				else if (mParameters[colnum] == "backref"){
					bref = true;
				}
				else if (mParameters[colnum] == "join" ){
					std::string cc2 = ccHandler.getCC();
					re = getJoin(cc, cc2);
					if (!re.empty())
					fullRE.push_back(re);
				}
				else if (mParameters[colnum] == "look_ahead"){
					re = getLookAhead(cc);
					if (!re.empty())
					assertions.push_back(re);
				}
				else if (mParameters[colnum] == "nlook_ahead"){
					if (cc == ".") {
						continue;
					}
					else {
						re = getNegativeLookAhead(cc);
						if (!re.empty())
						assertions.push_back(re);
					}
				}
				else if (mParameters[colnum] == "look_behind"){
					re = getLookBehind(cc);
					if (!re.empty())
					assertions.push_back(re);
				}
				else if (mParameters[colnum] == "nlook_behind"){
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

				if (mParameters[colnum] == "start"){
						first = "^";
				}
				else if (mParameters[colnum] == "end"){
						last = "$";
				}
				else if (mParameters[colnum] == "word_begin") {
					fullRE.push_back(getWordBegin());
				}
				else if (mParameters[colnum] == "word_end") {
					fullRE.push_back(getWordEnd());
				}
			}
		}
	}

	vector<string> ccList = ccHandler.getRemainingCC();
	while(!ccList.empty()){
		fullRE.push_back(ccList.back());
		ccList.pop_back();
	}
	if(!fullRE.empty()){

		if (bref && !fullRE.empty()){
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

std::vector<string> RegexGen::parseFlags(){
	std::vector<string> flags;
	for (int colnum = 0; colnum < mValues.size(); colnum++){
		auto col = mValues[colnum];
		if ((col != "false") && (col != "off")){
			if (mParameters[colnum] == "-c"
			 || mParameters[colnum] == "-i"
			 || mParameters[colnum] == "-v"
			 || mParameters[colnum] == "-x"
			 || mParameters[colnum] == "-w"
			 || mParameters[colnum] == "-e"
			 || mParameters[colnum] == "-f"){
				flags.push_back(mParameters[colnum]);
			}
			else if (mParameters[colnum] == "-t"){
					flags.push_back("-t=" + col);
			}
			else if (mParameters[colnum] == "-BlockSize"){
					flags.push_back("-BlockSize=" + col);
			}
			else if (mParameters[colnum] == "syntax"){
					flags.push_back(col);
			}
		}
	}
	return flags;
}
