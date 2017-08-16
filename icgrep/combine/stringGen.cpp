#include "stringGen.h"

#include <re/re_re.h>
#include <re/re_alt.h>
#include <re/re_any.h>
#include <re/re_cc.h>
#include <re/re_name.h>
#include <re/re_end.h>
#include <re/re_rep.h>
#include <re/re_seq.h>
#include <re/re_start.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_name_resolve.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <re/re_parser.h>
#include <functional>
#include <locale>
#include <codecvt>
#include <stdlib.h>
#include <time.h> 
#include <UCD/resolve_properties.h>
#include <UCD/unicode_set.h>
#include <UCD/PropertyObjects.h>


using namespace std;
using namespace re;
using namespace llvm;


const std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

vector<string> appendRtoL(std::vector<string> LHS, std::vector<string> RHS){
	std::copy(RHS.begin(), RHS.end(), std::back_inserter(LHS));
	return LHS;
}

vector<string> getIntersect(vector<string> v1, vector<string> v2)
{

    vector<string> v3;

    sort(v1.begin(), v1.end());
    sort(v2.begin(), v2.end());

    set_intersection(v1.begin(),v1.end(),v2.begin(),v2.end(),back_inserter(v3));

    return v3;
}

vector<string> getDiff(vector<string> v1, vector<string> v2)
{

    vector<string> v3;

    sort(v1.begin(), v1.end());
    sort(v2.begin(), v2.end());

    set_difference(v1.begin(),v1.end(),v2.begin(),v2.end(),back_inserter(v3));

    return v3;
}

std::vector<string> getAllCodepoints(){
	std::vector<string> cpSet;
	unsigned int max = 0x10FFFF;
	for (unsigned cp = 0; cp < max; ++cp){
		if (cp < 0xD800 || cp > 0xDFFF) {
			std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> converter;
    		std::string u8str = converter.to_bytes(cp);
	    	cpSet.push_back(u8str);
		}
	}
	return cpSet;
}

string StringGenerator::stringifyVec(vector<string> elements, string separator){
	string line = "";
	bool sep = false;
	for (auto e : elements){
		line += sep? separator + e : e;
		sep = true;
	}
	return line;
}

bool StringGenerator::hasFlag(string flag, std::vector<string> flags){
	return (std::find(flags.begin(), flags.end(), flag) != flags.end()) ? true : false;
}


string StringGenerator::generate(string re, std::vector<string> flags, re::RE_Syntax syntax){

	bool caseInsensitive = hasFlag("-i", flags);

	re::RE * re_ast = re::RE_Parser::parse(re, caseInsensitive, syntax);
	string str = stringifyVec(generate(re_ast));
	return str;
}

std::vector<std::string> StringGenerator::generate(RE * re) {
	srand (time(NULL));
   	std::vector<string> retVec;
    if (re == nullptr) {
        return retVec;
    } else if ( Alt* re_alt = dyn_cast<Alt>(re)) {
    	std::vector<string> set;
    	for ( RE * re : *re_alt){
    		set = appendRtoL(set, generate(re));
    	}
		int random = rand() % set.size();
		retVec.push_back(set[random]);
		
    } else if (CC* re_cc = dyn_cast<CC>(re)) {
        for (auto i : *re_cc) {
        	for (auto cp = lo_codepoint(i); cp <= hi_codepoint(i); cp++){
        		std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> converter;
    			std::string u8str = converter.to_bytes(cp);
	        	retVec.push_back(u8str);
        	}
        }
    } else if (Name* re_name = dyn_cast<Name>(re)) {

    	switch (re_name->getType()) {
    		case Name::Type::Byte:
    		case Name::Type::Unicode:
		 		retVec = generate(re_name->getDefinition()); 
		 		break;
		 	case Name::Type::UnicodeProperty: {
                if (re_name->getName() == "whitespace"){
                    retVec.push_back(" ");
                    break;
                }
	 			UCD::UnicodeSet ucs = UCD::resolveUnicodeSet(re_name);
		 		for (auto i : ucs){
		 			for (auto cp = lo_codepoint(i); cp <= hi_codepoint(i); cp++){
			 			std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> converter;
	    				std::string u8str = converter.to_bytes(cp);
		   			 	retVec.push_back(u8str);
		        	}
		 		}
		 		break;
		 	}
    		case Name::Type::Capture: {
    			std::vector<string> set = generate(re_name->getDefinition());
    			int random = rand() % set.size();
    			string str = set[random];
    			references.push_back(str);
    			retVec.push_back(str);
    			break;
    		}
		 	case Name::Type::Reference:
		 	{
                bool found = false;
		 		for (unsigned i = 0; i < references.size(); i++){
		 			string ref = "\\" + to_string(i+1);
		 			if (ref == re_name->getName()){
		 				retVec.push_back(references[i]);
                        found = true;
		 			}
		 		}
                if (!found){
                    cerr << "reference not found\n";
                }
		 		break;
		 	}
		 	default: 
		 	retVec.push_back("Bad MyEnum");
		 }
    } else if (Assertion * a = dyn_cast<Assertion>(re)) {
    	//Do Nothing
    } else if (Diff* diff = dyn_cast<Diff>(re)) {

        std::vector<string> set = getDiff(generate(diff->getLH()), generate(diff->getRH()));
        retVec = appendRtoL(retVec, set);

    } else if (Intersect* x = dyn_cast<Intersect>(re)) {

        std::vector<string> set = getIntersect(generate(x->getLH()), generate(x->getRH()));
        retVec = appendRtoL(retVec, set);

    } else if (Rep* re_rep = dyn_cast<Rep>(re)) {

    	int lb = re_rep->getLB();
    	int ub = (re_rep->getUB() == Rep::UNBOUNDED_REP) ? lb + 100 : re_rep->getUB();
    	string ret = "";
    	
    	int range = (ub - lb) + 1;
    	int random = (lb == 0)? rand() % ub : rand() % range + lb;
    	
    	std::vector<string> set = generate(re_rep->getRE());
        for (auto i =0; i<random; ++i){
        	srand (time(NULL));
        	int random2 = rand() % set.size();
        	ret += set[random2];
        }
        retVec.push_back(ret);

    } else if (Seq* re_seq = dyn_cast<Seq>(re)) {

        for (RE * re : *re_seq) {
            std::vector<string> set = generate(re);
            
            if (!set.empty()){
	            int random = rand() % set.size();
	            retVec.push_back(set[random]);
        	}
        }
    } else if (isa<Start>(re) || isa<End>(re)) {
		retVec.push_back("");
    } else if (isa<Any>(re)) {
        retVec = getAllCodepoints();
    } else {
        cerr << "RE type not recognised\n";
    }
    return retVec;
}