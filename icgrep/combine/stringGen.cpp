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

#include <re/printer_re.h>
using namespace std;
using namespace re;
using namespace llvm;


CC * StringGenerator::getRandomCodepointCC(CC * cc){
    assert (cc);
    if (!cc->subset(*getAllCodepoints())) {
        cc = intersectCC(cc, getAllCodepoints());
    }
    if (cc->empty()) {
        return cc;
    }
    int random = rand() % cc->count();
    return makeCC(cc->at(random));

}

inline CC * StringGenerator::getAllCodepoints() {
    assert (allCodepointsCC);
    return allCodepointsCC;
}

string stringifyCC(CC * cc){
    if (cc->empty()){
        return "";
    }
    std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> converter;
    std::string u8str = converter.to_bytes(lo_codepoint(cc->begin()));
    return u8str;
}

string StringGenerator::stringifyVec(vector<CC *> elements) {
	string line = "";
	for (auto e : elements) {
        if (!e->empty()){
            line += stringifyCC(getRandomCodepointCC(e));
        }
	}
	return line;
}


string StringGenerator::generate() {
    if (mSyntax == re::RE_Syntax::FixedStrings) {
      return mRegex;
    }
    else {
      string result;
      bool caseInsensitive = std::find(mFlags.begin(), mFlags.end(), "-i") != mFlags.end();
      if (re::RE * re_ast = re::RE_Parser::parse(mRegex, caseInsensitive, mSyntax)){
         result = stringifyVec(generate(re_ast));
      }
      return result;
    }
}

std::vector<CC *> StringGenerator::generate(RE * re, bool Complement, bool getOne) {
    // cout << Printer_RE::PrintRE(re) << endl;
   	std::vector<CC*> retVec = {};
    if (re == nullptr) {
        return retVec;

    } else if (isa<Any>(re)) {
        if (getOne){
            retVec.push_back(getRandomCodepointCC(getAllCodepoints()));
        }
        else{
            retVec.push_back(getAllCodepoints());
        }

    } else if ( Alt* re_alt = dyn_cast<Alt>(re)) {
        int random = rand() % re_alt->size();
        retVec = generate((*re_alt)[random]);
    } else if (CC* re_cc = dyn_cast<CC>(re)) {
        if (!re_cc->empty()){
            retVec.push_back(subtractCC(re_cc, forbiddenCC));
        }
    } else if (Name* re_name = dyn_cast<Name>(re)) {

    	switch (re_name->getType()) {
        case Name::Type::Byte:
    		case Name::Type::Unicode:
		 		retVec = generate(re_name->getDefinition());
		 		break;
		 	case Name::Type::UnicodeProperty: {
                if ((re_name->getName() == "whitespace") && !Complement) {
                    retVec.push_back(makeCC(0x0020));
                    break;
                }
                UCD::UnicodeSet ucs = UCD::resolveUnicodeSet(re_name);

                if (!ucs.empty()) {
                    CC * propertyCC = makeCC(std::move(ucs));
                    if (propertyCC->intersects(*forbiddenCC)) {
                        propertyCC = subtractCC(propertyCC, forbiddenCC);
                    }
                    retVec.push_back(propertyCC);
		 		}
		 		break;
		 	}
    		case Name::Type::Capture: {
    			std::vector<CC *> set = generate(re_name->getDefinition());
                std::vector<CC *> ref;
    			if (!set.empty()){
                    for (auto s : set) {
                        if (!s->empty()){
                            CC * randomCC = getRandomCodepointCC(s);
                            retVec.push_back(randomCC);
                            ref.push_back(randomCC);
                        }
                    }
                }

                mReferences.push_back(ref);
    			break;
    		}
		 	case Name::Type::Reference:
		 	{
		 		for (unsigned i = 0; i < mReferences.size(); i++){
		 			string ref = "\\" + to_string(i+1);
		 			if (ref == re_name->getName()){
		 				retVec = mReferences[i];
                        break;
		 			}
		 		}
		 		break;
		 	}
		 	default:
		 	cerr << "Bad name type" << endl;
		}
    } else if (isa<Assertion>(re)) {
    	//Do Nothing
    } else if (Diff* diff = dyn_cast<Diff>(re)) {
        CC * LHS = makeCC();
        CC * RHS = makeCC();
        for (auto cc : generate(diff->getLH())) {
            LHS = makeCC(LHS, cc);
        }
        LHS = subtractCC(LHS, forbiddenCC);
        for (auto cc : generate(diff->getRH(), true, false)) {
            RHS = makeCC(RHS, cc);
        }
        retVec.push_back(subtractCC(LHS, RHS));
    } else if (Intersect* x = dyn_cast<Intersect>(re)) {

        CC * LHS = makeCC();
        CC * RHS = makeCC();
        for (auto cc : generate(x->getLH(), true, false)) {
            LHS = makeCC(LHS, cc);
        }
        for (auto cc : generate(x->getRH(), true, false)) {
            RHS = makeCC(RHS, cc);
        }
        retVec.push_back(intersectCC(LHS, RHS));

    } else if (Rep* re_rep = dyn_cast<Rep>(re)) {

        std::vector<CC *> set = generate(re_rep->getRE());
        if (!set.empty()){
        	int lb = re_rep->getLB();
            int ub = (re_rep->getUB() == Rep::UNBOUNDED_REP) ? lb + 1000 : re_rep->getUB();

            int boundRange = (lb == 0)? (ub+1) : (ub - lb + 1);
        	int random = rand() % boundRange + lb;
        	// cout << "random bound = " << to_string(random) << endl;
            for (auto i =0; i < random; ++i){
                for (auto *s : set) {
                    retVec.push_back(s);
                }
            }
        }

    } else if (Seq* re_seq = dyn_cast<Seq>(re)) {

        for (RE * re : *re_seq) {
            std::vector<CC *> set = generate(re);

            if (!set.empty()) {
                for (auto s : set) {
                    retVec.push_back(getRandomCodepointCC(s));
                }
        	}
        }
    } else if (isa<Start>(re) || isa<End>(re)) {
		retVec.push_back(makeCC());
    } else {
        cerr << "RE type not recognised\n";
    }
    // if (!retVec.empty())
    //     cout << retVec.back() << endl;
    // if (!returnCC->empty()) {
    //     cout << "returnCC = " << Printer_RE::PrintRE(returnCC) << endl;
    // }
    return retVec;
}



StringGenerator::StringGenerator(std::string re, std::vector<std::string> flags, re::RE_Syntax syntax)
: mRegex(re)
, mFlags(flags)
, mSyntax(syntax)
, asciiCC(re::makeCC(0, 0x007E))
, unicodeCC(re::makeCC(0, 0xEFFFF))
, forbiddenCC(re::makeCC({{0x0000, 0x0008}, {0x000A, 0x001F},
                          {0x007F, 0x007F}, {0x0085, 0x0085},
                          {0x2028, 0x2029}, {0x2424, 0x2424},
                          {0x2B89, 0x2B92}, {0x4DD7, 0x4DD7},
                          {0xD800, 0xDFFF}, {0xE01F0, 0xEFFFF}}))
, allCodepointsCC(subtractCC((syntax == re::RE_Syntax::PCRE) ? unicodeCC : asciiCC, forbiddenCC))
{

}

StringGenerator::~StringGenerator() {
    re::RE::Reset();
    // UCD::UnicodeSet::Reset();
}
