#ifndef RE_LOCAL_H
#define RE_LOCAL_H

#include <UCD/ucd_compiler.hpp>
#include <map>

namespace re {

class RE;

class RE_Local {
public:
	static UCD::UnicodeSet* first(RE * re);
	static UCD::UnicodeSet* final(RE * re);
	static void follow(RE * re, std::map<UCD::UnicodeSet*, UCD::UnicodeSet*> &follow_map);
	static bool isLocalLanguage(RE * re);
private:
	static RE * collect_UnicodeSets_helper(RE * re, std::vector<UCD::UnicodeSet> & UnicodeSets);
	static bool isNullable(const RE * re);
};

}

#endif
