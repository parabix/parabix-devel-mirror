#ifndef RE_LOCAL_H
#define RE_LOCAL_H

#include <UCD/ucd_compiler.hpp>
#include <map>

namespace re {

class RE;

class RE_Local {
public:
    static CC * first(RE * re);
    static CC * final(RE * re);
    static void follow(RE * re, std::map<CC*, CC*> &follow_map);
	static bool isLocalLanguage(RE * re);
private:
	static RE * collect_UnicodeSets_helper(RE * re, std::vector<UCD::UnicodeSet> & UnicodeSets);
	static bool isNullable(const RE * re);
};

}

#endif
