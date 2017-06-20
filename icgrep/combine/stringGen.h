#ifndef STRINGGEN_H
#define STRINGGEN_H

#include <string>
#include <vector>
#include <re/re_parser.h>

namespace re {
    class RE;
}

class StringGenerator
{

public:
    std::vector<std::string> generate( re::RE *re);
    std::string generate(std::string re, std::vector<std::string> flags, re::RE_Syntax syntax);
    static std::string stringifyVec(std::vector<std::string> elements, std::string separator="");
    

private:
	static std::vector<std::string> references;
	static bool hasFlag(std::string flag, std::vector<std::string> flags);

};

#endif // STRINGGEN_H