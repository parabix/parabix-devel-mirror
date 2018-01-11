#ifndef STRINGGEN_H
#define STRINGGEN_H

#include <string>
#include <vector>
#include <re/re_cc.h>
#include <re/re_parser.h>

namespace re {
    class RE;
}

class StringGenerator
{

public:
    StringGenerator(std::string re, std::vector<std::string> flags, re::RE_Syntax syntax);

    std::vector<re::CC* > generate(re::RE *re, bool Complement=false, bool getOne=false);
    std::string generate();
    std::string stringifyVec(std::vector<re::CC *> elements);

    ~StringGenerator();

private:

	std::string mRegex;
	std::vector<std::string> mFlags;
	re::RE_Syntax mSyntax;
	std::vector<std::vector<re::CC *>> mReferences;

    re::CC * const asciiCC;
    re::CC * const unicodeCC;
    re::CC * const forbiddenCC;
    re::CC * const allCodepointsCC;

    re::CC * getRandomCodepointCC(re::CC * cc);
    re::CC * getAllCodepoints();

};

#endif // STRINGGEN_H
