#ifndef RE_SIMPLIFIER_H
#define RE_SIMPLIFIER_H

//Regular Expressions
#include "re_seq.h"
#include <list>

namespace re {

class Alt;

class RE_Simplifier {
public:
    static RE * simplify(Alt * alt);
    static RE * simplify(Seq * seq);
    static RE * simplify(Rep * rep);
    static RE * simplify(RE * re);
private:
    static int ubCombine(const int h1, const int h2);
};

}

#endif // RE_SIMPLIFIER_H
