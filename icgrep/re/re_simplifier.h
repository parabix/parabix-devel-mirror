#ifndef RE_SIMPLIFIER_H
#define RE_SIMPLIFIER_H

//Regular Expressions
#include "re_seq.h"
#include <list>

namespace re {

class Alt;

class RE_Simplifier {
public:
    static RE * simplify(RE * re);
};

}

#endif // RE_SIMPLIFIER_H
