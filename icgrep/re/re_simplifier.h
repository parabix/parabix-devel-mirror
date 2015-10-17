#ifndef RE_SIMPLIFIER_H
#define RE_SIMPLIFIER_H

namespace re {

class RE;

class RE_Simplifier {
public:
    static RE * simplify(RE * re);
};

}

#endif // RE_SIMPLIFIER_H
