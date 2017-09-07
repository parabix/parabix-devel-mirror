#ifndef RE_ALT_MINIMIZATION_H
#define RE_ALT_MINIMIZATION_H

namespace re {

class RE;

class RE_Minimizer {
public:
    static RE * minimize(RE * re);
};

}

#endif // RE_ALT_MINIMIZATION_H
