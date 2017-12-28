#ifndef RE_LOCAL_H
#define RE_LOCAL_H

namespace re {

class RE; class CC;

struct RE_Local {
    static CC * getFirstUniqueSymbol(RE * re);
};

}

#endif
