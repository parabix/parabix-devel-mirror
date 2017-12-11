/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ALT_H
#define ALT_H

#include "re_re.h"
#include "re_cc.h"
#include <llvm/Support/Casting.h>

namespace re {

class Alt : public Vector {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::Alt;
    }
    static inline bool classof(const void *) {
        return false;
    }
protected:
    friend Alt * makeAlt();
    template<typename iterator> friend RE * makeAlt(iterator, iterator);
    Alt()
    : Vector(ClassTypeId::Alt) {

    }
    Alt(iterator begin, iterator end)
    : Vector(ClassTypeId::Alt, begin, end) {

    }
};

/**
 * @brief makeAlt
 *
 * Build an Alt, flattening alternative subgroups, and combining character classes and
 * move character classes towards the end of the list to ensure that all combinations are found.
 *
 * @param list
 * @return simplified RE representing the Alt
 */

inline Alt * makeAlt() {
    return new Alt();
}

template<typename iterator>
RE * makeAlt(iterator begin, iterator end) {
    Alt * newAlt = makeAlt();
    CC * unionCC = makeCC();
    for (auto i = begin; i != end; ++i) {
        if (const CC * cc = llvm::dyn_cast<CC>(*i)) {
            unionCC = makeCC(unionCC, cc);
        } else if (const Alt * alt = llvm::dyn_cast<Alt>(*i)) {
            // We have an Alt to embed within the alt.  We extract the individual
            // elements to include within the new alt.   Note that recursive flattening
            // is not required, if the elements themselves were created with makeAlt.
            for (RE * a : *alt) {
                if (CC * cc = llvm::dyn_cast<CC>(a)) {
                    unionCC = makeCC(unionCC, cc);
                }
                else newAlt->push_back(a);
            }
        }
        else {
            newAlt->push_back(*i);
        }
    }
    if (!unionCC->empty()) newAlt->push_back(unionCC);
    return newAlt;
}

inline RE * makeAlt(RE::InitializerList list) {
    return makeAlt(list.begin(), list.end());
}

}

#endif // ALT_H

