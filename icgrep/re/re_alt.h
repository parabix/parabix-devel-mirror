/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ALT_H
#define ALT_H

#include "re_re.h"
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/printer_re.h>
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
    
inline void add_CC(std::vector<CC *> & CCs_found, CC * cc) {
    for (unsigned j = 0; j < CCs_found.size(); j++) {
        if (CCs_found[j]->getAlphabet() == cc->getAlphabet()) {
            CCs_found[j] = makeCC(CCs_found[j], cc);
            return;
        }
    }
    CCs_found.push_back(cc);
}

template<typename iterator>
RE * makeAlt(iterator begin, iterator end) {
    Alt * newAlt = makeAlt();
    bool nullableAltFound = false;
    std::vector<CC *> CCs_found;  // CCs with possibly different alphabets
    for (auto i = begin; i != end; ++i) {
        if (CC * cc = llvm::dyn_cast<CC>(*i)) {
            add_CC(CCs_found, cc);
        } else if (const Alt * alt = llvm::dyn_cast<Alt>(*i)) {
            // We have an Alt to embed within the alt.  We extract the individual
            // elements to include within the new alt.   Note that recursive flattening
            // is not required, if the elements themselves were created with makeAlt.
            for (RE * a : *alt) {
                if (CC * cc = llvm::dyn_cast<CC>(a)) {
                    add_CC(CCs_found, cc);
                } else if (isEmptySeq(a)) {
                    nullableAltFound = true;
                } else {
                    newAlt->push_back(a);
                }
            }
        } else if (const Rep * rep = llvm::dyn_cast<Rep>(*i)) {
            if (rep->getLB() == 0) {
                nullableAltFound = true;
                newAlt->push_back(makeRep(rep->getRE(), 1, rep->getUB()));
            } else {
                newAlt->push_back(*i);
            }
        } else if (isEmptySeq(*i)) {
            nullableAltFound = true;
        } else {
            newAlt->push_back(*i);
        }
    }
    for (CC * cc : CCs_found) {
        newAlt->push_back(cc);
    }
    if (nullableAltFound) newAlt->push_back(makeSeq());
    if (newAlt->size() == 1) return newAlt->front();
    return newAlt;
}

inline RE * makeAlt(RE::InitializerList list) {
    return makeAlt(list.begin(), list.end());
}

// An Alt with no members represent the empty set.
inline bool isEmptySet(RE * r) {
    return llvm::isa<Alt>(r) && llvm::cast<Alt>(r)->empty();
}
}

#endif // ALT_H

