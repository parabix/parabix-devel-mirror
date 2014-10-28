/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ALT_H
#define ALT_H

#include "re_re.h"
#include "re_cc.h"
#include <queue>
#include <iostream>

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
    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }
private:
    template<typename iterator>
    void flatten(iterator begin, iterator end, std::queue<CC*> & ccQ) {
        for (auto i = begin; i != end; ++i) {
            if (Alt * alt = dyn_cast<Alt>(*i)) {
                flatten(alt->begin(), alt->end(), ccQ);
                continue;
            }
            else if (CC * cc = dyn_cast<CC>(*i)) {
                ccQ.push(cc);
                continue;
            }
            push_back(*i);
        }
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
    Alt * alt = makeAlt();
    std::queue<CC*> ccQ;
    alt->flatten(begin, end, ccQ);
    if (!ccQ.empty()) {
        while (ccQ.size() > 1) {
            CC * a = ccQ.front(); ccQ.pop();
            CC * b = ccQ.front(); ccQ.pop();
            ccQ.push(makeCC(a, b));
        }
        alt->push_back(ccQ.front());
    }
    if (alt->size() == 1) {
        return alt->back();
    }
    return alt;
}

inline RE * makeAlt(RE::InitializerList list) {
    return makeAlt(list.begin(), list.end());
}

}

#endif // ALT_H

