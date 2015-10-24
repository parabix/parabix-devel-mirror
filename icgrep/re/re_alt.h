/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ALT_H
#define ALT_H

#include "re_re.h"
#include "re_cc.h"

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
private:
    template<typename iterator>
    void flatten(iterator begin, iterator end) {
        for (auto i = begin; i != end; ++i) {
            if (LLVM_UNLIKELY(isa<Alt>(*i))) {
                flatten<Alt::iterator>(cast<Alt>(*i)->begin(), cast<Alt>(*i)->end());
            } else {
                push_back(*i);
            }
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
    if (LLVM_UNLIKELY(std::distance(begin, end) == 0)) {
        throw std::runtime_error("Alt objects cannot be empty!");
    } else if (std::distance(begin, end) == 1) {
        return *begin;
    } else {
        Alt * alt = makeAlt();
        alt->flatten(begin, end);
        if (alt->size() == 1) {
            return alt->front();
        }
        return alt;
    }
}

inline RE * makeAlt(RE::InitializerList list) {
    return makeAlt(list.begin(), list.end());
}

}

#endif // ALT_H

