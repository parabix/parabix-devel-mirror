/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ALT_H
#define ALT_H

#include "re_re.h"

namespace re {

class Alt : public Vector {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::Alt;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual RE * clone() const {
        return new Alt(*this);
    }
protected:
    friend Alt * makeAlt();
    friend Alt * makeAlt(Alt::iterator, Alt::iterator);
    Alt()
    : Vector(ClassTypeId::Alt) {

    }
    Alt(const Alt & alt)
    : Vector(ClassTypeId::Alt, alt.cbegin(), alt.cend(), true) {

    }
    Alt(iterator begin, iterator end)
    : Vector(ClassTypeId::Alt, begin, end) {

    }
};

inline Alt * makeAlt() {
    return new Alt();
}

inline Alt * makeAlt(Alt::iterator begin, Alt::iterator end) {
    return new Alt(begin, end);
}

}

#endif // ALT_H

