/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef ANY_H
#define ANY_H

#include "re_re.h"

#include "re_re.h"

namespace re {

class Any : public RE {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::Any;
    }
    static inline bool classof(const void *) {
        return false;
    }
protected:
    friend Any * makeAny();
    Any() : RE(ClassTypeId::Any) {}
    virtual ~Any() {}
};

inline Any * makeAny() {
    return new Any();
}

}

#endif // ANY_H
