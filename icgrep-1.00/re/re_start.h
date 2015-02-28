/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef START_H
#define START_H

#include <re/re_re.h>

namespace re {

class Start : public RE {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::Start;
    }
    static inline bool classof(const void *) {
        return false;
    }
protected:
    friend Start * makeStart();
    Start() : RE(ClassTypeId::Start) {}
    virtual ~Start() {}
};

inline Start * makeStart() {
    return new Start();
}

}

#endif // START_H

