/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef END_H
#define END_H

#include "re_re.h"

namespace re {

class End : public RE {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::End;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~End() {}
protected:
    friend End * makeEnd();
    End() : RE(ClassTypeId::End) {}
};

inline End * makeEnd() {
    return new End();
}

}

#endif // END_H
