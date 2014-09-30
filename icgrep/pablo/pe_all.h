/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_ALL_H
#define PE_ALL_H

#include "pe_pabloe.h"

namespace pablo {

class All : public PabloE {
    friend All * make_all(bool value);
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::All;
    }
    static inline bool classof(const void *) {
        return false;
    }

    virtual ~All() {

    }

    inline bool getValue() const {
        return mValue;
    }
protected:
    All(const bool value)
    : PabloE(ClassTypeId::All)
    , mValue(value)
    {

    }
private:
    const bool mValue;
};

inline All * make_all(bool value) {
    return new All(value);
}

}

#endif // PE_ALL_H


