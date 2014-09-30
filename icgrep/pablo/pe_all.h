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
public:

    All(const bool value)
    : PabloE(ClassTypeId::All)
    , mValue(value)
    {

    }

    virtual ~All() {

    }

    inline bool getValue() const {
        return mValue;
    }

    inline void setValue(const bool value) {
        mValue = value;
    }

private:
    bool mValue;
};

}

#endif // PE_ALL_H


