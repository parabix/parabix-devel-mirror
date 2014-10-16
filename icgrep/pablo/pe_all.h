/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_ALL_H
#define PE_ALL_H

#include "pabloAST.h"

namespace pablo {

class All : public PabloAST {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
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
    inline bool operator==(const All & other) const {
        return mValue == other.mValue;
    }
    virtual bool operator==(const PabloAST & other) const {
        return (isa<All>(other)) ? mValue == cast<All>(other).mValue : false;
    }
protected:
    All(const bool value)
    : PabloAST(ClassTypeId::All)
    , mValue(value)
    {

    }
private:
    const bool mValue;
};

}

#endif // PE_ALL_H


