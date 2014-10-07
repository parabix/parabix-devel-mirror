/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_ADVANCE_H
#define PE_ADVANCE_H

#include "pe_pabloe.h"

namespace pablo {

class Advance : public PabloE {
    friend struct PabloBlock;
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::Advance;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Advance() {
    }
    inline PabloE * getExpr() const {
        return mExpr;
    }
protected:
    Advance(PabloE * expr)
    : PabloE(ClassTypeId::Advance)
    , mExpr(expr) {

    }
private:
    PabloE * const mExpr;
};

}

#endif // PE_ADVANCE_H



