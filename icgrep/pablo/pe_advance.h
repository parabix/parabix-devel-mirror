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
public:
    Advance(PabloE * expr)
    : PabloE(ClassTypeId::Advance)
    , mExpr(expr) {

    }

    virtual ~Advance() {
        delete mExpr;
    }

    inline PabloE * getExpr() const {
        return mExpr;
    }

private:
    PabloE * mExpr;
};

}

#endif // PE_ADVANCE_H



