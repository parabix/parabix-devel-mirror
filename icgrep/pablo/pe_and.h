/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_AND_H
#define PE_AND_H

#include "pe_pabloe.h"

namespace pablo {

class And : public PabloE {
public:
    And(PabloE * expr1, PabloE * expr2)
    : PabloE(ClassTypeId::And)
    , mExpr1(expr1)
    , mExpr2(expr2)
    {

    }
    virtual ~And() {
        delete mExpr1;
        delete mExpr2;
    }
    PabloE * getExpr1() const {
        return mExpr1;
    }
    PabloE * getExpr2() const {
        return mExpr2;
    }
private:
    PabloE* mExpr1;
    PabloE* mExpr2;
};

}

#endif // PE_AND_H


