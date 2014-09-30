/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_OR_H
#define PE_OR_H

#include "pe_pabloe.h"

namespace pablo {

class Or : public PabloE {
public:
    Or(PabloE * expr1, PabloE * expr2)
    : PabloE(ClassTypeId::MatchStar)
    , mExpr1(expr1)
    , mExpr2(expr2)
    {

    }

    virtual ~Or() {
        delete mExpr1;
        delete mExpr2;
    }

    inline PabloE * getExpr1() const {
        return mExpr1;
    }

    inline PabloE* getExpr2() const {
        return mExpr2;
    }
private:
    PabloE* mExpr1;
    PabloE* mExpr2;
};

}

#endif // PE_OR_H



