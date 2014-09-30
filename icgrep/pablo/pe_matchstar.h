/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_MATCHSTAR_H
#define PE_MATCHSTAR_H

#include "pe_pabloe.h"

namespace pablo {

class MatchStar : public PabloE {
public:
    MatchStar(PabloE * expr1, PabloE * expr2)
    : PabloE(ClassTypeId::MatchStar)
    , mExpr1(expr1)
    , mExpr2(expr2)
    {

    }

    virtual ~MatchStar() {
        delete mExpr1;
        delete mExpr2;
    }

    inline PabloE * getExpr1() const {
        return mExpr1;
    }

    inline PabloE * getExpr2() const  {
        return mExpr2;
    }

private:
    PabloE* mExpr1;
    PabloE* mExpr2;
};

}

#endif // PE_MATCHSTAR_H



