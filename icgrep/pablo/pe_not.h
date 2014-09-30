/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_NOT_H
#define PE_NOT_H

#include "pe_pabloe.h"

namespace pablo {

class Not : public PabloE
{
public:
    Not(PabloE* expr)
    : PabloE(ClassTypeId::Not)
    , mExpr(expr) {

    }

    ~Not() {
        delete mExpr;
    }

    PabloE * getExpr() const {
        return mExpr;
    }

    void setExpr(PabloE * expr) {
        mExpr = expr;
    }

private:
    PabloE* mExpr;
};

}

#endif // PE_NOT_H


