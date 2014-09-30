/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_IF_H
#define PS_IF_H

#include "ps_pablos.h"
#include <list>

namespace pablo {

class If : public PabloE {
public:
    typedef std::list<PabloE*> List;

    If(PabloE * expr, List psl)
    : PabloE(ClassTypeId::If)
    , mExpr(expr)
    , mPSList(psl)
    {

    }

    virtual ~If() {
        delete mExpr;
        for (auto statement : mPSList) {
            delete statement;
        }
    }

    inline PabloE * getExpr() const {
        return mExpr;
    }

    inline const List & getPSList() const {
        return mPSList;
    }

private:
    PabloE* mExpr;
    List mPSList;
};

}

#endif // PS_IF_H


