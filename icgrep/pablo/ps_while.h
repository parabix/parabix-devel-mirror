/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_WHILE_H
#define PS_WHILE_H

#include "ps_pablos.h"
#include <list>

namespace pablo {

class While : public PabloE {
    typedef std::list<PabloE*> List;
public:
    While(PabloE* expr, List psl)
    : PabloE(ClassTypeId::While)
    , mExpr(expr)
    , mPSList(psl)
    {

    }

    virtual ~While() {
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
    PabloE *    mExpr;
    List        mPSList;
};

}

#endif // PS_WHILE_H


