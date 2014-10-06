/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_WHILE_H
#define PS_WHILE_H

#include <pablo/pe_pabloe.h>

namespace pablo {

class While : public PabloE {
    friend struct CodeGenState;
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::While;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~While() {
    }
    inline PabloE * getCondition() const {
        return mExpr;
    }
    inline const ExpressionList & getPSList() const {
        return mPSList;
    }
protected:
    While(PabloE * expr, ExpressionList && psl)
    : PabloE(ClassTypeId::While)
    , mExpr(expr)
    , mPSList(psl)
    {

    }
private:
    PabloE * const  mExpr;
    ExpressionList  mPSList;
};

}

#endif // PS_WHILE_H


