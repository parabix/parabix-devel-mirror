/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_IF_H
#define PS_IF_H

#include <pablo/pe_pabloe.h>

namespace pablo {

class If : public PabloE {
    friend struct CodeGenState;
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::If;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~If() {
    }
    inline PabloE * getExpr() const {
        return mExpr;
    }
    inline const ExpressionList & getPSList() const {
        return mPSList;
    }
protected:
    If(PabloE * expr, ExpressionList && psl)
    : PabloE(ClassTypeId::If)
    , mExpr(expr)
    , mPSList(psl)
    {

    }
private:
    PabloE * const mExpr;
    ExpressionList mPSList;
};

}

#endif // PS_IF_H


