/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_IF_H
#define PS_IF_H

#include <pablo/pabloAST.h>

namespace pablo {

class If : public PabloAST {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::If;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~If() {
    }
    inline PabloAST * getCondition() const {
        return mExpr;
    }
    inline const ExpressionList & getBody() const {
        return mPSList;
    }
protected:
    If(PabloAST * expr, ExpressionList && psl)
    : PabloAST(ClassTypeId::If)
    , mExpr(expr)
    , mPSList(psl)
    {

    }
private:
    PabloAST * const mExpr;
    ExpressionList mPSList;
};

}

#endif // PS_IF_H


