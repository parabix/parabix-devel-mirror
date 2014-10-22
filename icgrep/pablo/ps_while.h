/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_WHILE_H
#define PS_WHILE_H

#include <pablo/pabloAST.h>

namespace pablo {

class While : public PabloAST {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::While;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~While() {
    }
    inline PabloAST * getCondition() const {
        return mExpr;
    }
    inline const ExpressionList & getBody() const {
        return mBody;
    }
    inline void setInclusiveCarryCount(const unsigned count) {
        mCarryCount = count;
    }
    inline unsigned getInclusiveCarryCount() const {
        return mCarryCount;
    }
protected:
    While(PabloAST * expr, ExpressionList && body)
    : PabloAST(ClassTypeId::While)
    , mExpr(expr)
    , mBody(std::move(body))
    , mCarryCount(0)
    {

    }
private:
    PabloAST * const    mExpr;
    ExpressionList      mBody;
    unsigned            mCarryCount;
};

}

#endif // PS_WHILE_H


