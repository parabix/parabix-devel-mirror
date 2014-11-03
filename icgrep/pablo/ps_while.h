/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_WHILE_H
#define PS_WHILE_H

#include <pablo/pabloAST.h>

namespace pablo {

class While : public Statement {
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
    virtual PabloAST * getOperand(const unsigned index) const {
        assert (index == 0);
        return mExpr;
    }
    virtual unsigned getNumOperands() const {
        return 1;
    }
    virtual void setOperand(const unsigned index, PabloAST * value) {
        assert (index == 0);
        mExpr = value;
    }
    inline PabloAST * getCondition() const {
        return mExpr;
    }
    inline StatementList & getBody() {
        return mBody;
    }
    inline const StatementList & getBody() const {
        return mBody;
    }
    inline void setInclusiveCarryCount(const unsigned count) {
        mCarryCount = count;
    }
    inline unsigned getInclusiveCarryCount() const {
        return mCarryCount;
    }
    inline void setInclusiveAdvanceCount(const unsigned count) {
        mAdvanceCount = count;
    }
    inline unsigned getInclusiveAdvanceCount() const {
        return mAdvanceCount;
    }
protected:
    While(PabloAST * expr, StatementList && body, StatementList * parent)
    : Statement(ClassTypeId::While, parent)
    , mExpr(expr)
    , mBody(std::move(body))
    , mCarryCount(0)
    , mAdvanceCount(0)
    {
        for (Statement * s : mBody) {
            s->mParent = &mBody;
        }
    }
private:
    PabloAST *          mExpr;
    StatementList       mBody;
    unsigned            mCarryCount;
    unsigned            mAdvanceCount;
};

}

#endif // PS_WHILE_H


