/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_IF_H
#define PS_IF_H

#include <pablo/pabloAST.h>

namespace pablo {

class If : public Statement {
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
protected:
    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }
    If(PabloAST * expr, StatementList && body, StatementList * parent)
    : Statement(ClassTypeId::If, parent)
    , mExpr(expr)
    , mBody(std::move(body))
    , mCarryCount(0)
    {
        for (Statement * s : mBody) {
            s->mParent = &mBody;
        }
    }
private:
    PabloAST *          mExpr;
    StatementList       mBody;
    unsigned            mCarryCount;
};

}

#endif // PS_IF_H


