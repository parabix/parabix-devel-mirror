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
    If(PabloAST * expr, StatementList && body)
    : PabloAST(ClassTypeId::If)
    , mExpr(expr)
    , mBody(std::move(body))
    , mCarryCount(0)
    {

    }
private:
    PabloAST * const    mExpr;
    StatementList       mBody;
    unsigned            mCarryCount;
};

}

#endif // PS_IF_H


