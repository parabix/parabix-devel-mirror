/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_AND_H
#define PE_AND_H

#include <pablo/pabloAST.h>

namespace pablo {

class PabloBlock;

class And : public PabloAST {
    friend struct OptimizeAnd;
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::And;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~And() {
    }
    PabloAST * getExpr1() const {
        return mExpr1;
    }
    PabloAST * getExpr2() const {
        return mExpr2;
    }
protected:
    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }
    And(PabloAST * expr1, PabloAST * expr2)
    : PabloAST(ClassTypeId::And)
    , mExpr1(expr1)
    , mExpr2(expr2)
    {

    }
private:
    PabloAST * const mExpr1;
    PabloAST * const mExpr2;
};

struct OptimizeAnd {
    inline OptimizeAnd(PabloBlock & cg) : cg(cg) {}
    PabloAST * operator()(PabloAST * expr1, PabloAST * expr2);
private:
    PabloBlock & cg;
};

}

#endif // PE_AND_H


