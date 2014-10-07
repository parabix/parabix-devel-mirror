/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_AND_H
#define PE_AND_H

#include <pablo/pe_pabloe.h>

namespace pablo {

struct PabloBlock;

class And : public PabloE {
    friend struct OptimizeAnd;
    friend struct PabloBlock;
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::And;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~And() {
    }
    PabloE * getExpr1() const {
        return mExpr1;
    }
    PabloE * getExpr2() const {
        return mExpr2;
    }
protected:
    And(PabloE * expr1, PabloE * expr2)
    : PabloE(ClassTypeId::And)
    , mExpr1(expr1)
    , mExpr2(expr2)
    {

    }
private:
    PabloE * const mExpr1;
    PabloE * const mExpr2;
};

struct OptimizeAnd {
    inline OptimizeAnd(PabloBlock & cg) : cg(cg) {}
    PabloE * operator()(PabloE * expr1, PabloE * expr2);
private:
    PabloBlock & cg;
};

}

#endif // PE_AND_H


