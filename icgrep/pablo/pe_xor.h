/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef XOR_H
#define XOR_H

#include <pablo/pe_pabloe.h>

namespace pablo {

class PabloBlock;

class Xor : public PabloE {
    friend struct OptimizeXor;
    friend class PabloBlock;
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::Xor;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Xor() {
    }
    inline PabloE * getExpr1() const {
        return mExpr1;
    }
    inline PabloE * getExpr2() const {
        return mExpr2;
    }
protected:
    Xor(PabloE * expr1, PabloE * expr2)
    : PabloE(ClassTypeId::Xor)
    , mExpr1(expr1)
    , mExpr2(expr2)
    {

    }
private:
    PabloE * const mExpr1;
    PabloE * const mExpr2;
};

struct OptimizeXor {
    inline OptimizeXor(PabloBlock & cg) : cg(cg) {}
    PabloE * operator()(PabloE * expr1, PabloE * expr2);
private:
    PabloBlock & cg;
};

}

#endif // XOR_H



