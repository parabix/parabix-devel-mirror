/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef XOR_H
#define XOR_H

#include "pe_pabloe.h"

namespace pablo {

class Xor : public PabloE {
    friend PabloE * make_xor(PabloE *, PabloE *);
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::Xor;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Xor() {
        delete mExpr1;
        delete mExpr2;
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

PabloE * make_xor(PabloE * expr1, PabloE * expr2);

}

#endif // XOR_H



