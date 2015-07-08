/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef XOR_H
#define XOR_H

#include <pablo/pabloAST.h>
#include <array>

namespace pablo {

class Xor : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Xor;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Xor() {
    }
    PabloAST * getExpr1() const {
        return getOperand(0);
    }
    PabloAST * getExpr2() const {
        return getOperand(1);
    }
protected:
    Xor(PabloAST * expr1, PabloAST * expr2, String * name)
    : Statement(ClassTypeId::Xor, {expr1, expr2}, name)
    {

    }
};

}

#endif // XOR_H



