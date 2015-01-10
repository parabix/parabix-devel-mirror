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

class PabloBlock;

class Xor : public Statement {
    friend struct OptimizeXor;
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
        return mOperand[0];
    }
    PabloAST * getExpr2() const {
        return mOperand[1];
    }
protected:
    Xor(PabloAST * expr1, PabloAST * expr2, PabloBlock * parent);
};

struct OptimizeXor {
    PabloAST * operator()(PabloAST * expr1, PabloAST * expr2, PabloBlock * pb);
};

}

#endif // XOR_H



