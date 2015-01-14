/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_OR_H
#define PE_OR_H

#include <pablo/pabloAST.h>
#include <array>

namespace pablo {

class PabloBlock;

class Or : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Or;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Or() {
    }
    PabloAST * getExpr1() const {
        return mOperand[0];
    }
    PabloAST * getExpr2() const {
        return mOperand[1];
    }
protected:
    Or(PabloAST * expr1, PabloAST * expr2, PabloBlock * parent);
};

}

#endif // PE_OR_H



