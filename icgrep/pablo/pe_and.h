/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_AND_H
#define PE_AND_H

#include <pablo/pabloAST.h>
#include <array>

namespace pablo {

class PabloBlock;

class And : public Statement {
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
        return mOperand[0];
    }
    PabloAST * getExpr2() const {
        return mOperand[1];
    }
protected:
    And(PabloAST * expr1, PabloAST * expr2, PabloBlock * parent);
};

struct OptimizeAnd {
    PabloAST * operator()(PabloAST * expr1, PabloAST * expr2, PabloBlock * pb);
};

}

#endif // PE_AND_H


