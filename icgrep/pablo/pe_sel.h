/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_SEL_H
#define PE_SEL_H

#include <pablo/pabloAST.h>
#include <array>

namespace pablo {

class PabloBlock;

class Sel : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Sel;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Sel() {
    }
    inline PabloAST * getCondition() const {
        return mOperand[0];
    }
    inline PabloAST * getTrueExpr() const {
        return mOperand[1];
    }
    inline PabloAST * getFalseExpr() const {
        return mOperand[2];
    }
protected:
    Sel(PabloAST* if_expr, PabloAST* t_expr, PabloAST* f_expr, PabloBlock * parent);
};

}

#endif // PE_SEL_H

