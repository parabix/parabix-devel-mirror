/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_SEL_H
#define PE_SEL_H

#include <pablo/pabloAST.h>

namespace pablo {

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
        return getOperand(0);
    }
    inline PabloAST * getTrueExpr() const {
        return getOperand(1);
    }
    inline PabloAST * getFalseExpr() const {
        return getOperand(2);
    }
protected:
    Sel(PabloAST * condExpr, PabloAST * trueExpr, PabloAST * falseExpr, const String * name)
    : Statement(ClassTypeId::Sel, trueExpr->getType(), {condExpr, trueExpr, falseExpr}, name) {

    }
};

}

#endif // PE_SEL_H

