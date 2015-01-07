/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_NOT_H
#define PE_NOT_H

#include <pablo/pabloAST.h>

namespace pablo {

class PabloBlock;

class Not : public Statement {
    friend struct OptimizeNot;
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Not;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual PabloAST * getOperand(const unsigned index) const {
        assert (index == 0);
        return mExpr;
    }
    virtual unsigned getNumOperands() const {
        return 1;
    }
    virtual void setOperand(const unsigned index, PabloAST * value) {
        assert (index == 0);
        mExpr = value;
    }
    virtual ~Not() {
    }
    PabloAST * getExpr() const {
        return mExpr;
    }
protected:
    Not(PabloAST * expr, PabloBlock * parent);
private:
    PabloAST * mExpr;
};

struct OptimizeNot {
    PabloAST * operator()(PabloAST * expr, PabloBlock * pb);
};

}

#endif // PE_NOT_H


