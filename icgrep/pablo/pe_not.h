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

class Not : public PabloAST {
    friend struct OptimizeNot;
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Not;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Not() {
    }
    PabloAST * getExpr() const {
        return mExpr;
    }
protected:
    Not(PabloAST * expr)
    : PabloAST(ClassTypeId::Not)
    , mExpr(expr) {

    }
private:
    PabloAST * const mExpr;
};

struct OptimizeNot {
    inline OptimizeNot(PabloBlock & cg) : cg(cg) {}
    PabloAST * operator()(PabloAST * expr);
private:
    PabloBlock & cg;

};

}

#endif // PE_NOT_H


