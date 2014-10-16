/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_OR_H
#define PE_OR_H

#include <pablo/pabloAST.h>

namespace pablo {

class PabloBlock;

class Or : public PabloAST {
    friend struct OptimizeOr;
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
    inline PabloAST * getExpr1() const {
        return mExpr1;
    }
    inline PabloAST* getExpr2() const {
        return mExpr2;
    }
protected:
    Or(PabloAST * expr1, PabloAST * expr2)
    : PabloAST(ClassTypeId::Or)
    , mExpr1(expr1)
    , mExpr2(expr2)
    {

    }
private:
    PabloAST * const mExpr1;
    PabloAST * const mExpr2;
};

struct OptimizeOr {
    inline OptimizeOr(PabloBlock & cg) : cg(cg) {}
    PabloAST * operator()(PabloAST * expr1, PabloAST * expr2);
private:
    PabloBlock & cg;
};

}

#endif // PE_OR_H



