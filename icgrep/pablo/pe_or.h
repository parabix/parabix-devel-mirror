/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_OR_H
#define PE_OR_H

#include <pablo/pe_pabloe.h>

namespace pablo {

struct CodeGenState;

class Or : public PabloE {
    friend struct OptimizeOr;
    friend struct CodeGenState;
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::Or;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Or() {
    }
    inline PabloE * getExpr1() const {
        return mExpr1;
    }
    inline PabloE* getExpr2() const {
        return mExpr2;
    }
protected:
    Or(PabloE * expr1, PabloE * expr2)
    : PabloE(ClassTypeId::Or)
    , mExpr1(expr1)
    , mExpr2(expr2)
    {

    }
private:
    PabloE * const mExpr1;
    PabloE * const mExpr2;
};

struct OptimizeOr {
    inline OptimizeOr(CodeGenState & cg) : cg(cg) {}
    PabloE * operator()(PabloE * expr1, PabloE * expr2);
private:
    CodeGenState & cg;
};

inline PabloE * makeOr(PabloE * expr1, PabloE * expr2, CodeGenState & cg) {
    OptimizeOr run(cg);
    return run(expr1, expr2);
}

}

#endif // PE_OR_H



