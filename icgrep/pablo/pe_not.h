/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_NOT_H
#define PE_NOT_H

#include <pablo/pe_pabloe.h>

namespace pablo {

struct CodeGenState;

class Not : public PabloE {
    friend struct OptimizeNot;
    friend struct CodeGenState;
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::Not;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Not() {
    }
    PabloE * getExpr() const {
        return mExpr;
    }
protected:
    Not(PabloE * expr)
    : PabloE(ClassTypeId::Not)
    , mExpr(expr) {

    }
private:
    PabloE * const mExpr;
};

struct OptimizeNot {
    inline OptimizeNot(CodeGenState & cg) : cg(cg) {}
    PabloE * operator()(PabloE * expr);
private:
    CodeGenState & cg;

};

}

#endif // PE_NOT_H


