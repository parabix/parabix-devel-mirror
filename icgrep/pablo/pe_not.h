/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_NOT_H
#define PE_NOT_H

#include "pe_pabloe.h"

namespace pablo {

class Not : public PabloE {
    friend PabloE * make_not(PabloE *);
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::Not;
    }
    static inline bool classof(const void *) {
        return false;
    }


    ~Not() {
        delete mExpr;
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

PabloE * make_not(PabloE * expr);

}

#endif // PE_NOT_H


