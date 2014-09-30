/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_ADVANCE_H
#define PE_ADVANCE_H

#include "pe_pabloe.h"

namespace pablo {

class Advance : public PabloE {
    friend Advance * make_advance(PabloE * expr);
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::Advance;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Advance() {
        delete mExpr;
    }

    inline PabloE * getExpr() const {
        return mExpr;
    }
protected:
    Advance(PabloE * expr)
    : PabloE(ClassTypeId::Advance)
    , mExpr(expr) {

    }
private:
    PabloE * const mExpr;
};

inline Advance * make_advance(PabloE * expr) {
    return new Advance(expr);
}

}

#endif // PE_ADVANCE_H



