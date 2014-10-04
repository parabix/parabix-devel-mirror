/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_MATCHSTAR_H
#define PE_MATCHSTAR_H

#include "pe_pabloe.h"

namespace pablo {

class MatchStar : public PabloE {
    friend MatchStar * makeMatchStar(PabloE *, PabloE *);
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::MatchStar;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~MatchStar() {
        delete mExpr1;
        delete mExpr2;
    }

    inline PabloE * getExpr1() const {
        return mExpr1;
    }

    inline PabloE * getExpr2() const  {
        return mExpr2;
    }
protected:
    MatchStar(PabloE * expr1, PabloE * expr2)
    : PabloE(ClassTypeId::MatchStar)
    , mExpr1(expr1)
    , mExpr2(expr2)
    {

    }
private:
    PabloE * const mExpr1;
    PabloE * const mExpr2;
};

inline MatchStar * makeMatchStar(PabloE * expr1, PabloE * expr2) {
    return new MatchStar(expr1, expr2);
}

}

#endif // PE_MATCHSTAR_H



