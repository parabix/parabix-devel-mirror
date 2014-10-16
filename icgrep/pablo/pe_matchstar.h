/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_MATCHSTAR_H
#define PE_MATCHSTAR_H

#include "pabloAST.h"

namespace pablo {

class MatchStar : public PabloAST {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::MatchStar;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~MatchStar() {
    }
    inline PabloAST * getExpr1() const {
        return mExpr1;
    }
    inline PabloAST * getExpr2() const  {
        return mExpr2;
    }
protected:
    MatchStar(PabloAST * expr1, PabloAST * expr2)
    : PabloAST(ClassTypeId::MatchStar)
    , mExpr1(expr1)
    , mExpr2(expr2)
    {

    }
private:
    PabloAST * const mExpr1;
    PabloAST * const mExpr2;
};

}

#endif // PE_MATCHSTAR_H



