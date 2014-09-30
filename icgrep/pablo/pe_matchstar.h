/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_MATCHSTAR_H
#define PE_MATCHSTAR_H

#include "pe_pabloe.h"

class MatchStar : public PabloE
{
public:
    MatchStar(PabloE* expr1, PabloE* expr2);
    ~MatchStar();
    PabloE* getExpr1();
    PabloE* getExpr2();
private:
    PabloE* mExpr1;
    PabloE* mExpr2;
};

#endif // PE_MATCHSTAR_H



