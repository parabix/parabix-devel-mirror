/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_AND_H
#define PE_AND_H

#include "pe_pabloe.h"

class And : public PabloE
{
public:
    And(PabloE* expr1, PabloE* expr2);
    ~And();
    PabloE* getExpr1() const;
    PabloE* getExpr2() const;
private:
    PabloE* mExpr1;
    PabloE* mExpr2;
};

#endif // PE_AND_H


