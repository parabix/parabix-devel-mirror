/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_OR_H
#define PE_OR_H

#include "pe_pabloe.h"

class Or : public PabloE
{
public:
    Or(PabloE* expr1, PabloE* expr2);
    ~Or();
    PabloE* getExpr1();
    PabloE* getExpr2();
private:
    PabloE* mExpr1;
    PabloE* mExpr2;
};

#endif // PE_OR_H



