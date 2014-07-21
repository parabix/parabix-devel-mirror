/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_NOT_H
#define PE_NOT_H

#include "pe_pabloe.h"

class Not : public PabloE
{
public:
    Not(PabloE* expr);
    ~Not();
    PabloE* getExpr();
private:
    PabloE* mExpr;
};

#endif // PE_NOT_H


