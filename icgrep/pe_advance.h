/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_ADVANCE_H
#define PE_ADVANCE_H

#include "pe_pabloe.h"

class Advance : public PabloE
{
public:
    Advance(PabloE* expr);
    ~Advance();
    PabloE* getExpr();
private:
    PabloE* mExpr;
};

#endif // PE_ADVANCE_H



