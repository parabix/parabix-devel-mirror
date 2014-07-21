/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef XOR_H
#define XOR_H

#include "pe_pabloe.h"

class Xor : public PabloE
{
public:
    Xor(PabloE* expr1, PabloE* expr2);
    ~Xor();
    PabloE* getExpr1();
    PabloE* getExpr2();
private:
    PabloE* mExpr1;
    PabloE* mExpr2;
};

#endif // XOR_H



