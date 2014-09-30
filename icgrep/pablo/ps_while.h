/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_WHILE_H
#define PS_WHILE_H

#include "ps_pablos.h"
#include <list>

class While : public PabloS
{
public:
    While(PabloE* expr, std::list<PabloS*> psl);
    ~While();
    PabloE* getExpr();
    std::list<PabloS*> getPSList();
private:
    PabloE* mExpr;
    std::list<PabloS*> mPSList;
};

#endif // PS_WHILE_H


