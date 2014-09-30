/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_IF_H
#define PS_IF_H

#include "ps_pablos.h"
#include <list>

class If : public PabloS
{
public:
    If(PabloE* expr, std::list<PabloS*> psl);
    ~If();
    PabloE* getExpr();
    std::list<PabloS*> getPSList();
private:
    PabloE* mExpr;
    std::list<PabloS*> mPSList;
};

#endif // PS_IF_H


