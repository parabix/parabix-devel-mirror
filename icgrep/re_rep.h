/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_REP_H
#define RE_REP_H

#include "re_re.h"

const int unboundedRep = -1;

class Rep : public RE
{
public:
    Rep(RE* re, int lb, int ub);
    ~Rep();
    RE* getRE();
    int getLB();
    void setLB(int lb);
    int getUB();
    void setUB(int ub);
private:
    RE* mRE;
    int mLB;
    int mUB;
};

#endif
