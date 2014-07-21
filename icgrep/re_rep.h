/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_REP_H
#define RE_REP_H

#include "re_re.h"
#include "rl_replimit.h"

class Rep : public RE
{
public:
    Rep(RE* re, int lb, RepLimit* ub);
    ~Rep();
    RE* getRE();
    int getLB();
    void setLB(int lb);
    RepLimit* getUB();
private:
    RE* mRE;
    int mLB;
    RepLimit* mUB;
};

#endif
