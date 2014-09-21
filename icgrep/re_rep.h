/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_REP_H
#define RE_REP_H

#include "re_re.h"

const int UNBOUNDED_REP = -1;

class Rep : public RE
{
public:
    Rep(RE* re, int lb, int ub);
    ~Rep();
    RE* getRE() const;
    int getLB() const;
    void setLB(int lb);
    int getUB() const;
    void setUB(int ub);
private:
    RE* mRE;
    int mLB;
    int mUB;
};

#endif
