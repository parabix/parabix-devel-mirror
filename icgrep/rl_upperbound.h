/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RL_UPPERBOUND_H
#define RL_UPPERBOUND_H

#include "rl_replimit.h"

class UpperBound : public RepLimit
{
public:
    UpperBound(int ub);
    ~UpperBound();
    int getUB();
    void setUB(int ub);
private:
    int mUB;
};

#endif // RL_UPPERBOUND_H


