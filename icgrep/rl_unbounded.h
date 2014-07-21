/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RL_UNBOUNDED_H
#define RL_UNBOUNDED_H

#include "rl_replimit.h"

class Unbounded : public RepLimit
{
public:
    Unbounded();
    ~Unbounded();
};

#endif // RL_UNBOUNDED_H
