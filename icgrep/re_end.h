/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef END_H
#define END_H

#include "re_re.h"
#include "re_cc.h"

class End : public RE
{
public:
    End();
    CC* getCC();
    ~End();
private:
    CC* mCC;
};

#endif // END_H
