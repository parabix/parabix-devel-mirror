/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_ALL_H
#define PE_ALL_H

#include "pe_pabloe.h"

class All : public PabloE
{
public:
    All(int num);
    ~All();
    int getNum();
    void setNum(int num);
private:
    int mNum;
};

#endif // PE_ALL_H


