/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_SETMARKER_H
#define PS_SETMARKER_H

#include "ps_pablos.h"
#include <string>

class Assign : public PabloS
{
public:
    Assign(std::string m, PabloE* expr);
    ~Assign();
    std::string getM();
    PabloE* getExpr();
private:
    std::string mM;
    PabloE* mExpr;
};

#endif // PS_SETMARKER_H

