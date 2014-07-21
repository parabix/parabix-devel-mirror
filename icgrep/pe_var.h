/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_VAR_H
#define PE_VAR_H

#include "pe_pabloe.h"
#include <string>

class Var : public PabloE
{
public:
    Var(std::string var);
    ~Var();
    void setVar(std::string var);
    std::string getVar();
private:
    std::string mVar;
};

#endif // PE_VAR_H


