/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_VAR_H
#define PE_VAR_H

#include "pe_pabloe.h"
#include <string>

namespace pablo {

class Var : public PabloE {
public:
    Var(std::string var)
    : PabloE(ClassTypeId::Var)
    , mVar(var)
    {

    }
    virtual ~Var(){

    }
    inline std::string getVar() const {
        return mVar;
    }
private:
    const std::string mVar;
};

}



#endif // PE_VAR_H


