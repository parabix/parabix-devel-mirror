/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_CHARCLASS_H
#define PE_CHARCLASS_H

#include "pe_pabloe.h"
#include <string>

namespace pablo {

class CharClass : public PabloE
{
public:
    CharClass(std::string charClass)
    : PabloE(ClassTypeId::CharClass)
    , mCharClass(charClass)
    {

    }

    virtual ~CharClass(){

    }

    inline const std::string & getCharClass() const {
        return mCharClass;
    }

private:
    const std::string mCharClass;
};

}

#endif // PE_CHARCLASS_H


