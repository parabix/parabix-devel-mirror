/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_CHARCLASS_H
#define PE_CHARCLASS_H

#include "pe_pabloe.h"
#include "string"

class CharClass : public PabloE
{
public:
    CharClass(std::string charClass);
    ~CharClass();
    std::string getCharClass();
private:
    std::string mCharClass;
};

#endif // PE_CHARCLASS_H


