/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pe_charclass.h"

CharClass::CharClass(std::string charClass)
{
    mCharClass = charClass;
}

CharClass::~CharClass(){}

std::string CharClass::getCharClass()
{
    return mCharClass;
}

