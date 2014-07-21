/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef SYMBOL_GENERATOR_H
#define SYMBOL_GENERATOR_H

#include <iostream>
#include <string>
#include <sstream>
#include <utility>
#include <map>

#define INT2STRING(i) static_cast<std::ostringstream*>(&(std::ostringstream() << i))->str()

class SymbolGenerator
{
public:
    SymbolGenerator();
    std::string gensym(std::string prefix);
private:
    std::map<std::string, int>* pfxmap;
};

#endif // SYMBOL_GENERATOR_H
