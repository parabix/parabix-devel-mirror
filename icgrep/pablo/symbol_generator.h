/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef SYMBOL_GENERATOR_H
#define SYMBOL_GENERATOR_H

#include <string>
#include <unordered_map>

namespace pablo {

class String;

class SymbolGenerator
{
public:
    SymbolGenerator();

    std::string ssa(std::string prefix);

    String * operator[](const std::string string);

private:
    std::unordered_map<std::string, unsigned>   mPrefixMap;
    std::unordered_map<std::string, String *>   mStringMap;
};

}

#endif // SYMBOL_GENERATOR_H
