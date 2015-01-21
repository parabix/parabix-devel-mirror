/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef SYMBOL_GENERATOR_H
#define SYMBOL_GENERATOR_H

#include <pablo/pabloAST.h>
#include <string>
#include <unordered_map>

namespace pablo {

class String;
class Integer;

class SymbolGenerator {
    friend class PabloBlock;
public:
    typedef u_int64_t integer_t;
    String * get(const std::string name);
    String * make(const std::string prefix);
    Integer * getInteger(const integer_t value);
    ~SymbolGenerator();
protected:
    SymbolGenerator();    
private:
    std::unordered_map<std::string, unsigned>   mPrefixMap;    
    std::unordered_map<std::string, String *>   mStringMap;
    std::unordered_map<integer_t, Integer *>    mIntegerMap;
};

}

#endif // SYMBOL_GENERATOR_H
