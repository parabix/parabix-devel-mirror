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

class SymbolGenerator {
    friend class PabloBlock;
public:
    String * get(const std::string name);
    String * make(const std::string prefix);
protected:
    SymbolGenerator();
    void* operator new (std::size_t size) noexcept {
        return PabloAST::mAllocator.allocate(size);
    }
private:
    std::unordered_map<std::string, unsigned>   mPrefixMap;
    std::unordered_map<std::string, String *>   mStringMap;
};

}

#endif // SYMBOL_GENERATOR_H
