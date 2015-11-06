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
#include <llvm/ADT/Twine.h>
#include <llvm/ADT/StringMap.h>
#include <llvm/Support/StringPool.h>

namespace pablo {

class String;
class Integer;

class SymbolGenerator {
    friend class PabloBlock;
    using Twine = llvm::Twine;
public:
    typedef u_int64_t integer_t;
    String * get(const std::string name, const bool generated = true);
    String * make(const std::string prefix, const bool generated = true);
    Integer * getInteger(const integer_t value);
    SymbolGenerator() = default;
    ~SymbolGenerator() = default;
private:
    std::unordered_map<std::string, integer_t>  mPrefixMap;
    std::unordered_map<std::string, String *>   mStringMap;
    std::unordered_map<integer_t, Integer *>    mIntegerMap;
};

static SymbolGenerator GlobalSymbolGenerator;

}

#endif // SYMBOL_GENERATOR_H
