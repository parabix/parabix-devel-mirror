/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef SYMBOL_GENERATOR_H
#define SYMBOL_GENERATOR_H

#include <pablo/pabloAST.h>
#include <unordered_map>
#include <string>

namespace IDISA { class IDISA_Builder; }
namespace pablo { class String; }
namespace pablo { class Integer; }

namespace pablo {

class SymbolGenerator {
    friend class PabloBlock;
    using Allocator = PabloAST::Allocator;
public:
    typedef int64_t integer_t;
    String * get(const std::string name, IDISA::IDISA_Builder * builder);
    String * make(const std::string prefix, IDISA::IDISA_Builder *builder);
    Integer * getInteger(const integer_t value, IDISA::IDISA_Builder * builder);
    SymbolGenerator(Allocator & allocator) : mAllocator(allocator) {}
    ~SymbolGenerator() = default;
private:
    Allocator &                                 mAllocator;
    std::unordered_map<std::string, integer_t>  mPrefixMap;
    std::unordered_map<std::string, String *>   mStringMap;
    std::unordered_map<integer_t, Integer *>    mIntegerMap;
};

}

#endif // SYMBOL_GENERATOR_H
