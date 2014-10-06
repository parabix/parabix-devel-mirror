/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/symbol_generator.h>
#include <pablo/pe_string.h>

namespace pablo {

SymbolGenerator::SymbolGenerator()
: mPrefixMap()
{
}

std::string SymbolGenerator::ssa(std::string prefix) {
    auto f = mPrefixMap.find(prefix);
    unsigned count = 0;
    if (f == mPrefixMap.end()) {
        mPrefixMap.insert(std::make_pair(prefix, 1));
    }
    else {
        count = f->second++;
    }
    return prefix + std::to_string(count);
}

String * SymbolGenerator::operator[](const std::string string) {
    auto f = mStringMap.find(string);
    String * result;
    if (f == mStringMap.end()) {
        result = makeString(string);
        mStringMap.insert(std::make_pair(*result, result));
    }
    else {
        result = f->second;
    }
    return result;
}

}
