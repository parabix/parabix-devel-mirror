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

String * SymbolGenerator::get(const std::string name) {
    auto f = mStringMap.find(name);
    String * result;
    if (f == mStringMap.end()) {
        result = makeString(name);
        mStringMap.insert(std::make_pair(name, result));
    }
    else {
        result = f->second;
    }
    return result;
}

String * SymbolGenerator::get_ssa(const std::string prefix) {
    auto f = mPrefixMap.find(prefix);
    unsigned count = 0;
    if (f == mPrefixMap.end()) {
        mPrefixMap.insert(std::make_pair(prefix, 1));
        return get(prefix);
    }
    else {
        count = f->second++;
        return get(prefix + std::to_string(count));
    }
}


}
