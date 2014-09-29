/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "symbol_generator.h"

namespace re {

SymbolGenerator::SymbolGenerator()
: pfxmap()
{
}

std::string SymbolGenerator::get(std::string prefix) {
    auto f = pfxmap.find(prefix);
    unsigned count = 0;
    if (f == pfxmap.end()) {
        pfxmap.insert(std::make_pair(prefix, 1));
    }
    else {
        count = f->second++;
    }
    return prefix + std::to_string(count);
}

}
